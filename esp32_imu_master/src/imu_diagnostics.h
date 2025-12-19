/*
 * IMU Diagnostic Logging System
 * 
 * Provides diagnostic logging to identify BNO055 teleportation jump causes:
 * - Quaternion sign flips (dot product with previous)
 * - Calibration status changes
 * - I2C read validity
 * - Gyro-based prediction errors
 * 
 * Enable with: -DENABLE_DIAGNOSTIC_LOGGING=1 in build flags
 */

#ifndef IMU_DIAGNOSTICS_H
#define IMU_DIAGNOSTICS_H

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include "sensor_fusion.h"

// Provided by main translation unit to synchronize BNO055 access
void lock_bno_i2c();
void unlock_bno_i2c();

#ifndef ENABLE_DIAGNOSTIC_LOGGING
#define ENABLE_DIAGNOSTIC_LOGGING 0
#endif

// Diagnostic logging state
struct DiagnosticState {
    Quaternion prev_raw_quat;
    Quaternion prev_raw_quat_valid;
    Quaternion prev_proc_quat;  // Previous processed quaternion (after tracker.process())
    bool initialized;
    uint32_t last_log_ms;
    uint32_t log_interval_ms;  // Log every N milliseconds (0 = every sample at 50Hz = 20ms)
    uint32_t sample_count;
    
    // For repeated packet detection
    Quaternion last_quat;
    uint32_t repeated_count;
    
    DiagnosticState() : initialized(false), last_log_ms(0), 
                       log_interval_ms(20), sample_count(0), repeated_count(0) {
        prev_raw_quat_valid = Quaternion();
        prev_proc_quat = Quaternion();
    }
};

// Global diagnostic state (one per IMU stream)
#if defined(DUAL_IMU_MODE) && DUAL_IMU_MODE
static DiagnosticState diag_state_bno;
static DiagnosticState diag_state_mpu;
static DiagnosticState diag_state_fused;
#else
static DiagnosticState diag_state;
#endif

/**
 * Check if quaternion is valid (not all zeros, finite, reasonable norm)
 */
inline bool check_i2c_validity(const Quaternion& q, bool& is_all_zero, bool& has_nan, bool& bad_norm) {
    is_all_zero = (q.w == 0.0f && q.x == 0.0f && q.y == 0.0f && q.z == 0.0f);
    
    has_nan = (!std::isfinite(q.w) || !std::isfinite(q.x) || 
               !std::isfinite(q.y) || !std::isfinite(q.z));
    
    float norm = q.norm();
    bad_norm = (norm < 0.1f || norm > 2.0f || !std::isfinite(norm));
    
    return !is_all_zero && !has_nan && !bad_norm;
}

/**
 * Detect if quaternion is identical to previous (stuck I2C read)
 */
inline bool detect_repeated_packets(DiagnosticState& state, const Quaternion& q) {
    const float EPSILON = 1e-6f;
    bool is_repeated = (fabsf(q.w - state.last_quat.w) < EPSILON &&
                       fabsf(q.x - state.last_quat.x) < EPSILON &&
                       fabsf(q.y - state.last_quat.y) < EPSILON &&
                       fabsf(q.z - state.last_quat.z) < EPSILON);
    
    if (is_repeated) {
        state.repeated_count++;
    } else {
        state.repeated_count = 0;
        state.last_quat = q;
    }
    
    return (state.repeated_count > 5);  // Flag if same value 5+ times
}

/**
 * Get BNO055 calibration status and operation mode
 */
inline void get_bno055_calibration_status(Adafruit_BNO055& bno, 
                                         uint8_t& sys, uint8_t& gyro, 
                                         uint8_t& accel, uint8_t& mag,
                                         uint8_t& mode) {
    lock_bno_i2c();
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    mode = bno.getMode();  // getMode() returns uint8_t, not adafruit_bno055_opmode_t
    unlock_bno_i2c();
}

/**
 * Log diagnostic data for a quaternion stream
 * Enhanced version that tracks q_raw, q_proc, q_pub and all intermediate metrics
 */
inline void log_diagnostic_data(const char* stream_name, 
                                DiagnosticState& state,
                                const Quaternion& q_raw,
                                const Quaternion* q_prev_raw,
                                Adafruit_BNO055* bno,
                                float gyro_prediction_error = -1.0f,
                                const Quaternion* q_proc = nullptr,
                                const Quaternion* q_prev_proc = nullptr,
                                bool smoothing_active = false,
                                int smoothing_samples = 0) {
#if ENABLE_DIAGNOSTIC_LOGGING
    uint32_t now_ms = millis();
    
    // Check if it's time to log (respect log_interval_ms)
    // On first call (last_log_ms == 0), always log
    if (state.log_interval_ms > 0 && state.last_log_ms > 0) {
        if (now_ms - state.last_log_ms < state.log_interval_ms) {
            return;
        }
    }
    
    state.last_log_ms = now_ms;
    state.sample_count++;
    
    // I2C validity checks
    bool is_all_zero, has_nan, bad_norm;
    bool valid = check_i2c_validity(q_raw, is_all_zero, has_nan, bad_norm);
    bool is_repeated = detect_repeated_packets(state, q_raw);
    
    // Compute dot product with previous raw quaternion
    float dot_raw_prev = 0.0f;
    if (state.initialized && q_prev_raw) {
        dot_raw_prev = q_raw.dot(*q_prev_raw);
    }
    
    // Compute metrics for processed quaternion if provided
    float dot_proc_prev = 0.0f;
    float norm_raw = q_raw.norm();
    float norm_proc = 0.0f;
    float angle_diff_raw_vs_proc = 0.0f;
    float angle_diff_proc_vs_prev = 0.0f;
    
    if (q_proc) {
        norm_proc = q_proc->norm();
        
        // Angular difference between raw and processed
        angle_diff_raw_vs_proc = QuaternionTracker::angular_distance(q_raw, *q_proc);
        
        // Dot product and angular difference for processed vs previous processed
        if (q_prev_proc) {
            dot_proc_prev = q_proc->dot(*q_prev_proc);
            angle_diff_proc_vs_prev = QuaternionTracker::angular_distance(*q_proc, *q_prev_proc);
        } else if (state.initialized && state.prev_proc_quat.norm() > 0.1f) {
            dot_proc_prev = q_proc->dot(state.prev_proc_quat);
            angle_diff_proc_vs_prev = QuaternionTracker::angular_distance(*q_proc, state.prev_proc_quat);
        }
    }
    
    // Get calibration status if BNO055 provided
    uint8_t sys_cal = 0, gyro_cal = 0, accel_cal = 0, mag_cal = 0;
    uint8_t mode = OPERATION_MODE_IMUPLUS;
    if (bno) {
        get_bno055_calibration_status(*bno, sys_cal, gyro_cal, accel_cal, mag_cal, mode);
    }
    
    // Log diagnostic line with all metrics
    Serial.printf("[DIAG %s] #%lu t=%lu ", stream_name, state.sample_count, now_ms);
    
    // Raw quaternion metrics
    Serial.printf("q_raw=[%.4f,%.4f,%.4f,%.4f] norm_raw=%.4f dot_raw_prev=%.4f ",
                  q_raw.w, q_raw.x, q_raw.y, q_raw.z, norm_raw, dot_raw_prev);
    
    // Processed quaternion metrics (if available)
    if (q_proc) {
        Serial.printf("q_proc=[%.4f,%.4f,%.4f,%.4f] norm_proc=%.4f dot_proc_prev=%.4f ",
                      q_proc->w, q_proc->x, q_proc->y, q_proc->z, norm_proc, dot_proc_prev);
        Serial.printf("ang_diff_raw_proc=%.4f ang_diff_proc_prev=%.4f ",
                      angle_diff_raw_vs_proc, angle_diff_proc_vs_prev);
    }
    
    // Smoothing state
    Serial.printf("smooth_active=%d smooth_samples=%d ",
                  smoothing_active ? 1 : 0, smoothing_samples);
    
    // Validity and I2C checks
    Serial.printf("valid=%d zero=%d nan=%d norm_bad=%d rep=%d ",
                  valid, is_all_zero, has_nan, bad_norm, is_repeated);
    
    // Calibration status
    if (bno) {
        Serial.printf("calib: sys=%d g=%d a=%d m=%d mode=%d ",
                     sys_cal, gyro_cal, accel_cal, mag_cal, mode);
    }
    
    // Gyro prediction error
    if (gyro_prediction_error >= 0.0f) {
        Serial.printf("gyro_err=%.4f ", gyro_prediction_error);
    }
    
    Serial.println();  // End of diagnostic line
    Serial.flush();    // Ensure output is sent immediately
    
    // Update state
    if (!state.initialized) {
        state.initialized = true;
    }
    state.prev_raw_quat = q_raw;
    state.prev_raw_quat_valid = q_raw;
    if (q_proc) {
        state.prev_proc_quat = *q_proc;
    }
#else
    (void)stream_name;
    (void)state;
    (void)q_raw;
    (void)q_prev_raw;
    (void)bno;
    (void)gyro_prediction_error;
    (void)q_proc;
    (void)q_prev_proc;
    (void)smoothing_active;
    (void)smoothing_samples;
#endif
}

/**
 * Initialize diagnostic logging (call once at startup)
 */
inline void init_diagnostic_logging(uint32_t log_interval_ms = 20) {
#if ENABLE_DIAGNOSTIC_LOGGING
    // Use build-time log interval if defined
    #ifdef DIAGNOSTIC_LOG_INTERVAL_MS
    if (log_interval_ms == 20) {  // Only override if using default
        log_interval_ms = DIAGNOSTIC_LOG_INTERVAL_MS;
    }
    #endif
    
    Serial.println("\n=== IMU Diagnostic Logging Enabled ===");
    Serial.printf("Log interval: %lu ms (%.1f Hz)\n", 
                 log_interval_ms, 1000.0f / log_interval_ms);
    Serial.println("Format: [DIAG stream] #sample t=time q_raw=[w,x,y,z] norm_raw=... dot_raw_prev=...");
    Serial.println("  q_proc=[w,x,y,z] norm_proc=... dot_proc_prev=... ang_diff_raw_proc=... ang_diff_proc_prev=...");
    Serial.println("  smooth_active=... smooth_samples=... valid=... calib=... gyro_err=...");
    Serial.println("=====================================\n");
    
#if defined(DUAL_IMU_MODE) && DUAL_IMU_MODE
    diag_state_bno.log_interval_ms = log_interval_ms;
    diag_state_mpu.log_interval_ms = log_interval_ms;
    diag_state_fused.log_interval_ms = log_interval_ms;
#else
    diag_state.log_interval_ms = log_interval_ms;
#endif
#else
    (void)log_interval_ms;
#endif
}

#endif // IMU_DIAGNOSTICS_H
