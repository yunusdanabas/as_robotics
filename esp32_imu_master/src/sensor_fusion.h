/*
 * Sensor Fusion Utilities for IMU System
 * 
 * This header provides:
 * - Quaternion math utilities (normalization, SLERP, weighted average)
 * - Sign continuity handling for smooth quaternion transitions
 * - Jump detection and smoothing to prevent teleportation
 * - Madgwick filter wrapper for MPU6050 sensor fusion (dual IMU mode)
 */

#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <cmath>
#include <Arduino.h>

// Quaternion structure for convenience
struct Quaternion {
    float w, x, y, z;
    
    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
    
    // Compute quaternion norm
    float norm() const {
        return sqrtf(w*w + x*x + y*y + z*z);
    }
    
    // Normalize quaternion in place
    void normalize() {
        float n = norm();
        if (n > 1e-6f) {
            w /= n;
            x /= n;
            y /= n;
            z /= n;
        }
    }
    
    // Return normalized copy
    Quaternion normalized() const {
        Quaternion q = *this;
        q.normalize();
        return q;
    }
    
    // Dot product with another quaternion
    float dot(const Quaternion& other) const {
        return w*other.w + x*other.x + y*other.y + z*other.z;
    }
    
    // Negate quaternion (represents same rotation)
    Quaternion operator-() const {
        return Quaternion(-w, -x, -y, -z);
    }
    
    // Scale quaternion
    Quaternion operator*(float s) const {
        return Quaternion(w*s, x*s, y*s, z*s);
    }
    
    // Add quaternions
    Quaternion operator+(const Quaternion& other) const {
        return Quaternion(w+other.w, x+other.x, y+other.y, z+other.z);
    }
};

/**
 * Apply sign continuity to prevent quaternion flipping
 * If the dot product with the previous quaternion is negative,
 * negate the current quaternion to take the shorter path.
 * 
 * @param current Current quaternion (will be modified if needed)
 * @param previous Previous quaternion for reference
 */
inline void apply_sign_continuity(Quaternion& current, const Quaternion& previous) {
    if (current.dot(previous) < 0.0f) {
        current = -current;
    }
}

/**
 * Align quaternion hemisphere for continuity
 * Wrapper around apply_sign_continuity with more descriptive name.
 * Ensures quaternion is in the same hemisphere as reference.
 * 
 * @param q Quaternion to align (will be modified if needed)
 * @param ref Reference quaternion for hemisphere alignment
 */
inline void alignHemisphere(Quaternion& q, const Quaternion& ref) {
    if (q.dot(ref) < 0.0f) {
        q = -q;
    }
}

/**
 * Spherical Linear Interpolation (SLERP) between two quaternions
 * 
 * @param q1 First quaternion (t=0)
 * @param q2 Second quaternion (t=1)
 * @param t Interpolation factor [0.0, 1.0]
 * @return Interpolated quaternion
 */
inline Quaternion slerp(const Quaternion& q1, const Quaternion& q2, float t) {
    // Compute dot product
    float dot = q1.dot(q2);
    
    // If dot is negative, negate one quaternion to take shorter path
    Quaternion q2_adj = q2;
    if (dot < 0.0f) {
        q2_adj = -q2;
        dot = -dot;
    }
    
    // If quaternions are very close, use linear interpolation
    const float DOT_THRESHOLD = 0.9995f;
    if (dot > DOT_THRESHOLD) {
        Quaternion result = q1 + (q2_adj + (q1 * -1.0f)) * t;
        result.normalize();
        return result;
    }
    
    // Calculate SLERP
    float theta_0 = acosf(dot);
    float theta = theta_0 * t;
    float sin_theta = sinf(theta);
    float sin_theta_0 = sinf(theta_0);
    
    float s1 = cosf(theta) - dot * sin_theta / sin_theta_0;
    float s2 = sin_theta / sin_theta_0;
    
    return Quaternion(
        q1.w * s1 + q2_adj.w * s2,
        q1.x * s1 + q2_adj.x * s2,
        q1.y * s1 + q2_adj.y * s2,
        q1.z * s1 + q2_adj.z * s2
    );
}

/**
 * Weighted average of two quaternions
 * 
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @param w1 Weight for first quaternion
 * @param w2 Weight for second quaternion
 * @return Weighted average quaternion (normalized)
 */
inline Quaternion weighted_average(const Quaternion& q1, const Quaternion& q2, 
                                    float w1, float w2) {
    // Ensure q2 is in the same hemisphere as q1
    Quaternion q2_adj = q2;
    if (q1.dot(q2) < 0.0f) {
        q2_adj = -q2;
    }
    
    // Compute weighted sum
    Quaternion result(
        q1.w * w1 + q2_adj.w * w2,
        q1.x * w1 + q2_adj.x * w2,
        q1.y * w1 + q2_adj.y * w2,
        q1.z * w1 + q2_adj.z * w2
    );
    
    result.normalize();
    return result;
}

/**
 * Fuse two quaternions using configurable method
 * 
 * @param q1 First quaternion (e.g., BNO055)
 * @param q2 Second quaternion (e.g., MPU6050)
 * @param blend_factor For SLERP: 0.0=q1, 1.0=q2, 0.5=equal blend
 * @param use_slerp If true, use SLERP; otherwise use weighted average
 * @return Fused quaternion
 */
inline Quaternion fuse_quaternions(const Quaternion& q1, const Quaternion& q2,
                                    float blend_factor, bool use_slerp = true) {
    // Align q2 to q1 hemisphere before fusion (critical for SLERP to take short path)
    Quaternion q2_aligned = q2;
    alignHemisphere(q2_aligned, q1);
    
    if (use_slerp) {
        return slerp(q1, q2_aligned, blend_factor);
    } else {
        float w1 = 1.0f - blend_factor;
        float w2 = blend_factor;
        return weighted_average(q1, q2_aligned, w1, w2);
    }
}

/**
 * Check if quaternion is valid (finite and non-zero norm)
 */
inline bool is_valid_quaternion(const Quaternion& q) {
    if (!std::isfinite(q.w) || !std::isfinite(q.x) || 
        !std::isfinite(q.y) || !std::isfinite(q.z)) {
        return false;
    }
    float n = q.norm();
    return n > 1e-6f && std::isfinite(n);
}

// Vector3 structure for gyro readings
struct Vec3 {
    float x, y, z;
    Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    float norm() const {
        return sqrtf(x*x + y*y + z*z);
    }
    
    Vec3 operator/(float s) const {
        return Vec3(x/s, y/s, z/s);
    }
};

/**
 * Predict next quaternion from previous quaternion and gyro reading
 * Uses gyro integration: dq/dt = 0.5 * q * [0, wx, wy, wz]
 * 
 * @param q_prev Previous quaternion
 * @param w Gyro reading in rad/s (Vec3)
 * @param dt Time step in seconds
 * @return Predicted quaternion
 */
inline Quaternion predictFromGyro(const Quaternion& q_prev, const Vec3& w, float dt) {
    float wnorm = w.norm();
    if (wnorm < 1e-6f) {
        return q_prev;  // No rotation
    }
    
    Vec3 u = w / wnorm;  // Unit vector
    float half = 0.5f * wnorm * dt;
    
    // Quaternion representing rotation: [cos(half*theta), sin(half*theta) * axis]
    Quaternion dq(cosf(half), u.x * sinf(half), u.y * sinf(half), u.z * sinf(half));
    
    // Multiply: q_new = q_prev * dq
    // Quaternion multiplication
    float w_new = q_prev.w * dq.w - q_prev.x * dq.x - q_prev.y * dq.y - q_prev.z * dq.z;
    float x_new = q_prev.w * dq.x + q_prev.x * dq.w + q_prev.y * dq.z - q_prev.z * dq.y;
    float y_new = q_prev.w * dq.y - q_prev.x * dq.z + q_prev.y * dq.w + q_prev.z * dq.x;
    float z_new = q_prev.w * dq.z + q_prev.x * dq.y - q_prev.y * dq.x + q_prev.z * dq.w;
    
    Quaternion q_pred(w_new, x_new, y_new, z_new);
    q_pred.normalize();
    return q_pred;
}

/**
 * Compute angular error between two quaternions (in radians)
 * Returns the angle of rotation needed to go from q1 to q2
 * 
 * @param a First quaternion
 * @param b Second quaternion
 * @return Angular error in radians [0, PI]
 */
inline float angErr(const Quaternion& a, const Quaternion& b) {
    float d = fabsf(a.dot(b));
    // Clamp to [0, 1] to avoid acos domain errors
    if (d > 1.0f) d = 1.0f;
    if (d < 0.0f) d = 0.0f;
    return 2.0f * acosf(d);
}

// ============================================================================
// Fast Madgwick Filter with Adjustable Beta (for responsive orientation)
// ============================================================================
class FastMadgwick {
public:
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion
  float beta = 0.5f;  // filter gain (higher = more responsive)
  float sampleFreq = 50.0f;
  
  void begin(float freq, float b = 0.5f) {
    sampleFreq = freq;
    beta = b;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
  }
  
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      // Normalise accelerometer measurement
      recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      // Auxiliary variables
      _2q0 = 2.0f * q0;
      _2q1 = 2.0f * q1;
      _2q2 = 2.0f * q2;
      _2q3 = 2.0f * q3;
      _4q0 = 4.0f * q0;
      _4q1 = 4.0f * q1;
      _4q2 = 4.0f * q2;
      _8q1 = 8.0f * q1;
      _8q2 = 8.0f * q2;
      q0q0 = q0 * q0;
      q1q1 = q1 * q1;
      q2q2 = q2 * q2;
      q3q3 = q3 * q3;

      // Gradient descent corrective step
      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
      recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      // Apply feedback step
      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }

    // Integrate rate of change
    float dt = 1.0f / sampleFreq;
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
  }
  
  void getQuaternion(float *w, float *x, float *y, float *z) {
    *w = q0;
    *x = q1;
    *y = q2;
    *z = q3;
  }
};

/**
 * State tracking for sign continuity per IMU with smooth jump interpolation
 */
struct QuaternionTracker {
    Quaternion last_output_quat;  // Last quaternion we output
    Quaternion smoothing_start;  // Starting point for current smoothing operation
    Quaternion target_quat;        // Target quaternion (from sensor)
    Vec3 last_gyro;                // Last gyro reading for prediction
    bool initialized;
    int warmup_count;
    bool smoothing_active;         // Are we currently smoothing a jump?
    int smoothing_samples;         // How many samples we've been smoothing
    float last_prediction_error;   // Last computed angular error (radians) for diagnostics
    static constexpr int SMOOTHING_DURATION = 15;  // ~300ms at 50Hz to smooth a jump
    
    // Glitch detection threshold (angular error in radians)
    // Start at 0.25 rad (~14 degrees) at 50 Hz
    // This distinguishes real fast motion from sensor glitches
    #ifdef GLITCH_THRESHOLD_RAD_VALUE
    static constexpr float GLITCH_THRESHOLD_RAD = GLITCH_THRESHOLD_RAD_VALUE;
    #else
    static constexpr float GLITCH_THRESHOLD_RAD = 0.25f;
    #endif
    
    // Smoothing parameters
    #ifdef ALWAYS_ON_SMOOTHING_ALPHA_VALUE
    static constexpr float ALWAYS_ON_SMOOTHING_ALPHA = ALWAYS_ON_SMOOTHING_ALPHA_VALUE;
    #else
    static constexpr float ALWAYS_ON_SMOOTHING_ALPHA = 0.15f;  // Light always-on smoothing
    #endif
    
    #ifdef GLITCH_MODE_SMOOTHING_ALPHA_VALUE
    static constexpr float GLITCH_MODE_SMOOTHING_ALPHA = GLITCH_MODE_SMOOTHING_ALPHA_VALUE;
    #else
    static constexpr float GLITCH_MODE_SMOOTHING_ALPHA = 0.01f;  // Very slow smoothing in glitch mode
    #endif
    
    // Update rate (Hz) - used for dt calculation
    static constexpr float UPDATE_RATE = 50.0f;
    static constexpr float DT = 1.0f / UPDATE_RATE;
    
    QuaternionTracker() : initialized(false), warmup_count(10), 
                          smoothing_active(false), smoothing_samples(0),
                          last_prediction_error(-1.0f) {}
    
    /**
     * Calculate angular distance between two quaternions (in radians)
     */
    static float angular_distance(const Quaternion& q1, const Quaternion& q2) {
        float dot = fabsf(q1.dot(q2));
        if (dot > 1.0f) dot = 1.0f;
        return 2.0f * acosf(dot);
    }
    
    /**
     * Process incoming quaternion with sign continuity, warmup, and physics-based glitch detection
     * @param q Incoming quaternion (will be modified to output value)
     * @param gyro_rad_s Gyro reading in rad/s (optional, nullptr if not available)
     * @param dt Time step in seconds (default: 1/50 Hz = 0.02s)
     * @return true if quaternion should be used (past warmup), false otherwise
     */
    bool process(Quaternion& q, const Vec3* gyro_rad_s = nullptr, float dt = DT) {
        if (!is_valid_quaternion(q)) {
            return false;
        }
        
        q.normalize();
        
        if (initialized) {
            // Apply sign continuity to incoming quaternion
            apply_sign_continuity(q, last_output_quat);
            
            if (smoothing_active) {
                // Glitch mode: use very slow smoothing or freeze
                smoothing_samples++;
                float t = (float)smoothing_samples / SMOOTHING_DURATION;
                
                if (t >= 1.0f || smoothing_samples >= SMOOTHING_DURATION) {
                    // Smoothing complete - apply always-on smoothing to target
                    Quaternion q_smoothed = slerp(last_output_quat, target_quat, ALWAYS_ON_SMOOTHING_ALPHA);
                    q = q_smoothed;
                    smoothing_active = false;
                    smoothing_samples = 0;
                } else {
                    // Glitch mode: very slow smoothing towards locked target
                    Quaternion q_interp = slerp(smoothing_start, target_quat, t);
                    // Apply glitch-mode smoothing (very slow)
                    q = slerp(last_output_quat, q_interp, GLITCH_MODE_SMOOTHING_ALPHA);
                    q.normalize();
                }
            } else {
                // Normal mode: apply always-on light smoothing first
                Quaternion q_measured = q;
                Quaternion q_smoothed = slerp(last_output_quat, q_measured, ALWAYS_ON_SMOOTHING_ALPHA);
                
                // Physics-based glitch detection using gyro prediction
                bool glitch = false;
                
                if (gyro_rad_s != nullptr) {
                    // Predict what the quaternion should be based on gyro
                    Quaternion q_pred = predictFromGyro(last_output_quat, *gyro_rad_s, dt);
                    
                    // Align measured quaternion to predicted hemisphere
                    alignHemisphere(q_measured, q_pred);
                    alignHemisphere(q_smoothed, q_pred);
                    
                    // Compute angular error between predicted and measured
                    float error = angErr(q_measured, q_pred);
                    last_prediction_error = error;  // Store for diagnostic logging
                    
                    if (error > GLITCH_THRESHOLD_RAD) {
                        // Glitch detected - enter glitch mode
                        glitch = true;
                    }
                    
                    // Store gyro for next prediction
                    last_gyro = *gyro_rad_s;
                } else {
                    // Fallback to simple threshold if no gyro available
                    float angle_diff = angular_distance(q_measured, last_output_quat);
                    last_prediction_error = angle_diff;  // Store for diagnostic logging
                    if (angle_diff > GLITCH_THRESHOLD_RAD * 3.0f) {  // More lenient without gyro
                        glitch = true;
                    }
                }
                
                if (glitch) {
                    // Glitch detected - enter glitch mode
                    smoothing_active = true;
                    smoothing_samples = 0;
                    smoothing_start = last_output_quat;  // Save starting point
                    target_quat = q_measured;  // Lock target - don't update during smoothing
                    
                    // First frame of glitch mode: freeze output (stay at last position)
                    q = last_output_quat;
                } else {
                    // Normal operation - use always-on smoothed result
                    q = q_smoothed;
                }
            }
        }
        
        // Update last output (this is what we actually published)
        last_output_quat = q;
        
        if (!initialized) {
            initialized = true;
        }
        
        if (warmup_count > 0) {
            warmup_count--;
            return false;
        }
        
        return true;
    }
    
    void reset() {
        initialized = false;
        warmup_count = 10;
        smoothing_active = false;
        smoothing_samples = 0;
        last_prediction_error = -1.0f;
    }
    
    /**
     * Get the last computed prediction error (for diagnostic logging)
     * @return Angular error in radians, or -1.0f if not computed yet
     */
    float get_last_prediction_error() const {
        return last_prediction_error;
    }
};

#endif // SENSOR_FUSION_H
