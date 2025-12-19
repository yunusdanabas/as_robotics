/*
 * ESP32 IMU Master Device - BNO055 with micro-ROS (WiFi Transport)
 * 
 * Build environments:
 * - esp32dev: Single BNO055 mode -> /imu_data
 * - esp32dev_dual: BNO055 + MPU6050 -> /imu1_data, /imu2_data, /imu_fused
 * 
 * Performance: I2C at 400kHz, Status check every 10s, Target 50+ Hz
 */

// Skip this file when building MPU6050-only mode
#if !defined(MPU6050_ONLY_MODE)

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <cmath>
#include <cstring>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>

#if POSITION_MODE
#include <nav_msgs/msg/odometry.h>
#include <micro_ros_utilities/string_utilities.h>
#include <Adafruit_BMP280.h>
#endif

// Sensor fusion utilities (quaternion processing, sign continuity)
#include "sensor_fusion.h"

// Diagnostic logging (optional, controlled by ENABLE_DIAGNOSTIC_LOGGING flag)
#include "imu_diagnostics.h"

// Default value for BLOCKING_STATUS_CHECK flag (0 = non-blocking, 1 = blocking)
// When non-blocking, status errors are logged but data continues to publish
#ifndef BLOCKING_STATUS_CHECK
#define BLOCKING_STATUS_CHECK 0
#endif

// Dual IMU mode includes
#if DUAL_IMU_MODE
#include <Adafruit_MPU6050.h>
// Note: Magnetometer includes (Adafruit_HMC5883_U.h, QMC5883LCompass.h) removed
// as magnetometer is disabled to avoid I2C errors on the 10DOF board
#endif

// WiFi configuration (create wifi_credentials.h from wifi_config.h)
#include "wifi_config.h"

// micro-ROS objects
#if DUAL_IMU_MODE
// Dual IMU mode: three publishers
rcl_publisher_t publisher_imu1;      // BNO055
rcl_publisher_t publisher_imu2;      // MPU6050 fused
rcl_publisher_t publisher_fused;     // Combined fusion
sensor_msgs__msg__Imu imu1_msg;
sensor_msgs__msg__Imu imu2_msg;
sensor_msgs__msg__Imu fused_msg;
#else
// Single IMU mode: one publisher
rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;

// Quaternion tracker for sign continuity (prevents teleportation)
QuaternionTracker quat_tracker;

// Previous raw quaternion for hemisphere alignment (BNO055 stream)
Quaternion prev_bno_raw_quat;
bool prev_bno_raw_initialized = false;
#endif

#if POSITION_MODE
rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;
#endif

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// BNO055 sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/**
 * Read BNO055 quaternion with validation and retry logic
 * @param bno BNO055 sensor object
 * @param quat Output quaternion (Adafruit format)
 * @param max_retries Maximum number of retry attempts
 * @return true if valid quaternion read, false otherwise
 */
bool read_bno055_quaternion_safe(Adafruit_BNO055& bno, imu::Quaternion& quat, int max_retries) {
    for (int attempt = 0; attempt < max_retries; attempt++) {
        quat = bno.getQuat();
        
        // Check for all-zero quaternion
        if (quat.w() == 0.0f && quat.x() == 0.0f && 
            quat.y() == 0.0f && quat.z() == 0.0f) {
            if (attempt < max_retries - 1) {
                delay(1);  // Brief delay before retry
                continue;
            }
            return false;  // All zeros after all retries
        }
        
        // Check for NaN/inf values
        if (!std::isfinite(quat.w()) || !std::isfinite(quat.x()) ||
            !std::isfinite(quat.y()) || !std::isfinite(quat.z())) {
            if (attempt < max_retries - 1) {
                delay(1);
                continue;
            }
            return false;  // NaN/inf after all retries
        }
        
        // Check quaternion norm (should be ~1.0)
        float norm = sqrtf(quat.w() * quat.w() + quat.x() * quat.x() + 
                          quat.y() * quat.y() + quat.z() * quat.z());
        if (norm < 0.1f || norm > 2.0f || !std::isfinite(norm)) {
            if (attempt < max_retries - 1) {
                delay(1);
                continue;
            }
            return false;  // Bad norm after all retries
        }
        
        // Valid quaternion read
        return true;
    }
    
    return false;  // Failed after all retries
}

// Grace period after initialization (3 seconds) - allow sensor to stabilize
static uint32_t bno055_init_time_ms = 0;
static const uint32_t BNO055_GRACE_PERIOD_MS = 3000;  // 3 seconds

// Status check interval - only check every N milliseconds to avoid I2C overhead
static const uint32_t STATUS_CHECK_INTERVAL_MS = 10000;  // Check every 10 seconds (reduced from 1s for performance)

/**
 * Check BNO055 status register for fusion errors
 * Less strict check - only fails on actual critical errors
 * NOTE: Only performs I2C read periodically to avoid slowing down the main loop
 * @param bno BNO055 sensor object
 * @return true if status is OK or in grace period, false if critical error detected
 */
bool check_bno055_status(Adafruit_BNO055& bno) {
    static uint32_t last_check_ms = 0;
    static bool last_status_ok = true;
    static uint8_t consecutive_errors = 0;
    static uint32_t last_status_log_ms = 0;
    static bool first_log = true;
    
    uint32_t now_ms = millis();
    
    // Allow grace period after initialization for sensor to stabilize
    bool in_grace_period = (bno055_init_time_ms > 0 && (now_ms - bno055_init_time_ms) < BNO055_GRACE_PERIOD_MS);
    
    if (in_grace_period) {
        return true;  // In grace period - allow operation
    }
    
    // Only perform I2C read periodically to avoid slowing down the main loop
    // Return cached status between checks
    if (now_ms - last_check_ms < STATUS_CHECK_INTERVAL_MS) {
        return last_status_ok;
    }
    last_check_ms = now_ms;
    
    // BNO055 status register is at 0x39
    // getSystemStatus() returns: system_status, self_test_result, system_error
    uint8_t system_status, self_test_result, system_error;
    
    // Read status (I2C operation - only happens once per STATUS_CHECK_INTERVAL_MS)
    bno.getSystemStatus(&system_status, &self_test_result, &system_error);
    
    // Retry once if we get suspicious values (might be I2C glitch)
    if (system_error == 0xFF || system_error > 0x0A) {
        delay(1);  // Brief delay before retry
        bno.getSystemStatus(&system_status, &self_test_result, &system_error);
    }
    
    // Check for errors
    bool error_detected = (system_error != 0 && system_error != 0xFF);
    
    if (error_detected) {
        consecutive_errors++;
    } else {
        consecutive_errors = 0;
    }
    
    // Log status periodically or on error
    bool should_log = first_log || (now_ms - last_status_log_ms > 10000);
    if (should_log || error_detected) {
        Serial.printf("[STATUS] sys=%d self_test=%d error=%d grace=%d elapsed=%lu consec_errors=%d\n", 
                      system_status, self_test_result, system_error,
                      in_grace_period ? 1 : 0,
                      bno055_init_time_ms > 0 ? (now_ms - bno055_init_time_ms) : 0,
                      consecutive_errors);
        last_status_log_ms = now_ms;
        first_log = false;
    }
    
    // Only fail on persistent errors (3+ consecutive checks with errors)
    if (error_detected && consecutive_errors >= 3) {
        Serial.printf("[STATUS ERROR] BNO055 system_error=%d (persistent, %d consecutive)\n", 
                      system_error, consecutive_errors);
        last_status_ok = false;
        return false;
    }
    
    last_status_ok = true;
    return true;
}

#if DUAL_IMU_MODE
// MPU6050 sensor object
Adafruit_MPU6050 mpu;

// Fast Madgwick filter for MPU6050 sensor fusion (responsive, like BNO055)
FastMadgwick madgwick_filter;

// Sensor status flags
bool mpu_initialized = false;

// Quaternion tracking for sign continuity
QuaternionTracker tracker_imu1;
QuaternionTracker tracker_imu2;
QuaternionTracker tracker_fused;

// Previous raw quaternions for hemisphere alignment
Quaternion prev_bno_raw_quat_dual;  // For BNO055 in dual IMU mode
bool prev_bno_raw_initialized_dual = false;
Quaternion prev_mpu_raw_quat;
bool prev_mpu_raw_initialized = false;

// Previous fused quaternion for hemisphere alignment
Quaternion prev_fused_quat;
bool prev_fused_initialized = false;

// Fusion parameters
static constexpr float FUSION_BLEND_FACTOR = 0.5f;  // 0.0 = IMU1 only, 1.0 = IMU2 only
static constexpr bool USE_SLERP_FUSION = true;      // true = SLERP, false = weighted average

// Filter settings for MPU6050
static constexpr float FILTER_UPDATE_RATE = 50.0f;  // Hz - publishing rate
static constexpr float MADGWICK_BETA = 0.8f;        // Higher = more responsive (like BNO055)
#endif

#if POSITION_MODE
// BMP280 sensor object
Adafruit_BMP280 bmp;

// Position integration constants
static constexpr float GRAVITY = 9.80665f;
static constexpr float SEA_LEVEL_PRESSURE_HPA = 1013.25f;
static constexpr float STATIONARY_ACCEL_THRESHOLD = 0.12f; // m/s^2
static constexpr float STATIONARY_GYRO_THRESHOLD = 0.05f;  // rad/s
static constexpr float VELOCITY_DAMPING = 0.08f;           // per second
static constexpr float VELOCITY_EPSILON = 0.002f;          // m/s
static constexpr float ALTITUDE_BLEND = 0.05f;
static constexpr float ALTITUDE_VELOCITY_GAIN = 0.2f;
static constexpr size_t ACCEL_BIAS_SAMPLE_COUNT = 200;
static constexpr size_t ALTITUDE_BASELINE_SAMPLES = 60;

struct PositionIntegrationState {
  bool initialized = false;
  bool bias_ready = false;
  uint16_t bias_samples = 0;
  float accel_bias[3] = {0.0f, 0.0f, 0.0f};
  float velocity[3] = {0.0f, 0.0f, 0.0f};
  float position[3] = {0.0f, 0.0f, 0.0f};
  uint32_t last_ms = 0;
} pos_state;

bool bmp_initialized = false;
bool altitude_ready = false;
float altitude_baseline = 0.0f;
size_t altitude_baseline_samples = 0;

inline float magnitude3(const float v[3]) {
  return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

inline void damp_velocity(float velocity[3], float factor) {
  const float clamp = fmaxf(0.0f, fminf(1.0f, factor));
  for (int i = 0; i < 3; ++i) {
    velocity[i] *= (1.0f - clamp);
    if (fabsf(velocity[i]) < VELOCITY_EPSILON) {
      velocity[i] = 0.0f;
    }
  }
}

void update_position_estimate(const imu::Quaternion &quat,
                              const imu::Vector<3> &accel_raw_sensor,
                              const imu::Vector<3> &gyro,
                              uint32_t now_ms) {
  imu::Quaternion q = quat;
  q.normalize();

  const float qw = q.w();
  const float qx = q.x();
  const float qy = q.y();
  const float qz = q.z();

  const float r00 = 1.0f - 2.0f * (qy * qy + qz * qz);
  const float r01 = 2.0f * (qx * qy - qz * qw);
  const float r02 = 2.0f * (qx * qz + qy * qw);
  const float r10 = 2.0f * (qx * qy + qz * qw);
  const float r11 = 1.0f - 2.0f * (qx * qx + qz * qz);
  const float r12 = 2.0f * (qy * qz - qx * qw);
  const float r20 = 2.0f * (qx * qz - qy * qw);
  const float r21 = 2.0f * (qy * qz + qx * qw);
  const float r22 = 1.0f - 2.0f * (qx * qx + qy * qy);

  float gravity_sensor[3];
  gravity_sensor[0] = r00 * 0.0f + r10 * 0.0f + r20 * GRAVITY;
  gravity_sensor[1] = r01 * 0.0f + r11 * 0.0f + r21 * GRAVITY;
  gravity_sensor[2] = r02 * 0.0f + r12 * 0.0f + r22 * GRAVITY;

  float linear_sensor[3];
  linear_sensor[0] = accel_raw_sensor.x() - gravity_sensor[0];
  linear_sensor[1] = accel_raw_sensor.y() - gravity_sensor[1];
  linear_sensor[2] = accel_raw_sensor.z() - gravity_sensor[2];

  if (!pos_state.bias_ready) {
    pos_state.accel_bias[0] += linear_sensor[0];
    pos_state.accel_bias[1] += linear_sensor[1];
    pos_state.accel_bias[2] += linear_sensor[2];
    pos_state.bias_samples++;
    if (pos_state.bias_samples >= ACCEL_BIAS_SAMPLE_COUNT) {
      pos_state.accel_bias[0] /= pos_state.bias_samples;
      pos_state.accel_bias[1] /= pos_state.bias_samples;
      pos_state.accel_bias[2] /= pos_state.bias_samples;
      pos_state.bias_ready = true;
      Serial.println("Acceleration bias calibrated.");
    }
    return;
  }

  linear_sensor[0] -= pos_state.accel_bias[0];
  linear_sensor[1] -= pos_state.accel_bias[1];
  linear_sensor[2] -= pos_state.accel_bias[2];

  float linear_world[3];
  linear_world[0] = r00 * linear_sensor[0] + r01 * linear_sensor[1] + r02 * linear_sensor[2];
  linear_world[1] = r10 * linear_sensor[0] + r11 * linear_sensor[1] + r12 * linear_sensor[2];
  linear_world[2] = r20 * linear_sensor[0] + r21 * linear_sensor[1] + r22 * linear_sensor[2];

  if (!pos_state.initialized) {
    pos_state.initialized = true;
    pos_state.last_ms = now_ms;
    return;
  }

  float dt = (now_ms - pos_state.last_ms) / 1000.0f;
  pos_state.last_ms = now_ms;

  if (dt <= 0.0f || dt > 0.2f) {
    return;
  }

  const float accel_norm = sqrtf(linear_world[0] * linear_world[0] +
                                 linear_world[1] * linear_world[1] +
                                 linear_world[2] * linear_world[2]);
  const float gyro_norm = sqrtf(gyro.x() * gyro.x() + gyro.y() * gyro.y() + gyro.z() * gyro.z());
  const bool stationary = accel_norm < STATIONARY_ACCEL_THRESHOLD && gyro_norm < STATIONARY_GYRO_THRESHOLD;

  if (!stationary) {
    pos_state.velocity[0] += linear_world[0] * dt;
    pos_state.velocity[1] += linear_world[1] * dt;
    pos_state.velocity[2] += linear_world[2] * dt;
    damp_velocity(pos_state.velocity, VELOCITY_DAMPING * dt);
  } else {
    damp_velocity(pos_state.velocity, 0.5f);
  }

  pos_state.position[0] += pos_state.velocity[0] * dt;
  pos_state.position[1] += pos_state.velocity[1] * dt;
  pos_state.position[2] += pos_state.velocity[2] * dt;

  if (bmp_initialized) {
    float altitude_m = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);
    if (!altitude_ready) {
      altitude_baseline += altitude_m;
      altitude_baseline_samples++;
      if (altitude_baseline_samples >= ALTITUDE_BASELINE_SAMPLES) {
        altitude_baseline /= altitude_baseline_samples;
        altitude_ready = true;
        Serial.println("Altitude baseline established.");
      }
    } else {
      const float altitude_rel = altitude_m - altitude_baseline;
      const float altitude_error = altitude_rel - pos_state.position[2];
      pos_state.position[2] += ALTITUDE_BLEND * altitude_error;
      pos_state.velocity[2] += ALTITUDE_VELOCITY_GAIN * altitude_error * dt;
    }
  }
}
#endif

// Status variables
bool sensor_initialized = false;
bool micro_ros_initialized = false;
bool wifi_connected = false;

// Glitch tracking removed - 400kHz I2C fixed root cause of glitches

// LED pin for status indication
const int LED_PIN = 2;

// Error macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Forward declarations
bool init_bno055();
bool check_bno055_status(Adafruit_BNO055& bno);
bool read_bno055_quaternion_safe(Adafruit_BNO055& bno, imu::Quaternion& quat, int max_retries = 3);

// Error handling function
void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

// WiFi connection function
bool connect_wifi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    
    if (millis() - start > WIFI_TIMEOUT) {
      Serial.println("\nWiFi connection timeout!");
      return false;
    }
  }
  
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Signal strength (RSSI): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  
  digitalWrite(LED_PIN, HIGH);
  return true;
}

/**
 * Initialize BNO055 sensor
 */
bool init_bno055() {
  Serial.println("\nInitializing BNO055 sensor...");
  if (!bno.begin()) {
    return false;
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
  
  // Use IMUPLUS mode (no magnetometer) to prevent Z-axis jumps from magnetic interference
  // Trade-off: Yaw will drift slowly over time, but no sudden heading corrections
  bno.setMode(OPERATION_MODE_IMUPLUS);
  Serial.println("BNO055 mode: IMUPLUS (magnetometer disabled)");
  Serial.println("BNO055 initialized successfully!");
  return true;
}

// Glitch recovery functions removed - 400kHz I2C fixed root cause

#if DUAL_IMU_MODE
/**
 * Initialize MPU6050 sensor
 */
bool init_mpu6050() {
  Serial.println("Initializing MPU6050...");
  
  if (!mpu.begin(0x68)) {
    Serial.println("WARNING: MPU6050 not found at 0x68");
    return false;
  }
  
  // Configure MPU6050 settings
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("MPU6050 initialized successfully!");
  Serial.print("  Accelerometer range: +/- 4G\n");
  Serial.print("  Gyroscope range: +/- 500 deg/s\n");
  
  return true;
}

#endif  // DUAL_IMU_MODE

/**
 * Fill IMU message with common fields
 */
void fill_imu_msg_header(sensor_msgs__msg__Imu& msg, const char* frame_id) {
  uint64_t epoch_millis = rmw_uros_epoch_millis();
  msg.header.stamp.sec = epoch_millis / 1000;
  msg.header.stamp.nanosec = (epoch_millis % 1000) * 1000000;
  msg.header.frame_id.data = (char*)frame_id;
  msg.header.frame_id.size = strlen(frame_id);
}


// Timer callback - publishes IMU data
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  
  if (timer == NULL || !sensor_initialized) {
#if ENABLE_DIAGNOSTIC_LOGGING
    static uint32_t last_debug_ms = 0;
    uint32_t now_ms = millis();
    if (now_ms - last_debug_ms > 1000) {  // Debug every 1 second
      Serial.printf("[TIMER DEBUG] timer=%p sensor_init=%d\n", timer, sensor_initialized);
      last_debug_ms = now_ms;
    }
#endif
    return;
  }

#if DUAL_IMU_MODE
  // ===== DUAL IMU MODE =====
  
  // --- IMU1: BNO055 ---
  imu::Quaternion bno_quat;
  if (!read_bno055_quaternion_safe(bno, bno_quat)) {
    // Invalid read - skip this sample
    return;
  }
  
  imu::Vector<3> bno_gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> bno_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  Quaternion q1_raw(bno_quat.w(), bno_quat.x(), bno_quat.y(), bno_quat.z());
  
  // Hemisphere alignment: align to previous raw quaternion (prevents sign flips)
  if (prev_bno_raw_initialized_dual) {
    alignHemisphere(q1_raw, prev_bno_raw_quat_dual);
  } else {
    prev_bno_raw_initialized_dual = true;
  }
  prev_bno_raw_quat_dual = q1_raw;
  
  Quaternion q1 = q1_raw;
  Vec3 gyro1(bno_gyro.x(), bno_gyro.y(), bno_gyro.z());
  bool q1_valid = tracker_imu1.process(q1, &gyro1);
  
  // Diagnostic logging FIRST (before status check) to ensure visibility
#if ENABLE_DIAGNOSTIC_LOGGING
  float gyro_error = tracker_imu1.get_last_prediction_error();
  log_diagnostic_data("BNO055", diag_state_bno, q1_raw, 
                      diag_state_bno.initialized ? &diag_state_bno.prev_raw_quat_valid : nullptr,
                      &bno, gyro_error,
                      &q1,  // q_proc
                      diag_state_bno.initialized ? &diag_state_bno.prev_proc_quat : nullptr,  // q_prev_proc
                      tracker_imu1.smoothing_active,
                      tracker_imu1.smoothing_samples);
#endif
  
  // Check BNO055 status for errors (non-blocking by default)
  bool status_ok_dual = check_bno055_status(bno);
#if BLOCKING_STATUS_CHECK
  if (!status_ok_dual) {
    // Critical error detected - skip this sample (blocking mode)
    return;
  }
#else
  // Non-blocking mode: just log the status, don't skip samples
  (void)status_ok_dual;
#endif
  
  
  // Fill IMU1 message
  fill_imu_msg_header(imu1_msg, "imu1_link");
  imu1_msg.orientation.x = q1.x;
  imu1_msg.orientation.y = q1.y;
  imu1_msg.orientation.z = q1.z;
  imu1_msg.orientation.w = q1.w;
  imu1_msg.angular_velocity.x = bno_gyro.x();
  imu1_msg.angular_velocity.y = bno_gyro.y();
  imu1_msg.angular_velocity.z = bno_gyro.z();
  imu1_msg.linear_acceleration.x = bno_accel.x();
  imu1_msg.linear_acceleration.y = bno_accel.y();
  imu1_msg.linear_acceleration.z = bno_accel.z();
  
  // Publish IMU1
  if (q1_valid) {
    RCSOFTCHECK(rcl_publish(&publisher_imu1, &imu1_msg, NULL));
  }
  
  // --- IMU2: MPU6050 + Magnetometer (Madgwick fusion) ---
  Quaternion q2;
  bool q2_valid = false;
  
  if (mpu_initialized) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu.getEvent(&accel_event, &gyro_event, &temp_event);
    
    // Update Madgwick filter (IMU only - magnetometer disabled to avoid I2C errors)
    float gx = gyro_event.gyro.x;  // Already in rad/s
    float gy = gyro_event.gyro.y;
    float gz = gyro_event.gyro.z;
    float ax = accel_event.acceleration.x;
    float ay = accel_event.acceleration.y;
    float az = accel_event.acceleration.z;
    
    madgwick_filter.updateIMU(gx, gy, gz, ax, ay, az);
    
    // Get quaternion from filter
    float qw, qx, qy, qz;
    madgwick_filter.getQuaternion(&qw, &qx, &qy, &qz);
    Quaternion q2_raw(qw, qx, qy, qz);
    
    // Hemisphere alignment: align to previous raw quaternion (prevents sign flips)
    if (prev_mpu_raw_initialized) {
      alignHemisphere(q2_raw, prev_mpu_raw_quat);
    } else {
      prev_mpu_raw_initialized = true;
    }
    prev_mpu_raw_quat = q2_raw;
    
    q2 = q2_raw;
    Vec3 gyro2(gx, gy, gz);
    q2_valid = tracker_imu2.process(q2, &gyro2);
    
    // Diagnostic logging (after processing to include prediction error)
#if ENABLE_DIAGNOSTIC_LOGGING
    float gyro_error_mpu = tracker_imu2.get_last_prediction_error();
    log_diagnostic_data("MPU6050", diag_state_mpu, q2_raw,
                        diag_state_mpu.initialized ? &diag_state_mpu.prev_raw_quat_valid : nullptr,
                        nullptr, gyro_error_mpu,
                        &q2,  // q_proc
                        diag_state_mpu.initialized ? &diag_state_mpu.prev_proc_quat : nullptr,  // q_prev_proc
                        tracker_imu2.smoothing_active,
                        tracker_imu2.smoothing_samples);
#endif
    
    
    // Fill IMU2 message
    fill_imu_msg_header(imu2_msg, "imu2_link");
    imu2_msg.orientation.x = q2.x;
    imu2_msg.orientation.y = q2.y;
    imu2_msg.orientation.z = q2.z;
    imu2_msg.orientation.w = q2.w;
    imu2_msg.angular_velocity.x = gx;
    imu2_msg.angular_velocity.y = gy;
    imu2_msg.angular_velocity.z = gz;
    imu2_msg.linear_acceleration.x = ax;
    imu2_msg.linear_acceleration.y = ay;
    imu2_msg.linear_acceleration.z = az;
    
    // Publish IMU2
    if (q2_valid) {
      RCSOFTCHECK(rcl_publish(&publisher_imu2, &imu2_msg, NULL));
    }
  }
  
  // --- Fused IMU: Combine both orientations ---
  if (q1_valid && q2_valid) {
    Quaternion q_fused = fuse_quaternions(q1, q2, FUSION_BLEND_FACTOR, USE_SLERP_FUSION);
    
    // Hemisphere alignment: align fused output to previous fused output (prevents jumps)
    if (prev_fused_initialized) {
      alignHemisphere(q_fused, prev_fused_quat);
    } else {
      prev_fused_initialized = true;
    }
    prev_fused_quat = q_fused;
    
    // Apply sign continuity to fused quaternion (use BNO055 gyro for prediction)
    Vec3 gyro_fused(bno_gyro.x(), bno_gyro.y(), bno_gyro.z());
    tracker_fused.process(q_fused, &gyro_fused);
    
    // Diagnostic logging for fused quaternion (after processing to include prediction error)
#if ENABLE_DIAGNOSTIC_LOGGING
    float gyro_error_fused = tracker_fused.get_last_prediction_error();
    log_diagnostic_data("FUSED", diag_state_fused, q_fused,
                        diag_state_fused.initialized ? &diag_state_fused.prev_raw_quat_valid : nullptr,
                        &bno, gyro_error_fused,
                        &q_fused,  // q_proc (fused is both raw and processed in this case)
                        diag_state_fused.initialized ? &diag_state_fused.prev_proc_quat : nullptr,  // q_prev_proc
                        tracker_fused.smoothing_active,
                        tracker_fused.smoothing_samples);
#endif
    
    // Fill fused message
    fill_imu_msg_header(fused_msg, "imu_fused_link");
    fused_msg.orientation.x = q_fused.x;
    fused_msg.orientation.y = q_fused.y;
    fused_msg.orientation.z = q_fused.z;
    fused_msg.orientation.w = q_fused.w;
    
    // Use BNO055 angular velocity and acceleration for fused message
    fused_msg.angular_velocity.x = bno_gyro.x();
    fused_msg.angular_velocity.y = bno_gyro.y();
    fused_msg.angular_velocity.z = bno_gyro.z();
    fused_msg.linear_acceleration.x = bno_accel.x();
    fused_msg.linear_acceleration.y = bno_accel.y();
    fused_msg.linear_acceleration.z = bno_accel.z();
    
    // Publish fused
    RCSOFTCHECK(rcl_publish(&publisher_fused, &fused_msg, NULL));
  }
  
#else
  // ===== SINGLE IMU MODE =====
  
  // Get quaternion data from BNO055 (with validation and retry)
  imu::Quaternion bno_quat;
  if (!read_bno055_quaternion_safe(bno, bno_quat)) {
    // Invalid read - skip this sample
#if ENABLE_DIAGNOSTIC_LOGGING
    static uint32_t last_fail_log_ms = 0;
    uint32_t now_ms = millis();
    if (now_ms - last_fail_log_ms > 1000) {  // Log failure every 1 second max
      Serial.printf("[TIMER] BNO055 read failed\n");
      last_fail_log_ms = now_ms;
    }
#endif
    return;
  }
  
  // Get angular velocity (gyroscope)
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  // Get linear acceleration (gravity-compensated) for IMU message
  imu::Vector<3> accel_linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

#if POSITION_MODE
  // Get raw acceleration (includes gravity) for integration
  imu::Vector<3> accel_raw = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
#endif
  
  // Raw quaternion before processing
  Quaternion q_raw(bno_quat.w(), bno_quat.x(), bno_quat.y(), bno_quat.z());
  
  // Hemisphere alignment: align to previous raw quaternion (prevents sign flips)
  if (prev_bno_raw_initialized) {
    alignHemisphere(q_raw, prev_bno_raw_quat);
  } else {
    prev_bno_raw_initialized = true;
  }
  prev_bno_raw_quat = q_raw;
  
  // Process quaternion for sign continuity and jump smoothing (prevents teleportation)
  Quaternion q = q_raw;
  Vec3 gyro_single(gyro.x(), gyro.y(), gyro.z());
  bool q_valid = quat_tracker.process(q, &gyro_single);
  
  // Diagnostic logging FIRST (before status check) to ensure visibility
#if ENABLE_DIAGNOSTIC_LOGGING
  // Force first log immediately to verify callback is working
  static bool first_log_done = false;
  if (!first_log_done) {
    Serial.println("[DIAG] First diagnostic log - timer callback is working!");
    first_log_done = true;
  }
  
  float gyro_error_single = quat_tracker.get_last_prediction_error();
  log_diagnostic_data("BNO055", diag_state, q_raw,
                      diag_state.initialized ? &diag_state.prev_raw_quat_valid : nullptr,
                      &bno, gyro_error_single,
                      &q,  // q_proc
                      diag_state.initialized ? &diag_state.prev_proc_quat : nullptr,  // q_prev_proc
                      quat_tracker.smoothing_active,
                      quat_tracker.smoothing_samples);
#endif
  
  // Check BNO055 status for errors (non-blocking by default)
  // Status check logs errors but continues operation unless BLOCKING_STATUS_CHECK=1
  bool status_ok = check_bno055_status(bno);
#if BLOCKING_STATUS_CHECK
  if (!status_ok) {
    // Critical error detected - skip this sample (blocking mode)
    return;
  }
#else
  // Non-blocking mode: just log the status, don't skip samples
  (void)status_ok;
#endif
  
  
  // Only publish after warmup period
  if (!q_valid) {
    return;
  }
  
  // Fill the IMU message
  uint64_t epoch_millis = rmw_uros_epoch_millis();
  imu_msg.header.stamp.sec = epoch_millis / 1000;
  imu_msg.header.stamp.nanosec = (epoch_millis % 1000) * 1000000;
  imu_msg.header.frame_id.data = (char*)"imu_link";
  imu_msg.header.frame_id.size = strlen("imu_link");
  
  // Orientation (quaternion) - processed with sign continuity and jump smoothing
  imu_msg.orientation.x = q.x;
  imu_msg.orientation.y = q.y;
  imu_msg.orientation.z = q.z;
  imu_msg.orientation.w = q.w;
  
  // Angular velocity (rad/s)
  imu_msg.angular_velocity.x = gyro.x();
  imu_msg.angular_velocity.y = gyro.y();
  imu_msg.angular_velocity.z = gyro.z();
  
  // Linear acceleration (m/s^2)
  imu_msg.linear_acceleration.x = accel_linear.x();
  imu_msg.linear_acceleration.y = accel_linear.y();
  imu_msg.linear_acceleration.z = accel_linear.z();
  
  // Publish the IMU message
  RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));

#if POSITION_MODE
    update_position_estimate(bno_quat, accel_raw, gyro, millis());

    if (pos_state.bias_ready && pos_state.initialized) {
      odom_msg.header.stamp.sec = imu_msg.header.stamp.sec;
      odom_msg.header.stamp.nanosec = imu_msg.header.stamp.nanosec;
      odom_msg.pose.pose.position.x = pos_state.position[0];
      odom_msg.pose.pose.position.y = pos_state.position[1];
      odom_msg.pose.pose.position.z = pos_state.position[2];
      odom_msg.pose.pose.orientation.x = imu_msg.orientation.x;
      odom_msg.pose.pose.orientation.y = imu_msg.orientation.y;
      odom_msg.pose.pose.orientation.z = imu_msg.orientation.z;
      odom_msg.pose.pose.orientation.w = imu_msg.orientation.w;
      odom_msg.twist.twist.linear.x = pos_state.velocity[0];
      odom_msg.twist.twist.linear.y = pos_state.velocity[1];
      odom_msg.twist.twist.linear.z = pos_state.velocity[2];
      odom_msg.twist.twist.angular.x = gyro.x();
      odom_msg.twist.twist.angular.y = gyro.y();
      odom_msg.twist.twist.angular.z = gyro.z();

      RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    }
#endif

#endif  // DUAL_IMU_MODE
  
  
  // Blink LED to show activity
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(2000);
  
#if DUAL_IMU_MODE
  Serial.println("ESP32 IMU Master - Dual IMU Mode (WiFi)");
#else
  Serial.println("ESP32 IMU Master - WiFi Mode");
#endif
  Serial.println("==============================");
  
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Connect to WiFi
  if (!connect_wifi()) {
    Serial.println("Failed to connect to WiFi. Rebooting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }
  wifi_connected = true;
  
  // Initialize I2C with fast clock for better performance
  Wire.begin();
  Wire.setClock(400000);  // 400kHz (fast mode) - 4x faster than 100kHz
  
  // Initialize BNO055 sensor (required for both modes)
  if (!init_bno055()) {
    Serial.println("ERROR: Failed to initialize BNO055. Check wiring!");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }
  }
  sensor_initialized = true;  // Mark sensor as ready for timer callback
  bno055_init_time_ms = millis();  // Record initialization time for grace period
  Serial.printf("[INIT] BNO055 init timestamp set: %lu ms (grace period: %lu ms)\n", 
                bno055_init_time_ms, BNO055_GRACE_PERIOD_MS);

#if DUAL_IMU_MODE
  // Initialize MPU6050
  mpu_initialized = init_mpu6050();
  
  // Initialize Fast Madgwick filter with high beta for BNO055-like responsiveness
  madgwick_filter.begin(FILTER_UPDATE_RATE, MADGWICK_BETA);
  Serial.printf("Fast Madgwick filter initialized (rate=%.0f Hz, beta=%.2f)\n", 
                FILTER_UPDATE_RATE, MADGWICK_BETA);
  
  if (!mpu_initialized) {
    Serial.println("WARNING: Running in degraded mode (BNO055 only)");
  }
#endif
  
#if POSITION_MODE
  Serial.println("Initializing BMP280 sensor...");
  if (bmp.begin(0x76)) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_63);
    bmp_initialized = true;
    altitude_baseline = 0.0f;
    altitude_baseline_samples = 0;
    altitude_ready = false;
    Serial.println("BMP280 initialized.");
    Serial.println("Hold the device steady for bias calibration...");
  } else {
    Serial.println("WARNING: BMP280 not detected. Altitude correction disabled.");
  }

  memset(&odom_msg, 0, sizeof(odom_msg));
  pos_state = PositionIntegrationState();
#endif

  // Display sensor details
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.println("------------------------------------");
  
  // Parse agent IP
  IPAddress agent_ip;
  agent_ip.fromString(AGENT_IP);
  
  // Set up micro-ROS WiFi transport
  Serial.println("\nSetting up micro-ROS WiFi transport...");
  Serial.print("Agent IP: "); Serial.println(AGENT_IP);
  Serial.print("Agent Port: "); Serial.println(AGENT_PORT);
  
  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASSWORD, agent_ip, AGENT_PORT);
  
  delay(1000);
  
  // Wait for micro-ROS agent with timeout
  Serial.println("Waiting for micro-ROS agent...");
  const int MAX_AGENT_ATTEMPTS = 30;  // 30 attempts * 500ms = 15 seconds max
  int agent_attempts = 0;
  bool agent_found = false;
  
  while (agent_attempts < MAX_AGENT_ATTEMPTS) {
    agent_attempts++;
    Serial.printf("  Ping attempt %d/%d... ", agent_attempts, MAX_AGENT_ATTEMPTS);
    
    // rmw_uros_ping_agent returns RMW_RET_OK if agent is reachable
    if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
      Serial.println("Agent found!");
      agent_found = true;
      break;
    }
    Serial.println("no response");
    delay(400);  // Wait before retry (100ms ping + 400ms delay = 500ms per attempt)
  }
  
  if (!agent_found) {
    Serial.println("ERROR: micro-ROS agent not found after 15 seconds!");
    Serial.println("Make sure the agent is running:");
    Serial.println("  cd ~/as_robotics && ./run_agent_wifi.sh 8888");
    Serial.println("Rebooting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }
  
  // Initialize micro-ROS
  Serial.println("Initializing micro-ROS...");
  allocator = rcl_get_default_allocator();
  
  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Create node
#if DUAL_IMU_MODE
  RCCHECK(rclc_node_init_default(&node, "esp32_dual_imu_master", "", &support));
#else
  RCCHECK(rclc_node_init_default(&node, "esp32_imu_master_wifi", "", &support));
#endif
  
  // Create publishers
#if DUAL_IMU_MODE
  // IMU1 publisher (BNO055)
  RCCHECK(rclc_publisher_init_default(
    &publisher_imu1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu1_data"));
  
  // IMU2 publisher (MPU6050)
  RCCHECK(rclc_publisher_init_default(
    &publisher_imu2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu2_data"));
  
  // Fused publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_fused,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_fused"));
  
  // Initialize messages
  memset(&imu1_msg, 0, sizeof(imu1_msg));
  memset(&imu2_msg, 0, sizeof(imu2_msg));
  memset(&fused_msg, 0, sizeof(fused_msg));
#else
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_data"));
#endif

#if POSITION_MODE
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "imu_odometry"));

  odom_msg.header.frame_id = micro_ros_string_utilities_init("world");
  odom_msg.child_frame_id = micro_ros_string_utilities_init("imu_link");
  for (size_t i = 0; i < 36; ++i) {
    odom_msg.pose.covariance[i] = 0.0f;
    odom_msg.twist.covariance[i] = 0.0f;
  }
#endif
  
  // Create timer (publish at 100 Hz to compensate for I2C and WiFi overhead)
  // Actual rate will be lower due to I2C reads (~5-10ms each) and network latency
  const unsigned int timer_timeout = 10;  // 10ms = 100Hz target
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  micro_ros_initialized = true;
  Serial.println("micro-ROS initialized successfully!");
  
#if DUAL_IMU_MODE
  Serial.println("Publishing IMU1 data to topic: /imu1_data");
  Serial.println("Publishing IMU2 data to topic: /imu2_data");
  Serial.println("Publishing fused data to topic: /imu_fused");
#else
  Serial.println("Publishing IMU data to topic: /imu_data");
#endif

#if POSITION_MODE
  Serial.println("Publishing odometry data to topic: /imu_odometry");
#endif

  Serial.println("Ready to communicate with micro-ROS agent!");
  Serial.println("==============================");
  
  // Initialize diagnostic logging
  #ifdef DIAGNOSTIC_LOG_INTERVAL_MS
  init_diagnostic_logging(DIAGNOSTIC_LOG_INTERVAL_MS);
  #else
  init_diagnostic_logging(100);  // Default: Log every 100ms (10 Hz) to avoid serial overflow
  #endif
  
  // Steady LED on to indicate ready state
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
}

void loop() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Reconnecting...");
    wifi_connected = false;
    if (!connect_wifi()) {
      Serial.println("Reconnection failed. Rebooting...");
      delay(5000);
      ESP.restart();
    }
    wifi_connected = true;
  }
  
  if (micro_ros_initialized) {
    // Execute callbacks - use minimal timeout for higher throughput
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  }
  
  // Minimal delay to yield to other tasks (WiFi stack, etc.)
  delay(1);
}

#endif  // !MPU6050_ONLY_MODE
