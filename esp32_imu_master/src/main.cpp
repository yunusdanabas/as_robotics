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

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#if POSITION_MODE
#include <nav_msgs/msg/odometry.h>
#include <micro_ros_utilities/string_utilities.h>
#include <Adafruit_BMP280.h>
#endif

// Sensor fusion utilities (quaternion processing, sign continuity)
#include "sensor_fusion.h"

// Diagnostic logging (optional, controlled by ENABLE_DIAGNOSTIC_LOGGING flag)
#include "imu_diagnostics.h"

// Centralized configuration constants
#include "imu_config.h"

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

// ============================================================================
// Lock-Free Sensor Cache for FreeRTOS Task
// ============================================================================

/**
 * Structure to hold cached sensor data from FreeRTOS polling task.
 * Uses volatile for cache coherency between cores.
 */
struct ImuSampleCache {
    float qw, qx, qy, qz;           // Quaternion components
    float gyro_x, gyro_y, gyro_z;   // Gyroscope (rad/s)
    float accel_x, accel_y, accel_z; // Linear acceleration (m/s^2)
    float accel_raw_x, accel_raw_y, accel_raw_z; // Raw acceleration (includes gravity)
    uint32_t timestamp_ms;           // Sample timestamp
    volatile bool valid;             // Data validity flag
    volatile bool updated;           // New data available flag
};

// Global sensor caches (accessed by sensor task and timer callback)
volatile ImuSampleCache bno_cache = {
    1.0f, 0.0f, 0.0f, 0.0f,                // Quaternion
    0.0f, 0.0f, 0.0f,                      // Gyro
    0.0f, 0.0f, 0.0f,                      // Linear accel
    0.0f, 0.0f, 0.0f,                      // Raw accel
    0, false, false};
#if DUAL_IMU_MODE
volatile ImuSampleCache mpu_cache = {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0, false, false};
#endif

// FreeRTOS task handle
TaskHandle_t sensorTaskHandle = NULL;

// Mutex guarding shared I2C bus transactions (BNO055 + MPU6050)
static SemaphoreHandle_t bno_i2c_mutex = nullptr;

inline void lock_bno_i2c() {
  if (bno_i2c_mutex != nullptr) {
    xSemaphoreTake(bno_i2c_mutex, portMAX_DELAY);
  }
}

inline void unlock_bno_i2c() {
  if (bno_i2c_mutex != nullptr) {
    xSemaphoreGive(bno_i2c_mutex);
  }
}

// Error counter for micro-ROS publish failures (non-blocking)
volatile uint32_t micro_ros_error_count = 0;

// Timestamp for variable dt calculation
volatile uint32_t last_sensor_poll_us = 0;

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

// BNO055 sensor object (using address from imu_config.h)
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_I2C_ADDR);

/**
 * Read BNO055 quaternion with validation and retry logic
 * @param bno BNO055 sensor object
 * @param quat Output quaternion (Adafruit format)
 * @param max_retries Maximum number of retry attempts
 * @return true if valid quaternion read, false otherwise
 */
bool read_bno055_quaternion_safe(Adafruit_BNO055& bno, imu::Quaternion& quat, int max_retries) {
    for (int attempt = 0; attempt < max_retries; attempt++) {
        lock_bno_i2c();
        quat = bno.getQuat();
        unlock_bno_i2c();
        
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

// Grace period after initialization - allow sensor to stabilize
static uint32_t bno055_init_time_ms = 0;

// Verbose debug flag - set to 1 to enable status logging (blocking)
#ifndef VERBOSE_STATUS_DEBUG
#define VERBOSE_STATUS_DEBUG 0
#endif

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
#if VERBOSE_STATUS_DEBUG
    static uint32_t last_status_log_ms = 0;
    static bool first_log = true;
#endif
    
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
    lock_bno_i2c();
    bno.getSystemStatus(&system_status, &self_test_result, &system_error);
    unlock_bno_i2c();
    
    // Retry once if we get suspicious values (might be I2C glitch)
    if (system_error == 0xFF || system_error > 0x0A) {
        delay(1);  // Brief delay before retry
        lock_bno_i2c();
        bno.getSystemStatus(&system_status, &self_test_result, &system_error);
        unlock_bno_i2c();
    }
    
    // Check for errors
    bool error_detected = (system_error != 0 && system_error != 0xFF);
    
    if (error_detected) {
        consecutive_errors++;
    } else {
        consecutive_errors = 0;
    }
    
    // Log status periodically or on error (only if VERBOSE_STATUS_DEBUG enabled)
#if VERBOSE_STATUS_DEBUG
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
#endif
    
    // Only fail on persistent errors (3+ consecutive checks with errors)
    if (error_detected && consecutive_errors >= 3) {
#if VERBOSE_STATUS_DEBUG
        Serial.printf("[STATUS ERROR] BNO055 system_error=%d (persistent, %d consecutive)\n", 
                      system_error, consecutive_errors);
#endif
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

// Error macro - RCCHECK halts on error, RCSOFTCHECK increments counter (non-blocking)
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ micro_ros_error_count++; }}

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
  
  if (!mpu.begin(MPU6050_I2C_ADDR)) {
    Serial.println("WARNING: MPU6050 not found at configured address");
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

// Static frame_id arrays to avoid dynamic allocation
static char frame_id_imu_link[] = "imu_link";
static char frame_id_imu1_link[] = "imu1_link";
static char frame_id_imu2_link[] = "imu2_link";
static char frame_id_imu_fused_link[] = "imu_fused_link";

/**
 * Fill IMU message header with cached timestamp
 * @param msg IMU message to fill
 * @param frame_id Static frame_id string
 * @param epoch_millis Cached timestamp from rmw_uros_epoch_millis()
 */
void fill_imu_msg_header_cached(sensor_msgs__msg__Imu& msg, char* frame_id, uint64_t epoch_millis) {
  msg.header.stamp.sec = epoch_millis / 1000;
  msg.header.stamp.nanosec = (epoch_millis % 1000) * 1000000;
  msg.header.frame_id.data = frame_id;
  msg.header.frame_id.size = strlen(frame_id);
  msg.header.frame_id.capacity = strlen(frame_id) + 1;
}

// ============================================================================
// FreeRTOS Sensor Polling Task
// ============================================================================

// Spinlock for cache access (must be declared before use)
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

/**
 * High-frequency sensor polling task (runs on Core 1)
 * Decouples I2C reads from timer callback for deterministic timing.
 */
void sensor_polling_task(void* param) {
    (void)param;
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    while (true) {
        uint32_t now_us = micros();
        uint32_t now_ms = millis();
        
        // Calculate dt for variable time-step filter
        float dt = 0.002f;  // Default 2ms (500Hz)
        if (last_sensor_poll_us > 0) {
            dt = (now_us - last_sensor_poll_us) / 1000000.0f;
            // Clamp dt to safe bounds
            if (dt < 0.0001f) dt = 0.0001f;
            if (dt > 0.05f) dt = 0.05f;
        }
        last_sensor_poll_us = now_us;
        
        // --- Poll BNO055 ---
        if (sensor_initialized) {
            lock_bno_i2c();
            imu::Quaternion q = bno.getQuat();
            unlock_bno_i2c();
            
            // Validate quaternion
            bool valid = true;
            if (q.w() == 0.0f && q.x() == 0.0f && q.y() == 0.0f && q.z() == 0.0f) {
                valid = false;
            }
            if (!std::isfinite(q.w()) || !std::isfinite(q.x()) ||
                !std::isfinite(q.y()) || !std::isfinite(q.z())) {
                valid = false;
            }
            float norm = sqrtf(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
            if (norm < 0.1f || norm > 2.0f) {
                valid = false;
            }
            
            if (valid) {
                // Get gyro and accel
                lock_bno_i2c();
                imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
                imu::Vector<3> accel_linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
                imu::Vector<3> accel_raw = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
                unlock_bno_i2c();
                
                // Update cache atomically (disable interrupts briefly)
                portENTER_CRITICAL(&spinlock);
                bno_cache.qw = q.w();
                bno_cache.qx = q.x();
                bno_cache.qy = q.y();
                bno_cache.qz = q.z();
                bno_cache.gyro_x = gyro.x();
                bno_cache.gyro_y = gyro.y();
                bno_cache.gyro_z = gyro.z();
                bno_cache.accel_x = accel_linear.x();
                bno_cache.accel_y = accel_linear.y();
                bno_cache.accel_z = accel_linear.z();
                bno_cache.accel_raw_x = accel_raw.x();
                bno_cache.accel_raw_y = accel_raw.y();
                bno_cache.accel_raw_z = accel_raw.z();
                bno_cache.timestamp_ms = now_ms;
                bno_cache.valid = true;
                bno_cache.updated = true;
                portEXIT_CRITICAL(&spinlock);
            }
        }
        
#if DUAL_IMU_MODE
        // --- Poll MPU6050 ---
        if (mpu_initialized) {
            sensors_event_t accel_event, gyro_event, temp_event;
            lock_bno_i2c();
            mpu.getEvent(&accel_event, &gyro_event, &temp_event);
            unlock_bno_i2c();
            
            float gx = gyro_event.gyro.x;
            float gy = gyro_event.gyro.y;
            float gz = gyro_event.gyro.z;
            float ax = accel_event.acceleration.x;
            float ay = accel_event.acceleration.y;
            float az = accel_event.acceleration.z;
            
            // Update Madgwick filter with variable dt
            madgwick_filter.updateIMU(gx, gy, gz, ax, ay, az, dt);
            
            float qw, qx, qy, qz;
            madgwick_filter.getQuaternion(&qw, &qx, &qy, &qz);
            
            // Update cache atomically
            portENTER_CRITICAL(&spinlock);
            mpu_cache.qw = qw;
            mpu_cache.qx = qx;
            mpu_cache.qy = qy;
            mpu_cache.qz = qz;
            mpu_cache.gyro_x = gx;
            mpu_cache.gyro_y = gy;
            mpu_cache.gyro_z = gz;
            mpu_cache.accel_x = ax;
            mpu_cache.accel_y = ay;
            mpu_cache.accel_z = az;
            mpu_cache.accel_raw_x = ax;
            mpu_cache.accel_raw_y = ay;
            mpu_cache.accel_raw_z = az;
            mpu_cache.timestamp_ms = now_ms;
            mpu_cache.valid = true;
            mpu_cache.updated = true;
            portEXIT_CRITICAL(&spinlock);
        }
#endif
        
        // Sleep until next poll (target 500Hz = 2ms period)
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SENSOR_POLL_PERIOD_MS));
    }
}


// Timer callback - publishes IMU data from cache (non-blocking)
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
  
  // Cache timestamp once for all messages in this cycle (Task 5)
  uint64_t epoch_millis = rmw_uros_epoch_millis();
  uint32_t now_ms = millis();

#if DUAL_IMU_MODE
  // ===== DUAL IMU MODE (Cache-based, non-blocking) =====
  
  // --- IMU1: BNO055 (from cache) ---
  float bno_qw, bno_qx, bno_qy, bno_qz;
  float bno_gx, bno_gy, bno_gz;
  float bno_ax, bno_ay, bno_az;
  uint32_t bno_ts;
  bool bno_valid;
  
  // Read from cache atomically
  portENTER_CRITICAL(&spinlock);
  bno_qw = bno_cache.qw;
  bno_qx = bno_cache.qx;
  bno_qy = bno_cache.qy;
  bno_qz = bno_cache.qz;
  bno_gx = bno_cache.gyro_x;
  bno_gy = bno_cache.gyro_y;
  bno_gz = bno_cache.gyro_z;
  bno_ax = bno_cache.accel_x;
  bno_ay = bno_cache.accel_y;
  bno_az = bno_cache.accel_z;
  bno_ts = bno_cache.timestamp_ms;
  bno_valid = bno_cache.valid;
  bno_cache.updated = false;
  portEXIT_CRITICAL(&spinlock);
  
  // Check cache staleness
  if (!bno_valid || (now_ms - bno_ts > CACHE_STALE_THRESHOLD_MS)) {
    // Cache is stale or invalid - skip this sample
    return;
  }
  
  Quaternion q1_raw(bno_qw, bno_qx, bno_qy, bno_qz);
  
  // Hemisphere alignment: align to previous raw quaternion (prevents sign flips)
  if (prev_bno_raw_initialized_dual) {
    alignHemisphere(q1_raw, prev_bno_raw_quat_dual);
  } else {
    prev_bno_raw_initialized_dual = true;
  }
  prev_bno_raw_quat_dual = q1_raw;
  
  Quaternion q1 = q1_raw;
  Vec3 gyro1(bno_gx, bno_gy, bno_gz);
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
  
  
  // Fill IMU1 message (using cached timestamp and static frame_id)
  fill_imu_msg_header_cached(imu1_msg, frame_id_imu1_link, epoch_millis);
  imu1_msg.orientation.x = q1.x;
  imu1_msg.orientation.y = q1.y;
  imu1_msg.orientation.z = q1.z;
  imu1_msg.orientation.w = q1.w;
  imu1_msg.angular_velocity.x = bno_gx;
  imu1_msg.angular_velocity.y = bno_gy;
  imu1_msg.angular_velocity.z = bno_gz;
  imu1_msg.linear_acceleration.x = bno_ax;
  imu1_msg.linear_acceleration.y = bno_ay;
  imu1_msg.linear_acceleration.z = bno_az;
  
  // Publish IMU1
  if (q1_valid) {
    RCSOFTCHECK(rcl_publish(&publisher_imu1, &imu1_msg, NULL));
  }
  
  // --- IMU2: MPU6050 (from cache) ---
  Quaternion q2;
  bool q2_valid = false;
  
  if (mpu_initialized) {
    float mpu_qw, mpu_qx, mpu_qy, mpu_qz;
    float mpu_gx, mpu_gy, mpu_gz;
    float mpu_ax, mpu_ay, mpu_az;
    uint32_t mpu_ts;
    bool mpu_valid;
    
    // Read from cache atomically
    portENTER_CRITICAL(&spinlock);
    mpu_qw = mpu_cache.qw;
    mpu_qx = mpu_cache.qx;
    mpu_qy = mpu_cache.qy;
    mpu_qz = mpu_cache.qz;
    mpu_gx = mpu_cache.gyro_x;
    mpu_gy = mpu_cache.gyro_y;
    mpu_gz = mpu_cache.gyro_z;
    mpu_ax = mpu_cache.accel_x;
    mpu_ay = mpu_cache.accel_y;
    mpu_az = mpu_cache.accel_z;
    mpu_ts = mpu_cache.timestamp_ms;
    mpu_valid = mpu_cache.valid;
    mpu_cache.updated = false;
    portEXIT_CRITICAL(&spinlock);
    
    // Check cache staleness
    if (mpu_valid && (now_ms - mpu_ts <= CACHE_STALE_THRESHOLD_MS)) {
      Quaternion q2_raw(mpu_qw, mpu_qx, mpu_qy, mpu_qz);
      
      // Hemisphere alignment: align to previous raw quaternion (prevents sign flips)
      if (prev_mpu_raw_initialized) {
        alignHemisphere(q2_raw, prev_mpu_raw_quat);
      } else {
        prev_mpu_raw_initialized = true;
      }
      prev_mpu_raw_quat = q2_raw;
      
      q2 = q2_raw;
      Vec3 gyro2(mpu_gx, mpu_gy, mpu_gz);
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
      
      
      // Fill IMU2 message (using cached timestamp and static frame_id)
      fill_imu_msg_header_cached(imu2_msg, frame_id_imu2_link, epoch_millis);
      imu2_msg.orientation.x = q2.x;
      imu2_msg.orientation.y = q2.y;
      imu2_msg.orientation.z = q2.z;
      imu2_msg.orientation.w = q2.w;
      imu2_msg.angular_velocity.x = mpu_gx;
      imu2_msg.angular_velocity.y = mpu_gy;
      imu2_msg.angular_velocity.z = mpu_gz;
      imu2_msg.linear_acceleration.x = mpu_ax;
      imu2_msg.linear_acceleration.y = mpu_ay;
      imu2_msg.linear_acceleration.z = mpu_az;
      
      // Publish IMU2
      if (q2_valid) {
        RCSOFTCHECK(rcl_publish(&publisher_imu2, &imu2_msg, NULL));
      }
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
    Vec3 gyro_fused(bno_gx, bno_gy, bno_gz);
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
    
    // Fill fused message (using cached timestamp and static frame_id)
    fill_imu_msg_header_cached(fused_msg, frame_id_imu_fused_link, epoch_millis);
    fused_msg.orientation.x = q_fused.x;
    fused_msg.orientation.y = q_fused.y;
    fused_msg.orientation.z = q_fused.z;
    fused_msg.orientation.w = q_fused.w;
    
    // Use BNO055 angular velocity and acceleration for fused message (from cache)
    fused_msg.angular_velocity.x = bno_gx;
    fused_msg.angular_velocity.y = bno_gy;
    fused_msg.angular_velocity.z = bno_gz;
    fused_msg.linear_acceleration.x = bno_ax;
    fused_msg.linear_acceleration.y = bno_ay;
    fused_msg.linear_acceleration.z = bno_az;
    
    // Publish fused
    RCSOFTCHECK(rcl_publish(&publisher_fused, &fused_msg, NULL));
  }
  
#else
  // ===== SINGLE IMU MODE (Cache-based, non-blocking) =====
  
  // Read from cache atomically
  float bno_qw, bno_qx, bno_qy, bno_qz;
  float bno_gx, bno_gy, bno_gz;
  float bno_ax, bno_ay, bno_az;
#if POSITION_MODE
  float bno_raw_ax, bno_raw_ay, bno_raw_az;
#endif
  uint32_t bno_ts;
  bool bno_valid;
  
  portENTER_CRITICAL(&spinlock);
  bno_qw = bno_cache.qw;
  bno_qx = bno_cache.qx;
  bno_qy = bno_cache.qy;
  bno_qz = bno_cache.qz;
  bno_gx = bno_cache.gyro_x;
  bno_gy = bno_cache.gyro_y;
  bno_gz = bno_cache.gyro_z;
  bno_ax = bno_cache.accel_x;
  bno_ay = bno_cache.accel_y;
  bno_az = bno_cache.accel_z;
#if POSITION_MODE
  bno_raw_ax = bno_cache.accel_raw_x;
  bno_raw_ay = bno_cache.accel_raw_y;
  bno_raw_az = bno_cache.accel_raw_z;
#endif
  bno_ts = bno_cache.timestamp_ms;
  bno_valid = bno_cache.valid;
  bno_cache.updated = false;
  portEXIT_CRITICAL(&spinlock);
  
  // Check cache validity and staleness
  if (!bno_valid || (now_ms - bno_ts > CACHE_STALE_THRESHOLD_MS)) {
#if ENABLE_DIAGNOSTIC_LOGGING
    static uint32_t last_fail_log_ms = 0;
    if (now_ms - last_fail_log_ms > 1000) {  // Log failure every 1 second max
      Serial.printf("[TIMER] BNO055 cache stale or invalid\n");
      last_fail_log_ms = now_ms;
    }
#endif
    return;
  }
  
  // Raw quaternion before processing
  Quaternion q_raw(bno_qw, bno_qx, bno_qy, bno_qz);
  
  // Hemisphere alignment: align to previous raw quaternion (prevents sign flips)
  if (prev_bno_raw_initialized) {
    alignHemisphere(q_raw, prev_bno_raw_quat);
  } else {
    prev_bno_raw_initialized = true;
  }
  prev_bno_raw_quat = q_raw;
  
  // Process quaternion for sign continuity and jump smoothing (prevents teleportation)
  Quaternion q = q_raw;
  Vec3 gyro_single(bno_gx, bno_gy, bno_gz);
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
  
  // Fill the IMU message (using cached timestamp and static frame_id)
  fill_imu_msg_header_cached(imu_msg, frame_id_imu_link, epoch_millis);
  
  // Orientation (quaternion) - processed with sign continuity and jump smoothing
  imu_msg.orientation.x = q.x;
  imu_msg.orientation.y = q.y;
  imu_msg.orientation.z = q.z;
  imu_msg.orientation.w = q.w;
  
  // Angular velocity (rad/s) - from cache
  imu_msg.angular_velocity.x = bno_gx;
  imu_msg.angular_velocity.y = bno_gy;
  imu_msg.angular_velocity.z = bno_gz;
  
  // Linear acceleration (m/s^2) - from cache
  imu_msg.linear_acceleration.x = bno_ax;
  imu_msg.linear_acceleration.y = bno_ay;
  imu_msg.linear_acceleration.z = bno_az;
  
  // Publish the IMU message
  RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));

#if POSITION_MODE
    // For position mode, create imu::Quaternion and vectors from cache
    imu::Quaternion bno_quat_pos(bno_qw, bno_qx, bno_qy, bno_qz);
    imu::Vector<3> accel_raw_pos(bno_raw_ax, bno_raw_ay, bno_raw_az);
    imu::Vector<3> gyro_pos(bno_gx, bno_gy, bno_gz);
    update_position_estimate(bno_quat_pos, accel_raw_pos, gyro_pos, now_ms);

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
      odom_msg.twist.twist.angular.x = bno_gx;
      odom_msg.twist.twist.angular.y = bno_gy;
      odom_msg.twist.twist.angular.z = bno_gz;

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
  
  // Create mutex guarding BNO055 I2C transactions
  bno_i2c_mutex = xSemaphoreCreateMutex();
  if (bno_i2c_mutex == nullptr) {
    Serial.println("ERROR: Failed to create BNO055 I2C mutex!");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }
  
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
  madgwick_filter.begin(FILTER_UPDATE_RATE_HZ, MADGWICK_DEFAULT_BETA);
  Serial.printf("Fast Madgwick filter initialized (rate=%.0f Hz, beta=%.2f)\n", 
                FILTER_UPDATE_RATE_HZ, MADGWICK_DEFAULT_BETA);
  
  if (!mpu_initialized) {
    Serial.println("WARNING: Running in degraded mode (BNO055 only)");
  }
#endif

  // Create FreeRTOS sensor polling task on Core 1 (WiFi runs on Core 0)
  xTaskCreatePinnedToCore(
    sensor_polling_task,        // Task function
    "SensorTask",               // Task name
    SENSOR_TASK_STACK_SIZE,     // Stack size
    NULL,                       // Task parameters
    SENSOR_TASK_PRIORITY,       // Priority
    &sensorTaskHandle,          // Task handle
    SENSOR_TASK_CORE            // Core to run on
  );
  Serial.println("[INIT] FreeRTOS sensor polling task started on Core 1");
  
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
  lock_bno_i2c();
  bno.getSensor(&sensor);
  unlock_bno_i2c();
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
  // MAX_AGENT_ATTEMPTS is defined in imu_config.h
  int agent_attempts = 0;
  bool agent_found = false;
  
  while (agent_attempts < MAX_AGENT_ATTEMPTS) {
    agent_attempts++;
    Serial.printf("  Ping attempt %d/%d... ", agent_attempts, MAX_AGENT_ATTEMPTS);
    
    // rmw_uros_ping_agent returns RMW_RET_OK if agent is reachable
    // AGENT_PING_TIMEOUT_MS and AGENT_PING_ATTEMPTS are defined in imu_config.h
    if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS) == RMW_RET_OK) {
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
  
  // Create publishers with Best-Effort QoS for low latency (Task 6)
#if DUAL_IMU_MODE
  // IMU1 publisher (BNO055) - Best Effort QoS
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_imu1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu1_data"));
  
  // IMU2 publisher (MPU6050) - Best Effort QoS
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_imu2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu2_data"));
  
  // Fused publisher - Best Effort QoS
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_fused,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_fused"));
  
  Serial.println("[QoS] Publishers configured with Best-Effort QoS for low latency");
  
  // Initialize messages
  memset(&imu1_msg, 0, sizeof(imu1_msg));
  memset(&imu2_msg, 0, sizeof(imu2_msg));
  memset(&fused_msg, 0, sizeof(fused_msg));
#else
  // Single IMU publisher - Best Effort QoS
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_data"));
  Serial.println("[QoS] Publisher configured with Best-Effort QoS for low latency");
#endif

#if POSITION_MODE
  RCCHECK(rclc_publisher_init_best_effort(
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
  
  // Create timer for publishing at 100 Hz
  // With FreeRTOS sensor task, I2C reads are decoupled - timer just reads from cache
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(TIMER_PERIOD_MS),
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
    // Execute callbacks with zero timeout for non-blocking polling (Task 10)
    RCSOFTCHECK(rclc_executor_spin_some(&executor, 0));
  }
  
  // Minimal delay to yield to other tasks (WiFi stack, sensor task, etc.)
  delay(1);
}

#endif  // !MPU6050_ONLY_MODE
