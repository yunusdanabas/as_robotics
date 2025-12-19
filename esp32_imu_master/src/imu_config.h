/*
 * IMU Configuration Header
 * 
 * Centralizes hardware constants and configuration parameters
 * for the ESP32 IMU system.
 */

#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

// ============================================================================
// I2C Addresses
// ============================================================================
#define BNO055_I2C_ADDR     0x28
#define MPU6050_I2C_ADDR    0x68
#define BMP280_I2C_ADDR     0x76

// ============================================================================
// I2C Configuration
// ============================================================================
#define I2C_CLOCK_HZ        400000  // 400kHz fast mode

// ============================================================================
// Timer Configuration
// ============================================================================
#define TIMER_PERIOD_MS     10      // 100Hz target publish rate
#define SENSOR_POLL_PERIOD_MS 2     // 500Hz sensor polling rate

// ============================================================================
// Sensor Cache Configuration
// ============================================================================
#define CACHE_STALE_THRESHOLD_MS 30 // Skip publish if cache older than this

// ============================================================================
// FreeRTOS Task Configuration
// ============================================================================
#define SENSOR_TASK_STACK_SIZE  4096
#define SENSOR_TASK_PRIORITY    2       // Higher than loop() but lower than WiFi
#define SENSOR_TASK_CORE        1       // Core 1 (WiFi runs on Core 0)

// ============================================================================
// Status Check Configuration
// ============================================================================
#define STATUS_CHECK_INTERVAL_MS    10000   // Check BNO055 status every 10s
#define BNO055_GRACE_PERIOD_MS      3000    // Grace period after init

// ============================================================================
// Micro-ROS Agent Configuration
// ============================================================================
#define AGENT_PING_TIMEOUT_MS   100
#define AGENT_PING_ATTEMPTS     1
#define MAX_AGENT_ATTEMPTS      30      // 30 * 500ms = 15 seconds max wait

// ============================================================================
// Filter Parameters
// ============================================================================
#define MADGWICK_DEFAULT_BETA   0.8f    // High beta for responsive orientation
#define FILTER_UPDATE_RATE_HZ   50.0f   // Default filter update rate

#endif // IMU_CONFIG_H

