/*
 * ESP32 IMU Test - MPU6050 + Magnetometer + BMP180 (10DOF)
 * 
 * This firmware tests the secondary IMU setup alone:
 * - MPU6050: Accelerometer + Gyroscope
 * - HMC5883L or QMC5883L: Magnetometer (auto-detect)
 * - BMP180: Barometric pressure (optional)
 * 
 * Uses custom Madgwick filter with adjustable beta for responsive orientation.
 * Publishes to /imu_data topic at 50 Hz.
 * 
 * Build: pio run -e esp32dev_mpu6050
 * Upload: pio run -e esp32dev_mpu6050 --target upload
 */

#if MPU6050_ONLY_MODE

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

#include <cmath>
#include <cstring>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>

// Sensor libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <QMC5883LCompass.h>
#include <Adafruit_BMP085.h>

// WiFi configuration
#include "wifi_config.h"

// Sensor fusion utilities (includes FastMadgwick class)
#include "sensor_fusion.h"

// micro-ROS objects
rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Sensor objects
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified hmc = Adafruit_HMC5883_Unified(12345);
QMC5883LCompass qmc;
Adafruit_BMP085 bmp;
FastMadgwick madgwick_filter;  // Custom filter with adjustable beta

// Sensor status
bool mpu_initialized = false;
bool mag_initialized = false;
bool bmp_initialized = false;
bool use_hmc = false;  // true = HMC5883L, false = QMC5883L

// Status variables
bool sensor_initialized = false;
bool micro_ros_initialized = false;
bool wifi_connected = false;

// LED pin
const int LED_PIN = 2;

// Filter settings
static constexpr float PUBLISH_RATE = 50.0f;      // Hz - publishing rate
static constexpr float MADGWICK_BETA = 0.8f;      // Higher = more responsive (0.1 default is too slow)

// Magnetometer calibration offsets (adjust for your environment)
static float mag_offset_x = 0.0f;
static float mag_offset_y = 0.0f;
static float mag_offset_z = 0.0f;

// Quaternion sign continuity
static float last_qw = 1.0f, last_qx = 0.0f, last_qy = 0.0f, last_qz = 0.0f;
static bool quat_initialized = false;
static int warmup_count = 10;  // Reduced warmup for faster startup

// Error macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

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

bool init_mpu6050() {
  Serial.println("\nInitializing MPU6050...");
  
  if (!mpu.begin(0x68)) {
    Serial.println("ERROR: MPU6050 not found at 0x68!");
    Serial.println("Check wiring:");
    Serial.println("  MPU6050 SDA -> ESP32 GPIO21");
    Serial.println("  MPU6050 SCL -> ESP32 GPIO22");
    Serial.println("  MPU6050 VCC -> 3.3V");
    Serial.println("  MPU6050 GND -> GND");
    return false;
  }
  
  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // Higher bandwidth = faster response (less internal filtering lag)
  // 21Hz is too slow, 94Hz gives good balance of speed and noise
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  
  Serial.println("MPU6050 initialized successfully!");
  Serial.println("  Accelerometer range: +/- 4G");
  Serial.println("  Gyroscope range: +/- 500 deg/s");
  Serial.println("  Filter bandwidth: 21 Hz");
  
  return true;
}

bool i2c_device_exists(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

bool init_magnetometer() {
  Serial.println("\nDetecting magnetometer...");
  
  // First check if HMC5883L actually exists on bus (address 0x1E)
  if (i2c_device_exists(0x1E)) {
    if (hmc.begin()) {
      use_hmc = true;
      Serial.println("HMC5883L detected at address 0x1E");
      return true;
    }
  }
  
  Serial.println("HMC5883L not found, trying QMC5883L...");
  
  // Check if QMC5883L exists on bus (address 0x0D)
  if (i2c_device_exists(0x0D)) {
    qmc.init();
    qmc.setMode(0x01, 0x0C, 0x10, 0x00);  // Continuous, 200Hz, 8G, 512 oversampling
    delay(100);
    
    qmc.read();
    int x = qmc.getX();
    int y = qmc.getY();
    int z = qmc.getZ();
    
    // Check if we get any readings
    if (x != 0 || y != 0 || z != 0) {
      use_hmc = false;
      Serial.println("QMC5883L detected at address 0x0D");
      return true;
    }
  }
  
  Serial.println("WARNING: No magnetometer detected!");
  Serial.println("Orientation will use IMU-only mode (no heading correction)");
  return false;
}

bool init_bmp180() {
  Serial.println("\nInitializing BMP180...");
  
  if (!bmp.begin()) {
    Serial.println("WARNING: BMP180 not found at 0x77");
    Serial.println("Altitude data will not be available");
    return false;
  }
  
  Serial.println("BMP180 initialized successfully!");
  Serial.print("  Current pressure: ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
  Serial.print("  Current temperature: ");
  Serial.print(bmp.readTemperature());
  Serial.println(" C");
  
  return true;
}

void read_magnetometer(float& mx, float& my, float& mz) {
  if (!mag_initialized) {
    mx = my = mz = 0.0f;
    return;
  }
  
  if (use_hmc) {
    sensors_event_t event;
    hmc.getEvent(&event);
    mx = event.magnetic.x - mag_offset_x;
    my = event.magnetic.y - mag_offset_y;
    mz = event.magnetic.z - mag_offset_z;
  } else {
    qmc.read();
    mx = (float)qmc.getX() - mag_offset_x;
    my = (float)qmc.getY() - mag_offset_y;
    mz = (float)qmc.getZ() - mag_offset_z;
  }
}

void apply_sign_continuity(float& qw, float& qx, float& qy, float& qz) {
  if (quat_initialized) {
    float dot = qw * last_qw + qx * last_qx + qy * last_qy + qz * last_qz;
    if (dot < 0.0f) {
      qw = -qw;
      qx = -qx;
      qy = -qy;
      qz = -qz;
    }
  }
  
  last_qw = qw;
  last_qx = qx;
  last_qy = qy;
  last_qz = qz;
  quat_initialized = true;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  
  if (timer == NULL || !sensor_initialized) {
    return;
  }
  
  // Read MPU6050
  sensors_event_t accel_event, gyro_event, temp_event;
  mpu.getEvent(&accel_event, &gyro_event, &temp_event);
  
  float ax = accel_event.acceleration.x;
  float ay = accel_event.acceleration.y;
  float az = accel_event.acceleration.z;
  
  float gx = gyro_event.gyro.x;  // rad/s
  float gy = gyro_event.gyro.y;
  float gz = gyro_event.gyro.z;
  
  // Read magnetometer
  float mx, my, mz;
  read_magnetometer(mx, my, mz);
  
  // Update Madgwick filter (IMU only - no magnetometer for now)
  madgwick_filter.updateIMU(gx, gy, gz, ax, ay, az);
  
  // Get quaternion from filter
  float qw, qx, qy, qz;
  madgwick_filter.getQuaternion(&qw, &qx, &qy, &qz);
  
  // Apply sign continuity
  apply_sign_continuity(qw, qx, qy, qz);
  
  // Handle warmup period
  if (warmup_count > 0) {
    warmup_count--;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    return;
  }
  
  // Fill IMU message
  uint64_t epoch_millis = rmw_uros_epoch_millis();
  imu_msg.header.stamp.sec = epoch_millis / 1000;
  imu_msg.header.stamp.nanosec = (epoch_millis % 1000) * 1000000;
  imu_msg.header.frame_id.data = (char*)"imu_link";
  imu_msg.header.frame_id.size = strlen("imu_link");
  
  // Orientation (quaternion from Madgwick filter)
  imu_msg.orientation.x = qx;
  imu_msg.orientation.y = qy;
  imu_msg.orientation.z = qz;
  imu_msg.orientation.w = qw;
  
  // Angular velocity (rad/s)
  imu_msg.angular_velocity.x = gx;
  imu_msg.angular_velocity.y = gy;
  imu_msg.angular_velocity.z = gz;
  
  // Linear acceleration (m/s^2)
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;
  
  // Publish
  RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
  
  // Blink LED
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("========================================");
  Serial.println("  ESP32 IMU Test - MPU6050 + Magnetometer");
  Serial.println("  (10DOF Sensor Board Test Mode)");
  Serial.println("========================================");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Connect to WiFi
  if (!connect_wifi()) {
    Serial.println("Failed to connect to WiFi. Rebooting...");
    delay(5000);
    ESP.restart();
  }
  wifi_connected = true;
  
  // Initialize I2C
  Wire.begin();
  delay(100);
  
  // Scan I2C bus
  Serial.println("\n--- I2C Bus Scan ---");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Device found at 0x");
      Serial.print(addr, HEX);
      
      // Identify known devices
      if (addr == 0x68) Serial.print(" (MPU6050)");
      else if (addr == 0x1E) Serial.print(" (HMC5883L)");
      else if (addr == 0x0D) Serial.print(" (QMC5883L)");
      else if (addr == 0x77) Serial.print(" (BMP180/BMP085)");
      else if (addr == 0x76) Serial.print(" (BMP280)");
      else if (addr == 0x28) Serial.print(" (BNO055)");
      
      Serial.println();
    }
  }
  Serial.println("--------------------");
  
  // Initialize sensors
  mpu_initialized = init_mpu6050();
  
  if (!mpu_initialized) {
    Serial.println("\nERROR: MPU6050 is required! Check wiring and restart.");
    error_loop();
  }
  
  mag_initialized = init_magnetometer();
  bmp_initialized = init_bmp180();
  
  // Initialize custom Madgwick filter with high beta for BNO055-like responsiveness
  madgwick_filter.begin(PUBLISH_RATE, MADGWICK_BETA);
  Serial.printf("\nFast Madgwick filter initialized (rate=%.0f Hz, beta=%.2f)\n", 
                PUBLISH_RATE, MADGWICK_BETA);
  Serial.println("  (beta 0.8 = very responsive, like BNO055)");
  
  sensor_initialized = true;
  
  // Print sensor summary
  Serial.println("\n--- Sensor Status ---");
  Serial.print("  MPU6050:      ");
  Serial.println(mpu_initialized ? "OK" : "FAILED");
  Serial.print("  Magnetometer: ");
  if (mag_initialized) {
    Serial.println(use_hmc ? "HMC5883L" : "QMC5883L");
  } else {
    Serial.println("NOT FOUND");
  }
  Serial.print("  BMP180:       ");
  Serial.println(bmp_initialized ? "OK" : "NOT FOUND");
  Serial.println("---------------------");
  
  // Setup micro-ROS
  IPAddress agent_ip;
  agent_ip.fromString(AGENT_IP);
  
  Serial.println("\nSetting up micro-ROS WiFi transport...");
  Serial.print("  Agent IP: ");
  Serial.println(AGENT_IP);
  Serial.print("  Agent Port: ");
  Serial.println(AGENT_PORT);
  
  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASSWORD, agent_ip, AGENT_PORT);
  delay(2000);
  
  Serial.println("Initializing micro-ROS...");
  allocator = rcl_get_default_allocator();
  
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_mpu6050_test", "", &support));
  
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_data"));
  
  memset(&imu_msg, 0, sizeof(imu_msg));
  
  // Create timer (50 Hz for smoother response - closer to BNO055 feel)
  const unsigned int timer_timeout = 20;  // 20ms = 50Hz
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  micro_ros_initialized = true;
  
  Serial.println("\n========================================");
  Serial.println("  micro-ROS initialized successfully!");
  Serial.println("  Publishing to: /imu_data");
  Serial.println("  Rate: 50 Hz (responsive mode)");
  Serial.println("  Ready for testing!");
  Serial.println("========================================\n");
  
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
}

void loop() {
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
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  }
  
  delay(10);
}

#endif  // MPU6050_ONLY_MODE
