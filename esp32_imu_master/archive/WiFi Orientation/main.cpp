/*
 * ESP32 IMU Master Device - BNO055 with micro-ROS (WiFi Transport)
 * 
 * This is the WiFi version of the firmware. To use this:
 * 1. Copy wifi_config.h to wifi_credentials.h
 * 2. Update WiFi credentials in wifi_credentials.h
 * 3. Rename main.cpp to main_serial.cpp
 * 4. Rename this file to main.cpp
 * 5. Build and upload
 * 
 * Hardware:
 * - WEMOS D1 R32 (ESP32)
 * - DFRobot Gravity BNO055 + BMP280 (SEN0253)
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>

// WiFi configuration (create wifi_credentials.h from wifi_config.h)
#include "wifi_config.h"

// micro-ROS objects
rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// BNO055 sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Status variables
bool sensor_initialized = false;
bool micro_ros_initialized = false;
bool wifi_connected = false;

// LED pin for status indication
const int LED_PIN = 2;

// Error macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

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

// Timer callback - publishes IMU data
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  
  if (timer != NULL && sensor_initialized) {
    // Get quaternion data from BNO055
    imu::Quaternion quat = bno.getQuat();
    
    // Get angular velocity (gyroscope)
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    
    // Get linear acceleration
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    
    // Fill the IMU message
    imu_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
    imu_msg.header.stamp.nanosec = (rmw_uros_epoch_millis() % 1000) * 1000000;
    imu_msg.header.frame_id.data = (char*)"imu_link";
    imu_msg.header.frame_id.size = strlen("imu_link");
    
    // Orientation (quaternion)
    imu_msg.orientation.x = quat.x();
    imu_msg.orientation.y = quat.y();
    imu_msg.orientation.z = quat.z();
    imu_msg.orientation.w = quat.w();
    
    // Angular velocity (rad/s)
    imu_msg.angular_velocity.x = gyro.x();
    imu_msg.angular_velocity.y = gyro.y();
    imu_msg.angular_velocity.z = gyro.z();
    
    // Linear acceleration (m/s^2)
    imu_msg.linear_acceleration.x = accel.x();
    imu_msg.linear_acceleration.y = accel.y();
    imu_msg.linear_acceleration.z = accel.z();
    
    // Publish the message
    RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
    
    // Blink LED to show activity
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("ESP32 IMU Master - WiFi Mode");
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
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize BNO055 sensor
  Serial.println("\nInitializing BNO055 sensor...");
  if (!bno.begin()) {
    Serial.println("ERROR: No BNO055 detected. Check wiring!");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
  sensor_initialized = true;
  Serial.println("BNO055 initialized successfully!");
  
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
  
  delay(2000);
  
  // Initialize micro-ROS
  Serial.println("Initializing micro-ROS...");
  allocator = rcl_get_default_allocator();
  
  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32_imu_master_wifi", "", &support));
  
  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_data"));
  
  // Create timer (publish at 20 Hz)
  const unsigned int timer_timeout = 50;
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
  Serial.println("Publishing IMU data to topic: /imu_data");
  Serial.println("Ready to communicate with micro-ROS agent!");
  Serial.println("==============================");
  
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
    // Execute callbacks
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  }
  
  delay(10);
}

