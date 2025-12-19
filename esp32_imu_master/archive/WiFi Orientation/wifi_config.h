/*
 * WiFi Configuration for ESP32 IMU Master
 * 
 * Copy this file and rename to wifi_credentials.h
 * Then update with your actual WiFi credentials
 */

#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

// WiFi credentials - UPDATE THESE!
#define WIFI_SSID "ASMETAL"
#define WIFI_PASSWORD "AACC223344"

// micro-ROS Agent IP and Port
// This should be the IP address of your computer running the agent
#define AGENT_IP "192.168.1.108"  // UPDATE THIS to your computer's IP
#define AGENT_PORT 8888

// WiFi connection timeout (milliseconds)
#define WIFI_TIMEOUT 20000

#endif // WIFI_CONFIG_H

