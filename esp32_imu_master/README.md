# ESP32 IMU Master Device

This is the firmware for the handheld IMU master device used in the IMU-Based Tele-Imitation project.

## Hardware

- **Microcontroller**: WEMOS D1 R32 (ESP32)
- **IMU Sensor**: DFRobot Gravity BNO055 + BMP280 (SKU: SEN0253)

## Wiring

| BNO055 | ESP32 |
|--------|-------|
| SDA    | GPIO21 (default I2C SDA) |
| SCL    | GPIO22 (default I2C SCL) |
| VCC    | 3.3V |
| GND    | GND |

## Software Setup

### Prerequisites

- PlatformIO Core or PlatformIO IDE
- ROS 2 Jazzy (on host computer)
- Docker (for micro-ROS agent)

### Building the Firmware

```bash
cd /home/yunusdanabas/as_robotics/esp32_imu_master
pio run
```

### Uploading to ESP32

Connect your WEMOS D1 R32 via USB and run:

```bash
pio run --target upload
```

### Monitoring Serial Output

```bash
pio device monitor
```

## Running the System

### 1. Start the micro-ROS Agent

On your host computer, run the agent script:

```bash
cd /home/yunusdanabas/as_robotics
./run_agent_serial.sh /dev/ttyUSB0 115200
```

(Replace `/dev/ttyUSB0` with your actual serial port)

### 2. View IMU Data

In another terminal, after the agent is running:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo /imu_data
```

### 3. Check Data Rate

```bash
ros2 topic hz /imu_data
```

Expected rate: ~20 Hz

## Troubleshooting

### BNO055 Not Detected

- Check wiring connections
- Verify I2C address (default: 0x28)
- Try power cycling the sensor
- Check if sensor LED is on

### micro-ROS Connection Issues

- Ensure micro-ROS agent is running first
- Check serial port permissions: `sudo usermod -a -G dialout $USER` (logout/login required)
- Verify baud rate matches (115200)
- Try different USB cable or port

### Serial Port Not Found

Find your device:
```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

## LED Status Indicators

- **Fast blinking (10 Hz)**: Error - BNO055 not detected or micro-ROS error
- **Slow blinking**: Normal operation, publishing IMU data
- **Solid on**: Initialization complete, waiting for agent

## Data Published

**Topic**: `/imu_data`  
**Type**: `sensor_msgs/msg/Imu`  
**Frame**: `imu_link`  
**Rate**: 20 Hz

The message includes:
- **Orientation**: Quaternion (x, y, z, w) from BNO055 fusion algorithm
- **Angular Velocity**: Gyroscope data (rad/s)
- **Linear Acceleration**: Accelerometer data (m/s²)

## Next Steps

- Add WiFi transport support
- Create TF broadcaster for visualization
- Integrate with RViz2
- Add battery monitoring
- Implement data recording

## Project Structure

```
esp32_imu_master/
├── src/
│   └── main.cpp          # Main firmware code
├── include/              # Header files (if needed)
├── lib/                  # Custom libraries (if needed)
├── extra_packages/       # Custom ROS messages (future)
├── platformio.ini        # PlatformIO configuration
└── README.md            # This file
```

