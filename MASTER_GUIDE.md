# IMU-Based Tele-Imitation System - Master Guide

## Project Overview

This project implements a wireless IMU-based master device for "Programming by Demonstration" robotics. An operator holds a handheld device containing IMU sensors connected to an ESP32 microcontroller, which captures hand orientation (roll, pitch, yaw) in real-time. The orientation data is transmitted to a ROS 2 host computer where it can be visualized in RViz2 and eventually used to control industrial robots.

**Key Components:**
- **Hardware**: ESP32 (WEMOS D1 R32) + BNO055 IMU (primary) + MPU6050 10DOF board (secondary)
- **Software**: ROS 2 Jazzy, micro-ROS, PlatformIO
- **Communication**: Serial (USB cable) or WiFi (UDP)
- **Performance**: Deterministic 50-100 Hz publish rate with <100ms latency (I2C at 400kHz, FreeRTOS sensor polling at 500Hz, status check every 10s)

**Single IMU Hardware Wiring (BNO055 only):**
```
BNO055 SDA  -->  ESP32 GPIO21
BNO055 SCL  -->  ESP32 GPIO22
BNO055 VCC  -->  3.3V
BNO055 GND  -->  GND
```

**Dual IMU Hardware Wiring (BNO055 + MPU6050 10DOF board):**
```
All sensors share the same I2C bus:
ESP32 GPIO21 (SDA) --> BNO055 SDA, MPU6050 SDA
ESP32 GPIO22 (SCL) --> BNO055 SCL, MPU6050 SCL
3.3V               --> BNO055 VCC, MPU6050 VCC
GND                --> BNO055 GND, MPU6050 GND

I2C Addresses:
- BNO055:    0x28 (primary IMU with hardware fusion)
- MPU6050:   0x68 (accelerometer + gyroscope)
- HMC5883L:  0x1E (magnetometer - currently disabled)
- QMC5883L:  0x0D (alternative magnetometer - currently disabled)
- BMP180:    0x77 (barometer - optional)
```

---

## Initial Setup (One Time)

### 1. Serial Port Permissions

```bash
sudo usermod -a -G dialout $USER
# Logout and login for this to take effect
```

### 2. Add PlatformIO to PATH

Add to `~/.bashrc`:
```bash
export PATH="$HOME/.local/bin:$PATH"
```

### 3. Find Your Serial Port

```bash
cd /home/yunusdanabas/as_robotics
./find_serial_port.sh
```

Note the device name (typically `/dev/ttyUSB0` or `/dev/ttyACM0`).

### 4. Build ROS 2 Visualization Package

```bash
cd /home/yunusdanabas/as_robotics/microros_ws
mamba deactivate
source /opt/ros/jazzy/setup.bash
colcon build --packages-select imu_visualization
```

---

## Option A: Read IMU Data via Serial (USB Cable)

This is the default and most reliable method for development and testing.

### Terminal 1: Start micro-ROS Agent

```bash
cd /home/yunusdanabas/as_robotics
./run_agent_serial.sh /dev/ttyUSB0 115200
```

Wait for: `[info] Agent running...`

### Terminal 2: Upload Firmware (first time or after changes)

```bash
cd /home/yunusdanabas/as_robotics/esp32_imu_master
mamba activate main
pio run --target upload
```

### Terminal 3: View IMU Data

```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
ros2 topic echo /imu_data --qos-reliability best_effort
```

### Terminal 4: Launch Visualization (optional)

```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
source ~/as_robotics/microros_ws/install/setup.bash
ros2 launch imu_visualization imu_visualization.launch.py
```

---

## Option B: Read IMU Data via WiFi (Wireless)

WiFi mode allows untethered operation for demonstrations and mobility.

### Step 1: Configure WiFi Credentials

```bash
cd /home/yunusdanabas/as_robotics/esp32_imu_master/src

# Copy the template
cp wifi_config.h wifi_credentials.h

# Edit wifi_credentials.h with your network settings:
# - WIFI_SSID: Your WiFi network name
# - WIFI_PASSWORD: Your WiFi password
# - AGENT_IP: Your computer's IP address (run: hostname -I)
# - AGENT_PORT: 8888 (default)
```

### Step 2: Upload WiFi Firmware

**Note:** The current firmware already supports WiFi mode. No file switching needed.

```bash
cd /home/yunusdanabas/as_robotics/esp32_imu_master
mamba activate main
pio run --target upload
```

### Terminal 1: Start WiFi micro-ROS Agent

```bash
cd /home/yunusdanabas/as_robotics
./run_agent_wifi.sh 8888
```

Wait for connection from ESP32. You should see:
```
[INFO] Agent running on port 8888
[INFO] Client connected
```

### Terminal 2: View IMU Data

```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
ros2 topic echo /imu_data --qos-reliability best_effort
```

### Terminal 3: Launch Visualization (optional)

```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
source ~/as_robotics/microros_ws/install/setup.bash
ros2 launch imu_visualization imu_visualization.launch.py
```

---

## Option C: Dual IMU Mode (Two Sensors)

Dual IMU mode uses both BNO055 and MPU6050 sensors for redundant orientation tracking and sensor fusion. The BNO055 provides hardware-fused orientation, while the MPU6050 uses software-based Madgwick filtering.

### Complete Workflow Summary

**5 Terminal Setup (in order):**

1. **Terminal 1:** Upload firmware → `cd /home/yunusdanabas/as_robotics/esp32_imu_master && mamba activate main && pio run -e esp32dev_dual --target upload`
2. **Terminal 2:** Start agent → `cd /home/yunusdanabas/as_robotics && ./run_agent_wifi.sh 8888`
3. **Terminal 3:** Check topics → `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && ros2 topic hz /imu1_data`
4. **Terminal 4:** Launch RViz → `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && source ~/as_robotics/microros_ws/install/setup.bash && ros2 launch imu_visualization dual_imu_spaced.launch.py`
5. **Terminal 5:** (Optional) Monitor serial → `cd /home/yunusdanabas/as_robotics/esp32_imu_master && mamba activate main && pio device monitor`

**See "Quick Start Guide" below for detailed step-by-step instructions.**

### Technical Details

| Sensor | I2C Address | Function | Fusion Method |
|--------|-------------|----------|---------------|
| BNO055 | 0x28 | Primary IMU | Hardware fusion (internal) |
| MPU6050 | 0x68 | Secondary IMU | FastMadgwick filter (beta=0.8) |
| Magnetometer | 0x1E/0x0D | Heading | **Disabled** (causes I2C errors) |

**Note:** The magnetometer on the 10DOF board is currently disabled to prevent I2C bus errors. The MPU6050 uses IMU-only fusion (accelerometer + gyroscope).

### Quick Start Guide (Complete Command Sequence)

Follow these steps in order. Each step should be run in a separate terminal.

#### Terminal 1: Upload Dual IMU Firmware

```bash
cd /home/yunusdanabas/as_robotics/esp32_imu_master
mamba activate main
pio run -e esp32dev_dual --target upload
```

**Expected output:**
```
Building...
Uploading...
Successfully uploaded!
```

#### Terminal 2: Start micro-ROS Agent (WiFi)

```bash
cd /home/yunusdanabas/as_robotics
./run_agent_wifi.sh 8888
```

**Wait for:**
```
[INFO] Agent running on port 8888
[INFO] Client connected
```

#### Terminal 3: Verify Topics Are Publishing

**Check that all three topics exist:**
```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

**You should see:**
```
/imu1_data
/imu2_data
/imu_fused
```

**Check publish rates (should be ~50 Hz each):**
```bash
# Check IMU1 (BNO055) rate
ros2 topic hz /imu1_data

# In another terminal or after Ctrl+C, check IMU2 (MPU6050) rate
ros2 topic hz /imu2_data

# Check fused IMU rate
ros2 topic hz /imu_fused
```

**Quick data check (one message each):**
```bash
# View IMU1 data
ros2 topic echo /imu1_data --qos-reliability best_effort --once

# View IMU2 data
ros2 topic echo /imu2_data --qos-reliability best_effort --once

# View fused data
ros2 topic echo /imu_fused --qos-reliability best_effort --once
```

#### Terminal 4: Launch RViz2 Visualization

**Recommended: Spaced Layout (all 3 IMUs visible, 8cm apart)**
```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
source ~/as_robotics/microros_ws/install/setup.bash
ros2 launch imu_visualization dual_imu_spaced.launch.py
```

**This will:**
- Open RViz2 automatically
- Display IMU1 (BNO055) 4cm to the left
- Display IMU2 (MPU6050) 4cm to the right
- Display Fused IMU in center with larger axes
- All three update in real-time as you rotate the device

**Verify in RViz2:**
1. You should see three sets of colored axes (red=X, green=Y, blue=Z)
2. Rotate the device - all three should move together
3. In the left panel, expand **TF** display
4. You should see frames: `world`, `imu1_link`, `imu2_link`, `imu_fused_link`

#### Terminal 5: Monitor Serial Output (Optional - for debugging)

```bash
cd /home/yunusdanabas/as_robotics/esp32_imu_master
mamba activate main
pio device monitor
```

**Expected output:**
```
ESP32 IMU Master - Dual IMU Mode (WiFi)
==============================
Connecting to WiFi: YOUR_SSID
WiFi connected!
IP address: 192.168.1.xxx

Initializing BNO055 sensor...
[E][Wire.cpp:499] requestFrom(): i2cWriteReadNonStop returned Error -1
... (I2C errors during init are NORMAL)
BNO055 initialized successfully!
Initializing MPU6050...
MPU6050 initialized successfully!
Magnetometer: DISABLED (to avoid I2C errors)
Fast Madgwick filter initialized (rate=50 Hz, beta=0.80)

micro-ROS initialized successfully!
Publishing IMU1 data to topic: /imu1_data
Publishing IMU2 data to topic: /imu2_data
Publishing fused data to topic: /imu_fused
Ready to communicate with micro-ROS agent!
```

### Detailed Step-by-Step Guide

#### Step 1: Upload Dual IMU Firmware

```bash
cd /home/yunusdanabas/as_robotics/esp32_imu_master
mamba activate main
pio run -e esp32dev_dual --target upload
```

### Step 2: Start micro-ROS Agent (Terminal 1)

```bash
cd /home/yunusdanabas/as_robotics
./run_agent_wifi.sh 8888
```

Wait for: `[info] Agent running...`

### Step 3: Monitor ESP32 Serial Output (Terminal 2 - Optional)

```bash
cd /home/yunusdanabas/as_robotics/esp32_imu_master
mamba activate main
pio device monitor
```

**Expected output:**
```
ESP32 IMU Master - Dual IMU Mode (WiFi)
==============================
Connecting to WiFi: YOUR_SSID
WiFi connected!
IP address: 192.168.1.xxx

Initializing BNO055 sensor...
[E][Wire.cpp:499] requestFrom(): i2cWriteReadNonStop returned Error -1
... (multiple I2C errors - THIS IS NORMAL during BNO055 reset sequence)
BNO055 initialized successfully!
Initializing MPU6050...
MPU6050 initialized successfully!
Magnetometer: DISABLED (to avoid I2C errors)
Fast Madgwick filter initialized (rate=50 Hz, beta=0.80)

micro-ROS initialized successfully!
Publishing IMU1 data to topic: /imu1_data
Publishing IMU2 data to topic: /imu2_data
Publishing fused data to topic: /imu_fused
Ready to communicate with micro-ROS agent!
```

**Important:** The I2C errors during BNO055 initialization are **normal** and expected. The BNO055 internally resets during initialization which temporarily disrupts the I2C bus. As long as you see "BNO055 initialized successfully!", the sensor is working correctly.

### Step 4: Verify Topics Are Publishing (Terminal 3)

**Important:** micro-ROS uses `best_effort` QoS. You must specify this when echoing topics:

```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
ros2 topic echo /imu1_data --qos-reliability best_effort --once
ros2 topic echo /imu2_data --qos-reliability best_effort --once
ros2 topic echo /imu_fused --qos-reliability best_effort --once
```

Check publish rates:
```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
ros2 topic hz /imu1_data
# Should show ~50 Hz
```

### Step 5: Visualize in RViz2

**Recommended: Spaced Layout (8cm apart, fused in center with larger axes)**

```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
source ~/as_robotics/microros_ws/install/setup.bash
ros2 launch imu_visualization dual_imu_spaced.launch.py
```

This will display:
- **IMU1 (BNO055)**: 4cm to the left (-0.04m on Y-axis)
- **IMU2 (MPU6050)**: 4cm to the right (+0.04m on Y-axis)
- **Fused IMU**: Center (0.0m) with **larger axes** (0.6m length, 0.06m radius)

**Alternative: Manual Setup (if you need custom configuration)**

**Method A: Launch IMU1 with RViz (Terminal 3)**

```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
source ~/as_robotics/microros_ws/install/setup.bash
ros2 launch imu_visualization imu_visualization.launch.py imu_topic:=/imu1_data imu_frame:=imu1_link
```

**Method B: Add IMU2 TF Broadcaster with offset (Terminal 4)**

```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
source ~/as_robotics/microros_ws/install/setup.bash
ros2 run imu_visualization imu_tf_broadcaster --ros-args -p imu_topic:=/imu2_data -p imu_frame:=imu2_link -p translation_y:=0.04 -r __node:=imu2_tf_broadcaster
```

**Method C: Add Fused IMU TF Broadcaster (Terminal 5)**

```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
source ~/as_robotics/microros_ws/install/setup.bash
ros2 run imu_visualization imu_tf_broadcaster --ros-args -p imu_topic:=/imu_fused -p imu_frame:=imu_fused_link -r __node:=imu_fused_tf_broadcaster
```

### Step 6: Configure RViz2 to Show All IMUs

In the RViz2 window (already open from Step 5 Method A):

1. Click **Add** button (bottom left panel)
2. Select **By display type** tab
3. Choose **TF** and click **OK**
4. Expand the TF display in the left panel
5. You should see frames: `world`, `imu1_link`, `imu2_link`, `imu_fused_link`

All three IMU orientations will now update in real-time as you rotate the device.

### Dual IMU Topics

| Topic | Description | Frame ID |
|-------|-------------|----------|
| `/imu1_data` | BNO055 orientation (hardware fusion) | `imu1_link` |
| `/imu2_data` | MPU6050 orientation (FastMadgwick filter) | `imu2_link` |
| `/imu_fused` | Combined orientation (SLERP blend) | `imu_fused_link` |

### Dual IMU TF Frames in RViz2

| Frame | Parent | Source |
|-------|--------|--------|
| `imu1_link` | `world` | BNO055 |
| `imu2_link` | `world` | MPU6050 |
| `imu_fused_link` | `world` | Sensor fusion |

### FastMadgwick Filter Settings

The MPU6050 uses a custom FastMadgwick filter for responsive orientation tracking:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `sampleFreq` | 50 Hz | Filter update rate |
| `beta` | 0.8 | Filter gain (higher = more responsive, more noise) |

These settings are configured in `esp32_imu_master/src/main.cpp` under `DUAL_IMU_MODE`.

### Dual IMU Verification Checklist

- [x] Serial monitor shows "BNO055 initialized successfully!"
- [x] Serial monitor shows "MPU6050 initialized successfully!"
- [x] Serial monitor shows "Magnetometer: DISABLED"
- [x] Serial monitor shows "micro-ROS initialized successfully!"
- [x] `ros2 topic echo /imu1_data --qos-reliability best_effort --once` shows data
- [x] `ros2 topic echo /imu2_data --qos-reliability best_effort --once` shows data
- [x] `ros2 topic echo /imu_fused --qos-reliability best_effort --once` shows data
- [x] RViz2 displays `imu1_link` frame responding to rotation
- [x] RViz2 displays `imu2_link` frame responding to rotation
- [x] RViz2 displays `imu_fused_link` frame responding to rotation

---

## Quick Command Reference

### Single IMU Mode

| Task | Command |
|------|---------|
| Find serial port | `cd /home/yunusdanabas/as_robotics && ./find_serial_port.sh` |
| Upload firmware | `cd /home/yunusdanabas/as_robotics/esp32_imu_master && mamba activate main && pio run --target upload` |
| Monitor serial output | `cd /home/yunusdanabas/as_robotics/esp32_imu_master && mamba activate main && pio device monitor` |
| Start serial agent | `cd /home/yunusdanabas/as_robotics && ./run_agent_serial.sh /dev/ttyUSB0 115200` |
| Start WiFi agent | `cd /home/yunusdanabas/as_robotics && ./run_agent_wifi.sh 8888` |
| List ROS topics | `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && ros2 topic list` |
| View IMU data | `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && ros2 topic echo /imu_data --qos-reliability best_effort` |
| Check data rate | `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && ros2 topic hz /imu_data` |
| View Euler angles | `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && source ~/as_robotics/microros_ws/install/setup.bash && ros2 topic echo /imu/euler` |
| Launch visualization | `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && source ~/as_robotics/microros_ws/install/setup.bash && ros2 launch imu_visualization imu_visualization.launch.py` |
| View TF transforms | `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && ros2 run tf2_ros tf2_echo world imu_link` |

### Dual IMU Mode

| Task | Command |
|------|---------|
| **Upload dual firmware** | `cd /home/yunusdanabas/as_robotics/esp32_imu_master && mamba activate main && pio run -e esp32dev_dual --target upload` |
| Start WiFi agent | `cd /home/yunusdanabas/as_robotics && ./run_agent_wifi.sh 8888` |
| List all topics | `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && ros2 topic list` |
| View IMU1 data (once) | `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && ros2 topic echo /imu1_data --qos-reliability best_effort --once` |
| View IMU2 data (once) | `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && ros2 topic echo /imu2_data --qos-reliability best_effort --once` |
| View fused data (once) | `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && ros2 topic echo /imu_fused --qos-reliability best_effort --once` |
| Check IMU1 rate | `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && ros2 topic hz /imu1_data` |
| Check IMU2 rate | `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && ros2 topic hz /imu2_data` |
| Check fused rate | `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && ros2 topic hz /imu_fused` |
| **Launch RViz (spaced layout)** | `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && source ~/as_robotics/microros_ws/install/setup.bash && ros2 launch imu_visualization dual_imu_spaced.launch.py` |
| View TF frames | `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && ros2 run tf2_ros tf2_echo world imu1_link` |
| List all TF frames | `cd /home/yunusdanabas/as_robotics && mamba deactivate && source /opt/ros/jazzy/setup.bash && ros2 run tf2_ros tf2_monitor` |
| Monitor serial output | `cd /home/yunusdanabas/as_robotics/esp32_imu_master && mamba activate main && pio device monitor` |

### Environment Setup (Run Before ROS 2 Commands)

**For PlatformIO/ESP32 commands:**
```bash
cd /home/yunusdanabas/as_robotics/esp32_imu_master
mamba activate main
```

**For ROS 2 commands:**
```bash
cd /home/yunusdanabas/as_robotics
# IMPORTANT: Deactivate conda/mamba environment first if active
mamba deactivate

# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source ~/as_robotics/microros_ws/install/setup.bash
```

**Note:** ROS 2 Jazzy requires Python 3.12. If you have a conda/mamba environment active (e.g., Python 3.10), you must deactivate it first or ROS 2 nodes will fail with `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`.

---

## Verification Checklist

After setup, verify the following:

- [ ] ESP32 serial monitor shows "BNO055 initialized successfully!"
- [ ] Agent shows connection messages
- [ ] `ros2 topic list` shows `/imu_data`
- [ ] `ros2 topic hz /imu_data` shows ~50-100 Hz (stable)
- [ ] Quaternion values change when rotating the sensor
- [ ] RViz2 shows IMU orientation moving with the device

---

## LED Indicators

- **Fast blinking**: Error (sensor not detected, check wiring)
- **Solid on**: Ready, waiting for agent connection
- **Slow blinking**: Normal operation, publishing data

---

## Troubleshooting

### Permission denied on serial port
```bash
sudo chmod 666 /dev/ttyUSB0
```

### Firmware won't upload
```bash
cd /home/yunusdanabas/as_robotics/esp32_imu_master
mamba activate main
pio device list  # Check device
```
- Try different USB cable (must support data, not just power)
- Hold BOOT button on ESP32 while uploading
- Verify device appears in `pio device list`

### Agent won't connect
- Start agent BEFORE uploading firmware
- Verify baud rate is 115200
- For WiFi: check IP address and port in `wifi_credentials.h`
- For serial: verify port name: `cd /home/yunusdanabas/as_robotics && ./find_serial_port.sh`

### No IMU data
- Check BNO055 wiring (SDA/SCL, 3.3V power)
- Run serial monitor: `cd /home/yunusdanabas/as_robotics/esp32_imu_master && mamba activate main && pio device monitor`
- Verify sensor initialization messages appear

### Orientation drift or inaccuracy
- Run calibration procedure (see CALIBRATION.md)
- Keep sensor away from metal objects and electronics during calibration

### I2C Errors During BNO055 Initialization

```
[E][Wire.cpp:499] requestFrom(): i2cWriteReadNonStop returned Error -1
```

**This is NORMAL!** The BNO055 performs an internal reset during initialization which temporarily disrupts I2C communication. As long as you see "BNO055 initialized successfully!" afterwards, the sensor is working correctly.

### `ros2 topic echo` Shows No Data

**All micro-ROS publishers use `best_effort` QoS** (explicitly configured for low latency). You must specify this when using `ros2 topic` commands:

```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
ros2 topic echo /imu1_data --qos-reliability best_effort
ros2 topic hz /imu1_data --qos-reliability best_effort
```

**Note:** The `--qos-reliability best_effort` flag is required for all `ros2 topic` commands with this firmware.

### RViz Not Showing IMU Movement

1. Check if TF is being published:
```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros tf2_echo world imu1_link
```
2. Ensure the TF broadcaster is listening to the correct topic (check terminal output)
3. In RViz, verify "Fixed Frame" is set to `world`
4. Add a TF display if not already present

### Python Version Mismatch Error

```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
```

**Cause:** ROS 2 Jazzy requires Python 3.12, but a conda/mamba environment with Python 3.10 is active.

**Solution:**
```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
source ~/as_robotics/microros_ws/install/setup.bash
```

### MPU6050 Orientation Is Slow/Unresponsive

The MPU6050 uses software-based Madgwick filtering with variable time-step. If it feels slow compared to BNO055:

1. Edit: `cd /home/yunusdanabas/as_robotics/esp32_imu_master/src && nano imu_config.h`
2. Increase the `MADGWICK_DEFAULT_BETA` value (default: 0.8, max: 1.0)
3. Higher beta = faster response but more noise
4. Rebuild and upload:
```bash
cd /home/yunusdanabas/as_robotics/esp32_imu_master
mamba activate main
pio run -e esp32dev_dual --target upload
```

**Note:** Configuration constants are now centralized in `imu_config.h` for easier tuning.

### ROS 2 Package Changes Not Taking Effect

If you modify Python files in the ROS 2 package but changes don't appear:

```bash
# Clean rebuild
cd /home/yunusdanabas/as_robotics
rm -rf ~/as_robotics/microros_ws/build/imu_visualization
rm -rf ~/as_robotics/microros_ws/install/imu_visualization

# Rebuild
cd /home/yunusdanabas/as_robotics/microros_ws
mamba deactivate
source /opt/ros/jazzy/setup.bash
colcon build --packages-select imu_visualization
source install/setup.bash
```

---

## Performance Validation

After deploying the optimized firmware, verify performance metrics:

### Timer Callback Duration
- **Target**: <10ms (timer period is 10ms)
- **Method**: Toggle GPIO pin at start/end of `timer_callback`, measure with oscilloscope
- **Expected**: <3ms with cache-based reads

### Publish Rate Stability
```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
ros2 topic hz /imu_data
```
- **Target**: Stable 50-100 Hz
- **Expected**: Consistent rate with minimal jitter

### End-to-End Latency
- **Target**: <100ms (motion to visualization)
- **Method**: Measure time from physical device rotation to RViz update
- **Expected**: 50-80ms typical

### Error Monitoring
The firmware maintains a non-blocking error counter. Monitor serial output for:
- `micro_ros_error_count`: Increments on publish failures (non-critical with Best-Effort QoS)

## Related Documentation

| Document | Purpose |
|----------|---------|
| `CALIBRATION.md` | Sensor calibration procedures |
| `project.txt` | Project description and architecture |
| `Performance/OPEX.md` | Performance optimization details |

---

## Workspace Structure

```
~/as_robotics/
├── esp32_imu_master/           # PlatformIO firmware project
│   ├── src/
│   │   ├── main.cpp            # Active firmware (supports single/dual IMU)
│   │   ├── main_mpu6050.cpp    # MPU6050-only test firmware
│   │   ├── sensor_fusion.h     # Quaternion utilities + FastMadgwick filter
│   │   ├── imu_config.h        # Centralized hardware constants & configuration
│   │   ├── imu_diagnostics.h   # Diagnostic logging utilities
│   │   ├── wifi_config.h       # WiFi credentials template
│   │   └── wifi_credentials.h  # Your WiFi settings (not in git)
│   └── platformio.ini          # Build configuration:
│                               #   - esp32dev: Single BNO055 (optimized)
│                               #   - esp32dev_dual: BNO055 + MPU6050 (optimized)
│                               #   - esp32dev_mpu6050: MPU6050 only
│                               #   - esp32dev_debug: Debug build with diagnostics
│
├── microros_ws/                # ROS 2 workspace
│   └── src/imu_visualization/
│       ├── imu_visualization/
│       │   ├── imu_tf_broadcaster.py      # TF broadcaster (configurable topic)
│       │   ├── dual_imu_tf_broadcaster.py # Dual IMU TF broadcaster
│       │   ├── imu_fusion_node.py         # IMU fusion (SLERP/weighted avg)
│       │   └── imu_to_euler.py            # Quaternion to Euler converter
│       ├── launch/
│       │   ├── imu_visualization.launch.py  # Single/configurable IMU launch
│       │   └── dual_imu.launch.py           # Dual IMU launch
│       └── rviz/
│           ├── imu_view.rviz              # Single IMU RViz config
│           └── dual_imu_view.rviz         # Dual IMU RViz config
│
├── run_agent_serial.sh         # Serial agent launcher
├── run_agent_wifi.sh           # WiFi agent launcher
├── find_serial_port.sh         # Port discovery helper
└── MASTER_GUIDE.md             # This file
```

---

## Technical Implementation Details

### Sensor Fusion Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        ESP32 (Dual IMU Mode)                     │
│                    Optimized for Real-Time Performance            │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  FreeRTOS Sensor Polling Task (Core 1, 500Hz)           │   │
│  │  ┌───────────┐     ┌───────────┐                         │   │
│  │  │  BNO055   │────>│  Cache   │                         │   │
│  │  │ (0x28)    │     │ (Lock-   │                         │   │
│  │  └───────────┘     │  Free)   │                         │   │
│  │                    └───────────┘                         │   │
│  │  ┌───────────┐     ┌───────────┐                         │   │
│  │  │  MPU6050  │────>│  Cache   │                         │   │
│  │  │ (0x68)    │     │ (Lock-   │                         │   │
│  │  └───────────┘     │  Free)   │                         │   │
│  └──────────────────────────────────────────────────────────┘   │
│                    │                    │                       │
│                    ▼                    ▼                       │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  Timer Callback (Core 0, 100Hz, <3ms execution)         │   │
│  │  ┌────────────────┐     ┌─────────────────┐              │   │
│  │  │ Hardware Fusion│────>│  /imu1_data     │              │   │
│  │  │ (from cache)   │     │  (50-100 Hz)    │              │   │
│  │  └────────────────┘     └─────────────────┘              │   │
│  │  ┌────────────────┐     ┌─────────────────┐              │   │
│  │  │ FastMadgwick   │────>│  /imu2_data     │              │   │
│  │  │ (variable dt)  │     │  (50-100 Hz)    │              │   │
│  │  └────────────────┘     └─────────────────┘              │   │
│  │  ┌─────────────────────────────────────┐                  │   │
│  │  │         SLERP Fusion                 │                  │   │
│  │  │  (blend_factor=0.5, equal weight)   │                  │   │
│  │  └─────────────────────────────────────┘                  │   │
│  │                    │                                      │   │
│  │                    ▼                                      │   │
│  │          ┌─────────────────┐                              │   │
│  │          │  /imu_fused     │                              │   │
│  │          │  (50-100 Hz)    │                              │   │
│  │          └─────────────────┘                              │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              │ WiFi UDP (Best-Effort QoS)
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Host Computer (ROS 2 Jazzy)                   │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────────────┐                                            │
│  │ micro-ROS Agent  │ (UDP port 8888)                            │
│  └──────────────────┘                                            │
│           │                                                      │
│           ▼                                                      │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    ROS 2 Topics                           │   │
│  │  /imu1_data ──> imu_tf_broadcaster ──> TF: imu1_link      │   │
│  │  /imu2_data ──> imu_tf_broadcaster ──> TF: imu2_link      │   │
│  │  /imu_fused ──> imu_tf_broadcaster ──> TF: imu_fused_link │   │
│  └──────────────────────────────────────────────────────────┘   │
│           │                                                      │
│           ▼                                                      │
│  ┌──────────────────┐                                            │
│  │      RViz2       │  (Visualizes all 3 TF frames)              │
│  └──────────────────┘                                            │
└─────────────────────────────────────────────────────────────────┘
```

### FastMadgwick Filter

The `FastMadgwick` class in `sensor_fusion.h` is a custom implementation of the Madgwick AHRS filter with configurable beta (filter gain) and variable time-step support:

- **beta = 0.1**: Slow response, very smooth (good for static applications)
- **beta = 0.5**: Balanced response and smoothness
- **beta = 0.8**: Fast response, some noise (current setting for responsiveness)
- **beta = 1.0**: Fastest response, most noise

**Performance Optimizations:**
- **Variable Time-Step**: Uses measured `dt` from `micros()` instead of fixed `1/sampleFreq` for improved stability during timing jitter
- **Fast-Path**: Quaternion tracker skips expensive SLERP/NLERP when orientation change is negligible (dot > 0.9998)
- **NLERP**: Uses Normalized Linear Interpolation instead of SLERP for faster computation during small movements

The Adafruit_AHRS library has a hardcoded beta value and fixed time-step, which is why we use this custom implementation.

### Performance Optimizations

The firmware has been optimized for deterministic real-time performance to prevent Pilot-Induced Oscillation (PIO):

**Architecture Changes:**
- **FreeRTOS Sensor Polling Task**: Dedicated task on Core 1 polls sensors at 500Hz (2ms period) and updates lock-free caches
- **Non-Blocking Timer Callback**: Timer callback reads from caches instead of blocking I2C, reducing execution time from 8-12ms to <3ms
- **Variable Time-Step Filtering**: Madgwick filter uses measured `dt` for improved stability during timing jitter
- **Fast-Path Optimization**: Quaternion tracker skips expensive math when orientation is nearly stationary
- **Compiler Optimizations**: `-O3 -ffast-math` flags for 20-30% performance improvement

**Expected Performance:**
- Timer callback duration: <3ms (was 8-12ms)
- Publish rate: Stable 50-100Hz (was 30-50Hz variable)
- End-to-end latency: <100ms (was 150-300ms)
- CPU usage: Reduced by 60-75% in timer interrupt

### QoS Settings

micro-ROS publishers are explicitly configured with `best_effort` reliability for low-latency performance over WiFi. The ROS 2 visualization nodes use matching QoS:

```python
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)
```

**Note:** All `ros2 topic` commands must include `--qos-reliability best_effort` to receive data.

---

**Performance Notes:**
- The firmware is optimized for deterministic real-time performance with FreeRTOS task separation
- Timer callback execution time is <3ms (down from 8-12ms) due to cache-based sensor reads
- All publishers use Best-Effort QoS for low latency - always include `--qos-reliability best_effort` in `ros2 topic` commands
- Configuration constants are centralized in `imu_config.h` for easy tuning

**Ready to start?** Follow "Option A: Serial" for initial testing, then switch to "Option B: WiFi" for wireless operation, and finally "Option C: Dual IMU" for dual sensor setup.
