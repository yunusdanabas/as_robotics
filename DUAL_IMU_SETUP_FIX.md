# Dual IMU Setup - Issues Fixed

## Problems Found and Fixed

### ✅ Issue 1: Transport Mismatch
**Problem:** Firmware was built for WiFi transport, but serial agent was running.
**Fix:** 
- Stopped serial agent
- Started WiFi agent on port 8888
- Agent is now running: `docker ps` shows container `26eaff9f65ff`

### ✅ Issue 2: WiFi Agent Not Running
**Problem:** WiFi agent wasn't started properly.
**Fix:** Started WiFi agent using:
```bash
docker run -d --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v6
```

### ✅ Issue 3: Firmware Upload
**Problem:** Multiple firmware versions were built, but dual IMU wasn't the active one.
**Fix:** Uploaded `esp32dev_dual` firmware successfully.

## Current Status

✅ **WiFi Agent:** Running on port 8888
✅ **ESP32 Connection:** Connected (receiving UDP packets)
✅ **Topics Available:** `/imu_data` is publishing at ~30 Hz
⚠️ **Dual IMU Topics:** Not visible yet (may need ESP32 reset)

## Next Steps to Verify Dual IMU Mode

### Step 1: Verify ESP32 is Running Dual IMU Firmware

The ESP32 may need a hard reset to load the new firmware. Try:

```bash
# Option A: Press RESET button on ESP32 board
# Option B: Unplug and replug USB cable
# Option C: Restart the agent to force reconnection
cd /home/yunusdanabas/as_robotics
docker stop $(docker ps -q --filter "ancestor=microros/micro-ros-agent:jazzy")
docker run -d --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v6
```

### Step 2: Check Serial Output (Optional)

```bash
cd /home/yunusdanabas/as_robotics/esp32_imu_master
mamba run -n main pio device monitor
```

**Expected output for dual IMU mode:**
```
ESP32 IMU Master - Dual IMU Mode (WiFi)
==============================
...
Publishing IMU1 data to topic: /imu1_data
Publishing IMU2 data to topic: /imu2_data
Publishing fused data to topic: /imu_fused
```

### Step 3: Verify Dual IMU Topics

```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

**Should see:**
- `/imu1_data` (BNO055)
- `/imu2_data` (MPU6050)
- `/imu_fused` (Combined)

### Step 4: Check Publish Rates

```bash
# Terminal 1: Check IMU1 rate
ros2 topic hz /imu1_data

# Terminal 2: Check IMU2 rate  
ros2 topic hz /imu2_data

# Terminal 3: Check fused rate
ros2 topic hz /imu_fused
```

**Expected:** ~50 Hz for each topic

### Step 5: Launch RViz Visualization

```bash
cd /home/yunusdanabas/as_robotics
mamba deactivate
source /opt/ros/jazzy/setup.bash
source ~/as_robotics/microros_ws/install/setup.bash
ros2 launch imu_visualization dual_imu_spaced.launch.py
```

## Troubleshooting

### If Dual IMU Topics Still Don't Appear

1. **Verify firmware was uploaded correctly:**
   ```bash
   cd /home/yunusdanabas/as_robotics/esp32_imu_master
   mamba run -n main pio run -e esp32dev_dual --target upload
   ```

2. **Check agent logs for connection:**
   ```bash
   docker logs $(docker ps -q --filter "ancestor=microros/micro-ros-agent:jazzy" | head -1) | tail -20
   ```

3. **Verify WiFi credentials:**
   ```bash
   cat /home/yunusdanabas/as_robotics/esp32_imu_master/src/wifi_config.h
   ```
   Make sure `AGENT_IP` matches your computer's IP: `192.168.1.108`

4. **Hard reset ESP32:**
   - Unplug USB cable
   - Wait 5 seconds
   - Plug back in
   - Wait 10 seconds for reconnection

## Quick Reference Commands

```bash
# Start WiFi agent
cd /home/yunusdanabas/as_robotics
docker run -d --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v6

# Upload dual IMU firmware
cd /home/yunusdanabas/as_robotics/esp32_imu_master
mamba run -n main pio run -e esp32dev_dual --target upload

# Check topics
mamba deactivate
source /opt/ros/jazzy/setup.bash
ros2 topic list

# Launch RViz
source ~/as_robotics/microros_ws/install/setup.bash
ros2 launch imu_visualization dual_imu_spaced.launch.py
```

