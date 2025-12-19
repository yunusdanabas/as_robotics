#!/bin/bash
# I2C Locking Validation Test Script
# Tests the FreeRTOS mutex protection for BNO055 I2C accesses

echo "=========================================="
echo "I2C Locking Validation Test"
echo "=========================================="
echo ""

# Source ROS 2 environment
mamba deactivate 2>/dev/null
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "ERROR: ROS 2 Jazzy not found at /opt/ros/jazzy/setup.bash"
    echo "Please source ROS 2 manually and run this script again"
    exit 1
fi

echo "Step 1: Checking available topics..."
echo "-----------------------------------"
if ros2 topic list 2>/dev/null | grep -q imu; then
    ros2 topic list | grep imu
    echo ""
else
    echo "WARNING: No IMU topics found. Is the micro-ROS agent running?"
    echo "Run: cd ~/as_robotics && ./run_agent_wifi.sh 8888"
    echo ""
    read -p "Press Enter to continue anyway or Ctrl+C to exit..."
fi

echo "Step 2: Testing IMU1 publish rate (10 seconds)..."
echo "-----------------------------------"
echo "Expected: Stable 50-100 Hz"
echo "Watch for: No 'no new messages' warnings"
echo ""
timeout 10 ros2 topic hz /imu1_data 2>&1 | head -20
echo ""

echo "Step 3: Testing IMU2 publish rate (10 seconds)..."
echo "-----------------------------------"
echo "Expected: Stable 50-100 Hz"
echo ""
timeout 10 ros2 topic hz /imu2_data 2>&1 | head -20
echo ""

echo "Step 4: Testing fused publish rate (10 seconds)..."
echo "-----------------------------------"
echo "Expected: Stable 50-100 Hz"
echo ""
timeout 10 ros2 topic hz /imu_fused 2>&1 | head -20
echo ""

echo "=========================================="
echo "Test Complete"
echo "=========================================="
echo ""
echo "IMPORTANT: Check serial monitor in another terminal for I2C errors!"
echo "Run: cd ~/as_robotics/esp32_imu_master && mamba activate main && pio device monitor"
echo ""
echo "What to look for in serial output:"
echo "  ✓ 'FreeRTOS sensor polling task started on Core 1'"
echo "  ✓ No I2C errors after initialization"
echo "  ✗ No [E][Wire.cpp:499] errors during operation"
echo "  ✗ No device resets or crashes"
echo ""
echo "For stress testing:"
echo "  1. Keep serial monitor running"
echo "  2. Run this script again while rapidly rotating the device"
echo "  3. Verify publish rates stay stable and no I2C errors appear"
