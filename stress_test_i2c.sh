#!/bin/bash
# I2C Locking Stress Test
# Tests the system under heavy load with rapid device movement

# Don't exit on errors (timeout commands return non-zero)
set +e

echo "=========================================="
echo "I2C Locking Stress Test"
echo "=========================================="
echo ""
echo "This test will stress the I2C locking implementation by:"
echo "  1. Monitoring publish rates during rapid movement"
echo "  2. Checking for I2C errors in serial output"
echo "  3. Verifying system stability"
echo ""
echo "PREREQUISITES:"
echo "  - ESP32 firmware uploaded and running"
echo "  - micro-ROS agent running (Terminal 1)"
echo "  - Serial monitor running (Terminal 2)"
echo "  - This script will run in Terminal 3"
echo ""
echo "Starting in 3 seconds..."
sleep 3

# Source ROS 2 environment
mamba deactivate 2>/dev/null
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "ERROR: ROS 2 Jazzy not found at /opt/ros/jazzy/setup.bash"
    exit 1
fi

# Check if topics exist
echo ""
echo "Step 1: Verifying topics are available..."
echo "-----------------------------------"
if ! ros2 topic list 2>/dev/null | grep -q imu1_data; then
    echo "ERROR: /imu1_data topic not found!"
    echo "Make sure micro-ROS agent is running and ESP32 is connected."
    exit 1
fi
echo "✓ Topics found:"
ros2 topic list | grep imu
echo ""

# Baseline rate check (stationary)
echo "Step 2: Baseline Rate Check (Device Stationary - 5 seconds)"
echo "-----------------------------------"
echo "Keep device STILL for this measurement..."
echo ""
echo "Starting baseline measurement in 3 seconds..."
sleep 3

echo "Measuring baseline rates (5 seconds)..."
echo ""
echo "=== IMU1 Baseline ==="
timeout 5 ros2 topic hz /imu1_data 2>&1 | tail -3
echo ""
echo "=== IMU2 Baseline ==="
timeout 5 ros2 topic hz /imu2_data 2>&1 | tail -3
echo ""
echo "=== Fused Baseline ==="
timeout 5 ros2 topic hz /imu_fused 2>&1 | tail -3
echo ""

# Stress test phase
echo "=========================================="
echo "Step 3: STRESS TEST - Rapid Movement"
echo "=========================================="
echo ""
echo "INSTRUCTIONS:"
echo "  1. Rapidly rotate the device in ALL axes simultaneously"
echo "  2. Continue for 60 seconds"
echo "  3. Make movements as fast and erratic as possible"
echo "  4. Watch Terminal 2 (serial monitor) for I2C errors"
echo ""
echo "What to watch for:"
echo "  ✗ [E][Wire.cpp:499] errors in serial output"
echo "  ✗ Rate drops to 0 or 'no new messages'"
echo "  ✗ Device resets or crashes"
echo "  ✓ Rates should stay stable (50-100 Hz)"
echo ""
echo "Starting stress test in 5 seconds..."
echo "GET READY TO ROTATE DEVICE RAPIDLY!"
sleep 5

echo ""
echo "Starting 60-second stress test..."
echo "ROTATE DEVICE RAPIDLY NOW!"
echo ""
echo "Monitoring IMU1 rate (will show updates every ~2 seconds)..."
echo ""

# Monitor IMU1 during stress test
timeout 60 ros2 topic hz /imu1_data 2>&1 &
IMU1_PID=$!

# Also monitor IMU2 in background
timeout 60 ros2 topic hz /imu2_data 2>&1 > /tmp/stress_imu2.log &
IMU2_PID=$!

# Monitor fused
timeout 60 ros2 topic hz /imu_fused 2>&1 > /tmp/stress_fused.log &
FUSED_PID=$!

# Wait for stress test to complete
wait $IMU1_PID 2>/dev/null || true

echo ""
echo "Stress test complete!"
echo ""

# Show results from background processes
echo "=== IMU2 Stress Test Results ==="
tail -3 /tmp/stress_imu2.log 2>/dev/null || echo "No data captured"
echo ""

echo "=== Fused Stress Test Results ==="
tail -3 /tmp/stress_fused.log 2>/dev/null || echo "No data captured"
echo ""

# Recovery test
echo "=========================================="
echo "Step 4: Recovery Test (Device Stationary - 5 seconds)"
echo "=========================================="
echo "Stop rotating device and keep it STILL..."
echo ""
echo "Starting recovery measurement in 3 seconds..."
sleep 3

echo "Measuring recovery rates (5 seconds)..."
echo ""
echo "=== IMU1 Recovery ==="
timeout 5 ros2 topic hz /imu1_data 2>&1 | tail -3
echo ""
echo "=== IMU2 Recovery ==="
timeout 5 ros2 topic hz /imu2_data 2>&1 | tail -3
echo ""
echo "=== Fused Recovery ==="
timeout 5 ros2 topic hz /imu_fused 2>&1 | tail -3
echo ""

# Final validation
echo "=========================================="
echo "Step 5: Final Validation"
echo "=========================================="
echo ""
echo "Check the following in Terminal 2 (Serial Monitor):"
echo ""
echo "✓ No I2C errors during stress test:"
echo "  - Look for [E][Wire.cpp:499] or similar errors"
echo "  - Should see NONE after initialization"
echo ""
echo "✓ No device resets:"
echo "  - Device should not restart during stress test"
echo "  - No 'ESP32 IMU Master - Dual IMU Mode' restart message"
echo ""
echo "✓ System remained stable:"
echo "  - Publish rates stayed above 50 Hz"
echo "  - No 'no new messages' warnings"
echo ""
echo "=========================================="
echo "Stress Test Complete"
echo "=========================================="
echo ""
echo "If all checks pass, I2C locking is working correctly under stress!"
echo ""
echo "Cleanup: Removing temporary files..."
rm -f /tmp/stress_imu2.log /tmp/stress_fused.log

