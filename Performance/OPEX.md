# IMU Tele-Imitation System – Performance & Implementation Guide

This document summarizes the key performance bottlenecks identified in the ESP32 IMU firmware and ROS 2 host code, along with concrete implementation steps and file/line references for quick edits.

---

## High-Priority Fixes

### 1) Move I2C reads out of the timer callback
- **Problem**: The 10 ms timer executes multiple blocking I2C reads (BNO055 quaternion + gyro + accel, MPU6050 events) plus fusion and publishing logic. This can exceed the 10 ms budget and drop the publish rate.  
  **Location**: `esp32_imu_master/src/main.cpp:520-730` (dual mode), `733-841` (single mode).
- **Implementation**:
  1. Create a dedicated FreeRTOS task (e.g., 400 Hz loop pinned to the opposite core) that polls BNO055 and MPU6050, then writes into volatile, lock-free caches:
     ```cpp
     struct ImuSample { imu::Quaternion q; imu::Vector<3> gyro; imu::Vector<3> accel; uint64_t stamp_ms; bool valid; };
     volatile ImuSample bno_cache, mpu_cache;
     ```
  2. In `timer_callback`, replace `getQuat`/`getVector`/`getEvent` calls with cached values and validity checks. If a cache is stale (e.g., older than 30 ms), skip publish for that stream.
  3. Keep fusion and publishing logic, but remove all sensor I/O from the timer.
  4. Reuse a single timestamp per tick (see item 4).

### 2) Use real `dt` in Madgwick integration
- **Problem**: `FastMadgwick::updateIMU` integrates with a fixed `dt = 1/sampleFreq`, so when the loop slows, the filter gain is wrong and forces more smoothing.  
  **Location**: `esp32_imu_master/src/sensor_fusion.h:337-349` and call at `src/main.cpp:619-684`.
- **Implementation**:
  1. Change `FastMadgwick::updateIMU` signature to accept `float dt` and remove the internal `dt = 1.0f / sampleFreq`.
  2. Track microsecond timestamps around each cached MPU/BNO read in the polling task or timer:
     ```cpp
     static uint32_t last_us = micros();
     uint32_t now_us = micros();
     float dt = (now_us - last_us) * 1e-6f;
     last_us = now_us;
     madgwick_filter.updateIMU(gx, gy, gz, ax, ay, az, dt);
     ```
  3. Provide a safe fallback `dt` clamp (e.g., `dt = min(max(dt, 1e-4f), 0.05f)`) to avoid spikes on wakeups.

### 3) Add a fast-path to `QuaternionTracker` to skip heavy math when stable
- **Problem**: Each call performs SLERP and gyro prediction even when quaternions barely change, costing multiple trig ops per stream.  
  **Location**: `esp32_imu_master/src/sensor_fusion.h:414-515`.
- **Implementation**:
  1. Compute `dot = last_output_quat.dot(q);` early; if `dot > 0.9998f` (~0.8° delta), set `last_output_quat = q` and return (respecting warmup). Skip SLERP/prediction.
  2. Only run `predictFromGyro` and `angErr` when the delta exceeds that threshold.
  3. Apply to all trackers (IMU1, IMU2, fused, single-mode).

### 4) Reuse a single micro-ROS timestamp per timer iteration
- **Problem**: Multiple `rmw_uros_epoch_millis()` calls per cycle add overhead in the hot path.  
  **Location**: `esp32_imu_master/src/main.cpp:601-616, 668-682, 714-730, 816-840`.
- **Implementation**:
  1. Capture `epoch_millis` once at the start of `timer_callback`.
  2. Add a helper that sets header stamp and frame_id using the cached timestamp for all messages (IMU1/IMU2/fused/single, and odom when enabled).

### 5) Remove blocking Serial prints from real-time paths
- **Problem**: Calibration and altitude baseline logs occur inside position integration, blocking the 100 Hz loop.  
- **Location**: `esp32_imu_master/src/main.cpp:337-409`.
- **Implementation**:
  1. Guard those `Serial.println` statements behind a compile-time flag (e.g., `#if VERBOSE_CALIB_LOGS`) or queue them to a non-realtime status task.
  2. Keep the integration loop free of Serial I/O.

### 6) Host-side fusion should gate on timestamp skew
- **Problem**: IMU1/IMU2 fusion averages data without checking stamp alignment, potentially mixing stale samples.  
- **Location**: `microros_ws/src/imu_visualization/imu_visualization/imu_fusion_node.py:114-181`.
- **Implementation**:
  1. Track the last fused stamp; only fuse when both messages are newer and within a skew window (e.g., ≤20 ms).
  2. Optionally use `message_filters.ApproximateTime` to synchronize before averaging velocities/accelerations.

---

## Suggested Execution Order
1. **Immediate**: Move sensor I/O out of the timer and reuse a single timestamp (items 1 & 4). Add `dt` support to Madgwick (item 2).
2. **Short-term**: Add the fast-path to `QuaternionTracker` (item 3) and remove Serial prints in the hot path (item 5).
3. **Host-side polish**: Add timestamp gating to Python fusion (item 6).

---

## Validation Checklist
- Wrap `timer_callback` with `micros()` logging (debug build) to confirm sub-10 ms execution after refactors.
- Measure publish rate over 60 s with a ROS 2 subscriber; target ≥50 Hz with minimal jitter.
- Verify Madgwick output stability by comparing orientation drift before/after `dt` fix under deliberate cycle-time variation.
- Confirm fused stream does not stall when one sensor is temporarily stale (cache validity checks). 

---

## Reference Paths
- Firmware timer and I2C usage: `esp32_imu_master/src/main.cpp:520-841`
- Quaternion smoothing: `esp32_imu_master/src/sensor_fusion.h:337-515`
- Madgwick filter math: `esp32_imu_master/src/sensor_fusion.h:284-358`
- Position integration logs: `esp32_imu_master/src/main.cpp:337-409`
- Host fusion node: `microros_ws/src/imu_visualization/imu_visualization/imu_fusion_node.py:114-181`