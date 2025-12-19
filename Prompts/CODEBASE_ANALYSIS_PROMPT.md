# Codebase Analysis Prompt for IMU Tele-Imitation System

## Your Task

You are an expert embedded systems and robotics engineer analyzing a complete codebase for an IMU-based tele-imitation system. Your goal is to examine **all code files** in the repository and provide comprehensive, actionable suggestions formatted specifically for use in **Cursor IDE**.

## Project Context

This is an ESP32-based IMU system that:
- Captures hand orientation (roll, pitch, yaw) in real-time using BNO055 and/or MPU6050 sensors
- Transmits data wirelessly via WiFi using micro-ROS to a ROS 2 host
- Targets 50+ Hz publish rate for real-time robot control
- Has performance issues: code is slower than when the project started

**Key Problem**: The system performance has degraded over time. It was faster initially but is now too slow for real-time operation.

## Repository Structure

The codebase includes:

### ESP32 Firmware (`esp32_imu_master/`)
- `src/main.cpp` - Main firmware (1133 lines, supports single/dual IMU modes)
- `src/sensor_fusion.h` - Quaternion math, sign continuity, Madgwick filter (535 lines)
- `src/imu_diagnostics.h` - Diagnostic logging system (263 lines)
- `src/main_mpu6050.cpp` - MPU6050-only test firmware (479 lines)
- `src/wifi_config.h` - WiFi configuration template
- `platformio.ini` - Build configuration with multiple environments

### ROS 2 Host Code (`microros_ws/src/imu_visualization/`)
- Python nodes for TF broadcasting, fusion, visualization
- Launch files for different configurations
- RViz configuration files

### Documentation
- `MASTER_GUIDE.md` - Complete setup and usage guide
- `CALIBRATION.md` - Sensor calibration procedures
- `project.txt` - Project description and architecture

## Analysis Requirements

Examine **ALL** code files in the repository and provide:

1. **Performance Bottleneck Analysis**
   - Identify specific code sections causing slowdowns
   - Quantify impact where possible
   - Compare current implementation with best practices

2. **Code Quality Assessment**
   - Architecture issues
   - Redundant operations
   - Memory management problems
   - Potential bugs or race conditions

3. **Optimization Opportunities**
   - I2C communication patterns
   - Quaternion processing efficiency
   - micro-ROS overhead
   - Task scheduling and CPU utilization

4. **Specific Code Improvements**
   - Exact file paths and line numbers
   - Before/after code examples
   - Implementation guidance

## Output Format Requirements

**CRITICAL**: Format your output as a **Cursor IDE prompt** that can be directly used. Your response should:

1. **Use Markdown formatting** with clear sections
2. **Include code references** using the format: `filepath:startLine:endLine`
3. **Provide actionable suggestions** with specific file locations
4. **Include code examples** showing improvements
5. **Prioritize suggestions** (High/Medium/Low impact)
6. **Be ready to copy-paste** into Cursor's chat interface

### Output Structure

Your response should follow this structure:

```markdown
# Codebase Analysis: IMU Tele-Imitation System

## Executive Summary
[Brief overview of findings]

## Critical Performance Issues (High Priority)

### Issue 1: [Title]
**Location**: `filepath:startLine:endLine`
**Impact**: [Description]
**Current Code**:
```cpp
// Show problematic code
```
**Recommended Fix**:
```cpp
// Show improved code
```
**Expected Improvement**: [Quantify if possible]

### Issue 2: [Title]
[...]

## Code Quality Issues (Medium Priority)

### Issue 1: [Title]
[...]

## Optimization Opportunities (Low Priority)

### Issue 1: [Title]
[...]

## Architecture Recommendations

### Recommendation 1: [Title]
[...]

## Implementation Priority

1. **Immediate (Do First)**: [List items]
2. **Short-term (This Week)**: [List items]
3. **Long-term (Future)**: [List items]

## Testing Recommendations

[Suggestions for validating improvements]
```

## Specific Areas to Examine

### 1. I2C Communication (`main.cpp`)
- Analyze `read_bno055_quaternion_safe()` function
- Check `check_bno055_status()` frequency and overhead
- Evaluate I2C read patterns in dual IMU mode
- Look for bus contention or unnecessary delays

### 2. Quaternion Processing (`sensor_fusion.h`, `main.cpp`)
- Examine `QuaternionTracker::process()` complexity
- Analyze SLERP calculations in `fuse_quaternions()`
- Check `FastMadgwick::updateIMU()` efficiency
- Evaluate gyro prediction overhead

### 3. Timer Callback (`main.cpp::timer_callback()`)
- Measure execution time of callback
- Check for blocking operations
- Analyze dual IMU mode sequential reads
- Evaluate message preparation overhead

### 4. micro-ROS Integration (`main.cpp`)
- Check executor spin time
- Analyze message serialization
- Evaluate WiFi transmission efficiency
- Check QoS settings impact

### 5. Build Configuration (`platformio.ini`)
- Review compiler flags
- Check optimization levels
- Evaluate feature flag impact
- Assess library dependencies

### 6. ROS 2 Host Code (`microros_ws/src/imu_visualization/`)
- Analyze Python node efficiency
- Check TF broadcasting overhead
- Evaluate fusion node performance
- Review launch file configurations

### 7. Memory Management
- Check for heap fragmentation
- Analyze stack usage
- Evaluate static vs dynamic allocation
- Check for memory leaks

### 8. Task Scheduling
- Analyze FreeRTOS task priorities
- Check WiFi stack interference
- Evaluate interrupt handling
- Review CPU core utilization

## Performance Metrics to Consider

When analyzing, consider:
- **Target**: 50+ Hz publish rate (20ms period)
- **Current**: Likely <50 Hz (needs measurement)
- **I2C Read Time**: ~5-10ms per sensor
- **Timer Period**: 10ms (100 Hz target)
- **WiFi Latency**: Variable, depends on network

## Code Patterns to Look For

1. **Redundant Operations**: Same calculation done multiple times
2. **Unnecessary Copies**: Quaternion/vector copies that could be references
3. **Blocking Calls**: `delay()`, `Serial.print()` in hot paths
4. **Inefficient Algorithms**: O(n²) where O(n) is possible
5. **Memory Allocations**: Dynamic allocation in timer callback
6. **String Operations**: `sprintf()`, `String` class in performance-critical code
7. **Floating Point**: Excessive trigonometric functions
8. **Conditional Compilation**: `#ifdef` checks in hot paths

## Questions to Answer

1. What specific code changes likely caused the performance degradation?
2. Which functions are called most frequently and could be optimized?
3. Are there any obvious algorithmic inefficiencies?
4. Could the ESP32's second core be utilized better?
5. Are there compiler optimizations not being used?
6. Is the quaternion processing pipeline optimal?
7. Could I2C reads be batched or parallelized?
8. Are there unnecessary validations or checks in hot paths?
9. Could the code structure be simplified to reduce overhead?
10. What would be the highest-impact single optimization?

## Additional Context

- **Hardware**: ESP32 (WEMOS D1 R32), BNO055 (I2C 0x28), MPU6050 (I2C 0x68)
- **I2C Speed**: 400 kHz (fast mode)
- **WiFi**: UDP transport via micro-ROS
- **ROS 2**: Jazzy Jalisco
- **Framework**: Arduino on PlatformIO

## Deliverables

Provide a comprehensive analysis document formatted for Cursor IDE that includes:

1. ✅ **Specific file paths and line numbers** for all issues
2. ✅ **Before/after code examples** showing improvements
3. ✅ **Prioritized action items** (High/Medium/Low)
4. ✅ **Quantified impact estimates** where possible
5. ✅ **Implementation guidance** for each suggestion
6. ✅ **Testing recommendations** to validate improvements

## Important Notes

- **Be specific**: Use exact file paths and line numbers
- **Be actionable**: Provide code examples, not just descriptions
- **Be prioritized**: Focus on high-impact changes first
- **Be practical**: Consider ESP32 constraints and real-world limitations
- **Be comprehensive**: Examine all files, not just main.cpp

---

**Begin your analysis now. Examine all code files and provide your findings in the specified Cursor IDE format.**

