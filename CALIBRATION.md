# BNO055 IMU Sensor Calibration Guide

The BNO055 sensor performs automatic calibration, but understanding the process helps achieve better accuracy.

## Why Calibration Matters

The BNO055 contains three sensors that need calibration:
- **Accelerometer**: Measures linear acceleration
- **Magnetometer**: Measures magnetic field (for absolute orientation)
- **Gyroscope**: Measures angular velocity

Without calibration, orientation data may drift or be inaccurate.

## Calibration Status

The BNO055 provides calibration status for each sensor (0-3):
- **0**: Uncalibrated
- **1**: Minimally calibrated
- **2**: Moderately calibrated
- **3**: Fully calibrated

## Automatic Calibration Procedure

### Initial Setup

1. **Power on the sensor** in a magnetically clean environment
2. **Wait 5-10 seconds** for initial stabilization
3. **Avoid metal objects** and electronics nearby

### Gyroscope Calibration

**Goal**: Calibrate for rotational accuracy

**Procedure**:
1. Place the IMU on a **stable, flat surface**
2. **Do not move it** for 5-10 seconds
3. The gyroscope will self-calibrate

**Status**: Usually calibrates quickly (reaches 3)

### Accelerometer Calibration

**Goal**: Calibrate for gravity direction

**Procedure**:
1. Place IMU in **6 different orientations**:
   - Flat (Z-axis up)
   - Upside down (Z-axis down)
   - On each edge (X-axis up, down, Y-axis up, down)
2. Hold each position **steady for 3-5 seconds**
3. Move smoothly between positions

**Status**: Reaches 3 after all 6 orientations

### Magnetometer Calibration

**Goal**: Calibrate for absolute heading

**Important**: Most challenging and environment-dependent

**Procedure**:
1. Move **away from metal objects**, magnets, and electronics
2. Perform **figure-8 motions** in 3D space:
   - Large sweeping figure-8 patterns
   - Rotate through all orientations
   - Continue for 30-60 seconds
3. Include some **slow 360° rotations** around each axis

**Environmental Factors**:
- Nearby steel/iron structures affect calibration
- Computer cases, USB hubs can interfere
- Best done outdoors or in center of room

**Status**: Takes longest to reach 3

## Monitoring Calibration (Advanced)

To check calibration status, you can modify the firmware to publish calibration data.

### Add to `main.cpp` (in setup function):

```cpp
// In setup(), after bno.begin()
uint8_t system, gyro, accel, mag;
system = gyro = accel = mag = 0;
bno.getCalibration(&system, &gyro, &accel, &mag);

Serial.print("Calibration: Sys=");
Serial.print(system, DEC);
Serial.print(" Gyro=");
Serial.print(gyro, DEC);
Serial.print(" Accel=");
Serial.print(accel, DEC);
Serial.print(" Mag=");
Serial.println(mag, DEC);
```

### In loop():

```cpp
// Check calibration every 5 seconds
static unsigned long last_calib_check = 0;
if (millis() - last_calib_check > 5000) {
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.printf("Calib: Sys=%d G=%d A=%d M=%d\\n", system, gyro, accel, mag);
  last_calib_check = millis();
}
```

## Quick Calibration Routine

**For Daily Use**:

1. **Power on** and wait 10 seconds
2. **Place flat** on table (gyro calibration)
3. **Tilt slowly** through 6 orientations (accel calibration)
4. **Figure-8 motions** in air for 30 seconds (mag calibration)
5. **Verify** smooth orientation changes

**Expected Time**: 1-2 minutes

## Calibration Profiles

The BNO055 can save and restore calibration:

### Save Calibration (Advanced)

```cpp
adafruit_bno055_offsets_t calibrationData;
bool calibrated = false;

if (bno.isFullyCalibrated()) {
  bno.getSensorOffsets(calibrationData);
  // Save to EEPROM or file
  calibrated = true;
}
```

### Restore Calibration

```cpp
if (calibrated) {
  bno.setSensorOffsets(calibrationData);
}
```

## Signs of Poor Calibration

- **Orientation drifts** over time
- **Sudden jumps** in quaternion values
- **Compass heading** doesn't match physical direction
- **Erratic behavior** when rotating

## Calibration for Different Use Cases

### For Orientation Only (No Magnetometer)

If you don't need absolute heading:

```cpp
// Use IMU fusion mode (no magnetometer)
bno.setMode(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
```

**Advantages**:
- Faster calibration
- Less environmental interference
- More stable in indoor environments

**Disadvantages**:
- No absolute heading
- Yaw will drift over time

### For Full 9-DOF (With Magnetometer)

Default mode uses all sensors:

```cpp
// NDOF mode (uses magnetometer)
bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
```

## Troubleshooting Calibration Issues

### Magnetometer Won't Calibrate

**Problem**: Mag status stuck at 0 or 1

**Solutions**:
1. Move away from computer and electronics
2. Go outdoors
3. Perform larger figure-8 motions
4. Check for nearby metal objects
5. Consider using IMU mode instead

### Calibration Resets on Power Cycle

**Problem**: Sensor needs recalibration every time

**Solutions**:
1. Implement calibration profile storage
2. Save offsets to ESP32 EEPROM
3. Restore offsets on startup

### Gyroscope Won't Stabilize

**Problem**: Gyro status low, values jittery

**Solutions**:
1. Ensure sensor is completely still during boot
2. Wait longer for initial calibration
3. Check sensor mounting (vibrations?)
4. Verify I2C connections

## Testing Calibration Quality

### Simple Test

1. **Place IMU flat** on table
2. **Note the quaternion** values in RViz2
3. **Rotate 90° around Z-axis** (yaw)
4. **Check if rotation is ~90°** in visualization
5. **Return to start position**
6. **Verify** original orientation is restored

### Advanced Test

1. **Use a smartphone gyroscope** app as reference
2. **Compare orientations** side-by-side
3. **Test all three axes**:
   - Roll (X-axis rotation)
   - Pitch (Y-axis rotation)
   - Yaw (Z-axis rotation)

## Calibration Best Practices

1. **Calibrate in the environment** where you'll use the sensor
2. **Avoid metal surfaces** during calibration
3. **Keep sensor away** from motors, magnets, current-carrying wires
4. **Perform calibration** before each session (or save/restore profiles)
5. **Be patient** - full calibration takes 1-2 minutes

## Calibration Quality Targets

**Acceptable for Testing**:
- System: ≥ 1
- Gyro: 3
- Accel: ≥ 2
- Mag: ≥ 1

**Good for Development**:
- System: ≥ 2
- Gyro: 3
- Accel: 3
- Mag: ≥ 2

**Optimal for Production**:
- System: 3
- Gyro: 3
- Accel: 3
- Mag: 3

## Next Steps After Calibration

Once calibrated:
1. **Test in RViz2** - Verify smooth visualization
2. **Record baseline** - Save known good calibration
3. **Test motion paths** - Try the movements you'll actually use
4. **Verify repeatability** - Check if movements are reproducible

---

**Remember**: The BNO055 is smart - it calibrates itself during normal use. Just give it the right motions and environment!

