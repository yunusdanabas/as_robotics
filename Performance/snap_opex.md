# ESP32 + BNO055 Yaw Snap Research Report

## System Recap
- Hardware/transport: ESP32 (PlatformIO/Arduino) with BNO055 at I2C address 0x28, 400 kHz, publishing IMUPLUS quaternions at 50 Hz over micro-ROS WiFi (UDP 8888) as `sensor_msgs/Imu`.
- Processing: `QuaternionTracker` handles hemisphere alignment, smoothing, and glitch detection before publishing; hemisphere alignment applied to raw quaternions first; status checks every 10 s; diagnostic logging optional.

## 1) Root Cause Analysis (teleportation/yaw snap)
### A. Quaternion representation ambiguity
- Quaternions are double-covered (q ≡ −q); any sign flip without continuity handling appears as a 180° jump. Hemisphere alignment fails if the reference is stale (warmup/reset), the incoming quaternion is noisy near a sign boundary (dot≈0), or when fusion restarts and re-seeds orientation. Rapid rotations near 180°/s can also push the tracker into the long path if the gyro prediction is inaccurate.
- Best practices: always align new samples to the last published quaternion (not last raw), apply dot-product thresholding with epsilon to avoid chattering near 0, and debounce resets so that a fusion reset triggers tracker reset.

### B. Sensor fusion artifacts (BNO055 IMUPLUS)
- In IMUPLUS, yaw is unconstrained and will drift; Bosch notes that gyroscope bias updates can cause state discontinuities when the internal EKF re-linearizes. Community reports show occasional quaternion sign flips and sudden yaw corrections after self-test or calibration flag changes when the magnetometer is disabled.
- The IMUPLUS mode is more sensitive to gyro bias jumps and FIFO flushes; power supply droop or brown-out can cause the fusion core to restart without an obvious status error.

### C. I2C communication integrity
- At 400 kHz, marginal wiring, level shifting, or long cables can create partial reads; a single corrupted byte in the 8-byte quaternion can reflect to a sign flip or large yaw delta. Clock stretching by the BNO055 during fusion updates can also cause the ESP32 controller to sample mid-byte if bus pull-ups are weak.
- Mitigations: repeated reads with CRC-like validation (norm window, monotonicity checks), bus-level monitoring (NAK counts, timeout stats), and reducing clock to 100–200 kHz on noisy layouts.

### D. ROS 2 / micro-ROS transport
- micro-ROS over WiFi/UDP is unordered; however, if TF/Euler conversion treats q and −q differently, out-of-order packets can show as jumps. QoS with best-effort & depth=1 can drop samples; missing intermediate samples can make an otherwise smooth rotation appear discontinuous if visualization unwraps yaw naïvely. Ensure downstream consumers apply sign continuity and reject older timestamps.

## 2) Literature Review (highlights)
- Quaternion filtering: Madgwick (2010) and Mahony (2008) complementary filters provide continuity if sign alignment is enforced each step; multiplicative EKF formulations (Markley 2003) handle quaternion normalization gracefully and allow bias estimation. SLERP/ squad smoothing offers C1 continuity; quaternion averaging with sign-consistent samples reduces jerk.
- Outlier rejection: gyro-prediction residual gating (chi-square), z-score on angular velocity increments, and Huber loss on innovation have been used in wearable IMU pipelines.
- Real-time IMU systems: low-latency pipelines favor fixed-point or fused multiply-add implementations; publish jitter below 5–10 ms is recommended for tele-operation comfort.
- BNO055 documentation and community threads report mode-switch discontinuities, calibration flag glitches, and recommend periodic status polling plus external filtering when magnetometer is disabled.

## 3) Solution Catalog
- Sign continuity & smoothing: always-on SLERP toward the last output with short horizon; epsilon-guarded dot tests; warm-start using last good quaternion after resets; quaternion deadband near sign boundary to avoid oscillation.
- Predictive filtering: propagate orientation with gyro integration and gate incoming samples by prediction error (already partially implemented); adaptive thresholds based on angular velocity magnitude to avoid false positives during aggressive motion.
- Glitch handling: freeze-and-fade (hold last good, then SLERP to new) vs. reject-and-reset (drop sample, reset tracker if N consecutive glitches). Statistical detectors (MAD/z-score) for angular increments; physics-based jerk bounds for tele-imitation.
- Diagnostics: per-sample validity flags, repetition detection (stuck I2C), and calibration state logging with timestamps; bus health metrics (I2C error counters, timing).
- Hardware/comm: reduce I2C rate on marginal wiring, add pull-ups ~2.2–4.7 kΩ, shielded twisted pairs, or switch to SPI/alternative IMU; WiFi QoS (reliable transport or sequence numbers) and timestamp monotonicity checks.

## 4) Codebase Findings (current implementation)
- Quaternion tracking uses sign continuity, gyro-based prediction gating (0.25 rad), warmup, and two-stage smoothing (~300 ms glitch smoothing, always-on α=0.15).【F:esp32_imu_master/src/sensor_fusion.h†L145-L238】
- Glitch mode freezes output for one frame then slowly SLERPs to the target; prediction error stored for diagnostics; without gyro, fallback threshold is 3× and more lenient.【F:esp32_imu_master/src/sensor_fusion.h†L200-L275】
- Diagnostic logging captures raw vs processed quaternions, norms, dot products, smoothing state, repetition detection, and calibration status; logging interval configurable; repetition flagged after 5 identical packets.【F:esp32_imu_master/src/imu_diagnostics.h†L25-L140】
- BNO055 reads include retries, norm checks, and NaN/all-zero detection; status polling is non-blocking every 10 s with a grace period and persistent-error counter to avoid spamming failures.【F:esp32_imu_master/src/main.cpp†L52-L140】【F:esp32_imu_master/src/main.cpp†L160-L234】

## 5) Improvement Proposals
### Immediate (low risk)
- Add explicit sequence numbers and timestamps in outgoing messages; ensure downstream nodes realign quaternion signs before Euler conversion.
- Tighten hemisphere alignment by using the last published quaternion and adding a small dot epsilon (e.g., 1e-3) to avoid flipping near zero; reset the tracker when `check_bno055_status` reports consecutive errors or when calibration state changes.
- Expand diagnostics to log prediction error histograms and I2C retry counts; expose build flags for I2C speed reduction and log current speed at boot.

### Algorithmic (medium risk)
- Adaptive glitch threshold: scale the 0.25 rad gate with |ω| (e.g., threshold = base + k·|ω|) to reduce false positives during fast motion; shorten glitch smoothing duration when motion is mild.
- Incorporate quaternion innovation filtering: use a multiplicative EKF or Mahony/Madgwick as a secondary check that fuses gyro+accel and gates BNO055 output.
- Dual-IMU fusion: weight by innovation consistency and temperature; implement outlier voting before SLERP fusion to prevent one IMU from injecting flips.

### Architectural (higher effort)
- Move to an IMU with higher-rate, lower-drift gyros (ICM-20948, ICM-42688, LSM6DSOX) and run host-side fusion (Mahony/Madgwick/EKF) with explicit bias estimation.
- Switch to SPI (if hardware allows) or dedicated IMU coprocessor; add power supervision (brown-out detector, RC reset) to prevent fusion restarts.
- Transport robustness: add sequence checking and reordering on the ROS 2 host; consider ESP-NOW for deterministic latency or BLE 5.0 Coded for interference-heavy spaces.

### Research gaps / future work
- Characterize BNO055 IMUPLUS yaw drift and reset behavior under varying temperatures and supply dips; log internal calibration flags vs. teleportation frequency.
- Evaluate adaptive thresholds vs. fixed 0.25 rad in tele-imitation latency budgets; collect user comfort metrics.
- Hardware-in-loop bench to compare I2C 100/200/400 kHz with different cable lengths and pull-ups.

## 6) Future Evolution & Enhancement
- Teleportation resilience: multi-stage filter (prediction → gating → SLERP smoothing), motion-model-based yaw prediction, and optional magnetometer reintroduction with hard/soft-iron calibration and interference rejection.
- System architecture: expand dual-IMU fusion to body networks (per-limb nodes) with time-sync (PTP/NTP-lite) and per-link QoS; edge inference for gesture detection.
- Transport/performance: evaluate higher publish rates (100–200 Hz) with DMA I2C and zero-copy micro-ROS buffers; dynamic rate adaptation based on network quality.
- Tooling: OTA firmware updates, remote diagnostics dashboard, rosbag2 capture pipeline with automated anomaly tagging, and calibration wizards.
- Application expansion: tele-operation/tele-imitation avatars, VR/AR head/hand tracking, sports/rehab analytics, industrial safety monitors.

## 7) Technology Assessment (upgrade options)
- Sensors: ICM-42688 (low noise, good bias stability), ICM-20948 (9-DOF with DMP), LSM6DSOX (embedded finite state machine/ML), BMI270 (low power), BNO086/BSX fusion co-processors for on-sensor EKF.
- Fusion software: open-source Madgwick/Mahony with bias estimation; OpenIMU/RTIMULib for EKF; on-host ukf/iekf (e.g., `robot_localization`) for richer state.
- Communication: ESP-NOW for peer-to-peer low-latency, BLE 5.2 for coexistence, or wired RS485/CAN for industrial robustness.

## 8) Solution Catalog (pros/cons)
- Freeze-and-fade smoothing: simple and low CPU; risk of lag during true fast motions.
- Adaptive gating with gyro prediction: robust to glitches; needs accurate dt and gyro calibration.
- Sequence-numbered micro-ROS messages with host-side reordering: fixes transport-induced jumps; adds minor bandwidth overhead.
- I2C down-clocking + stronger pull-ups: easy hardware fix; reduces throughput headroom but 50 Hz still well within limits.

## 9) Roadmap (prioritized)
1. Add host-side and firmware-side sign alignment + sequence numbering; log calibration transitions and I2C retry counts (immediate).
2. Implement adaptive glitch gating and shorter smoothing with motion-aware thresholds; expose tunables via build flags (near-term).
3. Run bench tests at multiple I2C speeds and cable conditions; pick default based on reliability (near-term).
4. Prototype dual-IMU fusion with innovation weighting and transport QoS tuning; add OTA + diagnostics dashboard (mid-term).
5. Evaluate next-gen IMUs (ICM-42688/LSM6DSOX) and SPI transport; decide on upgrade path (mid/long-term).
6. Expand to multi-node body networks with time-sync and cloud/edge analytics; integrate tele-imitation features (long-term).

## Bibliography / References
- Madgwick, S. “An efficient orientation filter for inertial and inertial/magnetic sensor arrays,” 2010.
- Mahony, R., et al. “Nonlinear Complementary Filters on the Special Orthogonal Group,” 2008.
- Markley, F. “Attitude error representations for Kalman filtering,” Journal of Guidance, Control, and Dynamics, 2003.
- Bosch Sensortec BNO055 Datasheet & Application Notes (fusion modes, reset behavior).
- Community/industry reports: Bosch forum threads on BNO055 fusion resets and quaternion flips in IMUPLUS; micro-ROS documentation on WiFi transport and QoS.
