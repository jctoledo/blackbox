# Sensor Fusion Technical Guide

This document explains how Blackbox combines GPS and IMU data to provide accurate, real-time vehicle dynamics. Understanding these concepts helps with troubleshooting, tuning, and extending the system.

---

## Why Sensor Fusion?

Neither GPS nor IMU alone provides what we need:

| Sensor | Strengths | Weaknesses |
|--------|-----------|------------|
| **GPS** | Accurate position, no drift | Slow (5-25 Hz), dropouts in tunnels |
| **IMU** | Fast (200 Hz), responsive | Drifts over time, affected by vibration |

**Solution:** Combine them. Use IMU for responsive, high-rate updates. Use GPS to correct drift. Result: fast AND accurate.

---

## The 7-State Extended Kalman Filter

The EKF is the core algorithm. It maintains estimates of 7 values (the "state vector") and continuously refines them as new sensor data arrives.

### State Vector

```
x = [x, y, ψ, vx, vy, bax, bay]
     │  │  │   │   │   │    │
     │  │  │   │   │   │    └─ Y-axis accelerometer bias (m/s²)
     │  │  │   │   │   └────── X-axis accelerometer bias (m/s²)
     │  │  │   │   └────────── Velocity Y component (m/s)
     │  │  │   └────────────── Velocity X component (m/s)
     │  │  └───────────────── Heading/yaw angle (radians)
     │  └──────────────────── Position Y (meters, local frame)
     └─────────────────────── Position X (meters, local frame)
```

### What Each State Represents

**Position (x, y)**
- Location in meters relative to where GPS first locked
- X is approximately east-west, Y is approximately north-south
- Origin resets each time the device powers on

**Heading (ψ)**
- Direction the vehicle is pointing, in radians
- 0 = East, π/2 = North, π = West, -π/2 = South
- Used to rotate accelerations into vehicle frame for mode detection

**Velocity (vx, vy)**
- Speed broken into X and Y components
- Combined: `speed = sqrt(vx² + vy²)`
- Used for position prediction between GPS updates

**Accelerometer Biases (bax, bay)**
- Systematic errors in the IMU readings
- Learned when stationary (ZUPT)
- Subtracted from raw accelerometer values

---

## Predict-Update Cycle

The EKF alternates between two steps:

### Predict (IMU, 200 Hz)

When new IMU data arrives:

1. Read accelerometer values (ax, ay)
2. Subtract learned biases: `ax_corrected = ax - bax`
3. Remove gravity using roll/pitch angles
4. Transform to earth frame (horizontal plane)
5. Update velocity: `vx += ax_earth × dt`
6. Update position: `x += vx × dt + 0.5 × ax_earth × dt²`
7. Update heading using gyroscope yaw rate
8. Increase uncertainty (covariance grows)

**Key insight:** Each predict step makes the estimate slightly less certain because we're integrating noisy sensors.

### Update (GPS, 5-25 Hz)

When new GPS data arrives:

1. Compare GPS position/velocity to current estimate
2. Calculate "innovation" (difference)
3. Compute Kalman gain (how much to trust GPS vs estimate)
4. Adjust state toward GPS measurement
5. Reduce uncertainty (covariance shrinks)

**Key insight:** GPS updates "pull" the estimate back toward truth, counteracting IMU drift.

---

## Motion Models

The EKF uses different physics models depending on speed and turn rate:

### CTRA (Constant Turn Rate and Acceleration)

Used when: `speed > 2 m/s AND |yaw_rate| > 0.0001 rad/s`

Accounts for curved motion. Position prediction follows an arc, not a straight line. Essential for accurate tracking through corners.

### CA (Constant Acceleration)

Used when: Going straight or moving slowly

Simple kinematic model:
```
x_new = x + vx × dt + 0.5 × ax × dt²
v_new = vx + ax × dt
```

---

## Coordinate Frames

Understanding coordinate frames is essential for debugging. Data flows through three frames:

### Body Frame (Sensor)

- What the IMU actually measures
- X-axis: forward (device's forward direction)
- Y-axis: left
- Z-axis: up
- Contains gravity on Z-axis

### Earth Frame (Horizontal)

- Gravity removed
- Aligned with horizontal plane
- X approximately east, Y approximately north
- Where physics calculations happen

### Vehicle Frame (Car-Aligned)

- Rotated to match vehicle heading
- X-axis: forward (car's forward direction)
- Y-axis: left (driver's left)
- Used for mode detection (accel/brake/corner)

### Transform Pipeline

```
Body Frame          Earth Frame          Vehicle Frame
(raw IMU)     →     (horizontal)    →    (car-aligned)
              │                     │
         Remove gravity        Rotate by yaw
         using roll/pitch      (EKF heading)
```

**Code locations:**
- `remove_gravity()` — `framework/src/transforms.rs`
- `body_to_earth()` — `framework/src/transforms.rs`
- Vehicle rotation — `framework/src/mode.rs`

---

## Zero-Velocity Update (ZUPT)

ZUPT is critical for preventing drift. When the vehicle is stationary, we know velocity is exactly zero — a perfect measurement.

### Detection Criteria

The system detects "stationary" when ALL conditions are met:

| Parameter | Threshold | Reason |
|-----------|-----------|--------|
| Acceleration magnitude | < 0.18g | No significant forces |
| Yaw rate | < 12°/s | Not rotating |
| GPS speed (reported) | < 3.5 km/h | GPS agrees we're stopped |
| GPS speed (position-derived) | < 5.0 km/h | Backup check |

### What Happens During ZUPT

1. **Force velocity to zero** — EKF state updated: `vx = 0, vy = 0`
2. **Learn biases** — Current acceleration reading should be zero; any non-zero value is bias
3. **Reset covariance** — Uncertainty reduced dramatically

### Why Biases Are Learned During ZUPT

When truly stationary:
- Gravity is removed (we know roll/pitch from IMU)
- Any remaining horizontal acceleration is IMU bias
- This "ground truth" allows bias estimation

### Practical Implications

- System improves at every stop (red lights, parking)
- Long drives without stops accumulate drift
- Track sessions benefit from pit stops

---

## GPS-Corrected Orientation

This is an ArduPilot-inspired technique to fix a fundamental IMU problem.

### The Problem

The IMU's AHRS (Attitude Heading Reference System) cannot distinguish linear acceleration from tilt. When you accelerate forward, the AHRS reports false pitch — it thinks the device is tilting backward.

**Result:** Gravity removal fails. Earth-frame accelerations are wrong. Mode detection is unreliable.

### The Solution

GPS velocity provides ground truth for horizontal acceleration:

```
true_acceleration = d(GPS_velocity) / dt
```

By comparing IMU-derived acceleration with GPS-derived acceleration, we can calculate orientation error:

```
pitch_error ≈ (ax_imu - ax_gps) / g
roll_error ≈ (ay_imu - ay_gps) / g
```

### Learning Process

The `OrientationCorrector` continuously refines its estimate:

1. During forward/backward acceleration → learns pitch correction
2. During left/right cornering → learns roll correction
3. Confidence grows with more observations
4. Corrections capped at ±15° for safety

### Blending Based on Confidence

| Confidence | GPS Weight | IMU Weight |
|------------|------------|------------|
| < 30% | 80% | 20% |
| 30-80% | 50% | 50% |
| > 80% | 30% | 70% |

As correction confidence grows, the system trusts the corrected IMU more.

---

## Tuning Parameters

### Process Noise (Q Matrix)

Controls how much the EKF trusts its motion model vs. sensors.

```rust
const Q_ACC: f32 = 0.40;    // Acceleration uncertainty
const Q_GYRO: f32 = 0.005;  // Gyro uncertainty
const Q_BIAS: f32 = 1e-3;   // Bias drift rate
```

**Higher Q values:**
- More responsive to sensor changes
- More noise in estimates
- Faster reaction to real motion

**Lower Q values:**
- Smoother estimates
- Slower reaction to real motion
- May miss rapid maneuvers

### Measurement Noise (R Matrix)

Controls how much the EKF trusts GPS measurements.

```rust
const R_POS: f32 = 20.0;    // GPS position uncertainty (m²)
const R_VEL: f32 = 0.2;     // GPS velocity uncertainty (m/s)²
const R_YAW: f32 = 0.10;    // Magnetometer uncertainty (rad²)
```

**Higher R values:**
- Less trust in GPS
- More reliance on IMU prediction
- Smoother but potentially drifty

**Lower R values:**
- More trust in GPS
- Jumpy when GPS is noisy
- Better long-term accuracy

### Tuning Guidelines

| Symptom | Likely Cause | Adjustment |
|---------|--------------|------------|
| Position jumps when GPS updates | R_POS too low | Increase R_POS |
| Position drifts between GPS updates | Q_ACC too high | Decrease Q_ACC |
| Sluggish mode detection | Q_ACC too low | Increase Q_ACC |
| Noisy acceleration display | Q_ACC too high | Decrease Q_ACC |
| Heading wanders | Q_GYRO too high | Decrease Q_GYRO |

---

## Diagnostics Interpretation

### IMU Rate

**Expected:** 195-200 Hz (if configured) or 10-20 Hz (factory default)

**If 0 Hz:**
- Check wiring
- Run `configure_wt901.py` to verify IMU responds
- Check baud rate configuration

**If much lower than expected:**
- Serial buffer overrun
- CPU overload

### GPS Rate

**Expected:** 5 Hz (NEO-6M) or up to 25 Hz (NEO-M9N)

**If 0 Hz:**
- Check wiring
- Ensure clear sky view
- GPS needs time for initial fix (30-60 seconds cold start)

### Position σ (Uncertainty)

Square root of position covariance. Represents 1-sigma confidence bound.

| Value | Meaning |
|-------|---------|
| < 2m | Excellent — recent GPS fix, stable |
| 2-5m | Good — normal operation |
| 5-10m | Fair — GPS intermittent or degraded |
| > 10m | Poor — GPS lost, relying on IMU prediction |

**Growth pattern:**
- Increases during GPS outages (rate depends on velocity uncertainty)
- Decreases sharply on GPS fix
- Resets to low value during ZUPT

### Velocity σ (Uncertainty)

Velocity estimate confidence.

| Value | Meaning |
|-------|---------|
| < 0.3 m/s | Excellent |
| 0.3-0.5 m/s | Good |
| 0.5-1.0 m/s | Fair |
| > 1.0 m/s | Poor |

### Bias X/Y

Learned accelerometer biases.

| Value | Meaning |
|-------|---------|
| < 0.2 m/s² | Normal — well-calibrated |
| 0.2-0.5 m/s² | Acceptable — minor offset |
| > 0.5 m/s² | High — recalibrate or check mounting |

**If growing during drive:**
- Temperature change affecting IMU
- Mounting shifted
- Vibration causing issues

### EKF/GPS Ratio

Number of EKF predictions per GPS update.

**Expected:** ~40 (200 Hz IMU / 5 Hz GPS) or ~8 (200 Hz / 25 Hz)

**If much higher:**
- GPS rate lower than expected
- GPS dropouts

**If much lower:**
- IMU rate lower than expected
- Processing bottleneck

---

## Common Issues

### Position Drift When Stationary

**Cause:** ZUPT not triggering

**Check:**
- Is the car truly stationary? Engine vibration can prevent detection
- Thresholds too tight for conditions
- GPS still reporting small speeds

**Fix:**
- Ensure engine is off for calibration
- Adjust ZUPT thresholds if needed

### Jerky Mode Detection

**Cause:** Noisy acceleration estimates

**Check:**
- IMU rate (higher = better filtering)
- Bias values (high bias = poor calibration)
- Orientation correction confidence

**Fix:**
- Recalibrate
- Check rigid mounting
- Wait for orientation correction to learn

### Position Jumps

**Cause:** GPS measurement noise being applied too aggressively

**Check:**
- R_POS value (too low = trusts GPS too much)
- GPS quality (HDOP should be < 2.0)

**Fix:**
- Increase R_POS
- Improve GPS antenna placement

---

## Code Reference

| Component | Location | Purpose |
|-----------|----------|---------|
| EKF | `framework/src/ekf.rs` | Core Kalman filter |
| Transforms | `framework/src/transforms.rs` | Coordinate frame math |
| Mode Detection | `framework/src/mode.rs` | ACCEL/BRAKE/CORNER logic |
| GPS/IMU Blending | `framework/src/fusion.rs` | Orientation correction, blending |
| Stationary Detection | `sensors/blackbox/src/main.rs` | ZUPT triggering |
| Diagnostics | `sensors/blackbox/src/diagnostics.rs` | Health monitoring |

---

## Further Reading

- [Extended Kalman Filter Explained](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/) — Visual introduction
- [ArduPilot EKF Documentation](https://ardupilot.org/dev/docs/ekf2-estimation-system.html) — Similar approach
- [IMU Noise Characteristics](https://www.analog.com/en/technical-articles/understanding-noise-analysis-allan-variance.html) — Understanding sensor noise
