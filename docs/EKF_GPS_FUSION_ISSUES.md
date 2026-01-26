# EKF/GPS Fusion Issues

This document tracks known issues with the EKF and GPS fusion system discovered during debugging. These should be investigated and addressed to improve system reliability.

## Issue 1: GPS Course vs EKF Yaw Reference Frame Mismatch

**Severity:** High
**Status:** Workaround applied (heading check removed from loop detection)
**Date discovered:** 2026-01-25

### Description

The `heading` field in the telemetry packet switches between two incompatible reference frames depending on vehicle speed:

```javascript
// websocket_server.rs - parseBinaryTelemetry()
heading: spd > 1.0 ? gps_course : yaw  // PROBLEM: different reference frames!
```

- **GPS Course:** Absolute bearing (0 = North, increases clockwise), valid only when moving
- **EKF Yaw:** Relative to heading at boot time (arbitrary reference)

### Evidence from CSV Analysis

Loop closure at rows 459-531 in `1_blackbox_2026-01-26-00-42-55.csv`:
- GPS course difference: **2.5°** (correct - vehicle facing same direction)
- EKF yaw difference: **155°** (wrong reference frame)

The vehicle completed a loop and was facing the same direction, but the EKF yaw showed a massive difference because it's measured from an arbitrary boot-time reference.

### Impact

- Loop detection failed because heading check compared start heading (captured as GPS course when speed > 1 m/s) with current heading (EKF yaw when stopped/slow)
- Any feature relying on absolute heading will have similar issues

### Workaround Applied

Removed heading check from loop detection entirely. Now uses only:
- Distance traveled >= 150m
- Proximity to start < 12m

### Proper Fix Options

1. **Align EKF yaw to GPS course during warmup:**
   - When GPS warmup completes and speed is sufficient, capture GPS course
   - Set EKF yaw offset so `ekf_yaw + offset = gps_course`
   - All future yaw values are now in absolute reference frame

2. **Always use GPS course for heading when available:**
   - Store last valid GPS course with timestamp
   - Only fall back to EKF yaw if GPS course is stale (>1-2 seconds)
   - Requires tracking GPS course validity/staleness

3. **Separate fields in telemetry:**
   - `gps_course`: Raw GPS course (NaN when invalid/stationary)
   - `ekf_yaw`: EKF yaw (always valid, relative reference)
   - Let consumers decide which to use

**Recommended:** Option 1 - Align EKF yaw to absolute reference during GPS warmup. This makes the entire system work in a consistent reference frame.

---

## Issue 2: EKF Position Explosion

**Severity:** Critical
**Status:** Partially addressed (checksum validation + sanity checks added)
**Date discovered:** 2026-01-25

### Description

The EKF position estimates occasionally "explode" - jumping hundreds or thousands of meters in a single timestep. This corrupts:
- Position tracking
- Distance calculations
- Track recording
- Any downstream processing

### Evidence from CSV Analysis

`1_blackbox_2026-01-26-00-51-33.csv`:
- 1890 lines of data
- **519 position jumps > 100m** (27% of samples!)
- Maximum jump: thousands of meters

`1_blackbox_2026-01-26-00-42-55.csv`:
- 4107 lines of data
- **172 position jumps > 100m** (4% of samples)

### Root Causes Identified

1. **Corrupted GPS data accepted by EKF:**
   - NMEA sentences with checksum errors were being processed
   - Invalid positions were fed directly to EKF measurement update

2. **No sanity checks on GPS measurements:**
   - Position jumps > 1000m accepted
   - Speed > 100 m/s accepted (360 km/h!)
   - These should be rejected as clearly invalid

### Fixes Applied

1. **NMEA checksum validation** (ublox-gps/lib.rs):
   - Verify XOR checksum before accepting any NMEA sentence
   - Reject sentences with checksum mismatch

2. **GPS position sanity checks** (main.rs):
   - Reject position jumps > 1000m from last valid position
   - Reject reported speed > 100 m/s
   - Log rejected measurements for debugging

### Remaining Concerns

1. **EKF process noise tuning:**
   - Q_ACC = 0.40 may be too high, allowing rapid state changes
   - Should the EKF reject measurements that cause large innovations?

2. **Innovation gating:**
   - EKF should reject measurements where innovation > k * sqrt(S)
   - Currently no gating implemented - all measurements accepted

3. **Position uncertainty growth:**
   - Between GPS updates, position uncertainty grows unbounded
   - Should cap maximum uncertainty or trigger recovery mode

4. **Recovery from divergence:**
   - Once EKF diverges, it may never recover
   - Need detection (innovation monitoring) and recovery (state reset) logic

### Suggested Improvements

```rust
// Innovation gating in EKF update
let innovation = z - H * x;  // Measurement residual
let S = H * P * H' + R;      // Innovation covariance
let mahalanobis = innovation.t() * S.inv() * innovation;
if mahalanobis > CHI2_THRESHOLD {
    // Reject measurement, don't update state
    return;
}
```

---

## Issue 3: pos_sigma Not Reflecting True Uncertainty

**Severity:** Medium
**Status:** Needs investigation
**Date discovered:** 2026-01-25

### Description

The `pos_sigma` field was added to track EKF position uncertainty:
```rust
packet.pos_sigma = (estimator.ekf.p[0] + estimator.ekf.p[1]).sqrt();
```

However, when the EKF explodes, `pos_sigma` may not accurately reflect the true uncertainty because:
- The EKF covariance matrix `P` is updated assuming the model is correct
- If the state is wrong, the covariance may still appear "confident"

### Impact

- GPS quality indicator may show "Good" or "Excellent" even when position is wildly wrong
- Cannot rely on pos_sigma alone for quality assessment

### Suggested Improvements

1. **Track innovation statistics:**
   - Monitor running average of normalized innovations
   - Large innovations indicate model/measurement mismatch

2. **Cross-check with GPS:**
   - Compare EKF position with raw GPS position
   - Large discrepancy should increase reported uncertainty

3. **Uncertainty floor:**
   - Never report pos_sigma below GPS accuracy (~2-3m for M9N)
   - EKF can smooth but can't know position better than GPS

---

## Issue 4: Yaw Drift When Stationary

**Severity:** Low
**Status:** Partially addressed by ZUPT
**Date discovered:** Previously known

### Description

When the vehicle is stationary, the EKF yaw estimate can drift due to:
- Gyro bias not fully estimated
- Magnetometer interference (parked near metal objects)
- No absolute yaw reference when stopped

### Current Mitigation

- ZUPT (Zero Velocity Update) resets velocity to zero when stationary
- Yaw rate is zeroed during ZUPT
- But yaw angle itself is not corrected

### Suggested Improvements

1. **Magnetometer yaw update when stationary:**
   - Use magnetometer as absolute reference when not moving
   - Weight by magnetic field strength consistency

2. **GPS course memory:**
   - Remember last valid GPS course before stopping
   - Use as prior when starting to move again

---

## Testing Recommendations

### Unit Tests Needed

1. **Reference frame alignment test:**
   - Verify GPS course and EKF yaw converge to same value after warmup
   - Test with simulated drive in known heading

2. **Innovation gating test:**
   - Feed outlier measurements to EKF
   - Verify they are rejected and state remains stable

3. **Position explosion detection test:**
   - Simulate GPS dropout followed by invalid measurement
   - Verify system detects and handles gracefully

### Field Tests Needed

1. **Loop detection reliability:**
   - Drive multiple loops of varying sizes
   - Record success/failure rate of loop closure detection

2. **Long-duration stability:**
   - 30+ minute drive
   - Monitor pos_sigma and position accuracy over time

3. **Urban canyon performance:**
   - Drive through areas with tall buildings
   - Monitor GPS rejection rate and EKF stability

---

## References

- ArduPilot EKF documentation: https://ardupilot.org/dev/docs/ekf2-estimation-system.html
- GPS accuracy characteristics: NEO-M9N datasheet, CEP specifications
- ZUPT implementation: `main.rs:422-432`
- EKF implementation: `ekf.rs`
- Sensor fusion: `fusion.rs`
