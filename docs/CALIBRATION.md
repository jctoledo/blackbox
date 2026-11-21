# Horizontal mount alignment (experimental)

Mount alignment learns the vehicle's forward axis relative to a nearly level device. It changes force projection and the associated orientation-feedback axes. It does not change GPS coordinates, timing-line direction, or the EKF's device-heading basis. This is a software-tested candidate workflow; physical alignment accuracy is not established.

## Use it

1. Park on level ground and secure the sensor level; boot motionless for the separate IMU bias calibration. Wait outdoors for fresh GPS and a stationary indication.
2. Open the dashboard menu → **Mount alignment**. Start while parked. The device must have writable NVS storage.
3. At a controlled site, drive **forward**, then brake steadily in a straight line. Do not interact with the phone while moving. The screen does not tell the driver when to perform a sudden maneuver.
4. Park and review the candidate. **Save candidate** writes it before activating it. A failed write leaves the previous offset active and permits retry. **Cancel attempt** discards the candidate. **Clear saved offset** restores the default axis assumption while parked.
5. Verify acceleration/braking and left/right force signs against known motion. Repeat after remounting. Keep recordings and mounting photos for review.

The operator must establish forward travel. Scalar GPS speed and a brake vector cannot independently distinguish braking while reversing: that can yield a 180° error. Slopes, banking, sensor latency, AHRS errors and valid-looking GPS outliers remain field-test cases. The current collection gate rejects more than 10° measured roll/pitch; this is a heuristic, not slope compensation.

## Driveway slopes and startup bias

The separate startup/recalibration routine currently assumes gravity lies along sensor Z. A stationary incline can therefore be saved as accelerometer bias: a 3° tilt projects about 0.052 g onto a horizontal sensor axis. This does not damage the device, but can skew subsequent force readings. Recalibrate on level ground with a level sensor; horizontal mount alignment does not fix this limitation.

## Evidence and gates

Only new valid GPS fixes after warmup contribute, paired with a recently received IMU acceleration sample (≤60 ms) and gyro/angle packets (≤200 ms). GPS host-receive intervals must be 20–300 ms. Signed speed change must be between −10 and −1.5 m/s², with a compatible horizontal IMU magnitude. Gyro and GPS-course gates require approximately straight motion. At least 0.6 seconds and four independent qualifying GPS samples are required; duplicate samples, gaps, interrupted braking and inconsistent directions cannot complete early. An attempt expires after 60 seconds even if sensors stop.

These thresholds support the project's 5 Hz and 25 Hz configurations; packet coalescing and noise can cause a conservative rejection. Rate targets do not establish delivered rates or time synchronization. No EKF-derived speed is used to prove braking, avoiding circular evidence from the IMU being calibrated.

## Frames and persistence

Gravity is removed in body coordinates; roll/pitch are levelled before finding the brake direction. The candidate is the circular mean brake angle plus π. With offset θ meaning vehicle-forward measured counterclockwise from levelled device X, vehicle vectors use `R(−θ)`. Force projection uses device yaw + θ; the original device yaw is retained for body-to-earth transforms. Tilt-feedback residuals are rotated back into device axes before adjusting device pitch/roll.

Saving resets learned force-fusion corrections so old mounting-dependent biases do not carry over. A single 16-byte versioned NVS value contains the offset, enabled flag and CRC32. The prior PR's separate `mount_off` / `mount_ok` keys are intentionally not trusted or migrated. Invalid records fall back to an unset offset. The CRC detects corruption; it is not authentication. Actual flash power-loss behavior still needs hardware testing, and a full merged firmware installation may replace NVS configuration.

## Software regressions and physical acceptance

Regressions cover multiple mount angles, both receiver cadences, wrapped clocks, acceleration-versus-braking, missing/stale/nonfinite data, early interruption, sticky failures, persistence failure/retry, corrupt records, vehicle-force projection and orientation-feedback convergence after rotation. See [hardware acceptance](HARDWARE_VALIDATION.md) for the remaining checks.

The workflow revises Adam Weber's brake-vector mounting proposal in PR #13. The original magnitude-only detector and heading-offset integration are not used.
