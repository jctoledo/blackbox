# Sensor fusion: equations, frames and practical limits

Blackbox combines GPS and a WT901 IMU for vehicle telemetry. It has two related processing paths: an earth-frame position/velocity estimator, and a GPS/IMU acceleration pipeline for the displayed forces and driving modes. Lap timing uses receiver fixes directly, so a lagging estimator heading does not veto a crossing.

Dashboard velocity and the scalar speed supplied to the force display now prefer
fresh, warmed, accepted GPS ENU velocity, including a valid zero. When that source
is unavailable, the internal packet can fall back to a finite EKF velocity above
0.1 m/s; both sources are bounded below 100 m/s. The browser still suppresses
speed when GPS freshness is unavailable. Components and magnitude always share
one source. This replaces the older EKF-first display policy; it does not correct
the estimator's internal position, velocity or yaw, validate mounting, or make GPS
physical ground truth. Older saved recordings retain their original speed values.

## Frames and units

| Quantity | Convention |
|---|---|
| GPS latitude/longitude | Degrees; north/east positive, receiver coordinates retained as f64 for crossings |
| GPS course | Radians, clockwise from north |
| Local position and velocity | East/north metres and m/s |
| Mathematical yaw / timing-line direction | Radians, counterclockwise from east |
| WT901 roll/pitch/yaw parser output | Degrees; gyro output rad/s; acceleration m/s² |
| Vehicle acceleration | Forward/left positive; divide by 9.80665 for g |

Convert compass course to mathematical yaw with `π/2 − course` only at the relevant boundary. Recorded timing-line bearings already use `atan2(y,x)` and must not be converted again. The normal setup is upright, parallel to the car floor and nearly level on level ground. Non-horizontal mounting remains R&D requiring physical validation. An arbitrary sensor mounting/yaw offset is not automatically validated by this convention table; bench/drive checks must establish axis signs and mounting alignment. Experimental [Align sensor to car](CALIBRATION.md) provides an explicitly reviewed horizontal offset for force projection. It leaves the EKF device-heading basis and GPS timing path unchanged.

`remove_gravity` subtracts the body-frame vector `g × [−sin(pitch), sin(roll)cos(pitch), cos(roll)cos(pitch)]`. `body_to_earth` applies `Rz(yaw) Ry(pitch) Rx(roll)`. `earth_to_car` applies the inverse horizontal yaw rotation. The full gravity-free body vector, including z, belongs in that transform.

Local geographic conversion uses `north = Δlatitude × 111320` and `east = Δlongitude × 111320 × cos(reference latitude)`. This is a short-distance planar approximation, not an ellipsoidal survey conversion. It has no antimeridian/polar route support. Store the geographic origin with traces because the live local origin resets at boot.

Zero latitude or longitude is a valid coordinate, not a missing-data sentinel. Browser route capture validates finite geographic values and tracks missing GPS separately from its finite local-position fallback. Firmware-origin adoption uses the explicit warmup flag for zero-valued origins; legacy responses without that flag need nonzero coordinates, or the browser can establish an origin from validated fixes. Exact polar origins are excluded from the local projection. GPS distance does not bridge unavailable coordinate evidence.

## Estimator prediction

The state is `[x, y, ψ, vx, vy, bax, bay]`; position, velocity and learned residual biases share the earth frame. After gravity removal and rotation:

```
a = measured_earth_acceleration − learned_earth_bias
position_new = position_old + velocity_old × dt + ½ × a × dt²
velocity_new = velocity_old + a × dt
yaw_new = wrap(yaw_old + euler_heading_rate × dt)
```

The application derives two distinct angular quantities from fresh roll/pitch and all
three body gyro axes. With roll φ, pitch θ and body rates (p,q,r):

```
euler_heading_rate = (q sinφ + r cosφ) / cosθ
earth_vertical_rate = −p sinθ + q sinφ cosθ + r cosφ cosθ
```

The first advances the EKF Euler yaw. Prediction is skipped when `|cosθ| < 0.1`,
when attitude/gyro evidence is invalid or older than 200 ms, or when acceleration
is invalid/stale; no zero heading rate is fabricated. The cosine guard bounds
numerical conditioning, not a validated mounting tolerance. The second supplies
the existing approximate planar force, mode and mounting-straightness paths.
They are not interchangeable during general roll/pitch motion. Telemetry v4 `wz`
remains raw sensor-body Z rate for compatibility.

Measured lateral acceleration already produces curved motion. The current code does not switch between CTRA and CA models or add another centripetal acceleration. GPS updates correct position/velocity; stationary updates constrain velocity and learn biases. The separate orientation-correction/display path does not retroactively correct every EKF input.

For the dashboard heading, `HeadingAligner` first converts mathematical EKF yaw to clockwise compass basis with `π/2 − yaw`, then learns a residual GPS-course reference offset while moving with fresh course evidence. A constant offset alone cannot reverse the angular direction. Before learning, the output has compass turn sense but an unverified absolute reference. This display conversion does not change EKF prediction, acceleration transforms or raw-GPS crossing directions; magnetometer/mount signs and absolute heading still require the controlled bench/turn checks.

## Covariance and measurement updates

Only seven diagonal variances are retained. Prediction propagates the diagonal of `F P Fᵀ + Q` for independent position/velocity/bias states:

```
P_position += P_velocity × dt² + ¼ × (P_bias + q_acc) × dt⁴
P_velocity += (P_bias + q_acc) × dt²
P_yaw += q_gyro × dt²
P_bias += q_bias × dt
```

`q_acc` and `q_gyro` describe per-step measurement variance in the current model; `q_bias` is a variance growth rate. They are tuning values, not measured sensor noise spectra. Dropping cross-covariances still underrepresents accumulated correlations, especially over long outages. A full covariance model and measured time/noise calibration remain future work.

Defaults in `framework/src/ekf.rs`: `q_acc=0.40`, `q_gyro=0.005`, `q_bias=0.001`, `r_pos=20 m²`, `r_vel=0.2 (m/s)²`, `r_yaw=0.10 rad²`. Position uses a two-axis normalized residual gate (sum threshold 32); yaw uses a wrapped residual gate (12). Finite velocity updates deliberately remain available for recovery after drift, with receiver validity and a 100 m/s bound checked by the application. This is not a guarantee against all GPS outliers. Nonfinite measurements are rejected before state mutation.

The position diagnostic combines diagonal variance, accepted-update residual statistics, discrepancy from the last accepted GPS fix, and a 2.5 m heuristic floor. It is **not a calibrated 68% error radius**. Gating biases accepted-only residual statistics. Do not turn these values into a physical accuracy claim or tune away real failures just to make the display look healthy.

## Acceleration and orientation correction

`GpsAcceleration` differentiates scalar GPS speed using the application sample interval, filters the result, and expires stale data. Differentiation amplifies noise; it is a reference signal, not ground truth. `speed × bias-corrected earth_vertical_rate` estimates lateral acceleration under approximately planar motion and limited sideslip. Banking, slip, vibration and mounting errors violate those assumptions.

When an offset is saved, force projection rotates into vehicle axes and learning residuals rotate back into device pitch/roll axes. Saving/clearing resets learned force-fusion corrections.

`OrientationCorrector` compares the filtered longitudinal IMU signal with the speed derivative, and lateral IMU acceleration with the centripetal estimate. Small-angle pitch/roll residual corrections accumulate with opposite signs, bounded near ±15°. Cruise-bias learning also accumulates the residual; averaging it as an absolute bias would converge to half the true offset. Its confidence is a learning-sample counter, not a probability of correctness. GPS blend weight interpolates approximately 80% → 50% → 30% as pitch confidence grows. Freshness controls fallback to IMU-only estimates.

The filter attenuates vibration; it cannot eliminate arbitrary vibration or distinguish every road slope from attitude error. This pipeline does not supply validated slope compensation, brake pressure, tyre grip, apex location or driver coaching.

## Stationary detection and rates

The application removes predicted gravity before stationary gating. It requires
finite three-axis linear-acceleration magnitude below 0.18 g, three-axis angular
speed below 12°/s and GPS speed below 3.5 km/h. Norms make the evidence invariant
to sensor mounting rotation and reject horizontal-axis rotation and vertical
acceleration too. The application also requires fresh GPS, acceleration/gyro/attitude
packets no older than 200 ms and five consecutive candidate fixes. Invalid/stale
GPS or IMU evidence, or moving IMU evidence, interrupts that collection. A recovered
GPS fix starts a new five-fix collection even if no polling step observed the gap. Very slow motion and vibration remain hardware validation cases. Missing/invalid
IMU evidence pauses force learning and preserves the last finite packet values;
HTTP `imu_age_ms: null` marks forces unavailable while GPS timing continues.
Legacy v4/UDP alone has no equivalent force-validity metadata. ZUPT constrains velocity; it does not establish absolute position truth.

M9N builds target 25 Hz GPS, 6M builds 5 Hz. WT901 output can be configured up to 200 Hz; UART packet counts, processed estimator steps and the roughly 33 ms telemetry cadence are different rates. GPS crossings currently use host receive times, not receiver epoch/PPS synchronization. Serial buffering, receiver latency and browser sampling therefore remain in the error budget.

## What to validate

Synthetic regressions cover gravity/rotation conventions, earth-frame integration, covariance propagation, invalid measurements, residual correction convergence, crossing interpolation and outages. They prove those software behaviors, not driving accuracy. For an M9N drive, record actual rates, antenna placement, orientation, firmware commit, gaps and an independent crossing reference. See [the hardware checklist](HARDWARE_VALIDATION.md) and [timing guide](LAP_TIMER.md).

Before adjusting Q/R, establish mounting signs, timestamp cadence, receiver validity and whether the fault is in the estimator, display fusion or raw-GPS timing path. Changing Q does not directly smooth the separate mode/display filters.
