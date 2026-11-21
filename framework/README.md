# Sensor Fusion Framework

Reusable Rust components for Blackbox telemetry: sensor interfaces, coordinate transforms, a seven-state estimator, GPS/IMU acceleration blending, stationary gating, and circuit/stage timing.

## Use the estimator

```rust
use sensor_fusion::ekf::Ekf;
let mut ekf = Ekf::new();
// Earth-frame acceleration in m/s², yaw rate in rad/s, interval in seconds.
ekf.predict(0.5, 0.0, 0.0, 0.033);
ekf.update_position(1.0, 2.0); // local east/north metres
// Check receiver validity and physical bounds before supplying measurements.
ekf.update_velocity(3.0, 0.0);
let position = ekf.position();
```

State: `[east, north, yaw, velocity_east, velocity_north, bias_east, bias_north]`.
The estimator integrates earth-frame acceleration, including measured turning acceleration. Its covariance retains only diagonal entries; it is not a full covariance navigation EKF. Biases are learned while stationary. Timing uses fresh GPS fixes directly, independently of the estimator position.

## Modules and integration

- `ekf`: prediction, measurement updates, stationary constraints, diagnostic variance.
- `transforms`: gravity removal, body/earth/vehicle rotations, local geographic projection.
- `fusion`: filtered GPS speed derivative, orientation corrections, acceleration blending.
- `lap_timer`: finite directional timing lines, interpolated crossings, circuit/stage states.
- `sensor_framework`: sensor registry and typed readings. `poll_all()` returns a vector; match each reading's data rather than calling nonexistent `get_imu()`/`get_gps()` helpers.
- `stationary`, `velocity`, `mode`, `filter`: supporting processing.

Use a path dependency: `sensor-fusion = { path = "../../framework" }`.
The crate currently uses `std`, including its registry and allocation. A bare-metal `no_std` port is not supplied. Optional `esp32` and `mqtt` features are declared in [Cargo.toml](Cargo.toml); inspect the application for concrete transport integration. No crates.io publication or measured execution-time/memory budget is asserted here.

[Blackbox firmware](../sensors/blackbox/README.md) is the maintained integration example. Run host checks and embedded builds using [the contribution workflow](../CONTRIBUTING.md). See [sensor fusion mathematics](../docs/SENSOR_FUSION.md), [timing](../docs/LAP_TIMER.md), and [physical validation](../docs/HARDWARE_VALIDATION.md).

MIT license; see the repository LICENSE.
