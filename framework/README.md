# Motorsport Telemetry Framework

A modular, extensible Rust framework for building custom automotive sensor fusion systems. Build professional-grade telemetry for track days, racing, vehicle dynamics research, and fleet monitoring without vendor lock-in.

## Features

- **🔌 Plugin Architecture** - Add sensors without modifying existing code
- **📊 Sensor Fusion** - Extended Kalman Filter (7-state: position, velocity, yaw, biases)
- **🔄 Coordinate Transforms** - Body ↔ Earth ↔ Vehicle frame conversions
- **🚫 No-std Compatible** - Works on embedded systems without operating system
- **🌐 Distributed Support** - MQTT-based multi-board sensor networks (optional)
- **🎯 Hardware Agnostic** - Abstract sensor interfaces work with any hardware

## What It Does

The framework handles the hard parts of building a telemetry system:

1. **Sensor Abstraction** - Define hardware-independent sensor interfaces
2. **Plugin System** - Register and poll multiple sensors without coupling
3. **Sensor Fusion** - Combine noisy sensor data into accurate state estimates
4. **Coordinate Math** - Transform accelerations between body/earth/vehicle frames

You focus on your specific sensors and application - the framework handles the fusion logic.

## Example Usage

```rust
use motorsport_telemetry::sensor_framework::{SensorRegistry, Sensor};
use motorsport_telemetry::ekf::Ekf;
use motorsport_telemetry::transforms::{body_to_earth, remove_gravity};

// Create sensor registry
let mut registry = SensorRegistry::new();

// Add your sensors (implement Sensor trait)
registry.register(Box::new(MyImuSensor::new()))?;
registry.register(Box::new(MyGpsSensor::new()))?;

// Create Extended Kalman Filter
let mut ekf = Ekf::new();

// Main fusion loop
loop {
    // Poll all sensors
    let readings = registry.poll_all();

    // Process IMU reading
    if let Some(imu_reading) = readings.get_imu() {
        // Remove gravity from acceleration
        let (ax_b, ay_b, az_b) = remove_gravity(
            imu_reading.ax, imu_reading.ay, imu_reading.az,
            imu_reading.roll, imu_reading.pitch
        );

        // Transform to earth frame
        let (ax_e, ay_e) = body_to_earth(
            ax_b, ay_b, az_b,
            imu_reading.roll, imu_reading.pitch, ekf.yaw()
        );

        // EKF prediction step
        ekf.predict(ax_e, ay_e, imu_reading.wz, dt);
    }

    // Process GPS reading
    if let Some(gps_reading) = readings.get_gps() {
        ekf.update_position(gps_reading.x, gps_reading.y);
        ekf.update_velocity(gps_reading.vx, gps_reading.vy);
    }

    // Get fused state estimate
    let (x, y) = ekf.position();
    let (vx, vy) = ekf.velocity();
    let yaw = ekf.yaw();

    // Stream telemetry, log to SD card, etc.
}
```

## Modules

### `sensor_framework`
Core plugin architecture. Define sensor traits and register implementations:

```rust
pub trait Sensor {
    fn capabilities(&self) -> SensorCapabilities;
    fn poll(&mut self) -> Option<SensorReading>;
}

let mut registry = SensorRegistry::new();
registry.register(Box::new(MySensor::new()))?;
```

### `ekf`
Extended Kalman Filter for sensor fusion:

```rust
let mut ekf = Ekf::new();
ekf.predict(ax_earth, ay_earth, wz, dt);  // IMU prediction
ekf.update_position(x, y);                // GPS update
ekf.update_velocity(vx, vy);              // GPS velocity
ekf.zupt();                               // Zero-velocity update when stopped
```

**State vector (7D):**
- Position: `x, y` (m)
- Velocity: `vx, vy` (m/s)
- Yaw angle: `ψ` (rad)
- Accelerometer biases: `bax, bay` (m/s²)

### `transforms`
Coordinate frame transformations for vehicle dynamics:

```rust
// Remove gravity component from IMU reading
let (ax_body, ay_body, az_body) = remove_gravity(
    ax_measured, ay_measured, az_measured,
    roll, pitch
);

// Transform body frame → earth frame (horizontal plane)
let (ax_earth, ay_earth) = body_to_earth(
    ax_body, ay_body, az_body,
    roll, pitch, yaw
);

// Transform earth frame → vehicle frame (car-centric)
let (ax_lon, ay_lat) = earth_to_car(ax_earth, ay_earth, yaw);

// GPS coordinate conversions
let (east_m, north_m) = gps_to_local(lat, lon, ref_lat, ref_lon);
```

### `sensors`
Basic sensor trait definitions (IMU, GPS, generic sensors).

## Architecture

```text
┌─────────────────────────────────────────┐
│  Your Application (sensors/your-app/)   │
│  • Hardware drivers                     │
│  • Application logic                    │
│  • Telemetry output                     │
└──────────────┬──────────────────────────┘
               │ uses
               ▼
┌─────────────────────────────────────────┐
│  Motorsport Telemetry Framework         │
│  ┌─────────────────────────────────┐    │
│  │ sensor_framework (Plugin API)   │    │
│  ├─────────────────────────────────┤    │
│  │ ekf (Sensor Fusion)             │    │
│  ├─────────────────────────────────┤    │
│  │ transforms (Coordinate Math)    │    │
│  └─────────────────────────────────┘    │
└─────────────────────────────────────────┘
```

## Use Cases

- **Track Day Data Logging** - Record lap times, g-forces, racing lines
- **Vehicle Dynamics Research** - Suspension, brake, aero tuning data
- **Fleet Monitoring** - Driver behavior and vehicle health tracking
- **Autonomous Vehicles** - Sensor fusion for localization/navigation
- **Racing Series** - Custom telemetry for spec racing classes
- **Educational Projects** - Learn sensor fusion and embedded Rust

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
motorsport-telemetry = { path = "../path/to/framework" }

# Or from crates.io (when published):
# motorsport-telemetry = "0.1"
```

## Features

The framework supports optional features:

```toml
[dependencies]
motorsport-telemetry = { version = "0.1", features = ["esp32", "mqtt"] }
```

- `esp32` - ESP32-specific HAL support
- `mqtt` - MQTT distributed sensor support

## Examples

See the `sensors/active-wing/` directory for a complete working example:
- ESP32-C3 firmware
- WT901 IMU + NEO-6M GPS
- TCP telemetry streaming (20 Hz)
- MQTT status messages
- Driving mode classification

## Performance

- **EKF update**: <500µs per prediction step (200 Hz capable on ESP32)
- **Memory footprint**: ~10KB framework overhead
- **Coordinate transforms**: <50µs per transformation
- **Sensor polling**: 200+ Hz (hardware-limited)

## Design Principles

1. **SOLID Principles** - Maintainable, testable, extensible code
2. **Dependency Inversion** - Framework depends on abstractions, not implementations
3. **Open/Closed** - Open for extension (new sensors), closed for modification
4. **Single Responsibility** - Each module has one clear purpose
5. **Hardware Agnostic** - Works with any sensor via trait implementations

## Contributing

Contributions welcome! This framework is designed to support the entire motorsport and automotive community. See `CONTRIBUTING.md` in the repository root.

## License

MIT License - Use it, modify it, sell it, race with it. See `LICENSE` for details.

## Related Projects

- **Active Wing** - ESP32-based telemetry system using this framework
- **wt901 driver** - Standalone WT901 IMU driver crate
- **neo6m driver** - Standalone NEO-6M GPS driver crate

## Documentation

- [API Documentation](https://docs.rs/motorsport-telemetry) (when published)
- [Sensor Toolkit Guide](../docs/SENSOR_TOOLKIT_GUIDE.md)
- [Architecture Guide](../docs/ARCHITECTURE.md)
- [Workspace Structure](../WORKSPACE_STRUCTURE.md)
