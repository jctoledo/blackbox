# Sensor Projects

This directory contains sensor projects built using the **Sensor Fusion Framework**.

Each subdirectory is a complete, independent sensor application that uses the framework for sensor fusion and coordinate transformations.

## Current Projects

### [blackbox](./blackbox/)
ESP32-C3 based vehicle telemetry system with WT901 IMU and a supported NEO-6M/NEO-M9N GPS.

**Hardware:**
- ESP32-C3 microcontroller
- WT901 9-axis IMU (configurable output up to 200 Hz)
- NEO-6M (5 Hz target) or NEO-M9N (25 Hz target)
- WS2812 RGB LED

**Features:**
- Local sensor fusion (diagonal covariance approximation)
- HTTP/UDP binary v4 telemetry (33 ms configured target interval)
- MQTT status messages
- Driving mode detection (IDLE, ACCEL, BRAKE, CORNER, ACCEL+CORNER, BRAKE+CORNER)
- GPS-corrected position tracking

**Use case:** An inspectable DIY instrument for circuit/stage timing and vehicle telemetry. [Compare capabilities and evidence](../docs/COMPARISON.md).

[See full documentation →](./blackbox/README.md)

---

## Adding Your Own Sensor Project

Want to build your own custom telemetry system? Here's how:

### 1. Create Your Project Directory

```bash
mkdir sensors/my-sensor-system
cd sensors/my-sensor-system
```

### 2. Initialize with Cargo

```bash
cargo init --name my-sensor-system
```

### 3. Add Framework Dependency

Edit `Cargo.toml`:

```toml
[package]
name = "my-sensor-system"
version = "0.1.0"
edition = "2021"

[dependencies]
# Core framework
sensor-fusion = { path = "../../framework" }

# Sensor drivers (if using existing ones)
wt901 = { path = "../../drivers/wt901" }
ublox-gps = { path = "../../drivers/ublox-gps" }

# Your platform HAL
# For ESP32:
esp-idf-hal = "0.45"
esp-idf-svc = "0.51"

# For other platforms:
# embedded-hal = "1.0"
# cortex-m-rt = "0.7"
```

### 4. Implement Your Application

```rust
use sensor_fusion::ekf::Ekf;
use sensor_fusion::transforms::{body_to_earth, remove_gravity};

fn main() {
    // Initialize your hardware
    let mut imu = MyImuSensor::new();
    let mut gps = MyGpsSensor::new();

    // Create EKF
    let mut ekf = Ekf::new();

    // Main loop
    loop {
        // Read sensors
        let imu_data = imu.read();

        // Transform accelerations
        let (ax_b, ay_b, az_b) = remove_gravity(
            imu_data.ax, imu_data.ay, imu_data.az,
            imu_data.roll, imu_data.pitch
        );

        let (ax_e, ay_e) = body_to_earth(
            ax_b, ay_b, az_b,
            imu_data.roll, imu_data.pitch, ekf.yaw()
        );

        // Update EKF
        ekf.predict(ax_e, ay_e, imu_data.wz, dt);

        // Get fused state
        let (x, y) = ekf.position();
        let (vx, vy) = ekf.velocity();

        // Output telemetry (your choice: TCP, UART, SD card, etc.)
    }
}
```

### 5. Add to Workspace

Add the new path to the root Cargo workspace members explicitly. Configure the target/toolchain for that hardware and review its build integration. The example above is pseudocode: hardware types and `dt` must be supplied by your application.

### 6. Validate the build

For Blackbox, follow the root README and `.github/workflows/ci.yml`: host tests target pure crates; firmware builds run from `sensors/blackbox` for `riscv32imc-esp-espidf`. All project builds/tests run on lat.

## CI integration

CI checks Blackbox explicitly. Release discovery finds project metadata, but its build step still assumes the C3 target and 6M/M9N variants. Adding metadata alone does not provide another board's toolchain, pin mapping or release validation.

## Example Use Cases

### Racing Series
Build a custom telemetry system for your racing class:
- Spec hardware constraints
- Custom data logging format
- Integration with series timing systems

### Vehicle Development
Create specialized data acquisition for:
- Suspension tuning (ride height + accelerations)
- Brake testing (pressure + temperature + g-forces)
- Aero testing (speed + pressure sensors)

### Fleet Monitoring
Build commercial vehicle monitoring:
- Driver behavior tracking
- Fuel efficiency analysis
- Predictive maintenance

### Research & Education
Academic projects:
- Autonomous vehicle sensor fusion
- Control systems development
- Machine learning training data

## Framework Documentation

See the framework documentation for details on:
- [Extended Kalman Filter](../framework/src/ekf.rs)
- [Coordinate Transformations](../framework/src/transforms.rs)
- [Sensor Plugin System](../framework/src/sensor_framework.rs)
- [Framework README](../framework/README.md)

## Driver Crates

Reusable sensor drivers available:
- [wt901](../drivers/wt901/) - WT901 9-axis IMU
- [ublox-gps](../drivers/ublox-gps/) - u-blox GPS driver (NEO-6M, NEO-M9N, etc.)

Create your own driver crates in `drivers/` for reuse across projects!

## Questions?

- Check the [Blackbox implementation](./blackbox/) for a complete example
- Read the [framework guide](../framework/README.md) for current integration boundaries
- Open an issue on GitHub for help

Happy building! 🏁
