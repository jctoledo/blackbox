# Sensor Projects

This directory contains sensor projects built using the **Sensor Fusion Framework**.

Each subdirectory is a complete, independent sensor application that uses the framework for sensor fusion and coordinate transformations.

## Current Projects

### [blackbox](./blackbox/)
ESP32-C3 based vehicle telemetry system with WT901 IMU and NEO-6M GPS.

**Hardware:**
- ESP32-C3 microcontroller
- WT901 9-axis IMU (200 Hz)
- NEO-6M GPS receiver (5 Hz)
- WS2812 RGB LED

**Features:**
- Real-time sensor fusion (Extended Kalman Filter)
- TCP binary telemetry streaming (20 Hz)
- MQTT status messages
- Driving mode detection (IDLE, ACCEL, BRAKE, CORNER, ACCEL+CORNER, BRAKE+CORNER)
- GPS-corrected position tracking

**Use case:** Track day data logging, autocross, rally, vehicle dynamics research

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
neo6m = { path = "../../drivers/neo6m" }

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
        let (ax_b, ay_b, _) = remove_gravity(
            imu_data.ax, imu_data.ay, imu_data.az,
            imu_data.roll, imu_data.pitch
        );

        let (ax_e, ay_e) = body_to_earth(
            ax_b, ay_b, 0.0,
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

The workspace is configured to automatically include all projects in `sensors/`. Just make sure your `Cargo.toml` is valid and it will be built by CI.

### 6. Test Your Build

```bash
# For ESP32:
rustup target add riscv32imc-unknown-none-elf
cargo build --release --target riscv32imc-unknown-none-elf

# For ARM Cortex-M:
rustup target add thumbv7em-none-eabihf
cargo build --release --target thumbv7em-none-eabihf

# For native (testing):
cargo build --release
```

## CI Integration

The CI automatically:
1. **Discovers** all sensor projects in `sensors/*/`
2. **Builds** each project for the target architecture
3. **Uploads** firmware artifacts
4. **Reports** build status for each project

No CI configuration changes needed when adding new projects!

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
- [neo6m](../drivers/neo6m/) - NEO-6M GPS receiver

Create your own driver crates in `drivers/` for reuse across projects!

## Questions?

- Check the [Blackbox implementation](./blackbox/) for a complete example
- See [SENSOR_DRIVERS.md](../SENSOR_DRIVERS.md) for driver architecture
- Read [WORKSPACE_STRUCTURE.md](../WORKSPACE_STRUCTURE.md) for workspace organization
- Open an issue on GitHub for help

Happy building! 🏁
