# Open Motorsport Telemetry

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Rust](https://img.shields.io/badge/rust-1.75%2B-orange.svg)](https://www.rust-lang.org/)
[![ESP32](https://img.shields.io/badge/platform-ESP32-blue.svg)](https://www.espressif.com/)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](CONTRIBUTING.md)

An open-source framework for building custom automotive sensor systems. Built by enthusiasts who wanted professional-grade telemetry without the professional price tag.

### Sensor Implementations

#### [Active Wing](sensors/active-wing/) - Track the "attitude" of your car
**Hardware:** ESP32-C3 + WT901 9-axis IMU + NEO-6M GPS
Use Cases:


**What it does:**
- Real-time sensor fusion using Extended Kalman Filter
- GPS-corrected position tracking (eliminates IMU drift)
- Gravity-compensated acceleration measurements
- Coordinate frame transformations (body to earth frame)
- Zero-velocity updates (ZUPT) when stationary
- Driving mode detection (stopped, cruising, braking, cornering)
- Binary telemetry streaming over TCP (20Hz, 66-byte packets)
- MQTT status messages
- Python visualization tools included

**Use cases:**

- Vehicle dynamics analysis
- Driver training
- Suspension/brake/aero testing

---

## What Can You Build With This Framework?

### Motorsport Data Loggers
- Track day telemetry systems
- Autocross/rally data acquisition
- Drag racing performance monitors
- Motorcycle lean angle sensors

### Vehicle Development Tools
- Suspension testing rigs (accelerometers + ride height)
- Brake system analysis (pressure + temperature + deceleration)
- Drivetrain testing (wheel speed + torque + temperature)
- Aerodynamics testing (pressure sensors + speed + GPS)

### Fleet/Commercial Vehicles
- Delivery route optimization (GPS + driving behavior)
- Driver behavior monitoring (acceleration, braking, cornering)
- Fuel efficiency analysis (speed + throttle + terrain)
- Predictive maintenance (vibration + temperature monitoring)

### Research & Education
- Autonomous vehicle sensor fusion
- Control systems development
- Machine learning training data collection
- Embedded systems teaching platform

**The framework handles the hard parts (fusion, transforms, abstractions) so you can focus on your specific application.**

---

## Build Your Own Sensor System

The framework is designed to be reusable. Here's how to create your own sensor project:

### 1. Create Your Project

```bash
mkdir sensors/my-sensor-system
cd sensors/my-sensor-system
cargo init
```

### 2. Add Framework Dependency

```toml
# Cargo.toml
[dependencies]
active-wing-framework = { path = "../.." }
esp-idf-hal = "0.45"  # Or your platform's HAL
```

### 3. Implement Your Sensors

```rust
use active_wing_framework::sensor_framework::Sensor;

pub struct MyCustomSensor {
    // Your hardware interface
}

impl Sensor for MyCustomSensor {
    fn poll(&mut self) -> Option<SensorReading> {
        // Read your sensor
        Some(SensorReading { /* ... */ })
    }
}
```

### 4. Use the Framework Components

```rust
use active_wing_framework::{Ekf, transforms};

fn main() {
    // Create sensor registry
    let mut registry = SensorRegistry::new();
    registry.register(Box::new(MyCustomSensor::new()));

    // Use the EKF for fusion
    let mut ekf = Ekf::new();

    // Main loop
    loop {
        for reading in registry.poll_all() {
            // Use framework transforms
            let (ax_e, ay_e) = transforms::body_to_earth(/*...*/);
            ekf.predict(ax_e, ay_e, wz, dt);
        }
    }
}
```

**See `sensors/active-wing/` for a complete working example.**

---
<!-- 
## What's Included

### Reusable Framework Components (in `/framework`)

Build any sensor system using these modules:
- **`sensor_framework.rs`** - Plugin architecture, sensor registry
- **`ekf.rs`** - Extended Kalman Filter (7-state fusion)
- **`transforms.rs`** - Coordinate frame transformations
- **`sensors.rs`** - Sensor trait abstractions

### Standalone Sensor Drivers (in `/drivers`)

Reusable, no-std compatible sensor drivers:
- **`wt901`** - WT901 9-axis IMU driver (UART protocol parser)
- **`neo6m`** - NEO-6M GPS driver (NMEA parser + coordinate transforms)

These drivers have zero dependencies and work in any Rust embedded project.

### Active Wing Reference Implementation (in `/sensors/active-wing`)

A complete working example showing the framework in action:
- **ESP32-C3 firmware** with sensor fusion
- **WT901 IMU adapter** (wraps driver crate, 200Hz sampling)
- **NEO-6M GPS adapter** (wraps driver crate, 5Hz with NMEA parsing)
- **Binary telemetry protocol** (67-byte versioned packets, 20Hz over TCP)
- **Python visualization tools** for data display
- **Full documentation** and build instructions

### Documentation (in `/docs`) -->

Learn the framework and build your own systems:
- **[Sensor Toolkit Guide](docs/SENSOR_TOOLKIT_GUIDE.md)** - Step-by-step sensor integration
- **[Architecture Guide](docs/ARCHITECTURE.md)** - System design patterns
- **[Sensor Drivers Guide](SENSOR_DRIVERS.md)** - Standalone driver crates architecture
- **[Workspace Structure](WORKSPACE_STRUCTURE.md)** - Cargo workspace organization
- **[Active Wing README](sensors/active-wing/README.md)** - Reference implementation details

---

## Framework Performance

The reusable components are optimized for embedded systems:
- **EKF update**: <500µs per prediction step (200Hz capable on ESP32)
- **Memory footprint**: ~10KB framework overhead
- **Coordinate transforms**: <50µs per transformation
- **Sensor polling**: 200Hz+ (hardware-limited)

---

## Framework Architecture

### Conceptual Layers

```
┌──────────────────────────────────────────────┐
│  Your Sensor Project (sensors/your-project/) │
│  • Hardware-specific drivers                 │
│  • Application logic                         │
│  • Telemetry output format                   │
└────────────────┬─────────────────────────────┘
                 │ uses
                 ▼
┌──────────────────────────────────────────────┐
│  Sensor Fusion Framework (src/)              │
│  ┌──────────────────────────────────────┐    │
│  │ Sensor Abstractions (sensor traits)  │    │
│  ├──────────────────────────────────────┤    │
│  │ Fusion Algorithms (EKF, transforms)  │    │
│  ├──────────────────────────────────────┤    │
│  │ Plugin System (registry, poll)       │    │
│  └──────────────────────────────────────┘    │
└──────────────────────────────────────────────┘
```

### Active Wing System Diagram

How the reference implementation uses the framework:

```
┌──────────────────────────────────────┐
│         ESP32-C3 (on your car)       │
│                                      │
│  ┌────────────────────────────────┐  │
│  │  Sensor Drivers (active-wing)  │  │
│  │  ├─ WT901 IMU (200Hz)          │  │
│  │  └─ NEO-6M GPS (5Hz)           │  │
│  └────────────────────────────────┘  │
│              ▼                       │
│  ┌────────────────────────────────┐  │
│  │  Framework Components          │  │
│  │  • EKF (7-state)               │  │
│  │  • Coordinate transforms       │  │
│  │  • Sensor registry             │  │
│  └────────────────────────────────┘  │
│              ▼                       │
│  ┌────────────────────────────────┐  │
│  │  Application Layer             │  │
│  │  ├─ TCP telemetry (20Hz)       │  │
│  │  └─ MQTT status                │  │
│  └────────────────────────────────┘  │
└──────────────────────────────────────┘
                 │
                 │ WiFi
                 ▼
      ┌───────────────────────┐
      │  Python Receiver      │
      │  • Live visualization │
      │  • CSV logging        │
      └───────────────────────┘
```

---

## Extending with Custom Sensors

The framework's plugin architecture makes it trivial to add new sensors:

### Example: Add Wheel Speed to Active Wing

```rust
use active_wing_framework::sensor_framework::Sensor;

pub struct WheelSpeedSensor {
    gpio_pin: GpioPin,
    pulse_count: u32,
}

impl Sensor for WheelSpeedSensor {
    fn capabilities(&self) -> SensorCapabilities {
        SensorCapabilities {
            id: SensorId(3),
            name: "Wheel Speed",
            measurement_types: &[MeasurementType::Velocity3D],
            update_rate_hz: 50.0,
            // ...
        }
    }

    fn poll(&mut self) -> Option<SensorReading> {
        // Read Hall effect sensor pulses
        let speed = self.calculate_speed();
        Some(SensorReading {
            data: SensorData::WheelSpeed(speed),
            // ...
        })
    }
}

// In main.rs - just register it:
registry.register(Box::new(WheelSpeedSensor::new(gpio_pin)))?;
```

**That's it! No changes to IMU, GPS, EKF, or telemetry code.**

See the [Sensor Toolkit Guide](docs/SENSOR_TOOLKIT_GUIDE.md) for step-by-step instructions.

---

## Repository Structure

This is a **Cargo workspace** containing multiple related packages:

```
active_wing/
│
├── Cargo.toml                     ← Workspace definition
├── .rustfmt.toml                  ← Shared formatting config
├── clippy.toml                    ← Shared linting config
│
├── framework/                     ← FRAMEWORK (reusable)
│   ├── Cargo.toml
│   └── src/
│       ├── lib.rs                 ← Framework entry point
│       ├── sensor_framework.rs    ← Plugin architecture
│       ├── ekf.rs                 ← Extended Kalman Filter
│       ├── transforms.rs          ← Coordinate transformations
│       └── sensors.rs             ← Sensor trait abstractions
│
├── drivers/                       ← SENSOR DRIVERS (reusable)
│   ├── wt901/                     ← WT901 IMU driver crate
│   │   ├── Cargo.toml             ← No dependencies, no-std
│   │   ├── README.md              ← Driver documentation
│   │   └── src/lib.rs             ← Parser, state machine
│   │
│   └── neo6m/                     ← NEO-6M GPS driver crate
│       ├── Cargo.toml             ← No dependencies, no-std
│       ├── README.md              ← Driver documentation
│       └── src/lib.rs             ← NMEA parser, transforms
│
├── sensors/                       ← SENSOR PROJECTS
│   └── active-wing/               ← Reference implementation
│       ├── Cargo.toml             ← Depends on framework + drivers
│       ├── config.toml.example    ← Runtime configuration
│       ├── src/
│       │   ├── main.rs            ← Application entry
│       │   ├── config.rs          ← Configuration management
│       │   ├── imu.rs             ← WT901 adapter (wraps driver)
│       │   ├── gps.rs             ← NEO-6M adapter (wraps driver)
│       │   ├── system.rs          ← System architecture
│       │   └── ...
│       ├── tools/python/          ← Telemetry receivers
│       ├── docs/                  ← Project-specific docs
│       └── README.md              ← Build instructions
│
└── docs/                          ← Framework documentation
    ├── SENSOR_TOOLKIT_GUIDE.md    ← How to add sensors
    ├── ARCHITECTURE.md            ← System design
    ├── SENSOR_DRIVERS.md          ← Driver crates guide
    └── WORKSPACE_STRUCTURE.md     ← Workspace organization
```

**Workspace Structure Benefits:**
- **Shared configuration** - Single .rustfmt.toml and clippy.toml
- **Unified builds** - `cargo build` builds everything
- **Local dependencies** - Drivers reference each other via `path = "../.."`
- **Publishable crates** - Drivers can be published to crates.io independently

See [WORKSPACE_STRUCTURE.md](WORKSPACE_STRUCTURE.md) for detailed explanation.

**Package Roles:**
- **Framework** (`framework/`) - Reusable sensor fusion components
- **Drivers** (`drivers/`) - Standalone, reusable sensor parsers (no dependencies)
- **Active Wing** (`sensors/active-wing/`) - Complete ESP32 application
- **Your Project** - Add to `sensors/` and reference framework + drivers

---

## Data Export & Analysis

### Real-Time Streaming
- **TCP binary protocol** (67 bytes/packet, 20Hz, versioned)
- **MQTT status messages** (JSON)
- **WebSocket** support (coming soon)

### Data Logging
- Python receiver logs to CSV
- Import to:
  - **Excel** / Google Sheets
  - **RaceChrono** (coming soon)
  - **Circuit Tools** (coming soon)
  - **Custom analysis** (Python, MATLAB, etc.)

### Telemetry Format

Binary packet (67 bytes, versioned):
```c
struct TelemetryPacket {
    uint8_t version;       // Protocol version (currently 1)
    uint16_t header;       // 0xAA55
    uint32_t timestamp_ms;
    float ax, ay, az;      // Acceleration (m/s²)
    float wz;              // Yaw rate (rad/s)
    float roll, pitch, yaw; // Orientation (rad)
    float x, y;            // Position (m)
    float vx, vy;          // Velocity (m/s)
    float speed_kmh;       // Speed (km/h)
    float lat, lon;        // GPS (degrees)
    uint8_t gps_valid;
    uint8_t mode;          // Driving mode
    uint16_t checksum;
}
```

---

## Documentation

- **[Active Wing System](sensors/active-wing/README.md)** - Complete example implementation
- **[Sensor Toolkit Guide](docs/SENSOR_TOOLKIT_GUIDE.md)** - How to add sensors
- **[Architecture Guide](docs/ARCHITECTURE.md)** - System design
- **[Contributing Guide](CONTRIBUTING.md)** - How to contribute

---

## Roadmap

### Current (v0.1)
-  ESP32-C3 + WT901 IMU + NEO-6M GPS
-  Extended Kalman Filter fusion
-  TCP telemetry (20Hz, versioned protocol)
-  Python visualization
-  Plugin architecture
-  Cargo workspace structure
-  Standalone driver crates (wt901, neo6m)
-  Configuration management system

### Next Release (v0.2)
-  Web-based dashboard (real-time)
-  Data logging to SD card
-  Wheel speed sensor support
-  Lap timing (GPS-based)
-  RaceChrono export format

### Future
-  CAN bus integration (OBD2 data)
-  Camera integration (lane detection)
-  Multi-board distributed sensors
-  Machine learning (corner prediction, optimal line)
-  Mobile app (iOS/Android)

---

## Community & Support

- **Issues**: [GitHub Issues](https://github.com/jctoledo/active_wing/issues)
- **Discussions**: [GitHub Discussions](https://github.com/jctoledo/active_wing/discussions)
- **Contributing**: See [CONTRIBUTING.md](CONTRIBUTING.md)

### Show Your Build!

Built an Active Wing system? We'd love to see it!
- Post in [Discussions](https://github.com/jctoledo/active_wing/discussions)
- Share your racing data
- Submit improvements via PR

---

## License

MIT License - see [LICENSE](LICENSE) for details.

**TL;DR**: Use it, modify it, sell it, race with it. Just keep the license notice.

---

## Acknowledgments

- Built with Rust for performance and safety
- ESP-IDF for ESP32 support
- Inspired by professional motorsport telemetry systems
- Designed with SOLID principles for production-grade code

---

## FAQ

**Q: Can I use this in competition?**
A: Yes! Check your racing series rules, but most allow data loggers.

**Q: Will it work on motorcycles?**
A: Absolutely! May need different mounting strategy.

**Q: Can I add my own sensors?**
A: Yes! The plugin architecture makes it easy. See the [Sensor Guide](docs/SENSOR_TOOLKIT_GUIDE.md).

**Q: Does it work offline?**
A: Yes for logging. WiFi only needed for live telemetry.

**Q: How accurate is it?**
A: GPS: ~2m position, IMU: 0.01g acceleration. Professional-grade for the price.

**Q: Can I sell devices based on this?**
A: Yes! MIT license allows commercial use.

---

**Ready to track your performance? Start with the [Quick Start](#quick-start)** 
