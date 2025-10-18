# Open Motorsport Telemetry

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Rust](https://img.shields.io/badge/rust-1.75%2B-orange.svg)](https://www.rust-lang.org/)
[![ESP32](https://img.shields.io/badge/platform-ESP32-blue.svg)](https://www.espressif.com/)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](CONTRIBUTING.md)

An open-source framework for building custom automotive sensor systems. Built by enthusiasts who wanted professional-grade telemetry without the professional price tag.

## What Is This?

This repository contains a sensor fusion framework and sensor implementations built with it.

### Sensor Fusion Framework

A production-ready framework for building multi-sensor telemetry systems:
- **Plugin architecture** - add sensors without modifying core code
- **Extended Kalman Filter** - 7-state sensor fusion (position, velocity, yaw, biases)
- **Coordinate transformations** - body/earth/vehicle frame math
- **Hardware abstraction** - works with any sensor via traits
- **SOLID principles** - maintainable, testable, extensible code

### Sensor Implementations

#### [Active Wing](sensors/active-wing/) - ESP32 Motorsport Telemetry
**Hardware:** ESP32-C3 + WT901 9-axis IMU + NEO-6M GPS
**Sensors:**
- WT901 9-axis IMU (200Hz via UART)
  - 3-axis accelerometer
  - 3-axis gyroscope
  - 3-axis magnetometer
- NEO-6M GPS receiver (5Hz via UART)
  - Position (lat/lon)
  - Velocity and course
  - HDOP and satellite count
- WS2812 RGB LED (status indicator)

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
- Track day data logging
- Autocross/rally telemetry
- Vehicle dynamics analysis
- Driver training
- Suspension/brake/aero testing

[See Active Wing documentation →](sensors/active-wing/README.md)

---

## What Can You Build With This Framework?

### Motorsport Data Loggers (like Active Wing)
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

## What's Included

### Reusable Framework Components (in `/src`)

Build any sensor system using these modules:
- **`sensor_framework.rs`** - Plugin architecture, sensor registry
- **`ekf.rs`** - Extended Kalman Filter (7-state fusion)
- **`transforms.rs`** - Coordinate frame transformations
- **`sensors.rs`** - Sensor trait abstractions

### Active Wing Reference Implementation (in `/sensors/active-wing`)

A complete working example showing the framework in action:
- **ESP32-C3 firmware** with sensor fusion
- **WT901 IMU driver** (200Hz sampling)
- **NEO-6M GPS driver** (5Hz with NMEA parsing)
- **Binary telemetry protocol** (66-byte packets, 20Hz over TCP)
- **Python visualization tools** for data display
- **Full documentation** and build instructions

### Documentation (in `/docs`)

Learn the framework and build your own systems:
- **[Sensor Toolkit Guide](docs/SENSOR_TOOLKIT_GUIDE.md)** - Step-by-step sensor integration
- **[Architecture Guide](docs/ARCHITECTURE.md)** - System design patterns
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

```
automotive-sensor-fusion/
│
├── src/                           ← FRAMEWORK (reusable)
│   ├── lib.rs                     ← Framework entry point
│   ├── sensor_framework.rs        ← Plugin architecture
│   ├── ekf.rs                     ← Extended Kalman Filter
│   ├── transforms.rs              ← Coordinate transformations
│   └── sensors.rs                 ← Sensor trait abstractions
│
├── docs/                          ← Framework documentation
│   ├── SENSOR_TOOLKIT_GUIDE.md    ← How to add sensors
│   ├── ARCHITECTURE.md            ← System design
│   └── ARCHITECTURE_CLARIFICATION.md
│
├── sensors/                       ← SENSOR PROJECTS
│   │
│   ├── active-wing/               ← Reference implementation
│   │   ├── src/                   ← ESP32 application code
│   │   │   ├── main.rs            ← Application entry
│   │   │   ├── imu.rs             ← WT901 driver
│   │   │   ├── gps.rs             ← NEO-6M driver
│   │   │   └── ...
│   │   ├── tools/python/          ← Telemetry receivers
│   │   ├── docs/                  ← Project-specific docs
│   │   ├── Cargo.toml             ← Depends on framework
│   │   └── README.md              ← Build instructions
│   │
│   └── your-project/              ← Your custom sensor system
│       ├── src/
│       ├── Cargo.toml             ← Depends on framework
│       └── README.md
│
├── Cargo.toml                     ← Framework library config
└── README.md                      ← This file
```

**Framework vs Projects:**
- **Framework** (`/src`) - Reusable components for ANY sensor system
- **Active Wing** (`/sensors/active-wing`) - ONE example using the framework
- **Your Project** (`/sensors/your-project`) - YOUR sensor system using the framework

---

## Data Export & Analysis

### Real-Time Streaming
- **TCP binary protocol** (66 bytes/packet, 20Hz)
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

Binary packet (66 bytes):
```c
struct TelemetryPacket {
    uint16_t header;       // 0xAA55
    uint32_t timestamp_ms;
    float ax, ay, az;      // Acceleration (m/s²)
    float wz;              // Yaw rate (rad/s)
    float roll, pitch, yaw; // Orientation (rad)
    float x, y;            // Position (m)
    float vx, vy;          // Velocity (m/s)
    float speed_kmh;       // Speed (km/h)
    double lat, lon;       // GPS (degrees)
    uint8_t gps_valid;
    uint8_t mode;          // Driving mode
    uint16_t crc;
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
-  TCP telemetry (20Hz)
-  Python visualization
-  Plugin architecture

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
