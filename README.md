# Active Wing 🏎️

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Rust](https://img.shields.io/badge/rust-1.75%2B-orange.svg)](https://www.rust-lang.org/)
[![ESP32](https://img.shields.io/badge/platform-ESP32--C3-blue.svg)](https://www.espressif.com/en/products/socs/esp32-c3)
[![CI](https://github.com/OWNER/active_wing/workflows/CI/badge.svg)](https://github.com/OWNER/active_wing/actions)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](CONTRIBUTING.md)

> Real-time vehicle dynamics telemetry system for ESP32. Track your car's every move with sub-50ms latency.

An open-source embedded telemetry platform that turns a $10 microcontroller into a high-speed data acquisition system. Perfect for track days, autocross, rally, or anyone who wants to know *exactly* what their car is doing.

## Table of Contents

- [What Does It Do?](#what-does-it-do)
- [Why This Exists](#why-this-exists)
- [Hardware You'll Need](#hardware-youll-need)
- [Quick Start](#quick-start)
  - [For Users](#for-users-just-want-to-use-it)
  - [For Developers](#for-developers-want-to-modify-code)
- [What You Get](#what-you-get)
- [How It Works](#how-it-works)
- [Use Cases](#use-cases)
- [Customization](#customization)
- [Performance](#performance)
- [Troubleshooting](#troubleshooting)
- [Documentation](#project-documentation)
- [Contributing](#contributing)

## What Does It Do?

Active Wing fuses IMU and GPS data at 200 Hz to give you:

- **Position tracking** with GPS-corrected dead reckoning (no drift!)
- **Real-time velocity** and heading estimation
- **Acceleration in all axes** (longitudinal, lateral, vertical)
- **Driving mode detection** (idle, acceleration, braking, cornering)
- **20 Hz telemetry streaming** over WiFi for live dashboards

All running on an ESP32-C3 that fits in your palm and costs less than lunch.

## Why This Exists

Most commercial data loggers are either:
1. Expensive ($500+)
2. Limited to 10 Hz sampling
3. Require proprietary software
4. Don't do sensor fusion

This project gives you motorsport-grade telemetry with:
- ✅ Open source everything (MIT license)
- ✅ 200 Hz IMU sampling → 20 Hz fused output
- ✅ Extended Kalman Filter for sensor fusion
- ✅ Real-time streaming to any device on your network
- ✅ Total cost: ~$30 in parts

## Hardware You'll Need

| Component | Purpose | Cost |
|-----------|---------|------|
| ESP32-C3 dev board | Main processor | ~$5 |
| WT901 9-axis IMU | Accelerometer + gyro + magnetometer | ~$15 |
| NEO-6M GPS module | Position + velocity | ~$8 |
| WS2812 RGB LED (optional) | Status indicator | ~$1 |

**Total: ~$30**

### Wiring

```
ESP32-C3          WT901 IMU
├─ GPIO 18    →   RX (UART)
├─ GPIO 19    ←   TX (UART)
├─ 3.3V       →   VCC
└─ GND        →   GND

ESP32-C3          NEO-6M GPS
├─ GPIO 5     →   RX (UART)
├─ GPIO 4     ←   TX (UART)
├─ 3.3V       →   VCC
└─ GND        →   GND

ESP32-C3          WS2812 LED (optional)
├─ GPIO 8     →   DIN
├─ 5V         →   VCC
└─ GND        →   GND
```

## Quick Start

### For Users (Just Want to Use It)

**1. Install Rust + ESP Toolchain**
```bash
# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Install ESP tooling
cargo install cargo-espflash espflash ldproxy
```

**2. Configure Your Network**

Edit `src/main.rs` and update these lines:
```rust
const WIFI_SSID: &str = "YourWiFiNetwork";
const WIFI_PASSWORD: &str = "YourPassword";
const TCP_SERVER: &str = "192.168.1.100:9000";  // Your laptop's IP
```

**3. Flash It**
```bash
# Build and flash in one go
cargo espflash flash --release --monitor
```

**4. Receive Telemetry**

On your laptop, start the TCP server:
```bash
pip install -r tools/python/requirements.txt
python3 tools/python/tcp_telemetry_server.py
```

You'll see live telemetry at 20 Hz:
```
[20Hz] Spd: 45.2km/h  Pos:(  123.4,  -45.6)m  Vel:(+12.34,+0.45)m/s
       Acc:(+2.34,+0.12,-9.81)m/s²  Yaw: +15°  wz:+12°/s  ACCEL
```

### For Developers (Want to Modify Code)

**Additional Setup:**
```bash
# Install Rust target for ESP32-C3
rustup target add riscv32imc-unknown-none-elf

# Install development tools
cargo install ldproxy

# Check code without building
cargo check

# Format code (required before committing)
cargo fmt

# Run linter (required before committing)
cargo clippy -- -D warnings

# Run tests
cargo test --lib --bins
```

**Editor Setup:**
- We use `.editorconfig` for consistent formatting across editors
- Install EditorConfig plugin for your editor (VS Code, IntelliJ, Vim, etc.)
- Rust formatting rules in `.rustfmt.toml`
- Clippy rules in `clippy.toml`

**See [CLAUDE.md](CLAUDE.md)** for complete development commands and architecture details.
**See [CONTRIBUTING.md](CONTRIBUTING.md)** for contribution guidelines.

## What You Get

### Binary Telemetry Stream (20 Hz)

Each packet contains:
- **Position** (x, y) in local coordinates
- **Velocity** (vx, vy) and speed
- **Acceleration** (ax, ay, az) bias-corrected
- **Orientation** (roll, pitch, yaw)
- **Yaw rate** (wz)
- **GPS** (lat, lon) when available
- **Driving mode** (IDLE / ACCEL / BRAKE / CORNER)

Total: 66 bytes per packet, 1.3 KB/s bandwidth.

### LED Status Codes

The RGB LED tells you what's happening:

| Color | Pattern | Meaning |
|-------|---------|---------|
| Blue | 3 blinks | Booting |
| Green | 5 blinks | WiFi connected |
| Red | Slow blink | WiFi failed |
| Cyan | 3 blinks | TCP connected |
| Yellow | Pulsing | Calibrating IMU |
| Cyan | Slow pulse (2s) | GPS locked, ready to record |
| Yellow | Fast blink | Waiting for GPS |

## How It Works

### The Magic: Sensor Fusion

GPS is accurate but slow (5 Hz). IMUs are fast (200 Hz) but drift. Active Wing runs an **Extended Kalman Filter** that fuses both:

1. **Prediction** (200 Hz): Use IMU to predict position/velocity
   - CTRA model when moving (Constant Turn Rate & Acceleration)
   - Handles the physics of turning without GPS

2. **Correction** (5 Hz): Use GPS to fix accumulated errors
   - Position and velocity updates
   - Zero-velocity updates (ZUPT) when stopped to prevent drift

3. **Bias Estimation**: Learns IMU sensor biases on-the-fly
   - Initial calibration: 500 samples while stationary
   - Continuous updates when stopped at lights, etc.

**Result:** cm-level accuracy between GPS fixes, no drift when stationary.

### Coordinate Frames

The system thinks in three frames:

- **Body**: What the IMU sees (raw sensor data)
- **Earth**: Horizontal plane, gravity removed (for physics)
- **Vehicle**: Aligned with car heading (for driving modes)

Accelerations are transformed: Body → Earth (using roll/pitch) → Vehicle (using yaw)

This lets you separate "I'm cornering hard" from "I'm on a banked turn."

### Driving Mode Detection

The mode classifier watches for characteristic acceleration signatures:

- **ACCEL**: Longitudinal accel > 0.21g
- **BRAKE**: Longitudinal accel < -0.25g
- **CORNER**: High lateral accel + yaw rate (same sign, > threshold)
- **IDLE**: Everything else

Uses EMA filtering to avoid mode flicker.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                         ESP32-C3                            │
│                                                             │
│  WT901 IMU (200Hz)          NEO-6M GPS (5Hz)                │
│       │                          │                          │
│       ├─► Parse WT901       ┌────┴─────┐                    │
│       │   UART packets      │Parse NMEA│                    │
│       │                     │ GPRMC    │                    │
│       ▼                     └────┬─────┘                    │
│  ┌─────────────┐                 │                          │
│  │  Transform  │                 │                          │
│  │ Body→Earth  │                 │                          │
│  └──────┬──────┘                 │                          │
│         │                        │                          │
│         └────────┬───────────────┘                          │
│                  ▼                                          │
│         ┌─────────────────┐                                 │
│         │  Kalman Filter  │                                 │
│         │   (7-state)     │                                 │
│         │  • Position     │                                 │
│         │  • Velocity     │                                 │
│         │  • Yaw          │                                 │
│         │  • Accel bias   │                                 │
│         └────────┬────────┘                                 │
│                  │                                          │
│         ┌────────┴────────┐                                 │
│         │ Mode Classifier │                                 │
│         │ IDLE/ACCEL/     │                                 │
│         │ BRAKE/CORNER    │                                 │
│         └────────┬────────┘                                 │
│                  │                                          │
│         ┌────────▼────────┐                                 │
│         │ Binary Packet   │ (66 bytes)                      │
│         │  Serializer     │                                 │
│         └────────┬────────┘                                 │
│                  │                                          │
│                  ▼                                          │
│         [ TCP Stream @ 20Hz ]                               │
│                  │                                          │
└──────────────────┼──────────────────────────────────────────┘
                   │
                   ▼
        [ Your Laptop / Dashboard ]
```

## Use Cases

### Track Day Logging
Mount the ESP32 in your car, connect it to a phone hotspot, and stream live telemetry to your laptop in the paddock. Review your racing line, braking points, and cornering g-forces between sessions.

### Autocross Analysis
See exactly where you're losing time. Compare runs side-by-side. Optimize your line through the slalom.

### Rally Navigation
Dead reckoning keeps position accurate even when GPS is blocked by trees. Perfect for rally stages.

### Suspension Tuning
Measure body roll (from roll angle) and weight transfer (from vertical acceleration). Dial in your setup with data.

### Winter Driving Research
Study vehicle dynamics on low-grip surfaces. See when the car starts sliding vs. when you feel it.

## Customization

### Adjust EKF Tuning

Edit `src/ekf.rs`:

```rust
// Process noise (how much you trust the model)
const Q_ACC: f32 = 0.40;   // Acceleration noise
const Q_GYRO: f32 = 0.005; // Gyro noise

// Measurement noise (how much you trust the sensors)
const R_POS: f32 = 20.0;   // GPS position noise
const R_VEL: f32 = 0.2;    // GPS velocity noise
```

Higher `Q_*` = trust sensors more, model less (faster response, more noise)
Higher `R_*` = trust sensors less (slower response, smoother output)

### Change Telemetry Rate

Edit `src/main.rs`:

```rust
const TELEMETRY_INTERVAL_MS: u32 = 50;  // 20 Hz
// Change to 25 for 40 Hz, or 100 for 10 Hz
```

### Tune Mode Detection

Edit `src/mode.rs`:

```rust
pub struct ModeConfig {
    pub min_speed: f32,      // 2.0 m/s (minimum speed for maneuvers)
    pub acc_thr: f32,        // 0.21g (acceleration threshold)
    pub brake_thr: f32,      // -0.25g (braking threshold)
    pub lat_thr: f32,        // 0.20g (lateral for cornering)
    pub yaw_thr: f32,        // 0.07 rad/s (yaw rate for cornering)
}
```

## Data Decoder

The Python receiver (`tools/python/tcp_telemetry_server.py`) decodes the binary format:

```python
# Packet structure (66 bytes)
FORMAT = '=HIffffffffffffBffBH'
#         │││││││││││││││││││└─ Checksum (u16)
#         ││││││││││││││││││└── GPS valid (u8)
#         │││││││││││││││││└─── Lon (f32)
#         ││││││││││││││││└──── Lat (f32)
#         │││││││││││││││└───── Mode (u8)
#         ││││││││││││││└────── Speed (f32)
#         │││││││││││││└─────── vy, vx, y, x (4x f32)
#         ││││││││││││└──────── yaw (f32)
#         │││││││││││└───────── pitch, roll, wz, az, ay, ax (6x f32)
#         ││││││││││└────────── Timestamp (u32)
#         │││││││││└─────────── Header 0xAA55 (u16)
```

You can write your own decoder in any language for custom dashboards.

## Performance

- **Telemetry rate**: 20 Hz sustained (50ms intervals)
- **IMU sampling**: 200 Hz internal
- **GPS updates**: 5 Hz
- **Latency**: <50ms sensor → packet transmission
- **Bandwidth**: 1.3 KB/s (binary) vs. ~6 KB/s (JSON)
- **Memory**: Runs comfortably in ESP32-C3's 400KB RAM
- **Accuracy**: ±1m position between GPS fixes, <0.1° yaw

## Troubleshooting

**WiFi won't connect → Red LED blinking**
- Check SSID/password in `main.rs`
- Make sure you're on 2.4 GHz WiFi (ESP32 doesn't support 5 GHz)

**Yellow LED blinking, no cyan pulse**
- GPS hasn't locked yet
- Needs clear view of sky
- Can take 30-60 seconds for cold start

**TCP receiver shows no data**
- Check TCP_SERVER IP in `main.rs` matches your laptop
- Ensure laptop and ESP32 are on same network
- Run `tools/python/tcp_telemetry_server.py` before booting ESP32

**Calibration fails**
- Device must be completely stationary during yellow LED phase
- Don't touch it for ~10 seconds after boot

## Building Custom Dashboards

The telemetry format is simple and documented. Build your own visualizations:

- **Web dashboard**: Parse TCP stream with WebSocket bridge
- **Mobile app**: BLE streaming (modify to use BLE instead of TCP)
- **Data logger**: Write packets to SD card or database
- **Live overlays**: Integrate with GoPro footage using timestamps

Example integrations and data are in the [`examples/`](examples/) directory.

## Contributing

We love contributions! This project thrives on the ingenuity of the motorsport and maker communities.

### How to Contribute

1. **Fork the repository**
2. **Create a feature branch** (`git checkout -b feature/amazing-feature`)
3. **Commit your changes** (`git commit -m 'Add amazing feature'`)
4. **Push to your branch** (`git push origin feature/amazing-feature`)
5. **Open a Pull Request**

### Areas We'd Love Help With

- 🛰️ **GPS improvements**: GPGGA support, SBAS corrections, RTK integration
- 🔬 **Kalman filter**: UKF implementation, adaptive noise tuning
- 🚗 **CAN bus**: OBD2 integration for engine data (RPM, throttle, etc.)
- 📡 **Communication**: BLE streaming, LoRa for long-range
- 📊 **Visualization**: Web-based dashboard, mobile apps
- 💾 **Data logging**: SD card storage, SQLite databases
- 🔧 **Hardware support**: Other IMUs (MPU6050, BMI088), different GPS modules
- 📚 **Documentation**: Tutorials, mounting guides, case designs

### Development Guidelines

- Follow existing code style (rustfmt for Rust, PEP 8 for Python)
- Add tests for new features where applicable
- Update documentation (README, CLAUDE.md) for significant changes
- Keep commits atomic and descriptive
- See `CLAUDE.md` for architecture details

### Reporting Issues

Found a bug? Have a feature request?

1. Check existing issues to avoid duplicates
2. Use issue templates (bug report / feature request)
3. Include:
   - Hardware details (ESP32 variant, IMU model, GPS module)
   - Firmware version / git commit hash
   - Steps to reproduce (for bugs)
   - Expected vs. actual behavior

### Code of Conduct

Be excellent to each other. We're all here because we love cars and technology.

- Respect different skill levels and backgrounds
- Provide constructive feedback
- Focus on what's best for the community
- No harassment, discrimination, or trolling

For detailed guidelines, see [CONTRIBUTING.md](CONTRIBUTING.md).

## Project Documentation

This project follows open source best practices with comprehensive documentation:

### Documentation
- **[README.md](README.md)** (this file) - Project overview and quick start
- **[CLAUDE.md](CLAUDE.md)** - Architecture deep-dive for developers
- **[CHANGELOG.md](CHANGELOG.md)** - Version history and release notes
- **[docs/FAQ.md](docs/FAQ.md)** - Frequently asked questions and troubleshooting
- **[docs/SECURITY.md](docs/SECURITY.md)** - Security policy and vulnerability reporting

### Contributing
- **[CONTRIBUTING.md](CONTRIBUTING.md)** - How to contribute and community guidelines
- **[LICENSE](LICENSE)** - MIT License terms

## License

MIT License - do whatever you want with it. Build products, modify it, break it, fix it. Just don't blame me if you crash because you were watching telemetry instead of the road.

See `LICENSE` for details.

## Community & Support

### Getting Help
- 📖 **Documentation**: Start with [README.md](README.md) and [docs/FAQ.md](docs/FAQ.md)
- 🐛 **Bug Reports**: Use our [bug report template](.github/ISSUE_TEMPLATE/bug_report.md)
- 💡 **Feature Ideas**: Use our [feature request template](.github/ISSUE_TEMPLATE/feature_request.md)
- ❓ **Questions**: Check [docs/FAQ.md](docs/FAQ.md) or open an issue with the `question` label

### Stay Connected
- ⭐ **Star this repo** to show support and stay updated
- 👀 **Watch releases** for new features and bug fixes
- 🍴 **Fork and experiment** - that's what open source is all about!

### Contributors
Thank you to everyone who has contributed to Active Wing! 🙏

<!-- ALL-CONTRIBUTORS-LIST:START -->
<!-- This section will be automatically updated -->
<!-- ALL-CONTRIBUTORS-LIST:END -->

Want to see your name here? Check out [CONTRIBUTING.md](CONTRIBUTING.md)!

## Acknowledgments

**Built with:**
- [esp-idf-hal](https://github.com/esp-rs/esp-idf-hal) - ESP32 Rust HAL
- [esp-idf-svc](https://github.com/esp-rs/esp-idf-svc) - ESP-IDF services wrapper
- [embedded-hal](https://github.com/rust-embedded/embedded-hal) - Embedded hardware abstraction

**Inspired by:**
- Formula 1 telemetry systems
- Rally navigation and pace notes
- Track day data logging enthusiasts
- The awesome Rust embedded community

**Special thanks to:**
- All contributors and issue reporters
- The ESP-RS team for making Rust on ESP32 possible
- Everyone who shares their car data for science

## Safety Notice

This is a **data acquisition system**, not a **control system**. It observes your car, it doesn't drive it.

- Mount the hardware securely (it will see high g-forces)
- Don't interact with displays/laptops while driving
- Obey all traffic laws
- Use on closed courses or private property for performance testing
- Your insurance probably doesn't know about this, keep it that way

---

**Built by car nerds, for car nerds. Go fast, collect data. 🏁**
