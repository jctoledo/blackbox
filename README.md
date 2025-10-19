# Open Motorsport Telemetry

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Rust](https://img.shields.io/badge/rust-1.75%2B-orange.svg)](https://www.rust-lang.org/)
[![ESP32](https://img.shields.io/badge/platform-ESP32-blue.svg)](https://www.espressif.com/)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](CONTRIBUTING.md)

**Professional-grade vehicle telemetry on hobbyist hardware.** Real-time sensor fusion using Extended Kalman Filtering, built for racing, research, and DIY vehicle projects.

---

## Table of Contents

- [What Is This?](#what-is-this)
- [Quick Start](#quick-start)
- [Active Wing Reference System](#active-wing-reference-system)
- [Building From Source](#building-from-source)
- [Architecture](#architecture)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [License](#license)

---

## What Is This?

An open-source framework for building custom automotive sensor systems. Track your vehicle's dynamics with the same algorithms used in autonomous vehicles—for $50 instead of $5,000.

**What you get:**
- 7-state Extended Kalman Filter with GPS+IMU sensor fusion
- Real-time position, velocity, and acceleration tracking
- Driving mode detection (stopped, cruising, braking, cornering)
- 20Hz binary telemetry streaming
- Python visualization tools

**Use cases:**
- Track day data logging and driver training
- Vehicle dynamics research and education
- DIY EV builds and kit cars
- Suspension/brake/aero testing
- Autonomous vehicle prototyping

**Why this exists:** Commercial racing telemetry costs $5k-50k. Consumer GPS loggers don't do sensor fusion. This bridges the gap with professional algorithms on accessible hardware.

---

## Quick Start

### Hardware You Need

**Core system (~$50):**
- ESP32-C3 development board ($5)
- WT901 9-axis IMU ($25)
- NEO-6M GPS module ($15)
- USB cable + basic wiring

**Optional:**
- WS2812 RGB LED for status ($2)
- SD card module for offline logging

### Software Setup

```bash
# 1. Install Rust and ESP toolchain
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
cargo install espup cargo-espflash
espup install

# 2. Clone and configure
git clone https://github.com/jctoledo/open-motorsport-telemetry.git
cd open-motorsport-telemetry/sensors/active-wing
cp config.toml.example config.toml
# Edit config.toml with your WiFi credentials

# 3. Build and flash
source $HOME/export-esp.sh
cargo espflash flash --release --monitor
```

### Receive Telemetry

```bash
# In another terminal
cd tools/python
pip install paho-mqtt
python3 tcp_telemetry_server.py
```

You'll see live telemetry at 20Hz. Drive around and watch the data!

---

## Active Wing Reference System

A complete working example showing the framework in action:

**Hardware:** ESP32-C3 + WT901 IMU (200Hz) + NEO-6M GPS (5Hz)

**What it does:**
- Fuses GPS and IMU using Extended Kalman Filter
- Eliminates IMU drift using GPS corrections
- Compensates for gravity to measure true acceleration
- Transforms between body frame and earth frame
- Detects zero-velocity for bias estimation
- Streams 66-byte binary packets at 20Hz over TCP

**Performance:**
- Position accuracy: ±1m between GPS updates
- Update latency: <20ms sensor-to-transmission
- Memory footprint: ~50KB RAM, ~800KB flash

See [`sensors/active-wing/README.md`](sensors/active-wing/README.md) for complete details.

---

## Building From Source

### Prerequisites

<details>
<summary><b>Click to expand full prerequisites</b></summary>

**1. Install Rust:**
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

**2. Install ESP-IDF dependencies:**

*Ubuntu/Debian:*
```bash
sudo apt-get install git wget flex bison gperf python3 python3-pip \
  python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

*macOS:*
```bash
brew install cmake ninja dfu-util
```

*Windows:* Install [Visual Studio Build Tools](https://visualstudio.microsoft.com/downloads/), [Python 3](https://www.python.org/downloads/), and [Git](https://git-scm.com/download/win)

**3. Install ESP Rust toolchain:**
```bash
cargo install espup cargo-espflash
espup install
source $HOME/export-esp.sh  # Load environment (do this in each new terminal)
```

</details>

### Build Commands

```bash
# Load ESP environment (required in each terminal session)
source $HOME/export-esp.sh

# Check compilation
cargo check --target riscv32imc-esp-espidf

# Build release binary
cargo build --release --target riscv32imc-esp-espidf

# Flash to ESP32
cargo espflash flash --release --monitor

# Format code
cargo fmt

# Lint
cargo clippy --target riscv32imc-esp-espidf
```

### Troubleshooting

<details>
<summary><b>Common build issues</b></summary>

**"error: linking with `riscv32-esp-elf-gcc` failed"**
```bash
source $HOME/export-esp.sh  # Load ESP environment
```

**"CMake Error: Could not find CMAKE_C_COMPILER"**
```bash
sudo apt-get install build-essential cmake
```

**"error: failed to run custom build command for `esp-idf-sys`"**
```bash
cargo clean
rm -rf ~/.espressif  # Remove cached ESP-IDF
espup install        # Reinstall
cargo build
```

**Permission denied accessing serial port (Linux):**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

</details>

**Build times:** First build 10-20 min (downloads ESP-IDF), incremental builds 30s-2min.

---

## Architecture

### System Overview

```
┌─────────────────────────────────┐
│  Your Sensor Project            │
│  • Hardware drivers             │
│  • Application logic            │
└────────────┬────────────────────┘
             │ uses
             ▼
┌─────────────────────────────────┐
│  Sensor Fusion Framework        │
│  ├─ Sensor abstractions         │
│  ├─ EKF (7-state)               │
│  ├─ Coordinate transforms       │
│  └─ Plugin system               │
└─────────────────────────────────┘
```

### Repository Structure

```
open-motorsport-telemetry/
├── framework/              # Reusable sensor fusion components
│   ├── ekf.rs             # Extended Kalman Filter
│   ├── transforms.rs      # Coordinate frame math
│   └── sensor_framework.rs # Plugin architecture
│
├── drivers/               # Standalone sensor drivers (no-std)
│   ├── wt901/            # WT901 IMU UART parser
│   └── neo6m/            # NEO-6M GPS NMEA parser
│
├── sensors/              # Complete sensor projects
│   └── active-wing/      # ESP32-C3 reference implementation
│       ├── src/          # Application code
│       ├── tools/python/ # Telemetry receivers
│       └── docs/         # Project-specific docs
│
└── docs/                 # Framework documentation
```

**Package roles:**
- `framework/` - Reusable fusion algorithms
- `drivers/` - Hardware-agnostic sensor parsers
- `sensors/active-wing/` - Complete working example
- Your new project - Add to `sensors/` directory

### Key Technologies

- **Extended Kalman Filter:** Fuses GPS and IMU optimally
- **CTRA Motion Model:** Handles turning dynamics properly  
- **ZUPT:** Eliminates drift when stationary
- **Binary Protocol:** Efficient 66-byte packets with checksums

---

## Documentation

- **[Active Wing System](sensors/active-wing/README.md)** - Complete reference implementation
- **[Sensor Toolkit Guide](docs/SENSOR_TOOLKIT_GUIDE.md)** - Add your own sensors
- **[Architecture Guide](docs/ARCHITECTURE.md)** - System design deep-dive
- **[FAQ](docs/FAQ.md)** - Common questions and troubleshooting
- **[Contributing Guide](CONTRIBUTING.md)** - How to contribute

---

## Contributing

We welcome contributions! Whether it's:
- New sensor drivers (wheel speed, CAN bus, cameras)
- Platform support (ESP32-S3, Raspberry Pi, STM32)
- Documentation improvements
- Bug reports and feature requests

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

---

## License

MIT License - use it, modify it, sell it, race with it. See [LICENSE](LICENSE) for details.

---

## Acknowledgments

Built by racing enthusiasts who wanted pro-grade telemetry without the pro price tag. Inspired by projects like RaceCapture, openpilot, and VESC that proved open-source can compete with commercial systems.

**Questions?** Open an [issue](https://github.com/jctoledo/open-motorsport-telemetry/issues) or [discussion](https://github.com/jctoledo/open-motorsport-telemetry/discussions).

**Ready to track your performance?** [Start building →](#quick-start)
