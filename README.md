# Blackbox - ESP32 Vehicle Telemetry System

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Rust](https://img.shields.io/badge/rust-1.75%2B-orange.svg)](https://www.rust-lang.org/)
[![ESP32-C3](https://img.shields.io/badge/platform-ESP32--C3-blue.svg)](https://www.espressif.com/)

**Real-time vehicle dynamics tracking using GPS+IMU sensor fusion.** Professional-grade algorithms on $50 hardware.

---

## What Is This?

A complete ESP32-C3 firmware that fuses GPS and IMU data to track your vehicle's position, velocity, and acceleration in real-time. Built for track day logging, vehicle dynamics research, and DIY automotive projects.

**What it does:**
- Tracks position (±1m accuracy between GPS updates)
- Measures velocity in 2D (earth frame)
- Calculates true acceleration (gravity compensated)
- Detects driving modes (idle, accelerating, braking, cornering)
- Streams telemetry at 20Hz over TCP or MQTT

**How it works:**
- 7-state Extended Kalman Filter fuses GPS (5Hz) and IMU (50Hz)
- Gyro integrates yaw between GPS updates to maintain heading
- Body-frame accelerations transformed to earth frame using current orientation
- Zero-velocity updates eliminate IMU drift when stationary
- Constant Turn Rate and Acceleration (CTRA) model for cornering

**Why it's useful:**
- Track day data logging without $5k commercial systems
- Vehicle dynamics research and education
- DIY EV/kit car development
- Test suspension, brakes, and aerodynamics
- Learn sensor fusion practically

---

## Hardware Requirements

### Core Components ($50 total)

| Component | Cost | Purpose | Where to Buy |
|-----------|------|---------|--------------|
| ESP32-C3 DevKit | $5 | Main controller | AliExpress, Amazon |
| WT901 IMU | $25 | 9-axis motion sensor | WitMotion store |
| NEO-6M GPS | $15 | Position/velocity | AliExpress, Amazon |
| Wires + USB cable | $5 | Connections | - |

### Optional Add-ons

- **WS2812 RGB LED** ($2) - Visual status indicator
- **SD card module** ($5) - Offline data logging (not yet implemented)
- **CAN transceiver** ($10) - Read vehicle data (future feature)

### Wiring Diagram

```
ESP32-C3 DevKitC-02          WT901 IMU               NEO-6M GPS
┌─────────────────┐          ┌──────────┐            ┌──────────┐
│                 │          │          │            │          │
│ 3V3 ────────────┼──────────┤ VCC      │            │          │
│ GND ────────────┼──────────┤ GND      │            │          │
│ GPIO19 (RX) ────┼──────────┤ TX       │            │          │
│ GPIO18 (TX) ────┼──────────┤ RX       │            │          │
│                 │          └──────────┘            │          │
│ 3V3 ────────────┼──────────────────────────────────┤ VCC      │
│ GND ────────────┼──────────────────────────────────┤ GND      │
│ GPIO4  (RX) ────┼──────────────────────────────────┤ TX       │
│ GPIO5  (TX) ────┼──────────────────────────────────┤ RX       │
│                 │                                  └──────────┘
│ GPIO8 ──────────┼── (Optional: WS2812 LED data pin)
│                 │
└─────────────────┘
```

**Important:** GPS uses hardware UART0 (the same as USB console). During GPS operation, you won't see serial output unless using a separate USB-JTAG debugger. This is by design to free up the UART for GPS.

---

## Quick Start

### 1. Install Rust Toolchain

```bash
# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env

# Install ESP32 tools
cargo install espup cargo-espflash
espup install

# Load ESP environment (required in EVERY new terminal)
source $HOME/export-esp.sh

# Add rust-src component (required for ESP32)
rustup component add rust-src --toolchain esp
```

### 2. Clone and Configure

```bash
git clone https://github.com/jctoledo/blackbox.git
cd blackbox/sensors/blackbox

# Edit WiFi credentials
nano src/main.rs
# Change these lines:
# const WIFI_SSID: &str = "YourNetworkName";
# const WIFI_PASSWORD: &str = "YourPassword";
# const MQTT_BROKER: &str = "mqtt://192.168.1.100:1883";  // Your broker IP
# const TCP_SERVER: &str = "192.168.1.100:9000";          // Your server IP
```

### 3. Build and Flash

The project includes a `.cargo/config.toml` that configures everything automatically.

```bash
# Navigate to the sensor project
cd sensors/blackbox

# Load ESP environment first (in every new terminal!)
source $HOME/export-esp.sh

# Quick check (fast, no linking)
cargo check

# Build firmware
cargo build --release

# Build and flash in one command
cargo run --release
```

**First build takes 10-20 minutes** as it downloads ESP-IDF. Subsequent builds: 30s-2min.

### 4. Receive Telemetry

**Option A: TCP Stream (recommended for high-rate data)**

```bash
# In another terminal, from repo root
cd tools/python
python3 tcp_telemetry_server.py
```

Receives 20Hz binary telemetry and displays:
```
[20Hz] Spd: 45.3km/h Pos:(123.4,456.7)m Vel:(+12.58,+0.32)m/s Acc:(+1.23,-0.15,+9.81)m/s² Yaw:+45° wz:+12°/s CORNER
```

**Option B: MQTT (for status messages and debugging)**

```bash
# Requires mosquitto or another MQTT broker
mosquitto_sub -h 192.168.1.100 -t 'car/#' -v
```

---

## Building From Source

### Prerequisites

<details>
<summary><b>Ubuntu/Debian Linux</b></summary>

```bash
# System dependencies
sudo apt-get update
sudo apt-get install -y git wget flex bison gperf python3 python3-pip \
  python3-venv cmake ninja-build ccache libffi-dev libssl-dev \
  dfu-util libusb-1.0-0 build-essential

# Rust and ESP toolchain
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
cargo install espup cargo-espflash
espup install
source $HOME/export-esp.sh
rustup component add rust-src --toolchain esp
```

</details>

<details>
<summary><b>macOS</b></summary>

```bash
# Install Homebrew if needed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# System dependencies
brew install cmake ninja dfu-util

# Rust and ESP toolchain
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
cargo install espup cargo-espflash
espup install
source $HOME/export-esp.sh
rustup component add rust-src --toolchain esp
```

</details>

<details>
<summary><b>Windows</b></summary>

1. Install [Visual Studio Build Tools](https://visualstudio.microsoft.com/downloads/) (C++ development)
2. Install [Python 3](https://www.python.org/downloads/)
3. Install [Git](https://git-scm.com/download/win)
4. Install Rust via [rustup-init.exe](https://rustup.rs/)
5. Open PowerShell and run:

```powershell
cargo install espup cargo-espflash
espup install
# Restart terminal to load environment
rustup component add rust-src --toolchain esp
```

</details>

### Build Commands

The `.cargo/config.toml` file in the sensor project configures the target and build settings automatically:

```bash
# Navigate to the sensor project first
cd sensors/blackbox

# ALWAYS load ESP environment first!
source $HOME/export-esp.sh

# Quick check (no linking, fast)
cargo check

# Full build
cargo build --release

# Build and flash with monitor
cargo run --release

# Format code
cargo fmt

# Lint
cargo clippy -- -D warnings

# Clean build (if things go wrong)
cargo clean
```

**No need to specify `--target` or `-Zbuild-std` - it's all configured!**

### Common Build Issues

<details>
<summary><b>Click to see troubleshooting</b></summary>

**Error: "can't find crate for `core`" or "can't find crate for `std`"**

This means the ESP toolchain isn't loaded or rust-src is missing:

```bash
# Solution 1: Make sure you're in the sensor directory
cd sensors/blackbox

# Solution 2: Load ESP environment
source $HOME/export-esp.sh

# Solution 3: Add rust-src component
rustup component add rust-src --toolchain esp

# Solution 4: Verify .cargo/config.toml exists
cat .cargo/config.toml
# Should contain:
# [unstable]
# build-std = ["std", "panic_abort"]
```

**Error: "linking with `riscv32-esp-elf-gcc` failed"**
```bash
source $HOME/export-esp.sh  # You forgot this!
```

**Error: "CMake not found"**
```bash
# Ubuntu/Debian
sudo apt-get install cmake ninja-build

# macOS
brew install cmake ninja
```

**Error: "Permission denied" on serial port (Linux)**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for group change to take effect
```

**Build hangs or crashes**
```bash
cargo clean
rm -rf ~/.espressif
espup install
source $HOME/export-esp.sh
rustup component add rust-src --toolchain esp
cargo build --release
```

**Error: "espflash: command not found"**
```bash
cargo install cargo-espflash
```

</details>

---

## System Architecture

### Data Flow

```
GPS (5Hz)           IMU (50Hz)           
   │                   │                  
   │ NMEA              │ Binary packets    
   │ sentences         │                   
   ▼                   ▼                   
┌─────────────────────────────────┐       
│   Sensor Parsers                │       
│   • Parse lat/lon/speed         │       
│   • Parse ax,ay,az,wx,wy,wz     │       
│   • Parse roll,pitch,yaw        │       
└────────────┬────────────────────┘       
             │                            
             ▼                            
┌─────────────────────────────────┐       
│   Coordinate Transforms         │       
│   • Remove gravity from accel   │       
│   • Body → Earth frame          │       
└────────────┬────────────────────┘       
             │                            
             ▼                            
┌─────────────────────────────────┐       
│   Extended Kalman Filter        │       
│   State: [x, y, ψ, vx, vy,      │       
│           bias_ax, bias_ay]     │       
│                                 │       
│   • Predict using IMU (50Hz)    │       
│   • Update using GPS (5Hz)      │       
│   • ZUPT when stationary        │       
└────────────┬────────────────────┘       
             │                            
             ▼                            
┌─────────────────────────────────┐       
│   Mode Classifier               │       
│   • Detect: IDLE, ACCEL,        │       
│     BRAKE, CORNER               │       
└────────────┬────────────────────┘       
             │                            
             ▼                            
┌─────────────────────────────────┐       
│   Binary Telemetry (20Hz)       │       
│   • 66 bytes with checksum      │       
│   • TCP stream or MQTT          │       
└─────────────────────────────────┘       
```

### File Structure

```
blackbox/
├── sensors/
│   └── blackbox/
│       ├── .cargo/
│       │   └── config.toml       # ESP32-C3 build configuration
│       ├── src/
│       │   ├── main.rs            # Main loop and setup
│       │   ├── imu.rs             # WT901 UART parser
│       │   ├── gps.rs             # NMEA parser with warmup
│       │   ├── ekf.rs             # 7-state Extended Kalman Filter
│       │   ├── transforms.rs      # Body↔Earth coordinate math
│       │   ├── mode.rs            # Driving mode classifier
│       │   ├── binary_telemetry.rs # 66-byte packet format
│       │   ├── tcp_stream.rs      # High-speed TCP client
│       │   ├── mqtt.rs            # MQTT client for status
│       │   ├── wifi.rs            # WiFi connection manager
│       │   └── rgb_led.rs         # WS2812 status LED
│       ├── Cargo.toml             # Rust dependencies
│       ├── sdkconfig.defaults     # ESP-IDF configuration
│       └── build.rs               # Build script
├── tools/
│   └── python/
│       ├── tcp_telemetry_server.py  # Python receiver (TCP)
│       └── mqtt_decoder.py          # Python receiver (MQTT)
└── README.md                        # This file
```

### Key Algorithms

**Extended Kalman Filter (ekf.rs)**
- 7-state vector: `[x, y, ψ, vx, vy, bias_ax, bias_ay]`
- Prediction: IMU accelerations integrated to update velocity and position
- Update: GPS position/velocity fused with Kalman gain
- CTRA motion model when speed > 2 m/s and turning

**Zero-Velocity Update (main.rs)**
- Detects stationary: `|accel| < 0.18g && |yaw_rate| < 12°/s && GPS_speed < 3.5m/s`
- Forces velocity to zero
- Estimates IMU biases from residual accelerations
- Eliminates drift over time

**Mode Detection (mode.rs)**
- **IDLE**: Low acceleration, low speed
- **ACCEL**: Longitudinal accel > 0.21g
- **BRAKE**: Longitudinal accel < -0.25g  
- **CORNER**: Lateral accel > 0.20g AND yaw rate > 0.07 rad/s AND same sign

---

## Telemetry Format

### Binary Packet (66 bytes)

```rust
struct TelemetryPacket {
    header: u16,        // 0xAA55 magic
    timestamp_ms: u32,
    
    ax, ay, az: f32,    // Acceleration (m/s²)
    wz: f32,            // Yaw rate (rad/s)
    roll, pitch: f32,   // Orientation (rad)
    
    yaw: f32,           // EKF yaw (rad)
    x, y: f32,          // Position (m, ENU frame)
    vx, vy: f32,        // Velocity (m/s)
    speed_kmh: f32,     // Display speed
    mode: u8,           // 0=IDLE, 1=ACCEL, 2=BRAKE, 3=CORNER
    
    lat, lon: f32,      // GPS (degrees)
    gps_valid: u8,      // 0=no fix, 1=valid
    
    checksum: u16,      // Sum of all bytes (excl. checksum)
}
```

### MQTT Topics

- `car/telemetry_bin` - Binary packets (if using MQTT instead of TCP)
- `car/status` - JSON status messages
- `car/config` - Configuration updates
- `car/gps_raw` - NMEA sentences (debug only)

---

## Configuration

### WiFi and Network

Edit `sensors/blackbox/src/main.rs`:
```rust
const WIFI_SSID: &str = "YourNetwork";
const WIFI_PASSWORD: &str = "YourPassword";
const MQTT_BROKER: &str = "mqtt://192.168.1.100:1883";
const TCP_SERVER: &str = "192.168.1.100:9000";
```

### EKF Tuning

Edit `sensors/blackbox/src/ekf.rs`:
```rust
const Q_ACC: f32 = 0.40;    // Process noise: acceleration (m/s²)²
const Q_GYRO: f32 = 0.005;  // Process noise: gyro (rad/s)²
const Q_BIAS: f32 = 1e-3;   // Process noise: bias drift (m/s²)²

const R_POS: f32 = 20.0;    // Measurement noise: GPS position (m)²
const R_VEL: f32 = 0.2;     // Measurement noise: GPS velocity (m/s)²
const R_YAW: f32 = 0.10;    // Measurement noise: magnetometer (rad)²
```

### Mode Detection Thresholds

Edit `sensors/blackbox/src/mode.rs`:
```rust
pub min_speed: f32 = 2.0;      // Minimum speed for maneuvers (m/s)
pub acc_thr: f32 = 0.21;       // Acceleration threshold (g)
pub brake_thr: f32 = -0.25;    // Braking threshold (g)
pub lat_thr: f32 = 0.20;       // Lateral accel threshold (g)
pub yaw_thr: f32 = 0.07;       // Yaw rate threshold (rad/s)
```

---

## Performance

| Metric | Value |
|--------|-------|
| Update rate | 50 Hz (IMU predict) / 5 Hz (GPS update) |
| Telemetry rate | 20 Hz (TCP stream) |
| Position accuracy | ±1-2m (GPS dependent) |
| Velocity accuracy | ±0.2 m/s |
| Latency | <20ms sensor-to-transmission |
| Memory usage | ~50KB RAM, ~800KB flash |
| Power | ~0.5W @ 5V |

---

## LED Status Codes

| Pattern | Meaning |
|---------|---------|
| 3 blue blinks | Boot sequence |
| 5 green blinks | WiFi connected |
| Solid cyan 2s on, 2s off | GPS locked |
| Yellow flashing | Waiting for GPS fix |
| Rapid red flashing | Error (WiFi/MQTT failed) |

---

## Future Enhancements

- [ ] SD card logging for offline operation
- [ ] Web dashboard for live visualization
- [ ] CAN bus integration for vehicle data
- [ ] Support for external wheel speed sensors
- [ ] Bluetooth for phone connectivity
- [ ] Over-the-air firmware updates

---

## Contributing

Contributions welcome! Whether it's:
- Bug fixes
- New sensor drivers
- Documentation improvements
- Performance optimizations

Open an issue or pull request on GitHub.

---

## License

MIT License - see [LICENSE](LICENSE) for details.

---

## Acknowledgments

Built with:
- [esp-idf-hal](https://github.com/esp-rs/esp-idf-hal) - Rust ESP32 HAL
- [esp-idf-svc](https://github.com/esp-rs/esp-idf-svc) - ESP-IDF services
- WT901 IMU from WitMotion
- U-blox NEO-6M GPS module

Inspired by open-source motorsport and robotics projects that prove pro-grade systems don't need pro prices.

---

**Questions?** Open an [issue](https://github.com/jctoledo/blackbox/issues).

**Ready to track?** [Start with Quick Start ↑](#quick-start)
