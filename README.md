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
- Streams telemetry at 20-30Hz over WiFi
- **Built-in mobile dashboard** - view live data on your phone

**How it works:**
- 7-state Extended Kalman Filter fuses GPS (5Hz) and IMU (50Hz)
- Gyro integrates yaw between GPS updates to maintain heading
- Body-frame accelerations transformed to earth frame using current orientation
- Zero-velocity updates eliminate IMU drift when stationary
- Constant Turn Rate and Acceleration (CTRA) model for cornering

**Why it's useful:**
- Track day data logging without $500+ commercial systems
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

### Option A: Web Flasher (Easiest)

Flash directly from your browser - no toolchain required:

1. Go to **https://jctoledo.github.io/blackbox/**
2. Connect your ESP32-C3 via USB
3. Select "Blackbox" from the dropdown
4. Click "Connect Device" and select your serial port
5. Click "Flash Firmware"

**Requirements:** Chrome, Edge, or Opera (Web Serial API)

**Note:** You may need to hold the BOOT button when connecting to enter flash mode.

---

### Option B: Build from Source

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

**UDP Telemetry (recommended)**

```bash
# In another terminal, from repo root
cd tools/python
python3 udp_telemetry_server.py
```

Receives 20Hz binary telemetry and displays:
```
[20Hz] Spd: 45.3km/h Pos:(123.4,456.7)m Vel:(+12.58,+0.32)m/s Acc:(+1.23,-0.15,+9.81)m/s² Yaw:+45° wz:+12°/s CORNER
```

**MQTT (for status messages and debugging)**

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
│   • 67 bytes with checksum      │
│   • UDP stream                  │       
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
│       │   ├── binary_telemetry.rs # 67-byte packet format
│       │   ├── websocket_server.rs # Mobile dashboard & WebSocket server
│       │   ├── udp_stream.rs      # High-speed UDP client
│       │   ├── mqtt.rs            # MQTT client for status
│       │   ├── wifi.rs            # WiFi connection manager
│       │   └── rgb_led.rs         # WS2812 status LED
│       ├── Cargo.toml             # Rust dependencies
│       ├── sdkconfig.defaults     # ESP-IDF configuration
│       └── build.rs               # Build script
├── tools/
│   └── python/
│       ├── udp_telemetry_server.py  # Python receiver (UDP)
│       └── mqtt_binary_decoder.py   # Python receiver (MQTT)
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
- **IDLE**: Low acceleration or below minimum speed
- **ACCEL**: Longitudinal accel exceeds threshold (configurable via presets)
- **BRAKE**: Longitudinal decel exceeds threshold (configurable via presets)
- **CORNER**: Lateral accel AND yaw rate exceed thresholds, same sign
- **ACCEL+CORNER**: Trail throttle / corner exit
- **BRAKE+CORNER**: Trail braking / corner entry

Thresholds are adjustable via the dashboard's **Driving Preset** selector with 4 built-in profiles plus custom tuning.

---

## Telemetry Format

### Binary Packet (67 bytes)

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
    mode: u8,           // 0=IDLE, 1=ACCEL, 2=BRAKE, 4=CORNER, 5=ACCEL+CORNER, 6=BRAKE+CORNER
    
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

Mode detection thresholds can be configured **live from the mobile dashboard** using the Driving Preset selector. Choose from 4 built-in presets or create custom settings:

| Preset | Accel | Brake | Lateral | Yaw | Min Speed | Best For |
|--------|-------|-------|---------|-----|-----------|----------|
| **Track** | 0.30g | 0.50g | 0.50g | 0.15 rad/s | 3.0 m/s | Racing, autocross |
| **Canyon** | 0.20g | 0.35g | 0.30g | 0.10 rad/s | 2.5 m/s | Spirited twisty roads |
| **City** | 0.10g | 0.18g | 0.12g | 0.05 rad/s | 2.0 m/s | Daily driving (default) |
| **Highway** | 0.08g | 0.15g | 0.10g | 0.04 rad/s | 4.0 m/s | Cruise, subtle inputs |
| **Custom** | User-defined via sliders | | | | | Fine-tuning |

Each threshold has an **entry** and **exit** value (hysteresis) to prevent oscillation. Exit thresholds are typically 50% of entry values.

**Preset Selection:**
1. Open dashboard at `http://192.168.4.1`
2. Scroll to "Driving Preset" section
3. Tap a preset button - settings apply immediately
4. Tap "Custom" to access individual sliders for fine-tuning

**Note:** Mode detection requires `speed > min_speed`. This prevents false detection from IMU noise when stationary. Higher min_speed on Highway preset filters out parking lot maneuvers.

---

## Performance

| Metric | Value |
|--------|-------|
| Update rate | 50 Hz (IMU predict) / 5 Hz (GPS update) |
| Telemetry rate | 20 Hz (UDP stream) |
| Position accuracy | ±1-2m (GPS dependent) |
| Velocity accuracy | ±0.2 m/s |
| Latency | <20ms sensor-to-transmission |
| Memory usage | ~50KB RAM, ~800KB flash |
| Power | ~0.5W @ 5V |

---

## LED Status Codes

### Boot Sequence (Access Point Mode)
| Pattern | Meaning |
|---------|---------|
| 3 magenta blinks | Boot sequence started (AP mode) |
| 5 green blinks | WiFi AP started |
| 3 cyan blinks | HTTP server ready |
| Yellow pulses | IMU calibration in progress |
| Continuous red blink | Critical error (WiFi AP failed) |

### Boot Sequence (Station Mode)
| Pattern | Meaning |
|---------|---------|
| 3 blue blinks | Boot sequence started (Station mode) |
| 5 green blinks | WiFi connected to network |
| 3 magenta blinks | MQTT connected |
| 5 fast red blinks | MQTT connection failed (continuing without MQTT) |
| 3 cyan blinks | UDP socket ready |
| Yellow pulses | IMU calibration in progress |
| Continuous red blink | Critical error (WiFi connection failed) |

### Main Loop (operational)
| Pattern | Meaning |
|---------|---------|
| Cyan pulse (2s cycle) | GPS locked, system operational |
| Yellow fast blink | Waiting for GPS fix |
| 3 green-white flashes | Settings changed via dashboard |
| 3 orange blinks | WiFi disconnected (repeats every 5s) |
| 2 red blinks | MQTT disconnected (Station mode only, repeats every 5s) |

**Note:** MQTT status LED only applies to Station mode. In Access Point mode, MQTT is not used and no red blinks will occur for MQTT status.

---

## Mobile Dashboard

The firmware includes a built-in web dashboard that runs directly on the ESP32. No external server needed - just connect your phone and view live telemetry.

### How to Use

1. **Power on the device** - it creates a WiFi network called "Blackbox"
2. **Connect your phone** to the "Blackbox" WiFi (password: `blackbox123`)
3. **Open a browser** and go to `http://192.168.4.1`
4. **View live telemetry** - data streams at ~30Hz via WebSocket

### Dashboard Features

| Feature | Description |
|---------|-------------|
| **G-meter** | Real-time lateral/longitudinal G display with trail |
| **Max G values** | Tracks peak L/R/Accel/Brake G-forces |
| **Speed** | Current speed and session max speed |
| **Driving mode** | IDLE, ACCEL, BRAKE, CORNER, or combined states (ACCEL+CORNER, BRAKE+CORNER) |
| **Driving presets** | One-tap presets for Track, Canyon, City, Highway, or Custom tuning |
| **Session timer** | Time since dashboard loaded |
| **GPS status** | Current coordinates and fix status |
| **Recording** | Capture data locally for CSV export |

### Controls

| Action | Effect |
|--------|--------|
| **CLR button** | Clear max G values |
| **Tap max speed** | Reset max speed only |
| **Double-tap G-meter** | Reset max G values only |
| **Preset buttons** | Switch driving profile (Track/Canyon/City/Highway/Custom) |
| **Reset button** | Reset sliders to current preset defaults (Custom mode only) |
| **Apply button** | Send custom slider values to device (Custom mode only) |
| **REC button** | Start/stop recording data |
| **Export button** | Download recorded data as CSV |

### Connection Status

The indicator in the top-right shows connection quality:
- **Blue dot + "WS"** - WebSocket connected (~30Hz, best)
- **Green dot + "HTTP"** - HTTP polling fallback (~15Hz)
- **Gray dot + "Offline"** - No connection

### WiFi Modes

The firmware supports two WiFi modes, each with different connectivity options:

#### Access Point Mode (Default)

**Best for:** Mobile use, track days, standalone operation

```
┌──────────────┐     WiFi: "Blackbox"      ┌──────────────┐
│   ESP32      │◄─────────────────────────►│    Phone     │
│  (creates    │     192.168.4.1:80        │  (connects)  │
│   network)   │                           │              │
└──────────────┘                           └──────────────┘
```

- ESP32 creates its own WiFi network: `Blackbox` / `blackbox123`
- Connect your phone directly to view the dashboard
- No router or internet required
- Data delivered via WebSocket (fast) or HTTP polling (fallback)
- **No Python tools** - dashboard only

**To use (default, no changes needed):**
```bash
cargo build --release
```

#### Station Mode

**Best for:** Home testing, data logging to laptop, multiple receivers

```
┌──────────────┐                           ┌──────────────┐
│   ESP32      │     Your WiFi Network     │   Laptop     │
│  (connects   │◄─────────────────────────►│  (Python     │
│   to router) │                           │   tools)     │
└──────────────┘                           └──────────────┘
        │                                         │
        │              ┌──────────┐              │
        └──────────────►│  Router  │◄────────────┘
                       └──────────┘
```

- ESP32 connects to your existing WiFi network
- Telemetry sent via **UDP** to your laptop (high-speed, 20Hz)
- Status messages via **MQTT** broker
- Python tools can receive and log data
- Dashboard NOT available (no web server in this mode)

**To use:**
```bash
# Set environment variables before building
export WIFI_MODE="station"
export WIFI_SSID="YourHomeNetwork"
export WIFI_PASSWORD="YourPassword"
export MQTT_BROKER="mqtt://192.168.1.100:1883"
export UDP_SERVER="192.168.1.100:9000"
cargo build --release
```

### Using Python Tools (Station Mode Only)

The Python tools in `tools/python/` only work in **Station mode** when ESP32 is on the same network as your laptop.

**Setup:**
```bash
# Install dependencies
cd tools/python
pip install -r requirements.txt

# Start MQTT broker (if not already running)
# On Ubuntu: sudo apt install mosquitto && sudo systemctl start mosquitto
# On macOS: brew install mosquitto && brew services start mosquitto
```

**Receive UDP Telemetry (recommended):**
```bash
python3 udp_telemetry_server.py
```
Output:
```
[20Hz] Spd: 45.3km/h Pos:(123.4,456.7)m Acc:(+1.23,-0.15)m/s² CORNER
```

**Receive MQTT Status:**
```bash
python3 mqtt_binary_decoder.py
```

**Subscribe to raw MQTT topics:**
```bash
mosquitto_sub -h localhost -t 'car/#' -v
```

### Choosing a Mode

| Feature | Access Point | Station |
|---------|--------------|---------|
| Mobile dashboard | ✅ Yes | ❌ No |
| Python tools | ❌ No | ✅ Yes |
| Needs router | ❌ No | ✅ Yes |
| Range | ~30m | Network dependent |
| Multiple receivers | ❌ No | ✅ Yes |
| Data logging to laptop | ❌ No | ✅ Yes |

---

## Future Enhancements

- [ ] SD card logging for offline operation
- [x] ~~Web dashboard for live visualization~~ ✓ Built-in mobile dashboard
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
