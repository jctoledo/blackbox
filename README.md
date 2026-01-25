# Blackbox - ESP32 Vehicle Telemetry System

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Rust](https://img.shields.io/badge/rust-1.75%2B-orange.svg)](https://www.rust-lang.org/)
[![ESP32-C3](https://img.shields.io/badge/platform-ESP32--C3-blue.svg)](https://www.espressif.com/)

Real-time GPS + IMU sensor fusion for vehicle dynamics. Track position, velocity, acceleration, and driving modes on $50 hardware. Built-in mobile dashboard with lap timer.

**What you get:**
- Live G-meter with mode detection (idle, accel, brake, corner)
- Lap timer with delta-to-best display
- 20-30 Hz telemetry streaming over WiFi
- Works standalone — no phone app or internet needed

**Documentation:** [Lap Timer Guide](docs/LAP_TIMER.md) · [Sensor Fusion Guide](docs/SENSOR_FUSION.md)

---

## Quick Start

### Option A: Flash from Browser (Easiest)

1. Go to **https://jctoledo.github.io/blackbox/**
2. Connect ESP32-C3 via USB
3. Click **Connect Device** → select port → **Flash Firmware**

*Requires Chrome, Edge, or Opera. Hold BOOT button if device isn't detected.*

### Option B: Build from Source

```bash
# Install toolchain (one-time)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
cargo install espup cargo-espflash
espup install

# Build and flash
source $HOME/export-esp.sh  # Required in every new terminal
cd sensors/blackbox
cargo run --release
```

First build takes 10-20 minutes. See [Build Guide](#building-from-source) for detailed instructions.

### Use the Dashboard

1. Power on device → creates WiFi network **"Blackbox"**
2. Connect your phone (password: `blackbox123`)
3. Open browser → go to `192.168.71.1`
4. Drive!

---

## Hardware

### Required Components (~$50-100)

| Component | Price | Notes |
|-----------|-------|-------|
| ESP32-C3 DevKit | $5 | Any C3 dev board works |
| WT901 IMU | $25 | 9-axis, configurable to 200Hz |
| GPS Module | $15-75 | NEO-6M (budget) or NEO-M9N (recommended) |
| Wires + USB | $5 | — |

**GPS Choice:** NEO-M9N ($75) provides 25Hz updates and better accuracy. Worth it for serious use. NEO-6M ($15) works fine for learning.

### Wiring

```
ESP32-C3              WT901 IMU         GPS Module
─────────             ─────────         ──────────
3V3  ─────────────────► VCC
GND  ─────────────────► GND
GPIO19 (RX) ──────────► TX
GPIO18 (TX) ──────────► RX
3V3  ─────────────────────────────────► VCC
GND  ─────────────────────────────────► GND
GPIO4  (RX) ──────────────────────────► TX
GPIO5  (TX) ──────────────────────────► RX
GPIO8 ───► (Optional: WS2812 LED)
```

<details>
<summary><b>IMU Configuration (optional)</b></summary>

The WT901 ships at 10Hz. Firmware auto-detects baud rate, so **try it first without configuration**.

Check the diagnostics page — if IMU Rate shows ~10Hz instead of ~200Hz, configure it:

```bash
# Requires USB-serial adapter (~$5)
# Connect: IMU TX→Adapter RX, IMU RX→Adapter TX, GND, 5V
cd tools/python
pip install pyserial
python3 configure_wt901.py /dev/ttyUSB0
```

Settings save to EEPROM — only needed once per IMU.

</details>

---

## Mobile Dashboard

Connect to the device's WiFi and open `192.168.71.1` in any browser.

### Features

| Feature | Description |
|---------|-------------|
| G-Meter | Real-time lateral/longitudinal display with trail |
| Speed | Current and max speed |
| Mode Detection | IDLE, ACCEL, BRAKE, CORNER, or combined states |
| Driving Presets | Track, Canyon, City, Highway, or Custom |
| Lap Timer | Record tracks, time laps, see delta-to-best |
| Recording | Capture sessions, export to CSV |
| Diagnostics | Sensor rates, GPS quality, EKF health |

### Controls

| Action | Effect |
|--------|--------|
| Tap CLR | Reset max G values |
| Tap max speed | Reset max speed |
| Preset buttons | Switch driving profile |
| REC button | Start/stop recording |

---

## Lap Timer

Record tracks anywhere and get real-time lap timing with delta display.

**[Full Lap Timer Guide →](docs/LAP_TIMER.md)**

### Quick Overview

1. **Record a Track** — Tap lap timer card → **+** → drive one lap → **Stop**
2. **Activate Track** — Tap saved track to enable timing
3. **Drive** — Cross start line, timing begins automatically
4. **Watch Delta** — Green = ahead, Red = behind your best

### Delta Bar

```
    -2.3s                     2.3 seconds AHEAD of best
◄████████████░░░░░░░░►        Green bar extends left

    +1.5s                     1.5 seconds BEHIND best
◄░░░░░░░░████████████►        Red bar extends right
```

- **Scale:** ±2 seconds = full bar. Number shows actual delta beyond ±2s.
- **Trend arrows:** ▲ gaining time, ▼ losing time
- **Glow effect:** Appears when delta exceeds 1 second

### Features

| Feature | Description |
|---------|-------------|
| Circuit & Stage | Loop tracks or point-to-point routes |
| Session History | All lap times saved and grouped by date |
| Track Auto-Detection | Notifies you when near a saved track |
| Data Management | View storage, export CSV, delete sessions |

See the [full documentation](docs/LAP_TIMER.md) for details on recording, delta calculation, and troubleshooting.

---

## Calibration

Proper calibration is critical for accurate readings.

1. Mount device rigidly in final position
2. Park on level ground, engine **OFF**
3. Power on and **don't touch anything** during yellow LED (~10 seconds)
4. Wait for cyan pulse = ready

**Signs of bad calibration:** Mode triggers incorrectly, G-meter not centered when parked, non-zero acceleration when stationary.

**To recalibrate:** Tap CLR in dashboard while stationary with engine off.

---

## LED Status

| Pattern | Meaning |
|---------|---------|
| Cyan pulse (2s) | GPS locked, operational |
| Yellow fast blink | Waiting for GPS fix |
| Yellow pulses | Calibration in progress |
| Green blinks (boot) | WiFi ready |
| Red blink | Error (check wiring) |

---

## Building from Source

<details>
<summary><b>Prerequisites by Platform</b></summary>

**Ubuntu/Debian:**
```bash
sudo apt-get install -y git wget flex bison gperf python3 python3-pip \
  python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
cargo install espup cargo-espflash
espup install
rustup component add rust-src --toolchain esp
```

**macOS:**
```bash
brew install cmake ninja dfu-util
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
cargo install espup cargo-espflash
espup install
rustup component add rust-src --toolchain esp
```

**Windows:**
1. Install Visual Studio Build Tools, Python 3, Git
2. Install Rust via rustup-init.exe
3. Run: `cargo install espup cargo-espflash && espup install`

</details>

### Build Commands

```bash
source $HOME/export-esp.sh  # Required in every new terminal!
cd sensors/blackbox

cargo check              # Quick syntax check
cargo build --release    # Build firmware
cargo run --release      # Build + flash + monitor
cargo fmt                # Format code
cargo clippy             # Lint
```

### GPS Configuration

```bash
# Default: NEO-6M at 5Hz
cargo build --release

# NEO-M9N at 25Hz
export GPS_MODEL="m9n" GPS_RATE="25"
cargo build --release
```

<details>
<summary><b>Troubleshooting Build Issues</b></summary>

**"can't find crate for core"**
```bash
source $HOME/export-esp.sh
rustup component add rust-src --toolchain esp
```

**"linking with riscv32-esp-elf-gcc failed"**
```bash
source $HOME/export-esp.sh  # You forgot this
```

**Permission denied on serial port (Linux)**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

**Build hangs**
```bash
cargo clean
rm -rf ~/.espressif
espup install
```

</details>

---

## Configuration Reference

### WiFi Modes

**Access Point (default)** — Device creates its own network. Best for mobile use.
```bash
cargo build --release  # No config needed
```

**Station Mode** — Device joins your network. Enables UDP streaming to laptop.
```bash
export WIFI_MODE="station"
export WIFI_SSID="YourNetwork"
export WIFI_PASSWORD="YourPassword"
export UDP_SERVER="192.168.1.100:9000"
cargo build --release
```

### Mode Detection Thresholds

Configure via dashboard presets or custom sliders:

| Preset | Accel | Brake | Lateral | Best For |
|--------|-------|-------|---------|----------|
| Track | 0.35g | 0.55g | 0.50g | Racing |
| Canyon | 0.22g | 0.35g | 0.28g | Mountain roads |
| City | 0.10g | 0.18g | 0.12g | Daily driving |
| Highway | 0.12g | 0.22g | 0.14g | Cruising |

<details>
<summary><b>EKF Tuning (Advanced)</b></summary>

Edit `sensors/blackbox/src/ekf.rs`:

```rust
const Q_ACC: f32 = 0.40;    // Process noise: acceleration
const Q_GYRO: f32 = 0.005;  // Process noise: gyro
const R_POS: f32 = 20.0;    // Measurement noise: GPS position
const R_VEL: f32 = 0.2;     // Measurement noise: GPS velocity
```

Higher Q = trust sensors more. Higher R = trust sensors less.

</details>

<details>
<summary><b>Diagnostics Metrics Explained</b></summary>

| Metric | Healthy | Meaning |
|--------|---------|---------|
| IMU Rate | 195-200 Hz | Accelerometer samples/sec |
| GPS Rate | 5-25 Hz | Position fixes/sec |
| Position σ | <5m | EKF position uncertainty |
| Velocity σ | <0.5 m/s | EKF velocity uncertainty |
| Bias X/Y | <0.5 m/s² | Learned accelerometer bias |
| HDOP | <2.0 | GPS quality (lower = better) |

**Troubleshooting:**
- IMU 0 Hz → Check wiring, run configure_wt901.py
- GPS 0 Hz → Check wiring, ensure sky view
- High bias → Recalibrate (CLR button)

</details>

---

## Technical Overview

**[Full Sensor Fusion Guide →](docs/SENSOR_FUSION.md)**

### How Sensor Fusion Works

The system combines fast IMU (200Hz) with accurate GPS (5-25Hz):

1. **Predict** — IMU accelerations update position/velocity 200×/sec
2. **Correct** — GPS fixes correct drift 5-25×/sec
3. **ZUPT** — When stopped, velocity forced to zero, biases learned

**7-State Kalman Filter:** Position (x,y), heading, velocity (vx,vy), accelerometer biases (bax,bay)

### Data Flow

```
GPS (5-25Hz) ──┐
               ├──► EKF ──► Mode Classifier ──► Telemetry (20Hz)
IMU (200Hz) ───┘
```

See the [full sensor fusion documentation](docs/SENSOR_FUSION.md) for details on coordinate frames, ZUPT, EKF tuning, and diagnostics interpretation.

### File Structure

```
blackbox/
├── docs/                # Documentation
│   ├── LAP_TIMER.md     # Lap timer user guide
│   └── SENSOR_FUSION.md # EKF and sensor fusion details
├── framework/           # Sensor fusion library (host-testable)
│   └── src/
│       ├── ekf.rs       # Extended Kalman Filter
│       ├── transforms.rs # Coordinate math
│       ├── mode.rs      # Mode detection
│       └── fusion.rs    # GPS/IMU blending
├── sensors/blackbox/    # ESP32 firmware
│   └── src/
│       ├── main.rs      # Main loop
│       ├── websocket_server.rs  # Dashboard
│       └── ...
└── tools/
    ├── dashboard-dev/   # Dashboard simulator
    └── python/          # Receivers and tools
```

---

## Python Tools

Station mode only. Connect ESP32 and laptop to same network.

```bash
cd tools/python
pip install -r requirements.txt

# Receive telemetry
python3 udp_telemetry_server.py

# Analyze CSV export
python3 analyze_telemetry.py recorded_session.csv

# Configure IMU
python3 configure_wt901.py /dev/ttyUSB0
```

---

## Future Enhancements

- [ ] SD card logging
- [ ] CAN bus integration
- [ ] Bluetooth connectivity
- [ ] Over-the-air updates
- [x] ~~Mobile dashboard~~ ✓
- [x] ~~High-rate GPS~~ ✓
- [x] ~~Lap timer~~ ✓

---

## Contributing

Contributions welcome — bug fixes, new features, documentation improvements. Open an issue or PR on GitHub.

## License

MIT License — see [LICENSE](LICENSE).

---

**Questions?** Open an [issue](https://github.com/jctoledo/blackbox/issues).

**Ready to build?** [Start with Quick Start ↑](#quick-start)
