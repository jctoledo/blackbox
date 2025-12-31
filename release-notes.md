# Blackbox v0.0.1 - First Release 🚀

ESP32-C3 vehicle telemetry system with built-in mobile dashboard and dual WiFi modes.

## 🎯 Major Features

### Built-in Mobile Dashboard
- Mobile-optimized web interface running directly on ESP32
- Real-time telemetry display at ~20 Hz via HTTP polling
- G-meter with enhanced trail visualization and glow effects
- Live speed display with car speedometer-like smoothing
- Mode detection visualization (6 states including combined modes)
- Session timer and GPS coordinates
- CSV data recording and export
- **Live settings configuration** with validation

**Usage:** Connect phone to `Blackbox` WiFi → Open browser to `http://192.168.4.1`

### Dual WiFi Operating Modes

**Access Point Mode (Default)**
- ESP32 creates its own WiFi network - no router required
- Perfect for track days, autocross, mobile use
- Fixed IP: `192.168.4.1:80`
- SSID: `Blackbox` / Password: `blackbox123`

**Station Mode**
- ESP32 connects to your existing WiFi network
- UDP telemetry streaming (20 Hz) to laptop
- MQTT status messages
- For data logging and multi-client scenarios

### Enhanced Mode Detection
- **6 driving states:** IDLE, ACCEL, BRAKE, CORNER, ACCEL+CORNER, BRAKE+CORNER
- Combined states using bitflags (e.g., trail braking = BRAKE+CORNER)
- Independent component detection with hysteresis
- Tuned for street driving (lowered acceleration threshold to 0.10g)
- **177 comprehensive unit tests**

## 🔧 Hardware Requirements

- **ESP32-C3** microcontroller
- **WT901** 9-axis IMU (UART, 200Hz)
- **NEO-6M** GPS module (UART, 5Hz)
- **WS2812** RGB LED (status indicator)

## 📥 Installation

### Flash the Firmware

**Method 1: Using espflash (recommended)**
```bash
espflash write-bin 0x0 blackbox-v0.0.1.bin
```

**Method 2: Using ESP Web Flasher**
1. Go to https://espressif.github.io/esptool-js/
2. Connect ESP32-C3 via USB
3. Select `blackbox-v0.0.1.bin`
4. Flash offset: `0x0`
5. Click "Program"

### First Boot
1. Power on ESP32 - LED shows boot sequence (see README for codes)
2. **AP Mode (default):** Connect phone to WiFi network "Blackbox" (password: `blackbox123`)
3. Open browser to `http://192.168.4.1`
4. View live telemetry!

### Switch to Station Mode
To use UDP streaming and MQTT (requires rebuild):
```bash
export WIFI_MODE="station"
export WIFI_SSID="YourNetwork"
export WIFI_PASSWORD="YourPassword"
export MQTT_BROKER="mqtt://192.168.1.100:1883"
export UDP_SERVER="192.168.1.100:9000"
cargo build --release
# Then flash the new binary
```

## 📊 Dashboard Features

- **G-meter:** Real-time visualization with gradient trail, tick marks, enhanced glow
- **Max G tracking:** L/R/Accel/Brake peak values
- **Speed display:** EMA-filtered for smoothness (like car speedometer)
- **Mode indicator:** Visual icons for each driving state
- **Settings:** Adjust all 8 mode detection thresholds live
- **Recording:** Capture session data and export to CSV

## 🎨 Technical Highlights

- 7-state Extended Kalman Filter for sensor fusion
- Zero-velocity updates (ZUPT) prevent drift when stopped
- Gravity-compensated accelerations
- GPS warmup with local coordinate mapping
- HTTP polling architecture (replaced WebSocket to fix thread blocking)
- Binary telemetry protocol (67 bytes, 20Hz)
- Configurable mode detection thresholds

## 📝 Binary Protocol

Mode byte changed to bitflags:
- `0` = IDLE
- `1` = ACCEL
- `2` = BRAKE
- `4` = CORNER
- `5` = ACCEL+CORNER (trail throttle)
- `6` = BRAKE+CORNER (trail braking)

## 🐛 Known Issues

None reported yet! This is the first release.

## 📚 Documentation

- [README](https://github.com/jctoledo/blackbox/blob/main/README.md) - Full setup guide
- [CLAUDE.md](https://github.com/jctoledo/blackbox/blob/main/CLAUDE.md) - Architecture details
- [Hardware Setup](https://github.com/jctoledo/blackbox/blob/main/sensors/blackbox/README.md) - Pin assignments

## 🙏 Credits

Built with:
- Rust ESP-IDF framework
- ESP32-C3 RISC-V chip
- motorsport-telemetry library

---

**Full Changelog:** https://github.com/jctoledo/blackbox/compare/...v0.0.1
