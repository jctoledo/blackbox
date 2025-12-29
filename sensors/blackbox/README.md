# Blackbox - ESP32 Telemetry System

High-performance sensor fusion telemetry system for ESP32-C3 using the Blackbox Framework.

## Hardware

- **1x ESP32-C3** microcontroller
- **1x WT901 9-axis IMU** (200Hz via UART)
- **1x NEO-6M GPS** (5Hz via UART)
- **1x WS2812 RGB LED** (status indicator)

## Features

- ✅ **Real-time sensor fusion** using Extended Kalman Filter (EKF)
- ✅ **200Hz IMU processing** with gravity compensation and bias correction
- ✅ **GPS integration** with position and velocity updates
- ✅ **Binary telemetry** over UDP at 20Hz (67-byte packets)
- ✅ **MQTT status messages** for monitoring
- ✅ **Visual status feedback** via RGB LED
- ✅ **Zero-velocity updates (ZUPT)** for drift prevention
- ✅ **Mode classification** (stopped, cruising, braking, cornering)

## System Architecture

```
ESP32-C3 Firmware
├─ SensorManager (polls IMU + GPS via UART)
├─ StateEstimator (EKF fuses IMU + GPS locally)
├─ TelemetryPublisher (UDP 20Hz + MQTT status)
└─ StatusManager (LED control)
```

All sensor fusion happens **locally on the ESP32** - no external processing required.

## Quick Start

### Prerequisites

```bash
# Install Rust ESP32 toolchain
rustup target add riscv32imc-esp-espidf

# Install espflash
cargo install espflash
```

### Configuration

1. Copy the config template:
```bash
cp config.toml.example config.toml
```

2. Edit `config.toml` with your WiFi and network settings:
```toml
[wifi]
ssid = "YourWiFiNetwork"
password = "YourWiFiPassword"

[network]
mqtt_broker = "mqtt://192.168.1.100:1883"
tcp_server = "192.168.1.100:9000"
```

⚠️ **NEVER commit `config.toml`** - it contains credentials!

### Build and Flash

```bash
# Check for compilation errors
cargo check

# Build release binary
cargo build --release

# Flash to ESP32 and monitor output
cargo espflash flash --release --monitor
```

## Telemetry Receiver

Run the Python telemetry receiver on your laptop:

```bash
cd tools/python
pip install -r requirements.txt
python udp_telemetry_server.py
```

This will display:
- Real-time position (X, Y)
- Velocity (Vx, Vy, Speed)
- Orientation (Yaw, Roll, Pitch)
- Accelerations (ax, ay, az)
- GPS status

## Binary Telemetry Protocol

67-byte binary packet structure sent at 20Hz over UDP:

```
[Version: 1 byte] Protocol version (0x01)
[Header: 2 bytes] 0xAA55
[Timestamp: 4 bytes] milliseconds
[Accel: 12 bytes] ax, ay, az (f32, m/s²)
[Gyro: 4 bytes] wz (f32, rad/s)
[Orientation: 12 bytes] roll, pitch, yaw (f32, rad)
[Position: 8 bytes] x, y (f32, m)
[Velocity: 8 bytes] vx, vy (f32, m/s)
[Speed: 4 bytes] speed (f32, km/h)
[Mode: 1 byte] driving mode (0=IDLE, 1=ACCEL, 2=BRAKE, 3=CORNER)
[GPS: 9 bytes] lat, lon (f32, deg), valid flag (u8)
[Checksum: 2 bytes] sum of first 65 bytes
```

## LED Status Codes

### Boot Sequence
| Sequence | Pattern | Meaning |
|----------|---------|---------|
| 1 | 3 blue blinks | Boot sequence started |
| 2 | 5 green blinks | WiFi connected |
| 3a | 3 magenta blinks | MQTT connected successfully |
| 3b | 5 fast red blinks | MQTT connection failed (continuing without MQTT) |
| 4 | 3 cyan blinks | UDP socket ready |
| 5 | Yellow pulses | IMU calibration in progress |
| Error | Continuous red blink | Critical error (WiFi failed at boot) |

### Main Loop (operational)
| Pattern | Meaning |
|---------|---------|
| Yellow fast blink | Waiting for GPS fix |
| Cyan pulse (2s on/off) | GPS locked, system operational |
| 3 orange blinks (repeating) | WiFi disconnected |
| 2 red blinks (repeating) | MQTT disconnected |

Connectivity is checked every 5 seconds. Orange/red blinks repeat every 5 seconds while the connection remains down.

## Project Structure

```
blackbox/
├── src/
│   ├── main.rs              - Application entry point
│   ├── system.rs            - System components (SensorManager, etc.)
│   ├── config.rs            - Configuration management
│   ├── imu.rs               - WT901 9-axis IMU driver
│   ├── gps.rs               - NEO-6M GPS NMEA parser
│   ├── binary_telemetry.rs  - Binary protocol encoding
│   ├── mode.rs              - Driving mode classifier
│   ├── mqtt.rs              - MQTT client
│   ├── udp_stream.rs        - UDP telemetry stream
│   ├── wifi.rs              - WiFi connection management
│   └── rgb_led.rs           - WS2812 LED driver
│
├── tools/python/
│   ├── udp_telemetry_server.py     - Telemetry receiver + visualizer
│   ├── mqtt_binary_decoder.py      - MQTT telemetry decoder
│   └── requirements.txt
│
├── Cargo.toml               - Project dependencies
├── build.rs                 - ESP-IDF build script
└── config.toml.example      - Configuration template
```

## Performance Metrics

- **Main loop frequency**: 200Hz (IMU-limited)
- **Telemetry rate**: 20Hz
- **GPS update rate**: 5Hz
- **EKF prediction**: 200Hz
- **EKF updates**: 5Hz (GPS) + 200Hz (magnetometer yaw)
- **Memory usage**: ~60KB RAM
- **Flash usage**: ~450KB

## Troubleshooting

### GPS Not Locking
- Ensure clear sky view (not indoors)
- Wait 30-60 seconds for warm-up
- Check UART connection (RX/TX not swapped)

### IMU Data Noisy
- Run calibration with sensor stationary
- Check accelerometer bias in telemetry
- Verify mounting is secure (no vibrations)

### UDP Connection Issues
- Check firewall settings on receiver
- Verify IP address in config.toml
- Monitor WiFi signal strength
- UDP is connectionless - packets may be dropped on poor networks

### Compilation Errors
```bash
# Clean build artifacts
cargo clean

# Update dependencies
cargo update

# Re-flash bootloader if needed
espflash flash --erase-parts nvs
```

## License

See root LICENSE file.

## Contributing

See [CONTRIBUTING.md](../../CONTRIBUTING.md) in the root repository.
