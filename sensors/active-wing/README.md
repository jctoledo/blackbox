# Active Wing - ESP32 Telemetry System

High-performance sensor fusion telemetry system for ESP32-C3 using the Active Wing Framework.

## Hardware

- **1x ESP32-C3** microcontroller
- **1x WT901 9-axis IMU** (200Hz via UART)
- **1x NEO-6M GPS** (5Hz via UART)
- **1x WS2812 RGB LED** (status indicator)

## Features

- ✅ **Real-time sensor fusion** using Extended Kalman Filter (EKF)
- ✅ **200Hz IMU processing** with gravity compensation and bias correction
- ✅ **GPS integration** with position and velocity updates
- ✅ **Binary telemetry** over TCP at 20Hz (66-byte packets)
- ✅ **MQTT status messages** for monitoring
- ✅ **Visual status feedback** via RGB LED
- ✅ **Zero-velocity updates (ZUPT)** for drift prevention
- ✅ **Mode classification** (stopped, cruising, braking, cornering)

## System Architecture

```
ESP32-C3 Firmware
├─ SensorManager (polls IMU + GPS via UART)
├─ StateEstimator (EKF fuses IMU + GPS locally)
├─ TelemetryPublisher (TCP 20Hz + MQTT status)
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
python tcp_telemetry_server.py
```

This will display:
- Real-time position (X, Y)
- Velocity (Vx, Vy, Speed)
- Orientation (Yaw, Roll, Pitch)
- Accelerations (ax, ay, az)
- GPS status

## Binary Telemetry Protocol

66-byte binary packet structure sent at 20Hz over TCP:

```
[Header: 2 bytes] 0xAA 0x55
[Timestamp: 4 bytes] milliseconds
[Accel: 12 bytes] ax, ay, az (m/s²)
[Gyro: 4 bytes] wz (rad/s)
[Orientation: 12 bytes] roll, pitch, yaw (rad)
[Position: 8 bytes] x, y (m)
[Velocity: 8 bytes] vx, vy (m/s)
[Speed: 4 bytes] speed (km/h)
[GPS: 9 bytes] lat, lon (deg), valid flag
[Mode: 1 byte] driving mode
[Checksum: 2 bytes] CRC16
```

## LED Status Codes

- **Yellow Blinking (fast)** - Waiting for GPS lock
- **Cyan Pulse (slow)** - GPS locked, system operational
- **Red** - Critical error
- **Green** - Calibration complete

## Project Structure

```
active-wing/
├── src/
│   ├── main.rs              - Application entry point
│   ├── system.rs            - System components (SensorManager, etc.)
│   ├── sensor_plugins.rs    - WT901 + NEO-6M plugin implementations
│   ├── fusion_coordinator.rs- Multi-sensor fusion coordinator
│   ├── imu.rs               - WT901 9-axis IMU driver
│   ├── gps.rs               - NEO-6M GPS NMEA parser
│   ├── binary_telemetry.rs  - Binary protocol encoding
│   ├── mode.rs              - Driving mode classifier
│   ├── mqtt.rs              - MQTT client
│   ├── tcp_stream.rs        - TCP telemetry stream
│   ├── wifi.rs              - WiFi connection management
│   ├── rgb_led.rs           - WS2812 LED driver
│   └── web_sockets.rs       - WebSocket support
│
├── tools/python/
│   ├── tcp_telemetry_server.py     - Telemetry receiver + visualizer
│   ├── mqtt_binary_decoder.py      - MQTT telemetry decoder
│   └── requirements.txt
│
├── docs/
│   ├── FINAL_SUMMARY.md            - Complete project summary
│   ├── REFACTORING_SUMMARY.md      - SOLID refactoring details
│   └── CODE_REVIEW_REFACTORED.md   - Code quality analysis
│
├── Cargo.toml               - Project dependencies
├── build.rs                 - ESP-IDF build script
└── config.toml.example      - Configuration template
```

## Adding More Sensors

This project uses the Active Wing Framework, making it easy to add new sensors to the same ESP32:

```rust
// Example: Add wheel speed sensors
use active_wing_framework::sensor_framework::Sensor;

pub struct WheelSpeedSensor {
    gpio_pin: GpioPin,
}

impl Sensor for WheelSpeedSensor {
    fn poll(&mut self) -> Option<SensorReading> {
        // Read GPIO pulses
        Some(SensorReading { /* ... */ })
    }
}

// In main.rs - just add one line:
let wheels = Box::new(WheelSpeedSensor::new(gpio_pin));
registry.register(wheels).unwrap();
```

No changes to existing IMU/GPS code required!

See the [Sensor Toolkit Guide](../../docs/SENSOR_TOOLKIT_GUIDE.md) for detailed instructions.

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

### TCP Connection Drops
- Check firewall settings on receiver
- Verify IP address in config.toml
- Monitor WiFi signal strength

### Compilation Errors
```bash
# Clean build artifacts
cargo clean

# Update dependencies
cargo update

# Re-flash bootloader if needed
espflash flash --erase-parts nvs
```

## Documentation

- [Final Summary](docs/FINAL_SUMMARY.md) - Complete project overview
- [Refactoring Summary](docs/REFACTORING_SUMMARY.md) - SOLID improvements
- [Code Review](docs/CODE_REVIEW_REFACTORED.md) - Code quality analysis
- [Framework Guide](../../docs/SENSOR_TOOLKIT_GUIDE.md) - Adding sensors
- [Architecture](../../docs/ARCHITECTURE.md) - System design

## License

See root LICENSE file.

## Contributing

See [CONTRIBUTING.md](../../CONTRIBUTING.md) in the root repository.
