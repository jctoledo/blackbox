# Changelog

All notable changes to Active Wing will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Comprehensive documentation (README, CONTRIBUTING, CODE_OF_CONDUCT, SECURITY)
- GitHub issue and PR templates
- CLAUDE.md architecture documentation

## [0.1.0] - 2025-01-XX

### Added
- Initial release of Active Wing
- ESP32-C3 firmware for vehicle telemetry
- WT901 IMU driver with UART parsing
- NEO-6M GPS driver with NMEA parsing
- 7-state Extended Kalman Filter for sensor fusion
  - Position, velocity, yaw, and accelerometer bias estimation
  - CTRA motion model for turning dynamics
  - ZUPT for drift prevention when stationary
- Coordinate frame transformations (body → earth → vehicle)
- Driving mode classifier (IDLE, ACCEL, BRAKE, CORNER)
- Binary telemetry protocol (66 bytes per packet)
- TCP streaming at 20 Hz
- MQTT status messaging
- RGB LED status indication
- Python telemetry receivers (in `tools/python/`):
  - `tcp_telemetry_server.py` for high-speed TCP reception
  - `mqtt_binary_decoder.py` for MQTT binary decoding
  - `mqtt_decoder.py` for legacy JSON decoding
- WiFi connectivity with automatic connection
- IMU calibration on startup
- GPS warmup and local coordinate reference
- Online accelerometer bias estimation

### Hardware Support
- ESP32-C3 microcontroller
- WT901 9-axis IMU (accelerometer, gyroscope, magnetometer)
- NEO-6M GPS module (5 Hz update rate)
- WS2812 RGB LED (optional status indicator)

### Performance
- 200 Hz IMU sampling rate
- 20 Hz telemetry output rate
- 5 Hz GPS updates
- Sub-50ms sensor-to-transmission latency
- 1.3 KB/s bandwidth (binary protocol)
- ±1m position accuracy between GPS fixes

### Documentation
- Comprehensive README with:
  - Hardware requirements and wiring guide
  - Quick start instructions
  - Architecture overview
  - Use cases and examples
  - Troubleshooting guide
- CLAUDE.md with detailed architecture documentation
- Inline code comments and documentation

## Release Types

This project uses [Semantic Versioning](https://semver.org/):

- **MAJOR** version (X.0.0) - Incompatible API/protocol changes
- **MINOR** version (0.X.0) - New features, backward compatible
- **PATCH** version (0.0.X) - Bug fixes, backward compatible

## How to Read This Changelog

### Categories

- **Added** - New features
- **Changed** - Changes to existing functionality
- **Deprecated** - Features that will be removed in future versions
- **Removed** - Features that have been removed
- **Fixed** - Bug fixes
- **Security** - Security vulnerability fixes

### Links

- [Unreleased]: Compare unreleased changes
- [0.1.0]: First release tag

---

## Upcoming Features (Roadmap)

These are planned but not yet implemented:

### Short Term (v0.2.0)
- [ ] Additional IMU support (MPU6050, BMI088)
- [ ] GPGGA sentence parsing for better GPS accuracy
- [ ] Configurable telemetry rate
- [ ] SD card logging option
- [ ] Web-based configuration interface

### Medium Term (v0.3.0)
- [ ] BLE streaming for mobile apps
- [ ] CAN bus integration for OBD2 data
- [ ] Track mapping and lap detection
- [ ] Improved mode classifier with drift detection
- [ ] Adaptive Kalman filter tuning

### Long Term (v1.0.0)
- [ ] RTK GPS support for cm-level accuracy
- [ ] Multi-sensor fusion (barometer, wheel speed)
- [ ] Real-time dashboard web app
- [ ] Data analysis toolkit
- [ ] Pre-built PCB design

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for how to propose changes.

## Support

Having issues? See [README.md](README.md#troubleshooting) for common problems.

For bug reports and feature requests, open an issue on GitHub.
