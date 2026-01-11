# Changelog

All notable changes to Blackbox will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed
- Renamed `neo6m` driver to `ublox-gps` to reflect support for multiple u-blox GPS modules (NEO-6M, NEO-M9N, etc.)
- Updated driver documentation with UBX protocol examples and dynamic platform models

### Removed
- Obsolete documentation files (RELEASE.md, IMPLEMENTATION_SUMMARY.md, release-notes.md)

## [0.0.2] - 2025-01-10

### Added
- NEO-M9N GPS support with UBX protocol configuration (up to 25 Hz)
- Loop rate, ZUPT rate, and EKF/GPS metrics in diagnostics dashboard
- `configure_wt901.py` and `probe_wt901.py` tools for IMU configuration
- `take_new_fix()` method in GPS parser for accurate fix counting

### Changed
- GPS rate now counts only valid RMC position fixes (excludes GGA/GSA sentences)
- ZUPT display changed from cumulative count to rate per minute (rolling average)
- Improved diagnostics accuracy for sensor rate monitoring

### Fixed
- GPS rate showing inflated values (~3x actual) due to counting all NMEA sentences
- ZUPT count growing unboundedly (now shows useful rate metric)

## [0.0.1] - 2025-01-XX

### Added
- Initial release with Access Point mode and mobile dashboard
- HTTP polling-based telemetry at ~30 Hz
- Driving presets (Track, Canyon, City, Highway, Custom)
- G-meter with trail visualization
- Live settings configuration
- CSV data export

## [0.1.0] - 2025-01-XX

### Added
- Initial release of Blackbox
- ESP32-C3 firmware for vehicle telemetry
- WT901 IMU driver with UART parsing
- u-blox GPS driver with NMEA parsing (supports NEO-6M, NEO-M9N, etc.)
- 7-state Extended Kalman Filter for sensor fusion
  - Position, velocity, yaw, and accelerometer bias estimation
  - CTRA motion model for turning dynamics
  - ZUPT for drift prevention when stationary
- Coordinate frame transformations (body → earth → vehicle)
- Driving mode classifier (IDLE, ACCEL, BRAKE, CORNER, ACCEL+CORNER, BRAKE+CORNER)
- Binary telemetry protocol (67 bytes per packet)
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
