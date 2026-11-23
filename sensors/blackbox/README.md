# Blackbox — ESP32-C3 vehicle telemetry

Build a local GPS + inertial instrument for circuit/stage timing, reference pace, and portable drive recordings. Rust firmware performs sensing and crossing detection on the ESP32-C3; a phone browser provides the dashboard, reference comparison, and recording storage.

## Capabilities and configuration

- **GPS:** NEO-M9N configuration target 25 Hz, or NEO-6M target 5 Hz. Choose the matching release image and verify received fixes in Diagnostics.
- **IMU:** WT901 UART output configurable up to 200 Hz. Output rate is separate from estimator processing and browser reception rates.
- **Timing:** directional start/finish lines with crossing interpolation between fresh GPS fixes; circuits and point-to-point stages.
- **Reference pace:** spatial position/time traces, with direction, continuity, and ambiguity rejection. Historical best is displayed separately.
- **Recording:** browser-local sessions, CSV exports, and validated JSON backup/restore as copies. Keep an external backup; recordings are not autonomously logged to device flash.
- **Ownership:** MIT-licensed source, a local Wi-Fi dashboard, and no project subscription or cloud account requirement.

The current estimator retains per-axis position/velocity/bias covariance with independent yaw. Yaw-to-translation and ENU cross-axis correlations remain omitted. The project publishes regression evidence and configuration targets; physical accuracy, latency, and durability need measured acceptance results. See the [sourced capability comparison](../../docs/COMPARISON.md).

## Hardware and installation

Use an ESP32-C3 with the published 4 MiB image, WT901, a supported u-blox receiver and antenna, suitable power, wiring, and a mount. The firmware pin mapping must match your actual board. In particular, GPIO18/19 are used for IMU UART and conflict with native USB on ESP32-C3.

Follow the maintained [root setup/build instructions](../../README.md), [compatibility FAQ](../../docs/FAQ.md), and [hardware acceptance checklist](../../docs/HARDWARE_VALIDATION.md). Firmware releases distinguish GPS variants. The schematic under review is not proof of board compatibility.

## System architecture

```text
GPS + IMU → SensorManager → StateEstimator / GPS crossing timer
                                  ↓
                        HTTP dashboard / UDP telemetry
                                  ↓
                   Browser recording + spatial reference analysis
```

Device sensing and crossing detection run locally. Browser session recording and live reference analysis depend on the phone/browser connection. The configured device telemetry interval is 33 ms (about 30 Hz); actual reception and processing rates must be measured.

## Binary telemetry contract

Current binary protocol is **version 4, 82 bytes**, little-endian, with header magic `0xAA55` and a trailing u16 sum of the preceding 80 bytes. It contains device uptime, acceleration, angular rate, orientation/estimator state, speed and estimator uncertainty, mode, compact GPS coordinates/course/validity, and current lap time/count/flags.

Use [the source layout](src/binary_telemetry.rs) and [the cross-language fixture](../../tests/fixtures/telemetry-v4.hex) as the authoritative contract. HTTP telemetry adds persistent completed/best lap times, timing state/revision, GPS age, and higher-precision geographic values. Binary f32 coordinates have location-dependent quantization; numeric precision is separate from physical GPS accuracy. Older 67-byte/v1 descriptions do not apply to current firmware.

## Further reading

- [Lap timer and spatial reference guide](../../docs/LAP_TIMER.md)
- [Sensor fusion and coordinate frames](../../docs/SENSOR_FUSION.md)
- [Engineering review and evidence](../../docs/REVIEW.md)
- [Python receiver tools](../../tools/python/)
- [Contributing](../../CONTRIBUTING.md)

Acceleration/drag measurement, slope correction, adaptive corner coaching, and linked post-drive replay are proposed extensions, not available modes. See [the capability comparison](../../docs/COMPARISON.md) for the roadmap distinction.
