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

## [0.0.4-rc.8] - 2026-11-21

Release candidate for hardware evaluation. Includes rc.6 and the rc.7 source changes below; rc.7 binary packaging was stopped after a display-resize bug was reported.

### Fixed
- G-meter canvas follows its actual frame through Controls toggles, orientation and browser chrome changes; inherited size transitions no longer leave the canvas overflowing the card.
- Landscape Controls exposes Menu, with bounded scrolling on short screens.

### Added
- Persistent mph/km/h preference for Speed and MAX. Recorded units and explicitly named rolling intervals are unchanged.
- Synthetic circuit with left/right bends, common speed at lap boundaries, populated force summaries and lap history. Two named example recordings seed Saved data once without replacing existing sessions; rolling evidence includes two accepted intervals and a deliberate gap rejection.
- Synthetic provenance in ordinary Session CSV as well as GPS/Run exports. Updated website screenshots and demo guidance.

### Still to validate
- Real driving, calibration, receiver cadence and reference-timed accuracy remain hardware work. Expanded run detection is tracked in #79; 0–60 mph and slope-corrected performance are not implemented.

## [0.0.4-rc.7] - 2026-11-21

Release candidate for weekend data collection; hardware timing accuracy remains unmeasured. Includes rc.6.

### Added
- Bounded, per-RMC GPS timestamp evidence: receiver fractional UTC/date, device parse time, per-boot sequence, invalid fixes and explicit capture gaps.
- Separate GPS CSV in Saved data, with deduplication, restart boundaries, backup/restore preservation and a final fetch before stopping. Ordinary Session CSV also carries the latest GPS timestamp snapshot.
- Offline timestamp analyzer and collection guide; synthetic demo GPS logs are explicitly labelled.

### Changed
- Prominent How it works explainer beside the website CTA, with the full measurement pipeline, worked maths and a dedicated build/assembly journal.
- Landscape instrument picker collapses after selection; larger force plot, space-efficient Peaks and maximum speed with fresh-GPS gating. Circuit demo now anticipates braking zones and limits corner-exit acceleration using route curvature, with continuous lap-boundary speeds.
- Startup bias calibration separates AHRS-predicted gravity from accelerometer offset, rejects stale/moving/unstable collection and uses precise averaging. Inclined-pose software tests pass; physical driveway validation remains open.
- Receiver-timed 45–70 mph Runs instrument and derived Runs CSV with rejection reasons; explicit synthetic interval scenario and banner-free website demo. Standing-start and slope-corrected timing remain unimplemented.
- Landscape Speed / G-meter / Peaks tabs and swipe selection; compact status and a collapsible toolbar. Fixed force scale, dropout clearing and consistent lateral/longitudinal signs; time-weighted half-second page-session peaks.
- Circuit-driving synthetic demo, removed demo banner, simpler timing labels and refreshed website screenshots.
- Sensor-to-car alignment schematic in the setup screen and guide; live forces require fresh acceleration, gyro and attitude packets, with IMU age exported in Session CSV.

### Fixed
- Dashboard polling subtracts request/processing time from its 33 ms interval instead of adding another 33 ms; the measured UI receive rate is labelled separately from the firmware telemetry target.
- RMC fractional time is retained without floating-point rounding; malformed UTC fields are rejected and repeated newlines cannot parse a sentence twice.
- Website dashboard showcase uses versioned, correctly proportioned captures with the landscape cluster first and full-size links.

### Limits
- Host timestamps mark completed sentence parsing, not PPS or an antenna measurement instant. The RAM window holds 64 events; longer phone disconnects lose events and are reported. No change to lap-crossing timestamps or physical accuracy claims.

## [0.0.4-rc.6] - 2026-11-20

Release candidate; physical mount, phone and USB acceptance remains required. Includes rc.5.

### Added
- Experimental horizontal mount alignment: signed fresh GPS braking evidence, continuous sample gates, candidate review, explicit parked save/clear and one checked persistent record.
- Mount-aware force projection and orientation-feedback axes, with frame/invalid-input/persistence regressions. GPS timing and the EKF device-heading basis are unchanged.
- One-tap fullscreen for supported browsers and in-app Android/iPhone Home Screen instructions, including backup/restore guidance for separate browser storage.

### Fixed
- Website portrait and landscape screenshots preserve their aspect ratios at phone widths.
- Replaced PR #13's magnitude-only braking test, premature finalization, inconsistent heading application and silent persistence failures.

## [0.0.4-rc.5] - 2026-11-20

Release candidate; includes rc.4 backup/restore. The spatial matcher is software-validated and ready for controlled field evaluation; this release does not establish a measured timing-error bound.

### Fixed
- Estimator position variance now propagates velocity/bias variance with the correct time powers; nonfinite measurements cannot poison state. Diagonal covariance remains approximate.
- Cruise-bias residual learning converges to the full persistent offset instead of roughly half.

### Added
- Automatic landscape speed-and-timing cluster, explicit Dashboard escape, and bundled offline Inter/JetBrains Mono fonts.
- Position/time reference traces, route-position delta, and matched route progress for circuits and stages.
- Conservative direction, continuity, freshness, and overlapping-route rejection; local continuity across fresh stationary pauses.
- Spatial reference backup/restore coverage and complete synthetic-circuit matching tests.
- A sourced build-versus-buy guide comparing capabilities with RaceBox, RaceChrono Pro, and Performance Box Touch.
- A dark, silent aerial circuit background selected by the owner, with footer motion preferences, offscreen suspension, reduced-motion/data-saving fallback, and repository-only media notes.

### Changed
- Old distance-only references upgrade after a clean lap/run. The reference time and historical best have distinct labels.
- README, setup/technical docs, contributor examples, bot guidance, and website copy distinguish available features, configuration targets, and measured performance. Obsolete protocol descriptions and unsupported historical figures are corrected or annotated.
- Dashboard screenshot and demo backend updated together; backend assets use content-versioned URLs.

## [0.0.4-rc.4] - 2026-11-20

Release candidate; physical USB/drive acceptance remains required. Includes rc.3.

### Added
- Full JSON backup download and validated restore-as-copies for recordings, tracks, reference traces, and lap history.
- Integrity checksum, preview counts, selective recording/track backups, and version 2 track-export import.
- Atomic per-database restore with explicit partial-result reporting and retry of the remaining group.
- IndexedDB integration coverage for round trips, aborts, collisions, recovery, and concurrent session allocation.

### Fixed
- Saved track names use full-width, wrapping headings above their controls in both track panels, including on desktop and for restored copies.
- New recordings allocate IDs above existing sessions within one transaction, preventing imported timestamp IDs or concurrent starts from overwriting recordings.

## [0.0.4-rc.3] - 2026-11-19

Follow-up release candidate; hardware acceptance is still required. Includes all rc.2 changes.

### Changed
- Removed the G-meter from the default dashboard; optional Analysis retains the force plot for parked inspection.
- Clarified route capture versus saving, timing readiness, estimated turn counts, recording recovery, and estimator quality labels.
- Corrected the turn-rate readout unit to degrees per second.
- Upgraded the public demo to a varied synthetic circuit with consistent position/velocity, braking, acceleration, lap-to-lap pace differences, and a seeded reference.
- Added restrained website motion with pause, reduced-motion, and offscreen controls; refreshed screenshots and product descriptions.

## [0.0.4-rc.2] - 2026-11-19

Release candidate for bench and controlled-site validation, not a claim of measured timing accuracy. Select `blackbox-m9n.bin` for NEO-M9N (25 Hz configuration) or `blackbox-6m.bin` for NEO-6M (5 Hz). Verify actual sensor rates after boot. Full merged images replace device configuration.

### Added
- Driver-focused display, compact/expandable G-meter, and a five-second completed-lap result panel.
- Estimated distance progress after a usable reference lap; no corner-location or coaching claim.
- Explicit connection-loss, stale telemetry, and unavailable-delta states.
- Five-second recording saves, visible interrupted-session recovery, and emergency export of unsaved rows after storage failure.
- Session median/spread summaries and recording-health indicators for observed GPS validity and sample gaps.
- Timing evidence in CSV exports: device timestamps, lap counters/results, GPS age, and timing revision.
- Interactive website demo using the device dashboard with isolated synthetic telemetry/storage.
- Hardware acceptance checklist and NEO-M9N weekend data-collection instructions.

### Fixed
- Non-finite motion and invalid GPS coordinates are rejected before display/recording.
- Long-stage and finish-outage replay coverage, with clearer stage setup and completion feedback.
- Slower first laps after re-arming no longer overwrite faster saved references.
- Delta no longer extrapolates beyond reference samples; gaps, late starts, and large position jumps invalidate distance estimates.
- Track/reference storage now waits for commit; reference replacement is atomic and failed timing-stop acknowledgements preserve active configuration.
- Stored track names and IDs are escaped in both track lists; clearing a reference also clears its live display.
- Recorded-session duration uses sample timestamps instead of assuming a fixed sample rate; summaries stream chunks to limit phone memory usage.
- Track JSON exports retain binary reference samples and all stored lap history.
- Firmware installer and dependency actions are validated on the dedicated lat runner.

### Changed
- Consistent instrument typography, restrained signal colors, visible recording controls, and a dedicated landscape driving layout.
- Website, FAQ, and searchable guides distinguish implemented features from unvalidated accuracy/coaching claims.
- All build and deployment workflows use the self-hosted `blackbox-lat` runner.

## [0.0.4-rc.1] - 2026-11-20

Release candidate for hardware validation. No real USB write/boot or reference-timed drive has been performed for this candidate.

### Fixed
- Fresh-fix lap detection, crossing-time interpolation, GPS-outage continuity, stage re-crossings, and timestamp rollover.
- Persistent exact lap results and valid JSON during unavailable GPS acceleration.
- Earth-frame motion integration, orientation feedback, stationary speed units, and duplicate GPS assimilation.
- Sensor-loop LED stalls, malformed GPS fields, and missing-IMU calibration timeout.
- Browser recording races and v4 desktop telemetry decoding.
- USB installer merged-image offset (0x0), release download links, same-origin staging, and firmware verification.

### Changed
- Refreshed device dashboard, project website, setup documentation, and accuracy claims.
- Added deterministic replay/physics/protocol regressions, reproducible CI, and an isolated manually dispatched runner.
- Added crawlable technical guides, sitemap, and project metadata.

### Firmware variants
- `blackbox-6m.bin`: NEO-6M, 5 Hz GPS.
- `blackbox-m9n.bin`: NEO-M9N, 25 Hz GPS.
- Both are complete 4 MiB ESP32-C3 images for offset `0x0`; full-image flashing replaces device configuration. Verify the matching `SHA256SUMS-*.txt` before CLI flashing.
- The current IMU wiring uses GPIO18/19, also used by native ESP32-C3 USB. Verify board/USB-bridge compatibility; do not share those pins with an IMU and native USB concurrently.
- Keep the unit still during calibration, verify sensor rates, then follow `docs/REVIEW.md` before relying on lap results. Synthetic replay accuracy is not a physical timing-accuracy guarantee.

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

### Historical performance targets (not measured guarantees)

These original figures describe design targets, not validated results. In particular, the latency and ±1 m position figures were not established by reference measurements. Current capabilities and evidence are maintained in [the comparison guide](docs/COMPARISON.md).

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
