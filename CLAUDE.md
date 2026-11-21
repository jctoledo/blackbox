# Repository engineering guidance

Blackbox is an ESP32-C3 GPS/WT901 telemetry instrument. Firmware uses ESP-IDF and Rust; the browser UI uses embedded HTML/JavaScript and IndexedDB. The S3 schematic is a separate proposal, not a compatible firmware target.

## Work and validation

- Use the actual source and current tests as authority. Read `docs/SENSOR_FUSION.md` before modifying frames, timing or estimation; `CLAUDE-design.md` before UI changes.
- All builds, automated tests, generated demo/docs, media encoding and font embedding run on **lat**, using the dedicated `blackbox-lat` runner. Do not consume GitHub-hosted minutes. Do not automatically execute unreviewed fork PRs on the persistent runner.
- Use `codex/` branches for Codex work. The maintainer can dispatch CI for reviewed branches; merge only after relevant checks pass.
- For runner/cache changes, read `docs/BUILD_INFRASTRUCTURE.md`; firmware output stays checkout-local to preserve ESP-IDF Git-derived version metadata.
- CI commands and pinned toolchains live in `.github/workflows/ci.yml`. Host tests target `sensor-fusion`, `wt901`, `ublox-gps`, and `blackbox-host-tests`, not the ESP32 workspace on the host target.
- Embedded builds run from `sensors/blackbox` with explicit `GPS_MODEL=m9n GPS_RATE=25` or `GPS_MODEL=6m GPS_RATE=5`, nightly-2025-10-18, locked dependencies and Clippy warnings denied.
- Keep assertions focused on physical invariants, state transitions, persistence failures and protocol boundaries. Do not mistake passing synthetic tests for hardware acceptance.

## Critical boundaries

GPS course is clockwise from north; local east/north bearings and timing-line directions use mathematical angles. Convert `π/2 − course` only where required. Track recording already produces mathematical directions. Verify mounting/WT901 yaw semantics with real data rather than assuming automatic calibration.

The estimator integrates earth-frame acceleration and uses diagonal covariance. It does not implement CTRA, full covariance propagation, RTK or calibrated error probabilities. The force-display orientation/blending path is distinct from EKF prediction. Timing uses consecutive valid receiver positions and velocities, interpolated with host receive timestamps; receiver epoch/PPS synchronization is not implemented.

Binary telemetry is v4, **82 bytes**. Use `binary_telemetry.rs`, the shared fixture and Python/JS decoders rather than copying a stale offset table. Legacy decoder support is explicit. Latitude/longitude in the packet are f32 and may lose metre-scale precision; firmware crossing calculations retain f64.

## UI and data

Production: `sensors/blackbox/web/dashboard.html`, embedded by `websocket_server.rs` despite that module's historical name; transport is HTTP polling. Demo: `scripts/build_demo.py` generates the same UI with a labeled synthetic backend. `tools/dashboard-dev` is historical.

Preserve recording/track databases and transactional writes. Backup restore adds fresh IDs, never overwrites originals. Two database groups are not one atomic transaction; partial failures must remain explicit. CRC is corruption detection, not authentication. GPS traces and backups contain location history.

## Documentation and claims

Lead with implemented capabilities and configuration targets. Use `docs/COMPARISON.md` for sourced product comparison; keep maintainer findings in `docs/REVIEW.md`. Update website assets when UI changes. Do not claim millisecond physical accuracy, drift-free operation, automatic mounting calibration or commercial parity without measured evidence. Rolling speed intervals are implemented with evidence gates; standing-start timing, slope compensation and adaptive coaching remain proposals.

Check the current README wiring: GPIO18/19 conflict with C3 native USB. Confirm the physical board before changing/flashing hardware. See `docs/HARDWARE_VALIDATION.md` for bench and drive acceptance.

## Agent skills

Matt Pocock skills installed in the maintainer's Codex are documented in `docs/agents/SKILLS.md`. For issue operations, use GitHub `jctoledo/blackbox` and the existing labels; preserve hardware/R&D issues until their evidence is collected. For content/discovery changes, read `docs/research/DISCOVERY.md` and validate rendered navigation with `scripts/check_docs_site.py` on lat. Public launch drafts and the maintainer's photo/data plan live in `docs/LAUNCH_KIT.md`.
