# Engineering review and validation

This review covers the lap timer, coordinate maths and fusion, GPS/IMU ingestion, telemetry protocol, device dashboard, recording, browser installer, CI, public claims, and open issues. It is based on source inspection, deterministic host tests, browser checks, and ESP32-C3 cross-builds. No physical device was flashed and no instrumented drive was performed during the review.

## Confirmed failures addressed

| Area | Failure and effect | Change / evidence |
|---|---|---|
| Lap start | Raw GPS positions were checked against lagging EKF velocity; held fixes were repeatedly evaluated at telemetry rate | Fresh-fix detection uses receiver velocity from the same measurement; crossing replay regressions |
| Lap timestamps | Segment intersection fraction was discarded | Interpolate crossing time; test uneven intervals, multiple headings/latitudes, and u32 rollover |
| Outages / stages | Long GPS gaps could form false crossing segments; re-crossing a stage start reset the timer | Invalidate continuity and reject gaps over two seconds; stage start only accepted while armed |
| Browser lap results | Transient flags were lost between polls; first completion could disappear; displayed last lap used a prior sample | Persistent exact last/best times, timer state, and configuration revision in atomic HTTP snapshots |
| HTTP telemetry | Unavailable acceleration serialized as NaN, invalidating JSON | serde_json emits null for nonfinite values and escapes text; actual serializers tested on host |
| Coordinate guidance | Saved-track origin differed from the new device origin | Transform saved line into current frame; preserve f64 GPS coordinates in HTTP metadata |
| Motion model | Turning prediction rotated earth-frame velocity by absolute yaw again | Integrate earth-frame acceleration once; northbound-turn regression |
| Orientation learning | Feedback treated residual error as an absolute angle; roll sign and confidence counter were incorrect | Closed-loop pitch/roll convergence tests, independent roll confidence |
| GPS assimilation | Repeated/duplicate updates over-counted observations; invalid derivatives refreshed stale acceleration | Assimilate once per new fix; remove duplicate position and correlated scalar-speed updates; finite/time validation |
| Stationary detector | A 3.5 threshold was interpreted in m/s rather than km/h; repeated polls counted as evidence | Explicit SI threshold, fresh-fix count, no GPS velocity overwrite immediately after ZUPT |
| Main-loop latency | Connectivity/settings LED flashes slept for up to 900 ms in the sensor loop | Nonblocking LED status; GPS LED uses freshness rather than historical warmup alone |
| GPS parsing | Malformed fields could reuse prior motion; std/no_std RMC implementations differed | Shared allocation-free parser; complete finite motion fields, coordinate bounds, valid-only warmup; both build modes checked |
| Calibration | Missing IMU could leave an unbounded busy loop | Timeout and scheduler yield while awaiting samples |
| Recording | A completed async write erased new samples; request success was mistaken for transaction commit | Serialized detached batches; transaction completion; ordered recovery after abort; stop capture before flush |
| Desktop receivers | All three receivers expected obsolete 67-byte v1 packets | Shared v1/v4 decoder, strict checksum, TCP fragmentation/recovery, Rust-generated golden packet |
| USB installer | Missing release filename; merged image written at application offset 0x10000 | Verified same-origin binary, chip/size/SHA-256 validation, merged-image offset 0x0, explicit state handling |
| Build infrastructure | Moving nightly and host OS broke ESP-IDF; flash tools required newer host Rust | Pinned firmware nightly and Ubuntu; stable host tools; host regression CI; manually dispatched isolated runner |

The current ESP-IDF dependency enables the GPS driver's `std` feature. The old alternate RMC implementation therefore affected no_std consumers, not this firmware build; parser consolidation is coverage and portability hardening, not evidence of the reported hardware failure.

## Claims audit

| Previous claim | Assessment / replacement |
|---|---|
| “Professional-grade”, rivals named commercial systems | No comparative track measurements supplied. Removed; describe implemented features and validation limits. |
| 30 Hz means ~33 ms timing accuracy | Display cadence is not physical accuracy. Crossing interpolation reduces sampling quantization; GPS error and latency remain. |
| 200 Hz fusion can deliver 1–2 ms accuracy | Unsupported without synchronized timestamping and reference measurements. At 100 km/h, 1 ms is only 2.8 cm along track. |
| 25 Hz NEO-M9N | Supported as a receiver maximum, not a guaranteed measured application rate. Verify configuration and diagnostics. |
| ~1 cm f32 GPS coordinates / millimeter GPS accuracy | Incorrect conflation of numeric representation and measurement quality. Binary f32 has location-dependent quantization; HTTP/timing geometry uses f64. |
| 200 predictions per second | Configured sensor output is not necessarily processing cadence; UART batching and scheduling matter. Diagnostics must measure both. |
| Drift eliminated / perfect zero-velocity observation | GPS and stationary updates constrain drift conditionally; neither supplies perfect truth. |
| Fixed parts prices and commercial-price comparisons | Removed from the project pitch; supplier prices and breakout capabilities vary. |
| Browser flashing works | Source and release validation now cover the discovered defects. Real USB write, reset, and boot remain hardware acceptance tests. |

Primary reference: [u-blox NEO-M9N product summary](https://content.u-blox.com/sites/default/files/NEO-M9N_ProductSummary_UBX-19027207.pdf) specifies up to 25 Hz and meter-scale horizontal CEP under its stated conditions. CEP is not a worst-case error bound or a lap-time guarantee. The flasher's [official implementation](https://github.com/adam-weber/esp-webflash-toolkit/blob/c9b61384b1217545bdb70e7762e4e4fa36dfdc7c/src/core/firmware-flasher.js) defaults to application offset 0x10000, so Blackbox must override it for merged images.

## Remaining engineering risks

**Board pin compatibility:** the current IMU UART uses GPIO18/19, which are also native USB D−/D+ on ESP32-C3. Review the actual board/USB bridge and schematic before wiring or flashing; these signals cannot be shared concurrently with the IMU. The schematic in PR #39 identifies an ESP32-S3-WROOM-1-N16R8, not the ESP32-C3 firmware target; it needs a board-specific port and electrical validation. Do not flash the C3 release to that board. The WIP mounting-calibration PR #13 also has a reproduced software blocker: magnitude-only acceleration detection accepts increasing-speed forward acceleration as braking, producing a 180-degree mounting correction. Its existing unit tests first fail compilation because two atan2 operands have ambiguous float types; correcting only those literals in a temporary lat probe exposes the false calibration. It must not be merged as a validated mounting solution.

1. **Timestamp and sample integrity.** GPS crossings use host arrival times, not synchronized receiver epochs/PPS. UART batches can coalesce fixes and IMU samples; IMU diagnostics count packets while prediction consumes the latest sample. Capture arrival timestamps and receiver epochs before claiming higher-rate accuracy. Do not add EKF-based timing solely to increase the displayed decimal precision.
2. **Estimator covariance and heading.** The diagonal approximation omits position/velocity/bias correlations. Absolute WT901 yaw, mounting orientation, GPS compass course, and ENU mathematical yaw need a controlled heading/turn test. Do not interpret position sigma as a calibrated error bound.
3. **Filtering and calibration.** The display fusion path operates at telemetry cadence, after high-rate IMU ingestion. Decimation can alias vibration. Mounting-angle gravity compensation and startup bias calibration need level/tilted static tests and acceleration/braking replays. Cruise/tilt/orientation adaptation should be assessed together, not only in isolated tests.
4. **Storage lifecycle.** IndexedDB lives in a browser origin, is quota-limited, and may be cleared by the browser. Export recordings regularly. A failed batch is retained in memory, so closing that tab before retry/export can lose the unsaved tail.
5. **Local network trust.** The AP has a documented shared password and device controls are not authenticated individually. Keep it on a trusted local network; this review does not establish an Internet-facing security model.
6. **Published firmware versus source.** The web installer serves the published release. Merging changes is not equivalent to producing a new hardware-validated release. Version, GPS model, build rate, and release artifact must be recorded for every field test.

## Open issues reviewed

- [#36: 200 Hz EKF lap timing](https://github.com/jctoledo/blackbox/issues/36): interpolation and fresh-fix detection address parts of the underlying problem. Keep open: 1–2 ms real-world accuracy is unproven, and EKF timing needs latency/covariance/frame validation first.
- [#26: automatic sleep](https://github.com/jctoledo/blackbox/issues/26): keep open. Sleep/wake behavior depends on board power domains, sensor power, wake hardware, and logging continuity. ESP32 deep-sleep current alone does not determine whole-unit battery life; a 2000 mAh capacity divided by 170 mA is an ideal estimate, excluding converter losses and usable-capacity effects.
- [#21: lever-arm estimation](https://github.com/jctoledo/blackbox/issues/21): keep open. A single IMU plus GPS cannot simply assume acceleration at the center of gravity is known. Start with measured mounting offset and repeatable turns; automatic estimation needs observability and noise analysis.
- [#14: altitude EKF](https://github.com/jctoledo/blackbox/issues/14): keep open. Requires vertical-frame/bias validation and additional measurement hardware. Accuracy reported by another system is not transferable to this implementation.

Existing hardware and dependency PRs are outside the implemented fix stack and must be reviewed on their own merits. They are not evidence that these issues are complete.

## Follow-up software and PR review

- PRs #30–#35 were integrated through #53 and validated on lat: both GPS firmware variants, host regressions, Clippy, artifact upload/download equality, and Pages packaging. All workflows remain lat-only.
- PR #40 upgrades a direct embedded-svc dependency to 0.29 while esp-idf-svc 0.51 still uses 0.28. The current application has no direct embedded_svc imports. The unused direct dependency has been removed instead; esp-idf-svc retains its compatible transitive version. This avoids adding a redundant API generation. The lockfile was refreshed by Cargo on lat; it removes unused optional feature dependencies without upgrading package versions. A future service-API migration should move with the ESP-IDF wrappers.
- The dashboard follow-up addresses stalled requests falsely remaining LIVE, unacknowledged timing-stop requests, premature track-save success, nonatomic reference replacement, and reference/history deletion consistency. New regressions exercise commit/abort and request timeout behavior.
- The public demo uses the actual dashboard with isolated browser databases and an explicitly synthetic backend. It is a product demonstration, not a firmware-in-the-loop or accuracy test.
- The old FAQ's price/performance ratio, blanket board/GPS compatibility, “always reliable”, multi-band M9N description, and 30-second dead-reckoning guarantee were unsupported and replaced with explicit configuration/validation limits.
- Issue #36 also overstates its illustrative drift calculation: from rest with constant 0.01 m/s² acceleration error for 0.04 s, displacement error is 0.5 × 0.01 × 0.04² = 0.000008 m, not 0.008 m. This idealized calculation excludes initial velocity error, timestamp misalignment, vibration, and receiver errors; it cannot justify a physical timing guarantee.

## Hardware acceptance plan

Use the expanded [hardware checklist and results record](HARDWARE_VALIDATION.md) for bench, USB, sensor, mounting, crossing, reference-timing, recording, and endurance gates.


1. Save the current firmware/configuration and export browser recordings. Record board, GPS module, antenna, IMU orientation/rate, and firmware commit.
2. Verify the installer on desktop Chrome/Edge: download/hash, correct-chip connection, wrong-chip rejection, write, disconnect/reset, and boot. Repeat a cancelled port selection and a connection timeout.
3. Static test: verify calibration completes; unplugged IMU times out; GPS warmup and loss are visible. Confirm no false laps from noise while stationary.
4. At a safe controlled test location, record a wide start line. Cross in both directions at several speeds. Test arming before the line, starting near the line, finite-line endpoints, and a GPS outage near the line. Export the telemetry and timing state.
5. Repeat circuits and stages with an independent timing reference. Compare error distributions, missed starts, false starts, and worst cases—not only average error. Test phone polling gaps and Wi-Fi reconnection.
6. Review logs before changing sensor-fusion gains or enabling inferred crossings during outages. Promote a hardware-validation candidate to a stable release only after its GPS configuration and boot/drive results are recorded.

## Reproduce automated checks

```sh
cargo test --locked -p sensor-fusion -p wt901 -p ublox-gps -p blackbox-host-tests
cargo check -p ublox-gps --no-default-features
npm ci --ignore-scripts
node --test tests/*.test.cjs
cargo run --quiet -p blackbox-host-tests --example packet_fixture > /tmp/telemetry-v4.hex
diff tests/fixtures/telemetry-v4.hex /tmp/telemetry-v4.hex
python3 -m unittest discover -s tests -p 'test_*.py'
cd sensors/blackbox
GPS_MODEL=m9n GPS_RATE=25 cargo +nightly-2025-10-18 clippy --release --locked -- -D warnings
GPS_MODEL=m9n GPS_RATE=25 cargo +nightly-2025-10-18 build --release --locked
```

The `blackbox-lat` runner is dedicated to this repository, limited to four CPUs and 8 GiB RAM, without host-directory or Docker-socket mounts. All workflows now target its custom label to avoid consuming GitHub-hosted minutes. CI runs on trusted main-branch pushes or explicit dispatch; pull-request code is validated by maintainer dispatch, never automatically on this persistent runner. Release tags and Pages deployments also use lat. One runner serializes jobs within the resource limit.


## Driver UX and recording follow-up

The UI separates parked setup, Drive view, and force analysis. Drive view hides editing controls and the G-meter, keeps lap time/delta/speed visible, and briefly shows the exact completed lap result. Portrait and landscape need physical mounting/glare/glance testing even after browser QA. No phone wake-lock or background recording guarantee is made.

Position-matched delta replaces accumulated-distance comparison for new references. Samples retain x/y/time in a persistent geographic frame, with GPS fix age subtracted from comparison times. Segment projection uses a 25 m spatial lookup grid, a 12 m corridor, 60° direction agreement, sequential progress bounds, and rejection of competing nearby branches more than 25 m apart along the reference. Circuit seam candidates are treated as the same seam within 15 m of each endpoint. These are conservative heuristics, not confidence probabilities or physical accuracy guarantees. Stationary fresh fixes maintain a local match without displaying a delta; gaps and invalid traces suppress it. The reference label is distinct from historical best. Legacy references require a clean spatial replacement; backup preserves both supported formats. Synthetic circuit, overlap, coordinate-frame, staleness, continuity, and restore tests cover implementation behavior; field replay validation remains required.

A session-best firmware flag resets on arming. It must not overwrite a faster browser reference from a prior session: regression coverage now preserves the saved reference and separately chooses the fastest usable distance trace. Track names/IDs are escaped in list markup, and reference deletion waits for commit before clearing the displayed reference.

CSV recordings now retain browser epoch milliseconds (`time`), device uptime milliseconds (`device_time_ms`), current and completed lap times/count/flags, GPS age, and configuration revision. These are sampled browser observations, not a complete receiver/raw-IMU log. Offline periods leave gaps, and intervening lap events can be absent. Missing numeric fields export as empty cells instead of invented zeros. CSV row timestamps determine session duration; data gaps are included in that elapsed span.

### Product usefulness assessment

- **Useful now, subject to physical validation:** open, inspectable GPS line-crossing lap timing; live speed/G feedback; circuit and stage setup; browser-local recording/export; reproducible firmware and regressions; a clear DIY path with no project subscription requirement.
- **Potentially useful but approximate:** position-matched reference delta and progress. Good for exploration and coarse pace feedback once tested; unsuitable as evidence of braking-point or apex precision.
- **Not established:** commercial-product timing parity, centimeter positioning, millisecond physical accuracy, sustained GNSS-denied accuracy, environmental durability, automatic mounting calibration, and adaptive corner coaching.
- **Highest-value next evidence:** reference-timed M9N crossings plus antenna/mount/actual-rate metadata, followed by timestamp alignment, raw sample logging, and reference-path matching with ambiguous-section rejection. Higher display rates alone cannot solve measurement uncertainty.

The project can be compelling as an enthusiast instrument and a transparent engineering platform. Its competitive argument is ownership, repairability, customization, and inspectable evidence. A better-looking dashboard must not be presented as equivalent to a validated commercial timing/coaching system.


### Follow-up evidence and research tickets

The phone uses JavaScript/IndexedDB, not a cloud backend. Saving was shortened from a minute to approximately five seconds while receiving data. Aborted writes restore the unsaved batch and stop collection; export snapshots saved plus pending rows without another write. Reload exposes unfinished sessions without silently resuming. Browser suspension, quota/eviction, and unsaved tails remain limitations; no background/durable-storage guarantee is made.

The normal-drive collection plan and future acceptance criteria are tracked in [#54 corner matching](https://github.com/jctoledo/blackbox/issues/54), [#55 timestamp/crossing evidence](https://github.com/jctoledo/blackbox/issues/55), and [#56 braking/cornering comparisons](https://github.com/jctoledo/blackbox/issues/56). These are research plans, not implemented coaching features. Neighborhood/highway driving is useful for continuity and frame checks; independent crossing observations are needed for accuracy.

Stage validation now includes a deterministic 20-mile replay and a missing-GPS finish rejection case. UI tests cover exact completed-stage results, preservation of faster references across arming, nonfinite data rejection, persistence failures, CSV evidence, and emergency export. All compilation and automated test execution remain on lat.


## Copy, demo data, and motion review

The primary dashboard no longer shows a G-meter. Optional Analysis exposes forces while parked; Drive view remains focused on timing and speed. This separation leaves room for future post-drive analysis without adding unimplemented controls. Copy now distinguishes a captured route from a committed save, a session recording from route creation, and estimator confidence from physical GPS accuracy. Heuristic turn counts are labeled estimated. The turn-rate readout now states degrees per second rather than implying heading.

The public demo integrates distance over a smooth synthetic circuit with speed limited by curvature, varying sector pace, and a seeded reference. Packet position/velocity consistency, bounded acceleration, monotonic reference samples, lap-result timing, and connection loss are regression-tested. The synthetic source does not execute the firmware crossing detector and rejects stage configuration rather than pretending to validate it. No real driving data or location history is used.

The website motion treatment is decorative and separate from telemetry. Motion respects reduced-motion preferences and hidden/offscreen state; asset provenance belongs in docs/assets/README.md. It must not imply a measured Blackbox drive.


## Backup and restore acceptance

Version 1 full backups serialize both IndexedDB databases, including pending unsaved recording rows and portable reference arrays. CRC32 over the JSON data detects accidental corruption; strict schema and relationship checks run before importing. Restore creates new identifiers and uses `add`, never `put`, so a collision aborts rather than overwriting existing records. Each database group commits atomically; a retained retry plan skips the recording group after commit if the track group fails. This is explicitly not an all-or-nothing transaction across two databases. Reloading discards the retry plan; repeated fresh imports intentionally produce copies.

New recording sessions allocate their IDs within a serialized IndexedDB transaction above existing keys, preventing restored/future timestamp IDs or concurrent tabs from being overwritten. Integration tests use a pinned host-only IndexedDB implementation and cover full round trips, references/history, corrupt files, invalid geometry, abort rollback, collision preservation, partial retry, pending-tail export, selective backups, version 2 track exports, and concurrent new-session allocation. Browser acceptance additionally checks real download/file selection/restore/reload. Backups remain manual, unencrypted files with a 100 MiB per-file limit; the app cannot guarantee against loss without a copy outside browser storage.

## Maths and documentation review — 2026-11-20

Prediction previously added `0.25*q_acc*dt²` to position variance and omitted velocity/bias propagation. It now uses the diagonal of `F P Fᵀ + Q`: velocity variance times dt² and acceleration/bias variance times dt⁴/4. Cross-correlations are still discarded, so this is not a full covariance filter or calibrated uncertainty. Regression checks use independently calculated expected variances.

Cruise correction previously averaged a residual as an absolute bias, yielding a half-offset fixed point. It now accumulates the residual; positive/negative offset convergence is tested. Nonfinite EKF measurements and orientation-learning inputs are rejected before mutation, with recovery tests for subsequent valid data.

Reviewed crossing geometry/interpolation, direction conventions, stage state transitions, GPS/IMU ingestion, transforms, spatial-reference matching and persistence boundaries. Remaining limitations include host receive timestamps, UART coalescing, mounting/yaw alignment, correlated diagonal uncertainty, GPS outliers that pass receiver validity, and unvalidated matching thresholds. These need measured evidence or further modeling, not speculative gain changes.

The document pass covered root README/CLAUDE/CLAUDE-design/CONTRIBUTING/CHANGELOG; all maintained docs guides, discovery/index and media notes; framework/sensor/driver READMEs; the hardware schematic description; sample-track provenance; and both historical simulator documents. Corrections include retired UI instructions, obsolete protocol tables, fictitious registry helpers, bare-metal framework claims, broken links, partition/build assumptions, unsafe calibration instructions, security/data handling, and S3-versus-C3 status. Third-party font licenses and sample-data attribution remain preserved. Historical changelog entries remain history. Public engineering content now describes architecture; the historical claims audit remains repository-only.


## Calibration PR revision

PR #13's horizontal acceleration magnitude could accept forward acceleration as braking and infer a 180° offset; short interrupted bursts could finalize without the intended duration, failures were immediately reset, persistence errors did not prevent activation, and changing EKF heading alone left acceleration frames inconsistent. The replacement requires signed fresh GPS speed loss, continuous consistent brake evidence, sticky candidate/error state and explicit parked save. Force vectors rotate into vehicle axes while learning residuals rotate back into device axes; EKF heading and GPS timing remain unchanged. One validated NVS record replaces the two-key validity scheme. Host tests cover fault paths and frame invariants; hardware latency, forward/reverse ambiguity and flash power-loss behavior remain explicit limitations.
