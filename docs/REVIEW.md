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

**Board pin compatibility:** the current IMU UART uses GPIO18/19, which are also native USB D−/D+ on ESP32-C3. Review the actual board/USB bridge and schematic before wiring or flashing; these signals cannot be shared concurrently with the IMU. The hardware PR #39 adds battery/wake circuitry but has not been electrically validated by this software review. The WIP mounting-calibration PR #13 changes several of the same fusion paths and needs rebasing plus controlled calibration tests before merge.

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

## Hardware acceptance plan

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
node --test tests/*.test.cjs
cargo run --quiet -p blackbox-host-tests --example packet_fixture > /tmp/telemetry-v4.hex
diff tests/fixtures/telemetry-v4.hex /tmp/telemetry-v4.hex
python3 -m unittest discover -s tests -p 'test_*.py'
cd sensors/blackbox
GPS_MODEL=m9n GPS_RATE=25 cargo +nightly-2025-10-18 clippy --release --locked -- -D warnings
GPS_MODEL=m9n GPS_RATE=25 cargo +nightly-2025-10-18 build --release --locked
```

The `blackbox-lat` runner is dedicated to this repository, limited to four CPUs and 8 GiB RAM, without host-directory or Docker-socket mounts. All workflows now target its custom label to avoid consuming GitHub-hosted minutes. CI runs on trusted main-branch pushes or explicit dispatch; pull-request code is validated by maintainer dispatch, never automatically on this persistent runner. Release tags and Pages deployments also use lat. One runner serializes jobs within the resource limit.
