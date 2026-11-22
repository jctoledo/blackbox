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

**Board pin compatibility:** the current IMU UART uses GPIO18/19, which are also native USB D−/D+ on ESP32-C3. Review the actual board/USB bridge and schematic before wiring or flashing; these signals cannot be shared concurrently with the IMU. The schematic in PR #39 identifies an ESP32-S3-WROOM-1-N16R8, not the ESP32-C3 firmware target; it needs a board-specific port and electrical validation. Do not flash the C3 release to that board. The original mounting-calibration proposal in PR #13 had a reproduced software blocker: magnitude-only acceleration detection accepted increasing-speed forward acceleration as braking, producing a 180-degree mounting correction. At the audited proposal revision, unit tests first failed compilation because two atan2 operands had ambiguous float types; correcting only those literals in a temporary lat probe exposed the false calibration. The current replacement workflow is described under Calibration PR revision below; its software checks do not establish physical mounting accuracy.

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

CSV recordings now retain browser epoch milliseconds (`time`), device uptime milliseconds (`device_time_ms`), current and completed lap times/count/flags, GPS age, and configuration revision. These are sampled browser observations, not a complete receiver/raw-IMU log. Offline periods leave gaps, and intervening lap events can be absent. Missing numeric fields export as empty cells instead of invented zeros. CSV row timestamps describe the received-sample span, including gaps; they are not the new Drive recording duration. Drive stores monotonic browser elapsed checkpoints, freezes at Stop and labels interrupted checkpoint lower bounds separately.

CSV export now uses the existing composite `[sessionId, chunkIndex]` key range in a single readonly session/chunk transaction. Each chunk is serialized and released before advancing; an owned pending tail is captured after the writer settles and appended once. Session/GPS serializers and the existing RollingRuns detector/finalizer remain shared with array-based consumers. Raw row/reference retention is bounded by the largest stored chunk plus pending tail (and bounded detector state), replacing whole-session `getAll` and quadratic repeated concatenation. Output strings/Blob still scale with CSV size. Fake IndexedDB regressions cover byte identity, numeric insertion order, mixed/unknown quality, pending ownership, concurrent writes excluded from the snapshot, aborted/failed export suppression and Runs retention refusal; they do not establish native-phone memory capacity. The 100 MiB version-1 JSON backup limit is unchanged; the multipart follow-up below addresses #138, with actual-phone acceptance tracked in #153.


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

The default portrait dashboard keeps the G-meter hidden; landscape offers it as a selectable instrument. Optional Analysis exposes forces while parked; Drive view remains focused on timing and speed. This separation leaves room for future post-drive analysis without adding unimplemented controls. Copy now distinguishes a captured route from a committed save, a session recording from route creation, and estimator confidence from physical GPS accuracy. Heuristic turn counts are labeled estimated. The turn-rate readout now states degrees per second rather than implying heading.

The public demo integrates distance over a smooth synthetic circuit with speed limited by curvature, varying sector pace, and a seeded reference. Packet position/velocity consistency, bounded acceleration, monotonic reference samples, lap-result timing, and connection loss are regression-tested. The synthetic source does not execute the firmware crossing detector and rejects stage configuration rather than pretending to validate it. No real driving data or location history is used.

The website motion treatment is decorative and separate from telemetry. Motion respects reduced-motion preferences and hidden/offscreen state; asset provenance belongs in docs/assets/README.md. It must not imply a measured Blackbox drive.


## Backup and restore acceptance

Version 1 full backups serialize both IndexedDB databases, including pending unsaved recording rows and portable reference arrays. CRC32 over the JSON data detects accidental corruption; strict schema and relationship checks run before importing. Restore creates new identifiers and uses `add`, never `put`, so a collision aborts rather than overwriting existing records. Each database group commits atomically; a retained retry plan skips the recording group after commit if the track group fails. This is explicitly not an all-or-nothing transaction across two databases. Reloading discards the retry plan; repeated fresh imports intentionally produce copies.

New recording sessions allocate their IDs within a serialized IndexedDB transaction above existing keys, preventing restored/future timestamp IDs or concurrent tabs from being overwritten. Integration tests use a pinned host-only IndexedDB implementation and cover full round trips, references/history, corrupt files, invalid geometry, abort rollback, collision preservation, partial retry, pending-tail export, selective backups, version 2 track exports, and concurrent new-session allocation. Browser acceptance additionally requires real download/file selection/restore/reload. The latest file-picker test stalled in the browser tooling, so that attempt is not recorded as a successful GUI import; the software restore/replay checks are separate evidence. Legacy version 1 backups remain manual, unencrypted files with a 100 MiB per-file limit; the app cannot guarantee against loss without a copy outside browser storage.

## Maths and documentation review — 2026-11-20

Prediction previously added `0.25*q_acc*dt²` to position variance and omitted velocity/bias propagation. It now uses the diagonal of `F P Fᵀ + Q`: velocity variance times dt² and acceleration/bias variance times dt⁴/4. Cross-correlations are still discarded, so this is not a full covariance filter or calibrated uncertainty. Regression checks use independently calculated expected variances.

Cruise correction previously averaged a residual as an absolute bias, yielding a half-offset fixed point. It now accumulates the residual; positive/negative offset convergence is tested. Nonfinite EKF measurements and orientation-learning inputs are rejected before mutation, with recovery tests for subsequent valid data.

Reviewed crossing geometry/interpolation, direction conventions, stage state transitions, GPS/IMU ingestion, transforms, spatial-reference matching and persistence boundaries. Remaining limitations include host receive timestamps, UART coalescing, mounting/yaw alignment, correlated diagonal uncertainty, GPS outliers that pass receiver validity, and unvalidated matching thresholds. These need measured evidence or further modeling, not speculative gain changes.

The document pass covered root README/CLAUDE/CLAUDE-design/CONTRIBUTING/CHANGELOG; all maintained docs guides, discovery/index and media notes; framework/sensor/driver READMEs; the hardware schematic description; sample-track provenance; and both historical simulator documents. Corrections include retired UI instructions, obsolete protocol tables, fictitious registry helpers, bare-metal framework claims, broken links, partition/build assumptions, unsafe calibration instructions, security/data handling, and S3-versus-C3 status. Third-party font licenses and sample-data attribution remain preserved. Historical changelog entries remain history. Public engineering content now describes architecture; the historical claims audit remains repository-only.


## Calibration PR revision

PR #13's horizontal acceleration magnitude could accept forward acceleration as braking and infer a 180° offset; short interrupted bursts could finalize without the intended duration, failures were immediately reset, persistence errors did not prevent activation, and changing EKF heading alone left acceleration frames inconsistent. The replacement requires signed fresh GPS speed loss, continuous consistent brake evidence, sticky candidate/error state and explicit parked save. Force vectors rotate into vehicle axes while learning residuals rotate back into device axes; EKF heading and GPS timing remain unchanged. One validated NVS record replaces the two-key validity scheme. Host tests cover fault paths and frame invariants; hardware latency, forward/reverse ambiguity and flash power-loss behavior remain explicit limitations.


## Recording and driver follow-up — rc.12 candidate

The no-track Drive panel makes an ordinary trip useful without inventing a lap or requiring performance maneuvers. Recording owns monotonic duration and qualified receiver-speed distance; Stop freezes both before finalization waits. Raw chunks and Drive checkpoints commit together, metadata merges preserve their owner, and strict replay checks imported distance against retained GPS rows. Missing spans remain missing; distance is always a partial observed estimate. Reload shows historical/interrupted context without resuming capture.

Saved data now derives maximum qualified speed and the strongest half-second force averages from committed ordinary telemetry. It streams chunks, keeps pending samples separate and exports versioned SI context. Unknown legacy evidence stays unavailable. The synthetic demo now carries boot identity through seeded rows and the actual live recording adapter, so populated examples exercise the same quality gates rather than bypassing them.

The landscape Runs chooser uses readable in-page options and two unit-dependent categories, with four independent literal detectors. Track suggestions reserve space, respect page-session dismissal, withdraw stale context and retain the coordinate origin paired with each position across asynchronous lookup. IMU consumers distinguish Euler heading rate from earth-vertical angular rate, and stationary evidence removes predicted gravity and resets across GPS/IMU gaps. These corrections improve internal consistency without validating arbitrary mounting.

The combined source tree passed [lat CI44962405419](https://github.com/jctoledo/blackbox/actions/runs/44962405419), including both receiver builds and software regressions. Browser checks covered populated summaries, pending-data exclusion, reload, units, compact warning/suggestion rows and focus/dismissal. A real file-picker import attempt stalled in the automation tool and remains unverified; actual Android/iPhone import, USB flash/boot and installed driving evidence are still acceptance work.

The practical next step is the documented M9N ordinary drive: stationary baselines, an uninterrupted foreground recording, exports/backup, and preserved antenna/mount/phone notes. It can expose cadence, storage, frame and dropout problems. Reference-timed crossings, labelled rolling positives/negatives, tilt experiments and independent force evidence are still required for physical accuracy, detection F1 or commercial-parity claims. Quarter-mile timing, slope correction and adaptive coaching remain R&D.

### Backup producer/consumer boundaries after rc.12

The ordinary recording adapter creates explicit `undefined` properties when optional GPS-event or older-firmware fields are absent. IndexedDB retains these properties, while backup v1 only admits finite numbers or null in sample values. Consequently a recording begun before its first RMC could fail backup validation even though its telemetry was retained. Backup export now projects missing properties to null in a fresh copy, before checksum/validation, for both committed chunks and the recovery tail. It does not relax imported-file validation or convert NaN, infinity or objects into valid evidence. Existing data and the v1 schema remain unchanged.

A separate boundary mismatch allowed a 1000-code-unit track name into a valid backup, then appended the restored-copy suffix beyond that limit. Restored copy names now reserve suffix space; original tracks remain intact and subsequent backups remain valid.

Focused regressions executed on lat: both new reproductions failed on rc.12 (invalid sample value / invalid track), then all 25 backup tests passed with the fixes. The missing-field regression invokes the actual production recording block, persists/restores its rows and also covers the unsaved-tail path; the name regression repeats restore and backup three times. Additional checks retain rejection of NaN, infinities and nested objects. These tests validate the data flow, not native phone file-picker behavior or storage retention guarantees.
## Gross-position-jump timing guard

Issue #125 reproduced a missing guard: two checksum-valid, finite fixes 200 m apart in 40 ms, both reporting 10 m/s, started timing. The maths guide had already described a displacement gate that did not exist. The application timing queue now breaks segments longer than `25 m + 100 m/s × host interval`, using the existing speed ceiling and an intentionally broad position-jitter allowance. The allowance is heuristic; it is not a measured GNSS error bound. The endpoint becomes a new anchor, so later ordinary consecutive fixes can recover without crossing the rejected segment. Raw GPS events and generic LapTimer geometry stay unchanged.

On lat, the new start regression failed before the fix and passed afterward. The host suite passed 200 tests and three doctests, including stage-finish rejection/recovery, 25 Hz and 5 Hz motion at the supported speed ceiling, and longitude projection at 60° latitude. This guards gross jumps only. Smaller GPS errors, bad course/speed evidence and crossing-time latency remain physical validation concerns; no absolute accuracy improvement is claimed.
## Follow-up: Session CSV analysis observations (#128)

The legacy offline analyzer crashed on a one-row CSV (reproduced on lat), described sample fractions as time shares, treated dashboard-speed finite differences as ground truth and recommended calibration/threshold changes from that comparison. The replacement streams a versioned JSON report with no filename, coordinates or absolute timestamps. It separates browser and same-boot device cadence, uint32 wraps, gaps/reversals/duplicates, provenance and quality-qualified km/h/g sample statistics. Missing legacy freshness stays unknown. It does not calculate classifier performance, calibrated accuracy, sustained Peaks or recording duration. The CLI argument remains a Session CSV; its output intentionally changes to JSON.

Independent Spec/Standards source review found one minor omission: known boot changes were hidden when device timestamps were unavailable. That was corrected with a regression. Lat validation covers empty/single and legacy exports, quality boundaries, clock wrap/reversal/reboot, units, malformed fields and reduced-report privacy. Seven initial regression methods passed; an eighth covers the review correction. This is an analysis-tool change, not firmware evidence or a new accuracy claim.
## Display-heading handedness

Issue #132 traced the production call chain from positive mathematical EKF yaw through SensorFusion to the packet and dashboard's compass-heading consumer. HeadingAligner learned a constant GPS-course-minus-yaw offset without reversing angular direction. A dynamic Ekf prediction → SensorFusion regression failed on lat: the first left turn produced +0.031416 rad compass heading where −0.039270 rad was expected. Static alignment tests had missed this because a learned offset can match any single orientation.

The aligner now converts `π/2 − mathematical yaw` before learning/applying its residual GPS reference offset. Tests cover both turning directions, a full wrap, initial offsets, unlearned cardinal basis and continued turn sense when low speed prevents relearning. All 200 host tests and three doctests passed on lat. Estimator prediction, stationary/covariance behavior, force transforms and raw-GPS timing remain unchanged. The historical CSV `ekf_yaw` name denotes this display-heading packet field; old recordings are not rewritten. Magnetometer/mount signs, GPS/course latency and absolute heading still require physical validation.
### Recording persistence during phone clock corrections

A five-second flush used browser wall time for both ordinary telemetry and GPS collection. After a backward clock correction, no periodic writes occurred until the wall clock caught up; the bounded pending buffer could stop an otherwise healthy recording. A forward correction caused a premature flush and misleading save-age labels. A separate monotonic completion clock now schedules both adapters and the age-since-save label. Calendar timestamps in the raw rows/chunk metadata and Drive's existing ownership, duration qualification and Stop fences remain unchanged.

Production-adapter regressions on lat reproduced both failures before the fix. The rollback case feeds 30 Hz telemetry plus 25 Hz GPS for 180 seconds: recording remains active beyond 6000 total incoming rows, every row remains saved or pending, and backward raw wall timestamps are preserved. Separate GPS-only and delayed-commit cases cover its scheduling path and completion-based age. This does not promise persistence while the phone suspends JavaScript or guarantee retention under browser eviction.

## Ordered track configuration follow-up

Rapid track selections previously overlapped device configuration and IndexedDB reference lookup. An older continuation could install its reference and labels after a newer selection or Stop. Complete activate/clear/delete commands, including saved-track lookup before device configuration, now run in invocation order, and activation publishes its track, revision, reference, history owner and labels without an intervening await. Failed commands leave the queue usable; failed reference lookup publishes no reference for the new owner. Production-source deferred-promise regressions cover rapid A/B selection, Stop during lookup, configure rejection, failed Stop and subsequent recovery. On lat, the two ordering regressions failed before the fix and all 240 browser regressions passed afterward. Independent review also found deletion and clear-all evaluating the active owner before the queue. Both complete intents now join the same queue, decide which owner to stop only when they execute, and call the internal Stop command without enqueueing recursively. Three additional deferred regressions failed before that correction; failed Stop also retains tracks and leaves later commands usable. Object-based suggestion/route-save activation re-reads its ID inside the same queue, preserving current saved geometry and new-stage state, so a continuation cannot revive a row removed by an earlier queued deletion. Three stale-object/lookup regressions failed before this final guard. Timing math, protocol and recording schemas are unchanged. This validates command ownership in software, not device/network latency or physical timing accuracy.

## Long numeric timing display

Landscape elapsed values overflowed their column at large minute counts: browser QA reproduced `1000:00.123` crossing into the adjacent delta area at 600×320. Length-aware font caps now use the timing card width for elapsed, history and numeric delta values; a small observer updates character counts only when displayed length changes, without reading layout. Normal font sizes remain the upper limit, including the 16 px history size in warning-reduced cards. The time formatter, precision, raw recording and protocol remain unchanged.

Browser review checked normal 844×390 and 667×375 views, offline warning plus Controls at 667×375, and forced maximum uint32-millisecond text `71582:47.295` at 600×320 with Controls visible/hidden and 667×375 with Controls hidden. Full digits remained inside their columns. These are synthetic formatter-input layout checks, not evidence of a real 49-day recording or device uptime. All 226 existing browser regressions passed on lat; the public demo was regenerated there from production UI.
### Motion display and elapsed-clock follow-up

Issue #142 found a second wall-clock boundary: a backward calendar step made the G-meter EMA interval negative and could overflow its exponential weight, poisoning finite force samples. Trail expiry and page SESSION/route-capture elapsed displays also used wall-clock differences. Those elapsed paths now use `performance.now()`, with null distinct from a valid zero origin. Real frame gaps over 250 ms and explicit clear/reset discard filter/trail history. Raw recording receipt values, track-point calendar stamps, creation metadata and device lap/Peaks clocks remain unchanged.

Lat regressions reproduced the force corruption, wrong trail expiry and negative elapsed displays before the fix. Afterward, 236 Node tests and 14 Python tests passed; focused strengthened checks also cover zero-origin filter weighting, recovery after a real frame gap, reset and preserved track-point calendar timestamps. Normal visual layout is unchanged; the public demo inherits the production source. Browser sleep/suspension and real sensor/heading accuracy remain separate validation limits.
### Zero-valued GPS coordinates

Issue #145 traced valid zero latitude/longitude from the receiver's explicit validity/reference flags to browser helpers that incorrectly treated zero as missing. JS origin enrollment and firmware-origin adoption could prevent route creation on a coordinate axis; route projection, observed distance and loop closure also used numeric truthiness. These checks now validate finite ranges, retain exact zero, and distinguish missing coordinate evidence from local-position fallback. Missing GPS clears its distance anchor instead of creating an observation at `(0,0)` or bridging to the next fix. Firmware zero-valued origins require explicit completed warmup, while legacy nonzero origins remain compatible. Exact polar origins remain outside the local projection.

Six initial boundary regressions failed on lat before the fix. Afterward, all 238 Node tests and 14 Python tests passed, including seven focused cases for Equator/Prime-Meridian circuits, initialized/unknown origins, invalid/missing values, finite local fallback and GPS distance continuity. Raw point calendar timestamps and firmware/estimator behavior are unchanged. These are software boundary checks, not a physical GNSS accuracy result.

## Integrated rc.13 software evidence

Fresh independent Standards and Spec reviews found zero findings in `9742d5d...9ee9e21`. The integration reconciled zero-coordinate route capture with monotonic capture time and validated the combined adapter. Full [lat CI 44975026545](https://github.com/jctoledo/blackbox/actions/runs/44975026545) passed 203 Rust host tests, 3 doctests, 277 Node tests, 31 Python tests, both GPS receiver builds and Clippy, the protocol artifact round-trip, embedded fonts and 17 guides / 18 published pages. PR #147's exact-head status changed from pending to success; authorization, build and reporting all ran on blackbox-lat. The main merge `6861161` retains the tested tree byte for byte.

Five current landscape views were captured from the actual generated dashboard with synthetic data at 844×390 and encoded on lat without resizing. Existing portrait, Drive and saved-data captures retain their native proportions. Hardware acceptance, actual phone memory and restore behavior, delivered rates, independent crossing error and tilted installation remain unmeasured. At rc.13, large restorable backups beyond the existing 100 MiB limit remained open in issue #138; incremental CSV export does not turn CSV into a backup.

The final public installer check verified the M9N rc.13 image and enabled Connect. At a narrower desktop width, the version-first selection label clipped the receiver identity. Catalog labels now lead with receiver/rate, keeping that choice visible while the Status card separately confirms the full verified version. Binary URLs, manifests and validation are unchanged.


## Multipart backup and safe recovery, rc.14

Issue #138 adds bounded version 2 multipart files and a final manifest while retaining version 1/legacy track imports. Source passes compare canonical record digests and counts through captured store fences, with the pending tail captured before awaits and metadata overlaid without consuming or requiring a successful save. Part descriptors bind exact UTF-8 bytes as well as semantic checksums. CRC detects accidental damage, not authenticity. The existing recording and track database versions stay unchanged; a separate staging database holds validated import data and persistent fresh-ID mappings.

Restore promotes child rows then marked roots with add-only transactions, one atomic destination group at a time. Callback-driven pending requests keep the destination transaction active while bounded staging reads proceed. Transactional attempt ownership prevents a competing tab from downgrading committed evidence; root markers recover a lost acknowledgement. Unknown absent or mixed/conflicting commits stop safely. Per-collection digests detect staging mutation/deletion before commit. Stage writes and terminal updates require the job still to exist, so discard cannot recreate jobs or leave invisible new rows. Recording/track commits remain separate and every later failure retains known committed-group context. Bounded Drive replay avoids loading all GPS rows when reopening a long recording. Its retained previous GPS anchor contains only six validated numeric fields, preventing an unusually large row from accumulating across checked sessions.

Focused lat regressions cover round trips, exact UTF-8 limits, source mutation, pending-tail detachment, cancelled validation, missing/corrupt/mixed/reordered input, quota/collision/source failures after destination adds, partial cancellation/retry, lost acknowledgements, controlled stale concurrent resumes, staging discard, record-size boundaries and legacy picker compatibility. The independent Standards/Spec reviews found and resolved stale-ledger, raw-byte binding, partial-status and discard races; reviewed test faults now assert that destination writes actually occurred before the injected late failure. The [workload record](research/LARGE_BACKUP_VALIDATION.md) documents deterministic 20/60/120-minute file round trips and process observations. Those tests and Node/fake-IndexedDB memory do not establish phone capacity or native Safari transaction scheduling.

The generated production demo's explicit per-part save, final manifest and completion message were observed at a phone-sized viewport; the [backup guide](BACKUP.md) includes the capture. Downloads are described as requested until the user checks Files/Downloads. Actual Android/iPhone multi-file selection, native quota/cancellation/reload behavior and Home Screen storage acceptance remain [#153](https://github.com/jctoledo/blackbox/issues/153). No physical device was flashed and no existing user recordings were opened or modified by this validation.


## Captured-route recovery and reference persistence (#158)

Route completion now freezes the capture before asynchronous work and retains one finalized identity until its add-only IndexedDB transaction commits or the user explicitly discards it. Invalid early finishes leave capture resumable. Rejected writes retain Save, portable version 2 track JSON download without another write, Continue and Discard. Duplicate actions are fenced while commit/configuration is pending; stale owners cannot publish recovery UI or start timing. A saved route with unconfirmed arming retries configuration without another save. Successful completion or closing a saved capture releases its recorder so track suggestions are not permanently suppressed. The existing explicit Save track & arm action remains; a failed response does not claim the device stopped timing.

Reference replacement still uses its existing single transaction. Failure now reaches the visible warning area without replacing the previous displayed or stored reference. A later successful reference clears that warning.

On lat, four targeted regressions failed against the unchanged baseline: consumed invalid finish, aborted actual IndexedDB write escaping the handler, missing saved/arming-failed outcome, and cancellation during an in-flight save. The updated suite passes all 317 browser regressions, including eight route/reference tests with fakeIndexedDB committed/aborted transactions, preserved original rows on ID collision, stable retry/export identity, freeze/discard, duplicate clicks, changed owners, save versus arm outcomes, and aborted reference replacement. No database version/schema changes or accuracy claims are introduced. Final compact-phone rendering and full firmware/site checks belong to the coordinating integration pass; native-phone storage remains physical acceptance.


Independent Spec review caught a recovery-export history envelope that the production importer rejected. The recovery file now uses the established `lapHistory.sessions` structure. Its regression parses the actual downloaded Blob with production `normalizeBackup` and `validateBackup`, compares the imported geometry/identity, and rejects any IndexedDB access during export. This stronger test failed on lat with `Invalid backup: invalid exported history` before the envelope correction.

## Driver receiver-log status and measurement scope

Issue #159 reproduced three software failures against rc.14 on lat: Runs continued to claim collection after GPS-log request failure, empty/repeated evidence stayed presented as collection indefinitely, and a late failed request from an older recording overwrote the new owner's status. The fix tracks monotonic event-delivery freshness and recording ownership separately from ordinary telemetry. It preserves accepted results and pending receiver evidence through temporary transport failure; the original sequence/time gates remain authoritative when data returns. Waiting/unavailable status is visible in Runs and the recording-backed Drive panel and announced only when its text changes. Escape also dismisses the setup menu and returns focus without changing recording or timing.

Speed maximum and elapsed page time now visibly identify their page scope, and the touch-only Peaks view identifies its half-second averaging window. Route overlays are bounded to the dynamic viewport and recovery actions wrap with touch-sized targets. These are presentation and persistence-state improvements, not new sensor accuracy, detection F1 or native-phone guarantees. Parent #155 tracks the related sensor and track-storage fixes; the existing physical/R&D acceptance remains open.

### Integrated rc.15 visual acceptance

The generated production dashboard was inspected at 390×844, 667×375, 844×390 and 844×320. Route quota-failure recovery remains readable and scrollable on short landscape screens; retry preserves the saved track when stage arming cannot be confirmed. Compact Peaks keeps each value and unit together using the instrument card width. Website captures are native-aspect screenshots of the actual synthetic preview, not measured driving results. The combined browser suite passed 323 tests on lat before the final CSS refinement; full exact-head CI covers the published branch. Physical sunlight, glove use, mounting and phone endurance remain hardware acceptance.

## Critical pass after rc.15: reference ownership and peak endpoints

Deferred reference regressions reproduced old-session failure messages replacing a new owner's warning, same-track reactivation accepting old publication, slower candidates replacing faster stored spatial references, and late saves recreating orphan references after track deletion. Reference transactions now verify track existence and compare spatial times atomically. Reference saves and clears preserve invocation order across delayed database opening, so an earlier pending save cannot silently undo a later clear. Publication captures both the active object and lap-history session; only the current owner's latest request changes its failure warning. Successful old-owner writes may remain saved for that original track but cannot overwrite the current interface. Legacy distance-only references can still upgrade to a clean spatial trace.

The mathematical review found no new confirmed production frame/crossing defect. It clarified the existing sustained-force algorithm: it integrates 500 ms windows ending at retained samples, not all continuous endpoints. An analytic triangle fixture produces 0.675 g versus the continuous optimum 0.6875 g. This is a documented sampling limit, not an algorithm/version change or physical accuracy claim.
