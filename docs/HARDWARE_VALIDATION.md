# Blackbox hardware acceptance checklist

Use this checklist for a specific board, wiring revision, GPS receiver, mounting arrangement, browser, and firmware artifact. A passing software build is a prerequisite, not a substitute. Do not label the release hardware-validated until the evidence below is recorded.

## Record the test configuration

- [ ] Firmware tag, commit, binary filename, SHA-256, and GPS build rate.
- [ ] ESP32-C3 board revision, flash capacity, USB bridge/native-USB arrangement, antenna, power supply, WT901 settings, GPS model, and a wiring photo.
- [ ] IMU mounting axes and location; browser/OS version; dashboard address.
- [ ] Test date, operator, independent timing reference, reference resolution, and environment.
- [ ] Save the existing firmware/configuration if available; export browser recordings and tracks.

**Wiring gate:** the current IMU assignment is RX GPIO19 / TX GPIO18. These overlap native USB. Confirm the actual board routing before connecting the IMU and USB. Do not share these signals. Changing the pin mapping requires matching firmware and wiring changes. Check supply and signal voltages against the actual module specifications; do not assume a breakout label describes every pin's tolerance.

**Target gate:** current release images target ESP32-C3. The proposed schematic in PR #39 uses ESP32-S3-WROOM-1-N16R8; it requires a separate firmware port and electrical review. Do not flash the C3 image onto that design.

## Weekend NEO-M9N data collection

1. Complete the bench, USB, and axis checks below before driving. Select the M9N firmware variant and record the artifact SHA-256.
2. Outdoors, wait for a fresh fix; record the Diagnostics GPS rate and satellite count. The configured target is 25 Hz, not proof of the actual delivered rate. Photograph the antenna position and IMU orientation.
3. While parked, select/record the track, arm timing, and press **Record session**. Check the recording indicator before switching to **Drive view**. Keep the dashboard foregrounded; screen lock or browser suspension can interrupt browser recording.
4. For a normal neighborhood/highway drive, collect a couple of minutes parked outdoors, the ordinary drive, and a short parked segment at the end. No hard braking, weaving, or special maneuvers are needed. Repeated line-crossing validation belongs at a controlled site with an operator/reference. A passenger/operator should note any missed or unexpected crossing and an approximate time. Do not operate the UI while driving.
5. While parked, stop recording. Open **Saved data**, export both Session CSV and GPS CSV (on timestamp-logging firmware), and use **Tracks → Export** for the track geometry. Keep a short text note with firmware, GPS model, wiring/mounting, browser/phone, weather/obstructions, and observed problems. Export before clearing site data.
6. An independent timer or timestamped crossing video is needed to measure timing error. Without it, the logs can establish internal consistency and investigate failures, but not absolute lap-time accuracy. Share location-containing recordings privately, not in a public GitHub issue.

## 1. USB installation and recovery — bench

- [ ] Select the correct GPS variant. The installer must show the expected verified version before connection.
- [ ] Cancel the serial-port picker, then retry. The page must remain usable.
- [ ] Flash the correct ESP32-C3 using a data cable; record whether BOOT/RESET is required.
- [ ] Reset after completion. Confirm firmware boots, the expected Wi-Fi mode appears, and the dashboard loads.
- [ ] Reconnect after an interrupted connection without writing to an unintended port.
- [ ] If a different ESP chip is available, verify rejection before any erase/write. Do not use a valuable device merely to perform this negative test.

**Pass:** successful correct-chip write/reset/boot; cancelled/failed connections recover; no wrong-chip write. Record any configuration reset caused by the full image.

## 2. Startup, sensors, and stale-data handling — bench/outdoors

- [ ] Keep the unit motionless through startup calibration; save the serial log and Diagnostics screenshot.
- [ ] Test missing IMU at startup using power-off rewiring. Calibration must exit with a visible/logged failure rather than hang indefinitely.
- [ ] Outdoors, wait for valid GPS and warmup. Record accepted fix rate, IMU packet rate, loop rate, and actual receiver configuration. Distinguish received packets from estimator updates.
- [ ] Remove valid GPS reception by a controlled method. Within the firmware freshness window (currently 1.5 seconds without a new valid fix), the GPS status must stop implying a fresh fix. Restore reception and confirm recovery.
- [ ] With controlled sensor-output configuration, interrupt gyro or angle packets while GPS and telemetry continue. G-meter/readouts must become unavailable and peaks must stop accumulating; verify IMU age in Session CSV. Do not disconnect live wiring.
- [ ] Simulate Wi-Fi loss and a stalled connection. The browser must show stale/offline status rather than LIVE; reconnect without reloading if possible.
- [ ] Power-cycle the device with a track selected. The UI must require re-arming when firmware timing state resets.

**Pass:** valid data and loss/recovery are distinguishable; no indefinite calibration/network wait; expected receiver configuration is confirmed rather than inferred from the filename.

## 3. Axes, calibration, and mounting — bench, then controlled motion

- [ ] Compare startup bias and stationary residuals on level ground and modest nose-up/nose-down driveway inclines, without remounting. Capture AHRS roll/pitch and serial calibration results; the revised gravity subtraction is not physically validated yet.
- [ ] Try gently disturbing the unit during calibration, then letting it settle: rejected evidence must not complete early.
- [ ] At rest on a level surface, record acceleration, yaw, tilt corrections, and bias estimates for at least two minutes.
- [ ] Tilt about each documented axis separately. Verify axis signs and gravity compensation; restore level and check recovery.
- [ ] At a controlled location, record straight acceleration and braking, then left and right turns. Confirm longitudinal/lateral signs and GPS course convention against the physical motion.
- [ ] Repeat after a power cycle and with the enclosure fully mounted. Investigate direction reversals, unexplained cross-axis acceleration, or calibration drift before relying on results.
- [ ] If using experimental Mount alignment, follow the candidate/save workflow below; do not assume that a software-accepted candidate establishes physical alignment.

**Pass:** axes/signs and frame conventions agree with the known motion; no nonfinite values or discontinuous corrections. Record residuals rather than claiming a universal g-force accuracy bound.

## 4. Lap start/finish — controlled site

Use an operator/passenger or inspect logs after each run; do not manipulate the dashboard while driving. Mark a physical reference line and use a sufficiently wide software line spanning the route.

- [ ] Arm before approaching the start. Confirm visible Armed / Waiting for GPS states are appropriate.
- [ ] Perform at least 20 intended-direction crossings across multiple speeds and lateral positions. Log each physical crossing and each detected start/finish.
- [ ] Perform at least 10 wrong-direction crossings and check rejection.
- [ ] Test approaches near each finite line endpoint, a near miss outside the segment, and starting parked close to the line.
- [ ] Leave the unit stationary near the line for at least 10 minutes; check for false laps.
- [ ] Test a valid crossing immediately after arming, a normal loop, and a point-to-point run. Re-crossing a stage start during a run must not restart timing.
- [ ] Introduce a GPS outage around a crossing. A crossing across missing data must not be fabricated; document that a real crossing can be missed during an outage.
- [ ] Interrupt browser polling/Wi-Fi across a completed lap. After reconnecting, check exact last/best time and count against device output. Do not assume all intermediate lap events can be reconstructed.
- [ ] Repeat with both GPS variants you intend to support; record actual fix rates and antenna conditions.

**Functional pass:** zero unexplained misses/false triggers in the defined valid-signal cases, correct direction and stage behavior, and explicit handling of outages. These sample counts are a minimum smoke test, not a statistical reliability guarantee.

## 5. Timing accuracy against a reference

- [ ] Choose the required timing-error budget **before** collecting data. Record the independent reference's own uncertainty and synchronization method.
- [ ] Measure repeat crossings/laps at several speeds, line angles, and lateral positions; preserve raw telemetry and reference times.
- [ ] Report signed bias, median absolute error, 95th-percentile absolute error, maximum error, missed starts, false starts, and sample count.
- [ ] Keep ideal synthetic-test error separate from physical measurements. Never substitute display decimals, GPS Hz, or a manufacturer's position specification for measured lap-time error.

**Accuracy pass:** the predefined budget is met under documented conditions, with no omitted failures. If no budget or adequate reference was defined, report “accuracy not established.”

## 6. Recording, storage, and export

- [ ] Record a full intended-length session; stop and export it. Confirm CSV timestamps/order, speed units, position, and acceleration fields.
- [ ] Reload the browser and export the saved session again; compare row counts and content.
- [ ] Record while changing views and while telemetry pauses; gaps must remain identifiable rather than becoming invented samples.
- [ ] Exercise storage failure in a disposable browser profile. The UI must report the failure; a failed reference replacement must not destroy the prior reference.
- [ ] Delete a disposable track and confirm its reference/history are removed. If it is active, stop must be acknowledged by the device first.
- [ ] Export important data before clearing storage. Never use your only copy for destructive tests.

**Pass:** stored data survives reload, exports are usable, gaps/failures are visible, and failed writes do not silently report success.

## 7. Power and endurance

- [ ] Run for the intended session duration with the intended supply and enclosure. Record brownouts, resets, reconnects, temperature, memory trend, and storage growth.
- [ ] Test Wi-Fi reconnect and receiver recovery after a controlled power interruption.
- [ ] For battery hardware, measure current for the **whole assembled unit** in every claimed mode, including sensors and regulators. Automatic shutdown/wake is not implemented/validated by the current release.
- [ ] Do not claim battery life from MCU sleep current alone. Charger protection, cell behavior, wake routing, and power domains require a separate electrical review.

## Results record

| Test / condition | Firmware & hardware | Expected | Observed | Evidence file | Pass / fail / untested |
| --- | --- | --- | --- | --- | --- |
| USB write / reset / boot | | | | | |
| GPS / IMU loss and recovery | | | | | |
| Axes / mounting / calibration | | | | | |
| Crossings / false triggers | | | | | |
| Reference timing error | | | | | |
| Recording / export / storage failure | | | | | |
| Endurance / power interruption | | | | | |

Keep failures and untested cases visible. Promote a candidate to stable only when its intended hardware configuration meets the agreed acceptance criteria; a different board, mounting change, or GPS configuration needs renewed checks.

## Point-to-point stage acceptance

- [ ] First record and save the route with separate start/finish lines. Stop safely before marking the finish. The initial route recording is not a timed stage.
- [ ] On a later pass, arm before the start and cross each line in its recorded direction. Verify the completed result survives reconnect/reload of the browser's saved history.
- [ ] A second start crossing during an active run must not restart the clock; crossing the finish before starting must not create a run.
- [ ] GPS loss spanning the finish must not fabricate completion. Document a missed finish and stop/re-arm while parked.
- [ ] Test a short stage before a long run. The software includes a deterministic 20-mile replay, but this does not replace physical testing.
- [ ] Export both session CSV and stage geometry. An ordinary highway run without an independent reference cannot establish timing accuracy.

## Phone storage and bad-data acceptance

- [ ] Record for at least 15 seconds, wait for the saved status, reload, and export the interrupted session from Saved data. Confirm no automatic resumption or overwritten chunks.
- [ ] In a disposable browser profile, simulate quota failure. Recording must stop visibly, pending samples must remain exportable, and no success should be shown before transaction completion.
- [ ] Suspend/lock the phone and restore it; inspect recorded gaps and do not assume background capture. Confirm the same browser/origin is used when recovering.
- [ ] Compare exported row counts/timestamps and GPS-valid fractions to the saved-session health summary. Bad/missing data is not a zero measurement.


## Backup and restore — browser acceptance

- [ ] Stop session/route recording. Download a full backup and verify the file exists outside browser storage.
- [ ] In a separate browser profile, choose the file, review counts, and restore copies. Check recording row counts/values, track geometry, reference delta, and lap history.
- [ ] Reload that profile and export a restored recording. Keep the original backup until content is verified.
- [ ] Restore into a profile containing disposable existing data; confirm the existing data remains and restored items have separate IDs.
- [ ] In a disposable profile, exercise transaction failure. Check partial-result messaging and retry only the remaining group while the tab stays open.
- [ ] Keep location-containing backups private. Do not clear the only existing copy to test recovery.

## Spatial reference acceptance

- [ ] Back up saved data before updating. Verify old reference times and best results remain visible; a new clean lap/run upgrades the reference without changing historical best.
- [ ] On a repeated controlled route, compare delta at the same physical locations on slightly different driven lines. Save CSV and a full JSON backup, which includes spatial reference points.
- [ ] Check a stationary pause: delta hides, then resumes locally when fresh positions and direction return. Test GPS loss separately: do not expect uninterrupted comparison or automatic recovery.
- [ ] At nearby parallel/crossed sections, verify ambiguous matches are hidden rather than jumping along the route. Check start/finish seam behavior and repeated stage starts.
- [ ] Record phone/browser, GPS installation, reference time, and observed failures. The matching gates are heuristics; independently timed crossings remain necessary to establish physical timing error.

## Landscape cluster and offline fonts

- [ ] While parked, start recording and rotate portrait → landscape → portrait. Timing and recording must continue, and controls must return in portrait.
- [ ] In landscape, use Dashboard to return to setup. Check speed, elapsed time, reference, recording and connection states in both orientations.
- [ ] Disconnect the phone from internet while staying on Blackbox Wi-Fi. Confirm the instrument fonts load from the device page.
- [ ] Check long stage times, daylight glare, mounting distance, light/dark themes and browser bars/notches. A browser viewport test cannot establish in-car readability.
- [ ] Test connection loss: stale values must be visibly identified and delta hidden. Stop while parked, export CSV and download a full backup.


## Mount alignment and phone launch acceptance

- [ ] Back up browser data. Record firmware, phone/browser, mounting orientation and the saved offset before changing anything.
- [ ] On both Android and iPhone, check portrait/landscape, notches and address-bar behavior. Android: enter/exit Full screen. iPhone: install through Safari → Add to Home Screen → Open as Web App if offered. Confirm the dashboard remains connected on device Wi-Fi.
- [ ] Check recordings/tracks after Home Screen launch; restore a backup as copies if storage is separate. Test reload and export. Do not assume background recording works.
- [ ] With a nearly level, secure mount at a controlled site, confirm ordinary forward acceleration cannot complete mount calibration. Test sustained straight forward braking, interrupted braking and turning separately.
- [ ] Park before saving. Confirm the candidate does not affect forces before Save, the previous offset survives Cancel/failure, and a successfully saved offset reloads after power cycling.
- [ ] Compare known aligned, 90° and 180° mounts. Check acceleration/braking and left/right signs after saving; preserve logs and photos. Do not use reverse braking as calibration evidence.
- [ ] Test stale GPS/IMU and NVS failure on a bench setup; no false success, indefinite collection or lost prior calibration. A successful NVS API call is not a physical power-loss test.
- [ ] Record rejected attempts, actual receiver cadence, road slope/bank and any sign mismatch. The software tests do not establish alignment-angle accuracy.

## GPS evidence capture

- [ ] Record at least two minutes with M9N outdoors; GPS CSV should show receiver epochs and host parse times. Check the actual cadence with [the timestamp analyzer](GPS_LOGGING.md).
- [ ] While parked, disconnect the phone for five seconds, reconnect, stop, and export. Sequence loss beyond the 64-event RAM window must appear in `missing_before`.
- [ ] Power-cycle the device during a disposable recording. The GPS CSV must show a new boot boundary; UTC must not be fabricated during reacquisition.
- [ ] Reload the phone, export again, then back up and restore as a copy. Check GPS row counts and timestamps match.
- [ ] Keep the phone foregrounded for the intended drive duration; record any collection gaps, unavailable logger status or storage failure.

## Mounting tilt consistency acceptance (issue #104)

Software regressions do not close physical incline/mounting validation (#73/#21).
Record module identity, WT901 installation-direction settings, firmware commit,
mounting photos and measured roll/pitch. Establish the upright, nearly level, parallel-to-car-floor baseline first. In a
separate R&D experiment compare modest fixed dash-like pitch/roll, while keeping the road/site conditions comparable.
The startup/alignment collection gates are not accepted physical tolerances.

- At rest, verify startup bias and sustained stationary detection on level ground and
  a modest driveway incline; then return to level and compare residuals.
- Check known left/right turns at matching speed against an independent angular or
  lateral reference, including compound fixed tilt. Inspect all three raw gyros.
- Check straight travel with pitch/roll motion and rough-road vibration for false
  cornering/stationary evidence. Repeat with warm/cold hardware.
- Interrupt gyro, attitude and acceleration independently: forces must become
  unavailable in HTTP/dashboard without interrupting healthy GPS timing; recovery
  must collect fresh stationary evidence before reasserting parked state.
- Assess magnetic interference, vibration, temperature and rigid attachment for the
  actual dash/hood installation. Keep road grade/bank distinct from fixed mounting.

Do not infer arbitrary/inverted mounting support, full 3D vehicle attitude,
validated bank/grade compensation or tyre loads from these checks.

## Runs chooser phone acceptance (issue #106)

- [ ] Check opened options match the closed Runs button typography and remain touch-readable on actual Android and iPhone browsers, including short landscape with warning rows. Check scrolling, selection, Escape/keyboard focus and outside dismissal where supported.
- [ ] Switch mph/km/h and confirm the selected lower-speed or overtaking family persists while its own literal preset/result changes; recording and timing continue. Desktop device emulation is software layout evidence only.
