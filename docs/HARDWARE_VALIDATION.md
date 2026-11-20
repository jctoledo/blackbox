# Blackbox hardware acceptance checklist

Use this checklist for a specific board, wiring revision, GPS receiver, mounting arrangement, browser, and firmware artifact. A passing software build is a prerequisite, not a substitute. Do not label the release hardware-validated until the evidence below is recorded.

## Record the test configuration

- [ ] Firmware tag, commit, binary filename, SHA-256, and GPS build rate.
- [ ] ESP32-C3 board revision, flash capacity, USB bridge/native-USB arrangement, antenna, power supply, WT901 settings, GPS model, and a wiring photo.
- [ ] IMU mounting axes and location; browser/OS version; dashboard address.
- [ ] Test date, operator, independent timing reference, reference resolution, and environment.
- [ ] Save the existing firmware/configuration if available; export browser recordings and tracks.

**Wiring gate:** the current IMU assignment is RX GPIO19 / TX GPIO18. These overlap native USB. Confirm the actual board routing before connecting the IMU and USB. Do not share these signals. Changing the pin mapping requires matching firmware and wiring changes. Check supply and signal voltages against the actual module specifications; do not assume a breakout label describes every pin's tolerance.

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
- [ ] Simulate Wi-Fi loss and a stalled connection. The browser must show stale/offline status rather than LIVE; reconnect without reloading if possible.
- [ ] Power-cycle the device with a track selected. The UI must require re-arming when firmware timing state resets.

**Pass:** valid data and loss/recovery are distinguishable; no indefinite calibration/network wait; expected receiver configuration is confirmed rather than inferred from the filename.

## 3. Axes, calibration, and mounting — bench, then controlled motion

- [ ] At rest on a level surface, record acceleration, yaw, tilt corrections, and bias estimates for at least two minutes.
- [ ] Tilt about each documented axis separately. Verify axis signs and gravity compensation; restore level and check recovery.
- [ ] At a controlled location, record straight acceleration and braking, then left and right turns. Confirm longitudinal/lateral signs and GPS course convention against the physical motion.
- [ ] Repeat after a power cycle and with the enclosure fully mounted. Investigate direction reversals, unexplained cross-axis acceleration, or calibration drift before relying on results.
- [ ] Do not enable the unmerged automatic mounting calibration: PR #13's magnitude-only brake gate can also accept acceleration.

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
