# Drive recording: duration, GPS distance and recovery

With no selected track, Blackbox shows a Drive recording panel in the timing card. It works in portrait and the right-hand landscape frame. Choose **Record session** to start a browser-local raw recording. Read connection, GPS, IMU and storage readiness before setting off. A missing sensor does not fabricate a value or prevent collecting the other available evidence.

During recording, the panel prioritizes recording duration and **GPS distance · partial**. Choose **Stop recording** to freeze both immediately. Saving may finish afterward; keep any storage warning visible and use **Saved data** while parked to inspect or export the recording. If ordinary telemetry remains connected but the receiver timestamp log fails or stops delivering new events, Drive shows the same explicit GPS-log warning as Runs; qualified distance stays partial rather than being guessed. Selecting a circuit/stage restores the timing panel while raw recording continues. Clearing the track reveals Drive again. Rotation, reconnect and page reload never start or resume recording.

Speed units in **Menu → Speed units** also select km or mi for Drive distance. Stored distance is metres. Ordinary telemetry speed and Session CSV speed are km/h; GPS CSV receiver speed is m/s. Those recorded units do not change with the display preference.

## What the numbers mean

**Recording duration** measures monotonic browser elapsed time from successful recording creation to the Stop action. Network waits and final storage writes cannot extend the stopped value. Browser wall time remains calendar metadata and does not repair the duration. Backgrounding or a substantial difference between wall-clock and monotonic-clock changes marks the duration uncertain. Phone sleep and background recording are not guaranteed.

**GPS distance** integrates receiver speed over qualified consecutive receiver epochs using the trapezoidal rule: `0.5 × (previous speed + current speed) × receiver seconds`. It does not accumulate noisy latitude/longitude changes, integrate the displayed speed, infer movement from the IMU, or estimate missing distance. Course and mounting orientation are not inputs to this path-length estimate.

The panel shows known GPS seconds and reasons for excluded evidence. Zero appears only after qualified stationary intervals; before any qualifying interval, distance is unavailable. Every distance remains a **partial observed estimate**, including the conservatively omitted start/end spans. Neither coverage nor the qualification gates establishes measured distance accuracy.

The receiver endpoint can initially return 64 events from before Record. Blackbox preserves the entire first successful snapshot as raw evidence but excludes it from Drive distance. An empty successful snapshot establishes a zero-row fence; a failed request does not establish a fence. Only subsequent observations form distance intervals. At Stop, another fence closes Drive immediately; late final GPS rows may be preserved as raw evidence without extending the summary.

Version 1 qualification requires active and accepted GPS, speed between 0 and 100 m/s, valid receiver date/time with at least two fractional digits, the same boot/date, consecutive sequence numbers, no reported missing observations, increasing controller parse time, receiver intervals from 20 to 300 ms, and speed change no greater than 15 m/s². A failure breaks continuity. The next valid observation establishes a new anchor; there is no bridge across the excluded interval. Midnight conservatively breaks continuity too. Browser receipt time, receiver UTC and controller parse time are distinct clocks.

## Saving, interruption and historical summaries

Raw chunks and their Drive checkpoints commit in one recording-database transaction. The saved checkpoint cannot describe raw evidence beyond that transaction. Firmware/configuration metadata is merged independently into the same owning session without replacing Drive data.

After reload, the latest persisted recording may appear as **Saved recording** or **Interrupted · saved checkpoint**. An interrupted duration is shown with `≥` and is a checkpoint lower bound. Blackbox does not query the currently connected controller to fill a historical summary and never resumes recording automatically. Older recordings without a Drive summary show it as unavailable; their original samples remain available.

If a write fails, capture stops visibly and the unsaved raw tail remains in the current tab with its matching Drive summary. Keep the tab open and download a backup before closing it. An empty-tail backup can retain available summary context for an existing row; context alone never recreates a deleted recording. Browser page-hide writes are best effort and cannot recover observations never received.

## Saved speed and force summary

In **Saved data**, expand **Summary · saved samples** for that recording’s maximum qualified speed and strongest half-second acceleration, braking and left/right force estimates. These use committed ordinary telemetry, independently of Drive distance’s receiver rows. Pending samples are excluded until saved. The summary is recomputed after reload/restore and included in details JSON in SI units; older data lacking quality fields remains unavailable. See [summary calculations and limits](RECORDING_SUMMARY.md).

## Reproduce and move the evidence

While parked, use **Saved data → Recording details** for duration, known GPS distance/time, exclusion reasons and enrollment fence. Download **GPS CSV** and **Download details JSON**, or a complete restorable backup. GPS CSV alone contains the measurements but lacks the Drive fences and recorded monotonic duration.

The optional `session.drive` envelope is version 1, algorithm `receiver-speed-trapezoid-v1`. Its GPS-row ordinals count `rowType:1` rows in chunk-index/row order, independently of session IDs. Replay uses the same calculation and verifies cached distance, interval counts, exclusions and synthetic provenance against raw rows. Duration comes from the saved monotonic checkpoint, since raw wall timestamps cannot reproduce it. Strict validation rejects unsupported/malformed envelopes and evidence mismatches before backup restore writes.

New full backups use version 2 multipart files; version 1 backups remain importable. The `session.drive` envelope stays version 1 and telemetry remains v4. Restore adds new IDs/copies, retains stopped/interrupted meaning and never activates capture. The two destination databases still commit separately. Multipart staging and ID mappings persist across reload for safe retry; ambiguous commits stop for inspection. See [backup guidance](FAQ.md#how-do-i-back-up-and-restore-everything), [GPS logging](GPS_LOGGING.md) and [the first-drive plan](FIRST_DRIVE.md).

The public demo offers **Menu → Preview Drive recording panel** and [a direct Drive preview](https://jctoledo.github.io/blackbox/demo.html?view=drive). Seeded summaries are marked SIM and use the same receiver-evidence calculation. Synthetic checks do not validate road accuracy. Keep the normal mounting baseline upright and parallel to the car floor; tilted mounting remains separate R&D.
