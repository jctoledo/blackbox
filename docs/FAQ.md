# Blackbox compatibility, accuracy, and ownership FAQ

## What is Blackbox?

Blackbox is an open-source ESP32-C3 vehicle telemetry project. It combines GPS and a WT901 IMU, serves a browser dashboard over local Wi-Fi, detects configured start/finish crossings, and records telemetry in browser storage. The source and current validation limits are public.

It is in active development. “Professional-grade”, “always reliable”, “90% of the functionality at 5% of the price”, and equivalent-to-commercial accuracy are **not established claims**. Earlier versions of this FAQ used those descriptions without supporting measurements; they have been removed.

## Can it replace a paid lap timer or data logger?

That depends on your requirements and measured results. Blackbox implements a lap timer, g-force display, circuit/stage recording, reference-lap delta display, and CSV exports. It does not establish parity with any named product's timing accuracy, durability, support, sensor configuration, or integrations.

Compare specific requirements: missed/false crossings, timing error against a reference, sunlight readability, installation, recovery after power/network loss, export compatibility, and support. Use the [hardware validation checklist](HARDWARE_VALIDATION.md). Do not infer equivalent performance from update-rate numbers or the number of decimal places on screen.

## Is there a subscription or cloud account?

The project requires neither. Firmware is MIT licensed. The device serves its dashboard locally; recordings are stored in the browser. Hardware, assembly, and validation have costs that vary by supplier and setup. There is no maintained fixed-price bill of materials or substantiated commercial price comparison.

## Can I try it without hardware?

Use the [interactive demo](https://jctoledo.github.io/blackbox/demo.html). It renders the actual dashboard with explicitly synthetic data and separate demo storage. A varied synthetic circuit includes changing speed, braking, acceleration, repeat-lap pace differences, and a preloaded synthetic reference. You can record/export it and exercise connection loss. The demo supports circuit timing only; it does not simulate arbitrary user-created course geometry or stage detection. It does not run physical sensors or prove firmware timing accuracy.

## Which board is compatible?

The published images target **ESP32-C3 with 4 MiB flash**, not every board bearing the ESP32 name. Check exposed GPIOs, power requirements, USB implementation, and the firmware pin assignment. ESP32, S2, S3, and C6 need their own build and wiring review.

The current firmware assigns IMU UART RX/TX to GPIO19/18 and GPS UART RX/TX to GPIO4/5, with the status LED on GPIO8. GPIO18/19 are also ESP32-C3 native USB D−/D+: the IMU and native USB cannot share these pins concurrently. Use a compatible external USB/UART bridge or change firmware pins and wiring together. See [Espressif's serial connection documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/establish-serial-connection.html).

Board names alone are insufficient to certify compatibility. The new battery/wake schematic in PR #39 has not passed a hardware acceptance review.

## Which GPS receiver should I choose in the installer?

Choose the firmware matching the receiver actually wired to the unit:

| Published candidate | Build configuration | Verification still required |
| --- | --- | --- |
| NEO-6M | 5 Hz | UART baud, accepted configuration, antenna, actual valid-fix rate |
| NEO-M9N | 25 Hz | UART baud, accepted configuration, antenna, actual valid-fix rate |

NEO-M9N is a **standard-precision, multi-constellation** receiver; this project does not establish multi-band/RTK capability. Its [manufacturer summary](https://content.u-blox.com/sites/default/files/NEO-M9N_ProductSummary_UBX-19027207.pdf) specifies up to 25 Hz under stated conditions. That rate is not a measured Blackbox rate or a lap-accuracy guarantee. Other NMEA receivers are not automatically compatible with the configuration commands, electrical interface, baud rate, or packet assumptions.

## Can I substitute another IMU?

The current driver targets the WT901 UART protocol. Another IMU requires a suitable driver, unit/frame conversion, configuration, and validation. Rigid mounting helps maintain a consistent frame; it does not by itself make measurements accurate. Keep the documented mounting orientation until a validated mounting-calibration path exists.

## How accurate is lap timing?

There is no published reference-timed hardware error distribution yet. Software tests cover interpolation, crossing direction, discontinuities, wraparound, and several synthetic trajectories. Those establish behavior for the tested inputs, not real-world accuracy.

The timer uses fresh receiver fixes and interpolates the crossing between them. It uses host arrival timestamps rather than synchronized receiver epochs/PPS. Receiver error, latency, multipath, line geometry, speed, and missed/coalesced UART samples all affect results. Millisecond storage is resolution, not millisecond accuracy.

## What happens if GPS or Wi-Fi is lost?

Stale/invalid GPS breaks crossing continuity; the timer does not invent a crossing through the outage. A missed start during a real GPS outage can therefore remain missed. There is no validated “30 seconds of accurate dead reckoning” guarantee.

The device's timing loop is separate from browser polling. Persistent last/best timing state helps the browser recover results after a polling gap, but it is not a complete persistent lap-event log. Browser recording cannot recover telemetry that it never received. The dashboard marks stale/offline values and reconnects; test power resets separately because they reset device timing state.

## Is the estimator a full covariance EKF?

No. It maintains seven state values using a diagonal covariance approximation. Cross-covariances and full Jacobian covariance propagation are not implemented. Position sigma is an internal diagnostic, not a calibrated error bound. GPS and stationary updates constrain drift conditionally; they do not eliminate it.

## Where are recordings stored? Can they be lost?

Recordings, tracks, and reference laps are local to the browser origin and device. Quota exhaustion, private browsing behavior, clearing site data, switching browsers, or losing an unsaved in-memory tail can lose data. Export important sessions before closing the tab or changing storage. A visible recording indicator does not mean the current tail has already committed to storage.

## Which browser supports USB flashing?

Use a desktop browser exposing Web Serial over HTTPS, such as compatible Chrome or Edge. The page detects support and verifies the selected image's chip header, size, and SHA-256 before enabling connection. A data-capable cable and the correct board boot mode are still required. Physical write/reset/boot testing remains part of hardware acceptance.

The merged image writes at address zero and replaces the full flash image, including device configuration. Browser-local recordings live separately. Use the release notes and back up the current setup before changing firmware.

## Does battery sleep, lever-arm estimation, or altitude fusion work today?

Those are open development issues, not completed features. Automatic sleep requires a verified power/wake design; IMU/GPS power draw can dominate standby even if the MCU sleeps. Automatic lever-arm estimation needs observable motion and frame/noise validation. Altitude fusion needs additional measurements and vertical-frame validation. Do not extrapolate another system's results to Blackbox.

## How should I evaluate it in a vehicle?

Begin on the bench, verify the mounting/axes, and test at a controlled location with an independent timing reference. Configure the device while stopped; use a passenger/operator or recorded logs during motion. See the [step-by-step hardware checklist](HARDWARE_VALIDATION.md) and [engineering review](REVIEW.md).

## What does estimated lap progress mean?

After a usable reference lap, the dashboard divides accumulated distance in the current lap by the reference's recorded distance. It shows whole percentages, capped at 99%; only the firmware's line crossing completes a lap. Different driving lines, position noise, and sample gaps affect the estimate. It is not map matching, remaining-time prediction, or a measured GPS confidence interval.

Progress and live delta are suppressed after detected connection/GPS gaps, late tracking starts, or large position jumps. A new lap resets tracking. Delta compares elapsed time at equal accumulated distance and does not extrapolate past the reference samples. Last/best completed times remain separate from these estimates.

## Can Blackbox call out corners or tell me when to brake?

No. The current reference stores distance/time samples, not a spatially matched corner model. Wrong identification would otherwise repeat each lap. Before adding coaching, we need position traces, ambiguous-section rejection, consistent corner detection across laps, and independent validation. A polished display is not evidence of those capabilities.

## Which display should I use?

Set up the track and recording while parked. Drive view emphasizes lap time, delta, speed, and last/best results while hiding track editing and the G-meter. Exit drive view to access setup and saved data. The standard dashboard also keeps the G-meter out of the way. Open Analysis while parked to inspect forces, and use Expand G-meter for a larger plot. A completed lap briefly shows its exact recorded result for five seconds. Drive view does not start recording automatically or keep a phone awake.

## Can I see whether my pace is becoming more consistent?

Open Tracks → Laps for a saved track. Sessions with at least two displayed laps include median time and spread (slowest minus fastest), alongside best time. Slow/in/out laps are included: this is a description of the displayed laps, not a clean-lap classifier, coaching score, or timing-accuracy measurement. History displays recent sessions from the latest 200 recorded laps.

Saved data shows sample count, GPS-valid percentage among received samples, the largest interval between browser sample timestamps, and gaps over 1.5 seconds. It does not count samples that were never received, prove good satellite geometry, or establish timing accuracy. Clock changes are flagged separately. Use these summaries to decide which recordings need closer inspection.

## What runs on my phone, and how does recovery work?

The dashboard is plain JavaScript served by the ESP32. IndexedDB stores recordings/tracks in the phone browser; there is no cloud backend. While receiving telemetry, recording requests a chunk save about every five seconds and waits for transaction completion before calling it saved. A hidden-page/page-exit save is best effort, not a guarantee against OS termination.

On reload, unfinished sessions remain accessible in Saved data, with a visible recovery notice. Collection does not silently resume or invent missing samples. If a storage write fails, recording stops, the failed batch remains in memory, and export includes committed plus pending rows without requiring another successful write. Keep the tab open until you export: memory cannot survive a reload/crash. A pending-buffer cap also stops collection if storage stalls.

This improves recovery but does not make browser storage permanent. Clearing site data, eviction, private browsing, switching origin/phone/browser, or OS termination can still lose data. Persistent-storage APIs are browser/security-context dependent and are not assumed available on the device's HTTP origin. See [browser storage behavior](https://developer.mozilla.org/en-US/docs/Web/API/Storage_API/Storage_quotas_and_eviction_criteria). Device-side logging or a native application is a future option for unattended/background capture.

Track JSON exports use version 2 and explicitly encode the reference's distance/time pairs as numeric arrays; native ArrayBuffers would otherwise serialize as empty objects. Track exports include all saved lap-history entries, even though the on-screen history is limited. Keep the original JSON and CSV files for analysis; Saved data → Choose backup also accepts these version 2 track exports (single or bundled). They have no original checksum; full backups add an integrity check. CSV and older version 1 track exports cannot be restored through this flow.


## How do I back up and restore everything?

While parked, stop session and route recording. Open **Saved data → Download backup**. The versioned JSON file includes recording samples, tracks, binary reference traces encoded as numbers, and all lap history. An unsaved recording tail is included without requiring a successful storage write. Confirm that the download actually reached Files/Downloads and copy it outside the browser, ideally to another device or your own backup storage. Blackbox does not upload it automatically.

On the destination browser, open **Saved data → Choose backup**, select the file, review its counts, then choose **Restore as new copies**. The full-backup checksum and data relationships are checked before any writes. Existing data is never replaced; track/reference/history IDs are remapped together. Restored recordings are stopped, and restored tracks are not automatically armed. Existing track timing may continue, and backup/restore does not require a device connection. Importing the same file a second time adds another set of copies.

Recordings and tracks use separate IndexedDB databases. Each group commits atomically, but both cannot share one transaction. If tracks fail after recordings commit, keep the tab open and choose **Retry remaining data**; it skips the committed recordings. Reloading loses that retry plan. Check Saved data before starting a fresh import after a reload to avoid duplicate copies. Keep the original backup file until restoration is verified.

The maximum supported JSON file is **100 MiB**. For larger libraries, use **Backup contents** to select one recording plus all tracks, or tracks only. A single recording over that limit still needs CSV export for analysis; CSV is not a restorable backup. Backups contain precise location history and are not encrypted. The CRC checksum detects accidental damage; it is not proof of who created a file. Browser eviction and hardware failure still require an external copy to protect your data.
