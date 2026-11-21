# Build a DIY GPS lap timer with ESP32-C3

Blackbox is an open-source GPS lap timer and telemetry instrument built around an ESP32-C3, a supported u-blox receiver and a WT901 UART IMU. It serves a local dashboard to your phone over Wi-Fi, times circuits and point-to-point stages, and exports recordings you can inspect. This guide takes a first build from choosing parts to a saved session; the [assembly guide](BUILD.md) and [lap timer guide](LAP_TIMER.md) cover the detailed procedures.

Choose this project if understanding and modifying the instrument is part of the appeal. Assembly, mounting and validation are part of the work. There is no published complete-build price or reference-timed accuracy result yet. The [comparison with packaged lap timers](COMPARISON.md) explains the tradeoffs without treating DIY hardware as a tested commercial equivalent.

## Choose a compatible build

The implemented firmware target is **ESP32-C3**. The repository's ESP32-S3 schematic is a separate proposal, not a wiring alternative for the current image.

| Part | What to establish before buying or wiring |
| --- | --- |
| ESP32-C3 development board | Exact revision, flash capacity and whether its USB connector uses native USB or a serial bridge |
| NEO-M9N or NEO-6M receiver and antenna | Actual module identity, supply and logic requirements; matching firmware configuration |
| WT901 UART IMU | UART variant, power requirements, baud and enabled acceleration/gyro/attitude output |
| WS2812 status LED | Whether your board has a compatible LED on GPIO8 or needs an external one |
| Power, enclosure and mounting | Suitable supply, insulation, retained connectors and cable strain relief |
| Phone and browser | Local Wi-Fi dashboard and browser storage for the recording workflow |

The firmware targets 25 Hz for M9N and 5 Hz for 6M. These are configuration targets, not measured delivery rates for your build. Budget for all the parts, antenna, enclosure and installation, rather than quoting the controller's price as the cost of a complete lap timer.

## Check the signal wiring before flashing

These assignments come from the current [firmware entry point](../sensors/blackbox/src/main.rs). Use the [build guide](BUILD.md#2-wire-against-the-actual-board) and the documentation for your exact boards to complete power and ground wiring.

| Connection | ESP32-C3 pin |
| --- | --- |
| GPS TX → controller RX | GPIO4 |
| Controller TX → GPS RX | GPIO5 |
| WT901 TX → controller RX | GPIO19 |
| Controller TX → WT901 RX | GPIO18 |
| Status LED data | GPIO8 |

**GPIO18/19 also carry ESP32-C3 native USB.** Do not attach the IMU UART to those pins while using that native USB connection. Identify your board's USB arrangement first; a firmware pin change and its physical wiring must agree. This signal table does not establish voltage compatibility.

## Install the matching firmware and prove the bench setup

Follow the [README installation instructions](../README.md#quick-start) and select the receiver variant that matches the hardware. Source builds use the pinned toolchain and explicit GPS configuration documented in [Building from Source](../README.md#building-from-source). Repository builds and automated checks run on the dedicated lat runner. Keep the firmware version and image checksum in your build notes.

Power on with the sensor stationary and level. Join the device's **Blackbox** Wi-Fi network with the credentials in the README and open `http://192.168.71.1`. Wait for fresh GPS outdoors, then inspect Diagnostics. Confirm that GPS fixes arrive and acceleration, gyro and attitude remain fresh. A lit LED or moving dashboard is only an initial check.

Before installation, record a short disposable session, stop it, and export its Session CSV, GPS CSV and full backup from **Saved data**. Reload the dashboard and check that the saved session remains available. Exercise Wi-Fi reconnect and use the [hardware acceptance checklist](HARDWARE_VALIDATION.md) to record failures as well as successes.

## Mount it, then record an ordinary drive

Secure the enclosure so its orientation cannot change, give the antenna a suitable view of the sky, and document both locations. Stationary startup bias calibration and [sensor-to-car alignment](CALIBRATION.md) solve different problems; neither justifies assuming arbitrary mounting is automatically calibrated. Verify acceleration/braking and left/right signs with gentle known motion.

While parked, start recording and keep the phone dashboard in the foreground. Include a stationary segment before and after an ordinary drive. Stop and export while parked, waiting for saving to complete. Recordings live in the browser's IndexedDB; this is not autonomous session storage on the ESP32. Keep external backups, especially before changing browsers or installations.

## Set up circuit or stage timing

At a suitable controlled site, open **Tracks**, record the route and save it as a circuit or a stage. For a stage, stop safely at the end and choose **Mark Finish**. Select the saved track with **Use** to arm timing for a later pass. Review the line direction and displayed state before attempting a timed run. Follow the [operating guide](LAP_TIMER.md) for reference pace and lap history.

Timing uses the segment between consecutive valid receiver positions, intersects a finite timing line and interpolates host timestamps. It checks direction and continuity rather than triggering merely because the car is nearby. Missing GPS can mean a missed crossing. The estimator and a faster browser display do not supply extra measured GPS positions.

Times are stored in milliseconds, but that is a representation, not a physical accuracy claim. At a shallow crossing angle, a position error can produce a larger timing error. See [the timing mathematics and rate budgets](MATHS_AND_RATES.md) before interpreting a finely printed result.

## Pending real build photos and timing evidence

No assembly or driving evidence is supplied by this section yet. It reserves a place for documented results, without substituting demo screenshots for hardware proof.

| Pending addition | Evidence and caption to collect |
| --- | --- |
| Parts and open-enclosure photos | Exact module revisions, labelled connections, USB arrangement and strain relief |
| Installed sensor and antenna photos | Car-forward direction, sensor axes and actual antenna location |
| First-drive recording | Firmware checksum, phone/browser, mounting notes, Session CSV, GPS CSV and backup |
| Controlled timing comparison | Independent reference, crossing geometry/speeds, sample count, rejected passes and error distribution |

Add real photos to the [build journal](BUILD.md#5-keep-a-build-journal) once available. Keep private routes and identifying details out of public images. Until independent crossing measurements are published, the useful first milestone is a reproducible build that captures and exports evidence reliably.
