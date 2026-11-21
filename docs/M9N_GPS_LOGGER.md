# NEO-M9N GPS logging with ESP32-C3

Blackbox combines a NEO-M9N receiver, ESP32-C3 and WT901 IMU with a phone dashboard. Its GPS evidence stream helps answer a practical question: did the receiver epochs reach the firmware and the saved recording at the cadence you expected? This guide covers a first capture and how to interpret its timestamps. For assembly, start with the [DIY lap timer guide](DIY_GPS_LAP_TIMER.md) and [build guide](BUILD.md).

The current M9N firmware configuration targets **25 Hz**, or a calculated 40 ms between epochs. That target is not a measured result for an assembled logger. Antenna reception, receiver settings, serial traffic, firmware processing and phone collection can all affect the evidence you obtain. Browser update rate is a separate measurement.

## Prepare the receiver and firmware

Identify the actual receiver and antenna, check the module's supply and logic requirements, and use the M9N firmware variant. The current controller maps GPS TX to GPIO4 and GPS RX to GPIO5. GPIO18/19 are assigned to the WT901 and conflict with active C3 native USB; check your exact board before connecting or flashing it.

The [source build instructions](../README.md#building-from-source) use `GPS_MODEL=m9n GPS_RATE=25` with the pinned toolchain. Builds and automated checks for this repository run on lat. Keep the firmware version, checksum, module revision and installed antenna arrangement with each capture.

The [configuration](../sensors/blackbox/src/config.rs) selects 115200 baud for M9N, and the [u-blox initialization code](../drivers/ublox-gps/src/ubx.rs) sends receiver setup commands. Sending configuration does not establish that every setting took effect: the firmware does not yet read back all settings or explicitly budget all enabled messages. Inspect delivered epochs before declaring the installation a 25 Hz logger. The [rate budget](MATHS_AND_RATES.md#hardware-budgets) explains why baud, message traffic and receiver rate must be considered together.

## Capture a useful first session

1. Power on stationary and level, connect the phone to the device's Wi-Fi and open `http://192.168.71.1`.
2. Outdoors, wait for fresh GPS and inspect Diagnostics. Keep the dashboard in the foreground.
3. Choose **Record session**. With firmware supporting `/api/gps-log`, this also collects the separate GPS event stream. Check its status message; ordinary dashboard recording can continue even when GPS timestamp logging is unavailable.
4. Capture a few minutes parked, an ordinary drive and a final parked segment. Note antenna placement, reception conditions, phone/browser and any disconnections.
5. While parked, stop recording and wait for saving. In **Saved data**, download **GPS CSV**, **Session CSV** and a full backup; retain a track/stage export if relevant.

The receiver evidence is not a raw serial dump. Each checksum-valid GPRMC/GNRMC sentence produces an event when parsing finishes, including invalid-status messages, warmup samples and duplicate UTC epochs. Corrupt sentences, bytes lost before parsing, other NMEA messages and raw IMU packets are not captured in this stream. Unaccepted position fields export blank rather than repeating an old position.

## Understand the three clocks

| Timestamp | What it tells you | What it cannot establish |
| --- | --- | --- |
| `receiver_utc_us_of_day` with `receiver_date_ddmmyy` | RMC-reported UTC epoch and date; useful for receiver cadence and repeated epochs | Absolute timing accuracy merely from the number of fractional digits |
| `host_parse_us` | ESP32 monotonic microseconds when a complete RMC sentence was parsed | Antenna measurement time, first UART byte arrival or PPS synchronization |
| `browser_received_ms` | Phone wall clock when a fetched batch arrived | Individual receiver-event arrival times; several rows can share this value |

Keep the receiver date with the UTC field. It uses a two-digit year; pad the numeric date to six digits for display. Do not subtract UTC-of-day across midnight without handling the boundary. The supplied analysis tool excludes date transitions instead of guessing their alignment.

Use `boot_id` to separate device restarts and `rmc_seq` to follow parsed events within a boot. `rmc_active` records receiver status; `fix_accepted` records parser acceptance after its checks and warmup. Acceptance does not prove absolute position accuracy. `synthetic=1` marks demo data; keep it separate from real captures.

Session CSV carries the latest RMC metadata alongside dashboard samples. A repeated sequence there is another dashboard snapshot of the same GPS event. Use GPS CSV for event cadence, not the number of Session CSV rows. The [complete field reference](GPS_LOGGING.md#gps-csv-fields) documents units, missing values and boundary codes.

## Know where data can be lost

The [device logger](../sensors/blackbox/src/gps_log.rs) holds **64 events in RAM**. At the target 25 events/s, that is a calculated 2.56-second window. The [dashboard](../sensors/blackbox/web/dashboard.html) fetches it once per second while recording, removes duplicates and saves events in browser IndexedDB chunks. This does not provide device flash recording or guaranteed background-phone collection.

A long Wi-Fi interruption can overwrite events before the phone fetches them. `missing_before` counts skipped parsed-event sequences between saved rows in the same boot. It cannot count receiver output or serial bytes that never became parser events. Initial capture and restart boundaries have unknown missing history, not a proven zero. The first fetch can also contain events from before you pressed Record; compare device timestamps with Session CSV to select the intended interval.

## Inspect cadence before interpreting performance

The repository includes a coordinate-free timestamp summary tool:

```sh
python3 tools/python/analyze_gps_timestamps.py blackbox_gps_YOUR_SESSION.csv
```

Follow the repository's lat workflow when running analysis or automated checks. The tool reports receiver and host intervals, duplicate/backward epochs, sequence gaps, invalid fixes, boot segments and closely batched arrivals. Preserve the original CSV files alongside the JSON output.

Start with receiver intervals: are distinct valid epochs delivered near the configured target, and where are duplicates or gaps? Then compare host intervals: regular receiver epochs with clustered host parses suggest delivery or processing batching worth investigating. Finally examine sequence gaps and browser batch receipt to locate collection interruptions. These patterns guide investigation; they do not uniquely identify the faulty component.

Interval comparisons remove a constant clock offset but still include clock drift and delivery variation. They are not one-way latency measurements. The lap timer continues to interpolate its existing host timestamps; GPS logging does not implement receiver-epoch or PPS synchronization. A clean drive log demonstrates collection behavior, not lap-crossing accuracy.

## Pending real capture and installation evidence

No field measurements are published in this section yet. Add a reproducible capture before replacing these pending entries with results.

| Pending evidence | Required context |
| --- | --- |
| Receiver/antenna and installed-unit photos | Module revision, antenna location, USB arrangement and mounting |
| Parked/drive/parked capture | Firmware checksum, phone/browser, conditions and original exports |
| Receiver cadence summary | Duration, valid/invalid counts, interval distribution, duplicates and exclusions |
| Collection reliability summary | Missing sequences, reboot boundaries, disconnects and recovery behavior |
| Independent timing comparison | Reference method and error distribution; separate from cadence results |

Precise GPS exports contain location history. Keep original routes private and share coordinate-free summaries where practical. Use [hardware validation](HARDWARE_VALIDATION.md) for the remaining bench and controlled-crossing checks, and [GPS logging](GPS_LOGGING.md) for protocol details.
