# GPS timestamp evidence for your drive

Use **Record session** while connected to the device. With firmware supporting `/api/gps-log`, the dashboard also collects a separate GPS event stream. After stopping while parked, open **Saved data** and download both **Session CSV** and **GPS CSV**, plus a full backup and your track/stage export. Older firmware can still record dashboard samples, but cannot provide this GPS log.

The GPS CSV is the useful new evidence for issue #55. It helps distinguish receiver cadence, duplicate epochs, UART/application batching and phone collection gaps. It does not establish absolute position or crossing accuracy.

## What is actually captured

Each checksum-valid GPRMC/GNRMC sentence gets a sequential event at the point the UART polling code finishes parsing it. This includes invalid-status RMC messages, warmup samples and repeated UTC epochs. Invalid/unaccepted position fields are blank instead of repeating the previous position. Corrupt checksums, bytes lost before parsing, other NMEA sentences and raw IMU packets are not logged.

The device retains **64 events in RAM**: about 2.56 seconds at 25 Hz or 12.8 seconds at 5 Hz. The phone fetches that window once per second while recording, removes duplicates and saves new events in the same IndexedDB chunks as the recording. GPS events survive the existing backup, restore and interrupted-session recovery flows. No device flash logging or background-phone guarantee is implied.

A longer disconnect can overwrite the RAM window. The first recovered row counts missing sequence numbers in `missing_before`. Device restarts begin a new segment; the first row's missing history is unknown, not zero. The initial fetch may include up to 64 events from before you pressed Record. Compare device timestamps to Session CSV to select the recording interval. Each boot uses a random 32-bit identifier, a practical restart discriminator rather than a globally unique ID.

## Recording context

In **v0.0.4-rc.11 and later**, the dashboard requests one device metadata snapshot when a new recording starts. Recording begins immediately while context is **pending**; valid details become **captured**, and an old endpoint, timeout or invalid response becomes **unavailable** without stopping collection. In **Saved data**, expand **Recording details** and choose **Download details JSON** to save a companion to Session/GPS CSV. This browser capture/export is not available in rc.10.

The snapshot records firmware/build identity when available, the device boot ID and assembly uptime, configured GPS/IMU settings, successfully applied accelerometer bias and saved/applied mount alignment. Configured rates and baud choices are not receiver readback or delivered-rate measurements; use Diagnostics and GPS CSV for delivery evidence. The accelerometer bias is in sensor axes, before mount alignment, and is separate from the estimator's learned earth-frame bias. Null means unknown; a known zero is a value. One snapshot describes its assembly time, not constant settings or calibration throughout the trip.

The dashboard preserves available context in the existing version 1 full backup, restore copies and interrupted-recording recovery. Older recordings/backups without context remain **unknown**; reopening or restoring them must not fill historical details from today's device. Demo details are explicitly **SIM** / simulated and must not be used as hardware evidence. See the [metadata API and browser contract](RECORDING_METADATA_API.md).

Keep manual phone/OS/browser, antenna, mounting and reception notes, plus the flashed image's SHA-256. An optional source commit is not an image checksum. Details JSON complements the CSVs and restorable backup; it does not replace them.

## GPS CSV fields

| Field | Meaning |
| --- | --- |
| `browser_received_ms` | Browser wall clock at batch receipt. Several GPS events share this value. Phone clock adjustments may change it. |
| `boot_id`, `rmc_seq` | Device boot discriminator and monotonically increasing event sequence within that boot. |
| `host_parse_us` | 64-bit device monotonic microseconds when the complete RMC was parsed. Not the first UART byte, antenna measurement instant or PPS edge. |
| `receiver_utc_us_of_day` | Receiver-reported RMC UTC with fractional seconds, represented as microseconds since midnight. Null/missing time exports blank. A reported 23:59:60 is retained beyond the ordinary day's range. |
| `receiver_date_ddmmyy` | The receiver's two-digit-year date as a numeric code. Pad to six digits when displaying it. No century/Unix-epoch inference is made. |
| `utc_fractional_digits` | Number of decimal digits supplied by the receiver. Finer than microsecond fractions are truncated. More digits do not mean greater accuracy. |
| `rmc_active`, `fix_accepted` | Receiver status A and parser acceptance after coordinate/speed validation and warmup, respectively. Neither proves absolute accuracy or a lap crossing. |
| `gps_lat`, `gps_lon`, `speed_mps`, `course_rad` | Accepted RMC position, speed in metres/second and course clockwise from north. Blank for unaccepted fixes. These are not the filtered dashboard values. |
| `missing_before` | Missing parsed events since the preceding saved event in the same boot; blank at an initial/restart boundary. Does not count receiver output or UART bytes lost before parsing. |
| `boundary` | 0: continuation, 1: initial capture, 2: changed boot. |
| `synthetic` | 0: device logger; 1: synthetic website demo. Never mix demo rows with driving evidence. |

Session CSV gains `gps_boot_id`, `gps_rmc_seq`, `gps_host_parse_us`, `gps_receiver_utc_us_of_day`, `gps_receiver_date_ddmmyy`, and `gps_utc_fractional_digits` for the latest parsed RMC accompanying each dashboard sample. A repeated sequence is a repeated snapshot, not another receiver fix. The existing `device_time_ms` wraps at 2³² milliseconds; use the full-width GPS host time and boot boundaries when aligning logs.

## Analyze without publishing your route

```sh
python3 tools/python/analyze_gps_timestamps.py blackbox_gps_YOUR_SESSION.csv
```

The tool emits JSON with receiver and host interval summaries, duplicate/backward epochs, detected log gaps, invalid fixes, boot segments and closely batched arrivals. It prints no coordinates. Interval differences remove a constant clock offset but still mix delivery jitter and clock drift; they are not one-way latency measurements. Date transitions are excluded instead of guessing midnight/leap-second alignment. Preserve the original files and report failures alongside successful intervals.

For the weekend: capture a few minutes parked outdoors, your ordinary drive and another parked segment. Keep the dashboard in the foreground. Export before changing browsers/Home Screen installations. Keep precise route files private. Independent timestamped crossing evidence is still needed to assess lap accuracy.

Protocol reference: [u-blox M9 SPG 4.04 interface description, RMC section 2.7.16](https://content.u-blox.com/sites/default/files/u-blox-M9-SPG-4.04_InterfaceDescription_UBX-21022436.pdf). RMC carries UTC time/date and speed in knots; the driver converts speed to m/s. Logging preserves that unit and does not change the current host-time lap-crossing algorithm.

## Why v0.0.3 could show about 16 Hz in the dashboard

The old browser loop waited 33 ms **after** each telemetry request completed. A 30 ms request/processing time therefore gave about `1000 / (33 + 30) = 15.9` received updates per second, even with a faster firmware/GPS source. That explains a plausible UI bottleneck; it does not measure your old Wi-Fi latency or prove the receiver was at 25 Hz.

The current loop subtracts request/processing time from that interval and keeps one telemetry request in flight. Its ideal ceiling is about 30 Hz, with actual throughput limited by firmware publication, network latency, browser work and timer throttling. The UI counter uses the actual elapsed measurement window. GPS evidence is captured at the receiver parser and collected in batches independently of that UI rate.

## Drive recording summary

Without a selected track, the Drive panel shows recording readiness, monotonic duration and partial receiver-speed distance. Stop freezes its summary before network/storage waits. Gaps are excluded rather than estimated; interrupted reloads show a saved checkpoint and never resume recording. Keep GPS CSV with Recording details JSON or the full backup to explain the enrollment fences and clocks. See the [Drive recording guide](DRIVE.md).
