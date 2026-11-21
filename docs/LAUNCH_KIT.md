# Blackbox launch kit

These are editable drafts for the maintainer to publish after the bench/drive checks. Nothing here has been posted to an external community. Match community rules, disclose that you maintain Blackbox, and answer technical questions in the thread.

## Core description

Blackbox is an open-source ESP32-C3 GPS driving instrument with Rust firmware and a phone dashboard served over local Wi-Fi. Record circuits and stages, compare position-matched reference pace, inspect forces and rolling speed intervals, and export recordings you can back up and restore. Build it, understand the measurements, and help validate it.

## Short post: drivers and builders

**Title:** I’m building an open-source GPS lap timer with a phone dashboard — looking for builders and test feedback

I wanted a driving instrument I could inspect from GPS input to exported recording, so I built Blackbox around an ESP32-C3, u-blox GPS and WT901 IMU. The phone connects over local Wi-Fi; there’s no project cloud account or subscription.

It has circuit/stage timing, a landscape dashboard with Speed/G-meter/Peaks/Runs, position-matched reference pace and local recordings with CSV export and backup/restore. NEO-M9N is configured for a 25 Hz GPS target; delivered rates and real-world timing accuracy still need installation-specific evidence.

You can try the synthetic dashboard without hardware: https://jctoledo.github.io/blackbox/demo.html?v=0.0.4-rc.10

Source, build instructions and verified release installer: https://jctoledo.github.io/blackbox/

I’m especially interested in build reproducibility, antenna/mounting experience, phone storage/reconnection behavior and independently timed crossing tests. The demo is synthetic; I’m not claiming commercial-system accuracy. What would you need to see before building one?

Before posting: attach your actual hardware photo and a short real dashboard clip, and replace this note with what your first drive established. If hardware testing is incomplete, retain the development-status wording.

## Short post: embedded Rust / ESP32

**Title:** Blackbox: Rust ESP32-C3 GPS + IMU telemetry with a local browser instrument panel

Blackbox combines ESP-IDF/Rust firmware, u-blox GPS and a WT901 IMU with a device-served phone dashboard. The interesting engineering pieces are direction-aware finite-line crossings, explicit freshness checks, receiver/host timestamp evidence, position-matched reference pace and transactional browser-local recordings.

The estimator currently uses a diagonal covariance approximation. Interpolated millisecond outputs are numerical resolution; measured physical timing accuracy is still an open validation task. Both supported GPS builds and regression checks run on a dedicated self-hosted runner.

Repository: https://github.com/jctoledo/blackbox

Demo and hardware guide: https://jctoledo.github.io/blackbox/

I’d value review of coordinate-frame assumptions, timestamp handling, reproducible hardware setup and bad-data recovery. Real installation photos and drive findings will accompany the build log as they become available.

## 60–90 second video plan

| Time | Shot | Message |
| --- | --- | --- |
| 0–10 s | Real box, receiver and phone together | “I built a GPS driving instrument I can inspect and change.” |
| 10–25 s | Connect while parked, open local dashboard | Local Wi-Fi, no project cloud login |
| 25–45 s | Landscape instrument selection and timing | Show real data; label any synthetic inserts on screen |
| 45–60 s | Saved data, GPS CSV and backup | Show how evidence leaves the browser |
| 60–75 s | One concrete finding from the drive | State the observation and its limits |
| 75–90 s | Build guide and repository | Invite builders and reproducible bug reports |

Use a passenger/static camera for moving footage. Hide private route details, credentials and identifying paperwork. A decorative background clip is not a product demonstration.

[Short first-drive checklist](FIRST_DRIVE.md) · [Build journal](BUILD.md)

## First four weeks

1. **Evidence:** complete the bench checks, collect a normal drive, document the exact parts and installed wiring. Export a backup before updating or changing browser storage.
2. **Publish:** add a build photo set and one honest drive report. Use the actual installed bill of materials with purchase date, currency and exclusions before publishing a cost comparison.
3. **Share:** publish one audience-specific post at a time. Suggested places to evaluate are Hackaday.io, ESP32/embedded Rust communities and relevant track-day/autocross forums. Read their current rules first; avoid copy-pasted blasts.
4. **Learn:** record builder successes, failed setup steps, questions and relevant search queries. Improve the guide that answers the repeated question before adding another feature.

## Measurement and account setup

Use Search Console to verify the exact Pages URL-prefix property and submit `https://jctoledo.github.io/blackbox/sitemap.xml`. Record a baseline, then compare impressions, clicks, queries, landing pages and indexing status over time. Configure Bing Webmaster Tools separately if wanted. Account verification requires the maintainer's account; no ownership verification or analytics service has been configured by this document.

Repository work can improve page usefulness and crawlability; it cannot guarantee indexing, rankings, AI citations or traffic. `llms.txt` is a reading index. See [the primary-source discovery research](research/DISCOVERY.md).

A simple private weekly log is enough to start: date, search impressions/clicks, demo interest, new builder reports, first successful recordings and the source of each useful referral. GitHub stars are interest, not proof that someone built or used the instrument. Avoid publishing location-containing user data as a marketing metric.
