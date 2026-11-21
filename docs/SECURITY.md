# Security and privacy

Blackbox is an observational telemetry project. Current firmware does not actuate brakes, steering or throttle. It does expose unauthenticated local HTTP endpoints for telemetry and timing configuration, plus configurable UDP/MQTT outputs. Anyone with network access may be able to read data or change timing configuration. Do not expose these services to the public internet.

## Data and storage

GPS coordinates, timestamps and driving traces can reveal location history. Recordings, saved tracks and references live in the browser's IndexedDB, scoped to the site origin/browser profile. CSV and JSON backups are unencrypted files. A CRC detects accidental corruption, not malicious modification or authenticity. Share only data you intend to disclose, and keep a backup outside browser storage.

Device HTTP and telemetry transports do not provide application-layer encryption or user authentication. Use a trusted dedicated network; Wi-Fi credentials are build/configuration data and can be recovered from firmware. Check `sensors/blackbox/src/config.rs` and build settings before publishing a binary. Secure boot, flash encryption and signed OTA updates are not enabled by this project as a blanket guarantee.

The website's USB installer verifies firmware hashes against release metadata to detect mismatched downloads. This does not provide an independent signature or protection if both firmware and metadata are compromised.

## Reporting

Report sensitive findings through the repository's GitHub Security → Report a vulnerability flow when available. Do not put credentials or private recordings into public issues. Include affected commit/release, a minimal reproduction and impact. This volunteer project does not promise a response SLA or a supported stable-release series; fixes target current source and subsequent releases.

## Contributor checks

Validate numeric inputs and bounded payloads before they reach estimation, persistence or configuration. Preserve transactional writes and explicit errors. Never add credentials to source, logs or fixtures. Treat imported backups as untrusted. Keep lat's persistent runner restricted to reviewed maintainer-dispatched code and trusted main/tag workflows; do not run arbitrary fork PRs automatically.

Board certification, installation compliance and suitability for automotive safety functions are not established by software tests. The proposed S3 schematic is not the released C3 target. See [hardware validation](HARDWARE_VALIDATION.md).
