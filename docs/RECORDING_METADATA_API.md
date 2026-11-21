# Recording metadata API

`GET /api/recording-metadata` returns JSON with `Cache-Control: no-store`. Before initialization (or if the snapshot cannot be supplied), it returns HTTP 503 with `{"status":"unavailable"}`. HTTP 200 supplies the typed version 1 snapshot below. This endpoint does not change binary telemetry or automatically add metadata to browser recordings; browser capture is a separate integration.

```json
{
  "version": 1,
  "simulated": false,
  "boot_id": 123,
  "captured_uptime_ms": 4294976295,
  "build": {
    "app_version": "v0.0.4-rc.10",
    "idf_version": "v5.1.4",
    "source_sha": null,
    "target": "esp32c3"
  },
  "gps": {
    "configured_model": "NEO-M9N",
    "configured_rate_hz": 25,
    "configured_uart_baud": 115200,
    "configuration_verified": false
  },
  "imu": {
    "model": "WT901",
    "transport": "uart",
    "configured_uart_baud": null,
    "baud_selection": "pending",
    "configured_output_rate_hz": null
  },
  "calibration": {
    "accelerometer_bias_mps2": null,
    "mount_alignment": null
  }
}
```

The example is illustrative. `boot_id` reuses the GPS log's random boot identity. `captured_uptime_ms` is the full monotonic controller uptime in milliseconds when assembled, without the telemetry loop's 32-bit wrap. The first snapshot appears after HTTP startup, before IMU probing; subsequent snapshots publish the selected baud and successfully applied startup bias. During normal acquisition a refresh occurs at most once per second, including saved alignment and successful recalibration changes. Blocking startup/recalibration work can delay refresh; the timestamp continues to describe assembly time, not request time. A snapshot does not assert that settings were constant throughout a drive.

Application and IDF versions come from the real ESP-IDF application descriptor. Each field reads at most 32 bytes and stops at its first NUL; empty or invalid UTF-8 fields are null. `source_sha` is a compile-time `GITHUB_SHA` only when exactly 40 ASCII hexadecimal characters are present; otherwise it is null. No arbitrary environment or configuration map is returned.

GPS values describe the effective configured model, requested rate and controller UART baud (normally NEO-6M at 5 Hz/9600 baud or NEO-M9N at 25 Hz/115200 baud). They are neither receiver readback nor delivered-rate measurements. `configuration_verified` remains false: ACK/readback verification is not implemented.

IMU baud selection is `pending` with null baud until probing completes; `header_probe` means the controller selected a baud after seeing enough WT901 header bytes; `fallback` means the probe failed and the controller requested 9600 baud (the baud is null if the controller UART change failed). The baud is also null if a probe-stage UART change failed, even if header bytes were observed afterward. A header probe does not verify complete packets or sensor configuration. Output rate remains unknown/null.

`accelerometer_bias_mps2` is either null or `{ "x": 0.0, "y": 0.0, "z": 0.0 }` with the successfully applied bias in metres per second squared. Zero is a known value. Failed recalibration retains the previously applied bias. `mount_alignment` is either null or `{ "is_calibrated": false, "yaw_offset_rad": 0.0 }` for the saved/applied mounting state; candidates never appear here. The yaw offset follows [the calibration frame convention](SENSOR_FUSION.md). Storage unavailable means initial alignment is unknown/null; an available store with no valid record uses the firmware's uncalibrated zero default. Nonfinite calibration values are unavailable/null.

No Wi-Fi credentials, tokens, broker/server settings, or additional hardware identity are exposed. Hardware detection, receiver readback and physical accuracy still require bench/drive evidence.
