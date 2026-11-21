---
name: Bug Report
about: Report a bug to help us improve Blackbox
title: '[BUG] '
labels: bug
assignees: ''
---

## Description
A clear and concise description of what the bug is.

## Hardware Setup
- **ESP32 Variant**: (exact ESP32-C3 board/revision; S3 is not supported by current images)
- **IMU Model**: (WT901 variant, UART baud and packet settings)
- **GPS Module**: (NEO-M9N or NEO-6M, antenna and delivered GPS rate)
- **Other Sensors**: (if applicable)

## Software Version
- **Firmware Version/Commit**: (release tag, image variant and checksum, or commit hash)
- **Build Date**: (if known)
- **Modified Code**: Yes/No (describe modifications if yes)

## Phone and saved data
- Phone, OS and browser version; local dashboard or website demo:
- Was the dashboard foregrounded, suspended or reconnecting?
- Recording active? Were Session CSV, GPS CSV and a backup exported?
- Timing type and state (circuit/stage, armed/waiting/running):

Keep precise routes, home coordinates, credentials and unredacted backups out of public issues. Describe the failure and share a minimal redacted example if possible.

## Steps to Reproduce
1. Power on ESP32...
2. Wait for GPS lock...
3. See error...

## Expected Behavior
What you expected to happen.

## Actual Behavior
What actually happened.

## Serial Output / Logs
```
Paste relevant serial monitor output or logs here
```

## LED Status
What LED patterns did you observe? (if applicable)

## Additional Context
- Did this ever work before?
- When did the problem start?
- Any recent changes to hardware/software?
- Environmental conditions (temperature, vibration, etc.)

## Possible Solution
If you have ideas on what might be causing this or how to fix it.
