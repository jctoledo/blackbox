# How Blackbox works

Blackbox splits the job between a sensor controller and a local browser instrument. The ESP32-C3 reads GPS/IMU data, estimates motion and detects directional timing-line crossings. The phone displays telemetry, compares a spatial reference, and saves recordings in IndexedDB.

## Timing and reference pace

Circuits use a shared start/finish line; stages use separate lines. The timer intersects consecutive fresh GPS positions with the finite line and interpolates the crossing time. Direction checks, minimum duration and continuity gates reject several common false triggers. The receiver's measured error and host receive-time jitter remain part of the result.

The phone compares current route position with a saved position/time trace. Matching requires fresh GPS, direction agreement and sequential progress, and rejects competing route branches. The result is reference pace feedback. It does not identify corners or measure apex precision. Read [the timing guide](LAP_TIMER.md).

## A useful instrument in either orientation

Portrait keeps setup and saved data close at hand. Landscape provides a speed-and-timing cluster automatically; Dashboard restores controls. The same production HTML drives the device and synthetic website demo. Bundled interface/instrument fonts work without internet access. Optional Analysis exposes forces when parked.

## Data ownership and recovery

Recordings, tracks, references and history stay in the browser. Full JSON backups can be validated and restored as new copies; CSV exports support analysis. Saves are transactional within each database, but browser suspension, quota and eviction still matter. Keep a copy outside browser storage. See [backup and recovery](FAQ.md#how-do-i-back-up-and-restore-everything).

## Mathematics and verification

The estimator has seven state values and diagonal covariance. Its diagnostic uncertainty is not a calibrated physical accuracy bound. The force-display pipeline combines filtered inertial data with a GPS speed derivative and a centripetal model. See [equations, units and assumptions](SENSOR_FUSION.md).

Host regressions exercise physics, crossings, parsers, protocol compatibility, persistence failures and UI state. Both GPS firmware variants are built on lat. The [hardware acceptance checklist](HARDWARE_VALIDATION.md) covers real USB installation, axes, missed/false crossings, timing error, phone behavior and endurance. Configuration rates and display decimals are not substitutes for these measurements.

[Compare capabilities and costs](COMPARISON.md), [try the synthetic demo](https://jctoledo.github.io/blackbox/demo.html), or inspect the implementation on GitHub. Maintainer defect history remains in the repository; this guide describes the current product.
