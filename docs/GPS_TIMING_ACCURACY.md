# How accurate is a DIY GPS lap timer?

A GPS lap timer can display thousandths of a second without measuring crossings to that accuracy. Blackbox stores interpolated crossing times in milliseconds; its real-world timing error has not yet been measured against an independent reference. This guide explains what determines that error and what evidence would make an accuracy claim useful.

[Try the instrument](https://jctoledo.github.io/blackbox/demo.html?v=0.0.4-rc.13) · [Build a GPS lap timer](DIY_GPS_LAP_TIMER.md) · [Detailed maths and rate budgets](MATHS_AND_RATES.md)

## What happens between two GPS fixes?

Blackbox represents the start/finish as a finite line with a direction. A valid approach must cross that segment in the recorded direction, using fresh GPS evidence. A nearby position alone does not start a lap. This matters when a road runs near the line, when GPS wanders while parked, or when you pass outside a line endpoint.

For a simple straight crossing, let signed perpendicular distance to the line change from −0.6 m to +0.2 m between observations 40 ms apart. Linear interpolation puts the crossing at `0.6 / (0.6 + 0.2) = 0.75` of the interval: 30 ms after the first observation. Blackbox's circuit/stage crossing clock uses device host receive times; receiver UTC is also retained separately for analysis. PPS/receiver-epoch synchronization of circuit timing is not implemented.

Interpolation removes the need to round every crossing to a sample boundary. It cannot correct a displaced GPS position, uncertain receive latency, a wrongly placed line or a curved path that differs from the assumed segment. The [engineering explainer](ENGINEERING.md) describes those implementation boundaries.

## What does 25 Hz buy you?

At 60 mph, the car moves approximately 26.82 m/s:

| Delivered GPS rate | Time between observations | Travel at 60 mph |
| --- | --- | --- |
| 5 Hz | 200 ms | 5.36 m |
| 10 Hz | 100 ms | 2.68 m |
| 25 Hz | 40 ms | 1.07 m |

These are arithmetic examples at constant speed, not observed receiver performance. NEO-M9N firmware targets 25 Hz; the NEO-6M variant targets 5 Hz. Diagnostics and exported receiver epochs reveal the delivered cadence. Manufacturer capability and your assembled installation are different pieces of evidence. [u-blox NEO-M9N product summary](https://content.u-blox.com/sites/default/files/NEO-M9N_ProductSummary_UBX-19027207.pdf).

A higher rate gives more frequent observations of the path. It does not, by itself, establish centimetre positioning or a particular lap-time error. The browser refresh counter is another rate again: it measures delivery to the display, not new GPS fixes.

## Position error becomes time error

A useful first-order estimate is `crossing time error ≈ position error normal to line / velocity normal to line`.

For illustration, 1 m of normal position error at 20 m/s normal velocity corresponds to 0.05 s. At 5 m/s it corresponds to 0.2 s. These examples are sensitivities, not Blackbox accuracy bounds: we have not established that the position error is 1 m, nor that it is independent between crossings.

A shallow crossing angle reduces normal velocity. A start line should span the intended route, and its direction should agree with the approach. Satellite obstruction, antenna placement and multipath can change position quality. Bad line geometry can be fixed in setup; a missing GPS crossing should remain a missing crossing rather than becoming a confident invented lap.

## Why a clean-looking trace is not validation

Smoothing may make a map or G-meter easier to read while hiding disagreement with physical motion. Blackbox's synthetic demo is designed to exercise UI and evidence paths. Its repeatable lap and interval values establish behavior for those inputs only.

Likewise, the estimator's diagonal covariance is an internal model quantity, not a calibrated probability that a real car lies within a displayed distance. Blackbox does not implement RTK or full covariance propagation. Compare commercial systems using their documented capabilities and the same independent reference, rather than comparing decimal places. [Build or buy comparison](COMPARISON.md).

## Two different timing jobs

Circuit and stage timing estimates a position crossing. Rolling 45–70 mph, 30–60 mph, 50–100 km/h and 80–120 km/h tests estimate speed-threshold crossings from receiver UTC. Those rolling detectors require recording, retained lead-in evidence, sustained thresholds and continuity. They reject gaps and other invalid sequences; they do not correct slope or implement standing-start 0–60. [Rolling interval rules](RUNS.md).

## The measurement report we intend to publish

**Status: awaiting real hardware evidence.** No measured error values are available in this section yet.

A useful report will identify the exact board, receiver, antenna, mounting, firmware checksum, phone/browser and conditions. It will describe the independent timing reference and synchronization uncertainty, retain every attempted crossing, and distinguish missed detections from timing errors on detected crossings.

| Evidence to add | What it should establish |
| --- | --- |
| Antenna and line-layout photos | Reproducible installation and crossing geometry |
| Receiver-epoch cadence report | Delivered rate, repeated epochs and missing samples |
| Controlled intended/wrong-direction crossings | Detection counts and failure cases |
| Reference-timed crossings at several speeds | Signed bias, median, 95th percentile and maximum absolute error |
| Parked near-line observations | False triggers and stationary drift behavior |
| Private raw logs and redacted public examples | Reproducibility without exposing a home route |

An ordinary neighborhood/highway drive is an excellent first data-quality test. It can reveal interruptions, mounting-sign errors and receiver behavior. It cannot establish absolute lap accuracy without an adequate independent reference. Start with the [drive checklist](HARDWARE_VALIDATION.md) and [GPS logging workflow](M9N_GPS_LOGGER.md).
