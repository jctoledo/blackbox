# Blackbox versus paid GPS lap timers

## Build the instrument you want to understand

Blackbox brings circuit and stage timing, GPS + inertial telemetry, reference pace feedback, and portable recordings to an ESP32-C3 instrument you can inspect and change. It is a compelling option for an enthusiast who wants to build, experiment, and own the complete software stack. A finished commercial system is a different purchase: hardware integration, mature analysis workflows, and vendor support have value.

## Concrete capabilities, with their evidence

| What you get | Current implementation | What the number or claim means |
|---|---|---|
| GPS configuration for two receivers | NEO-M9N target 25 Hz; NEO-6M target 5 Hz | Configuration targets. Check delivered valid-fix rate in Diagnostics. |
| Fast inertial input | WT901 can be configured for 200 Hz output | Sensor output rate; not proof that every sample reaches the estimator or browser. |
| Interpolated GPS timing | Direction-aware start/finish crossings; circuits and point-to-point stages | Crossing time between fresh fixes, stored in milliseconds. Physical timing error needs reference tests. |
| Position-matched reference pace | New references store position/time; direction, continuity, and ambiguity gates suppress bad matches | Comparison against a labeled reference trace, distinct from historical best. Not automatic corner coaching. |
| Phone-friendly local display | Device-hosted browser dashboard over Wi-Fi; focused Drive view and optional force Analysis | No project cloud login or subscription. Browser/device connectivity and phone power management still matter. |
| Portable session data | CSV export, full JSON backups, validated restore as copies | Browser-local persistence. Keep external backups; this is not autonomous onboard session logging. |
| An inspectable instrument | MIT-licensed Rust firmware, browser source, protocol, and regression tests | You can modify the implementation and reproduce builds. Tests establish behavior for their inputs, not a road accuracy certification. |

The NEO-M9N manufacturer specifies a standard-precision receiver with up to 25 Hz output under stated conditions. Update rate, GPS position accuracy, crossing accuracy, and browser refresh rate are different quantities. [u-blox product summary](https://content.u-blox.com/sites/default/files/NEO-M9N_ProductSummary_UBX-19027207.pdf).

## Compare by the job you need done

Vendor descriptions checked **2026-11-20**. These are feature comparisons, not a head-to-head performance test. Products, pricing, and app compatibility can change.

| Option | Documented strengths | When it is a better fit |
|---|---|---|
| **Blackbox** | DIY hardware; editable firmware and mathematics; circuit/stage timing; local dashboard; CSV and restorable backups | You want an instrument you can build and extend, and are willing to validate your installation. |
| **RaceBox Mini / Mini S** | Vendor-specified 25 Hz GPS; packaged hardware; lap and drag measurement. Mini requires a phone connection; Mini S adds standalone storage. | You want packaged GPS hardware and ready-made performance-test workflows. [Mini](https://www.racebox.pro/products/racebox-mini), [Mini S](https://www.racebox.pro/products/racebox-mini-s), [connection requirements](https://www.racebox.pro/info/faq). |
| **RaceChrono Pro** | Sector/optimal laps, track library, synchronized graphs/maps/video, predictive timing, external GPS and OBD support, multiple export formats | You want a mature phone analysis workflow or video/vehicle-data integration. Receiver choice matters. [Official feature comparison](https://racechrono.com/?lang=en). |
| **Performance Box Touch** | Vendor-specified 25 Hz logger, dedicated 4.3-inch display, performance/lap/speed modes, live slope correction for acceleration tests | You want a dedicated display and integrated performance-testing system. [Official product page](https://www.vboxautomotive.co.uk/en/products/data-loggers/performance-box-touch). |

Open interfaces, exports, and DIY integrations also exist in commercial ecosystems. Blackbox's pitch is direct ownership of this implementation, not the claim that other products prevent access to data.

## What does it cost?

The software has no project license fee or required subscription. Budget for the ESP32-C3 board, receiver and antenna, WT901, power supply, wiring, enclosure, mount, and your build time. A phone is also needed for the browser workflow. We do not publish a fixed total without a dated, reproducible bill of materials. Compare a complete installed build with the vendor's current regional price, not bare component prices with a complete commercial package.

## Measured performance and the next features

Reference-timed hardware error distributions, environmental durability, and commercial-system equivalence are not published yet. The [hardware checklist](HARDWARE_VALIDATION.md) specifies how to collect useful evidence. A normal neighborhood/highway recording can reveal gaps and coordinate problems; it cannot establish lap timing accuracy by itself.

Acceleration/drag tests with explicit rollout settings, braking-distance tests, slope estimation/correction, linked drive replay, section-loss visualization, and adaptive corner analysis are **proposals, not shipped features**. The priority is to retain the necessary evidence and validate each calculation before presenting it as a driver aid. Spatial references establish a foundation for later route-based analysis; they do not by themselves deliver those features.

Choose Blackbox for the build, the transparency, and the freedom to develop it. Choose against your actual requirements using the [demo](https://jctoledo.github.io/blackbox/demo.html), [timing guide](LAP_TIMER.md), and [engineering review](REVIEW.md).
