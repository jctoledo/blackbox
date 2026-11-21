# Blackbox terminology

- **Track:** saved circuit or stage geometry and timing lines. It is separate from a raw recording.
- **Circuit:** repeats one directed start/finish line. **Stage:** uses distinct directed start and finish lines.
- **Recording session:** raw telemetry/GPS chunks in the browser recording database. REC controls this capture.
- **Lap-history session:** a grouping of completed results for an activated track. It is not the raw recording session; code must preserve each result's original ownership across awaits.
- **Reference:** the labelled position/time trace used for pace comparison. Its time may differ from the historical best lap.
- **GPS fix:** a receiver observation, including validity. Freshness and continuity determine whether it can support timing.
- **Receiver UTC:** time carried by the receiver message. **Host completion time:** controller clock when parsing completes. **Browser receipt time:** phone clock on delivery. These clocks are not interchangeable; see the timing/logging guides.
- **Configured rate:** requested sensor behavior. **Delivered GPS rate:** observations actually received. **Dashboard rate:** updates reaching the display.
- **Rolling interval:** named speed-threshold test evaluated from recorded receiver evidence. It is distinct from a standing-start/rollout test.
- **Force estimate / Peaks:** displayed motion estimates; Peaks summarizes qualified half-second windows. Neither is a calibrated force-accuracy claim.
- **Controller target:** current release firmware supports ESP32-C3; the proposed S3 schematic is a separate hardware port.

Frame definitions and equations live in `docs/SENSOR_FUSION.md` and `docs/MATHS_AND_RATES.md`; this glossary names concepts rather than repeating their implementation.
