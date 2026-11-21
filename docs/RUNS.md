# Rolling speed-interval estimates

The landscape **Runs** instrument reports the last and best 45–70 mph intervals from GPS evidence collected while recording. Tap the instrument name to reveal the picker and select Runs, or swipe horizontally. **Record session** uses the ordinary recording flow; no performance maneuver is required for an ordinary neighborhood or highway data-collection trip.

This first implementation is an uncorrected GPS estimate. It is not a validated drag timer, and it does not yet implement standing-start 0–60 mph. Defining launch from noisy near-zero speed needs separate validation; a nonzero threshold must not silently be sold as a true zero-speed start. There is no distance rollout, slope correction, wind correction or vehicle-efficiency inference.

## Timing and rejection rules

- Use receiver UTC epochs, never browser receipt times. Interpolate the start and finish between adjacent speed observations: `crossing = t0 + (threshold − speed0)/(speed1 − speed0) × (t1 − t0)`.
- Arm below 44 mph, then require a rising crossing of 45 mph. Starting a recording above the lower threshold does not invent the missing start.
- Require accepted active fixes, finite speed/course and UTC with at least two fractional digits. Consecutive receiver intervals must be 20–300 ms, with consecutive sequence numbers in the same boot and date.
- Reject a capture gap, backwards/duplicate UTC, restart, date boundary, speed change over 15 m/s², heading deviation over 15° from the start, or an attempt lasting over 120 seconds. Identical sequence duplicates add no evidence.
- Confirm the 70 mph crossing with at least 80 ms of subsequent evidence no more than 0.15 m/s below the threshold. This rejects a brief threshold spike. The reported finish time remains the interpolated crossing, not the confirmation time.
- Returning below 44 mph aborts an unfinished attempt and permits rearming. A date change is conservatively rejected rather than guessing midnight/leap-second behaviour.

These gates reject some bad data, not every plausible GNSS error. Heading is GPS course, not steering angle. Two displayed decimals describe numerical resolution, not measured timing accuracy.

## Evidence, saving and export

The browser polls the bounded GPS log while recording. Its initial batch can include up to 64 buffered receiver events from before the record button was pressed. Runs analysis includes that explicitly retained pre-roll. Interrupted background collection can lose evidence; sequence gaps reject affected attempts.

Last/best display values reset with a new recording or page reload. **Saved data → Runs CSV** reconstructs accepted and rejected attempts from the stored GPS evidence, including source sequence endpoints, boot/date, receiver crossing times and the synthetic marker. Export GPS CSV alongside it to retain all underlying observations. Backup/restore already preserves those GPS rows; there is no separate result database that can drift away from its evidence. More than 1000 attempts require exporting the full GPS CSV instead of silently truncating the derived export.

A run may never qualify: a short recording, no lower-threshold crossing, missing receiver time, an interrupted stream or a turn can all explain that. An empty result is not zero seconds.

Changing dashboard speed units does not redefine this interval: it remains explicitly **45–70 mph**. Additional 30–60 mph and 50–100 km/h detectors and labelled precision/recall evaluation are tracked in [issue #79](https://github.com/jctoledo/blackbox/issues/79).

## Try the synthetic scenario

The demo starts with a clearly labelled saved rolling example: two accepted intervals and a third rejected by a deliberate six-event GPS gap. Its Last/Best values come from the same GPS evidence available in Saved data, not a separately invented result. Starting your own recording resets those example results.

In the website demo, open **Menu → Simulate rolling interval**, then record a session and select **Runs** in landscape. This explicitly changes the synthetic data source to a straight-line acceleration/deceleration scenario and disarms circuit timing. The 2 m/s² acceleration produces an interpolated 45–70 interval of 5.588 s (displayed as 5.59 s). The menu can return to the textured circuit scenario. **SIM** remains visible; these are not measured driving results.

## Remaining acceptance

Replay known recordings and compare repeated controlled runs against independent timing before making an accuracy claim. Check rate, speed noise, course stability, slope/wind context, phone background interruption and saved-data recovery. Standing starts, braking-distance tests and custom intervals remain separate work in issue #62.
