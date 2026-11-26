# Performance run estimates

The landscape **Runs** instrument reports the last and best performance estimates from GPS evidence collected while recording. The interval button opens ordinary page buttons with the same readable type as the closed control. Choose a family using the current **Menu → Speed units** preference:

| Family | mph preference | km/h preference |
| --- | --- | --- |
| Lower speed | 30–60 mph | 50–100 km/h |
| Overtaking | 45–70 mph | 80–120 km/h |
| Short rolling | 50–60 mph | 80–100 km/h |
| Higher rolling | 60–80 mph | 100–130 km/h |
| Standing start · experimental | 0–60 mph | 0–100 km/h |
| Standing distance · experimental | 1/4 mile | 1/4 mile |

These are purpose counterparts, not exact unit conversions. Changing units preserves the selected family and shows that preset's own results; it never converts or relabels a completed time. All eleven independent detectors (eight rolling, two standing speed tests and one shared quarter-mile) keep collecting qualified evidence during recording, regardless of the displayed instrument or units. Selecting a family does not reset recording, timing or detector history. The opened chooser scrolls when space is short; Escape returns focus to its interval button, and choosing an option, clicking outside or leaving Runs closes it.

Tap the instrument name to reveal the picker and select Runs, or swipe horizontally. **Record session** uses the ordinary recording flow; no performance maneuver is required for an ordinary neighborhood or highway data-collection trip.

All results are uncorrected GPS estimates. Standing starts and the quarter-mile are explicitly **experimental**, with a visible no-rollout label. They do not establish physical launch accuracy or parity with a drag-strip timeslip. There is no distance rollout, slope correction, wind correction or vehicle-efficiency inference. The [primary-source research and selection plan](research/PERFORMANCE_RUNS.md) explains these conventions and the compact six-option menu.

## Receiver-log health

Runs shows an amber status when its GPS timestamp log is unavailable or no new receiver events have arrived for more than 2.5 seconds. Ordinary dashboard telemetry can remain live during a log failure; it is a separate input. The display distinguishes waiting for initial timestamps, waiting for fresh timestamps and a failed log request. Last and best accepted results remain visible.

This is a display-readiness interval, not a new timing gate. A temporary HTTP failure does not by itself invalidate a pending interval: the retained receiver window may still provide complete evidence after recovery. Existing sequence, epoch, quality and continuity rules decide the result. Late responses from a previous recording cannot replace the current recording's status. The demo menu's **Pause GPS timestamp log** control demonstrates this condition without stopping ordinary synthetic telemetry.

## Rolling timing and rejection rules

- Use receiver UTC epochs, never browser receipt times. Interpolate the start and finish between adjacent speed observations: `crossing = t0 + (threshold − speed0)/(speed1 − speed0) × (t1 − t0)`.
- Require at least 200 ms of continuous valid evidence below the selected start speed minus 0.44704 m/s (1 mph), then a rising start crossing sustained for 80 ms within 0.15 m/s of the threshold. Starting a recording above the lower threshold does not invent the missing start.
- Require accepted active fixes, finite speed/course and UTC with at least two fractional digits. Consecutive receiver intervals must be 20–300 ms, with consecutive sequence numbers in the same boot and date.
- Reject a capture gap, backwards/duplicate UTC, restart, date boundary, speed change over 15 m/s², heading deviation over 15° from the start, or an attempt lasting over 120 seconds. Identical sequence duplicates add no evidence.
- Confirm the selected finish crossing with at least 80 ms of subsequent evidence no more than 0.15 m/s below the threshold. This rejects a brief threshold spike. The reported finish time remains the interpolated crossing, not the confirmation time.
- Returning below the start speed minus 0.44704 m/s aborts an unfinished attempt and permits rearming. A date change is conservatively rejected rather than guessing midnight/leap-second behaviour.

These gates reject some bad data, not every plausible GNSS error. Heading is GPS course, not steering angle. Two displayed decimals describe numerical resolution, not measured timing accuracy.

## Experimental standing starts and quarter-mile

The two standing speed tests and quarter-mile use a separate launch detector. The thresholds below are Blackbox engineering policy, not a vendor standard or a measured accuracy guarantee.

1. Require at least one second of consecutive accepted receiver fixes at **0.3 m/s or below** before arming. Starting a recording while moving cannot invent a launch. GPS course may be absent while stationary.
2. Anchor the estimated start to the **last stationary fix**, and confirm movement at **0.5 m/s or above**. Confirm within 300 ms, allowing at most two consecutive receiver intervals on slower receivers, with a 600 ms absolute cap. Reject unresolved creeping launches. Export the actual confirmation bracket and anchor speed.
3. Establish GPS course after speed reaches 2 m/s. Thereafter require direction to stay within 15° of that heading. Apply the rolling detector's fix, sequence, date/boot, 20–300 ms interval, 15 m/s² plausibility and 120-second timeout gates. Returning to rest aborts an unfinished attempt. A new stationary lead-in is required after completion or rejection.
4. For 0–60 mph / 0–100 km/h, interpolate the exact finish-speed crossing and require 80 ms of sustained finish evidence, as for rolling tests.
5. For the quarter-mile, integrate consecutive **receiver speed in m/s**, using trapezoidal integration, from the same launch anchor. Finish at exactly **402.336 m**, retaining the literal quarter-mile distance in both unit preferences. Solve the partially traversed final segment under the same locally linear velocity assumption; do not round up to a whole receiver interval. Confirm with 80 ms of subsequent valid evidence.

`stationary-fix-v1` is a GPS launch convention, not detection of the exact physical instant of movement. A near-zero threshold, receiver filtering, speed noise, and finite sample spacing can shift the anchor in either direction. The exported launch bracket describes the observed candidate/confirmation interval; it does **not** bound physical launch error. No first-foot time is subtracted. Do not compare it directly with a published rollout-adjusted 0–60 without aligning conventions.

The quarter-mile export includes instantaneous **finish speed**, not drag-strip trap speed. NHRA's trap convention averages speed across the final 66 feet; that is a different quantity. No trap-speed display, slope validation badge, eighth-mile option, or custom interval editor is added in this version.

## Evidence, saving and export

The browser polls the bounded GPS log while recording. Its initial batch can include up to 64 buffered receiver events from before the record button was pressed. Runs analysis includes that explicitly retained pre-roll. Interrupted background collection can lose evidence; sequence gaps reject affected attempts.

Last/best display values reset with a new recording or page reload. **Saved data → Runs CSV** reconstructs accepted and rejected attempts from the stored GPS evidence, including source sequence endpoints, boot/date, receiver crossing times and the synthetic marker. Standing exports additionally retain start convention, zero rollout, exact target distance, launch bracket, anchor speed and finish speed. Accepted and rejected attempts are reconstructed by the same code used live. Export GPS CSV alongside it to retain all underlying observations. Backup/restore already preserves those GPS rows; there is no separate result database that can drift away from its evidence. More than 1000 attempts require exporting the full GPS CSV instead of silently truncating the derived export.

A run may never qualify: a short recording, no lower-threshold crossing, missing receiver time, an interrupted stream or a turn can all explain that. An empty result is not zero seconds.

CSV interval names retain their literal fixed thresholds for all eleven detectors, independently of the currently displayed units. Labelled real-drive precision/recall evaluation is tracked in [issue #79](https://github.com/jctoledo/blackbox/issues/79).

## Try the synthetic scenario

The demo includes labelled saved circuit, rolling, and standing-start examples. The rolling example covers the six rolling presets within its speed range. The separate standing example reaches all eight rolling presets and contains two completed launches plus one rejected by a deliberate GPS gap for each standing test and the quarter-mile. Its Last/Best values come from the same GPS evidence available in Saved data, not a separately invented result. Starting your own recording resets those example results.

In the website demo, open **Menu → Simulate rolling interval**, then record a session and select **Runs** in landscape. This explicitly changes the synthetic data source to a straight-line acceleration/deceleration scenario and disarms circuit timing. The 2 m/s² acceleration produces an interpolated 45–70 interval of 5.588 s (displayed as 5.59 s); 30–60 mph takes 6.7056 s, 50–100 km/h takes 6.9444 s, and 80–120 km/h takes 5.5556 s. Use **Menu → Simulate standing start** for stationary lead-ins, acceleration through the speed targets and a full quarter-mile. The menu can return to the textured circuit scenario. **SIM** remains visible; these are not measured driving results.

## Remaining acceptance

Replay known recordings and compare repeated controlled runs against independent timing before making an accuracy claim. Check rate, speed noise, course stability, slope/wind context, phone background interruption and saved-data recovery. Physical launch validation, braking-distance tests and custom intervals remain separate work; rollout and the eighth-mile are deferred to keep the chooser compact.

Synthetic regressions cover exact crossing times, incomplete lead-in, threshold spikes, gaps, restarts and duplicate events. They do not establish real-drive precision, recall or F1. Hold out entire trips and use independently labelled positives and negatives before tuning or claiming detection quality.

Check opened and closed chooser readability and touch/keyboard focus on physical Android and iPhone browsers, including short landscape cards. Desktop device emulation does not establish physical-phone behaviour.

![Synthetic Runs and Drive preview with receiver timestamp logging unavailable and completed results retained](assets/gps-log-health-v1.jpg)

This synthetic interruption leaves ordinary telemetry live. Runs retains completed estimates; Drive keeps the qualified distance partial rather than guessing across the missing log.
