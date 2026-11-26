# Compact GPS performance runs: timing contract and staged plan

Research date: 2026-11-25. Primary-source reading and repository documentation inspection only; no code changes, builds, tests, CI dispatches or physical measurements. These are recommendations, not implemented capabilities. Implementation follow-up: the six-choice plan is implemented with the experimental `stationary-fix-v1` launch convention; exact thresholds, the cadence-aware confirmation window and remaining physical acceptance are documented in [Runs](../RUNS.md). Existing behavior is documented in [Runs](../RUNS.md); accuracy claims remain subject to [hardware evidence](../GPS_TIMING_ACCURACY.md).

## Recommended preset set

Keep one small chooser with six entries per speed preference. This is a product selection, not a claim that all six are standardized or equally validated:

| Purpose | mph preference | km/h preference | Delivery stage |
| --- | --- | --- | --- |
| Broad rolling interval | 30–60 mph | 50–100 km/h | Existing detector |
| Overtaking | 45–70 mph | 80–120 km/h | Existing detector |
| Short rolling interval | 50–60 mph | 80–100 km/h | Extend existing rolling detector |
| Higher rolling interval | 60–80 mph | 100–130 km/h | Extend existing rolling detector |
| Standing acceleration | 0–60 mph | 0–100 km/h | Separate launch validation |
| Standing distance | 1/4 mile | 1/4 mile | Separate launch/distance validation |

These speed pairs serve similar purposes; they are not conversions. The quarter-mile remains exactly 402.336 m in both preferences; do not relabel it 400 m. Keep independent preset IDs and thresholds in SI units. A unit change changes the selected counterpart and its own results, never the name of an already measured result. The 100–130 km/h upper interval is a compact-menu choice, not a claimed industry standard.

Preserve the existing lower/overtaking choices and defer the 1/8 mile (exactly 201.168 m) to avoid menu growth. Its launch/distance requirements are the same as the quarter-mile; it is a straightforward later preset once those are accepted. Short intervals such as 50–60 mph increase the relative effect of speed noise; evaluate them separately instead of inheriting accuracy claims from longer runs.

Commercial precedent supports both speed and distance tests, but does not dictate this particular list: Racelogic supports configurable start/end speed or start-speed/end-distance tests, and its legacy Sport app includes rolling intervals and custom ranges. [VBOX Touch acceleration settings](https://en.racelogic.support/automotive/data-loggers/vbtouch/ug/user-interface/accel-settings/), [VBOX Sport app manual](https://racelogic.support/automotive/discontinued-legacy/data-loggers/vbox-sport-v1-v2/ug/07-performance-test-ios-application/).

## What the sources establish

**Interpolation:** Racelogic estimates threshold crossing between adjacent velocity samples using a locally linear acceleration model. Its explanation concerns its normal 20–100 Hz samples; that is not evidence that Blackbox's 5 Hz option has the same error. [Racelogic interpolation](https://support-centre.racelogic.co.uk/automotive/kb/how-does-interpolation-work/).

**Near-zero start:** Racelogic's zero-velocity article uses 0.5 km/h to suppress stationary false starts and reports a small typical effect for its equipment. Its performance-meter rollout article instead describes a first sample above 0.8 km/h. These are product/context-specific conventions, not a universal physical zero detector. Neither quoted threshold nor the vendor's error statement validates a Blackbox launch algorithm. [Racelogic zero velocity](https://racelogic.support/automotive/kb/what-does-the-vbox-use-as-zero-velocity/), [Racelogic rollout](https://support-centre.racelogic.co.uk/motorsport/kb/one-foot-rollout/).

**Rollout:** Car and Driver adopted one-foot rollout in 2019 and publishes its duration so readers can recover a movement-start result. It previously approximated rollout with a 3 mph start, an approximation it abandoned as launches changed. Do not implement a fixed time subtraction or a 3 mph start and call that one-foot rollout. [Car and Driver's procedure change](https://www.caranddriver.com/features/a30085446/acceleration-testing-procedure-change-rollout/).

**Distance rollout endpoints need explicit definition:** Racelogic's VBOX Tools recipe sets the finish to 1321 ft and adds a line at 1 ft, obtaining the time over the intervening 1320 ft. That differs from subtracting the first-foot time while retaining a finish 1320 ft from movement origin. Store distance origin, timing origin and finish distance explicitly; a boolean named rollout is insufficient to define both. Recommend no rollout initially; add a labeled compatibility mode only after choosing and testing its exact contract. [Racelogic rollout recipe](https://support-centre.racelogic.co.uk/motorsport/kb/one-foot-rollout/).

**Distance source:** Racelogic measures distance by integrating Doppler-derived velocity rather than summing noisy position displacements. Its centimetre brake-stop claim belongs to its own installation and products. It does not establish Blackbox distance accuracy. [Racelogic distance measurement](https://racelogic.support/automotive/kb/how-is-distance-measured-so-accurately-given-gps-is-only-accurate-to-a-few-m/).

**Finish speed and trap speed differ:** NHRA defines the speed trap as the final 66 ft before the finish. Its timing description uses a clock started at that trap's entrance and stopped at the finish, so the result is distance divided by elapsed time. Instantaneous GPS speed at the distance crossing must be labeled finish speed, not silently compared with a drag-strip timeslip. [NHRA glossary](https://www.nhra.com/nhra-101/nhra-glossary), [NHRA timing-system description, p.301](https://media.nhra.com/wp-content/uploads/2022/02/CWDRS22-MediaGuide-Digital.pdf#page=301).

**Noise/filtering:** Racelogic warns that multipath or antenna-related velocity spikes can finish a speed test early. It also warns that excessive filtering can delay detected movement and shorten standing-start times. Keep display smoothing separate from timing evidence and do not assume an attractive trace validates launch time. [Velocity noise](https://support-centre.racelogic.co.uk/automotive/kb/what-are-the-effects-of-velocity-noise-on-test-measurements/), [Filtering effects](https://support-centre.racelogic.co.uk/automotive/kb/how-does-filtering-affect-results/).

**Conditions and corrections:** Car and Driver averages best runs in opposing directions and applies atmospheric correction; a raw Blackbox best attempt is a different measurement. Racelogic exposes slope correction as a labeled option. Its road-gradient note warns of noisy velocity-derived slope. Do not infer validated road grade from body pitch or add a guessed correction. [Car and Driver testing](https://www.caranddriver.com/features/a32018270/how-we-test-cars/), [VBOX slope setting](https://en.racelogic.support/automotive/data-loggers/vbtouch/ug/user-interface/accel-settings/), [Racelogic gradient](https://support-centre.racelogic.co.uk/automotive/kb/calculating-road-gradient/).

**Dragy evidence limit:** Its official site identifies quarter-mile testing, receiver rate and product hardware, but the inspected public page does not specify the complete launch, interpolation, trap-speed or run-validity algorithm. No authoritative public slope cutoff was established in this reading. Do not turn forum descriptions of a “1%” rule into Blackbox requirements or claim Dragy validation parity. [Dragy official site](https://www.godragy.com/).

## Proposed implementation contract

### Shared evidence and rolling tests

Use the existing retained receiver GPS rows and receiver UTC epochs, not browser arrival time, rendered speed, or the circuit timer's host clock. Preserve all existing acceptance gates: active valid fixes, timestamp precision, boot/date/sequence continuity, 20–300 ms adjacent epochs, speed plausibility, sustained lead-in/start/finish evidence, turn rejection and timeout. UI selection must not reset or decide evidence collection. Saved results remain derived from saved observations. [Existing timing/export contract](../RUNS.md), [clock distinction](../GPS_TIMING_ACCURACY.md).

Add rolling presets through the same definition table used by live analysis and CSV reconstruction. Preserve current source-unit adaptation: GPS-log speeds are m/s, ordinary telemetry is km/h. Do not accidentally allow wider gaps because a new short interval has few samples. Missing bracketing or confirmation evidence produces no accepted result.

### Standing launch: separate state machine

Do not simply pass zero into the current rolling detector: its below-start hysteresis cannot be satisfied by ordinary nonnegative speed. Proposed states are waiting for stationary evidence, armed, launch candidate, active, finished/rejected. Require retained continuous near-zero evidence before any candidate and invalidate on gaps or movement before arming. Starting recording while moving cannot recover an invented launch.

Determine a near-zero envelope and dwell using actual parked/creeping/launch logs for both supported receivers. Keep the inferred movement origin bracket and launch-method version in derived evidence. An initial threshold-triggered prototype should expose the actual threshold in its label/export and remain experimental; it must not be sold as independently validated true 0–60 timing. Confirmation may arrive later without moving the stored candidate timestamp. Do not extrapolate far backward from a later high-speed sample.

A practical initial engineering policy is continuous speed at or below 0.3 m/s for at least 1 s, then a candidate at or above 0.5 m/s, using the last qualified stationary epoch as the timing/distance origin and retaining that fix's actual speed. These values are proposed policy, not a sourced standard. Reject or explicitly bound a prolonged 0.3–0.5 m/s creep phase before accepting launch; otherwise the candidate can be multiple epochs away from the last stationary point. Even an adjacent launch bracket bounds the observed samples, not physical onset under receiver noise/filtering. Publish the bracket duration, and avoid a general “accuracy within one GPS epoch” claim. This supplies a reproducible GPS launch estimate while independent evidence remains outstanding.

GPS course is poorly useful near rest. Accept missing course during otherwise qualified stationary lead-in; establish a straightness reference only after reliable motion and check it over the active segment. Reusing a parked course as the launch direction can reject good launches. Do not invent a numeric heading-speed threshold without inspecting receiver behavior. IMU-assisted onset is separate work requiring a common clock and validation, not an excuse to blend display acceleration into GPS timing.

### Distance and optional speed results

For a no-rollout estimate, distance origin and timing origin coincide at the qualified movement estimate. Integrate consecutive valid speed samples using receiver epoch intervals. With speed assumed linear over a segment of duration h, use `deltaDistance = (v0 + v1) * h / 2`. To find a distance endpoint inside that segment, solve the monotone partial integral `remaining = v0*x + 0.5*((v1-v0)/h)*x*x`, with `0 <= x <= h`; use a stable constant-speed branch or bounded numerical solve. This is a proposed consistent integration model, not an assertion about a vendor's internal algorithm.

Finish time is the interpolated distance crossing, not the first whole sample beyond it. Include the partial launch and finish segments. Reject nonfinite/negative distance contributions, discontinuities and incomplete evidence instead of bridging missing time. If the eighth-mile is added later, both distance presets may complete from one qualified launch; later rejection must not erase an already completed shorter result whose required evidence is intact.

If speed is shown, initially use a small secondary finish-speed value with that exact label. An optional modeled 66 ft trap speed would require independently interpolated times at `D - 20.1168 m` and `D`, and `20.1168 / deltaTime`, using the same distance origin. Keep that method in export; do not compute a simple arithmetic mean of samples or call maximum run speed trap speed. Leave trap speed out of the first compact UI if its added semantics are unnecessary.

### Provenance, exclusions and comparison

Retain preset ID, exact thresholds/distance, launch-method version, timing/distance origins, rollout distance, correction mode, source sequence endpoints and interpolated epochs. Separate best results by full timing convention. Export rejected attempts with reason codes where their evidence exists. Missing results remain empty, never zero.

Keep current sequence/time/fix/gap/turn/plausibility rejection rules and add missing-stationary-lead-in and unresolved-launch reasons. GPS quality gates cannot prove a level road or remove wind, multipath and receiver filtering. Label results as uncorrected GPS estimates; absence of a validated grade estimate is unknown grade, not a zero-percent slope or a verified badge. Do not apply third-party leaderboard validity rules without an authoritative specification and matching observables.

## Minimal UI and implementation order

Keep the readable existing preset button and scrolling chooser, one selected interval, Last/Best, receiver status and recording flow. Add no launch wizard, arbitrary custom intervals, separate unit selector, leaderboard, correction toggles or new data store. Standing/distance results need a short “Experimental · no rollout” convention label when enabled. Distinguish waiting for stationary evidence from waiting for fresh GPS. Numerical hundredths remain display resolution, not an accuracy claim.

1. Finish the current font/speed work first. Add new rolling thresholds and shared definitions; preserve existing results/export semantics. Validate exact threshold crossings, short spikes and gap cases on lat.
2. Implement standing-start analysis as an explicitly experimental estimate with replayable launch evidence and honest labels. Use synthetic fixtures for parked noise, slow creep, false spikes, missing lead-in, delayed confirmation, stops/rearming and a recording begun mid-launch. Physical parked/launch observations are required before promoting the label or tuning claims.
3. Add the exact quarter-mile endpoint with partial-segment integration. Validate constant acceleration, constant speed, a crossing near either segment boundary, launch/finish in partial segments and gaps around endpoints. Validate reconstruction/backup equivalence and preset/unit changes on lat. Add eighth-mile, rollout or modeled trap speed only with a pinned additional contract.
4. On a controlled course, retain every attempted run and compare both receiver variants against an independent reference with known synchronization and rollout. Measure false launches, missed runs, signed bias and absolute-error distribution separately for rolling, launch and distance events. Repeat with receiver/antenna installation documented; include opposing directions and grade/wind context. An ordinary drive evaluates data continuity, not absolute performance accuracy. [Hardware acceptance](../HARDWARE_VALIDATION.md).

Software regressions establish behavior for their inputs. They cannot establish real launch accuracy, detection precision/recall or parity with VBOX/Dragy. Documentation/demo generation and automated checks belong on lat under [repository workflow](../process/workflow.md).
