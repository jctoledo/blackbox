# Record, save and replay a drive

Recordings stay in this browser's local storage. **Record session** captures dashboard samples plus available receiver timestamp evidence. **Stop recording** saves the remaining samples. In **Saved data**, choose **Replay** next to a recording.

Replay is a separate, read-only viewer. Choose a recording, press Play, pause, or scrub the timeline. Speed follows the current mph/km/h preference. The G-meter shows the combined horizontal acceleration and an occupant-load view: a right turn loads the left side, braking moves forward/up and acceleration rearward/down. The scale is fixed per axis and can be changed deliberately. Numeric values remain unclipped.

Playback never starts circuit timing, changes tracks, records replayed samples, or replaces live statistics. Stop an active recording before opening the viewer. Closing it returns to the dashboard or Saved data. Backgrounding the page pauses playback.

## Examples on the website

[Open the stored SIM drives](https://jctoledo.github.io/blackbox/demo.html?view=replay). The demo seeds four persistent example recordings: a winding road, a circuit, a rolling drive with a GPS gap, and standing starts with a GPS gap. They use the same saved chunks and playback reader as ordinary recordings. Existing user recordings and older example versions are preserved. These examples are synthetic and labeled **SIM REPLAY**; they are not measured driving results.

The homepage's Play a saved drive button opens that same viewer. Record a session in the demo to create another synthetic recording, then stop and replay it. On a real Blackbox, the ordinary recording flow instead stores the actual received telemetry. No recordings are uploaded by playback.

The SIM winding drive shows a route alongside the existing G-meter. Green and dashed amber segments mark acceleration and braking when the half-second smoothed longitudinal estimate reaches ±0.10 g. **Jump to** uses those smoothed forces to find sustained maneuvers. The G-meter keeps its normal display filter and occupant-load directions. **Close-up** follows the car while the full route stays visible.

## Evidence and missing data

Replay reads committed dashboard samples in chunk order; receiver-only GPS rows remain available through GPS CSV. Pending unsaved data is excluded. Device timestamps determine playback spacing when continuous; a restart or incompatible clock falls back to a positive browser-receipt interval and breaks force continuity. The force display reconstructs the 70 ms smoothing filter, resetting at gaps rather than blending across missing evidence.

GPS and IMU freshness gates still apply. Missing freshness evidence, invalid fixes, gaps, and old samples yield unavailable force values and an empty target, not invented zero acceleration. Seeking into a recording gap does not interpolate a maneuver through it. Replay labels synthetic, real-marked and mixed/unknown provenance separately; it does not certify physical accuracy.

Interactive replay is bounded to 200,000 dashboard samples to keep memory usage explicit. Larger recordings remain exportable through CSV and backup; an oversized replay is refused rather than silently truncated. Browser storage can be cleared by the user or OS; keep portable CSV/JSON backups.

## Offline use

The device dashboard, fonts, player and force renderer are bundled into the served page. They require the device's local connection, not internet access. The public website and its same-origin example assets must first be loaded from the web. Local device disconnection is a separate unavailable state; a frozen value is never presented as fresh.

## Capture a new start line

When creating a circuit or stage, park at the intended start and select the route type. Keep the car stationary through the five-second countdown. Movement, missing receiver fixes or a GPS restart resets the countdown. The device averages distinct accepted GPS fixes to anchor the start line, then begins recording the route. Its direction comes from the subsequent driving path. This reduces position jitter; it does not establish surveyed accuracy. Existing lap timing and ordinary data recording do not require this countdown.
