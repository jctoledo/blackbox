# Saved drive summary: speed, sustained forces and gaps

Open **Saved data** while parked and expand **Summary · saved samples** on a recording. It shows that recording’s maximum qualified speed estimate and strongest half-second acceleration, braking, left and right force estimates. An ordinary drive is enough to populate useful observations; no performance maneuvers are required.

These values belong to the saved recording. They are separate from the live page-session MAX and Peaks, which reset when the page reloads or you choose Reset speed / force stats. Opening another drive does not transfer its results into an earlier recording.

## What qualifies

Maximum speed uses finite, nonnegative ordinary telemetry speed with a valid GPS flag and GPS age from 0 through 300 ms. Stored ordinary `sp` and Session CSV speed are km/h; the calculation converts them once to m/s. The display follows the existing mph/km/h preference. A qualified stationary sample can establish a known zero; no qualifying sample means **Unavailable**.

Force estimates reuse the live Peaks calculation and qualification: finite longitudinal/lateral g values, speed at least 10 km/h, GPS age no greater than 300 ms, and all IMU packet ages represented by an oldest-packet age no greater than 200 ms. Saved analysis also requires a uint32 device timestamp and boot ID so it can establish continuity. Missing legacy quality/clock fields cannot manufacture force results.

Each result is the greatest positive or negative **time-weighted 0.5-second average**, using trapezoidal integration and interpolation at the start of the window. The windows may overlap; the complete-window count is not a count of independent maneuvers. The 10 km/h gate and freshness thresholds are heuristics, not physical accuracy guarantees. Isolated shocks do not become instantaneous records, but plausible sensor errors can still affect the estimates.

A gap over 250 ms, a reboot, backwards device time, invalid force evidence or missing quality breaks the force window. Repeated device timestamps add no force evidence; uint32 rollover is accepted when the forward interval remains valid. Browser wall-clock changes do not repair device-time gaps. Raw receiver GPS rows (`rowType:1`, speed in m/s) are counted separately and never become force samples or enter the speed maximum.

## Saved versus pending

The summary streams committed chunks in session/chunk order. It does not include the current tab’s unsaved tail. Saved data states how many samples are pending; save the recording before relying on a completed summary. If storage fails, keep the tab open and export a backup containing the recoverable tail. After that raw data is successfully restored as copies, it can contribute to the restored recording’s summary.

No full recording is held in memory for this calculation: each recording keeps a bounded half-second window and aggregate counts while chunks pass through. Summaries are calculated when saved data or details export is requested, not on every telemetry update. They are derived views, not replacements for raw recordings.

## Quality and provenance

The disclosure reports saved sample counts, speed/force qualification, complete windows, exclusions, device gaps, duplicate times, clock reversals and reboot boundaries. These counts describe retained evidence; they do not infer missing observations or establish an accuracy percentage. Missing data stays unavailable. A known zero remains distinct from no result.

Synthetic, real-labelled and unknown sample counts remain separate. Any synthetic ordinary sample remains visible in provenance, even when it was only a window endpoint or did not qualify. A zero synthetic count does not independently certify that the source is real or physically accurate. The public demo remains explicitly synthetic.

## Export, reload and restore

**Recording details → Download details JSON** includes `summary` version 1, algorithm `saved-telemetry-v1`, scope `saved-chunks`. Speed is stored as `maxSpeedMps`; `forcePeaksG` is in g. `samples`, `excluded`, `continuity` and `provenance` contain the supporting counts. `pendingSamplesAtExport` describes the unsaved tail observed when the export starts; it is excluded from the summary.

Reload and backup restore recompute the same result from retained raw rows, independently of remapped session IDs. No cached summary from an imported file is trusted. Raw Session CSV and full backups remain authoritative; keep the details JSON with the CSVs to retain the algorithm and context. Device protocol v4 and backup format version 1 are unchanged.

These are vehicle-motion estimates, not calibrated tyre grip, brake-distance measurements, crash detection or a commercial accuracy certificate. Physical frame signs, mounting, receiver timing and ordinary-drive quality still need [hardware acceptance](HARDWARE_VALIDATION.md). See [Drive duration/distance](DRIVE.md) and [maths and update rates](MATHS_AND_RATES.md) for the other measurement paths.
