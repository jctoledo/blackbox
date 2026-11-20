# Lap Timer Guide

GPS-based lap timing for an open telemetry project. Record circuits or stages, select a track to arm timing, and compare laps in the device dashboard. Accuracy remains dependent on receiver performance and field validation; commercial-system equivalence has not been established.

---

## Quick Start

### Record a Track
1. Open dashboard at `192.168.71.1`
2. Tap **lap timer card** → **+** to record
3. Drive one complete lap
4. For circuits: auto-detects loop completion. For stages: tap **Stop Recording**
5. Name and save

### Start Timing
1. Tap saved track → **Use**
2. **Start line indicator** shows distance and direction: "Start line: 150 m ↗"
3. Drive across start line — timing begins automatically
4. Complete laps; best lap auto-saves as reference

### Watch the Delta
After first lap, delta bar shows real-time comparison:
- **Green** = ahead of best, **Red** = behind
- **Arrows** show trend: ▲ gaining, ▼ losing

---

## Delta Bar

The delta bar is the key feature — it shows *during the lap* if you're ahead or behind.

```
    -2.3s                     2.3 seconds AHEAD
◄████████████░░░░░░░░►        Green bar extends LEFT

    +1.5s                     1.5 seconds BEHIND
◄░░░░░░░░████████████►        Red bar extends RIGHT
```

**Scale:** ±2 seconds = full bar. Number shows actual delta if beyond ±2s.

**Glow effect:** Activates when delta exceeds 1 second for visual emphasis.

---

## Track Types

### Circuit (Loop)
Closed loop where start and finish are the same. Auto-detection triggers when:
- 150+ meters traveled
- Within 25m of starting point
- Heading within 30° of start

*Use cases: controlled circuits and repeatable closed-course routes*

### Stage (Point-to-Point)
Separate start and finish. First record the route; stop safely at the end and tap "Mark Finish" to save its finish line. Arm the saved stage on a later pass to time it.

*Use cases: point-to-point routes, hill climbs, and rally stages. Normal road recordings are useful for diagnostics; performance testing belongs at a controlled site.*

UI shows "Run" instead of "Lap" and "Running" instead of "Timing."

---

## How Timing Works

### Line Detection
1. **Finite segment intersection** — the segment between consecutive fresh fixes intersects the saved timing line. Proximity alone does not start timing.
2. **Direction validation** — receiver velocity agrees with the recorded direction (±90°). Positions below 0.5 m/s do not trigger crossings.
3. **Guards** — loops use a 500 ms debounce; completed laps/runs must last at least 10 seconds. A second start crossing does not restart an active stage. Gaps over two seconds break crossing continuity.

### Precision
- Crossing detection on new GPS fixes, independent of browser refresh rate
- At 100 km/h, the vehicle travels about 1.11 m between 25 Hz fixes; that is sample spacing, not a position-error bound.
- Suitability for a particular timing requirement must be established against an independent reference.

### Delta Calculation
Uses **arc-length comparison**:
1. Track cumulative distance traveled
2. Find time at that distance on reference lap
3. Difference = delta

Arc-length comparison is inexpensive, but equal distance need not mean the same track location when lines differ. Position noise can accumulate extra distance. Treat the delta and progress bar as estimates; detected gaps, late starts, and large jumps invalidate them until a new lap. This is not corner-matched coaching.

---

## Timer States

```text
Idle -- select track --> Armed -- valid start crossing --> Timing
Timing -- circuit crossing --> Timing (next lap)
Timing -- stage finish --> Armed (ready for another run)
Armed or Timing -- deactivate --> Idle
```

| State | Display | Notes |
|-------|---------|-------|
| **Idle** | "Tap to configure" | No track active |
| **Armed** | Track name + start indicator | Waiting for line crossing |
| **Timing** | Running clock + delta bar | Active timing |

---

## Start/Finish Indicators

Arrow updates in real-time as you approach:

| Distance | Message |
|----------|---------|
| > 100m | "Start line: 250 m ↗" |
| 50-100m | "Approaching start: 75 m ↗" |
| 15-50m | "Almost there: 30 m ↗" |
| < 15m | "Cross to begin! ↗" |

For P2P tracks, similar messages show for finish line once timing starts.

---

## Recording

### During Recording
The overlay shows:
- GPS quality (Excellent/Good/Fair/Poor)
- Distance traveled
- Corner count
- Elapsed time

**GPS Quality:** Record with Good or better. Fair works but accuracy suffers. Poor — wait for better signal.

### Loop Detection
For circuits, "Loop Detected!" appears when criteria met. Options:
- **Save Track** — open save dialog
- **Keep Driving** — continue for more accuracy

### Tips
- Wait for GPS lock (cyan LED pulse)
- Drive smoothly and consistently
- Drive your typical racing line
- Don't push to the limit — smooth lap records better

---

## Track Auto-Detection

When driving near a saved track, the system offers to activate it.

**How it works:**
- Checks position every 3 seconds against saved tracks
- Requires 2 consecutive matches (~6 seconds)
- Toast notification: "Track detected: [name]"
- Tap **Activate** or **Dismiss**

**Thresholds:** 50m from centerline, combined distance + heading score

---

## Session History

Every lap time saves automatically, grouped by session.

### Viewing
1. Open track list (tap lap timer card)
2. Tap **H** button next to tracks with history
3. View sessions grouped by date

### What's Saved
- Every lap time with session grouping
- Best lap per session highlighted
- Reference lap (for delta) updated by a faster usable trace; an invalid trace cannot replace it

---

## Data Management

Access via **Data** in hamburger menu.

**Storage Overview:** Total usage, recordings vs tracks breakdown

**Recordings:** Export CSV, delete individual sessions, clear all

**Tracks:** Clear best lap, delete track, clear all tracks

---

## Troubleshooting

### Timer Not Starting
**Causes:**
- Not close enough to line (GPS inaccuracy)
- Wrong direction approach
- For new P2P tracks: warmup not complete (must be 50m+ away first)
- GPS signal lost or degraded

**Solutions:**
- Drive closer to recorded start
- Ensure correct direction
- For new P2P: drive away, then return
- Wait for GPS lock (cyan LED pulse) before starting

### Delta Shows "No Best"
Complete at least one full lap. First completed lap becomes reference.

### Lap Times Seem Wrong
- Re-record with cleaner GPS
- Cross at reasonable speed
- Check diagnostics for GPS quality

### Loop Not Detected
- Distance < 150m? Keep driving
- > 25m from start? Drive closer
- Heading off > 30°? Approach from starting direction

---

## Technical Reference

### Timing Precision
Crossing times are interpolated between two fresh GPS fixes and stored in integer milliseconds. Millisecond storage and smooth display do **not** establish millisecond physical accuracy. Receiver latency, timestamp alignment, multipath, line geometry, and speed all matter. For scale, 1 m of along-track error at 100 km/h corresponds to about 36 ms; this is an illustrative conversion, not a measured error bound.

### Storage Estimates
| Data | Size | 10 Tracks |
|------|------|-----------|
| Track definition | 5-20 KB | 200 KB |
| Reference lap | 30-60 KB | 600 KB |
| Lap history | 100 bytes/lap | 100 KB (1000 laps) |
| **Total** | | ~1 MB |

Browser quota varies; storage can fill or be evicted. Export important data and use the visible save/error status. Do not assume a fixed IndexedDB capacity.

### Corner Detection
State machine with hysteresis:
- Entry threshold: 0.025 rad/m (sharp turn)
- Exit threshold: 0.012 rad/m (straightening out)
- Minimum length: 3 meters

### Loop Detection Thresholds
| Parameter | Value |
|-----------|-------|
| Min track length | 150m |
| Close proximity | 25m |
| Heading tolerance | 30° |

---

## Validation and limits

Synthetic replays cover multiple track headings, latitudes, irregular sample intervals, and timestamp rollover. They verify algorithm behavior with known inputs; they do not measure the receiver's real-world errors. See [Review & validation](REVIEW.md) for the remaining field test plan.

Timing uses fresh raw GPS positions and velocity from the same receiver. Gaps longer than two seconds break crossing continuity. A crossing during an outage may be missed; the software intentionally does not infer it from a long reconnect segment. Do not treat EKF interpolation or a higher display rate as additional position truth.

---

## GPS Coordinate Architecture

The lap timer uses **GPS lat/lon coordinates** instead of local EKF (Extended Kalman Filter) coordinates for timing line detection. This architectural choice ensures tracks work across device restarts and sessions.

### Why GPS Instead of EKF?

**The Problem:** Each time the device starts, the EKF computes a new local coordinate origin from the first few GPS fixes. This "GPS origin" varies by several meters between sessions. Tracks saved in one session had timing lines in that session's coordinate frame—unusable in a different session.

**The Solution:** Store and transmit timing lines as absolute GPS lat/lon coordinates. The firmware receives GPS directly from the sensor, eliminating any coordinate frame mismatch.

### How It Works

1. **Track Recording:** Dashboard records track using current GPS origin for local display
2. **Track Saving:** Timing line stored as local x,y + `gpsOrigin` reference
3. **Track Activation:** Dashboard converts local→GPS using track's own `gpsOrigin`
4. **Firmware API:** Receives GPS lat/lon directly (no session dependency)
5. **Line Detection:** Firmware converts GPS to local coords relative to timing line's p1

The key insight: each timing line carries its own GPS reference (the p1 endpoint). All coordinate math uses this consistent reference point.

### Known Limitations & Mitigations

#### GPS noise and line geometry

**Issue:** Consumer GPS has meter-scale uncertainty that changes with reception, antenna placement, and multipath. A fixed 1 m jitter bound is not established for this device.

**Mitigation:** The lap timer uses trajectory-based intersection detection—it checks if the *path* from previous position to current position crosses the timing line, not just proximity. This catches crossings between samples, but cannot eliminate position noise.

A sufficiently wide line reduces missed intersections near the track edge. Direction and minimum-lap gates reduce false triggers, but cannot guarantee their elimination. Verify that the saved line spans the route and that its direction matches the intended crossing.

#### GPS Dropout

**Issue:** During brief GPS signal loss (tunnels, heavy tree cover), no crossings are detected.

**Current Behavior:** The lap timer only processes valid GPS fixes. No detection during dropout is correct behavior—better than wrong detections.

**Potential Enhancement:** Fall back to EKF position during GPS dropout by converting EKF (x,y) to GPS using firmware's GPS origin:
```rust
// Pseudocode for potential enhancement
if gps_fix.valid {
    lap_timer.update(gps_fix.lat, gps_fix.lon, ...);
} else if gps_parser.is_warmed_up() {
    // Convert EKF position to GPS using firmware's origin
    let (ekf_x, ekf_y) = ekf.position();
    let ref_origin = gps_parser.reference();
    let fallback_lat = ref_origin.lat + ekf_y / 111320.0;
    let fallback_lon = ref_origin.lon + ekf_x / (111320.0 * cos(ref_origin.lat));
    lap_timer.update(fallback_lat, fallback_lon, ...);
}
```
This maintains session independence while providing continuity during brief dropouts.

#### Coordinate Precision

**Numeric precision:** timing-line calculations and HTTP GPS coordinates use f64. Binary v4 packets retain f32 latitude/longitude for compatibility, introducing location-dependent sub-meter quantization. Numeric representation is separate from GPS measurement accuracy.

**Local Conversion:** The `to_local()` function uses the timing line's p1 as reference, converting both line endpoints and vehicle position consistently. The intersection calculation uses f64 throughout.
