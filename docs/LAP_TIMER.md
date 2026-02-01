# Lap Timer Guide

Professional-grade timing that rivals $700-1200 commercial systems. Record tracks anywhere, get real-time lap times, and see live delta-to-best while driving.

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

*Use cases: race tracks, autocross, karting, neighborhood circuits*

### Stage (Point-to-Point)
Separate start and finish. Tap "Stop Recording" at finish.

*Use cases: hill climbs, rally stages, touge, drag strips*

UI shows "Run" instead of "Lap" and "Running" instead of "Timing."

---

## How Timing Works

### Line Detection
1. **Position proximity** — within meters of line
2. **Direction validation** — traveling correct direction (±90°)
3. **Debounce** — 500ms minimum between crossings

### Precision
- 30 Hz update rate (~33ms resolution)
- At 100 km/h: ~1 meter position uncertainty per update
- Sufficient for amateur and club-level timing

### Delta Calculation
Uses **arc-length comparison**:
1. Track cumulative distance traveled
2. Find time at that distance on reference lap
3. Difference = delta

*Why arc-length?* Position matching fails with different racing lines. Arc-length works regardless of path — 500m into the lap compares to best at 500m.

---

## Timer States

```
┌─────────┐    Use Track    ┌─────────┐   Cross Start   ┌─────────┐
│  IDLE   │ ───────────────► │  ARMED  │ ───────────────► │ TIMING  │
└─────────┘                  └─────────┘                  └─────────┘
     ▲                            │                            │
     └────────────────────────────┴────────────────────────────┘
                          Deactivate / Lap Complete
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
- Reference lap (for delta) auto-updated on new best

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
30 Hz = 33ms resolution. At 100 km/h, ~0.9m position uncertainty per frame.

### Storage Estimates
| Data | Size | 10 Tracks |
|------|------|-----------|
| Track definition | 5-20 KB | 200 KB |
| Reference lap | 30-60 KB | 600 KB |
| Lap history | 100 bytes/lap | 100 KB (1000 laps) |
| **Total** | | ~1 MB |

Well within IndexedDB limits (50+ MB).

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

## Comparison to Commercial Systems

| Feature | Blackbox | VBOX LapTimer ($1,160) | AiM Solo 2 DL ($700) |
|---------|----------|------------------------|----------------------|
| Timing precision | ~33ms | ~10ms | ~10ms |
| Delta display | Yes | Yes | Yes |
| Track recording | Yes | Yes | Yes |
| Session history | Yes | Yes | Yes |
| Track auto-detect | Yes | Yes | Yes |
| Separate display | No (phone) | Yes | Yes |
| Price | ~$50 | $1,160 | $700 |

Main tradeoffs: timing precision (adequate for amateur use) and phone as display instead of dedicated screen.

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

#### GPS Noise (~1m jitter at 25Hz)

**Issue:** Consumer GPS has position noise of approximately 1 meter.

**Mitigation:** The lap timer uses trajectory-based intersection detection—it checks if the *path* from previous position to current position crosses the timing line, not just proximity. This is robust to per-sample noise.

**Why it works:** A 24-meter timing line (12m each side of track center) easily absorbs 1m of noise. False triggers are prevented by direction validation (must be traveling the correct way).

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

**f64 (double) Precision:** GPS coordinates use f64 for approximately 1mm precision at equator. More than sufficient for timing applications.

**Local Conversion:** The `to_local()` function uses the timing line's p1 as reference, converting both line endpoints and vehicle position consistently. The intersection calculation uses f64 throughout.
