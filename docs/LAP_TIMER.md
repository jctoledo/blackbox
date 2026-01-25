# Lap Timer User Guide

The Blackbox lap timer provides professional-grade timing features that rival commercial systems costing $700-1200+. Record tracks anywhere, get real-time lap times, and see live delta-to-best while driving.

---

## Quick Start

### 1. Record a Track

1. Open the dashboard at `192.168.71.1`
2. Tap the **lap timer card** at the bottom of the screen
3. Tap **+** to start recording
4. Drive one complete lap (or your point-to-point route)
5. For circuits: Recording auto-detects when you return to start
6. For stages: Tap **Stop Recording** when finished
7. Name your track and tap **Save**

### 2. Start Timing

1. Tap your saved track in the list to activate it
2. The timer shows "Armed" — waiting for you to cross the start line
3. Drive across the start line — timing begins automatically
4. Complete laps and your times appear
5. Best lap automatically saves as your reference

### 3. Watch the Delta

After your first lap, a delta bar appears showing real-time comparison:
- **Green** = you're ahead of your best lap
- **Red** = you're behind your best lap
- **Arrows** show if you're currently gaining or losing time

---

## Understanding the Delta Bar

The delta bar is the key feature that separates this from basic lap timers. It tells you *during the lap* whether you're faster or slower than your best.

### Visual Layout

```
        -2.3s                     You are 2.3 seconds AHEAD
   ◄████████████░░░░░░░░►         Green bar extends LEFT

        +1.5s                     You are 1.5 seconds BEHIND
   ◄░░░░░░░░████████████►         Red bar extends RIGHT

        +0.0s                     Exactly matching your best
   ◄░░░░░░░░░░░░░░░░░░░░►         Bar is centered
```

### Bar Scale

- **±2 seconds = full bar width**
- Half bar = approximately 1 second ahead/behind
- Full bar = 2 or more seconds ahead/behind
- The **number always shows the actual delta**, even if it exceeds ±2s

### Examples

| Delta | Bar | Meaning |
|-------|-----|---------|
| -0.5s | ◄██░░░░░░► | Half second ahead, small green bar left |
| -2.0s | ◄████████░► | Two seconds ahead, full green bar |
| -4.5s | ◄████████░► | 4.5 seconds ahead, full green bar (number shows -4.5s) |
| +1.0s | ◄░░░░████► | One second behind, medium red bar right |
| +3.2s | ◄░████████► | 3.2 seconds behind, full red bar |

### Trend Arrows

The arrows next to the delta show how the gap is changing:

| Arrow | Meaning | What's Happening |
|-------|---------|------------------|
| ▲ | Gaining time | Gap is shrinking — you're going faster than your best lap right now |
| ▼ | Losing time | Gap is growing — you're going slower than your best lap right now |
| (none) | Stable | Gap is staying roughly the same |

**Example:** If you see "+1.5s ▲" it means you're 1.5 seconds behind, but you're currently gaining time — the gap is shrinking.

### Glow Effect

When the delta exceeds 1 second (positive or negative), the bar displays a subtle glow effect for visual emphasis. This helps you notice significant gaps at a glance.

---

## Track Types

### Circuit (Loop Track)

A circuit is a closed loop where start and finish are the same location.

**Auto-detection:** When recording, the system detects you've completed the circuit when:
- You've driven at least 150 meters total
- You're within 25 meters of your starting point
- Your heading is within 30° of your starting heading

**Use cases:**
- Race tracks
- Autocross courses
- Karting tracks
- Neighborhood circuits

### Stage (Point-to-Point)

A stage has separate start and finish locations.

**Recording:** Tap "Stop Recording" when you reach the finish line. The system saves your entire route.

**Use cases:**
- Hill climbs
- Rally stages
- Touge runs
- Drag strips

**Terminology:** For stages, the UI shows "Run" instead of "Lap" and "Running" instead of "Timing."

---

## How Timing Works

### Line Crossing Detection

The system detects when you cross the start/finish line using:

1. **Position proximity** — You must be within a few meters of the line
2. **Direction validation** — You must be traveling in the correct direction (±90°)
3. **Debounce** — 500ms minimum between crossings to prevent double-counting

### Timing Precision

- **30 Hz update rate** — Approximately 33ms timing resolution
- At 100 km/h, this translates to ~1 meter position uncertainty per update
- Sufficient for amateur and club-level timing

### Delta Calculation Method

The delta uses **arc-length comparison**, not position matching:

1. Track your cumulative distance traveled during the current lap
2. Find what time you had at that distance on your reference lap
3. Subtract to get delta

**Why arc-length?** Position matching fails when you take different racing lines. Arc-length works regardless of your exact path — if you're 500m into the lap, it compares to your best lap at 500m.

---

## Session History

Every lap time is saved automatically and grouped by session.

### Viewing History

1. Open the track list (tap lap timer card)
2. Look for the **H** button next to tracks that have history
3. Tap **H** to expand the history section

### What's Saved

| Data | Saved? |
|------|--------|
| Every lap time | Yes |
| Session grouping | Yes (by date) |
| Best lap per session | Highlighted |
| Reference lap (for delta) | Yes, auto-updated when you set a new best |

### Session Grouping

Laps are grouped by when you activated the track:
- **Today** — Sessions from today
- **Yesterday** — Sessions from yesterday
- **Date** — Older sessions show the date

Each session shows:
- Start time
- Best lap in session
- Total laps completed
- All individual lap times

### Clearing History

Tap **Clear History** within the history section to delete all lap history for that track. This does not affect the track itself or your reference lap.

---

## Track Auto-Detection

When you're driving near a saved track, the system automatically offers to activate it.

### How It Works

1. Every 3 seconds, your position is checked against all saved tracks
2. If you're within 50 meters of a track's centerline with similar heading
3. After 2 consecutive matches (~6 seconds), a toast notification appears
4. Tap **Activate** to enable timing, or **Dismiss** to ignore

### Why 6 Seconds?

Two consecutive matches are required to prevent false positives from:
- Driving past a track on a nearby road
- GPS noise momentarily placing you on the track
- Brief proximity while navigating to the track

### Detection Thresholds

| Parameter | Value |
|-----------|-------|
| Check interval | 3 seconds |
| Required matches | 2 consecutive |
| Max centerline distance | 50 meters |
| Heading tolerance | Combined with distance in score |

---

## Data Management

Access the **Data** modal from the hamburger menu to manage all stored data.

### Storage Overview

The modal shows:
- **Total storage used** across all IndexedDB data
- **Breakdown** between recordings and tracks

### Recordings Section

All telemetry recording sessions (from the REC button):
- Date and time
- Number of chunks and total size
- Duration
- Status (Recording, Complete, Recovered)

**Actions:**
- **Export** — Download as CSV
- **Delete** — Remove recording and free storage
- **Clear All Recordings** — Delete all recordings

### Tracks Section

All saved tracks with their reference laps:
- Track name
- Corner count and point count
- Storage size
- Best lap time (if reference exists)

**Actions:**
- **Clear Best** — Delete reference lap (keeps track)
- **Delete** — Remove track, reference lap, and all history
- **Clear All Tracks** — Delete everything

---

## Recording Tips

### For Best Results

| Do | Don't |
|----|-------|
| Wait for GPS lock (cyan LED pulse) | Record with yellow LED blinking |
| Drive smoothly and consistently | Make sudden lane changes |
| Complete one clean lap | Cut corners or go off-track |
| Record in good GPS conditions | Record in urban canyons or tunnels |

### GPS Quality

During recording, the overlay shows GPS quality:
- **Excellent/Good** — Proceed with recording
- **Fair** — Recording will work but accuracy may suffer
- **Poor** — Wait for better signal

### Ideal Recording Lap

Your first recorded lap becomes the centerline for the track. For best results:
- Drive the racing line you'll typically use
- Maintain consistent speed through corners
- Don't push to the limit — a smooth lap records better

---

## Troubleshooting

### Timing Not Starting

**Symptom:** Crossed start line but timer stays "Armed"

**Causes:**
- Not close enough to the line (GPS inaccuracy)
- Approaching from wrong direction
- Track coordinates don't match current GPS origin

**Solutions:**
- Drive closer to where you recorded the start
- Ensure you're traveling in the correct direction
- If device was restarted, GPS origin may have shifted — re-record track

### Delta Shows "No Best"

**Symptom:** Completed laps but no delta bar appears

**Cause:** Reference lap not yet saved

**Solution:** Complete at least one full lap. The first completed lap becomes your reference, and delta will appear on subsequent laps.

### Lap Times Seem Wrong

**Symptom:** Times don't match expectations

**Causes:**
- Start/finish line in wrong location
- GPS drift during recording
- Debounce triggering (very slow crossings)

**Solutions:**
- Re-record track with cleaner GPS signal
- Cross start/finish at reasonable speed
- Check diagnostics for GPS quality

### Track Not Auto-Detected

**Symptom:** Driving at a saved track but no notification appears

**Causes:**
- GPS origin shifted (device restarted since track recorded)
- More than 50m from centerline
- Heading differs significantly from recorded direction

**Solutions:**
- Drive closer to where you originally recorded
- Re-record track with current GPS origin

---

## Technical Details

### Centerline Storage

When you record a track, GPS points are sampled approximately every 5 meters to create a "centerline." This centerline is used for:

- **Auto-detection** — Checking if you're near a saved track
- **Loop closure** — Detecting when you complete a circuit
- **Future features** — Track learning to improve accuracy over time

### Reference Lap Storage

Reference laps are stored as distance-time pairs:
- Every ~100ms during your lap, a sample is recorded
- Each sample stores: distance traveled, elapsed time
- Typical storage: 30-60 KB per reference lap

### IndexedDB Structure

Data is stored in IndexedDB with these stores:
- `tracks` — Track definitions (centerline, start/finish lines)
- `reference_laps` — Best lap data for delta calculation
- `lap_history` — All completed lap times

### Storage Limits

| Data Type | Typical Size | Estimate for 10 Tracks |
|-----------|--------------|------------------------|
| Track definition | 5-20 KB | 100-200 KB |
| Reference lap | 30-60 KB | 300-600 KB |
| Lap history | ~100 bytes/lap | 100 KB (1000 laps) |
| **Total** | | **~1 MB** |

Well within IndexedDB limits (50+ MB typical).

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
| GPS antenna | External module | Internal | Internal |
| Price | ~$50 | $1,160 | $700 |

The main tradeoffs are timing precision (adequate for amateur use) and using your phone as the display instead of a dedicated screen.
