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

1. Tap your saved track in the list, then tap **Use**
2. Timer card changes to show "Armed" with track name
3. **Start line indicator** appears showing distance and direction: "Start line: 150 m ↗"
4. Follow the arrow toward the start line — messages update as you approach
5. When you see "Cross to begin!" you're at the start line
6. Drive across — timing begins automatically
7. Complete laps and your times appear
8. Best lap automatically saves as your reference

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

## Timer States

The lap timer operates as a state machine with three states:

```
┌─────────┐    Click Use    ┌─────────┐   Cross Start   ┌─────────┐
│  IDLE   │ ──────────────► │  ARMED  │ ──────────────► │ TIMING  │
└─────────┘                 └─────────┘                 └─────────┘
     ▲                           │                           │
     │         Deactivate        │      Cross Start/Finish   │
     └───────────────────────────┴───────────────────────────┘
                                          (lap complete)
```

| State | UI Display | What's Happening |
|-------|------------|------------------|
| **Idle** | "Tap to configure" | No track active, timer hidden |
| **Armed** | "Armed" + Start line indicator | Track active, waiting for start line crossing |
| **Timing** | Running clock + delta bar | Actively timing a lap/run |

**Circuit tracks:** After crossing start, state goes TIMING → ARMED (ready for next lap).

**P2P tracks:** After crossing finish, state goes TIMING → "Finished!" → ARMED. A brief "Finished!" animation plays before returning to Armed state.

---

## Track Activation (Use Button)

When you tap a saved track and click **Use**, the timer immediately activates:

1. **Timer card transitions** from "Tap to configure" to show track name + "Armed"
2. **Start line indicator appears** showing distance and direction to the start line
3. **Delta bar initializes** (shows "No ref" if no reference lap exists)

### What You See at Different Distances

The start line indicator updates in real-time as you drive toward the start:

| Distance | Message | Visual State |
|----------|---------|--------------|
| > 100m | "Start line: 250 m ↗" | Default styling |
| 50-100m | "Approaching start: 75 m ↗" | Amber/approaching |
| 15-50m | "Almost there: 30 m ↗" | Highlight/close |
| < 15m | "Cross to begin! ↗" | Ready/at-line |

The arrow (↗, ↑, ←, ↓, etc.) shows the direction to drive based on your current heading.

### Distance Has No Limit

You can activate a track from any distance. If you're 2 km away, it shows "Start line: 2.0 km ↗". The indicator guides you to the start line regardless of how far you are.

---

## Start Line Indicator

The start line indicator is your navigation aid when a track is armed but you haven't crossed the start yet.

### How It Works

1. Calculates distance from your current position to the center of the start line
2. Computes bearing (compass direction) from you to the start line
3. Converts bearing to an arrow relative to your current heading
4. Updates every telemetry frame (~30 Hz)

### Arrow Directions

| Arrow | Meaning |
|-------|---------|
| ↑ | Start line is directly ahead |
| ↗ | Start line is ahead and to the right |
| → | Start line is to your right |
| ↘ | Start line is behind and to the right |
| ↓ | Start line is behind you |
| ↙ | Start line is behind and to the left |
| ← | Start line is to your left |
| ↖ | Start line is ahead and to the left |

### When It's Hidden

The start line indicator is hidden when:
- Timer is in IDLE state (no track active)
- Timer is in TIMING state (you've already crossed the start)
- Position data is unavailable

---

## Finish Line Indicator (P2P Only)

Point-to-point tracks have separate start and finish lines. Once timing begins, a **finish line indicator** appears instead of the start line indicator.

### Messages

| Distance | Message |
|----------|---------|
| > 100m | "Distance to finish: 250 m ↗" |
| 50-100m | "Approaching finish: 75 m ↗" |
| 15-50m | "Almost there: 30 m ↗" |
| < 15m | "Cross to finish! ↗" |

### After Finishing

When you cross the finish line:
1. Timer stops and shows your run time
2. "Finished!" animation plays briefly
3. State returns to Armed with message: "Return to start: X m ↗"
4. You must drive back to the start line for another run

---

## Recording a New Track

### Starting a Recording

1. Tap the lap timer card to open track selection
2. Tap the **+** button
3. Choose track type:
   - **Circuit (Loop)** — Start and finish are the same location
   - **Stage (Point-to-Point)** — Separate start and finish locations
4. Recording overlay appears showing:
   - GPS quality indicator
   - Distance traveled
   - Corner count
   - Elapsed time

### During Recording

Drive your intended route. The system:
- Samples GPS position approximately every 5 meters
- Detects corners via curvature analysis
- Monitors GPS quality continuously

**GPS Quality Indicators:**

| Quality | Color | Meaning |
|---------|-------|---------|
| Excellent | Green | Proceed confidently |
| Good | Green | Normal operation |
| Fair | Yellow | Acceptable but not ideal |
| Poor | Red | Wait for better signal |

### Loop Detection (Circuit Tracks)

For circuit tracks, the system automatically detects when you complete the loop:

**Detection Criteria:**
- Traveled at least 150 meters total
- Within 25 meters of starting position
- Heading within 30° of starting heading

When detected, you see:
```
┌────────────────────────────────┐
│        ✓ Loop Detected!        │
│                                │
│   [Save Track]  [Keep Driving] │
└────────────────────────────────┘
```

- **Save Track** — Opens the save dialog
- **Keep Driving** — Continues recording (add more distance or complete another lap for accuracy)

### Finishing P2P Tracks

For point-to-point tracks, manually stop recording:
1. Drive to your intended finish location
2. Tap **Mark Finish** (or **Stop Recording**)
3. Save dialog appears

### Save Dialog

After recording completes:
1. Enter a track name
2. Tap **Save**
3. Track is saved and **automatically activated**
4. Timer transitions to Armed state

---

## After Recording: First Lap Behavior

### Circuit Tracks

After saving a new circuit track:
1. Timer shows Armed state
2. Start line indicator shows: "Drive a lap, then cross start to begin"
3. This reminds you to complete one full lap before timing starts
4. Once you cross the start line, timing begins normally

### P2P Tracks: Warmup Requirement

For newly recorded P2P tracks, there's a **warmup requirement**:

1. After saving, you're at the finish location
2. Message shows: "Return to start: X m ↗"
3. You must drive at least **50 meters away** from the start line
4. Only then will start line crossings be detected

**Why warmup?** Without this, the timer would immediately trigger because you're already near where you started recording. The warmup ensures you intentionally approach the start for your first timed run.

### Subsequent Sessions

When you activate a previously-saved track (not newly recorded):
- No warmup requirement
- Standard start line indicator shows distance/direction
- Cross the start to begin timing immediately

---

## Deactivating a Track

To stop timing and deactivate the current track:

1. Tap the lap timer card
2. Tap the **×** next to the active track name

Or:
1. Open the track list
2. The active track shows a checkmark
3. Tapping it again (or another track) deactivates it

When deactivated:
- Timer returns to IDLE state
- Shows "Tap to configure"
- All session data (lap times) remains saved
- Reference lap (best time) remains saved

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

### Timer Not Showing After Clicking "Use"

**Symptom:** Clicked Use on a saved track but the timer doesn't appear

**Causes:**
- UI state not properly initialized
- JavaScript error in browser console

**Solutions:**
- Refresh the dashboard page
- Check browser console for errors
- Verify track was saved correctly (appears in track list)

### Start Line Indicator Not Showing

**Symptom:** Track is active (Armed) but no distance/direction indicator appears

**Causes:**
- GPS position not valid yet
- Timer already in TIMING state (indicator hidden while timing)
- Track missing start line data

**Solutions:**
- Wait for GPS lock (cyan LED on device)
- Check if timer shows "Timing" instead of "Armed"
- Re-record the track if data appears corrupted

### Timing Not Starting

**Symptom:** Crossed start line but timer stays "Armed"

**Causes:**
- Not close enough to the line (GPS inaccuracy)
- Approaching from wrong direction
- Track coordinates don't match current GPS origin
- **P2P warmup not completed** — for new P2P tracks, must be 50m+ away first

**Solutions:**
- Drive closer to where you recorded the start
- Ensure you're traveling in the correct direction
- For new P2P tracks: drive away from start, then return
- If device was restarted, GPS origin may have shifted — re-record track

### P2P Track Won't Start Timing

**Symptom:** New point-to-point track, crossed start multiple times, timer stays Armed

**Cause:** P2P warmup requirement not met

**Solution:** After recording a new P2P track, you must drive at least 50 meters away from the start line before timing will engage. This prevents accidental triggers since you're already at the recording location.

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

### Loop Not Detected During Recording

**Symptom:** Completed a full circuit but "Loop Detected" message didn't appear

**Causes:**
- Total distance less than 150 meters
- Not close enough to starting point (>25m away)
- Heading differs too much from starting heading (>30°)

**Solutions:**
- Keep driving to increase total distance
- Drive directly toward your starting point
- Approach from the same direction you started
- If track is very small (<150m), it may not qualify as a circuit

### Track Not Auto-Detected

**Symptom:** Driving at a saved track but no notification appears

**Causes:**
- GPS origin shifted (device restarted since track recorded)
- More than 50m from centerline
- Heading differs significantly from recorded direction
- Detection requires ~6 seconds of proximity (2 consecutive checks)

**Solutions:**
- Drive closer to where you originally recorded
- Stay on the track centerline for at least 6 seconds
- Re-record track with current GPS origin

---

## UI Quick Reference

### Timer Card States

| State | Display | Indicator | Action Available |
|-------|---------|-----------|------------------|
| Idle | "Tap to configure" | None | Tap to open track list |
| Armed | Track name + "Armed" | Start line distance/direction | Wait for crossing |
| Timing | Running clock | Delta bar (if reference exists) | — |
| Finished (P2P) | Final time + "Finished!" | Brief animation | Returns to Armed |

### Start Line Indicator Messages

| Distance | Circuit Track | P2P Track (new) | P2P Track (returning) |
|----------|---------------|-----------------|----------------------|
| Far | "Start line: X m ↗" | "Return to start: X m ↗" | "Start line: X m ↗" |
| Near | "Almost there: X m ↗" | "Return to start: X m ↗" | "Almost there: X m ↗" |
| At line | "Cross to begin! ↗" | "Cross to begin! ↗" | "Cross to begin! ↗" |
| After new record | "Drive a lap, then cross start to begin" | — | — |

### Recording Overlay Elements

| Element | Description |
|---------|-------------|
| GPS Quality | Excellent/Good/Fair/Poor indicator |
| Distance | Total meters traveled |
| Corners | Number of corners detected |
| Time | Recording duration |
| Loop Detected | Appears when circuit completion detected |

### Delta Bar Colors

| Color | Meaning | Example |
|-------|---------|---------|
| Green | Ahead of best | -1.5s (you're faster) |
| Red | Behind best | +2.3s (you're slower) |
| Gray | No reference | First lap, no comparison |

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
