# Feature: Predictive Lap Timer with Start/Finish Line Detection

## Summary

Add a lap timer feature to the blackbox dashboard that provides real-time lap timing with predictive delta display, comparable to commercial solutions like VBOX LapTimer ($1,160) and AiM Solo 2 DL ($700).

## Motivation

Commercial lap timers are expensive ($500-1,500+) and locked into proprietary ecosystems. Blackbox already has all the sensor data needed for accurate lap timing:
- EKF-fused position at 200 Hz (interpolated between GPS fixes)
- Precise timestamps (ms resolution)
- Heading/yaw for line orientation
- Dashboard for display

Adding lap timing would make blackbox competitive with devices costing 10-20x more.

| Feature | VBOX LapTimer | AiM Solo 2 DL | Blackbox (proposed) |
|---------|---------------|---------------|---------------------|
| Price | $1,160 | $700 | ~$50 |
| Lap Timing | ✓ | ✓ | ✓ |
| Predictive Delta | ✓ | ✓ | ✓ |
| GPS Rate | 25 Hz | 10 Hz | 5-10 Hz + 200 Hz EKF |
| Set Line via Button | ✗ (predefined tracks) | ✗ | ✓ |
| Open Data Format | ✗ | ✗ | ✓ |

## User Experience

### What is a "Track"?

A track is a saved route with timing lines. Two types are supported:

**Loop (circuit):** Single start/finish line. Cross it to start, cross it again to complete a lap.
- Race tracks, autocross courses, karting tracks
- Also works for out-and-back canyon roads (drive up, turn around, drive back)

**Point-to-Point:** Separate start and finish lines at different locations.
- Hill climbs, canyon runs, rally stages
- Timing starts at start line, ends at finish line

```javascript
Track {
  id: "uuid-1234",
  name: "Belmont Grand Prix",
  type: "loop",                // or "point-to-point"
  start_line: { p1: [lat, lon], p2: [lat, lon], direction: 1.57 },
  finish_line: null,           // Same as start_line for loops
  width_m: 30,
  best_lap_ms: 94823,
  total_laps: 12,
  created: "2024-01-15",
  last_used: "2024-01-20"
}
```

### Creating a New Track

```
┌─────────────────────────────────────────────┐
│  🏁 LAP TIMER                               │
├─────────────────────────────────────────────┤
│                                             │
│  ┌─────────────────────────────────────┐    │
│  │ 📍 No track loaded                  │    │
│  │                                     │    │
│  │ [Load Saved Track]  [Create New]    │    │
│  └─────────────────────────────────────┘    │
│                                             │
└─────────────────────────────────────────────┘

         User taps "Create New"
                   ↓

┌─────────────────────────────────────────────┐
│  🏁 TRACK TYPE                              │
├─────────────────────────────────────────────┤
│                                             │
│  ┌─────────────────────────────────────┐    │
│  │ 🔄 Loop / Circuit                   │    │
│  │    Cross same line to complete lap  │    │
│  └─────────────────────────────────────┘    │
│                                             │
│  ┌─────────────────────────────────────┐    │
│  │ 📍 Point-to-Point                   │    │
│  │    Separate start and finish lines  │    │
│  └─────────────────────────────────────┘    │
│                                             │
└─────────────────────────────────────────────┘
```

### Creating a Loop Track

```
         User selects "Loop / Circuit"
                   ↓

┌─────────────────────────────────────────────┐
│  🏁 CREATE LOOP TRACK                       │
├─────────────────────────────────────────────┤
│                                             │
│  1. Drive to start/finish line              │
│  2. Stop on the line, facing track direction│
│  3. Tap button below                        │
│                                             │
│  Current position:                          │
│  37.5202°N, 122.2758°W                      │
│  Speed: 0.0 km/h ✓                          │
│                                             │
│  ┌─────────────────────────────────────┐    │
│  │      [📍 SET START/FINISH]          │    │
│  └─────────────────────────────────────┘    │
│                                             │
└─────────────────────────────────────────────┘

         User taps "Set Start/Finish"
                   ↓

┌─────────────────────────────────────────────┐
│  🏁 NAME YOUR TRACK                         │
├─────────────────────────────────────────────┤
│                                             │
│  Track name:                                │
│  ┌─────────────────────────────────────┐    │
│  │ Belmont Grand Prix               ▼  │    │
│  └─────────────────────────────────────┘    │
│  (Near: Belmont, CA)                        │
│                                             │
│  Line width: [====●====] 30m                │
│                                             │
│  [Cancel]              [Save & Start]       │
│                                             │
└─────────────────────────────────────────────┘
```

### Creating a Point-to-Point Track

```
         User selects "Point-to-Point"
                   ↓

┌─────────────────────────────────────────────┐
│  🏁 CREATE POINT-TO-POINT TRACK             │
├─────────────────────────────────────────────┤
│                                             │
│  Step 1 of 2: Set START line                │
│                                             │
│  Drive to where you want timing to begin.   │
│  Stop on the line, facing run direction.    │
│                                             │
│  Current position:                          │
│  37.5202°N, 122.2758°W                      │
│  Speed: 0.0 km/h ✓                          │
│                                             │
│  ┌─────────────────────────────────────┐    │
│  │      [📍 SET START LINE]            │    │
│  └─────────────────────────────────────┘    │
│                                             │
└─────────────────────────────────────────────┘

         User sets start, drives to end of run
                   ↓

┌─────────────────────────────────────────────┐
│  🏁 CREATE POINT-TO-POINT TRACK             │
├─────────────────────────────────────────────┤
│                                             │
│  Step 2 of 2: Set FINISH line               │
│                                             │
│  ✓ Start line set (Hwy 35 & 84)             │
│                                             │
│  Drive to where you want timing to end.     │
│  Stop on the line.                          │
│                                             │
│  Current position:                          │
│  37.4891°N, 122.2234°W                      │
│  Speed: 0.0 km/h ✓                          │
│                                             │
│  ┌─────────────────────────────────────┐    │
│  │      [📍 SET FINISH LINE]           │    │
│  └─────────────────────────────────────┘    │
│                                             │
└─────────────────────────────────────────────┘

         User taps "Set Finish Line"
                   ↓

┌─────────────────────────────────────────────┐
│  🏁 NAME YOUR TRACK                         │
├─────────────────────────────────────────────┤
│                                             │
│  Track name:                                │
│  ┌─────────────────────────────────────┐    │
│  │ Skyline to La Honda               ▼ │    │
│  └─────────────────────────────────────┘    │
│  (Point-to-point · 8.2 km)                  │
│                                             │
│  Line width: [====●====] 30m                │
│                                             │
│  [Cancel]              [Save & Start]       │
│                                             │
└─────────────────────────────────────────────┘
```

### Active Timing Screen

```
┌─────────────────────────────────────────────┐
│  🏁 BELMONT GRAND PRIX                      │
├─────────────────────────────────────────────┤
│   CURRENT        DELTA                      │
│   1:42.34        +0.52s                     │
│                  ████████░░ (slower)        │
│                                             │
│   BEST           LAST                       │
│   1:41.82        1:42.89                    │
│                                             │
│   LAP 7                                     │
│                                             │
├─────────────────────────────────────────────┤
│  [⚙️ Track Settings]  [📋 Load Different]   │
└─────────────────────────────────────────────┘
```

The delta updates in real-time as you drive, showing whether you're ahead or behind your best lap at your current position on track.

### Saved Tracks List

```
┌─────────────────────────────────────────────┐
│  📚 SAVED TRACKS                            │
├─────────────────────────────────────────────┤
│                                             │
│  ┌─────────────────────────────────────┐    │
│  │ 🏁 Belmont Grand Prix               │    │
│  │    Best: 1:42.34 · 12 laps          │    │
│  │    Ralston Ave loop, Belmont     →  │    │
│  └─────────────────────────────────────┘    │
│                                             │
│  ┌─────────────────────────────────────┐    │
│  │ 🏁 Safeway Parking Lot TT           │    │
│  │    Best: 0:31.20 · 8 laps           │    │
│  │    Sunday morning special        →  │    │
│  └─────────────────────────────────────┘    │
│                                             │
│  ┌─────────────────────────────────────┐    │
│  │ 🏁 Carlmont Shopping Center GP      │    │
│  │    Best: 2:05.67 · 3 laps           │    │
│  │    The full perimeter            →  │    │
│  └─────────────────────────────────────┘    │
│                                             │
├─────────────────────────────────────────────┤
│  [+ Create New]           [📥 Import]       │
└─────────────────────────────────────────────┘
```

### Track Settings

```
┌─────────────────────────────────────────────┐
│  ⚙️ BELMONT GRAND PRIX                      │
├─────────────────────────────────────────────┤
│                                             │
│  Name: [Belmont Grand Prix     ]            │
│                                             │
│  Start/Finish Line:                         │
│  37.5202°N, 122.2758°W                      │
│  (Corner of Ralston & El Camino)            │
│  [🔄 Re-set Line]                           │
│                                             │
│  Best Lap: 1:42.34                          │
│  [Clear Best Lap]                           │
│                                             │
│  Total Laps: 12                             │
│                                             │
├─────────────────────────────────────────────┤
│  [📤 Export]  [🗑️ Delete]  [Done]           │
└─────────────────────────────────────────────┘
```

### Auto-Detection (Returning to Known Track)

When GPS position is within ~500m of a saved track's start/finish line:

```
┌─────────────────────────────────────────────┐
│  📍 TRACK DETECTED                          │
├─────────────────────────────────────────────┤
│                                             │
│  You appear to be at:                       │
│                                             │
│  🏁 Belmont Grand Prix                      │
│     Best: 1:42.34                           │
│     12 legendary laps recorded              │
│                                             │
│  [Load This Track]    [Choose Different]    │
│                                             │
└─────────────────────────────────────────────┘
```

## How It Works

### Position-Based Delta Comparison

The predictive delta shown on the timing screen (+0.52s in the mockup above) compares your current lap to your best lap. But how do you compare two laps that might take different racing lines?

**The naive approach (distance-based) doesn't work:**

Compare times based on how far you've traveled, like an odometer:

```
Reference lap: At 1,500m traveled, elapsed time was 45.0s
Current lap:   At 1,500m traveled, elapsed time is 45.3s
Delta: +0.3s
```

The problem: different racing lines cover different distances.

```
    BIRD'S EYE VIEW OF A 90° CORNER

              Entry                           Exit
                │                               │
                │    ╭──────────────────────╮   │
                │   ╱   WIDE LINE (longer)   ╲  │
    ════════════╪══╱═════════════════════════╲═╪════════
                │ ╱    TIGHT LINE (shorter)   ╲│
                │╱  ╭───────────────────╮      ╲
                ▼  ╱                     ╲      ▼
                  ╱                       ╲
                 ╱                         ╲
                │            ↑              │
                │         APEX              │
                │                           │


    DISTANCE-BASED PROBLEM:

    Tight line from start to apex:  1,500m
    Wide line from start to apex:   1,520m (goes wider, covers more ground)

    When your odometer reads "1,500m":
    ┌─────────────────────────────────────────────────┐
    │  Tight line driver: Already AT the apex         │
    │  Wide line driver:  Still 20m BEFORE the apex   │
    └─────────────────────────────────────────────────┘

    Comparing lap times at "1,500m" compares two completely
    different spots on the track. The delta is garbage.
```

**The correct approach (position-based):**

Compare lap times based on WHERE you are (x, y coordinates), not how far you've traveled.

```
    SAME CORNER - POSITION-BASED COMPARISON

              Entry                           Exit
                │                               │
                │    ╭──────────────────────╮   │
                │   ╱   WIDE LINE (longer)   ╲  │
    ════════════╪══╱═════════════════════════╲═╪════════
                │ ╱    TIGHT LINE (shorter)   ╲│
                │╱  ╭───────────────────╮      ╲
                ▼  ╱                     ╲      ▼
                  ╱                       ╲
                 ╱           ★             ╲
                │         APEX              │
                │      (x=234, y=89)        │
                │                           │


    POSITION-BASED SOLUTION:

    Both lines pass through the apex at (x=234, y=89).

    When you reach position (234, 89):
    ┌─────────────────────────────────────────────────┐
    │  Reference lap: Was here at 45.0s               │
    │  Current lap:   You're here at 45.3s            │
    │  Delta: +0.3s (you're 0.3s slower)              │
    └─────────────────────────────────────────────────┘

    Doesn't matter that you took different paths to get here.
    Same physical location = valid comparison.
```

On a 3-mile road course with 15 corners, line variations compound - you might travel 50+ meters more than the reference lap while hitting all the same apexes. Distance-based falls apart; position-based remains accurate.

### Timing Resolution

| Component | Rate | Precision |
|-----------|------|-----------|
| EKF position updates | 200 Hz | 5 ms |
| GPS fixes | 5-10 Hz | 100-200 ms |
| Telemetry timestamp | u32 | 1 ms |

**Achievable precision: milliseconds (hundredths displayed)**

The line crossing detection interpolates between EKF samples. Even though GPS is only 5-10 Hz, the IMU + EKF gives 200 Hz position estimates:

```
Sample at t=0ms:    position = 2m before line
Sample at t=5ms:    position = 1m after line
Interpolated crossing: t ≈ 3.3ms
```

Display shows hundredths like F1 timing: `1:42.34`

Compared to commercial: VBOX at 25 Hz = 40ms between samples. Blackbox at 200 Hz EKF = 5ms between samples - potentially better resolution.

## Technical Implementation

### Data Structures

```rust
enum TrackType {
    Loop,          // Single start/finish line
    PointToPoint,  // Separate start and finish lines
}

struct TimingLine {
    p1: (f64, f64),           // GPS coords (lat, lon)
    p2: (f64, f64),           // GPS coords (lat, lon)
    direction: f32,           // Valid crossing direction (radians)
    width_m: f32,             // Line width in meters
}

struct ReferenceLap {
    samples: Vec<(f32, f32, u32)>,  // (x, y, elapsed_ms) in local coords
    total_time_ms: u32,
}

struct LapTimerState {
    track_type: TrackType,
    start_line: Option<TimingLine>,
    finish_line: Option<TimingLine>,  // None for loops (uses start_line)
    current_lap_start_ms: u32,
    best_lap_ms: Option<u32>,
    last_lap_ms: Option<u32>,
    lap_count: u16,
    reference_lap: Option<ReferenceLap>,
    current_lap_samples: Vec<(f32, f32, u32)>,
    on_outlap: bool,
}
```

### Line Crossing Detection

```rust
fn check_line_crossing(
    prev_pos: (f32, f32),
    curr_pos: (f32, f32),
    velocity: (f32, f32),
    line: &StartFinishLine,
) -> Option<f32> {  // Returns interpolated crossing time offset
    // 1. Convert line GPS coords to local x,y
    // 2. Check if path segment intersects line segment
    // 3. Verify crossing direction (dot product with line normal)
    // 4. Interpolate exact crossing point for sub-sample precision
    // 5. Return time offset from prev sample, or None if no crossing
}
```

### Position-Based Delta Calculation

```rust
fn calculate_delta(
    current_pos: (f32, f32),
    current_elapsed_ms: u32,
    reference: &ReferenceLap,
) -> i32 {
    // Find closest point on reference lap by POSITION (not distance traveled)
    let nearest_idx = reference.samples.iter()
        .enumerate()
        .min_by_key(|(_, (x, y, _))| {
            squared_distance(current_pos, (*x, *y))
        })
        .map(|(i, _)| i)?;

    let reference_elapsed = reference.samples[nearest_idx].2;
    (current_elapsed_ms as i32) - (reference_elapsed as i32)
}
```

### API Endpoints

```
GET  /api/laptimer              - Current state (timing, delta, lap count)
POST /api/laptimer/setstart     - Set start line at current position
POST /api/laptimer/setfinish    - Set finish line at current position (point-to-point only)
POST /api/laptimer/loadtrack    - Load track config from client-provided data
GET  /api/laptimer/position     - Current GPS for track detection
```

Note: Track storage is entirely client-side (browser). ESP32 only needs to know the current active lines and do the timing math.

## Storage

All track data stored in browser localStorage (phone remembers tracks):

```javascript
// localStorage key: "blackbox_tracks"
{
  "tracks": [
    {
      // Loop track example
      "id": "a1b2c3",
      "name": "Belmont Grand Prix",
      "type": "loop",
      "start_line": {
        "p1": { "lat": 37.5202, "lon": -122.2758 },
        "p2": { "lat": 37.5204, "lon": -122.2755 },
        "direction": 1.57
      },
      "finish_line": null,  // Same as start for loops
      "width_m": 30,
      "best_lap_ms": 102340,
      "total_laps": 12,
      "created": "2024-01-15T10:30:00Z",
      "last_used": "2024-01-20T14:22:00Z"
    },
    {
      // Point-to-point example
      "id": "d4e5f6",
      "name": "Skyline to La Honda",
      "type": "point-to-point",
      "start_line": {
        "p1": { "lat": 37.5202, "lon": -122.2758 },
        "p2": { "lat": 37.5204, "lon": -122.2755 },
        "direction": 1.57
      },
      "finish_line": {
        "p1": { "lat": 37.4891, "lon": -122.2234 },
        "p2": { "lat": 37.4893, "lon": -122.2231 },
        "direction": 2.35
      },
      "width_m": 30,
      "best_lap_ms": 487230,
      "total_laps": 5,
      "created": "2024-01-18T09:15:00Z",
      "last_used": "2024-01-19T16:45:00Z"
    }
  ],
  "active_track_id": "a1b2c3"
}
```

### Export/Import (File Download)

**Export:** User taps "Export" → browser downloads `belmont-grand-prix.json`:
```json
{
  "name": "Belmont Grand Prix",
  "type": "loop",
  "start_line": {
    "p1": { "lat": 37.5202, "lon": -122.2758 },
    "p2": { "lat": 37.5204, "lon": -122.2755 },
    "direction": 1.57
  },
  "width_m": 30,
  "notes": "Start at Ralston & El Camino, go clockwise around the block. Watch for the speed bump on Fifth Ave."
}
```

**Import:** User taps "Import" → file picker opens → select JSON → track added to list.

This makes sharing tracks easy - text a friend the JSON file, they import it, instant shared track definition.

## Edge Cases

1. **GPS not locked**: Disable lap timer, show warning
2. **Crossing line backwards**: Ignore (check velocity direction)
3. **Shortcuts / wrong way**: Debounce with minimum lap time (10s default)
4. **Outlap**: First crossing starts timer, doesn't record a lap
5. **Inlap**: Partial lap discarded on session end
6. **Position drift at low speed**: Only detect crossings above minimum speed (5 km/h)
7. **Multiple phones**: Each phone has its own track database - that's fine

## Testing

1. **Unit tests**: Line crossing math, delta calculation
2. **Simulation**: Feed recorded GPS track, verify lap detection
3. **Parking lot**: Drive figure-8, test line crossing both directions
4. **Real test**: Belmont Grand Prix world record attempt

## Labels

`enhancement` `feature-request` `dashboard` `lap-timing`

---

*This feature would transform blackbox from a $50 telemetry logger into a $50 lap timer + telemetry logger, competing with $700-1,500 commercial solutions. Plus you can finally prove to your friends that your Ralston Ave line is 0.3 seconds faster than theirs.*
