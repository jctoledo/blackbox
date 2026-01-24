# Lap Timer Implementation Plan

## Overview

Real-time lap timing with predictive delta display for the blackbox dashboard. Comparable to commercial solutions like VBOX LapTimer ($1,160) and AiM Solo 2 DL ($700).

**Key Features:**
- Millisecond-precision lap timing
- Predictive delta (current vs best lap by position)
- Track management (save/load/auto-detect)
- Loop and Point-to-Point track types
- Track recording by driving

## Architecture

### Hybrid Design
- **ESP32 (firmware)**: Line crossing detection at 200Hz for sub-sample precision
- **Browser (dashboard)**: Storage (IndexedDB), delta calculation, track management, rich UI

### Data Flow
```
ESP32:
  Position (x,y) @ 200Hz → Line Crossing Detection → lap_time_ms, lap_count, flags
                                                            ↓
Dashboard:                                           Telemetry (74 bytes)
  IndexedDB ← Track Management                              ↓
       ↓                                           Decode + Display
  Reference Lap → Position-Based Delta → Live Delta Display
```

## Storage Strategy (IndexedDB)

**Why IndexedDB:**
- 50MB+ capacity (vs localStorage ~2.5MB effective)
- Binary blob support for reference laps
- Structured queries for track lookup
- Persists across sessions

**Schema:**
```javascript
// tracks store
{
  id: "uuid",
  name: "Circuit of the Americas",
  type: "loop" | "point_to_point",
  created: timestamp,
  startLine: { p1: [x,y], p2: [x,y], direction: radians },
  finishLine: { p1: [x,y], p2: [x,y], direction: radians } | null,  // null for loops
  bounds: { minX, maxX, minY, maxY },  // for auto-detection
  path: [[x,y], ...],  // recorded path (optional, for visualization)
}

// reference_laps store
{
  id: "uuid",
  trackId: "track-uuid",
  lapTime: 125432,  // ms
  created: timestamp,
  isBest: boolean,
  samples: ArrayBuffer,  // packed [timestamp_ms:u32, x:f32, y:f32] @ ~25Hz
}

// sessions store (optional, for session history)
{
  id: "uuid",
  trackId: "track-uuid",
  date: timestamp,
  laps: [{ lapTime, delta }, ...]
}
```

**Storage Estimates:**
- Track metadata: ~500 bytes each
- Reference lap: 40-200KB each (3min lap @ 25Hz = 4500 samples × 12 bytes = 54KB)
- 100 tracks + 500 reference laps = ~50MB (well within IndexedDB limits)

## Test Data (data/sample-tracks/)

The `austin.csv` and `austin_raceline.csv` files provide:
- **austin.csv**: 1104 centerline points with track widths (for validation)
- **austin_raceline.csv**: 1086 raceline points (simulates a driven path)

**Testing strategy:**
1. Load raceline as simulated telemetry (200Hz replay)
2. Test line crossing detection geometry
3. Validate lap completion detection
4. Test position-based delta with synthetic "slower" laps
5. Validate track recording captures similar shape

**Test harness** (Python for offline, Rust for unit tests):
- Replay sample data at configurable speed
- Inject timing lines at known positions
- Verify crossing detection, lap times, delta accuracy

---

## Phase 1: Core Timing Engine (Firmware)

### New Module: `framework/src/lap_timer.rs`

```rust
//! Lap timer with line crossing detection
//!
//! Detects when vehicle crosses timing lines and tracks lap times.
//! Runs at 200Hz for sub-sample precision crossing detection.

/// A timing line defined by two endpoints
#[derive(Clone, Debug)]
pub struct TimingLine {
    pub p1: (f32, f32),      // First endpoint (local meters)
    pub p2: (f32, f32),      // Second endpoint (local meters)
    pub direction: f32,       // Valid crossing direction (radians, 0 = +X)
}

/// Track configuration
#[derive(Clone, Debug)]
pub enum TrackType {
    /// Single start/finish line (circuit racing)
    Loop { line: TimingLine },
    /// Separate start and finish lines (hill climb, rally stage)
    PointToPoint { start: TimingLine, finish: TimingLine },
}

/// Lap timer state machine
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LapTimerState {
    /// No track configured
    Idle,
    /// Track configured, waiting for first line crossing
    Armed,
    /// Timing active, counting lap time
    Timing,
}

/// Crossing flags for telemetry (1 byte)
pub mod LapFlags {
    pub const NONE: u8 = 0;
    pub const CROSSED_START: u8 = 1 << 0;    // Crossed start line this frame
    pub const CROSSED_FINISH: u8 = 1 << 1;   // Crossed finish line this frame
    pub const NEW_LAP: u8 = 1 << 2;          // New lap completed this frame
    pub const NEW_BEST: u8 = 1 << 3;         // New best lap this frame
    pub const INVALID_LAP: u8 = 1 << 4;      // Lap was invalid (too short, wrong direction)
}

/// Main lap timer struct
pub struct LapTimer {
    state: LapTimerState,
    track: Option<TrackType>,

    // Timing
    lap_start_ms: u32,
    current_lap_ms: u32,
    lap_count: u16,
    best_lap_ms: Option<u32>,
    last_lap_ms: Option<u32>,

    // Crossing detection
    prev_pos: Option<(f32, f32)>,
    frame_flags: u8,
    last_crossing_ms: u32,

    // Thresholds
    crossing_debounce_ms: u32,  // 500ms between crossings
    min_lap_time_ms: u32,       // 10000ms minimum valid lap
    direction_tolerance: f32,    // ±90° from defined direction
}

impl LapTimer {
    pub fn new() -> Self;

    /// Configure for loop track (single timing line)
    pub fn configure_loop(&mut self, line: TimingLine);

    /// Configure for point-to-point (separate start/finish)
    pub fn configure_point_to_point(&mut self, start: TimingLine, finish: TimingLine);

    /// Clear configuration
    pub fn clear(&mut self);

    /// Update with new position. Returns flags for this frame.
    /// Call at 200Hz from main loop.
    pub fn update(
        &mut self,
        pos: (f32, f32),
        velocity: (f32, f32),  // For direction validation
        timestamp_ms: u32,
        speed_mps: f32,
    ) -> u8;  // Returns LapFlags

    // Getters for telemetry packet
    pub fn current_lap_ms(&self) -> u32;
    pub fn lap_count(&self) -> u16;
    pub fn best_lap_ms(&self) -> Option<u32>;
    pub fn last_lap_ms(&self) -> Option<u32>;
    pub fn state(&self) -> LapTimerState;
    pub fn take_flags(&mut self) -> u8;  // Returns and clears frame flags
}

// Geometry helpers (pure functions, easy to test)

/// Line segment intersection test
/// Returns Some(t) where t is the interpolation factor on segment p1-p2
/// if segments intersect, None otherwise
fn line_segment_intersection(
    p1: (f32, f32), p2: (f32, f32),  // Movement segment
    p3: (f32, f32), p4: (f32, f32),  // Timing line
) -> Option<f32>;

/// Normalize angle to [-π, π]
fn wrap_angle(angle: f32) -> f32;

/// Check if crossing direction is within tolerance of expected direction
fn direction_valid(
    velocity: (f32, f32),
    expected_dir: f32,
    tolerance: f32,
) -> bool;
```

### Telemetry Extension: 67 → 74 bytes

Add to `binary_telemetry.rs`:

```rust
// After existing fields (offset 64):
pub lap_time_ms: u32,    // Current lap time in ms (0 if not timing)
pub lap_count: u16,      // Total completed laps (wraps at 65535)
pub lap_flags: u8,       // LapFlags bitfield

// New total: 67 + 7 = 74 bytes
// Checksum recalculated over first 72 bytes
```

**Decoder update:**
```python
# Python struct format
TELEMETRY_FORMAT = '<HI 12f B 2f B I H B H'  # 74 bytes total
```

### API Endpoint: `/api/laptimer/configure`

```rust
// POST /api/laptimer/configure
// Content-Type: application/json

// Configure loop track:
{
    "type": "loop",
    "line": {
        "p1": [x, y],
        "p2": [x, y],
        "direction": radians  // or degrees, TBD
    }
}

// Configure point-to-point:
{
    "type": "point_to_point",
    "start": { "p1": [...], "p2": [...], "direction": ... },
    "finish": { "p1": [...], "p2": [...], "direction": ... }
}

// Clear configuration:
{
    "type": "clear"
}
```

### Unit Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_line_segment_intersection_basic() {
        // Perpendicular crossing
        let t = line_segment_intersection(
            (0.0, -1.0), (0.0, 1.0),   // Movement: vertical up
            (-1.0, 0.0), (1.0, 0.0),   // Line: horizontal
        );
        assert!(t.is_some());
        assert!((t.unwrap() - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_line_segment_intersection_no_cross() {
        // Parallel, no intersection
        let t = line_segment_intersection(
            (0.0, 0.0), (1.0, 0.0),
            (0.0, 1.0), (1.0, 1.0),
        );
        assert!(t.is_none());
    }

    #[test]
    fn test_line_segment_intersection_endpoint() {
        // Crosses exactly at endpoint
        let t = line_segment_intersection(
            (0.0, 0.0), (1.0, 0.0),
            (0.5, 0.0), (0.5, 1.0),
        );
        assert!(t.is_some());
    }

    #[test]
    fn test_direction_valid() {
        // Moving right (+X), expected direction 0 (East)
        assert!(direction_valid((1.0, 0.0), 0.0, PI/2.0));

        // Moving left, should fail direction check for East
        assert!(!direction_valid((-1.0, 0.0), 0.0, PI/2.0));

        // Moving diagonally, within ±90°
        assert!(direction_valid((1.0, 1.0), 0.0, PI/2.0));
    }

    #[test]
    fn test_lap_timer_loop_basic() {
        let mut timer = LapTimer::new();
        timer.configure_loop(TimingLine {
            p1: (-10.0, 0.0),
            p2: (10.0, 0.0),
            direction: PI / 2.0,  // Crossing northward
        });

        // Start below line
        timer.update((0.0, -5.0), (0.0, 10.0), 0, 10.0);
        assert_eq!(timer.state(), LapTimerState::Armed);

        // Cross line
        let flags = timer.update((0.0, 5.0), (0.0, 10.0), 100, 10.0);
        assert!(flags & LapFlags::CROSSED_START != 0);
        assert_eq!(timer.state(), LapTimerState::Timing);

        // Complete lap (cross again after min time)
        let flags = timer.update((0.0, -5.0), (0.0, -10.0), 15000, 10.0);
        // Moving wrong direction, should not count
        assert!(flags & LapFlags::NEW_LAP == 0);

        // Cross correctly
        let flags = timer.update((0.0, 5.0), (0.0, 10.0), 60000, 10.0);
        assert!(flags & LapFlags::NEW_LAP != 0);
        assert_eq!(timer.lap_count(), 1);
    }

    #[test]
    fn test_austin_raceline_lap() {
        // Load sample data and simulate a lap
        let raceline = load_test_raceline("austin_raceline.csv");
        let mut timer = LapTimer::new();

        // Set timing line near start/finish area
        timer.configure_loop(TimingLine {
            p1: (-10.0, 0.0),
            p2: (10.0, 0.0),
            direction: std::f32::consts::FRAC_PI_2,
        });

        // Simulate driving at 25Hz (40ms between samples)
        for i in 1..raceline.len() {
            let dt_ms = 40;
            let pos = raceline[i];
            let prev = raceline[i-1];
            let vel = ((pos.0 - prev.0) / 0.040, (pos.1 - prev.1) / 0.040);
            let speed = (vel.0*vel.0 + vel.1*vel.1).sqrt();

            timer.update(pos, vel, (i as u32) * dt_ms, speed);
        }

        // Should have completed at least one lap
        assert!(timer.lap_count() >= 1);
    }
}
```

---

## Phase 2: Dashboard UI - Basic Lap Timer Display

### HTML Structure

Add to dashboard within main telemetry page (not a separate tab):

```html
<!-- Lap Timer Section -->
<div id="lap-timer-section" class="hidden">
    <div id="lap-display">
        <div id="current-lap-time">0:00.000</div>
        <div id="lap-count">Lap 0</div>
        <div id="delta-display" class="delta-neutral">+0.000</div>
    </div>

    <div id="lap-history">
        <div id="best-lap">Best: --:--</div>
        <div id="last-lap">Last: --:--</div>
    </div>
</div>

<!-- Lap Timer Controls (in settings area) -->
<div id="lap-timer-controls">
    <button id="btn-track-manager">Track Manager</button>
    <button id="btn-record-track">Record Track</button>
</div>

<!-- Track Manager Modal -->
<div id="track-manager-modal" class="modal hidden">
    <div class="modal-content">
        <h2>Track Manager</h2>
        <div id="track-list"></div>
        <button id="btn-new-track">New Track</button>
        <button id="btn-close-modal">Close</button>
    </div>
</div>
```

### CSS

```css
#lap-timer-section {
    background: rgba(0, 0, 0, 0.8);
    border-radius: 8px;
    padding: 16px;
    margin: 8px;
}

#current-lap-time {
    font-size: 48px;
    font-family: monospace;
    font-weight: bold;
    color: white;
}

#delta-display {
    font-size: 36px;
    font-family: monospace;
    font-weight: bold;
}

.delta-faster { color: #00ff00; }  /* Green - ahead */
.delta-slower { color: #ff0000; }  /* Red - behind */
.delta-neutral { color: #888888; } /* Gray - no reference */

#lap-history {
    display: flex;
    gap: 24px;
    font-size: 18px;
    color: #aaa;
}
```

### JavaScript - Telemetry Decoding

```javascript
// Extended telemetry decoder
function decodeTelemetry(base64) {
    const bytes = atob(base64);
    const view = new DataView(new ArrayBuffer(74));
    for (let i = 0; i < 74; i++) {
        view.setUint8(i, bytes.charCodeAt(i));
    }

    // ... existing fields ...

    // New lap timer fields (offset 65)
    const lapTimeMs = view.getUint32(65, true);
    const lapCount = view.getUint16(69, true);
    const lapFlags = view.getUint8(71);

    return {
        // ... existing ...
        lapTimeMs,
        lapCount,
        lapFlags,
    };
}

// Lap flags constants
const LAP_FLAGS = {
    CROSSED_START: 1,
    CROSSED_FINISH: 2,
    NEW_LAP: 4,
    NEW_BEST: 8,
    INVALID_LAP: 16,
};
```

---

## Phase 3: Track Recording ("Record by Driving")

### User Flow

1. User taps "Record Track" button
2. Dashboard enters recording mode, shows instructions
3. User drives the circuit (or point-to-point route)
4. Dashboard collects GPS positions + detects loop closure
5. User confirms track, optionally adjusts timing line
6. Track saved to IndexedDB

### Recording Logic (JavaScript)

```javascript
class TrackRecorder {
    constructor() {
        this.recording = false;
        this.path = [];
        this.startPos = null;
        this.minLoopDistance = 50;  // meters, proximity to detect loop
    }

    start() {
        this.recording = true;
        this.path = [];
        this.startPos = null;
    }

    addSample(x, y, timestamp) {
        if (!this.recording) return;

        const sample = { x, y, t: timestamp };

        if (this.path.length === 0) {
            this.startPos = { x, y };
            this.path.push(sample);
            return;
        }

        // Downsample: only add if moved >2m from last point
        const last = this.path[this.path.length - 1];
        const dist = Math.sqrt((x - last.x)**2 + (y - last.y)**2);
        if (dist < 2.0) return;

        this.path.push(sample);

        // Check for loop closure (if enough distance traveled)
        const totalDist = this.calculateTotalDistance();
        if (totalDist > 500) {  // At least 500m before checking
            const distToStart = Math.sqrt(
                (x - this.startPos.x)**2 +
                (y - this.startPos.y)**2
            );

            if (distToStart < this.minLoopDistance) {
                this.onLoopDetected();
            }
        }
    }

    onLoopDetected() {
        // Notify user: "Loop detected! Tap to confirm or keep driving"
        showNotification("Loop detected! Tap to save track.");
    }

    finish(trackName, trackType) {
        this.recording = false;

        // Calculate timing line from start position
        const timingLine = this.calculateTimingLine();

        // Calculate bounds for auto-detection
        const bounds = this.calculateBounds();

        return {
            name: trackName,
            type: trackType,
            startLine: timingLine,
            finishLine: trackType === 'point_to_point' ? this.calculateFinishLine() : null,
            bounds,
            path: this.path.map(p => [p.x, p.y]),
        };
    }

    calculateTimingLine() {
        // Use first segment direction, perpendicular line through start
        const p0 = this.path[0];
        const p1 = this.path[Math.min(5, this.path.length - 1)];

        // Direction of travel
        const dx = p1.x - p0.x;
        const dy = p1.y - p0.y;
        const len = Math.sqrt(dx*dx + dy*dy);

        // Perpendicular direction (for timing line)
        const perpX = -dy / len;
        const perpY = dx / len;

        // Line width: 15 meters each side
        const width = 15;

        return {
            p1: [p0.x + perpX * width, p0.y + perpY * width],
            p2: [p0.x - perpX * width, p0.y - perpY * width],
            direction: Math.atan2(dy, dx),
        };
    }

    calculateBounds() {
        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;

        for (const p of this.path) {
            minX = Math.min(minX, p.x);
            maxX = Math.max(maxX, p.x);
            minY = Math.min(minY, p.y);
            maxY = Math.max(maxY, p.y);
        }

        return { minX, maxX, minY, maxY };
    }
}
```

### Recording UI

```html
<div id="recording-overlay" class="hidden">
    <div class="recording-status">
        <div class="recording-indicator"></div>
        <span>Recording Track...</span>
    </div>
    <div id="recording-stats">
        <div>Distance: <span id="rec-distance">0</span> m</div>
        <div>Points: <span id="rec-points">0</span></div>
    </div>
    <button id="btn-cancel-recording">Cancel</button>
    <button id="btn-finish-recording" class="hidden">Finish Track</button>
</div>
```

---

## Phase 4: Track Management & Auto-Detection

### IndexedDB Operations

```javascript
class TrackDatabase {
    constructor() {
        this.db = null;
    }

    async init() {
        return new Promise((resolve, reject) => {
            const request = indexedDB.open('blackbox-tracks', 1);

            request.onerror = () => reject(request.error);
            request.onsuccess = () => {
                this.db = request.result;
                resolve();
            };

            request.onupgradeneeded = (event) => {
                const db = event.target.result;

                // Tracks store
                const tracks = db.createObjectStore('tracks', { keyPath: 'id' });
                tracks.createIndex('name', 'name', { unique: false });

                // Reference laps store
                const laps = db.createObjectStore('reference_laps', { keyPath: 'id' });
                laps.createIndex('trackId', 'trackId', { unique: false });
                laps.createIndex('isBest', ['trackId', 'isBest'], { unique: false });
            };
        });
    }

    async saveTrack(track) {
        track.id = track.id || crypto.randomUUID();
        track.created = track.created || Date.now();

        return new Promise((resolve, reject) => {
            const tx = this.db.transaction('tracks', 'readwrite');
            tx.objectStore('tracks').put(track);
            tx.oncomplete = () => resolve(track);
            tx.onerror = () => reject(tx.error);
        });
    }

    async getAllTracks() {
        return new Promise((resolve, reject) => {
            const tx = this.db.transaction('tracks', 'readonly');
            const request = tx.objectStore('tracks').getAll();
            request.onsuccess = () => resolve(request.result);
            request.onerror = () => reject(request.error);
        });
    }

    async findTrackByPosition(x, y) {
        const tracks = await this.getAllTracks();

        // Find tracks where current position is within bounds
        const candidates = tracks.filter(track => {
            const b = track.bounds;
            const margin = 100;  // 100m margin
            return x >= b.minX - margin && x <= b.maxX + margin &&
                   y >= b.minY - margin && y <= b.maxY + margin;
        });

        if (candidates.length === 1) {
            return candidates[0];
        } else if (candidates.length > 1) {
            // Multiple matches - find closest by center distance
            return candidates.reduce((best, track) => {
                const bx = (track.bounds.minX + track.bounds.maxX) / 2;
                const by = (track.bounds.minY + track.bounds.maxY) / 2;
                const dist = Math.sqrt((x - bx)**2 + (y - by)**2);

                if (!best || dist < best.dist) {
                    return { track, dist };
                }
                return best;
            }, null)?.track;
        }

        return null;
    }

    async saveReferenceLap(trackId, lapTimeMs, samples) {
        const lap = {
            id: crypto.randomUUID(),
            trackId,
            lapTime: lapTimeMs,
            created: Date.now(),
            isBest: false,
            samples,  // ArrayBuffer
        };

        // Check if this is new best
        const existingBest = await this.getBestLap(trackId);
        if (!existingBest || lapTimeMs < existingBest.lapTime) {
            lap.isBest = true;
            if (existingBest) {
                existingBest.isBest = false;
                await this.updateLap(existingBest);
            }
        }

        return new Promise((resolve, reject) => {
            const tx = this.db.transaction('reference_laps', 'readwrite');
            tx.objectStore('reference_laps').put(lap);
            tx.oncomplete = () => resolve(lap);
            tx.onerror = () => reject(tx.error);
        });
    }

    async getBestLap(trackId) {
        return new Promise((resolve, reject) => {
            const tx = this.db.transaction('reference_laps', 'readonly');
            const index = tx.objectStore('reference_laps').index('isBest');
            const request = index.get([trackId, true]);
            request.onsuccess = () => resolve(request.result);
            request.onerror = () => reject(request.error);
        });
    }
}
```

### Auto-Detection Flow

```javascript
class LapTimerManager {
    constructor(trackDb) {
        this.trackDb = trackDb;
        this.currentTrack = null;
        this.autoDetectEnabled = true;
    }

    async onPositionUpdate(x, y) {
        if (!this.currentTrack && this.autoDetectEnabled) {
            const track = await this.trackDb.findTrackByPosition(x, y);
            if (track) {
                this.activateTrack(track);
            }
        }
    }

    async activateTrack(track) {
        this.currentTrack = track;

        // Send configuration to ESP32
        await fetch('/api/laptimer/configure', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                type: track.type,
                line: track.startLine,
                finish: track.finishLine,
            }),
        });

        // Load best lap for delta comparison
        this.bestLap = await this.trackDb.getBestLap(track.id);

        showNotification(`Track: ${track.name}`);
    }
}
```

---

## Phase 5: Predictive Delta Calculation

### Position-Based Delta Algorithm

```javascript
class DeltaCalculator {
    constructor() {
        this.referenceLap = null;  // Array of {t, x, y}
        this.spatialIndex = null;  // For fast nearest-neighbor lookup
    }

    setReferenceLap(samples) {
        // Decode samples from ArrayBuffer
        this.referenceLap = this.decodeSamples(samples);
        this.buildSpatialIndex();
    }

    decodeSamples(buffer) {
        const view = new DataView(buffer);
        const samples = [];

        for (let i = 0; i < buffer.byteLength; i += 12) {
            samples.push({
                t: view.getUint32(i, true),
                x: view.getFloat32(i + 4, true),
                y: view.getFloat32(i + 8, true),
            });
        }

        return samples;
    }

    buildSpatialIndex() {
        // Simple grid-based spatial index
        // For a 5km track, 10m grid = 500 cells max
        this.grid = new Map();
        const cellSize = 10;  // meters

        for (let i = 0; i < this.referenceLap.length; i++) {
            const { x, y } = this.referenceLap[i];
            const cellX = Math.floor(x / cellSize);
            const cellY = Math.floor(y / cellSize);
            const key = `${cellX},${cellY}`;

            if (!this.grid.has(key)) {
                this.grid.set(key, []);
            }
            this.grid.get(key).push(i);
        }
    }

    calculateDelta(currentX, currentY, currentLapTimeMs) {
        if (!this.referenceLap) return null;

        // Find nearest point in reference lap
        const nearest = this.findNearestPoint(currentX, currentY);
        if (!nearest) return null;

        // Delta = current time - reference time at same position
        const deltaMs = currentLapTimeMs - nearest.t;

        return {
            deltaMs,
            distanceToRef: nearest.distance,
            confidence: nearest.distance < 20 ? 'high' : 'low',
        };
    }

    findNearestPoint(x, y) {
        const cellSize = 10;
        const cellX = Math.floor(x / cellSize);
        const cellY = Math.floor(y / cellSize);

        let bestIdx = -1;
        let bestDist = Infinity;

        // Search current cell and neighbors
        for (let dx = -1; dx <= 1; dx++) {
            for (let dy = -1; dy <= 1; dy++) {
                const key = `${cellX + dx},${cellY + dy}`;
                const indices = this.grid.get(key) || [];

                for (const i of indices) {
                    const ref = this.referenceLap[i];
                    const dist = Math.sqrt((x - ref.x)**2 + (y - ref.y)**2);

                    if (dist < bestDist) {
                        bestDist = dist;
                        bestIdx = i;
                    }
                }
            }
        }

        if (bestIdx === -1) return null;

        return {
            t: this.referenceLap[bestIdx].t,
            distance: bestDist,
        };
    }
}
```

### Delta Display

```javascript
function updateDeltaDisplay(deltaMs, confidence) {
    const deltaEl = document.getElementById('delta-display');

    if (deltaMs === null || confidence === 'low') {
        deltaEl.textContent = '--';
        deltaEl.className = 'delta-neutral';
        return;
    }

    const sign = deltaMs > 0 ? '+' : '';
    const seconds = Math.abs(deltaMs) / 1000;

    deltaEl.textContent = `${sign}${seconds.toFixed(3)}`;
    deltaEl.className = deltaMs < 0 ? 'delta-faster' : 'delta-slower';
}
```

---

## Phase 6: Polish & Edge Cases

### Reference Lap Recording

During active timing, record samples for potential reference lap:

```javascript
class LapRecorder {
    constructor() {
        this.samples = [];
        this.recording = false;
    }

    startLap() {
        this.samples = [];
        this.recording = true;
    }

    addSample(x, y, lapTimeMs) {
        if (!this.recording) return;

        // Record at ~25Hz (every ~40ms)
        const lastT = this.samples.length > 0
            ? this.samples[this.samples.length - 1].t
            : -40;

        if (lapTimeMs - lastT >= 40) {
            this.samples.push({ t: lapTimeMs, x, y });
        }
    }

    finishLap() {
        this.recording = false;
        return this.encodeSamples();
    }

    encodeSamples() {
        const buffer = new ArrayBuffer(this.samples.length * 12);
        const view = new DataView(buffer);

        for (let i = 0; i < this.samples.length; i++) {
            view.setUint32(i * 12, this.samples[i].t, true);
            view.setFloat32(i * 12 + 4, this.samples[i].x, true);
            view.setFloat32(i * 12 + 8, this.samples[i].y, true);
        }

        return buffer;
    }
}
```

### Edge Cases to Handle

1. **GPS warmup**: Don't activate lap timer until GPS is warmed up
2. **Track boundary exit**: Warn if vehicle leaves track bounds significantly
3. **Power loss**: Reference lap recording lost (acceptable for MVP)
4. **Multiple crossings**: Debounce handles quick re-crossings
5. **Reverse direction**: Direction validation prevents backwards laps
6. **Very long laps**: Support laps up to ~70 minutes (u32 ms limit)
7. **Track overlap**: Use center distance for disambiguation

### UI Polish

1. **Lap time formatting**: `MM:SS.mmm` for lap times
2. **Audio cues**: Optional beep on line crossing (future)
3. **Large delta display**: Easy to read at a glance while driving
4. **Color coding**: Green (faster), Red (slower), White (no reference)
5. **Best lap highlight**: Visual indicator when new best is set

---

## Implementation Order

### Phase 1: Core Engine (ESP32)
1. Create `framework/src/lap_timer.rs` with geometry math
2. Add unit tests for line crossing and direction validation
3. Extend telemetry packet to 74 bytes
4. Add `/api/laptimer/configure` endpoint
5. Integrate into main loop

### Phase 2: Basic Display
1. Add lap timer section to dashboard HTML
2. Decode new telemetry fields
3. Display current lap time, lap count
4. Show best/last lap

### Phase 3: Track Recording
1. Implement TrackRecorder class
2. Add recording UI overlay
3. Loop detection with notification
4. Save to IndexedDB

### Phase 4: Track Management
1. Implement TrackDatabase class
2. Track list UI
3. Auto-detection logic
4. Track selection/activation

### Phase 5: Predictive Delta
1. Reference lap recording during timing
2. DeltaCalculator with spatial index
3. Position-based delta display
4. Best lap auto-save

### Phase 6: Polish
1. Edge case handling
2. UI refinements
3. Test with sample data
4. Documentation

---

## Testing Strategy

### Unit Tests (Rust)
- Line segment intersection math
- Direction validation
- Lap timer state machine
- Telemetry packet encoding

### Integration Tests (Python)
- Replay `austin_raceline.csv` through lap timer
- Verify lap completion detection
- Test timing line at various positions
- Delta calculation accuracy

### Manual Testing
1. Configure timing line via dashboard
2. "Drive" recorded data via TCP mock
3. Verify lap times display correctly
4. Test track recording flow
5. Test auto-detection

---

## File Changes Summary

### New Files
- `framework/src/lap_timer.rs` - Core lap timer logic

### Modified Files
- `sensors/blackbox/src/binary_telemetry.rs` - Extended packet (67 → 74 bytes)
- `sensors/blackbox/src/websocket_server.rs` - New endpoint, shared state for lap timer
- `sensors/blackbox/src/main.rs` - Integrate lap timer into main loop
- `framework/src/lib.rs` - Export lap_timer module
- Dashboard HTML/CSS/JS (embedded in websocket_server.rs)

### Test Files
- `framework/src/lap_timer.rs` - Inline unit tests
- `tools/python/test_lap_timer.py` - Integration tests with sample data
