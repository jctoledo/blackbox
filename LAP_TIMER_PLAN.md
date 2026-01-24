# Lap Timer Implementation Plan

## Overview

Real-time lap timing with predictive delta display for the Blackbox dashboard. Comparable to commercial solutions like VBOX LapTimer ($1,160) and AiM Solo 2 DL ($700).

**Target Use Cases:**
- Driving around a neighborhood block to create a "circuit"
- Track days at real circuits (autocross, road course)
- Point-to-point timing (hill climbs, rally stages)
- Personal best tracking across sessions

**Key Features:**
- Millisecond-precision lap timing
- Predictive delta (current vs best lap by position)
- Track management (save/load/auto-detect)
- Loop and Point-to-Point track types
- Track recording by driving
- Session history and personal bests

---

## Architecture

### Hybrid Design
- **ESP32 (firmware)**: Line crossing detection at telemetry rate for precision timing
- **Browser (dashboard)**: Storage (IndexedDB), delta calculation, track management, rich UI

### Data Flow
```
ESP32 Firmware:
  Position (x,y) from EKF → LapTimer.update() → lap_time_ms, lap_count, flags
                                                        ↓
                                                Telemetry Packet (74 bytes)
                                                        ↓
Dashboard (Browser):                              HTTP Polling (~30 Hz)
  IndexedDB ← Track Management                          ↓
       ↓                                       Decode + Display
  Reference Lap → Position-Based Delta → Live Delta Display
```

### Dashboard Synchronization Strategy

**CRITICAL**: `tools/dashboard-dev/index.html` and the production dashboard embedded in `sensors/blackbox/src/websocket_server.rs` MUST remain synchronized.

**Development Workflow:**
1. All UI changes are made FIRST in `dashboard-dev/index.html`
2. Test locally using simulation or CSV replay
3. Once working, port changes to `websocket_server.rs` embedded HTML
4. Both dashboards should have identical look, feel, and functionality
5. Only difference: dashboard-dev uses simulated data, production uses real telemetry

**Shared Components:**
- CSS styles (copy verbatim)
- HTML structure (copy verbatim)
- JavaScript logic (adapt for data source differences)
- IndexedDB schema and operations (identical)

---

## Storage Strategy (IndexedDB)

### Why IndexedDB
- 50MB+ capacity (vs localStorage ~2.5MB effective)
- Binary blob support for reference laps
- Structured queries for track lookup
- Persists across sessions and page refreshes

### Database Schema

**Database Name:** `blackbox-tracks` (version 1)

```javascript
// ============================================================
// TRACKS STORE - Saved track configurations
// ============================================================
{
  id: "uuid",                    // crypto.randomUUID()
  name: "My Block Loop",         // User-provided name
  type: "loop" | "point_to_point",
  created: timestamp,            // Date.now()
  modified: timestamp,           // Last edit time

  // Timing line(s) - in local meters (EKF coordinate system)
  startLine: {
    p1: [x, y],                  // First endpoint
    p2: [x, y],                  // Second endpoint
    direction: radians           // Valid crossing direction (0 = +X, π/2 = +Y)
  },
  finishLine: {                  // null for loop tracks
    p1: [x, y],
    p2: [x, y],
    direction: radians
  } | null,

  // GPS reference point (for coordinate transformation)
  gpsOrigin: {
    lat: degrees,                // Reference latitude
    lon: degrees                 // Reference longitude
  },

  // Bounding box for auto-detection (local meters)
  bounds: {
    minX: meters,
    maxX: meters,
    minY: meters,
    maxY: meters
  },

  // Recorded path (optional, for visualization)
  path: [[x, y], ...] | null,    // Downsampled to ~100-500 points

  // Statistics
  bestLapMs: number | null,      // Personal best on this track
  lapCount: number               // Total laps completed
}

// ============================================================
// REFERENCE_LAPS STORE - Position samples for delta calculation
// ============================================================
{
  id: "uuid",
  trackId: "track-uuid",         // Foreign key to tracks store
  lapTimeMs: number,             // Total lap time
  created: timestamp,
  isBest: boolean,               // Currently the reference lap?

  // Packed binary samples for efficiency
  // Format: [timestamp_ms:u32, x:f32, y:f32] × N samples
  // At 25Hz for a 2-minute lap = 3000 samples × 12 bytes = 36KB
  samples: ArrayBuffer
}

// ============================================================
// SESSIONS STORE - Session history
// ============================================================
{
  id: "uuid",
  trackId: "track-uuid",
  date: timestamp,               // Session start time

  laps: [
    {
      lapTimeMs: number,
      deltaMs: number | null,    // vs best at time of lap
      isValid: boolean           // false if cut short, wrong direction, etc.
    },
    ...
  ],

  bestLapMs: number | null,      // Best lap this session
  totalLaps: number
}
```

### Storage Estimates
- Track metadata: ~500 bytes each (with 500-point path: ~4KB)
- Reference lap: 40-200KB each (3min lap @ 25Hz = 4500 samples × 12 bytes = 54KB)
- Session record: ~200 bytes + 20 bytes per lap
- **100 tracks + 500 reference laps + 1000 sessions ≈ 60MB** (well within IndexedDB limits)

---

## Test Data (`data/sample-tracks/`)

### Data Source
Track data sourced from the TUM Global Race Trajectory Optimization project. Center lines extracted from OpenStreetMap GPS data with smoothing applied. Track widths measured from satellite imagery.

### Files

**`austin.csv`** - Circuit of the Americas centerline (1102 points)
```csv
# x_m,y_m,w_tr_right_m,w_tr_left_m
0.960975,4.022273,7.565,7.361
4.935182,0.985988,7.584,7.382
...
```
- `x_m, y_m`: Position in local meters (origin near start/finish)
- `w_tr_right_m, w_tr_left_m`: Track width to right/left of centerline

**`austin_raceline.csv`** - Optimized racing line (1084 points)
```csv
# x_m,y_m
-2.842561,-0.963418
1.142325,-3.976526
...
```
- `x_m, y_m`: Position in local meters
- Represents an optimal lap trajectory (minimum curvature)

### Testing Strategy

**Use Case 1: Line Crossing Validation**
```javascript
// Load raceline, set timing line near start/finish, simulate driving
const raceline = loadCSV('austin_raceline.csv');
const timingLine = { p1: [-10, 0], p2: [10, 0], direction: Math.PI/2 };

// Replay at 25Hz, count crossings
// Expected: exactly 1 crossing per lap
```

**Use Case 2: Delta Calculation Testing**
```javascript
// Load raceline as "best lap" reference
// Create synthetic "slower lap" by scaling timestamps +5%
// Verify delta shows +5% at each position
```

**Use Case 3: Track Recording Validation**
```javascript
// Replay raceline through TrackRecorder
// Verify captured path has similar shape
// Verify bounds calculation is correct
// Verify timing line auto-generation works
```

**Use Case 4: Dashboard-Dev Simulation**
```javascript
// Load austin_raceline.csv into CSV replay mode
// Simulate position updates with line crossing
// Test full lap timer UI flow
```

---

## Implementation Status

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Core Timing Engine (Firmware) | ✅ COMPLETE |
| 2 | Telemetry Protocol Extension | ✅ COMPLETE |
| 3 | Basic Lap Timer Display | ⚠️ PARTIAL (dashboard-dev only) |
| 4 | Production Dashboard Integration | ❌ NOT STARTED |
| 5 | Track Configuration UI | ❌ NOT STARTED |
| 6 | Track Persistence (IndexedDB) | ❌ NOT STARTED |
| 7 | Track Recording | ❌ NOT STARTED |
| 8 | Track Auto-Detection | ❌ NOT STARTED |
| 9 | Reference Lap & Predictive Delta | ❌ NOT STARTED |
| 10 | Session History | ❌ NOT STARTED |
| 11 | Polish & Testing | ❌ NOT STARTED |
| 12 | Documentation Updates | ❌ NOT STARTED |

---

## Phase 1: Core Timing Engine (Firmware) ✅ COMPLETE

### Implementation: `framework/src/lap_timer.rs`

**Completed Features:**
- `TimingLine` struct with endpoints and valid crossing direction
- `TrackType` enum: `Loop { line }` and `PointToPoint { start, finish }`
- `LapTimerState` state machine: `Idle → Armed → Timing`
- Line-segment intersection using parametric form
- Direction validation with configurable tolerance (±90° default)
- Crossing debounce (500ms default)
- Minimum lap time validation (10s default)
- Frame flags: `CROSSED_START`, `CROSSED_FINISH`, `NEW_LAP`, `NEW_BEST`, `INVALID_LAP`
- Helper: `timing_line_from_path(p0, p1, width)` for perpendicular line creation

**Public API:**
```rust
pub struct LapTimer { ... }

impl LapTimer {
    pub fn new() -> Self;
    pub fn configure_loop(&mut self, line: TimingLine);
    pub fn configure_point_to_point(&mut self, start: TimingLine, finish: TimingLine);
    pub fn clear(&mut self);
    pub fn update(&mut self, pos: (f32, f32), velocity: (f32, f32),
                  timestamp_ms: u32, speed_mps: f32) -> u8;  // Returns flags
    pub fn current_lap_ms(&self) -> u32;
    pub fn lap_count(&self) -> u16;
}
```

**Unit Tests:** 24 tests covering geometry and state machine

---

## Phase 2: Telemetry Protocol Extension ✅ COMPLETE

### Implementation: `sensors/blackbox/src/binary_telemetry.rs`

**Protocol Version:** 2 (was 1)
**Packet Size:** 74 bytes (was 67 bytes)

**New Fields (bytes 65-71):**
```rust
pub lap_time_ms: u32,    // Current lap time in ms (0 if not timing)
pub lap_count: u16,      // Completed lap count (wraps at 65535)
pub lap_flags: u8,       // Bitfield: 1=crossed_start, 2=crossed_finish,
                         //           4=new_lap, 8=new_best, 16=invalid
```

**Main Loop Integration (`main.rs:875-884`):**
```rust
let lap_flags = lap_timer.update((ekf_x, ekf_y), (ekf_vx, ekf_vy), now_ms, speed);
let lap_timer_data = Some((
    lap_timer.current_lap_ms(),
    lap_timer.lap_count(),
    lap_flags,
));
estimator.publish_telemetry(&sensors, &estimator, &sensor_fusion, now_ms, lap_timer_data)
```

**API Endpoint:** `/api/laptimer/configure`
```
GET /api/laptimer/configure?type=loop&p1_x=...&p1_y=...&p2_x=...&p2_y=...&dir=...
GET /api/laptimer/configure?type=point_to_point&...
GET /api/laptimer/configure?type=clear
```

---

## Phase 3: Basic Lap Timer Display ⚠️ PARTIAL

### Current State

**dashboard-dev (`tools/dashboard-dev/index.html`):** ✅ Complete
- Lap timer card with current time display
- Lap count and state indicator (Armed/Timing)
- Best/Last/Delta in bottom row
- Delta color coding (green=faster, red=slower)
- CSS animations for crossing flash and best lap highlight
- Simulated 20-second lap cycle triggers crossing at loop point
- Activate via "Simulate Track" button or menu

**Production (`websocket_server.rs`):** ❌ NOT IMPLEMENTED
- API endpoint exists but no UI to use it
- Dashboard HTML does not include lap timer section
- JavaScript does not decode lap timer fields from 74-byte packet

### Remaining Work

**3.1 Verify dashboard-dev Implementation**
- [ ] Test with austin_raceline.csv replay
- [ ] Verify crossing detection triggers at correct positions (not just loop time)
- [ ] Test all state transitions: Idle → Armed → Timing → (lap complete) → Timing
- [ ] Test best lap tracking and delta calculation

**3.2 Update dashboard-dev Simulation**

Current simulation triggers crossing based on loop time, not geometry:
```javascript
// CURRENT (incorrect for testing):
if (loopTime < 100) { onLineCrossing(now); }

// NEEDED (geometry-based):
// Simulate EKF position from austin_raceline.csv
// Check actual line crossing with timing line
```

Add position simulation to dashboard-dev:
```javascript
// Simulated position track (load from CSV or generate)
let simPath = [];  // [{x, y, t}, ...]
let simIndex = 0;

function getSimulatedPosition(elapsed) {
    // Interpolate position along path
    // Return {x, y, vx, vy}
}

function checkLineCrossing(prevPos, currPos, timingLine) {
    // Use same geometry math as firmware
    return lineSegmentIntersection(prevPos, currPos, timingLine.p1, timingLine.p2);
}
```

---

## Phase 4: Production Dashboard Integration ❌ NOT STARTED

### Goal
Port lap timer UI from dashboard-dev to production dashboard in `websocket_server.rs`.

### Tasks

**4.1 Add Lap Timer CSS**
Copy from dashboard-dev to embedded CSS in websocket_server.rs:
```css
/* LAP TIMER SECTION */
.bbLapCard { ... }
.bbLapSetup { ... }
.bbLapActive { ... }
.bbLapMain { ... }
.bbLapTime { ... }
.bbLapMeta { ... }
.bbLapCount { ... }
.bbLapState { ... }
.bbLapHistory { ... }
.bbLapHistItem { ... }
.bbLapHistValue { ... }
.bbLapFlash { ... }
.bbLapBestFlash { ... }
```

**4.2 Add Lap Timer HTML**
Insert before G-meter card:
```html
<!-- Lap Timer Section -->
<section class="bbCard bbLapCard inactive" id="lap-section">
    <div class="bbLapSetup" id="lap-setup">
        <span class="bbLapSetupText">No track configured</span>
        <button class="bbLapSetupBtn" id="btn-track-manager">Tracks</button>
    </div>
    <div class="bbLapActive" id="lap-active">
        <div class="bbLapMain">
            <div class="bbLapTime bbNum" id="lap-time">0:00.000</div>
            <div class="bbLapMeta">
                <span class="bbLapCount" id="lap-count">Lap 0</span>
                <span class="bbLapState" id="lap-state">Armed</span>
            </div>
        </div>
        <div class="bbLapHistory">
            <div class="bbLapHistItem">
                <span class="bbLapHistLabel">Best</span>
                <span class="bbLapHistValue bbNum best" id="best-lap">—:——</span>
            </div>
            <div class="bbLapHistDivider"></div>
            <div class="bbLapHistItem">
                <span class="bbLapHistLabel">Last</span>
                <span class="bbLapHistValue bbNum" id="last-lap">—:——</span>
            </div>
            <div class="bbLapHistDivider"></div>
            <div class="bbLapHistItem">
                <span class="bbLapHistLabel">Delta</span>
                <span class="bbLapHistValue bbNum delta" id="lap-delta">—</span>
            </div>
        </div>
    </div>
</section>
```

**4.3 Update Telemetry Decoder**
```javascript
// Current packet size: 67 bytes
// New packet size: 74 bytes

function decodeTelemetry(base64) {
    const bytes = Uint8Array.from(atob(base64), c => c.charCodeAt(0));
    const view = new DataView(bytes.buffer);

    // ... existing fields ...

    // NEW: Lap timer fields (offset 65)
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

// Flag constants
const LAP_FLAGS = {
    CROSSED_START: 1,
    CROSSED_FINISH: 2,
    NEW_LAP: 4,
    NEW_BEST: 8,
    INVALID: 16,
};
```

**4.4 Add Lap Timer Update Logic**
```javascript
let lapTimerActive = false;
let bestLapMs = null;
let lastLapMs = null;

function updateLapTimer(telemetry) {
    if (!lapTimerActive) return;

    // Update current time display
    $('lap-time').textContent = formatLapTime(telemetry.lapTimeMs);
    $('lap-count').textContent = 'Lap ' + telemetry.lapCount;

    // Handle flags
    if (telemetry.lapFlags & LAP_FLAGS.NEW_LAP) {
        // Flash animation
        // Update last lap
        // Check for new best
    }

    if (telemetry.lapFlags & LAP_FLAGS.NEW_BEST) {
        // Best lap animation
        // Update display
    }
}
```

**4.5 Add Menu Item**
Add to menu panel:
```html
<button class="bbMenuItem" id="menu-tracks">Track Manager</button>
```

---

## Phase 5: Track Configuration UI ❌ NOT STARTED

### Goal
Allow user to set a timing line at their current position without manual coordinate entry.

### UI Design

**Track Manager Modal:**
```html
<div class="bbModal" id="track-modal">
    <div class="bbModalContent">
        <div class="bbModalHeader">
            <h2>Track Manager</h2>
            <button class="bbModalClose" id="modal-close">×</button>
        </div>

        <div class="bbModalBody">
            <!-- Current Position Display -->
            <div class="bbPositionDisplay">
                <div class="bbPosLabel">Current Position</div>
                <div class="bbPosValue">
                    <span id="pos-x">0.0</span>, <span id="pos-y">0.0</span> m
                </div>
                <div class="bbPosGps">
                    <span id="pos-lat">0.000000</span>°, <span id="pos-lon">0.000000</span>°
                </div>
            </div>

            <!-- Quick Actions -->
            <div class="bbTrackActions">
                <button class="bbActionBtn primary" id="btn-set-start">
                    Set Start Line Here
                </button>
                <button class="bbActionBtn" id="btn-record-track">
                    Record Track by Driving
                </button>
            </div>

            <!-- Saved Tracks List -->
            <div class="bbTrackList" id="track-list">
                <div class="bbTrackListHeader">Saved Tracks</div>
                <!-- Populated dynamically -->
            </div>
        </div>
    </div>
</div>
```

### "Set Start Line Here" Flow

**Step 1: Capture Current State**
```javascript
async function setStartLineHere() {
    // Get current telemetry
    const telemetry = await fetchTelemetry();

    if (!telemetry.gps_valid) {
        showError('GPS not locked. Wait for GPS fix before setting start line.');
        return;
    }

    const x = telemetry.x;
    const y = telemetry.y;
    const yaw = telemetry.yaw;  // Current heading

    // Store GPS origin for this track
    const gpsOrigin = { lat: telemetry.lat, lon: telemetry.lon };

    // Create perpendicular timing line (20m wide)
    const width = 10;  // 10m each side
    const perpAngle = yaw + Math.PI / 2;  // Perpendicular to heading

    const timingLine = {
        p1: [x + Math.cos(perpAngle) * width, y + Math.sin(perpAngle) * width],
        p2: [x - Math.cos(perpAngle) * width, y - Math.sin(perpAngle) * width],
        direction: yaw  // Valid crossing direction = current heading
    };

    // Show confirmation
    showTimingLinePreview(timingLine, gpsOrigin);
}
```

**Step 2: Preview and Confirm**
```javascript
function showTimingLinePreview(line, gpsOrigin) {
    // Show mini-map with:
    // - Current position marker
    // - Timing line visualization
    // - Arrow showing valid crossing direction

    // Confirmation buttons:
    // - "Confirm & Start" - saves track and activates timer
    // - "Adjust" - allow manual tweaks
    // - "Cancel"
}
```

**Step 3: Save and Activate**
```javascript
async function confirmStartLine(line, gpsOrigin, trackName) {
    const track = {
        id: crypto.randomUUID(),
        name: trackName || `Track ${new Date().toLocaleDateString()}`,
        type: 'loop',
        created: Date.now(),
        modified: Date.now(),
        startLine: line,
        finishLine: null,
        gpsOrigin: gpsOrigin,
        bounds: calculateBounds([line.p1, line.p2], 100),  // 100m margin
        path: null,
        bestLapMs: null,
        lapCount: 0
    };

    // Save to IndexedDB
    await trackDb.saveTrack(track);

    // Send to ESP32
    await configureEspLapTimer(track);

    // Activate lap timer UI
    activateLapTimer(track);
}
```

**Step 4: Configure ESP32**
```javascript
async function configureEspLapTimer(track) {
    const params = new URLSearchParams({
        type: track.type,
        p1_x: track.startLine.p1[0],
        p1_y: track.startLine.p1[1],
        p2_x: track.startLine.p2[0],
        p2_y: track.startLine.p2[1],
        dir: track.startLine.direction
    });

    const response = await fetch(`/api/laptimer/configure?${params}`);
    if (!response.ok) {
        throw new Error('Failed to configure lap timer');
    }
}
```

### Position Display Requirements

The UI needs to show current position to help user understand where they're setting the line:

```javascript
function updatePositionDisplay(telemetry) {
    $('pos-x').textContent = telemetry.x.toFixed(1);
    $('pos-y').textContent = telemetry.y.toFixed(1);

    if (telemetry.gps_valid) {
        $('pos-lat').textContent = telemetry.lat.toFixed(6);
        $('pos-lon').textContent = telemetry.lon.toFixed(6);
    }
}
```

---

## Phase 6: Track Persistence (IndexedDB) ❌ NOT STARTED

### Goal
Save and load tracks across sessions.

### Implementation: `TrackDatabase` Class

```javascript
class TrackDatabase {
    constructor() {
        this.db = null;
        this.DB_NAME = 'blackbox-tracks';
        this.DB_VERSION = 1;
    }

    async init() {
        return new Promise((resolve, reject) => {
            const request = indexedDB.open(this.DB_NAME, this.DB_VERSION);

            request.onerror = () => reject(request.error);
            request.onsuccess = () => {
                this.db = request.result;
                resolve(this);
            };

            request.onupgradeneeded = (event) => {
                const db = event.target.result;

                // Tracks store
                if (!db.objectStoreNames.contains('tracks')) {
                    const tracks = db.createObjectStore('tracks', { keyPath: 'id' });
                    tracks.createIndex('name', 'name', { unique: false });
                    tracks.createIndex('modified', 'modified', { unique: false });
                }

                // Reference laps store
                if (!db.objectStoreNames.contains('reference_laps')) {
                    const laps = db.createObjectStore('reference_laps', { keyPath: 'id' });
                    laps.createIndex('trackId', 'trackId', { unique: false });
                    laps.createIndex('isBest', ['trackId', 'isBest'], { unique: false });
                }

                // Sessions store
                if (!db.objectStoreNames.contains('sessions')) {
                    const sessions = db.createObjectStore('sessions', { keyPath: 'id' });
                    sessions.createIndex('trackId', 'trackId', { unique: false });
                    sessions.createIndex('date', 'date', { unique: false });
                }
            };
        });
    }

    // ========== TRACKS ==========

    async saveTrack(track) {
        track.modified = Date.now();
        return this._put('tracks', track);
    }

    async getTrack(id) {
        return this._get('tracks', id);
    }

    async getAllTracks() {
        return this._getAll('tracks');
    }

    async deleteTrack(id) {
        // Also delete associated reference laps and sessions
        const laps = await this.getReferenceLaps(id);
        for (const lap of laps) {
            await this._delete('reference_laps', lap.id);
        }

        const sessions = await this.getSessions(id);
        for (const session of sessions) {
            await this._delete('sessions', session.id);
        }

        return this._delete('tracks', id);
    }

    // ========== REFERENCE LAPS ==========

    async saveReferenceLap(trackId, lapTimeMs, samples) {
        const lap = {
            id: crypto.randomUUID(),
            trackId,
            lapTimeMs,
            created: Date.now(),
            isBest: false,
            samples  // ArrayBuffer
        };

        // Check if this is new best
        const existingBest = await this.getBestLap(trackId);
        if (!existingBest || lapTimeMs < existingBest.lapTimeMs) {
            lap.isBest = true;

            // Clear previous best flag
            if (existingBest) {
                existingBest.isBest = false;
                await this._put('reference_laps', existingBest);
            }

            // Update track's best time
            const track = await this.getTrack(trackId);
            if (track) {
                track.bestLapMs = lapTimeMs;
                await this.saveTrack(track);
            }
        }

        return this._put('reference_laps', lap);
    }

    async getBestLap(trackId) {
        const tx = this.db.transaction('reference_laps', 'readonly');
        const index = tx.objectStore('reference_laps').index('isBest');

        return new Promise((resolve, reject) => {
            const request = index.get([trackId, true]);
            request.onsuccess = () => resolve(request.result || null);
            request.onerror = () => reject(request.error);
        });
    }

    async getReferenceLaps(trackId) {
        const tx = this.db.transaction('reference_laps', 'readonly');
        const index = tx.objectStore('reference_laps').index('trackId');

        return new Promise((resolve, reject) => {
            const request = index.getAll(trackId);
            request.onsuccess = () => resolve(request.result);
            request.onerror = () => reject(request.error);
        });
    }

    // ========== SESSIONS ==========

    async saveSession(session) {
        return this._put('sessions', session);
    }

    async getSessions(trackId) {
        const tx = this.db.transaction('sessions', 'readonly');
        const index = tx.objectStore('sessions').index('trackId');

        return new Promise((resolve, reject) => {
            const request = index.getAll(trackId);
            request.onsuccess = () => resolve(request.result);
            request.onerror = () => reject(request.error);
        });
    }

    // ========== HELPERS ==========

    _put(store, value) {
        return new Promise((resolve, reject) => {
            const tx = this.db.transaction(store, 'readwrite');
            const request = tx.objectStore(store).put(value);
            request.onsuccess = () => resolve(value);
            request.onerror = () => reject(request.error);
        });
    }

    _get(store, key) {
        return new Promise((resolve, reject) => {
            const tx = this.db.transaction(store, 'readonly');
            const request = tx.objectStore(store).get(key);
            request.onsuccess = () => resolve(request.result);
            request.onerror = () => reject(request.error);
        });
    }

    _getAll(store) {
        return new Promise((resolve, reject) => {
            const tx = this.db.transaction(store, 'readonly');
            const request = tx.objectStore(store).getAll();
            request.onsuccess = () => resolve(request.result);
            request.onerror = () => reject(request.error);
        });
    }

    _delete(store, key) {
        return new Promise((resolve, reject) => {
            const tx = this.db.transaction(store, 'readwrite');
            const request = tx.objectStore(store).delete(key);
            request.onsuccess = () => resolve();
            request.onerror = () => reject(request.error);
        });
    }
}
```

### Track List UI

```html
<div class="bbTrackItem" data-track-id="uuid">
    <div class="bbTrackInfo">
        <div class="bbTrackName">My Block Loop</div>
        <div class="bbTrackMeta">
            <span>Best: 1:23.456</span>
            <span>12 laps</span>
        </div>
    </div>
    <div class="bbTrackActions">
        <button class="bbTrackBtn" data-action="activate">Use</button>
        <button class="bbTrackBtn" data-action="edit">Edit</button>
        <button class="bbTrackBtn danger" data-action="delete">×</button>
    </div>
</div>
```

```javascript
async function renderTrackList() {
    const tracks = await trackDb.getAllTracks();
    const listEl = $('track-list');

    if (tracks.length === 0) {
        listEl.innerHTML = '<div class="bbTrackEmpty">No saved tracks</div>';
        return;
    }

    // Sort by most recently modified
    tracks.sort((a, b) => b.modified - a.modified);

    listEl.innerHTML = tracks.map(track => `
        <div class="bbTrackItem" data-track-id="${track.id}">
            <div class="bbTrackInfo">
                <div class="bbTrackName">${escapeHtml(track.name)}</div>
                <div class="bbTrackMeta">
                    <span>Best: ${track.bestLapMs ? formatLapTime(track.bestLapMs) : '—'}</span>
                    <span>${track.lapCount} laps</span>
                </div>
            </div>
            <div class="bbTrackActions">
                <button class="bbTrackBtn" data-action="activate">Use</button>
                <button class="bbTrackBtn danger" data-action="delete">×</button>
            </div>
        </div>
    `).join('');

    // Add event listeners
    listEl.querySelectorAll('.bbTrackBtn').forEach(btn => {
        btn.onclick = () => handleTrackAction(btn.dataset.action, btn.closest('.bbTrackItem').dataset.trackId);
    });
}
```

---

## Phase 7: Track Recording ❌ NOT STARTED

### Goal
Allow user to define a track by driving it once.

### User Flow
1. User taps "Record Track by Driving"
2. Dashboard shows recording overlay with instructions
3. User drives around their intended circuit
4. Dashboard detects loop closure (returned near start)
5. User confirms or continues driving
6. Dashboard calculates timing line automatically
7. User names and saves the track

### Implementation: `TrackRecorder` Class

```javascript
class TrackRecorder {
    constructor() {
        this.recording = false;
        this.path = [];           // [{x, y, t}, ...]
        this.startPos = null;
        this.totalDistance = 0;
        this.loopDetected = false;

        // Configuration
        this.minLoopDistance = 500;     // Minimum track length (meters)
        this.closeProximity = 30;       // Distance to detect loop closure (meters)
        this.sampleInterval = 2;        // Minimum distance between samples (meters)
    }

    start(currentPos) {
        this.recording = true;
        this.path = [];
        this.startPos = { x: currentPos.x, y: currentPos.y };
        this.totalDistance = 0;
        this.loopDetected = false;

        this.path.push({
            x: currentPos.x,
            y: currentPos.y,
            t: Date.now()
        });
    }

    addSample(x, y) {
        if (!this.recording) return { loopDetected: false };

        // Check distance from last sample
        const last = this.path[this.path.length - 1];
        const dx = x - last.x;
        const dy = y - last.y;
        const dist = Math.sqrt(dx * dx + dy * dy);

        // Only add if moved enough
        if (dist < this.sampleInterval) {
            return { loopDetected: false };
        }

        this.totalDistance += dist;
        this.path.push({ x, y, t: Date.now() });

        // Check for loop closure
        if (this.totalDistance > this.minLoopDistance) {
            const distToStart = Math.sqrt(
                (x - this.startPos.x) ** 2 +
                (y - this.startPos.y) ** 2
            );

            if (distToStart < this.closeProximity) {
                this.loopDetected = true;
                return { loopDetected: true, distToStart };
            }
        }

        return {
            loopDetected: false,
            totalDistance: this.totalDistance,
            pointCount: this.path.length
        };
    }

    finish(trackName, gpsOrigin) {
        this.recording = false;

        if (this.path.length < 10) {
            throw new Error('Not enough points recorded');
        }

        // Calculate timing line from first few points
        const timingLine = this.calculateTimingLine();

        // Calculate bounding box
        const bounds = this.calculateBounds();

        // Downsample path for storage
        const storedPath = this.downsamplePath(500);

        return {
            id: crypto.randomUUID(),
            name: trackName,
            type: 'loop',
            created: Date.now(),
            modified: Date.now(),
            startLine: timingLine,
            finishLine: null,
            gpsOrigin,
            bounds,
            path: storedPath,
            bestLapMs: null,
            lapCount: 0
        };
    }

    calculateTimingLine() {
        // Use first ~5 points to determine direction
        const p0 = this.path[0];
        const pN = this.path[Math.min(5, this.path.length - 1)];

        const dx = pN.x - p0.x;
        const dy = pN.y - p0.y;
        const len = Math.sqrt(dx * dx + dy * dy);

        if (len < 0.1) {
            throw new Error('Could not determine direction');
        }

        // Direction of travel
        const direction = Math.atan2(dy, dx);

        // Perpendicular direction
        const perpX = -dy / len;
        const perpY = dx / len;

        // 15m line on each side
        const width = 15;

        return {
            p1: [p0.x + perpX * width, p0.y + perpY * width],
            p2: [p0.x - perpX * width, p0.y - perpY * width],
            direction
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

        // Add 50m margin
        return {
            minX: minX - 50,
            maxX: maxX + 50,
            minY: minY - 50,
            maxY: maxY + 50
        };
    }

    downsamplePath(maxPoints) {
        if (this.path.length <= maxPoints) {
            return this.path.map(p => [p.x, p.y]);
        }

        const step = Math.ceil(this.path.length / maxPoints);
        const result = [];

        for (let i = 0; i < this.path.length; i += step) {
            result.push([this.path[i].x, this.path[i].y]);
        }

        return result;
    }

    cancel() {
        this.recording = false;
        this.path = [];
    }

    getStats() {
        return {
            recording: this.recording,
            pointCount: this.path.length,
            totalDistance: this.totalDistance,
            loopDetected: this.loopDetected
        };
    }
}
```

### Recording UI Overlay

```html
<div class="bbRecordOverlay" id="record-overlay">
    <div class="bbRecordStatus">
        <div class="bbRecordIndicator"></div>
        <span>Recording Track...</span>
    </div>

    <div class="bbRecordStats">
        <div class="bbRecordStat">
            <span class="bbRecordLabel">Distance</span>
            <span class="bbRecordValue" id="rec-distance">0 m</span>
        </div>
        <div class="bbRecordStat">
            <span class="bbRecordLabel">Points</span>
            <span class="bbRecordValue" id="rec-points">0</span>
        </div>
    </div>

    <div class="bbRecordInstructions">
        Drive around your circuit once. Return to starting point to complete.
    </div>

    <div class="bbRecordLoop hidden" id="loop-detected">
        <div class="bbLoopMessage">Loop detected!</div>
        <button class="bbRecordBtn primary" id="btn-finish-recording">Save Track</button>
        <button class="bbRecordBtn" id="btn-continue-recording">Continue Driving</button>
    </div>

    <button class="bbRecordBtn danger" id="btn-cancel-recording">Cancel</button>
</div>
```

```css
.bbRecordOverlay {
    position: fixed;
    bottom: 0;
    left: 0;
    right: 0;
    background: var(--surface);
    padding: 20px;
    border-radius: 16px 16px 0 0;
    box-shadow: 0 -4px 20px rgba(0,0,0,0.15);
    z-index: 200;
}

.bbRecordIndicator {
    width: 12px;
    height: 12px;
    background: #ff3b30;
    border-radius: 50%;
    animation: recordPulse 1s infinite;
}

@keyframes recordPulse {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.5; }
}
```

---

## Phase 8: Track Auto-Detection ❌ NOT STARTED

### Goal
Automatically offer to activate a saved track when the user is within its bounds.

### Implementation

```javascript
class TrackAutoDetector {
    constructor(trackDb) {
        this.trackDb = trackDb;
        this.currentTrack = null;
        this.lastCheckTime = 0;
        this.checkInterval = 5000;  // Check every 5 seconds
        this.enabled = true;
    }

    async checkPosition(x, y) {
        // Don't check too frequently
        const now = Date.now();
        if (now - this.lastCheckTime < this.checkInterval) return null;
        this.lastCheckTime = now;

        // Skip if already on a track
        if (this.currentTrack) return null;

        // Skip if disabled
        if (!this.enabled) return null;

        const tracks = await this.trackDb.getAllTracks();
        const candidates = [];

        for (const track of tracks) {
            if (this.isWithinBounds(x, y, track.bounds)) {
                const centerX = (track.bounds.minX + track.bounds.maxX) / 2;
                const centerY = (track.bounds.minY + track.bounds.maxY) / 2;
                const dist = Math.sqrt((x - centerX) ** 2 + (y - centerY) ** 2);
                candidates.push({ track, dist });
            }
        }

        if (candidates.length === 0) return null;

        // Return closest track
        candidates.sort((a, b) => a.dist - b.dist);
        return candidates[0].track;
    }

    isWithinBounds(x, y, bounds) {
        return x >= bounds.minX && x <= bounds.maxX &&
               y >= bounds.minY && y <= bounds.maxY;
    }

    setCurrentTrack(track) {
        this.currentTrack = track;
    }

    clearCurrentTrack() {
        this.currentTrack = null;
    }
}

// Usage in main update loop
async function onTelemetryUpdate(telemetry) {
    // ... other updates ...

    // Check for track auto-detection
    const detectedTrack = await autoDetector.checkPosition(telemetry.x, telemetry.y);
    if (detectedTrack) {
        showTrackDetectedPrompt(detectedTrack);
    }
}

function showTrackDetectedPrompt(track) {
    // Show toast/notification:
    // "Track detected: {track.name}"
    // [Activate] [Dismiss]
}
```

---

## Phase 9: Reference Lap & Predictive Delta ❌ NOT STARTED

### Goal
Record position samples during each lap and use best lap as reference for real-time delta calculation.

### Lap Sample Recording

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
        // Pack into ArrayBuffer: [u32 timestamp, f32 x, f32 y] × N
        const buffer = new ArrayBuffer(this.samples.length * 12);
        const view = new DataView(buffer);

        for (let i = 0; i < this.samples.length; i++) {
            const offset = i * 12;
            view.setUint32(offset, this.samples[i].t, true);
            view.setFloat32(offset + 4, this.samples[i].x, true);
            view.setFloat32(offset + 8, this.samples[i].y, true);
        }

        return buffer;
    }
}
```

### Delta Calculator with Spatial Index

```javascript
class DeltaCalculator {
    constructor() {
        this.referenceLap = null;
        this.grid = new Map();
        this.cellSize = 10;  // 10m grid cells
    }

    setReferenceLap(samplesBuffer) {
        this.referenceLap = this.decodeSamples(samplesBuffer);
        this.buildSpatialIndex();
    }

    decodeSamples(buffer) {
        const view = new DataView(buffer);
        const samples = [];

        for (let i = 0; i < buffer.byteLength; i += 12) {
            samples.push({
                t: view.getUint32(i, true),
                x: view.getFloat32(i + 4, true),
                y: view.getFloat32(i + 8, true)
            });
        }

        return samples;
    }

    buildSpatialIndex() {
        this.grid.clear();

        for (let i = 0; i < this.referenceLap.length; i++) {
            const { x, y } = this.referenceLap[i];
            const cellX = Math.floor(x / this.cellSize);
            const cellY = Math.floor(y / this.cellSize);
            const key = `${cellX},${cellY}`;

            if (!this.grid.has(key)) {
                this.grid.set(key, []);
            }
            this.grid.get(key).push(i);
        }
    }

    calculateDelta(currentX, currentY, currentLapTimeMs) {
        if (!this.referenceLap || this.referenceLap.length === 0) {
            return null;
        }

        // Find nearest reference point
        const nearest = this.findNearestPoint(currentX, currentY);
        if (!nearest) return null;

        // Delta = current time - reference time at same position
        const deltaMs = currentLapTimeMs - nearest.t;

        return {
            deltaMs,
            distance: nearest.distance,
            confidence: nearest.distance < 20 ? 'high' : 'low'
        };
    }

    findNearestPoint(x, y) {
        const cellX = Math.floor(x / this.cellSize);
        const cellY = Math.floor(y / this.cellSize);

        let bestIdx = -1;
        let bestDist = Infinity;

        // Search 3x3 neighborhood
        for (let dx = -1; dx <= 1; dx++) {
            for (let dy = -1; dy <= 1; dy++) {
                const key = `${cellX + dx},${cellY + dy}`;
                const indices = this.grid.get(key) || [];

                for (const i of indices) {
                    const ref = this.referenceLap[i];
                    const dist = Math.sqrt((x - ref.x) ** 2 + (y - ref.y) ** 2);

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
            distance: bestDist
        };
    }

    clear() {
        this.referenceLap = null;
        this.grid.clear();
    }
}
```

### Delta Display

```javascript
function updateDeltaDisplay(deltaResult) {
    const deltaEl = $('lap-delta');

    if (!deltaResult || deltaResult.confidence === 'low') {
        deltaEl.textContent = '—';
        deltaEl.className = 'bbLapHistValue bbNum delta';
        return;
    }

    const { deltaMs } = deltaResult;
    const sign = deltaMs > 0 ? '+' : '';
    const seconds = (deltaMs / 1000).toFixed(3);

    deltaEl.textContent = `${sign}${seconds}`;

    if (deltaMs < -100) {
        deltaEl.className = 'bbLapHistValue bbNum delta faster';
    } else if (deltaMs > 100) {
        deltaEl.className = 'bbLapHistValue bbNum delta slower';
    } else {
        deltaEl.className = 'bbLapHistValue bbNum delta';  // Neutral
    }
}
```

---

## Phase 10: Session History ❌ NOT STARTED

### Goal
Track lap times across sessions and display personal bests.

### Session Management

```javascript
class SessionManager {
    constructor(trackDb) {
        this.trackDb = trackDb;
        this.currentSession = null;
    }

    startSession(trackId) {
        this.currentSession = {
            id: crypto.randomUUID(),
            trackId,
            date: Date.now(),
            laps: [],
            bestLapMs: null,
            totalLaps: 0
        };
    }

    recordLap(lapTimeMs, deltaMs, isValid = true) {
        if (!this.currentSession) return;

        this.currentSession.laps.push({
            lapTimeMs,
            deltaMs,
            isValid
        });

        if (isValid) {
            this.currentSession.totalLaps++;

            if (!this.currentSession.bestLapMs || lapTimeMs < this.currentSession.bestLapMs) {
                this.currentSession.bestLapMs = lapTimeMs;
            }
        }
    }

    async endSession() {
        if (!this.currentSession) return;

        if (this.currentSession.laps.length > 0) {
            await this.trackDb.saveSession(this.currentSession);

            // Update track lap count
            const track = await this.trackDb.getTrack(this.currentSession.trackId);
            if (track) {
                track.lapCount = (track.lapCount || 0) + this.currentSession.totalLaps;
                await this.trackDb.saveTrack(track);
            }
        }

        const session = this.currentSession;
        this.currentSession = null;
        return session;
    }
}
```

### Session History UI

```html
<div class="bbSessionHistory" id="session-history">
    <div class="bbHistoryHeader">Recent Sessions</div>
    <div class="bbHistoryList" id="history-list">
        <!-- Populated dynamically -->
    </div>
</div>
```

```javascript
async function renderSessionHistory(trackId) {
    const sessions = await trackDb.getSessions(trackId);

    // Sort by date, newest first
    sessions.sort((a, b) => b.date - a.date);

    // Show last 10 sessions
    const recent = sessions.slice(0, 10);

    $('history-list').innerHTML = recent.map(session => `
        <div class="bbSessionItem">
            <div class="bbSessionDate">${formatDate(session.date)}</div>
            <div class="bbSessionStats">
                <span>Best: ${formatLapTime(session.bestLapMs)}</span>
                <span>${session.totalLaps} laps</span>
            </div>
        </div>
    `).join('');
}
```

---

## Phase 11: Polish & Testing ❌ NOT STARTED

### Edge Cases to Handle

1. **GPS Warmup**: Don't allow track creation or timing until GPS is warmed up
   ```javascript
   if (!telemetry.gps_valid) {
       showWarning('Waiting for GPS lock...');
       return;
   }
   ```

2. **Track Boundary Exit**: Warn if vehicle leaves track bounds significantly
   ```javascript
   if (!isWithinBounds(x, y, track.bounds)) {
       showWarning('Outside track area');
   }
   ```

3. **Power Loss / Page Refresh**:
   - Current lap is lost (acceptable)
   - Session data saved on each lap completion
   - Track configuration persisted in IndexedDB

4. **Multiple Crossings**: Handled by firmware debounce (500ms)

5. **Reverse Direction**: Handled by firmware direction validation

6. **Very Long Laps**: u32 milliseconds supports up to ~70 minutes

7. **Coordinate System Reset**: If ESP32 restarts, coordinate origin changes
   - Compare GPS lat/lon to track's gpsOrigin
   - Warn user if significant drift detected

### UI Polish

1. **Lap Time Formatting**: `M:SS.mmm` (e.g., `1:23.456`)
   ```javascript
   function formatLapTime(ms) {
       if (ms === null) return '—:——';
       const totalSec = ms / 1000;
       const min = Math.floor(totalSec / 60);
       const sec = totalSec % 60;
       return `${min}:${sec.toFixed(3).padStart(6, '0')}`;
   }
   ```

2. **Delta Formatting**: `+/-S.mmm` with color
   - Green: faster than reference
   - Red: slower than reference
   - Gray: no reference or low confidence

3. **Animations**:
   - Crossing flash (green pulse)
   - New best celebration (scale pulse)
   - State change transitions

4. **Audio Cues** (optional, future enhancement):
   - Beep on line crossing
   - Different tone for new best

### Testing with Sample Data

**Test 1: Geometry Validation**
```javascript
// Load austin_raceline.csv
// Set timing line at known position
// Replay path, verify exactly 1 crossing per lap
```

**Test 2: Delta Accuracy**
```javascript
// Use austin_raceline.csv as reference lap
// Create "slow" version with timestamps scaled 1.05x
// Verify delta shows +5% throughout lap
```

**Test 3: Track Recording**
```javascript
// Replay austin_raceline.csv through TrackRecorder
// Verify:
// - Loop detection triggers
// - Timing line direction is correct
// - Bounds contain entire track
// - Path shape matches original
```

**Test 4: IndexedDB Operations**
```javascript
// Create, read, update, delete tracks
// Save and retrieve reference laps
// Verify session history accumulates
```

---

## Phase 12: Documentation Updates ❌ NOT STARTED

### Files to Update

**1. README.md**
Add new section after "Mobile Dashboard":
```markdown
## Lap Timer

Real-time lap timing with millisecond precision.

### Features
- Set timing line at current position
- Record tracks by driving
- Auto-detect saved tracks
- Best/Last/Delta display
- Predictive delta vs best lap
- Session history and personal bests

### Quick Start
1. Connect to Blackbox WiFi
2. Open dashboard (192.168.71.1)
3. Tap "Tracks" → "Set Start Line Here"
4. Drive around your circuit
5. Cross the timing line to start timing
6. Each subsequent crossing completes a lap

### Track Recording
1. Tap "Tracks" → "Record Track by Driving"
2. Drive one complete lap of your circuit
3. Return to starting point (loop will be detected)
4. Name and save your track

### Saved Tracks
Tracks are stored in your browser's IndexedDB:
- Persist across sessions and page refreshes
- Include timing line, bounds, and optional path
- Track personal bests and total lap counts
```

**2. CLAUDE.md**
Add to "Module Details" section:
```markdown
### lap_timer.rs - Lap Timing Engine

**Purpose:**
High-frequency line crossing detection for lap timing.

**Key Types:**
- `TimingLine`: Two endpoints + valid crossing direction
- `TrackType`: `Loop` or `PointToPoint`
- `LapTimerState`: `Idle`, `Armed`, `Timing`

**Flags:**
- `CROSSED_START` (1): Crossed start line this frame
- `CROSSED_FINISH` (2): Crossed finish line (point-to-point)
- `NEW_LAP` (4): Lap completed this frame
- `NEW_BEST` (8): New personal best this frame
- `INVALID_LAP` (16): Lap rejected (too short, wrong direction)

**Configuration:**
- Debounce: 500ms between crossings
- Minimum lap time: 10 seconds
- Direction tolerance: ±90°

**Telemetry Extension:**
Protocol v2 adds 7 bytes (67 → 74 total):
- `lap_time_ms` (u32): Current lap time
- `lap_count` (u16): Completed laps
- `lap_flags` (u8): Frame flags

**API Endpoint:**
`GET /api/laptimer/configure?type=loop&p1_x=...&p1_y=...&p2_x=...&p2_y=...&dir=...`
```

**3. docs/index.html**
Add to features list in structured data:
```json
"featureList": [
    // ... existing features ...
    "Lap timer with millisecond precision",
    "Track recording by driving",
    "Auto-detection of saved tracks",
    "Predictive delta calculation",
    "Session history and personal bests"
]
```

Add to feature cards section:
```html
<div class="feature-card">
    <h3>Lap Timer</h3>
    <p>Millisecond-precision lap timing with predictive delta display.
    Record tracks by driving, auto-detect saved tracks, and track
    personal bests across sessions.</p>
</div>
```

**4. CHANGELOG.md** (create if doesn't exist)
```markdown
# Changelog

## [Unreleased]

### Added
- Lap timer with line crossing detection
- Track recording by driving
- Track persistence (IndexedDB)
- Auto-detection of saved tracks
- Reference lap recording for predictive delta
- Session history tracking

### Changed
- Telemetry protocol v2 (67 → 74 bytes)
- Dashboard includes lap timer section
```

---

## Implementation Order

### Critical Path (Required for Test Drive)
1. **Phase 4**: Port lap timer UI to production dashboard
2. **Phase 5**: Track configuration UI ("Set Start Line Here")
3. **Phase 6**: Track persistence (save/load tracks)

### Enhanced Experience
4. **Phase 3**: Fix dashboard-dev simulation (geometry-based crossing)
5. **Phase 7**: Track recording by driving
6. **Phase 8**: Track auto-detection

### Full Feature Set
7. **Phase 9**: Reference lap & predictive delta
8. **Phase 10**: Session history
9. **Phase 11**: Polish & testing
10. **Phase 12**: Documentation updates

### Estimated Complexity
| Phase | Complexity | Key Challenge |
|-------|------------|---------------|
| 3 | Low | Fix simulation logic |
| 4 | Medium | HTML/CSS/JS porting, packet decoding |
| 5 | Medium | UI design, coordinate handling |
| 6 | Medium | IndexedDB operations, track list UI |
| 7 | Medium | Recording state machine, loop detection |
| 8 | Low | Bounds checking, prompt UI |
| 9 | High | Spatial indexing, sample encoding |
| 10 | Low | Session CRUD, history UI |
| 11 | Medium | Edge cases, testing infrastructure |
| 12 | Low | Documentation writing |

---

## File Changes Summary

### New Files
- (none - all JavaScript in embedded HTML)

### Modified Files (Production)
- `sensors/blackbox/src/websocket_server.rs` - Dashboard HTML/CSS/JS

### Modified Files (Development)
- `tools/dashboard-dev/index.html` - Local testing dashboard

### Documentation
- `README.md` - Add lap timer section
- `CLAUDE.md` - Add lap_timer.rs documentation
- `docs/index.html` - Update features
- `CHANGELOG.md` - Create and document changes
- `LAP_TIMER_PLAN.md` - This file (keep updated)

---

## Acceptance Criteria

### Minimum Viable Product (Test Drive Ready)
- [ ] Can set timing line at current GPS position
- [ ] Lap timer displays current lap time
- [ ] Crossing timing line completes lap and shows time
- [ ] Best and last lap times displayed
- [ ] Track saved and can be reloaded after page refresh

### Full Feature Release
- [ ] All MVP criteria
- [ ] Can record track by driving
- [ ] Saved tracks auto-detected when nearby
- [ ] Predictive delta shows position-based comparison
- [ ] Session history tracks laps across sessions
- [ ] Documentation updated (README, CLAUDE.md, docs)
- [ ] Works identically in dashboard-dev and production
