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
| 3 | Basic Lap Timer Display | ✅ COMPLETE |
| 4 | Production Dashboard Integration | ✅ COMPLETE |
| 5 | Track Configuration UI | ✅ COMPLETE |
| 6 | Track Persistence (IndexedDB) | ✅ COMPLETE |
| 5A | Start Line Approach UX | ✅ COMPLETE |
| 5B | Point-to-Point Track Creation | ✅ COMPLETE |
| **7** | **Track Recording (Enhanced)** | ❌ NOT STARTED |
| **7B** | **Track Learning** | ❌ NOT STARTED |
| 8 | Track Auto-Detection | ❌ NOT STARTED |
| 9 | Reference Lap & Predictive Delta | ❌ NOT STARTED |
| 10 | Session History | ❌ NOT STARTED |
| 11 | Polish & Testing | ❌ NOT STARTED |
| 12 | Documentation Updates | ❌ NOT STARTED |

### Technical Capabilities Summary

| Capability | Source | Resolution |
|------------|--------|------------|
| Position (x, y) | EKF @ 200Hz | ±1-2m (good GPS) |
| Heading (ψ) | EKF yaw | ±2-5° |
| Position uncertainty (σ) | Diagnostics API | Quality indicator |
| GPS updates | NEO-M9N @ 25Hz | 40ms interval |

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

## Phase 3: Basic Lap Timer Display ✅ COMPLETE

### Implementation Complete

**dashboard-dev (`tools/dashboard-dev/index.html`):** ✅ Full Implementation
- Lap timer card with current time display
- Lap count and state indicator (Armed/Timing)
- Best/Last/Delta in bottom row
- Delta color coding (green=faster, red=slower)
- CSS animations for crossing flash and best lap highlight
- Activate via "Simulate Track" button or menu

**Geometry-Based Crossing Detection:** ✅ Implemented
- `BLOCK_TRACK`: Rectangular simulated track (~283m loop, 20s at ~51 km/h)
  - 4 legs: (0,0)→(0,50)→(70,60)→(80,0)→(10,-10)→(0,0)
  - 4 corners with 10m radius, 8-step interpolation
  - Proper heading calculation at each waypoint
- `lineSegmentIntersection()`: Parametric form intersection test (ported from Rust)
- `directionValid()`: Validates crossing direction within ±90° tolerance
- `wrapAngle()`: Normalizes angles to [-π, π]
- `getSimulatedPosition()`: Interpolates position along track by elapsed time

**Position Tracking for Lap Timer:** ✅ Implemented
- `simTimingLine`: Configured at (±10, 0) with direction π/2 (northbound)
- `simPrevPos`: Tracks previous position for crossing detection
- `simulateLapTimerUpdate(now, posData)`: Geometry-based crossing check

**CSV Recording/Replay with Position:** ✅ Implemented
- Recording captures `px, py` position from simulation
- CSV export includes `pos_x, pos_y` columns
- CSV parser reads position columns when present
- Lap timer works with CSV replay when position data available

**Production (`websocket_server.rs`):** ❌ NOT IMPLEMENTED (Phase 4)
- API endpoint exists but no UI to use it
- Dashboard HTML does not include lap timer section
- JavaScript does not decode lap timer fields from 74-byte packet

### Verified Functionality
- [x] Geometry-based crossing detection (not time-based)
- [x] State transitions: Idle → Armed → Timing → (lap complete) → Timing
- [x] Best lap tracking and delta calculation
- [x] Visual feedback (flash, delta colors)
- [x] Position data in CSV recording
- [x] CSV replay with lap timing (when position available)

---

## Phase 4: Production Dashboard Integration ✅ COMPLETE

### Implementation Complete

Lap timer UI has been ported from dashboard-dev to production dashboard in `websocket_server.rs`.

**4.1 Lap Timer CSS** ✅
Added all lap timer styles to embedded CSS:
- `.bbLapCard`, `.bbLapSetup`, `.bbLapActive`, `.bbLapMain`
- `.bbLapTime`, `.bbLapMeta`, `.bbLapCount`, `.bbLapState`
- `.bbLapHistory`, `.bbLapHistItem`, `.bbLapHistValue`
- `.bbLapFlash`, `.bbLapBestFlash` animations

**4.2 Lap Timer HTML** ✅
Inserted lap timer section before G-meter card:
- Setup view with "Tracks" button
- Active view with current lap time, lap count, state
- History row with Best, Last, Delta

**4.3 Telemetry Decoder Updated** ✅
Added 74-byte protocol v2 support:
```javascript
// Decode lap timer fields (backward compatible)
const lapTimeMs = buf.byteLength >= 72 ? d.getUint32(65, 1) : 0;
const lapCnt = buf.byteLength >= 72 ? d.getUint16(69, 1) : 0;
const lapFlags = buf.byteLength >= 72 ? d.getUint8(71) : 0;
```

**4.4 Lap Timer State Management** ✅
```javascript
// State variables
let lapTimerActive = false, lapCount = 0, bestLapMs = 0, lastLapMs = 0, prevLapFlags = 0;

// Flag constants
const LAP_FLAG_CROSSED_START=1, LAP_FLAG_CROSSED_FINISH=2;
const LAP_FLAG_NEW_LAP=4, LAP_FLAG_NEW_BEST=8, LAP_FLAG_INVALID=16;

// Update function handles:
// - Automatic activation when lap timer data received
// - Current lap time display (fmtLapTime)
// - Lap count with in-progress indicator
// - State indicator (Armed/Timing)
// - NEW_LAP flash animation
// - NEW_BEST highlight animation
```

**4.5 Menu Items** ✅
- Added "Tracks" button in lap timer setup view
- Added "Tracks" menu item in kebab menu
- Both show placeholder message for track configuration (Phase 5)

### Key Differences from dashboard-dev

| Aspect | dashboard-dev | Production |
|--------|--------------|------------|
| Lap timing source | Local simulation | ESP32 firmware |
| Crossing detection | JavaScript geometry | Rust lap_timer.rs |
| Configuration | Simulated track | /api/laptimer/configure |
| Data flow | Computed locally | Received via telemetry |

### API Integration

The production dashboard integrates with the existing lap timer API:
```
GET /api/laptimer/configure?type=loop&p1_x=...&p1_y=...&p2_x=...&p2_y=...&dir=...
GET /api/laptimer/configure?type=clear
```

When firmware sends lap timer data in telemetry (protocol v2, 74 bytes):
- Dashboard automatically activates lap timer display
- Updates in real-time from telemetry stream
- Flash animations triggered by lap flags

---

## Phase 5: Track Configuration UI ✅ COMPLETE

### Implementation Complete

**Dashboard-dev (`tools/dashboard-dev/index.html`):** ✅ Full Implementation
- Track Manager modal with position display
- "Set Start Line Here" creates perpendicular timing line at current position
- Track list with Use/Delete actions
- Active track display with best lap and lap count
- IndexedDB persistence for tracks

**Production (`sensors/blackbox/src/websocket_server.rs`):** ✅ Full Implementation
- Track Manager modal matching dashboard-dev
- Position display from telemetry (EKF x, y, yaw, speed)
- API integration: calls `/api/laptimer/configure` when track activated
- Track persistence via IndexedDB (shared schema)
- Best lap tracking: saves to track when lap completes

### Key Implementation Details

**Track Data Structure:**
```javascript
{
    id: "track_1234567890_abc123xyz",
    name: "My Track",
    type: "loop",
    created: timestamp,
    modified: timestamp,
    startLine: {
        p1: [x + 10*cos(heading+π/2), y + 10*sin(heading+π/2)],
        p2: [x - 10*cos(heading+π/2), y - 10*sin(heading+π/2)],
        direction: heading  // Valid crossing direction
    },
    origin: { x, y },  // Where track was created
    bestLapMs: null,
    lapCount: 0
}
```

**Timing Line Creation:**
- 20m wide (10m each side of center)
- Perpendicular to current heading
- Direction = current heading (valid crossing direction)

**Production API Integration:**
```javascript
await fetch('/api/laptimer/configure?type=loop' +
    '&p1_x=' + line.p1[0] + '&p1_y=' + line.p1[1] +
    '&p2_x=' + line.p2[0] + '&p2_y=' + line.p2[1] +
    '&dir=' + line.direction);
```

---

## Phase 5A: Start Line Approach UX ✅ COMPLETE

### Goal

Provide clear UX for users returning to a saved track. Answer: "I loaded my track, now what?"

### Problem Statement

When a user loads a saved track:
1. Lap timer shows "Armed" - good
2. User must drive to the start line - but where is it?
3. User must cross the line to start timing - but there's no feedback

**Current State:** User is left guessing where their start line is and gets no feedback until they accidentally cross it.

### Proposed Solution

#### 5A.1 Distance to Start Line Display

When a track is active but not timing, show distance and direction to start line:

```
┌──────────────────────────────┐
│  LAP TIMER                   │
│  ─────────────────           │
│  Track: My Block Loop        │
│  Status: Armed               │
│                              │
│  ┌────────────────────────┐  │
│  │ Start Line: 127m NE    │  │
│  │ ────────→              │  │
│  └────────────────────────┘  │
│                              │
│  Cross start line to begin   │
└──────────────────────────────┘
```

**Implementation:**
```javascript
function getDistanceToStartLine(currentX, currentY, track) {
    // Center of timing line
    const cx = (track.startLine.p1[0] + track.startLine.p2[0]) / 2;
    const cy = (track.startLine.p1[1] + track.startLine.p2[1]) / 2;

    const dx = cx - currentX;
    const dy = cy - currentY;
    const distance = Math.sqrt(dx*dx + dy*dy);
    const bearing = Math.atan2(dy, dx);  // Radians

    return { distance, bearing };
}

function bearingToCardinal(rad) {
    const deg = (rad * 180 / Math.PI + 360) % 360;
    const dirs = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
    return dirs[Math.round(deg / 45) % 8];
}
```

#### 5A.2 Approach Detection

As user gets closer, provide progressive feedback:

| Distance | Feedback |
|----------|----------|
| > 100m | "Start Line: 127m NE" |
| 50-100m | "Approaching... 67m" |
| 20-50m | "Getting close! 35m" |
| < 20m | "At start line - cross to begin!" |

**Visual Indicator:**
- Progress bar or proximity indicator
- Color change: gray → yellow → green

#### 5A.3 Standing Start Mode (Optional Enhancement)

For users who want to start from a standstill at the start line:

1. Detect: Stationary (speed < 2 km/h) AND within 15m of start line
2. After 2 seconds stationary, offer: "Ready for standing start?"
3. Show countdown: 3... 2... 1... GO!
4. Timing begins when:
   - User crosses line, OR
   - Speed exceeds 5 km/h (movement detected)

**UI:**
```
┌──────────────────────────────┐
│  STANDING START              │
│  ─────────────────           │
│                              │
│          ◉ READY             │
│                              │
│    Hold position for         │
│    countdown...              │
│                              │
│    [Cancel Standing Start]   │
└──────────────────────────────┘
```

Then countdown:
```
┌──────────────────────────────┐
│                              │
│            3                 │
│                              │
└──────────────────────────────┘
```

### Implementation Order

1. **5A.1 Distance Display** - Simple, high value
2. **5A.2 Approach Detection** - Progressive feedback
3. **5A.3 Standing Start** - Nice to have, more complex

### Complexity Assessment

| Sub-phase | Complexity | Value |
|-----------|------------|-------|
| 5A.1 | Low | High - answers "where is my start line?" |
| 5A.2 | Low | Medium - improves approach experience |
| 5A.3 | Medium | Medium - nice for competitive use |

### Implementation Notes (COMPLETE)

**5A.1 & 5A.2 implemented:**
- Start line indicator shows distance and directional arrow (↑↗→↘↓↙←↖) relative to heading
- Progressive feedback with color changes:
  - `> 100m`: Gray text "Start: Xm [arrow]"
  - `50-100m`: Yellow text "Approaching: Xm [arrow]"
  - `15-50m`: Green text "Getting close: Xm [arrow]"
  - `< 15m`: Pulsing green "Cross to begin! [arrow]"
- New track UX: Shows "Drive track, cross start to begin" instead of distance (user is already at the line)
- Indicator hidden during active timing (CSS class `.timing`)

**5A.3 Standing Start:** Not implemented (optional enhancement for future)

**Files modified:**
- `tools/dashboard-dev/index.html`: CSS, HTML, JavaScript for indicator
- `sensors/blackbox/src/websocket_server.rs`: Production dashboard with same functionality

---

## Phase 6: Track Persistence (IndexedDB) ✅ COMPLETE

### Implementation Complete

Track persistence is implemented as part of Phase 5. Key functionality:

**IndexedDB Schema:**
- Database: `blackbox-tracks`, version 1
- Object store: `tracks` with keyPath `id`
- Index: `modified` for sorting by recency

**CRUD Operations:**
- `saveTrack(track)` - Create/update with auto-modified timestamp
- `getTrack(id)` - Get single track by ID
- `getAllTracks()` - List all tracks
- `deleteTrackFromDB(id)` - Remove track (production uses different name to avoid collision)

**Best Lap Tracking:**
- `updateTrackBestLap(lapTimeMs)` called when lap completes
- Updates track's bestLapMs if new best
- Increments lapCount
- Saves to IndexedDB

---

## Phase 5B: Point-to-Point Track Creation ✅ COMPLETE

### Goal
Add UI for creating point-to-point tracks (hill climbs, rally stages, drag strips). Backend support already complete.

### Implementation Complete

**Dashboard-dev (`tools/dashboard-dev/index.html`):** ✅ Full Implementation
- Track type selector (Loop vs Point-to-Point) in track creation section
- Two-step P2P creation flow with progress indicators
- Step 1: "Set Start Line Here" → shows checkmark, enables "Set Finish Line"
- Step 2: "Set Finish Line Here" → completes track, prompts for name
- Live distance display during P2P creation (distance from start position)
- "NEW" badge for first-run tracks (both loop and P2P)
- Contextual terminology: "Run" instead of "Lap", "Running" instead of "Timing"
- Finish line indicator during active runs (distance + direction to finish)
- "Finished!" state briefly displayed (1.5s) after crossing finish

**Production (`sensors/blackbox/src/websocket_server.rs`):** ✅ Full Implementation
- All P2P CSS styles ported (track type selector, progress steps, finish indicator)
- P2P HTML elements added (selector, progress UI, finish line indicator)
- P2P JavaScript functions (selectTrackType, updateP2PCreationUI, setFinishLineHere, etc.)
- activateTrack sends P2P config to ESP32 with both start and finish lines
- API integration with `/api/laptimer/configure?type=point_to_point&...`
- Track name display in main UI (`.bbLapTrackName` element)
- P2P warmup state and logic for new tracks
- Demo Tracks section with Demo Loop and Demo P2P Stage
- formatDistance function for km/m display

**UX Cleanup (dashboard-dev only):**
- Removed confusing "Simulate Loop" and "Test P2P Stage" menu buttons
- These were development artifacts that served no user-facing purpose
- Demo Tracks in Track Manager now serve the testing purpose better

**Demo Tracks Feature:**
- Built-in "Demo Loop" and "Demo P2P Stage" tracks in Track Manager
- Always available without needing to create them
- Demo tracks match simulation geometries for testing
- Demo tracks marked with "Simulation track" label
- Demo tracks use `[x, y]` array format for coordinates (must match line crossing detection)

**Distance Formatting:**
- `formatDistance(meters)` displays distances appropriately for track scale
- Shows "X.Xkm" for distances >= 1000m (e.g., "4.4km" for long circuits)
- Shows "Xm" for distances < 1000m (e.g., "234m")
- Applied to: start line indicator, finish line indicator, P2P creation distance

**Track Name Display:**
- Active track name shown on main UI below lap time during timing
- Appears in subtle gray text (opacity 0.45) to avoid distraction
- Cleared when track is deactivated

**Post-Completion UX (P2P):**
- After finishing a P2P run, indicator shows "Return to start: X.Xkm" instead of "Start: X.Xm"
- Uses `suppressStartLineIndicator` flag to control messaging context
- Helps users understand they need to return to start for another run

### Key Implementation Details

**Track Type Selector CSS:**
```css
.bbTrackTypeSelector { display: flex; gap: 8px; margin: 12px 0; }
.bbTrackTypeBtn { flex: 1; padding: 10px; border-radius: 8px; ... }
.bbTrackTypeBtn.selected { background: #007aff; color: white; }
```

**P2P Creation State:**
```javascript
let selectedTrackType = 'loop';  // 'loop' or 'point_to_point'
let p2pCreationState = null;     // { startLine, startPos } when creating P2P track
```

**P2P Progress UI:**
```html
<div class="bbP2PProgress" id="p2p-progress" style="display:none">
    <div class="bbP2PStep completed" id="p2p-step-start">
        <span class="bbP2PIcon">✓</span>
        <span class="bbP2PLabel">Start Line</span>
    </div>
    <div class="bbP2PConnector"></div>
    <div class="bbP2PStep" id="p2p-step-finish">
        <span class="bbP2PIcon">2</span>
        <span class="bbP2PLabel">Finish Line</span>
    </div>
    <div class="bbP2PDistance" id="p2p-distance">0m</div>
</div>
```

**Finish Line Indicator (during P2P runs):**
```javascript
function updateFinishLineIndicator() {
    // Shows: "Approaching finish: Xm [arrow]"
    // Progressive feedback based on distance
    // Hidden when not timing or not P2P track
}
```

**Terminology Handling:**
```javascript
const isP2P = activeTrack?.type === 'point_to_point';
$('lap-count').textContent = isP2P ? 'Run ' + count : 'Lap ' + count;
$('lap-state').textContent = isP2P ? 'Running' : 'Timing';
```

**P2P Warmup (Prevents Immediate Timing Start):**
```javascript
let p2pNeedsWarmup = false;  // Set true for newly created P2P tracks

// In activateTrack:
p2pNeedsWarmup = isP2P && track.isNew;  // Only new P2P tracks need warmup

// In simulateLapTimerUpdate:
if (p2pNeedsWarmup) {
    // Calculate distance from start line center
    const cx = (startLine.p1[0] + startLine.p2[0]) / 2;
    const cy = (startLine.p1[1] + startLine.p2[1]) / 2;
    const distToStart = Math.sqrt((posData.x - cx) ** 2 + (posData.y - cy) ** 2);
    if (distToStart > 50) {
        p2pNeedsWarmup = false;  // Allow crossings after moving 50m away
    }
}
// Skip start line crossings while warmup is needed
```

**Rationale:** When creating a new P2P track, the user sets the start line at their current position.
Without warmup, crossing detection would trigger immediately. The 50m requirement ensures the user
has moved away from the start line before timing can begin, allowing them to approach naturally.

**Track Name Display:**
Track name displays on the main UI below the lap time when a track is active:
```css
.bbLapTrackName {
    font-size: 12px;
    font-weight: 500;
    opacity: 0.45;
    margin-top: 4px;
    letter-spacing: 0.02em;
    max-width: 200px;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
}
```

```html
<div class="bbLapTrackName" id="lap-track-name"></div>
```

```javascript
// In activateTrack:
$('lap-track-name').textContent = track.name;

// In deactivateTrack:
$('lap-track-name').textContent = '';
```

**Demo Tracks (Built-in):**
```javascript
// CRITICAL: Coordinates must use [x, y] array format, NOT {x, y} objects
// Line crossing detection expects arrays for p1/p2 coordinates
const DEMO_TRACKS = [
    {
        id: 'demo_loop',
        name: 'Demo Loop',
        type: 'loop',
        isDemo: true,  // Marks as demo track (non-deletable)
        startLine: { p1: [-5, 0], p2: [5, 0], direction: Math.PI / 2 }
    },
    {
        id: 'demo_p2p',
        name: 'Demo P2P Stage',
        type: 'point_to_point',
        isDemo: true,
        startLine: { p1: [-5, 0], p2: [5, 0], direction: Math.PI / 2 },
        finishLine: { p1: [195, 40], p2: [205, 40], direction: -Math.PI / 2 }
    }
];
```

**formatDistance Helper:**
```javascript
function formatDistance(meters) {
    if (meters >= 1000) {
        return (meters / 1000).toFixed(1) + 'km';
    }
    return Math.round(meters) + 'm';
}
```

### User Flow
1. User opens track modal, taps "New Track"
2. User selects track type: **Loop** or **Point-to-Point**
3. For Point-to-Point:
   - Step 1: User drives to start location, taps "Set Start Line"
   - Step 2: User drives to finish location, taps "Set Finish Line"
   - User names and saves the track
4. Track activates automatically after creation

### UI Changes

**Track Creation Modal:**
```
┌─────────────────────────────┐
│  CREATE NEW TRACK           │
│  ─────────────────          │
│                             │
│  Track Type:                │
│  ┌─────────┐ ┌─────────────┐│
│  │  Loop   │ │Point-to-Point│
│  │   ◉     │ │      ○      ││
│  └─────────┘ └─────────────┘│
│                             │
│  [Set Start Line Here]      │
│                             │
└─────────────────────────────┘
```

**Point-to-Point Creation (Step 1 - At Start):**
```
┌─────────────────────────────┐
│  POINT-TO-POINT TRACK       │
│  ─────────────────          │
│                             │
│  ✓ Start Line Set           │
│    Position: (127.3, 45.2)  │
│                             │
│  ○ Finish Line              │
│    Drive to finish location │
│                             │
│  [Set Finish Line Here]     │
│  [Cancel]                   │
└─────────────────────────────┘
```

**Point-to-Point Creation (Step 2 - At Finish):**
```
┌─────────────────────────────┐
│  POINT-TO-POINT TRACK       │
│  ─────────────────          │
│                             │
│  ✓ Start Line Set           │
│  ✓ Finish Line Set          │
│                             │
│  Track Name:                │
│  ┌─────────────────────────┐│
│  │ Mulholland Hill Climb   ││
│  └─────────────────────────┘│
│                             │
│  [Save Track]               │
│  [Cancel]                   │
└─────────────────────────────┘
```

### Active Point-to-Point Display

**Armed State (at start):**
```
┌──────────────────────────────┐
│         0:00.000             │
│       Demo P2P Stage         │ ← Track name displayed
│       Run 0 · Armed          │
│                              │
│  ┌────────────────────────┐  │
│  │ Cross start to begin   │  │
│  └────────────────────────┘  │
│                              │
│  Best     Last      Delta    │
│  —:——     —:——        —      │
└──────────────────────────────┘
```

**Running State:**
```
┌──────────────────────────────┐
│         1:23.456             │
│       Demo P2P Stage         │
│       Run 1 · Running        │
│                              │
│  ┌────────────────────────┐  │
│  │ Approaching finish: 234m ↑│  │ ← Uses formatDistance (km for long tracks)
│  └────────────────────────┘  │
│                              │
│  Best     Last      Delta    │
│  1:45.2   —:——        —      │
└──────────────────────────────┘
```

**Finished State:**
```
┌──────────────────────────────┐
│         1:42.789             │
│       Demo P2P Stage         │ ← Track name displayed
│    Run 1 · Finished ✓        │
│                              │
│  ┌────────────────────────┐  │
│  │ Return to start: 4.4km │  │ ← Shows distance back to start
│  └────────────────────────┘  │
│                              │
│  Best     Last      Delta    │
│  1:42.7   1:42.7    BEST     │
└──────────────────────────────┘
```

**Implementation Note:** After P2P run completion, `suppressStartLineIndicator = true` is set.
This changes the indicator from showing "Start: Xm" to "Return to start: X.Xkm", providing
contextually appropriate guidance for the user to return for another run.

### State Machine (Point-to-Point)

```
             cross start
    Armed ─────────────────► Running
      ▲                         │
      │                         │ cross finish
      │ return to               ▼
      │ start area          Finished
      └─────────────────────────┘
            (auto-reset)
```

### Implementation

**Track Schema Extension:**
```javascript
{
  type: "point_to_point",  // vs "loop"
  startLine: { p1: [x,y], p2: [x,y], direction: rad },
  finishLine: { p1: [x,y], p2: [x,y], direction: rad },
  // ...
}
```

**API Call:**
```javascript
async function activatePointToPointTrack(track) {
    const s = track.startLine, f = track.finishLine;
    const url = '/api/laptimer/configure?type=point_to_point' +
        '&p1_x=' + s.p1[0] + '&p1_y=' + s.p1[1] +
        '&p2_x=' + s.p2[0] + '&p2_y=' + s.p2[1] +
        '&dir=' + s.direction +
        '&f_p1_x=' + f.p1[0] + '&f_p1_y=' + f.p1[1] +
        '&f_p2_x=' + f.p2[0] + '&f_p2_y=' + f.p2[1] +
        '&f_dir=' + f.direction;
    await fetch(url);
}
```

**Terminology Changes:**
| Loop | Point-to-Point |
|------|----------------|
| Lap 1, Lap 2 | Run 1, Run 2 |
| Armed | Armed |
| Timing | Running |
| (continuous) | Finished |
| Best Lap | Best Run |
| Last Lap | Last Run |

### Finish Line Approach Indicator

Reuse Phase 5A approach indicator logic:
- When running, show distance to finish line
- Progressive feedback: "Distance to finish: 234m ↑" → "Approaching finish" → "Cross to finish!"

### UX Polish (Post-Review)

After comprehensive UX review, the following improvements were made:

**1. Clear Active Track Button**
- Production now shows "Clear Active Track" button in Track Manager when a track is active
- Allows users to completely deactivate lap timing without selecting another track

**2. Consistent "runs" vs "laps" Terminology**
- Track list and active track display now correctly show "runs" for P2P tracks
- Previously hardcoded as "laps" for all track types

**3. formatDistance() Used Consistently**
- All distance displays now use `formatDistance()` for proper km/m formatting
- Includes: start line indicator, return to start, P2P creation distance, stage length

**4. Demo Tracks Removed from Production**
- Demo tracks (Demo Loop, Demo P2P Stage) only exist in dashboard-dev for simulation testing
- Production uses real GPS coordinates that wouldn't match simulation track positions
- Demo tracks remain in dashboard-dev for development and testing

**5. Clearer Post-Creation Message**
- Changed "Drive track, cross start to begin" to "Drive a lap, then cross start to begin"
- Clearly instructs user to complete a circuit before timing begins

**6. Consistent Default Text**
- Both dashboards now show "Tap to configure" when no track is active
- More actionable than previous "No track configured" in dashboard-dev

---

## Phase 7: Track Recording (Enhanced) ❌ NOT STARTED

### Goal
Allow user to define a track by driving it once, with high-quality position capture that preserves corners and filters GPS noise.

### Technical Background

**Available Data (from EKF + GPS):**
| Field | Source | Update Rate | Typical Accuracy |
|-------|--------|-------------|------------------|
| Position (x, y) | EKF state | 200 Hz (predicted), 25 Hz (GPS corrected) | ±1-2m (good GPS) |
| Heading (ψ) | EKF yaw | 200 Hz | ±2-5° |
| Speed | EKF velocity | 200 Hz | ±0.5 m/s |
| Position σ | Diagnostics API | 1 Hz (polled) | Quality indicator |

**Resolution Analysis:**
- At 50 km/h (14 m/s): GPS updates every 0.56m
- At 100 km/h (28 m/s): GPS updates every 1.12m
- Corner detection requires heading changes, not just position
- EKF provides smoother positions than raw GPS

### Enhanced Sample Structure

```javascript
// Raw sample during recording (full detail)
{
    x: f32,           // EKF position X (meters)
    y: f32,           // EKF position Y (meters)
    heading: f32,     // EKF yaw (radians) - CRITICAL for corners
    speed: f32,       // Speed (m/s) - for adaptive sampling
    sigma: f32,       // Position uncertainty (meters) - quality filter
    t: u32            // Timestamp (ms since recording start)
}

// Centerline point (processed, stored)
{
    x: f32,           // Smoothed position X
    y: f32,           // Smoothed position Y
    heading: f32,     // Direction of travel
    curvature: f32,   // Local curvature (rad/m) - identifies corners
    confidence: f32,  // Quality score (0-1)
    lapCount: u16     // How many laps contributed (for learning)
}
```

### User Flow

**Loop Track Recording:**
1. User taps "Record Track" and selects "Loop"
2. Dashboard shows recording overlay with GPS quality indicator
3. User drives around their intended circuit
4. Dashboard detects loop closure (near start + heading aligned)
5. User confirms or continues driving
6. Dashboard processes samples, calculates timing line
7. User names and saves the track

**Point-to-Point Track Recording:**
1. User taps "Record Track" and selects "Point-to-Point"
2. Dashboard shows "Drive from start to finish"
3. User drives the route
4. User taps "Mark Finish" when done
5. Dashboard processes samples, calculates start/finish lines
6. User names and saves the track

### Implementation: `TrackRecorder` Class

```javascript
class TrackRecorder {
    constructor() {
        this.recording = false;
        this.trackType = 'loop';      // 'loop' or 'point_to_point'
        this.rawSamples = [];         // All samples during recording
        this.keyPoints = [];          // Adaptive-sampled key points
        this.startPos = null;
        this.startHeading = null;
        this.totalDistance = 0;
        this.loopDetected = false;
        this.lastSigma = 5.0;         // Track GPS quality

        // Adaptive sampling thresholds
        this.maxSigma = 5.0;          // Reject samples with worse uncertainty
        this.minDistance = 1.0;       // Minimum 1m between stored points
        this.headingThreshold = 0.05; // ~3° triggers a sample
        this.cornerCurvature = 0.02;  // rad/m = definitely a corner

        // Loop detection
        this.minLoopDistance = 200;   // Minimum track length (meters)
        this.closeProximity = 25;     // Distance to detect loop closure (meters)
        this.headingTolerance = 0.5;  // ~30° heading match for loop closure
    }

    start(pos, trackType = 'loop') {
        this.recording = true;
        this.trackType = trackType;
        this.rawSamples = [];
        this.keyPoints = [];
        this.startPos = { x: pos.x, y: pos.y };
        this.startHeading = pos.heading;
        this.totalDistance = 0;
        this.loopDetected = false;

        // First sample is always a key point
        this.keyPoints.push({
            x: pos.x,
            y: pos.y,
            heading: pos.heading,
            speed: pos.speed,
            sigma: pos.sigma || 3.0,
            t: 0,
            curvature: 0,
            isCorner: false
        });

        this.startTime = Date.now();
    }

    /**
     * Add a position sample during recording
     * @param {Object} pos - { x, y, heading, speed, sigma }
     * @returns {Object} - { stored, loopDetected, stats }
     */
    addSample(pos) {
        if (!this.recording) return { stored: false };

        const t = Date.now() - this.startTime;
        this.lastSigma = pos.sigma || 5.0;

        // Always store raw sample for post-processing
        this.rawSamples.push({
            x: pos.x,
            y: pos.y,
            heading: pos.heading,
            speed: pos.speed,
            sigma: pos.sigma || 5.0,
            t
        });

        // Quality gate - don't store poor GPS as key points
        if (pos.sigma > this.maxSigma) {
            return {
                stored: false,
                reason: 'poor_gps',
                stats: this.getStats()
            };
        }

        // Check if this should be a key point
        const last = this.keyPoints[this.keyPoints.length - 1];
        const dx = pos.x - last.x;
        const dy = pos.y - last.y;
        const dist = Math.sqrt(dx * dx + dy * dy);
        const headingChange = Math.abs(this.wrapAngle(pos.heading - last.heading));

        // Update total distance
        this.totalDistance += dist;

        // Adaptive distance threshold based on speed
        // At higher speeds, sample less frequently (by distance)
        const adaptiveMinDist = Math.max(
            this.minDistance,
            pos.speed * 0.08  // 80ms of travel
        );

        // Sample if: moved enough distance OR turned enough
        const shouldStore =
            dist >= adaptiveMinDist ||
            headingChange >= this.headingThreshold;

        if (shouldStore) {
            // Compute local curvature (heading change per meter)
            const curvature = dist > 0.1 ? headingChange / dist : 0;
            const isCorner = curvature > this.cornerCurvature;

            this.keyPoints.push({
                x: pos.x,
                y: pos.y,
                heading: pos.heading,
                speed: pos.speed,
                sigma: pos.sigma || 3.0,
                t,
                curvature,
                isCorner
            });

            // Check for loop closure (loop tracks only)
            if (this.trackType === 'loop') {
                const loopResult = this.checkLoopClosure(pos);
                if (loopResult.detected) {
                    this.loopDetected = true;
                    return {
                        stored: true,
                        loopDetected: true,
                        loopQuality: loopResult.quality,
                        stats: this.getStats()
                    };
                }
            }

            return {
                stored: true,
                curvature,
                isCorner,
                stats: this.getStats()
            };
        }

        return {
            stored: false,
            reason: 'too_close',
            stats: this.getStats()
        };
    }

    checkLoopClosure(pos) {
        if (this.totalDistance < this.minLoopDistance) {
            return { detected: false };
        }

        const distToStart = Math.sqrt(
            (pos.x - this.startPos.x) ** 2 +
            (pos.y - this.startPos.y) ** 2
        );

        // Must be close AND heading aligned (same direction as start)
        const headingDiff = Math.abs(this.wrapAngle(pos.heading - this.startHeading));
        const headingMatch = headingDiff < this.headingTolerance;

        if (distToStart < this.closeProximity && headingMatch) {
            return {
                detected: true,
                distToStart,
                headingDiff,
                quality: this.assessQuality()
            };
        }

        return { detected: false };
    }

    assessQuality() {
        const validPoints = this.keyPoints.filter(p => p.sigma < 3.0);
        const avgSigma = this.keyPoints.reduce((s, p) => s + p.sigma, 0) / this.keyPoints.length;
        const cornerCount = this.keyPoints.filter(p => p.isCorner).length;

        let rating = 'good';
        if (avgSigma > 3.5 || validPoints.length < this.keyPoints.length * 0.7) {
            rating = 'fair';
        }
        if (avgSigma > 4.5 || validPoints.length < this.keyPoints.length * 0.5) {
            rating = 'poor';
        }

        return {
            rating,
            avgUncertainty: avgSigma,
            goodSampleRatio: validPoints.length / this.keyPoints.length,
            corners: cornerCount,
            totalPoints: this.keyPoints.length
        };
    }

    /**
     * Finish recording and process into final track
     */
    finish(trackName, gpsOrigin) {
        this.recording = false;

        if (this.keyPoints.length < 20) {
            throw new Error('Not enough points recorded (minimum 20)');
        }

        // 1. Smooth the path (reduces GPS jitter)
        const smoothed = this.smoothPath(this.keyPoints, 3);

        // 2. Recompute curvatures on smoothed path
        const withCurvature = this.computeCurvatures(smoothed);

        // 3. Intelligent downsampling (preserves corners)
        const centerline = this.curvaturePreservingDownsample(withCurvature, 500);

        // 4. Calculate timing line(s)
        const startLine = this.calculateTimingLine(centerline, 'start');
        let finishLine = null;
        if (this.trackType === 'point_to_point') {
            finishLine = this.calculateTimingLine(centerline, 'finish');
        }

        // 5. Calculate bounds with margin
        const bounds = this.calculateBounds(centerline, 30);

        // 6. Calculate total track distance
        const totalDistance = this.calculatePathLength(centerline);

        return {
            id: crypto.randomUUID(),
            name: trackName,
            type: this.trackType,
            created: Date.now(),
            modified: Date.now(),
            startLine,
            finishLine,
            gpsOrigin,
            bounds,
            centerline,  // Full centerline with heading/curvature
            displayPath: centerline.map(p => [p.x, p.y]),  // Simple path for display
            bestLapMs: null,
            lapCount: 0,
            totalDistance,
            learningEnabled: true,
            lastLearnedLap: null,
            quality: this.assessQuality()
        };
    }

    smoothPath(points, windowSize) {
        const result = [];
        const halfWindow = Math.floor(windowSize / 2);

        for (let i = 0; i < points.length; i++) {
            const start = Math.max(0, i - halfWindow);
            const end = Math.min(points.length - 1, i + halfWindow);
            const window = points.slice(start, end + 1);

            // Weighted average favoring center and low-sigma points
            let sumX = 0, sumY = 0, sumWeight = 0;

            for (let j = 0; j < window.length; j++) {
                const distFromCenter = Math.abs(j - (i - start));
                const positionWeight = 1.0 / (1 + distFromCenter * 0.5);
                const qualityWeight = 1.0 / (window[j].sigma + 0.5);
                const weight = positionWeight * qualityWeight;

                sumX += window[j].x * weight;
                sumY += window[j].y * weight;
                sumWeight += weight;
            }

            result.push({
                x: sumX / sumWeight,
                y: sumY / sumWeight,
                heading: points[i].heading,  // Keep original heading
                speed: points[i].speed,
                sigma: points[i].sigma,
                t: points[i].t
            });
        }

        return result;
    }

    computeCurvatures(points) {
        const result = [];

        for (let i = 0; i < points.length; i++) {
            let curvature = 0;

            if (i > 0 && i < points.length - 1) {
                // Central difference for heading rate
                const dHeading = this.wrapAngle(points[i + 1].heading - points[i - 1].heading);
                const dx = points[i + 1].x - points[i - 1].x;
                const dy = points[i + 1].y - points[i - 1].y;
                const ds = Math.sqrt(dx * dx + dy * dy);

                if (ds > 0.5) {
                    curvature = Math.abs(dHeading / ds);
                }
            }

            result.push({
                ...points[i],
                curvature,
                isCorner: curvature > this.cornerCurvature
            });
        }

        return result;
    }

    curvaturePreservingDownsample(points, maxPoints) {
        if (points.length <= maxPoints) {
            return points.map(p => ({
                x: p.x,
                y: p.y,
                heading: p.heading,
                curvature: p.curvature,
                confidence: 1.0 / (p.sigma + 0.5),
                lapCount: 1
            }));
        }

        // Score each point by importance (curvature)
        const scored = points.map((p, i) => ({
            point: p,
            index: i,
            score: p.curvature || 0
        }));

        // Always keep first and last
        const kept = new Set([0, points.length - 1]);

        // Keep top curvature points (corners) - 30% of budget
        const cornerBudget = Math.floor(maxPoints * 0.3);
        const sortedByScore = scored.slice(1, -1).sort((a, b) => b.score - a.score);
        for (let i = 0; i < Math.min(cornerBudget, sortedByScore.length); i++) {
            kept.add(sortedByScore[i].index);
        }

        // Fill remaining with evenly spaced points
        const remaining = maxPoints - kept.size;
        const step = Math.floor(points.length / remaining);
        for (let i = 0; i < points.length && kept.size < maxPoints; i += step) {
            kept.add(i);
        }

        // Return in original order
        return Array.from(kept)
            .sort((a, b) => a - b)
            .map(i => ({
                x: points[i].x,
                y: points[i].y,
                heading: points[i].heading,
                curvature: points[i].curvature,
                confidence: 1.0 / (points[i].sigma + 0.5),
                lapCount: 1
            }));
    }

    calculateTimingLine(centerline, which = 'start') {
        // Use first or last N points depending on which line
        const windowSize = Math.min(10, Math.floor(centerline.length * 0.02));
        const window = which === 'start'
            ? centerline.slice(0, windowSize)
            : centerline.slice(-windowSize);

        // Weighted average position and heading (favor low sigma)
        let sumX = 0, sumY = 0, sumWeight = 0;
        let sumSin = 0, sumCos = 0;

        for (const p of window) {
            const weight = p.confidence || 0.5;
            sumX += p.x * weight;
            sumY += p.y * weight;
            sumSin += Math.sin(p.heading) * weight;
            sumCos += Math.cos(p.heading) * weight;
            sumWeight += weight;
        }

        const centerX = sumX / sumWeight;
        const centerY = sumY / sumWeight;
        const direction = Math.atan2(sumSin / sumWeight, sumCos / sumWeight);

        // For finish line, direction should be reversed (entering from opposite side)
        const lineDirection = which === 'finish'
            ? this.wrapAngle(direction + Math.PI)
            : direction;

        // Perpendicular direction for line endpoints
        const perpAngle = direction + Math.PI / 2;
        const halfWidth = 12;  // 24m total width

        return {
            p1: [
                centerX + Math.cos(perpAngle) * halfWidth,
                centerY + Math.sin(perpAngle) * halfWidth
            ],
            p2: [
                centerX - Math.cos(perpAngle) * halfWidth,
                centerY - Math.sin(perpAngle) * halfWidth
            ],
            direction: lineDirection
        };
    }

    calculateBounds(centerline, margin = 30) {
        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;

        for (const p of centerline) {
            minX = Math.min(minX, p.x);
            maxX = Math.max(maxX, p.x);
            minY = Math.min(minY, p.y);
            maxY = Math.max(maxY, p.y);
        }

        return {
            minX: minX - margin,
            maxX: maxX + margin,
            minY: minY - margin,
            maxY: maxY + margin
        };
    }

    calculatePathLength(centerline) {
        let total = 0;
        for (let i = 1; i < centerline.length; i++) {
            const dx = centerline[i].x - centerline[i - 1].x;
            const dy = centerline[i].y - centerline[i - 1].y;
            total += Math.sqrt(dx * dx + dy * dy);
        }
        return total;
    }

    cancel() {
        this.recording = false;
        this.rawSamples = [];
        this.keyPoints = [];
    }

    getStats() {
        return {
            recording: this.recording,
            trackType: this.trackType,
            pointCount: this.keyPoints.length,
            rawSampleCount: this.rawSamples.length,
            totalDistance: this.totalDistance,
            loopDetected: this.loopDetected,
            gpsQuality: this.lastSigma < 2.5 ? 'good' : (this.lastSigma < 4 ? 'fair' : 'poor'),
            corners: this.keyPoints.filter(p => p.isCorner).length
        };
    }

    wrapAngle(a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
```

### Recording UI Overlay

```html
<div class="bbRecordOverlay" id="record-overlay" style="display:none">
    <div class="bbRecordHeader">
        <div class="bbRecordIndicator"></div>
        <span class="bbRecordTitle">Recording Track</span>
        <span class="bbRecordType" id="rec-type">Loop</span>
    </div>

    <div class="bbRecordStats">
        <div class="bbRecordStat">
            <span class="bbRecordValue" id="rec-distance">0m</span>
            <span class="bbRecordLabel">Distance</span>
        </div>
        <div class="bbRecordStat">
            <span class="bbRecordValue" id="rec-corners">0</span>
            <span class="bbRecordLabel">Corners</span>
        </div>
        <div class="bbRecordStat">
            <span class="bbRecordValue" id="rec-points">0</span>
            <span class="bbRecordLabel">Points</span>
        </div>
    </div>

    <div class="bbRecordQuality">
        <span class="bbRecordQualityLabel">GPS Quality:</span>
        <div class="bbRecordQualityBar">
            <div class="bbRecordQualityFill" id="rec-quality-fill"></div>
        </div>
        <span class="bbRecordQualityText" id="rec-quality-text">Good</span>
    </div>

    <div class="bbRecordInstructions" id="rec-instructions">
        Drive around your circuit once. Return to starting point to complete.
    </div>

    <div class="bbRecordLoop" id="loop-detected" style="display:none">
        <div class="bbLoopIcon">✓</div>
        <div class="bbLoopMessage">Loop detected!</div>
        <div class="bbLoopQuality" id="loop-quality">Quality: Good</div>
    </div>

    <div class="bbRecordButtons">
        <button class="bbRecordBtn" id="btn-cancel-recording">Cancel</button>
        <button class="bbRecordBtn primary" id="btn-finish-recording" style="display:none">
            Save Track
        </button>
        <button class="bbRecordBtn" id="btn-continue-recording" style="display:none">
            Continue
        </button>
        <!-- P2P only -->
        <button class="bbRecordBtn primary" id="btn-mark-finish" style="display:none">
            Mark Finish
        </button>
    </div>
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
    box-shadow: 0 -4px 20px rgba(0,0,0,0.2);
    z-index: 200;
}

.bbRecordHeader {
    display: flex;
    align-items: center;
    gap: 10px;
    margin-bottom: 16px;
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
    50% { opacity: 0.4; }
}

.bbRecordTitle {
    font-weight: 600;
    font-size: 16px;
}

.bbRecordType {
    background: var(--text);
    color: var(--surface);
    padding: 2px 8px;
    border-radius: 4px;
    font-size: 12px;
    font-weight: 500;
}

.bbRecordStats {
    display: flex;
    justify-content: space-around;
    margin-bottom: 16px;
}

.bbRecordStat {
    text-align: center;
}

.bbRecordValue {
    font-size: 24px;
    font-weight: 600;
    font-variant-numeric: tabular-nums;
    display: block;
}

.bbRecordLabel {
    font-size: 12px;
    opacity: 0.6;
}

.bbRecordQuality {
    display: flex;
    align-items: center;
    gap: 10px;
    margin-bottom: 16px;
}

.bbRecordQualityLabel {
    font-size: 13px;
    opacity: 0.7;
}

.bbRecordQualityBar {
    flex: 1;
    height: 6px;
    background: rgba(128,128,128,0.2);
    border-radius: 3px;
    overflow: hidden;
}

.bbRecordQualityFill {
    height: 100%;
    background: #34c759;
    transition: width 0.3s, background 0.3s;
}

.bbRecordQualityFill.fair { background: #ff9500; }
.bbRecordQualityFill.poor { background: #ff3b30; }

.bbRecordQualityText {
    font-size: 13px;
    font-weight: 500;
    min-width: 40px;
}

.bbRecordInstructions {
    text-align: center;
    font-size: 14px;
    opacity: 0.7;
    margin-bottom: 16px;
}

.bbRecordLoop {
    text-align: center;
    padding: 16px;
    background: rgba(52, 199, 89, 0.1);
    border-radius: 12px;
    margin-bottom: 16px;
}

.bbLoopIcon {
    font-size: 32px;
    color: #34c759;
    margin-bottom: 8px;
}

.bbLoopMessage {
    font-size: 18px;
    font-weight: 600;
    color: #34c759;
}

.bbLoopQuality {
    font-size: 13px;
    opacity: 0.7;
    margin-top: 4px;
}

.bbRecordButtons {
    display: flex;
    gap: 10px;
}

.bbRecordBtn {
    flex: 1;
    padding: 14px;
    border: none;
    border-radius: 10px;
    font-size: 16px;
    font-weight: 600;
    cursor: pointer;
    background: rgba(128,128,128,0.15);
    color: var(--text);
}

.bbRecordBtn.primary {
    background: #007aff;
    color: white;
}

.bbRecordBtn.danger {
    color: #ff3b30;
}
```

### Integration with Track Manager

```javascript
// Add to track manager modal
let trackRecorder = null;
let diagnosticsInterval = null;

function startTrackRecording(trackType) {
    if (!currentPos || !currentPos.valid) {
        alert('GPS position not available. Please wait for GPS lock.');
        return;
    }

    trackRecorder = new TrackRecorder();
    trackRecorder.start({
        x: currentPos.x,
        y: currentPos.y,
        heading: currentPos.yaw,
        speed: currentPos.speed / 3.6,  // Convert km/h to m/s
        sigma: lastDiagnostics?.ekf?.pos_sigma || 3.0
    }, trackType);

    // Show recording overlay
    $('record-overlay').style.display = 'block';
    $('rec-type').textContent = trackType === 'loop' ? 'Loop' : 'Point-to-Point';
    $('rec-instructions').textContent = trackType === 'loop'
        ? 'Drive around your circuit once. Return to starting point to complete.'
        : 'Drive from start to finish. Tap "Mark Finish" when done.';

    // Show appropriate buttons
    $('btn-mark-finish').style.display = trackType === 'point_to_point' ? 'block' : 'none';

    // Close track manager modal
    $('track-modal').style.display = 'none';

    // Poll diagnostics for GPS quality
    diagnosticsInterval = setInterval(fetchDiagnosticsForRecording, 1000);
}

async function fetchDiagnosticsForRecording() {
    try {
        const r = await fetch('/api/diagnostics');
        const d = await r.json();
        lastDiagnostics = d;
    } catch (e) {
        // Ignore fetch errors during recording
    }
}

// Called from telemetry update loop
function updateTrackRecording() {
    if (!trackRecorder || !trackRecorder.recording) return;
    if (!currentPos || !currentPos.valid) return;

    const result = trackRecorder.addSample({
        x: currentPos.x,
        y: currentPos.y,
        heading: currentPos.yaw,
        speed: currentPos.speed / 3.6,
        sigma: lastDiagnostics?.ekf?.pos_sigma || 3.0
    });

    // Update UI
    const stats = result.stats;
    $('rec-distance').textContent = formatDistance(stats.totalDistance);
    $('rec-corners').textContent = stats.corners;
    $('rec-points').textContent = stats.pointCount;

    // GPS quality indicator
    const quality = stats.gpsQuality;
    const qualityPercent = quality === 'good' ? 90 : (quality === 'fair' ? 60 : 30);
    $('rec-quality-fill').style.width = qualityPercent + '%';
    $('rec-quality-fill').className = 'bbRecordQualityFill ' + quality;
    $('rec-quality-text').textContent = quality.charAt(0).toUpperCase() + quality.slice(1);

    // Loop detection
    if (result.loopDetected) {
        $('loop-detected').style.display = 'block';
        $('loop-quality').textContent = 'Quality: ' +
            result.loopQuality.rating.charAt(0).toUpperCase() +
            result.loopQuality.rating.slice(1);
        $('btn-finish-recording').style.display = 'block';
        $('btn-continue-recording').style.display = 'block';
        $('rec-instructions').style.display = 'none';
    }
}

function finishTrackRecording() {
    if (!trackRecorder) return;

    clearInterval(diagnosticsInterval);

    const trackName = prompt('Enter track name:', 'My Track');
    if (!trackName) {
        // User cancelled - keep recording
        return;
    }

    try {
        const track = trackRecorder.finish(trackName, {
            lat: lastGpsLat,
            lon: lastGpsLon
        });

        // Save to IndexedDB
        saveTrack(track).then(() => {
            // Activate the new track
            activateTrack(track);

            // Hide recording overlay
            $('record-overlay').style.display = 'none';
            trackRecorder = null;
        });
    } catch (e) {
        alert('Error saving track: ' + e.message);
    }
}

function cancelTrackRecording() {
    if (trackRecorder) {
        trackRecorder.cancel();
    }
    trackRecorder = null;
    clearInterval(diagnosticsInterval);
    $('record-overlay').style.display = 'none';
}

// P2P: Mark finish
function markFinishLine() {
    if (!trackRecorder || trackRecorder.trackType !== 'point_to_point') return;

    // Manually trigger finish UI
    $('loop-detected').style.display = 'block';
    $('loop-detected').querySelector('.bbLoopIcon').textContent = '🏁';
    $('loop-detected').querySelector('.bbLoopMessage').textContent = 'Finish marked!';
    $('loop-quality').textContent = 'Quality: ' + trackRecorder.assessQuality().rating;
    $('btn-finish-recording').style.display = 'block';
    $('btn-mark-finish').style.display = 'none';
    $('rec-instructions').style.display = 'none';
}
```

---

## Phase 7B: Track Learning ❌ NOT STARTED

### Goal
Improve track accuracy over multiple laps by averaging GPS positions. Each lap refines the centerline, reducing noise and improving timing line precision.

### How It Works

1. **During each lap**, position observations are matched to the stored centerline
2. **Running average** updates each centerline point with new observations
3. **Confidence increases** as more laps are recorded
4. **GPS quality weighting** gives more influence to low-sigma samples

### Benefits
- Single lap recording has ~2m GPS noise
- After 10 laps: noise reduced to ~0.6m (√10 improvement)
- Timing line position becomes more accurate
- Corners become sharper and more precise

### Implementation: `TrackLearner` Class

```javascript
class TrackLearner {
    constructor(track) {
        this.track = track;
        this.enabled = track.learningEnabled !== false;

        // Initialize learning state from centerline
        this.centerline = track.centerline.map(p => ({
            ...p,
            sumX: p.x * (p.lapCount || 1),
            sumY: p.y * (p.lapCount || 1),
            sumHeadingSin: Math.sin(p.heading) * (p.lapCount || 1),
            sumHeadingCos: Math.cos(p.heading) * (p.lapCount || 1),
            sumWeight: p.lapCount || 1
        }));

        // Build spatial index for fast lookups
        this.gridSize = 15;  // 15m cells
        this.grid = new Map();
        this.buildSpatialIndex();

        // Stats for current lap
        this.observationsThisLap = 0;
        this.lastObservationTime = 0;
    }

    buildSpatialIndex() {
        this.grid.clear();
        for (let i = 0; i < this.centerline.length; i++) {
            const p = this.centerline[i];
            const cellX = Math.floor(p.x / this.gridSize);
            const cellY = Math.floor(p.y / this.gridSize);
            const key = `${cellX},${cellY}`;

            if (!this.grid.has(key)) this.grid.set(key, []);
            this.grid.get(key).push(i);
        }
    }

    /**
     * Record an observation during a lap
     * @param {number} x - Current X position
     * @param {number} y - Current Y position
     * @param {number} heading - Current heading (radians)
     * @param {number} sigma - Position uncertainty (meters)
     * @param {number} speed - Current speed (m/s)
     */
    recordObservation(x, y, heading, sigma, speed) {
        if (!this.enabled) return;

        // Skip if poor GPS or too slow (might be reversing/stopped)
        if (sigma > 4.0 || speed < 2.0) return;

        // Rate limit observations (max ~10Hz)
        const now = Date.now();
        if (now - this.lastObservationTime < 100) return;
        this.lastObservationTime = now;

        // Find nearest centerline point
        const nearest = this.findNearestPoint(x, y);
        if (!nearest || nearest.distance > 15) return;  // Too far from track

        // Update the centerline point
        const point = this.centerline[nearest.index];
        const weight = 1.0 / (sigma + 0.5);  // Higher weight for better GPS

        point.sumX += x * weight;
        point.sumY += y * weight;
        point.sumHeadingSin += Math.sin(heading) * weight;
        point.sumHeadingCos += Math.cos(heading) * weight;
        point.sumWeight += weight;
        point.lapCount++;

        // Update position (running weighted average)
        point.x = point.sumX / point.sumWeight;
        point.y = point.sumY / point.sumWeight;
        point.heading = Math.atan2(
            point.sumHeadingSin / point.sumWeight,
            point.sumHeadingCos / point.sumWeight
        );

        // Update confidence based on lap count and consistency
        const expectedCount = point.lapCount;
        point.confidence = Math.min(1.0, 0.3 + 0.07 * expectedCount);

        this.observationsThisLap++;
    }

    findNearestPoint(x, y) {
        const cellX = Math.floor(x / this.gridSize);
        const cellY = Math.floor(y / this.gridSize);

        let best = null;
        let bestDist = Infinity;

        // Search 3x3 neighborhood
        for (let dx = -1; dx <= 1; dx++) {
            for (let dy = -1; dy <= 1; dy++) {
                const key = `${cellX + dx},${cellY + dy}`;
                const indices = this.grid.get(key) || [];

                for (const i of indices) {
                    const p = this.centerline[i];
                    const dist = Math.sqrt((x - p.x) ** 2 + (y - p.y) ** 2);

                    if (dist < bestDist) {
                        bestDist = dist;
                        best = { index: i, distance: dist, point: p };
                    }
                }
            }
        }

        return best;
    }

    /**
     * Finalize learning at end of lap
     * @returns {Object} Updated track
     */
    finalizeLap() {
        // Rebuild spatial index with updated positions
        this.buildSpatialIndex();

        // Update track centerline
        this.track.centerline = this.centerline.map(p => ({
            x: p.x,
            y: p.y,
            heading: p.heading,
            curvature: p.curvature,
            confidence: p.confidence,
            lapCount: p.lapCount
        }));

        // Update track metadata
        this.track.lastLearnedLap = Date.now();
        this.track.modified = Date.now();

        // Optionally recalculate timing line after significant learning
        const avgLapCount = this.centerline.reduce((s, p) => s + p.lapCount, 0) / this.centerline.length;
        if (avgLapCount >= 3 && avgLapCount % 3 === 0) {
            // Recalculate timing line every 3 laps
            this.track.startLine = this.recalculateTimingLine('start');
            if (this.track.type === 'point_to_point') {
                this.track.finishLine = this.recalculateTimingLine('finish');
            }
        }

        const result = {
            track: this.track,
            stats: this.getStats(),
            observationsThisLap: this.observationsThisLap
        };

        // Reset for next lap
        this.observationsThisLap = 0;

        return result;
    }

    recalculateTimingLine(which) {
        const windowSize = Math.min(10, Math.floor(this.centerline.length * 0.02));
        const window = which === 'start'
            ? this.centerline.slice(0, windowSize)
            : this.centerline.slice(-windowSize);

        let sumX = 0, sumY = 0, sumWeight = 0;
        let sumSin = 0, sumCos = 0;

        for (const p of window) {
            const weight = p.confidence || 0.5;
            sumX += p.x * weight;
            sumY += p.y * weight;
            sumSin += Math.sin(p.heading) * weight;
            sumCos += Math.cos(p.heading) * weight;
            sumWeight += weight;
        }

        const centerX = sumX / sumWeight;
        const centerY = sumY / sumWeight;
        const direction = Math.atan2(sumSin / sumWeight, sumCos / sumWeight);

        const lineDirection = which === 'finish'
            ? this.wrapAngle(direction + Math.PI)
            : direction;

        const perpAngle = direction + Math.PI / 2;
        const halfWidth = 12;

        return {
            p1: [centerX + Math.cos(perpAngle) * halfWidth, centerY + Math.sin(perpAngle) * halfWidth],
            p2: [centerX - Math.cos(perpAngle) * halfWidth, centerY - Math.sin(perpAngle) * halfWidth],
            direction: lineDirection
        };
    }

    getStats() {
        const avgConfidence = this.centerline.reduce((s, p) => s + p.confidence, 0) / this.centerline.length;
        const avgLapCount = this.centerline.reduce((s, p) => s + p.lapCount, 0) / this.centerline.length;
        const lowConfidencePoints = this.centerline.filter(p => p.confidence < 0.5).length;

        return {
            avgConfidence,
            avgLapCount: Math.round(avgLapCount * 10) / 10,
            lowConfidencePoints,
            totalPoints: this.centerline.length,
            learningEnabled: this.enabled
        };
    }

    setEnabled(enabled) {
        this.enabled = enabled;
        this.track.learningEnabled = enabled;
    }

    wrapAngle(a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
```

### Learning UI (in Track Manager)

```html
<div class="bbTrackLearning" id="track-learning">
    <div class="bbLearningHeader">
        <span class="bbLearningTitle">Track Learning</span>
        <label class="bbLearningToggle">
            <input type="checkbox" id="learning-enabled" checked>
            <span class="bbToggleSlider"></span>
        </label>
    </div>

    <div class="bbLearningStats">
        <div class="bbLearningStat">
            <span class="bbLearningValue" id="learning-confidence">—</span>
            <span class="bbLearningLabel">Confidence</span>
        </div>
        <div class="bbLearningStat">
            <span class="bbLearningValue" id="learning-laps">—</span>
            <span class="bbLearningLabel">Laps Learned</span>
        </div>
    </div>

    <div class="bbLearningBar">
        <div class="bbLearningBarFill" id="learning-bar"></div>
    </div>

    <div class="bbLearningHint" id="learning-hint">
        Drive more laps to improve accuracy
    </div>

    <button class="bbLearningReset" id="btn-reset-learning">Reset Learning</button>
</div>
```

```css
.bbTrackLearning {
    background: rgba(128,128,128,0.1);
    border-radius: 12px;
    padding: 16px;
    margin-top: 16px;
}

.bbLearningHeader {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 12px;
}

.bbLearningTitle {
    font-weight: 600;
}

.bbLearningToggle {
    position: relative;
    width: 44px;
    height: 24px;
}

.bbLearningToggle input {
    opacity: 0;
    width: 0;
    height: 0;
}

.bbToggleSlider {
    position: absolute;
    cursor: pointer;
    top: 0; left: 0; right: 0; bottom: 0;
    background: rgba(128,128,128,0.3);
    border-radius: 12px;
    transition: 0.2s;
}

.bbToggleSlider:before {
    position: absolute;
    content: "";
    height: 20px;
    width: 20px;
    left: 2px;
    bottom: 2px;
    background: white;
    border-radius: 50%;
    transition: 0.2s;
}

.bbLearningToggle input:checked + .bbToggleSlider {
    background: #34c759;
}

.bbLearningToggle input:checked + .bbToggleSlider:before {
    transform: translateX(20px);
}

.bbLearningStats {
    display: flex;
    justify-content: space-around;
    margin-bottom: 12px;
}

.bbLearningStat {
    text-align: center;
}

.bbLearningValue {
    font-size: 20px;
    font-weight: 600;
    display: block;
}

.bbLearningLabel {
    font-size: 11px;
    opacity: 0.6;
}

.bbLearningBar {
    height: 6px;
    background: rgba(128,128,128,0.2);
    border-radius: 3px;
    overflow: hidden;
    margin-bottom: 8px;
}

.bbLearningBarFill {
    height: 100%;
    background: linear-gradient(90deg, #ff9500, #34c759);
    transition: width 0.3s;
}

.bbLearningHint {
    font-size: 12px;
    opacity: 0.6;
    text-align: center;
    margin-bottom: 12px;
}

.bbLearningReset {
    width: 100%;
    padding: 10px;
    border: none;
    border-radius: 8px;
    background: rgba(255, 59, 48, 0.1);
    color: #ff3b30;
    font-weight: 500;
    cursor: pointer;
}
```

### Integration

```javascript
let trackLearner = null;

// When track is activated
function activateTrack(track) {
    // ... existing activation code ...

    // Initialize learner if track has centerline
    if (track.centerline && track.centerline.length > 0) {
        trackLearner = new TrackLearner(track);
        updateLearningUI(trackLearner.getStats());
    } else {
        trackLearner = null;
    }
}

// During telemetry updates (when timing)
function updateLapTiming() {
    // ... existing timing code ...

    // Record learning observation during lap
    if (trackLearner && isTimingActive) {
        trackLearner.recordObservation(
            currentPos.x,
            currentPos.y,
            currentPos.yaw,
            lastDiagnostics?.ekf?.pos_sigma || 3.0,
            currentPos.speed / 3.6
        );
    }
}

// When lap completes
function onLapComplete(lapTimeMs) {
    // ... existing lap complete code ...

    // Finalize learning
    if (trackLearner) {
        const result = trackLearner.finalizeLap();

        // Save updated track to IndexedDB
        saveTrack(result.track);

        // Update ESP32 with refined timing line
        if (result.stats.avgLapCount >= 3) {
            sendTimingLineToESP32(result.track);
        }

        updateLearningUI(result.stats);
    }
}

function updateLearningUI(stats) {
    if (!stats) return;

    $('learning-confidence').textContent = Math.round(stats.avgConfidence * 100) + '%';
    $('learning-laps').textContent = stats.avgLapCount.toFixed(1);
    $('learning-bar').style.width = (stats.avgConfidence * 100) + '%';

    if (stats.avgConfidence > 0.8) {
        $('learning-hint').textContent = 'Track well learned!';
    } else if (stats.avgLapCount < 3) {
        $('learning-hint').textContent = 'Drive more laps to improve accuracy';
    } else {
        $('learning-hint').textContent = 'Learning in progress...';
    }
}
```

---

## Phase 8: Track Auto-Detection (Enhanced) ❌ NOT STARTED

### Goal
Automatically detect when user is near a saved track and offer to activate it.

### Enhanced Detection

Uses centerline matching (not just bounding box) for more accurate detection:

```javascript
class TrackAutoDetector {
    constructor(trackDb) {
        this.trackDb = trackDb;
        this.currentTrack = null;
        this.candidateTrack = null;
        this.candidateConfidence = 0;
        this.lastCheckTime = 0;
        this.checkInterval = 3000;  // Check every 3 seconds
        this.enabled = true;
    }

    async checkPosition(x, y, heading) {
        const now = Date.now();
        if (now - this.lastCheckTime < this.checkInterval) return null;
        this.lastCheckTime = now;

        if (this.currentTrack || !this.enabled) return null;

        const tracks = await this.trackDb.getAllTracks();
        const candidates = [];

        for (const track of tracks) {
            // Quick bounds check first
            if (!track.bounds || !this.isWithinBounds(x, y, track.bounds)) continue;

            // Detailed centerline match
            const match = this.matchCenterline(x, y, heading, track);
            if (match && match.score > 0.4) {
                candidates.push({ track, ...match });
            }
        }

        if (candidates.length === 0) {
            this.candidateTrack = null;
            this.candidateConfidence = 0;
            return null;
        }

        // Best match
        candidates.sort((a, b) => b.score - a.score);
        const best = candidates[0];

        // Require consistent detection (2+ checks)
        if (this.candidateTrack?.id === best.track.id) {
            this.candidateConfidence += 0.4;
        } else {
            this.candidateTrack = best.track;
            this.candidateConfidence = 0.4;
        }

        if (this.candidateConfidence >= 0.8) {
            return {
                track: best.track,
                confidence: this.candidateConfidence,
                distance: best.distance
            };
        }

        return null;
    }

    matchCenterline(x, y, heading, track) {
        if (!track.centerline || track.centerline.length === 0) return null;

        // Find nearest centerline point
        let bestDist = Infinity;
        let bestPoint = null;

        for (const p of track.centerline) {
            const dist = Math.sqrt((x - p.x) ** 2 + (y - p.y) ** 2);
            if (dist < bestDist) {
                bestDist = dist;
                bestPoint = p;
            }
        }

        if (!bestPoint || bestDist > 50) return null;  // Too far

        // Score based on distance and heading alignment
        const distanceScore = Math.max(0, 1 - bestDist / 50);
        const headingDiff = Math.abs(this.wrapAngle(heading - bestPoint.heading));
        const headingScore = Math.max(0, 1 - headingDiff / Math.PI);

        const score = distanceScore * 0.6 + headingScore * 0.4;

        return { distance: bestDist, score };
    }

    isWithinBounds(x, y, bounds) {
        const margin = 100;  // Extra margin for detection
        return x >= bounds.minX - margin && x <= bounds.maxX + margin &&
               y >= bounds.minY - margin && y <= bounds.maxY + margin;
    }

    setCurrentTrack(track) {
        this.currentTrack = track;
        this.candidateTrack = null;
        this.candidateConfidence = 0;
    }

    clearCurrentTrack() {
        this.currentTrack = null;
    }

    wrapAngle(a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
```

### Auto-Detection UI

```javascript
function showTrackDetectedPrompt(detection) {
    const toast = document.createElement('div');
    toast.className = 'bbTrackDetectedToast';
    toast.innerHTML = `
        <div class="bbToastContent">
            <div class="bbToastIcon">📍</div>
            <div class="bbToastText">
                <div class="bbToastTitle">Track detected</div>
                <div class="bbToastName">${detection.track.name}</div>
            </div>
        </div>
        <div class="bbToastActions">
            <button class="bbToastBtn" onclick="dismissTrackDetection()">Dismiss</button>
            <button class="bbToastBtn primary" onclick="activateDetectedTrack()">Activate</button>
        </div>
    `;

    document.body.appendChild(toast);
    detectedTrackPending = detection.track;

    // Auto-dismiss after 10 seconds
    setTimeout(() => {
        if (toast.parentNode) {
            toast.parentNode.removeChild(toast);
            detectedTrackPending = null;
        }
    }, 10000);
}
```

```css
.bbTrackDetectedToast {
    position: fixed;
    top: 20px;
    left: 20px;
    right: 20px;
    background: var(--surface);
    border-radius: 12px;
    padding: 16px;
    box-shadow: 0 4px 20px rgba(0,0,0,0.2);
    z-index: 300;
    animation: slideDown 0.3s ease-out;
}

@keyframes slideDown {
    from { transform: translateY(-100%); opacity: 0; }
    to { transform: translateY(0); opacity: 1; }
}

.bbToastContent {
    display: flex;
    align-items: center;
    gap: 12px;
    margin-bottom: 12px;
}

.bbToastIcon {
    font-size: 24px;
}

.bbToastTitle {
    font-size: 13px;
    opacity: 0.7;
}

.bbToastName {
    font-size: 16px;
    font-weight: 600;
}

.bbToastActions {
    display: flex;
    gap: 10px;
}

.bbToastBtn {
    flex: 1;
    padding: 10px;
    border: none;
    border-radius: 8px;
    font-weight: 500;
    cursor: pointer;
    background: rgba(128,128,128,0.15);
    color: var(--text);
}

.bbToastBtn.primary {
    background: #007aff;
    color: white;
}
```

---

## Phase 9: Reference Lap & Predictive Delta (Enhanced) ❌ NOT STARTED

### Goal
Real-time delta calculation comparing current lap to best lap, using arc-length parameterization for accuracy.

### Why Arc-Length?

Position-based matching has problems:
- Driver takes different lines through corners
- GPS noise causes false position matches
- Doesn't work well on straights (many points at same position)

**Arc-length** (distance traveled) is more robust:
- At 500m into the lap, compare times regardless of exact position
- Works even if driver takes different lines
- More stable delta display

### Enhanced Reference Lap Structure

```javascript
// Reference lap stored in IndexedDB
{
    id: string,
    trackId: string,
    lapTimeMs: number,
    created: timestamp,
    isBest: boolean,

    // Samples with arc-length
    // Format: [arcLength:f32, x:f32, y:f32, timestamp:u32] × N
    // 16 bytes per sample, ~60KB for 2-minute lap at 25Hz
    samples: ArrayBuffer,

    // Metadata
    avgSpeed: number,
    maxSpeed: number,
    sampleCount: number
}
```

### Implementation: `DeltaCalculator` Class

```javascript
class DeltaCalculator {
    constructor() {
        this.reference = null;
        this.refArcLengths = [];

        // Current lap state
        this.currentArcLength = 0;
        this.lastPos = null;
        this.isActive = false;
    }

    setReferenceLap(samplesBuffer) {
        this.reference = this.decodeSamples(samplesBuffer);
        if (this.reference.length > 0) {
            this.refArcLengths = this.computeArcLengths(this.reference);
        }
    }

    decodeSamples(buffer) {
        const view = new DataView(buffer);
        const samples = [];

        for (let i = 0; i < buffer.byteLength; i += 16) {
            samples.push({
                arcLength: view.getFloat32(i, true),
                x: view.getFloat32(i + 4, true),
                y: view.getFloat32(i + 8, true),
                t: view.getUint32(i + 12, true)
            });
        }

        return samples;
    }

    computeArcLengths(samples) {
        return samples.map(s => s.arcLength);
    }

    startLap() {
        this.currentArcLength = 0;
        this.lastPos = null;
        this.isActive = true;
    }

    /**
     * Update with current position and get delta
     * @returns {Object|null} { deltaMs, method, confidence }
     */
    update(x, y, currentLapTimeMs) {
        if (!this.isActive || !this.reference || this.reference.length < 10) {
            return null;
        }

        // Update arc length (cumulative distance)
        if (this.lastPos) {
            const dx = x - this.lastPos.x;
            const dy = y - this.lastPos.y;
            this.currentArcLength += Math.sqrt(dx * dx + dy * dy);
        }
        this.lastPos = { x, y };

        // Find reference time at same arc length
        const refTime = this.interpolateTime(this.currentArcLength);
        if (refTime === null) return null;

        const deltaMs = currentLapTimeMs - refTime;

        return {
            deltaMs,
            arcLength: this.currentArcLength,
            method: 'arc_length',
            confidence: 'high'
        };
    }

    interpolateTime(arcLength) {
        if (this.refArcLengths.length === 0) return null;

        // Clamp to valid range
        const maxArc = this.refArcLengths[this.refArcLengths.length - 1];
        if (arcLength > maxArc) {
            // Past end of reference - extrapolate
            return this.reference[this.reference.length - 1].t;
        }

        // Binary search for bracket
        let lo = 0, hi = this.refArcLengths.length - 1;

        while (lo < hi - 1) {
            const mid = Math.floor((lo + hi) / 2);
            if (this.refArcLengths[mid] < arcLength) {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        // Linear interpolation
        const arcLo = this.refArcLengths[lo];
        const arcHi = this.refArcLengths[hi];
        const t = (arcLength - arcLo) / (arcHi - arcLo);

        return this.reference[lo].t + t * (this.reference[hi].t - this.reference[lo].t);
    }

    endLap() {
        this.isActive = false;
    }

    clear() {
        this.reference = null;
        this.refArcLengths = [];
        this.currentArcLength = 0;
        this.lastPos = null;
        this.isActive = false;
    }
}
```

### Lap Recorder (for creating reference laps)

```javascript
class LapRecorder {
    constructor() {
        this.samples = [];
        this.arcLength = 0;
        this.lastPos = null;
        this.recording = false;
    }

    startLap() {
        this.samples = [];
        this.arcLength = 0;
        this.lastPos = null;
        this.recording = true;
    }

    addSample(x, y, lapTimeMs) {
        if (!this.recording) return;

        // Update arc length
        if (this.lastPos) {
            const dx = x - this.lastPos.x;
            const dy = y - this.lastPos.y;
            this.arcLength += Math.sqrt(dx * dx + dy * dy);
        }
        this.lastPos = { x, y };

        // Sample at ~25Hz (every ~40ms)
        const lastT = this.samples.length > 0
            ? this.samples[this.samples.length - 1].t
            : -40;

        if (lapTimeMs - lastT >= 40) {
            this.samples.push({
                arcLength: this.arcLength,
                x, y,
                t: lapTimeMs
            });
        }
    }

    finishLap() {
        this.recording = false;
        return this.encodeSamples();
    }

    encodeSamples() {
        // Pack: [arcLength:f32, x:f32, y:f32, timestamp:u32] × N
        const buffer = new ArrayBuffer(this.samples.length * 16);
        const view = new DataView(buffer);

        for (let i = 0; i < this.samples.length; i++) {
            const offset = i * 16;
            view.setFloat32(offset, this.samples[i].arcLength, true);
            view.setFloat32(offset + 4, this.samples[i].x, true);
            view.setFloat32(offset + 8, this.samples[i].y, true);
            view.setUint32(offset + 12, this.samples[i].t, true);
        }

        return buffer;
    }

    cancel() {
        this.recording = false;
        this.samples = [];
    }
}
```

### Delta Display

```javascript
function updateLiveDelta(deltaResult) {
    const el = $('lap-delta');

    if (!deltaResult) {
        el.textContent = '—';
        el.className = 'bbLapHistValue bbNum delta';
        return;
    }

    const { deltaMs } = deltaResult;
    const sign = deltaMs >= 0 ? '+' : '';
    const seconds = Math.abs(deltaMs / 1000).toFixed(2);

    el.textContent = `${sign}${deltaMs >= 0 ? '' : '-'}${seconds}`;

    // Color coding with smooth transitions
    if (deltaMs < -500) {
        el.className = 'bbLapHistValue bbNum delta much-faster';
    } else if (deltaMs < -100) {
        el.className = 'bbLapHistValue bbNum delta faster';
    } else if (deltaMs > 500) {
        el.className = 'bbLapHistValue bbNum delta much-slower';
    } else if (deltaMs > 100) {
        el.className = 'bbLapHistValue bbNum delta slower';
    } else {
        el.className = 'bbLapHistValue bbNum delta neutral';
    }
}
```

```css
.delta.much-faster { color: #00d26a; font-weight: 700; }
.delta.faster { color: #34c759; }
.delta.neutral { color: var(--text); opacity: 0.7; }
.delta.slower { color: #ff9500; }
.delta.much-slower { color: #ff3b30; font-weight: 700; }
```

---

## Phase 10: Session History ❌ NOT STARTED

### Goal
Track lap times across sessions, display session history, and manage reference laps.

### Enhanced Session Structure

```javascript
{
    id: string,
    trackId: string,
    date: timestamp,

    // Session conditions (optional)
    conditions: {
        weather: 'dry' | 'wet' | 'damp' | null,
        temperature: number | null,  // Celsius
        notes: string | null
    },

    laps: [
        {
            lapTimeMs: number,
            deltaMs: number | null,     // vs best at time
            isValid: boolean,
            invalidReason: string | null,
            maxSpeed: number,           // km/h
            avgSpeed: number            // km/h
        },
        ...
    ],

    bestLapMs: number | null,
    totalLaps: number,
    totalDistance: number,  // meters

    // Learning contribution
    learningApplied: boolean,
    pointsUpdated: number
}
```

### Session Manager

```javascript
class SessionManager {
    constructor(trackDb) {
        this.trackDb = trackDb;
        this.currentSession = null;
    }

    startSession(trackId, conditions = {}) {
        this.currentSession = {
            id: crypto.randomUUID(),
            trackId,
            date: Date.now(),
            conditions,
            laps: [],
            bestLapMs: null,
            totalLaps: 0,
            totalDistance: 0,
            learningApplied: false,
            pointsUpdated: 0
        };
    }

    recordLap(lapData) {
        if (!this.currentSession) return;

        const { lapTimeMs, deltaMs, isValid = true, invalidReason = null, maxSpeed, avgSpeed, distance } = lapData;

        this.currentSession.laps.push({
            lapTimeMs,
            deltaMs,
            isValid,
            invalidReason,
            maxSpeed,
            avgSpeed
        });

        if (isValid) {
            this.currentSession.totalLaps++;
            this.currentSession.totalDistance += distance || 0;

            if (!this.currentSession.bestLapMs || lapTimeMs < this.currentSession.bestLapMs) {
                this.currentSession.bestLapMs = lapTimeMs;
            }
        }
    }

    setLearningStats(applied, pointsUpdated) {
        if (this.currentSession) {
            this.currentSession.learningApplied = applied;
            this.currentSession.pointsUpdated = pointsUpdated;
        }
    }

    async endSession() {
        if (!this.currentSession) return null;

        if (this.currentSession.laps.length > 0) {
            await this.trackDb.saveSession(this.currentSession);
        }

        const session = this.currentSession;
        this.currentSession = null;
        return session;
    }

    getCurrentSession() {
        return this.currentSession;
    }
}
```

### Session History UI

```html
<div class="bbSessionsSection" id="sessions-section">
    <div class="bbSectHeader">
        <span>Session History</span>
        <button class="bbSectToggle" id="btn-toggle-sessions">▼</button>
    </div>

    <div class="bbSessionsList" id="sessions-list">
        <!-- Populated dynamically -->
    </div>

    <div class="bbSessionsEmpty" id="sessions-empty">
        No sessions recorded yet
    </div>
</div>
```

```javascript
async function renderSessionHistory(trackId) {
    const sessions = await trackDb.getSessionsForTrack(trackId);

    if (sessions.length === 0) {
        $('sessions-list').style.display = 'none';
        $('sessions-empty').style.display = 'block';
        return;
    }

    $('sessions-list').style.display = 'block';
    $('sessions-empty').style.display = 'none';

    // Sort by date, newest first
    sessions.sort((a, b) => b.date - a.date);

    // Show last 10 sessions
    const recent = sessions.slice(0, 10);

    $('sessions-list').innerHTML = recent.map(session => `
        <div class="bbSessionItem" onclick="showSessionDetail('${session.id}')">
            <div class="bbSessionLeft">
                <div class="bbSessionDate">${formatSessionDate(session.date)}</div>
                <div class="bbSessionLaps">${session.totalLaps} ${session.totalLaps === 1 ? 'lap' : 'laps'}</div>
            </div>
            <div class="bbSessionRight">
                <div class="bbSessionBest">${fmtLapTime(session.bestLapMs)}</div>
                <div class="bbSessionLabel">best</div>
            </div>
        </div>
    `).join('');
}

function formatSessionDate(timestamp) {
    const d = new Date(timestamp);
    const now = new Date();
    const diff = now - d;

    if (diff < 86400000 && d.getDate() === now.getDate()) {
        return 'Today, ' + d.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
    } else if (diff < 172800000) {
        return 'Yesterday, ' + d.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
    } else {
        return d.toLocaleDateString([], { month: 'short', day: 'numeric' }) +
               ', ' + d.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
    }
}
```

```css
.bbSessionsSection {
    margin-top: 20px;
}

.bbSectHeader {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 12px 0;
    border-bottom: 1px solid rgba(128,128,128,0.2);
    font-weight: 600;
}

.bbSectToggle {
    background: none;
    border: none;
    font-size: 12px;
    opacity: 0.5;
    cursor: pointer;
}

.bbSessionItem {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 12px 0;
    border-bottom: 1px solid rgba(128,128,128,0.1);
    cursor: pointer;
}

.bbSessionItem:active {
    background: rgba(128,128,128,0.1);
}

.bbSessionDate {
    font-weight: 500;
}

.bbSessionLaps {
    font-size: 12px;
    opacity: 0.6;
}

.bbSessionBest {
    font-size: 18px;
    font-weight: 600;
    font-variant-numeric: tabular-nums;
    text-align: right;
}

.bbSessionLabel {
    font-size: 11px;
    opacity: 0.5;
    text-align: right;
}

.bbSessionsEmpty {
    text-align: center;
    padding: 30px;
    opacity: 0.5;
}
```

---

## Phase 11: Polish & Testing ❌ NOT STARTED

### Edge Cases to Handle

1. **GPS Warmup**: Don't allow track creation/recording until GPS warmed up
   ```javascript
   if (!diagnostics?.gps?.warmup_complete) {
       showWarning('Waiting for GPS warmup...');
       return;
   }
   ```

2. **Poor GPS Quality**: Warn during recording if sigma too high
   ```javascript
   if (sigma > 5.0) {
       showQualityWarning('Poor GPS signal');
   }
   ```

3. **Track Boundary Exit**: Warn if vehicle leaves track bounds
   ```javascript
   if (!isWithinBounds(x, y, track.bounds)) {
       showWarning('Outside track area');
   }
   ```

4. **Power Loss / Page Refresh**:
   - Current lap is lost (acceptable)
   - Active track ID persisted in localStorage
   - Session data saved on each lap completion

5. **Multiple Crossings**: Handled by firmware debounce (500ms)

6. **Reverse Direction**: Handled by firmware direction validation (±90°)

7. **Coordinate System Reset**: Compare GPS lat/lon to stored origin
   ```javascript
   const drift = gpsDistance(lat, lon, track.gpsOrigin.lat, track.gpsOrigin.lon);
   if (drift > 100) showWarning('GPS origin mismatch');
   ```

### Quality Indicators

**GPS Quality:**
| Sigma | Rating | Color |
|-------|--------|-------|
| < 2m | Excellent | Green |
| 2-4m | Good | Green |
| 4-5m | Fair | Yellow |
| > 5m | Poor | Red |

**Track Confidence:**
| Laps | Confidence |
|------|------------|
| 1 | 30% |
| 5 | 65% |
| 10 | 80% |
| 20+ | 95% |

### Testing with Sample Data

**Test Data Files (`data/sample-tracks/`):**

The existing `austin_raceline.csv` provides good test data. Additional test files would help:

1. **`austin_with_timestamps.csv`** - With timing for delta tests
   ```csv
   t_ms,x_m,y_m,heading_rad,speed_mps
   0,-2.84,-0.96,1.57,0
   40,-2.75,-0.90,1.56,8.5
   ```

2. **`austin_noisy.csv`** - With GPS-like noise (σ=1.5m)
   ```csv
   x_m,y_m,sigma_m
   -2.84,0.96,1.8
   ```

3. **`simple_oval.csv`** - Small test track
   ```csv
   x_m,y_m
   0,0
   100,0
   150,50
   100,100
   0,100
   -50,50
   0,0
   ```

**Automated Tests:**

```javascript
// Test 1: Track Recorder
async function testTrackRecorder() {
    const recorder = new TrackRecorder();
    const path = loadCSV('simple_oval.csv');

    recorder.start(path[0], 'loop');
    for (const p of path) recorder.addSample(p);

    const track = recorder.finish('Test', origin);
    assert(track.centerline.length > 10);
    assert(track.startLine);
}

// Test 2: Corner Preservation
async function testCornerPreservation() {
    // COTA has ~20 corners
    const track = recordTrack('austin_raceline.csv');
    const corners = track.centerline.filter(p => p.curvature > 0.02);
    assert(corners.length >= 15);
}

// Test 3: Delta Accuracy
async function testDelta() {
    const calc = new DeltaCalculator();
    calc.setReferenceLap(refLap);  // 90 seconds

    // Simulate 5% slower lap
    for (const p of refLap) {
        const delta = calc.update(p.x, p.y, p.t * 1.05);
        assert(Math.abs(delta.deltaMs - p.t * 0.05) < 500);
    }
}

// Test 4: Learning Convergence
async function testLearning() {
    const learner = new TrackLearner(noisyTrack);

    for (let lap = 0; lap < 10; lap++) {
        for (const p of truePath) learner.recordObservation(p.x, p.y, p.heading, 2.0, 30);
        learner.finalizeLap();
    }

    assert(learner.getStats().avgConfidence > 0.7);
}
```

### Manual Test Checklist

**Track Recording:**
- [ ] GPS quality indicator shows correctly
- [ ] Distance counter increments
- [ ] Corner counter detects turns
- [ ] Loop detection triggers at start
- [ ] P2P finish marking works
- [ ] Track saves to IndexedDB

**Track Learning:**
- [ ] Confidence increases per lap
- [ ] Reset clears learning data
- [ ] Timing line refines after laps

**Lap Timing:**
- [ ] Timer starts/stops on crossings
- [ ] Best lap updates correctly
- [ ] Delta displays during lap
- [ ] Flash on crossing
- [ ] New best animation

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
