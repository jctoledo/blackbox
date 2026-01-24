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

## Phase 7: Track Recording ❌ NOT STARTED

### Goal
Allow user to define a track by driving it once. Supports both loop and point-to-point tracks.

### User Flow

**Loop Track Recording:**
1. User taps "Record Track by Driving" and selects "Loop"
2. Dashboard shows recording overlay with instructions
3. User drives around their intended circuit
4. Dashboard detects loop closure (returned near start)
5. User confirms or continues driving
6. Dashboard calculates timing line automatically from start/end heading
7. User names and saves the track

**Point-to-Point Track Recording:**
1. User taps "Record Track by Driving" and selects "Point-to-Point"
2. Dashboard shows recording overlay: "Drive from start to finish"
3. User drives the route (no loop closure detection)
4. User taps "Mark Finish" when done
5. Dashboard calculates start line from initial heading, finish line from final heading
6. User names and saves the track

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
