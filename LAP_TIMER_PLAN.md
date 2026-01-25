# Lap Timer Implementation Plan

## Overview

Real-time lap timing with predictive delta display for the Blackbox dashboard. The goal is to match commercial solutions like VBOX LapTimer ($1,160) and AiM Solo 2 DL ($700) at a fraction of the cost.

**What Makes a Lap Timer Useful:**
1. **Accurate timing** - Know your lap time to millisecond precision ✅ DONE
2. **Delta to best** - Know if you're ahead or behind at every point on track ✅ DONE
3. **Progress tracking** - See improvement across sessions ❌ NOT DONE

Delta is THE killer feature - now implemented!

**Target Use Cases:**
- Neighborhood block circuits for practice
- Track days (autocross, road course, time attack)
- Point-to-point (hill climbs, rally stages, touge)
- Personal best tracking across sessions

---

## Architecture

### Hybrid Firmware/Browser Design
- **ESP32 (firmware)**: Line crossing detection at 30 Hz for timing precision
- **Browser (dashboard)**: Storage (IndexedDB), delta calculation, track management, UI

### Why Hybrid?
- Firmware has precise timing (no JavaScript GC pauses)
- Browser has unlimited storage and rich UI capabilities
- Delta calculation is CPU-intensive, better on browser than ESP32

### Dashboard Synchronization

**CRITICAL**: `tools/dashboard-dev/index.html` and production dashboard in `sensors/blackbox/src/websocket_server.rs` MUST stay synchronized.

**Workflow:**
1. Develop in `dashboard-dev/index.html` with simulation
2. Test thoroughly
3. Port to `websocket_server.rs` production dashboard
4. Verify both behave identically

---

## Implementation Status

| Phase | Description | Status | Priority |
|-------|-------------|--------|----------|
| 1-7 | Core Timing, UI, Track Recording | ✅ COMPLETE | - |
| 8 | Reference Lap & Predictive Delta | ✅ COMPLETE | - |
| 8B | Delta Bar UX Polish | ✅ COMPLETE | - |
| 9 | Data Management & Cleanup | ✅ COMPLETE | - |
| 10 | Session History | ❌ NOT STARTED | MEDIUM |
| 11 | Track Auto-Detection | ❌ NOT STARTED | LOW |
| 12 | Track Learning | ❌ NOT STARTED | LOW |
| 13 | Robustness & Edge Cases | ❌ NOT STARTED | LOW |
| 14 | Documentation | ❌ NOT STARTED | LOW |
| 15 | UX Improvements | ❌ NOT STARTED | MEDIUM |

---

## Completed Work Summary

### Firmware (Phases 1-2) ✅
- `LapTimer` struct with configurable start/finish lines
- Line crossing detection with direction validation (±90°)
- 500ms debounce prevents double-counting
- Telemetry packet: 74 bytes with lap_time_ms, lap_count, flags

### Dashboard UI (Phases 3-5) ✅
- Lap timer card: current time, lap count, best/last
- Mode states: Armed → Timing → Finished
- Track modal: saved tracks list, activate/delete
- Circuit (loop) and Stage (point-to-point) types
- Start/finish line approach indicators with distance/direction

### Track Recording (Phases 6-7) ✅
- `TrackRecorder` class with adaptive GPS sampling
- Corner detection via curvature state machine (hysteresis)
- Loop closure detection (proximity + heading alignment)
- IndexedDB persistence (`blackbox-tracks` database)
- Recording overlay with live GPS quality, distance, corners

### Reference Lap & Predictive Delta (Phase 8) ✅
- `LapTracker` class: tracks cumulative distance and samples during lap
- Arc-length based delta calculation with binary search + linear interpolation
- Reference lap storage in IndexedDB (`reference_laps` object store)
- Best lap automatically saved as new reference
- Delta bar UI: racing-game inspired design with text above, bar below
- Color coding: green (ahead), red (behind), glow effect for large deltas (>1s)
- Trend arrows: ▲ gaining time, ▼ losing time (EMA-smoothed derivative)
- Works for both loop tracks and point-to-point stages

### Delta Bar UX Polish (Phase 8B) ✅
- Bar scaling: 2000ms = full bar, graceful handling of larger deltas
- Demo tracks: ephemeral references (not persisted to IndexedDB)
- Speed variance simulation: creates realistic lap-to-lap variation for testing
- Lap count display: always shows at least "Lap 1" (no confusing "Lap 0")
- P2P terminology: "Run" instead of "Lap", "Running" instead of "Timing"
- "Finished!" animation on P2P completion
- Production and dashboard-dev synchronized

---

## Phase 8: Reference Lap & Predictive Delta ✅

**Completed!** Drivers can now see real-time delta to their best lap at every point on track.

### What Was Built

- **LapTracker class**: Accumulates distance traveled during lap, stores samples for potential reference
- **Arc-length delta**: Binary search + interpolation finds reference time at current distance
- **Delta bar UI**: Racing-game inspired design with large delta text and visual bar
- **Auto-save reference**: Best lap automatically becomes the new reference
- **IndexedDB storage**: `reference_laps` object store with `trackId` index
- **Demo mode**: Speed variance simulation creates realistic lap-to-lap differences

### The Problem (Solved)

Previously, the driver only saw their lap time after crossing the finish. They had no idea during the lap if they're ahead or behind their best. This is like running a race blindfolded - you only see results at the end.

### The Solution: Arc-Length Delta

**Why arc-length instead of position matching?**

Position matching fails because:
- Driver takes different lines through corners
- GPS noise (±2m) causes false matches
- On straights, many reference points are at similar positions

Arc-length (cumulative distance traveled) is robust:
- At 500m into the lap, compare times regardless of exact position
- Works even if driver takes different racing lines
- More stable delta display, less jumping around

### Data Structures

**Reference Lap (stored in IndexedDB):**
```javascript
// New object store: 'reference_laps' in blackbox-tracks DB
{
  id: string,
  trackId: string,           // Foreign key to tracks store
  lapTimeMs: number,         // Total lap time
  created: timestamp,

  // Efficient binary format: Float32Array
  // Every sample: [distance_m, time_ms] as float pairs
  // At 30 Hz for 2-min lap = 3600 samples × 8 bytes = 29 KB
  samples: ArrayBuffer,

  // Metadata for quick filtering
  sampleCount: number,
  totalDistance: number      // For validation
}
```

**Current Lap Tracking (in memory):**
```javascript
class LapTracker {
  constructor() {
    this.distance = 0;           // Cumulative distance this lap
    this.lastPos = null;         // Previous position for distance calc
    this.samples = [];           // [{distance, time}, ...] for potential new reference
    this.startTime = null;
  }
}
```

### Implementation Steps

**Step 1: Capture position samples during lap**
```javascript
// Called every telemetry update while timing
function onTelemetryUpdate(pos, lapTimeMs) {
  if (!timing) return;

  // Calculate distance traveled since last update
  if (lapTracker.lastPos) {
    const dx = pos.x - lapTracker.lastPos.x;
    const dy = pos.y - lapTracker.lastPos.y;
    const dist = Math.sqrt(dx*dx + dy*dy);

    // Sanity check: reject teleports (GPS glitch)
    if (dist < 50) {  // Max 50m between samples at 30Hz = 1500 m/s = impossible
      lapTracker.distance += dist;
    }
  }
  lapTracker.lastPos = { x: pos.x, y: pos.y };

  // Store sample for potential new reference lap
  // Downsample to ~10 Hz to save memory (every 3rd sample)
  if (lapTracker.samples.length === 0 ||
      lapTimeMs - lapTracker.samples[lapTracker.samples.length-1].time > 100) {
    lapTracker.samples.push({
      distance: lapTracker.distance,
      time: lapTimeMs
    });
  }

  // Calculate delta if we have a reference lap
  if (referenceLap) {
    const delta = calculateDelta(lapTracker.distance, lapTimeMs, referenceLap);
    updateDeltaDisplay(delta);
  }
}
```

**Step 2: Calculate delta using binary search**
```javascript
function calculateDelta(currentDistance, currentTime, refLap) {
  // Binary search to find reference sample at same distance
  const samples = new Float32Array(refLap.samples);
  const n = samples.length / 2;  // pairs of [distance, time]

  // Find bracketing samples
  let lo = 0, hi = n - 1;
  while (hi - lo > 1) {
    const mid = Math.floor((lo + hi) / 2);
    const refDist = samples[mid * 2];
    if (refDist < currentDistance) lo = mid;
    else hi = mid;
  }

  // Interpolate reference time at current distance
  const d0 = samples[lo * 2], t0 = samples[lo * 2 + 1];
  const d1 = samples[hi * 2], t1 = samples[hi * 2 + 1];

  if (d1 === d0) return { deltaMs: currentTime - t0, valid: true };

  const frac = (currentDistance - d0) / (d1 - d0);
  const refTime = t0 + frac * (t1 - t0);

  return {
    deltaMs: currentTime - refTime,  // Positive = behind, negative = ahead
    valid: true,
    refTime: refTime
  };
}
```

**Step 3: Save new best lap as reference**
```javascript
async function onLapComplete(lapTimeMs, isNewBest) {
  if (isNewBest && lapTracker.samples.length > 10) {
    // Convert samples to binary format
    const buffer = new ArrayBuffer(lapTracker.samples.length * 8);
    const view = new Float32Array(buffer);
    lapTracker.samples.forEach((s, i) => {
      view[i * 2] = s.distance;
      view[i * 2 + 1] = s.time;
    });

    // Delete old reference for this track
    await deleteReferenceLap(activeTrack.id);

    // Save new reference
    await saveReferenceLap({
      id: crypto.randomUUID(),
      trackId: activeTrack.id,
      lapTimeMs: lapTimeMs,
      created: Date.now(),
      samples: buffer,
      sampleCount: lapTracker.samples.length,
      totalDistance: lapTracker.distance
    });
  }

  // Reset for next lap
  lapTracker.reset();
}
```

**Step 4: Load reference lap when track activated**
```javascript
async function activateTrack(track) {
  // ... existing activation code ...

  // Load reference lap if exists
  referenceLap = await getReferenceLap(track.id);
  if (referenceLap) {
    console.log(`Loaded reference lap: ${fmtLapTime(referenceLap.lapTimeMs)}`);
  }
}
```

### Delta Display UI

**Location:** Below the main lap time, or as overlay on G-meter

**Design:**
```
┌─────────────────────────┐
│      1:23.456          │  ← Current lap time
│      -2.3s             │  ← Delta (green if negative/ahead)
│      ▼▼▼               │  ← Trend indicator (gaining/losing)
└─────────────────────────┘
```

**Color coding:**
- Green: Ahead of reference (negative delta)
- Red: Behind reference (positive delta)
- White/Gray: No reference lap yet

**Trend indicator:**
Track delta over last 3-5 seconds. If delta is decreasing (gaining time), show ▲. If increasing (losing time), show ▼.

### Edge Cases

1. **No reference lap yet**: Show "No best lap" instead of delta
2. **First lap**: Can't show delta, just record samples
3. **Lap invalid** (wrong direction, cut short): Don't save as reference
4. **Distance mismatch**: If current distance > reference total distance, stop showing delta (lap is longer, maybe went off track)
5. **Coordinate reset**: If GPS origin shifts between sessions, reference lap is invalid. Detect via total distance mismatch (>10% difference = invalid).

### Testing ✅

1. **Simulation test**: Dashboard-dev has speed variance simulation that creates realistic lap-to-lap differences (0.87x-1.15x speed modifiers)
2. **Delta verification**: Console logging verified delta matches actual lap time differences
3. **Demo tracks**: Two built-in demo tracks (loop and P2P) for testing without GPS
4. **Visual verification**: Delta bar, trend arrows, and color coding all verified working

### Implementation Notes

- **Bar scaling**: MAX_DELTA_MS = 2000ms gives good visual progression for 0-3s deltas
- **Trend threshold**: 3ms derivative threshold works well for simulated variance
- **Glow effect**: Triggers at 1000ms delta for dramatic visual feedback
- **Lap count**: Always shows minimum "Lap 1" to avoid confusing "Lap 0" display

---

## Phase 9: Data Management & Cleanup ✅

### The Problem

The `blackbox-rec` database stores telemetry recording sessions. Currently:
- Sessions accumulate forever
- No UI to view or delete old sessions
- Export only works on latest session
- Storage could eventually fill up

### Solution

**New UI: Unified Data Modal**

Accessible via "Data" menu item in hamburger menu:

```
┌─────────────────────────────────────┐
│ Data                            [×] │
├─────────────────────────────────────┤
│ Total Storage: 15.8 MB              │
│   Recordings: 12.3 MB | Tracks: 3.5 MB │
├─────────────────────────────────────┤
│ Recordings (5)                      │
│ ┌─────────────────────────────────┐ │
│ │ Today 2:34 PM                   │ │
│ │ 12 chunks • 4.2 MB • 45m        │ │
│ │ ● Complete     [Export][Delete] │ │
│ ├─────────────────────────────────┤ │
│ │ Yesterday 6:12 PM               │ │
│ │ 8 chunks • 2.1 MB • 28m         │ │
│ │ ● Complete     [Export][Delete] │ │
│ └─────────────────────────────────┘ │
│ [Clear All Recordings]              │
├─────────────────────────────────────┤
│ Tracks & Best Laps (3)              │
│ ┌─────────────────────────────────┐ │
│ │ Thunderhill East                │ │
│ │ 12 corners • 156 pts • 1.2 MB   │ │
│ │ Best: 1:42.354 (892 pts)        │ │
│ │         [Clear Best] [Delete]   │ │
│ ├─────────────────────────────────┤ │
│ │ Streets of Willow               │ │
│ │ 8 corners • 98 pts • 0.8 MB     │ │
│ │ No best lap                     │ │
│ │         [Clear Best] [Delete]   │ │
│ └─────────────────────────────────┘ │
│ [Clear All Tracks]                  │
└─────────────────────────────────────┘
```

### Implementation

**Step 1: Get storage usage**
```javascript
async function getStorageStats() {
  const sessions = await getAllSessions();
  let totalSize = 0;

  for (const session of sessions) {
    const chunks = await getChunksForSession(session.sessionId);
    // Estimate size: each chunk's data array
    for (const chunk of chunks) {
      totalSize += chunk.data.length * 50;  // ~50 bytes per record estimate
    }
  }

  return {
    sessionCount: sessions.length,
    totalBytes: totalSize,
    sessions: sessions.map(s => ({
      id: s.sessionId,
      date: s.startTime,
      status: s.status,
      chunkCount: s.chunks || 0
    }))
  };
}
```

**Step 2: Delete session and its chunks**
```javascript
async function deleteSession(sessionId) {
  const tx = db.transaction(['sessions', 'chunks'], 'readwrite');

  // Delete all chunks for this session
  const chunks = tx.objectStore('chunks');
  const index = chunks.index('sessionId');  // Need to add this index
  const range = IDBKeyRange.only(sessionId);

  let cursor = await index.openCursor(range);
  while (cursor) {
    cursor.delete();
    cursor = await cursor.continue();
  }

  // Delete session record
  tx.objectStore('sessions').delete(sessionId);

  await tx.done;
}
```

**Step 3: Export any session (not just latest)**
```javascript
async function exportSession(sessionId) {
  const tx = db.transaction(['sessions', 'chunks'], 'readonly');
  const session = await tx.objectStore('sessions').get(sessionId);
  const allChunks = await tx.objectStore('chunks').getAll();

  const sessionChunks = allChunks
    .filter(c => c.sessionId === sessionId)
    .sort((a, b) => a.chunkIndex - b.chunkIndex);

  // ... rest of export logic (same as current exportCSV) ...
}
```

**Step 4: Add sessionId index to chunks store**

Need to bump DB version and add index in upgrade handler:
```javascript
const DB_VERSION = 2;  // Bump from 1

// In onupgradeneeded:
if (oldVersion < 2) {
  const chunks = db.objectStoreNames.contains('chunks')
    ? tx.objectStore('chunks')
    : db.createObjectStore('chunks', { keyPath: ['sessionId', 'chunkIndex'] });

  if (!chunks.indexNames.contains('sessionId')) {
    chunks.createIndex('sessionId', 'sessionId', { unique: false });
  }
}
```

### Edge Cases

1. **Delete while recording**: Prevent deletion of active session
2. **Orphaned chunks**: Clean up chunks with no matching session
3. **Storage quota**: Handle QuotaExceededError gracefully

### Implementation Summary (Completed)

**Unified Data Modal** - Renamed from "Recordings" to "Data" with expanded scope:

1. **Storage Overview**
   - Total storage used across all IndexedDB data
   - Breakdown: Recordings vs Tracks storage

2. **Recordings Section**
   - List all recording sessions with date, chunks, size, duration
   - Status indicators: Recording, Complete, Recovered
   - Per-session actions: Export, Delete
   - Clear All Recordings button

3. **Tracks & Best Laps Section** (NEW)
   - List all saved tracks with corners, points, size
   - Reference lap info: Best time, sample count
   - Per-track actions: Clear Best, Delete Track
   - Clear All Tracks button

4. **UX Improvements**
   - Removed redundant "Tracks" from hamburger menu (still accessible from lap timer card)
   - "Data" menu item opens unified data management modal
   - Both dashboard-dev and production websocket_server.rs updated

5. **New Functions Added**
   - `getTrackDataStats()` - Get track info with reference lap details
   - `getCombinedStorageStats()` - Combined recordings + tracks stats
   - `deleteTrackWithRef()` - Delete track and its reference lap
   - `clearAllTracks()` - Clear all track data
   - `formatLapTimeMs()` - Format lap times (m:ss.ms)
   - `formatSessionDuration()` - Format durations (Xh Xm)
   - `estimateObjectSize()` - Estimate IndexedDB object sizes

---

## Phase 10: Session History ❌

### The Problem

Currently there's no record of lap times across sessions. If you did 20 laps last weekend, that data is gone (unless you exported CSV and analyzed manually).

### Solution

Track lap history per track, viewable in the track modal.

### Data Structure

**New object store: 'lap_history' in blackbox-tracks DB**
```javascript
{
  id: string,                    // crypto.randomUUID()
  trackId: string,               // Foreign key
  timestamp: number,             // When lap was completed
  lapTimeMs: number,
  lapNumber: number,             // Which lap in this session (1, 2, 3...)
  sessionDate: number,           // Date.now() at session start (groups laps)

  // Optional metadata
  isValid: boolean,              // false if wrong direction, cut short
  maxSpeedKmh: number | null,
  conditions: string | null      // "dry", "wet", etc. (future)
}
```

### Implementation

**Step 1: Record lap on completion**
```javascript
// In updateLapTimer when LAP_FLAG_NEW_LAP detected
async function recordLapHistory(lapTimeMs, lapNumber) {
  if (!activeTrack) return;

  const lap = {
    id: crypto.randomUUID(),
    trackId: activeTrack.id,
    timestamp: Date.now(),
    lapTimeMs: lapTimeMs,
    lapNumber: lapNumber,
    sessionDate: currentSessionStart,  // Set when timing starts
    isValid: true
  };

  await saveLapHistory(lap);
}
```

**Step 2: Display in track modal**

When viewing a track's details:
```
┌─────────────────────────────┐
│ My Block Loop               │
│ Best: 1:23.456              │
│ 47 laps total               │
├─────────────────────────────┤
│ Recent Sessions             │
│ ┌─────────────────────────┐ │
│ │ Today                   │ │
│ │ Best: 1:24.2  Laps: 12  │ │
│ │ 1:24.2 1:25.1 1:24.8... │ │
│ ├─────────────────────────┤ │
│ │ Yesterday               │ │
│ │ Best: 1:23.4  Laps: 8   │ │
│ │ 1:25.0 1:24.1 1:23.4... │ │
│ └─────────────────────────┘ │
│ [View All History]          │
└─────────────────────────────┘
```

**Step 3: Query lap history**
```javascript
async function getLapHistoryForTrack(trackId, limit = 100) {
  const tx = db.transaction('lap_history', 'readonly');
  const index = tx.objectStore('lap_history').index('trackId');
  const laps = await index.getAll(trackId);

  // Sort by timestamp descending
  laps.sort((a, b) => b.timestamp - a.timestamp);

  // Group by sessionDate
  const sessions = new Map();
  for (const lap of laps.slice(0, limit)) {
    const dateKey = new Date(lap.sessionDate).toDateString();
    if (!sessions.has(dateKey)) {
      sessions.set(dateKey, {
        date: lap.sessionDate,
        laps: [],
        bestMs: Infinity
      });
    }
    const session = sessions.get(dateKey);
    session.laps.push(lap);
    if (lap.lapTimeMs < session.bestMs) session.bestMs = lap.lapTimeMs;
  }

  return Array.from(sessions.values());
}
```

---

## Phase 11: Track Auto-Detection ❌

### The Problem

User has to manually open track modal and select their track every time. If they're at a track they've saved before, the system should offer to activate it automatically.

### Solution

Periodically check if current position is near a saved track's centerline. If consistent match, show toast offering to activate.

### Implementation

```javascript
class TrackAutoDetector {
  constructor() {
    this.lastCheck = 0;
    this.checkInterval = 3000;  // Check every 3 seconds
    this.candidateTrack = null;
    this.matchCount = 0;
    this.requiredMatches = 2;   // Need 2 consecutive matches
  }

  async check(x, y, heading) {
    const now = Date.now();
    if (now - this.lastCheck < this.checkInterval) return null;
    this.lastCheck = now;

    // Don't auto-detect if already have active track
    if (activeTrack) return null;

    const tracks = await getAllTracks();
    let bestMatch = null;
    let bestScore = 0;

    for (const track of tracks) {
      const score = this.scoreMatch(x, y, heading, track);
      if (score > bestScore && score > 0.5) {
        bestScore = score;
        bestMatch = track;
      }
    }

    if (bestMatch) {
      if (this.candidateTrack?.id === bestMatch.id) {
        this.matchCount++;
      } else {
        this.candidateTrack = bestMatch;
        this.matchCount = 1;
      }

      if (this.matchCount >= this.requiredMatches) {
        return { track: bestMatch, confidence: bestScore };
      }
    } else {
      this.candidateTrack = null;
      this.matchCount = 0;
    }

    return null;
  }

  scoreMatch(x, y, heading, track) {
    // Quick bounds check
    if (!track.bounds) return 0;
    const margin = 100;
    if (x < track.bounds.minX - margin || x > track.bounds.maxX + margin ||
        y < track.bounds.minY - margin || y > track.bounds.maxY + margin) {
      return 0;
    }

    // Find nearest centerline point
    let minDist = Infinity;
    let nearestPoint = null;
    for (const p of track.centerline || []) {
      const dist = Math.sqrt((x - p.x) ** 2 + (y - p.y) ** 2);
      if (dist < minDist) {
        minDist = dist;
        nearestPoint = p;
      }
    }

    if (!nearestPoint || minDist > 50) return 0;

    // Score based on distance and heading alignment
    const distScore = Math.max(0, 1 - minDist / 50);
    const headingDiff = Math.abs(wrapAngle(heading - nearestPoint.heading));
    const headingScore = Math.max(0, 1 - headingDiff / Math.PI);

    return distScore * 0.6 + headingScore * 0.4;
  }
}
```

**Toast UI:**
```javascript
function showTrackDetectedToast(track) {
  const toast = document.createElement('div');
  toast.className = 'bbToast';
  toast.innerHTML = `
    <div class="bbToastText">
      <strong>Track detected</strong><br>
      ${escHtml(track.name)}
    </div>
    <button onclick="activateDetectedTrack()">Activate</button>
    <button onclick="dismissToast()">✕</button>
  `;
  document.body.appendChild(toast);

  // Auto-dismiss after 10 seconds
  setTimeout(() => toast.remove(), 10000);
}
```

---

## Phase 12: Track Learning ❌

### The Problem

A single recording lap has ~2m GPS noise. The timing line position might be slightly off. Multiple laps could be averaged to improve accuracy.

### Solution

When completing laps on a track, update centerline points with weighted average of new observations. Timing line precision improves over time.

**Note:** This is a nice-to-have optimization, not critical for basic functionality.

### Implementation Sketch

```javascript
class TrackLearner {
  recordObservation(x, y, heading, sigma) {
    // Find nearest centerline point
    const nearest = findNearest(x, y);
    if (!nearest || nearest.distance > 10) return;

    // Weight by GPS quality (lower sigma = higher weight)
    const weight = 1 / (sigma + 0.5);

    // Update running average
    const p = nearest.point;
    p.sumX = (p.sumX || p.x) + x * weight;
    p.sumY = (p.sumY || p.y) + y * weight;
    p.sumWeight = (p.sumWeight || 1) + weight;
    p.x = p.sumX / p.sumWeight;
    p.y = p.sumY / p.sumWeight;
    p.lapCount = (p.lapCount || 1) + 1;
  }

  async finalizeLap() {
    // Recalculate timing line with improved centerline
    track.startLine = recalculateTimingLine(track.centerline, 'start');
    if (track.finishLine) {
      track.finishLine = recalculateTimingLine(track.centerline, 'finish');
    }
    await saveTrack(track);
  }
}
```

---

## Phase 13: Robustness & Edge Cases ❌

### GPS Warmup Check

Don't allow track recording or timing until GPS is warmed up:
```javascript
function canUseLapTimer() {
  // Check via diagnostics API or track gps_valid flag history
  if (!gpsWarmedUp) {
    showWarning('Waiting for GPS lock...');
    return false;
  }
  return true;
}
```

### Coordinate System Drift

If device is restarted, EKF origin resets. Saved tracks become invalid.

**Detection:**
```javascript
function validateTrackOrigin(track) {
  if (!track.gpsOrigin || !currentGpsOrigin) return true;  // Can't validate

  const drift = haversineDistance(
    track.gpsOrigin.lat, track.gpsOrigin.lon,
    currentGpsOrigin.lat, currentGpsOrigin.lon
  );

  if (drift > 100) {  // More than 100m drift
    showWarning('Track origin mismatch. Re-record track for best accuracy.');
    return false;
  }
  return true;
}
```

### Invalid Lap Detection

Laps that shouldn't count:
- Wrong direction crossing (already handled by firmware)
- Cut short (crossed finish without completing circuit)
- Off-track (position far from centerline)

### Power Loss Recovery

Store active track ID in localStorage:
```javascript
// On track activation
localStorage.setItem('activeTrackId', track.id);

// On page load
const savedTrackId = localStorage.getItem('activeTrackId');
if (savedTrackId) {
  const track = await getTrack(savedTrackId);
  if (track) activateTrack(track);
}
```

---

## Phase 14: Documentation ❌

Update README.md with:
- Lap timer feature overview
- Quick start guide (record track → drive → see times)
- Track types explained (Circuit vs Stage)
- Delta display explanation
- Troubleshooting (GPS warmup, coordinate drift)

Update CLAUDE.md with:
- Lap timer architecture in codebase guide
- Key files and their purposes
- Testing procedures

---

## Phase 15: UX Improvements ❌

Dedicated phase for user experience polish after core features are complete.

### Mobile & Touch Optimization

**Touch Targets:**
- Ensure all buttons are minimum 44x44px (iOS HIG) / 48x48dp (Material)
- Increase tap area for track list items and modal controls
- Add touch feedback (ripple/highlight) on interactive elements

**Viewport Handling:**
- Handle iOS Safari address bar resize gracefully
- Prevent zoom on double-tap for controls
- Lock orientation or adapt layout for landscape

**Gesture Support:**
- Swipe to dismiss modals
- Pull-to-refresh for track list (if applicable)
- Swipe between dashboard/diagnostics views

### Visual Polish

**Animations & Transitions:**
```css
/* Smooth modal entry/exit */
.bbModal { transition: opacity 0.2s, transform 0.2s; }
.bbModal.entering { opacity: 0; transform: translateY(20px); }

/* Delta bar value changes */
.deltaValue { transition: color 0.15s; }

/* Mode indicator transitions */
.modeIndicator { transition: background-color 0.2s; }
```

**Loading States:**
- Skeleton screens for track list while loading from IndexedDB
- Spinner overlay during track activation
- Progress indicator for CSV export

**Empty States:**
- Friendly illustration + message when no tracks saved
- "Record your first track" call-to-action button
- Empty lap history state with encouragement

### Feedback & Confirmation

**Toast Notifications:**
- Consistent styling across all toasts
- Queue multiple toasts (don't overlap)
- Swipe-to-dismiss on mobile
- Auto-dismiss with progress indicator

**Confirmation Dialogs:**
- "Delete track?" with track name displayed
- "Clear all recordings?" with count/size
- "Discard recording?" if leaving mid-record
- Destructive actions require explicit confirmation

**Progress Indicators:**
- Track recording: distance/corners/time elapsed
- CSV export: "Exporting... X of Y chunks"
- Reference lap save: brief "Saved as best lap" toast

### Accessibility

**Color & Contrast:**
- Verify WCAG AA contrast ratios (4.5:1 for text)
- Don't rely solely on color (add icons/patterns)
- Test with color blindness simulators

**Screen Readers:**
- Add `aria-label` to icon-only buttons
- Announce mode changes ("Now timing", "Lap complete")
- Live regions for delta updates

**Large Text Support:**
- Test with system font scaling
- Ensure layouts don't break at 150% text size
- Consider a "large display" mode for track use

### Onboarding & First-Use

**First Launch:**
```javascript
if (!localStorage.getItem('hasSeenOnboarding')) {
  showOnboardingOverlay();
  localStorage.setItem('hasSeenOnboarding', 'true');
}
```

**Onboarding Steps:**
1. "Welcome to Blackbox Lap Timer"
2. "Record a track by driving one lap"
3. "Then drive laps and see your times"
4. "Beat your best lap and watch the delta"

**Contextual Hints:**
- Tooltip on first track modal open: "Tap + to record a new track"
- Hint when first crossing start line: "Timing started!"
- Tip after first lap: "Drive more laps to improve your best"

### Settings UX

**Preset Selection:**
- Visual feedback when preset applied (flash/pulse)
- Show what changed ("Thresholds updated for Track mode")
- Preview threshold values before applying

**Custom Mode Sliders:**
- Show current value while dragging
- Snap to common values (optional)
- Reset to defaults button

**Validation Feedback:**
- Inline validation messages (not just blocked submit)
- Highlight invalid fields with red border
- Explain why value is invalid

### Error States

**Friendly Error Messages:**
| Technical | User-Friendly |
|-----------|---------------|
| IndexedDB error | "Couldn't save. Storage might be full." |
| GPS timeout | "Waiting for GPS signal..." |
| Track load failed | "Couldn't load track. Try refreshing." |

**Recovery Suggestions:**
- "GPS not ready" → "Move outside for better signal"
- "Track origin mismatch" → "Re-record track for best accuracy"
- "Storage full" → "Delete old recordings in Data Management"

### Performance Feel

**Optimistic Updates:**
- Show track as "saving..." immediately, confirm after IndexedDB write
- Update UI before async operations complete
- Roll back on failure with error message

**Perceived Speed:**
- Prioritize above-fold content rendering
- Defer non-critical JavaScript
- Lazy-load track centerlines (don't block list display)

**Smooth Scrolling:**
- Use CSS `scroll-behavior: smooth` for anchor links
- Momentum scrolling on track list
- Avoid layout shifts during scroll

### Implementation Checklist

**High Impact:**
- [ ] Touch target sizing audit and fixes
- [ ] Empty states for tracks and history
- [ ] Confirmation dialogs for destructive actions
- [ ] Loading states for async operations

**Medium Impact:**
- [ ] Modal animations (enter/exit)
- [ ] Toast notification queue
- [ ] First-use onboarding overlay
- [ ] Error message improvements

**Polish:**
- [ ] Skeleton screens
- [ ] Gesture support (swipe to dismiss)
- [ ] Accessibility audit (contrast, aria labels)
- [ ] Large text mode testing

---

## Technical Reference

### Timing Precision

- Firmware runs at 30 Hz → 33ms timing resolution
- At 100 km/h (28 m/s), position uncertainty per frame: 0.9m
- Could improve with crossing interpolation (future enhancement)

### Storage Estimates

| Data Type | Size per Item | Typical Count | Total |
|-----------|---------------|---------------|-------|
| Track | 5-20 KB | 10 tracks | 200 KB |
| Reference Lap | 30-60 KB | 10 tracks | 600 KB |
| Lap History | 100 bytes | 1000 laps | 100 KB |
| **Total** | | | **~1 MB** |

Well within IndexedDB limits (50+ MB).

### Corner Detection Algorithm

State machine with hysteresis:
```
Entry threshold:  0.025 rad/m (sharp turn)
Exit threshold:   0.012 rad/m (straightening out)
Minimum length:   3 meters

NOT_IN_CORNER → if curvature > 0.025 → IN_CORNER
IN_CORNER → if curvature < 0.012 for 3m → NOT_IN_CORNER, count++
```

### GPS Quality Thresholds

| Sigma (m) | Rating | Use for Recording? |
|-----------|--------|--------------------|
| < 2.0 | Excellent | Yes |
| 2.0 - 3.0 | Good | Yes |
| 3.0 - 4.0 | Fair | Yes (with warning) |
| 4.0 - 5.0 | Poor | No |
| > 5.0 | Bad | No |

### Loop Detection Thresholds

| Parameter | Value | Reason |
|-----------|-------|--------|
| Min track length | 150m | Avoid false triggers on U-turns |
| Close proximity | 25m | GPS uncertainty margin |
| Heading tolerance | 0.5 rad (30°) | Allow for different entry angles |
