# Blackbox Dashboard Design System

This document defines the visual language and implementation patterns for the Blackbox telemetry dashboard. Follow these guidelines exactly to maintain consistency across all UI work.

## Your Role (Design Context)

When working on UI for this project, you are a **racing telemetry interface designer** with expertise in:

**Visual Design:**
- Data visualization for real-time telemetry (G-meters, speedometers, trail plots)
- High-contrast interfaces optimized for peripheral vision at speed
- Color systems that communicate state changes instantly
- Typography that remains legible during vehicle motion

**Technical Implementation:**
- Embedded HTML/CSS/JS within Rust string literals (minified, single-file)
- Canvas-based rendering for smooth 30+ Hz updates
- EMA smoothing and time-based animations
- Mobile-first responsive layouts with `clamp()` and viewport units

**Automotive UX:**
- Glanceable information hierarchy (< 0.5s comprehension)
- Mode detection feedback that matches driver intuition
- Peak tracking and session statistics
- Recording and export workflows

---

## Design Philosophy

**"Tron meets racing telemetry"** - The interface should feel like a futuristic racing instrument cluster, not a consumer mobile app. Every element serves a data purpose. The UI itself communicates driving state through color and intensity shifts.

### Core Principles

1. **Data-first**: Speed and G-forces are the heroes. Everything else is subordinate.
2. **Glanceable**: Critical state visible in peripheral vision in < 0.5 seconds.
3. **State-responsive**: The entire UI shifts color based on acceleration/braking.
4. **No decoration without function**: Every glow, border, and animation communicates information.
5. **Mathematically grounded**: Layout uses golden ratio (φ). Smoothing uses proper EMA with time constants.

---

## Color System

### Primary Palette

```css
:root {
    --cyan: #00f0ff;     /* HSL(190, 100%, 50%) - accelerating, active, good */
    --amber: #ff6a00;    /* HSL(25, 100%, 50%) - braking, warning, peak/limit */
    --bg: #030508;       /* Near-black with slight blue tint, not pure #000 */
}
```

### Dynamic Hue System

The UI uses a CSS variable `--hue` that shifts based on driving state, allowing all themed elements to respond simultaneously:

```javascript
// In process() or simulate()
const isAccelerating = lng > 0.05;
currentHue = isAccelerating ? 190 : 25;  // Cyan : Amber
document.documentElement.style.setProperty('--hue', currentHue);
```

| Driving State | Hue Value | Resulting Color |
|---------------|-----------|-----------------|
| Accelerating | 190 | Cyan (#00f0ff) |
| Cruising/Idle | 190 | Cyan (#00f0ff) |
| Braking | 25 | Amber (#ff6a00) |
| At Peak Speed | 25 | Amber (#ff6a00) |

### Semantic Color Assignments

| Purpose | CSS Value | Hex Approximation | Usage |
|---------|-----------|-------------------|-------|
| Active data values | `hsl(var(--hue), 100%, 60%)` | Varies | Speed, current G values |
| Active borders | `hsla(var(--hue), 100%, 50%, 0.35)` | Varies | Box outlines |
| Active glow | `hsla(var(--hue), 100%, 50%, 0.12)` | Varies | Box shadows |
| Labels (dim) | `#556` to `#667` | — | Static text, axis labels |
| Values (muted) | `#5a5a6a` to `#7a8a8f` | — | Secondary numbers, units |
| Borders (inactive) | `#1a2a30` | — | Bottom box, subdued elements |
| Header gradient | `rgba(0,30,40,0.95)` → `rgba(0,20,30,0.8)` | — | Top bar background |
| Error/Recording | `#ff3333` / `#c66` | — | Recording indicator |
| Success/Valid | `#22c55e` | — | GPS lock, valid states |
| Warning | `var(--amber)` / `#ff6a00` | — | Diagnostic warnings |

### Glow Effects

Glow indicates activity level and emphasis. Never use glow purely decoratively.

```css
/* Status indicator - small, focused glow */
.dot.on {
    background: hsl(var(--hue), 100%, 50%);
    box-shadow: 0 0 10px hsl(var(--hue), 100%, 50%);
}

/* Container glow - subtle, ambient */
.golden-box {
    border: 1px solid hsla(var(--hue), 100%, 50%, 0.35);
    box-shadow: 0 0 20px hsla(var(--hue), 100%, 50%, 0.12);
}

/* Peak/limit state - intensified amber */
.at-peak {
    border-color: rgba(255, 106, 0, 0.45);
    box-shadow: 0 0 25px rgba(255, 106, 0, 0.15);
}

/* Speed text at peak */
.current-speed.at-peak {
    color: #ff6a00;
    text-shadow: 0 0 40px rgba(255, 106, 0, 0.3);
}
```

---

## Typography

### Font Stack

```css
font-family: 'Courier New', monospace;
```

Monospace is mandatory for:
- Tabular number alignment (no layout shift when values change)
- Retro-futuristic terminal aesthetic
- Consistent character width for live-updating displays

**Never use sans-serif or serif fonts in this UI.**

### Type Scale

| Element | Font Size | Weight | Letter-spacing | Additional |
|---------|-----------|--------|----------------|------------|
| Speed (hero) | `clamp(96px, 28vw, 160px)` | 600 | -1px | `line-height: 0.85` |
| Peak value | `clamp(20px, 5vw, 24px)` | 600 | 1px | tabular-nums |
| Mode text | `clamp(20px, 5vw, 24px)` | 600 | 3px | UPPERCASE |
| Speed unit | `clamp(20px, 5vw, 24px)` | 600 | 2px | lowercase |
| G-axis values | `clamp(14px, 3.5vw, 18px)` | 600 | — | tabular-nums |
| Metrics | 14px | 600 | — | tabular-nums |
| Labels | 8-10px | 500 | 1-3px | UPPERCASE |
| Logo | 14px | 400 | 4px | UPPERCASE |
| Timer | 11px | 400 | 2px | tabular-nums, opacity: 0.5 |

### Number Display Rules

**Always use tabular numerics:**
```css
font-variant-numeric: tabular-nums;
```

**Dead zone for zero display** (prevents -0.00 ↔ 0.00 flicker):
```javascript
function fmtG(v) {
    return Math.abs(v) < 0.005 ? '0.00' : v.toFixed(2);
}
```

**Speed display** (suppress sub-1 km/h noise):
```javascript
const dspd = speed_ema < 1 ? 0 : Math.round(speed_ema);
```

---

## Layout System

### Golden Ratio Structure

The main telemetry view divides vertical space using the golden ratio (φ = 1.6180339887):

```
┌─────────────────────────────────┐
│           HEADER                │  Fixed ~50px
├─────────────────────────────────┤
│                                 │
│        TOP BOX (Speed)          │  H / φ² ≈ 38.2%
│                                 │
├─────────────────────────────────┤
│                                 │
│      MIDDLE BOX (G-meter)       │  H / φ² ≈ 38.2%
│                                 │
├─────────────────────────────────┤
│      BOTTOM BOX (Controls)      │  H / φ³ ≈ 23.6%
└─────────────────────────────────┘
```

**Implementation:**
```javascript
const PHI = 1.6180339887;

function applyGoldenHeights() {
    const hdr = document.querySelector('.hdr');
    const hdrH = hdr ? hdr.offsetHeight : 50;
    const H = window.innerHeight - hdrH;

    const top = H / (PHI * PHI);      // 38.2%
    const middle = H / (PHI * PHI);   // 38.2%
    const bottom = H / (PHI * PHI * PHI); // 23.6%

    $('telemetryLayout').style.height = H + 'px';
    $('boxTop').style.height = top + 'px';
    $('boxMiddle').style.height = middle + 'px';
    $('boxBottom').style.height = bottom + 'px';
}

window.addEventListener('resize', applyGoldenHeights);
```

### Box Styling

**Active boxes (Top, Middle):**
```css
.golden-box {
    border: 1px solid hsla(var(--hue), 100%, 50%, 0.35);
    box-shadow: 0 0 20px hsla(var(--hue), 100%, 50%, 0.12);
}
```

**Subdued box (Bottom):**
```css
#boxBottom {
    background: var(--bg);
    border: 1px solid #1a2a30;
    box-shadow: none;
}
```

### Information Hierarchy

1. **Hero** (Speed): Massive, centered, fills top box, color-responsive
2. **Primary** (G-meter): Full visual attention, interactive canvas
3. **Secondary** (Mode): Background icon (260-400px @ 14% opacity) + text label
4. **Tertiary** (Metrics): Small grid, bottom row
5. **Controls** (Buttons): Minimal styling, functional

### Peak Speed Display Logic

The peak row has **fixed height** to prevent layout shift:

```css
.peak-row {
    height: 52px;  /* FIXED - never changes */
    transition: opacity 0.25s;
}
.peak-row.hidden {
    opacity: 0;
    pointer-events: none;
    /* Height stays - element is invisible but space is reserved */
}
```

**Update logic** (prevents flicker during acceleration):
```javascript
const PEAK_UPDATE_DELAY = 3000; // 3 seconds

// Track acceleration timing
if (isAccelerating) {
    lastAccelTime = now;
    wasAccelerating = true;
}

// Update session peak continuously
if (dspd > sessionPeak) sessionPeak = dspd;

// Only update DISPLAYED peak when:
// 1. Acceleration stopped for >= 3 seconds, OR
// 2. Currently braking
const accelStoppedLongEnough = wasAccelerating && !isAccelerating &&
                               (now - lastAccelTime >= PEAK_UPDATE_DELAY);
if (accelStoppedLongEnough || isBraking) {
    if (sessionPeak > displayedPeak) displayedPeak = sessionPeak;
    wasAccelerating = false;
}
```

---

## G-Meter Visualization

### Perspective Grid

The G-meter uses 1-point perspective to create depth, mimicking a receding floor grid:

**Grid dimensions:** 28 columns × 18 rows

**Perspective transformation:**
```javascript
function gridToScreen(col, row, w, h) {
    // Normalize to -1 to 1
    const nx = (col / GRID_COLS) * 2 - 1;
    const ny = (row / GRID_ROWS) * 2 - 1;

    // Perspective: top row is 40% width, bottom is 100%
    const depthFactor = (ny + 1) * 0.5;  // 0 at top, 1 at bottom
    const perspectiveFactor = 0.4 + depthFactor * 0.6;
    const px = nx * perspectiveFactor;

    // Foreshortening: compress vertical spacing toward top
    const py = ny * 0.85 + (1 - depthFactor) * 0.1;

    // Map to screen with padding
    const padX = w * 0.06;
    const padY = h * 0.08;
    return {
        x: padX + (px + 1) * 0.5 * (w - 2 * padX),
        y: padY + (py + 1) * 0.5 * (h - 2 * padY)
    };
}
```

### Auto-Zoom Camera System

The G-meter dynamically adjusts its visible range based on recent G-force magnitudes:

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `PEAK_WINDOW_MS` | 800ms | Rolling window for peak detection |
| `MIN_MAX_G` | 0.2g | Most zoomed in (gentle driving) |
| `MAX_MAX_G` | 2.0g | Most zoomed out (aggressive driving) |
| `ZOOM_OUT_RATE` | 0.12 | Fast (~150ms) - prevent dot clipping |
| `ZOOM_IN_RATE` | 0.025 | Slow (~500ms) - prevent "breathing" |
| `DEADBAND` | 0.04 | 4% hysteresis to prevent micro-adjustments |
| `SAFE_RADIUS_RATIO` | 2/3 | Peak magnitude lands at 2/3 of visible range |

**Implementation:**
```javascript
function updateAutoZoom(gx, gy) {
    const now = Date.now();
    const magnitude = Math.sqrt(gx * gx + gy * gy);

    magnitudeHistory.push({ time: now, mag: magnitude });
    magnitudeHistory = magnitudeHistory.filter(h => now - h.time < PEAK_WINDOW_MS);

    const m_peak = Math.max(...magnitudeHistory.map(h => h.mag), 0.05);
    const targetMaxG = Math.max(m_peak / SAFE_RADIUS_RATIO, MIN_MAX_G);

    const ratio = targetMaxG / currentMaxG;
    if (ratio > 1 + DEADBAND) {
        currentMaxG += (targetMaxG - currentMaxG) * ZOOM_OUT_RATE;
    } else if (ratio < 1 - DEADBAND) {
        currentMaxG += (targetMaxG - currentMaxG) * ZOOM_IN_RATE;
    }

    currentMaxG = Math.max(MIN_MAX_G, Math.min(MAX_MAX_G, currentMaxG));
}
```

### Trail Rendering

The G-meter shows a fading trail of recent positions:

- **Decay rate:** `intensity *= 0.96` per frame (~30 Hz)
- **Remove threshold:** `intensity < 0.03`
- **Max trail length:** 80 cells
- **Color:** Matches current hue (cyan when accelerating, amber when braking)
- **Alpha:** `intensity * 0.6`

```javascript
// Add new cell when position changes
if (!lastTrail || lastTrail.col !== gridPos.col || lastTrail.row !== gridPos.row) {
    gridTrail.push({
        col: gridPos.col,
        row: gridPos.row,
        intensity: 1.0,
        hue: isAccelerating ? HUE_CYAN : HUE_AMBER
    });
}

// Decay and cleanup
for (let i = gridTrail.length - 1; i >= 0; i--) {
    gridTrail[i].intensity *= 0.96;
    if (gridTrail[i].intensity < 0.03) gridTrail.splice(i, 1);
}
if (gridTrail.length > 80) gridTrail.shift();
```

---

## Animation & Smoothing

### Time-Based EMA

All displayed values use exponential moving average with a **time constant**, ensuring consistent smoothing regardless of frame rate:

```javascript
const EMA_TAU = 0.10;  // 100ms time constant

function process(buf) {
    const now = Date.now();
    const dt = lastProcessTime ? Math.min((now - lastProcessTime) / 1000, 0.2) : 0.033;
    lastProcessTime = now;

    // Time-based alpha: α = 1 - e^(-dt/τ)
    const alpha = 1 - Math.exp(-dt / EMA_TAU);

    emaGx = alpha * rawLatG + (1 - alpha) * emaGx;
    emaGy = alpha * rawLonG + (1 - alpha) * emaGy;
}
```

**Why time-based?** Frame-rate-independent smoothing. At 30 Hz or 60 Hz, the visual result is identical.

### CSS Transition Timings

| Element | Duration | Property |
|---------|----------|----------|
| Color shifts | 0.3s | `color`, `border-color` |
| Glow intensity | 0.15s | `box-shadow`, `text-shadow` |
| Peak row visibility | 0.25s | `opacity` |
| GPS status | 0.3s | `color`, `background` |

---

## Mode Icons

Geometric Unicode symbols representing driving states:

| Mode | Icon | Unicode | Code Point | Visual Meaning |
|------|------|---------|------------|----------------|
| Idle | ◇ | WHITE DIAMOND | U+25C7 | Empty, neutral |
| Accelerating | △ | WHITE UP-POINTING TRIANGLE | U+25B3 | Forward thrust |
| Braking | ▽ | WHITE DOWN-POINTING TRIANGLE | U+25BD | Deceleration |
| Cornering | ◈ | WHITE DIAMOND WITH CENTER DOT | U+25C8 | Lateral focus |
| Corner Exit | ⬡ | WHITE HEXAGON | U+2B21 | Complex maneuver (outline) |
| Trail Braking | ⬢ | BLACK HEXAGON | U+2B22 | Complex maneuver (filled) |

**Background usage:**
```css
.mode-bg {
    position: absolute;
    font-size: clamp(260px, 70vw, 400px);
    opacity: 0.14;
    color: hsl(var(--hue), 100%, 50%);
}
```

---

## Diagnostics Page

The `/diagnostics` page must match the main dashboard aesthetic exactly:

### Required Styling

```css
:root {
    --cyan: #00f0ff;
    --amber: #ff6a00;
    --bg: #030508;
}

body {
    font-family: 'Courier New', monospace;
    background: var(--bg);
    color: #e0e0e0;
}

h1 {
    color: var(--cyan);
    letter-spacing: 4px;
    text-transform: uppercase;
    font-weight: 400;
}

.section {
    background: rgba(0, 20, 30, 0.6);
    border: 1px solid hsla(190, 100%, 50%, 0.25);
}

.value { color: var(--cyan); font-weight: 600; }
.ok { color: #22c55e; }
.warn { color: var(--amber); }
.err { color: #ef4444; }

.back {
    color: var(--cyan);
    text-decoration: none;
    letter-spacing: 2px;
    opacity: 0.7;
}
```

### Navigation Link

The diagnostics link in the main dashboard header must be **subtle but accessible**:

```html
<a href="/diagnostics" style="color:#334;text-decoration:none;font-size:9px;letter-spacing:1px;opacity:0.5">DIAG</a>
```

---

## Common Pitfalls

### 1. Layout Shift on Value Updates

**Problem:** Numbers changing width causes elements to jump.
**Solution:** Always use `font-variant-numeric: tabular-nums` and fixed-width containers where appropriate.

### 2. Flicker Between 0.00 and -0.00

**Problem:** Floating point noise near zero shows as -0.00.
**Solution:** Use `fmtG()` with dead zone: `Math.abs(v) < 0.005 ? '0.00' : v.toFixed(2)`

### 3. Peak Speed Flickering During Acceleration

**Problem:** Peak updates immediately, causing rapid changes while accelerating.
**Solution:** Use `PEAK_UPDATE_DELAY` (3 seconds) and only update on deceleration or braking.

### 4. Jerky G-Meter at Variable Frame Rates

**Problem:** EMA with fixed alpha gives different smoothing at different frame rates.
**Solution:** Use time-based EMA with `alpha = 1 - exp(-dt/TAU)`.

### 5. Glow Overuse

**Problem:** Too many glowing elements creates visual noise.
**Solution:** Glow only on: status dots, active box borders, peak states. Never on static labels.

### 6. Color Transition Lag

**Problem:** Hue shift feels sluggish.
**Solution:** Hue changes immediately in JS; CSS transitions (0.3s) are only for smooth property interpolation.

---

## Implementation Checklist

When adding new UI elements, verify:

- [ ] Uses `--hue` CSS variable for theme-responsive colors
- [ ] Uses `font-variant-numeric: tabular-nums` for all numbers
- [ ] Uses appropriate letter-spacing (1-4px for labels)
- [ ] Uses UPPERCASE for labels and headers
- [ ] Has CSS transitions (0.15-0.3s) only where needed
- [ ] Follows information hierarchy (hero → primary → secondary → tertiary)
- [ ] Works with both cyan (hue: 190) and amber (hue: 25)
- [ ] No layout shift when values update
- [ ] Uses EMA smoothing for live values (time-based, TAU = 100ms)
- [ ] Matches diagnostics page styling if on that page
- [ ] Mobile-tested with `clamp()` for responsive sizing
- [ ] No decorative elements without data purpose

---

## Testing the UI

### Local Development

```bash
cd tools/dashboard-dev
python3 -m http.server 8080
# Open http://localhost:8080
```

The development version runs a 30-second canyon drive simulation loop automatically.

### What to Verify

1. **Golden ratio layout** - Three boxes fill screen with correct proportions
2. **Color shifts** - UI turns cyan during acceleration, amber during braking
3. **G-meter trail** - Smooth trail with perspective, auto-zoom working
4. **Peak tracking** - Peak appears after deceleration, not during acceleration
5. **Mode icons** - Background symbol changes with driving mode
6. **Responsive scaling** - Test on phone-sized viewport
7. **No flicker** - Values don't oscillate or jump
