# Dashboard Development Tool

A local simulator for testing and developing the Blackbox mobile dashboard without hardware. This tool lets you visualize the dashboard UI, test mode detection logic, and replay recorded telemetry sessions - all in your browser.

## Why This Exists

The production dashboard is embedded directly in the ESP32 firmware (`websocket_server.rs`). Making UI changes requires:
1. Edit the HTML/CSS/JS in a Rust string literal
2. Rebuild the firmware (~30 seconds)
3. Flash to the ESP32 (~10 seconds)
4. Connect to the device's WiFi
5. Open the dashboard in a browser

This development tool eliminates that cycle. You edit `index.html` directly, refresh your browser, and see changes instantly. The visual design here matches production exactly - when you're happy with changes, port them to `websocket_server.rs`.

## Quick Start

```bash
cd tools/dashboard-dev
python3 -m http.server 8080
```

Open http://localhost:8080 - you'll see a simulated aggressive canyon drive running automatically.

**From your phone:** Use `python3 -m http.server 8080 --bind 0.0.0.0` and navigate to your computer's IP (e.g., `http://192.168.1.42:8080`). Requires same WiFi network.

## Features

### Auto-Drive Mode (Default)

On load, the dashboard plays a 30-second simulated canyon drive on loop:
- Hairpin turns with trail braking
- Hard acceleration and braking zones
- Realistic mode transitions (ACCEL, BRAKE, CORNER, combined states)
- Small noise added for realism

This lets you see how the UI responds to different driving scenarios without needing hardware or recorded data.

**Technical details:**
- Uses Catmull-Rom spline interpolation for perfectly smooth, continuous motion
- No jumps or discontinuities - even at the loop point
- Peak values: ~0.40g braking, ~0.55g lateral, ~0.30g acceleration

### CSV Replay Mode

Load and replay telemetry sessions exported from the production dashboard:

**How to use:**
1. Click **LOAD** button
2. Select a CSV file exported from the production dashboard
3. Watch the session replay in real-time
4. Click anywhere on the progress bar to seek
5. Click **SIM** to return to canyon simulation

**Playback features:**
- Real-time playback at original recorded speed
- Auto-loops when finished
- Click-to-seek on progress bar
- Shows elapsed time and total duration
- Uses mode data directly from CSV (not recalculated)
- Resets max G values when loading a new file

**Supported CSV format:**
```
time,speed,ax,ay,wz,mode,lat_g,lon_g,gps_lat,gps_lon,gps_valid
```

This matches the export format from the production dashboard's EXPORT button.

### Recording

The dashboard can record simulated or replayed sessions:

- **REC** - Start recording (button turns red, shows STOP)
- **STOP** - Stop recording and save to browser localStorage
- **EXPORT** - Download the most recent recording as CSV

Recordings are stored in browser localStorage (persists across page reloads, up to 10 sessions).

### Mode Detection

The simulator includes a JavaScript port of the firmware's `mode.rs`:
- Same EMA filtering (alpha=0.35)
- Same threshold logic with hysteresis
- Same combined modes (ACCEL+CORNER for corner exit, BRAKE+CORNER for trail braking)

This ensures mode transitions you see here match what the firmware produces.

## UI Design

The dashboard uses Apple-style neumorphism:
- Light background (#e8ecef)
- Dual-shadow depth system (raised, inset, flat variants)
- Circular G-meter with trail and shadow dot
- System font stack (SF Pro, Segoe UI, system-ui)

All styling matches the production dashboard in `websocket_server.rs`.

## File Structure

```
tools/dashboard-dev/
├── index.html           # The entire dashboard (HTML + CSS + JS in one file)
├── README.md            # This file
└── REDESIGN-WORKFLOW.md # Design iteration workflow documentation
```

Everything lives in one file (`index.html`) for simplicity. The production dashboard is also single-file (embedded in Rust firmware as a string literal).

## Porting Changes to Production

When you're happy with changes in `index.html`:

1. Open `sensors/blackbox/src/websocket_server.rs`
2. Find `const DASHBOARD_HTML: &str = r#"...`
3. Replace the HTML content with your updated `index.html`
4. Build and test: `cargo build --release`

**Note:** Production has additional features not in the simulator:
- HTTP polling to `/api/telemetry` (instead of local simulation)
- GPS status indicator with firmware rate
- Inline fusion diagnostics for CSV export
- Link to diagnostics page

## Related Files

- `sensors/blackbox/src/websocket_server.rs` - Production dashboard (DASHBOARD_HTML constant)
- `REDESIGN-WORKFLOW.md` - Documents the design iteration process and branch conventions
