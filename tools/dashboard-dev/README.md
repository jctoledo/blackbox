# Dashboard Development Tool

Local simulator for testing the Blackbox dashboard without hardware.

## Quick Start

```bash
cd tools/dashboard-dev
python3 -m http.server 8080
```

Open http://localhost:8080 - you'll see a simulated aggressive canyon drive running automatically.

## Features

### Auto-Drive Mode (Default)
- 30-second loop of aggressive canyon road driving
- Hairpin turns, trail braking, hard acceleration
- Realistic mode transitions (ACCEL, BRAKE, CORNER, combined states)
- Small noise added for realism

### Manual Control
- **Long-press "DEV"** in the header to show/hide simulator controls
- Sliders for speed, longitudinal G, lateral G, yaw rate
- Preset buttons: Idle, Cruise, Hard Accel, Hard Brake, Left/Right Turn
- Checkbox to enable/disable auto-drive

### Keyboard Controls
- `W` - Accelerate (0.4g)
- `S` - Brake (-0.6g)
- `A` - Left turn (0.35g lateral + 25 deg/s yaw)
- `D` - Right turn
- `Space` - Reset g-forces to zero

Keyboard input disables auto-drive.

## Mode Detection

The simulator includes a JavaScript port of `mode.rs`:
- Same EMA filtering (alpha=0.35)
- Same threshold logic with hysteresis
- Same combined modes (ACCEL+CORNER, BRAKE+CORNER)

Change presets (Track/Canyon/City/Highway) to see how different sensitivity settings affect classification.

## Canyon Drive Sequence

The 30-second loop uses Catmull-Rom spline interpolation for perfectly smooth, continuous motion with no jumps or discontinuities - even at the loop point.

The drive simulates:
1. Build speed on short straight
2. Brake into left sweeper, trail brake through apex
3. Accelerate out, short straight
4. Hard brake into right hairpin
5. Deep trail braking through tight right
6. Quick left-right-left chicane
7. Sweeping right turn
8. Seamlessly loops back to start

Peaks: ~0.40g braking, ~0.55g lateral, ~0.30g acceleration
