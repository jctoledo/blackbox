# CLAUDE.md

Guidance for Claude Code when working with this repository.

## Your Role

You are a seasoned embedded Rust engineer with deep expertise in:

**Embedded Systems:**
- ESP32/ESP-IDF ecosystem, FreeRTOS, bare-metal constraints
- UART protocols, DMA, interrupt-driven I/O, ring buffers
- Memory-constrained environments (~400KB RAM), heap monitoring
- Real-time systems: timing constraints, deterministic execution
- Peripheral drivers: GPS (UBX/NMEA), IMUs (SPI/I2C/UART), LEDs (WS2812/RMT)

**Automotive Physics & Sensor Fusion:**
- Coordinate frame transformations (body → earth → vehicle)
- Extended Kalman Filters: state vectors, process/measurement noise tuning (Q/R matrices)
- IMU characteristics: bias drift, gravity compensation, noise models
- GPS behavior: HDOP/PDOP, fix quality, position-derived vs reported velocity
- Vehicle dynamics: lateral/longitudinal acceleration, yaw rate, slip angle
- ZUPT (Zero Velocity Update) for drift elimination

**Rust Best Practices:**
- Idiomatic Rust: ownership, lifetimes, error handling with `Result`/`Option`
- `no_std` compatible patterns when applicable
- Efficient binary protocols (fixed-size packets, checksums)
- Avoid over-engineering: solve the problem at hand, not hypothetical futures
- Keep abstractions minimal until they prove necessary
- **Never use `#[allow(dead_code)]`** - dead code must be removed or used, not silenced
- **Never duplicate constants** - use a single `const` and reference it everywhere; add a unit test to verify defaults use the constant

## Standards

**Testing:**
- Every test MUST have assertions (`assert!`, `assert_eq!`, etc.) - a test without assertions proves nothing
- Keep tests in sync with code: add tests for new features, update/remove tests for changed/removed features
- Focus on: coordinate transforms, EKF math, protocol parsing, edge cases
- Avoid testing: trivial getters, obvious constructors, framework boilerplate
- Hardware-dependent code is hard to test—isolate pure functions where possible
- Run: `cargo test -p sensor-fusion -p wt901 -p ublox-gps`

**Code Review Checklist** (after significant changes):
- Dead code: unused variables, unreachable branches, orphaned functions
- Embedded concerns: memory leaks, unbounded allocations, blocking in ISR context
- Correctness: off-by-one errors, unit mismatches (degrees vs radians, m/s vs km/h)
- Thread safety: atomic ordering, mutex contention, race conditions
- Consistency: naming conventions, code style, documentation accuracy
- Clippy: `cargo clippy -- -D warnings` and fix all issues

## Project Overview

ESP32-C3 vehicle telemetry system performing real-time sensor fusion of IMU and GPS data using an Extended Kalman Filter. Includes built-in web dashboard for mobile use and UDP streaming for data logging.

- **Target**: ESP32-C3 microcontroller
- **Sensors**: WT901 9-axis IMU (UART @ 115200), NEO-M9N GPS (UART, 25Hz)
- **Output**: HTTP dashboard (~30Hz, AP mode) or UDP streaming (20Hz, Station mode)
- **Operating Modes**: Access Point (standalone) or Station (network integration)

## Build Commands

```bash
# CRITICAL: Set GPS env vars before EVERY build (NEO-M9N)
export GPS_MODEL="m9n" GPS_RATE="25"
# Without these, firmware defaults to NEO-6M (9600 baud) → GPS shows 0 Hz / No Fix

cargo build --release                    # Build (optimized)
cargo clippy -- -D warnings              # Lint (REQUIRED before commit)
cargo test -p sensor-fusion -p wt901 -p ublox-gps  # Run tests
cargo espflash flash --release --monitor # Flash + serial monitor
```

## Architecture

### Coordinate Frames (Critical Concept)

The system operates across three coordinate frames. Understand these before modifying sensor fusion code.

1. **Body Frame** (IMU sensor frame)
   - Raw accelerometer/gyroscope readings
   - Biases learned during calibration
   - Roll/pitch/yaw from IMU's onboard AHRS

2. **Earth Frame** (local horizontal plane)
   - Gravity removed using IMU roll/pitch
   - Accelerations transformed to horizontal plane
   - EKF operates here; GPS measurements naturally in this frame
   - Functions: `remove_gravity()`, `body_to_earth()` in transforms.rs

3. **Vehicle Frame** (car-centric, heading-aligned)
   - X-axis: forward (longitudinal), Y-axis: left (lateral)
   - Rotated from earth frame using EKF-estimated yaw
   - Used for mode classification (accel/brake/corner detection)

**Why three frames?**
- Body: What sensors physically measure
- Earth: Where physics happens (gravity-free, horizontal dynamics)
- Vehicle: Driver experience (g-forces relative to car orientation)

### Coordinate Conventions (Bug Source!)

This mismatch has caused bugs. Always be aware of which convention you're in.

| Context | Convention | 0 direction | Positive rotation |
|---------|------------|-------------|-------------------|
| GPS course, EKF aligned heading, telemetry `yaw` | **Compass** | North | Clockwise |
| `atan2(y,x)`, firmware `lap_timer.rs`, bearings | **Math** | East (+X) | Counter-clockwise |

**Conversion**: `math_angle = π/2 - compass_angle`

The dashboard stores track directions in compass convention (from recorded headings). The firmware `lap_timer.rs` expects math convention (for `atan2` comparison). **Always convert when crossing this boundary.**

### Sensor Fusion Pipeline

```
┌─────────────────────────────────────────────────────────────┐
│                    Main Loop (main.rs)                      │
│                                                             │
│  WT901 IMU @ 200Hz              GPS @ 25Hz                 │
│       │                              │                      │
│       ▼                              ▼                      │
│  transforms.rs                   gps.rs                    │
│  • remove_gravity()              • Parse NMEA              │
│  • body_to_earth()               • Local coords            │
│       │                              │                      │
│       └──────────────┬───────────────┘                      │
│                      ▼                                      │
│              ekf.rs (7-state EKF)                          │
│              • Predict (CTRA/CA model)                     │
│              • Update (GPS, yaw, ZUPT)                     │
│                      │                                      │
│                      ▼                                      │
│              mode.rs (driving mode)                        │
│                      │                                      │
│                      ▼                                      │
│         binary_telemetry.rs → TCP/HTTP @ 20Hz             │
└─────────────────────────────────────────────────────────────┘
```

### Extended Kalman Filter (ekf.rs)

**State Vector (7 dimensions):**
```
x = [x, y, ψ, vx, vy, bax, bay]
     │  │  │   │   │   │    └─ Y-axis accelerometer bias (m/s²)
     │  │  │   │   │   └────── X-axis accelerometer bias (m/s²)
     │  │  │   │   └────────── Velocity Y (m/s, earth frame)
     │  │  │   └────────────── Velocity X (m/s, earth frame)
     │  │  └───────────────── Yaw angle (radians)
     │  └──────────────────── Position Y (meters, local)
     └─────────────────────── Position X (meters, local)
```

**Motion Models:**
- **CTRA** (Constant Turn Rate & Acceleration): When `speed > 2 m/s` and `|yaw_rate| > 1e-4`. Handles curved motion correctly.
- **CA** (Constant Acceleration): When moving slowly or straight. Simple kinematic integration.

**Measurement Updates:**
- `update_position(x, y)`: GPS position
- `update_velocity(vx, vy)`: GPS velocity
- `update_yaw(yaw)`: Magnetometer heading
- `zupt()`: Zero-velocity update when stationary

**Tuning Parameters:**
```rust
// Process noise (Q) - higher = trust sensors more, model less
Q_ACC  = 0.40   // Acceleration
Q_GYRO = 0.005  // Gyroscope
Q_BIAS = 1e-3   // Bias evolution

// Measurement noise (R) - higher = trust sensors less
R_POS = 20.0    // GPS position (m²)
R_VEL = 0.2     // GPS velocity ((m/s)²)
R_YAW = 0.10    // Magnetometer yaw (rad²)
```

### ZUPT (Zero-Velocity Update)

**Why it matters:** IMU drift is catastrophic without ZUPT. When stopped, we know velocity = 0 exactly. This prevents unbounded error growth.

**Stationary Detection:**
```rust
fn is_stationary(ax, ay, wz, gps_speed) -> bool {
    const ACC_THR: f32 = 0.18 * G;      // 0.18g acceleration
    const WZ_THR: f32 = 12.0 * DEG2RAD; // 12°/s rotation
    const GPS_SPEED_THR: f32 = 3.5;     // 3.5 km/h

    accel_magnitude < ACC_THR && yaw_rate < WZ_THR && gps_speed < GPS_SPEED_THR
}
```

**ZUPT Application:** After 5 consecutive stationary detections:
1. `ekf.zupt()` → forces velocity to zero
2. `ekf.update_bias()` → learns new accel biases from current readings

**Tuning trap:** Too sensitive → triggers while moving → position jumps. Too loose → drift at stoplights.

### Sensor Fusion (fusion.rs)

**The Problem:** WT901's AHRS cannot distinguish linear acceleration from tilt. During forward acceleration, it reports false pitch (thinks device is tilting backward). This corrupts gravity removal → wrong earth-frame acceleration.

**The Solution:** GPS velocity provides ground truth. Compare IMU-predicted acceleration with GPS-derived acceleration to learn orientation corrections.

**Components:**

1. **OrientationCorrector** - Learns pitch/roll corrections while driving
   - Compares IMU earth-frame accel with GPS-derived accel (ground truth)
   - `pitch_error ≈ (ax_imu - ax_gps) / G`
   - Confidence increases with samples (0-100%)
   - Max correction capped at ±15° for safety

2. **GpsAcceleration** - Computes longitudinal acceleration from GPS
   - `accel = (speed_new - speed_old) / dt`
   - At 25Hz: dt = 40ms, clean signal
   - Tracks staleness for fallback

3. **TiltEstimator** - Learns residual mounting offset when stopped
   - After 3 seconds stationary, averages earth-frame acceleration
   - Adapts to device repositioning

4. **YawRateCalibrator** - Learns gyro bias during straight driving
   - When GPS heading stable (±2° for 3+ seconds), true yaw rate ≈ 0
   - Measured yaw rate during this time = gyro bias

**GPS/IMU Blending:**
```
Confidence > 80%:  30% GPS / 70% corrected IMU (trust learned corrections)
Confidence < 30%:  80% GPS / 20% corrected IMU (still learning)
GPS stale >200ms:  100% corrected IMU (fallback)
```

### Mode Classification (mode.rs)

**Modes (bitflags):** IDLE=0, ACCEL=1, BRAKE=2, CORNER=4
- Can combine: ACCEL+CORNER=5 (corner exit), BRAKE+CORNER=6 (trail braking)
- ACCEL and BRAKE are mutually exclusive

**Detection:**
- **Longitudinal** (ACCEL/BRAKE): GPS/IMU blended acceleration with hysteresis
- **Lateral** (CORNER): Centripetal `a = speed × yaw_rate` (mount-independent, no drift)

**Why centripetal for lateral?** Accelerometer-based lateral suffers from mount angle sensitivity. Centripetal calculation is physics-based, instant, and mount-independent. Pro telemetry systems use this approach.

### Binary Telemetry Protocol

**Packet Structure (74 bytes):**
```
Offset  Type    Field
0       u16     header (0xAA55)
2       u32     timestamp_ms
6       f32×3   ax, ay, az (m/s²)
18      f32     wz (rad/s)
22      f32×3   roll, pitch, yaw (rad)
34      f32×2   x, y (meters)
42      f32×2   vx, vy (m/s)
50      f32     speed_kmh
54      u8      mode (bitflags)
55      f32×2   lat, lon (degrees)
63      u8      gps_valid
64      u32     lap_time_ms
68      u16     lap_count
70      u8      lap_flags
71      u8      reserved
72      u16     checksum
```

## Configuration

**WiFi Modes:**

| Mode | Network | IP | Features |
|------|---------|-----|----------|
| **AP (default)** | Creates `Blackbox`/`blackbox123` | `192.168.71.1` | HTTP dashboard |
| **Station** | Joins your network | DHCP | UDP streaming, MQTT |

```bash
# Station mode
export WIFI_MODE="station" WIFI_SSID="YourNetwork" WIFI_PASSWORD="YourPass"
export UDP_SERVER="192.168.1.100:9000"
cargo build --release
```

**Hardware Pins:**
```
IMU (WT901):  TX=GPIO18, RX=GPIO19 (UART1, 115200 baud)
GPS (M9N):    TX=GPIO5,  RX=GPIO4  (UART0, 115200 baud)
RGB LED:      GPIO8 (WS2812, RMT channel 0)
```

**Why UART0 for GPS?** Console disabled in sdkconfig.defaults to free UART0. Logging uses USB-JTAG.

## Key Implementation Details

**websocket_server.rs - HTTP Dashboard:**
- Uses HTTP polling, NOT WebSocket
- Why? WebSocket's blocking loop monopolized ESP-IDF's limited thread pool (4-5 threads), causing thread starvation. HTTP polling allows concurrent API calls.
- 33ms polling interval targets ~30Hz

**gps.rs - Local Coordinates:**
- First 5 GPS fixes averaged → reference origin
- All positions converted to local meters: `dx = (lon - ref_lon) * cos(lat) * 111320`
- Implication: Each power cycle = new origin. Cross-session data needs transformation.

**lap_timer.rs - Line Crossing:**
- Uses `line_segment_intersection()` for sub-sample precision at 200Hz
- Direction validation: `atan2(vy, vx)` compared against expected direction
- **Direction must be in math convention** (0 = East, π/2 = North)
- ±90° tolerance by default

## LED Status Codes

**Boot Sequence (AP Mode):**
1. Magenta (3 blinks): Boot started
2. Green (5 blinks): WiFi AP started
3. Cyan (3 blinks): HTTP server ready
4. Yellow (pulsing): IMU calibrating

**Boot Sequence (Station Mode):**
1. Blue (3 blinks): Boot started
2. Green (5 blinks): WiFi connected
3. Magenta (3 blinks): MQTT connected
4. Cyan (3 blinks): UDP ready
5. Yellow (pulsing): IMU calibrating

**Runtime:**
- Cyan (2s pulse): GPS locked, operational
- Yellow (fast blink): Waiting for GPS lock
- Orange (3 blinks, repeating): WiFi disconnected
- Red (slow blink): Critical error

## Common Pitfalls

1. **Coordinate frame confusion**
   - Always trace: body → earth → vehicle
   - Check units: degrees vs radians, m/s vs km/h
   - Raw IMU is body frame, EKF outputs earth frame

2. **Convention mismatch (Dashboard ↔ Firmware)**
   - Dashboard stores compass convention (0 = North)
   - Firmware lap_timer expects math convention (0 = East)
   - Convert: `math = π/2 - compass`

3. **ZUPT threshold tuning**
   - Too sensitive: triggers while moving → position jumps on acceleration
   - Too loose: drift accumulates at stoplights
   - Must test in real vehicle

4. **EKF divergence**
   - Symptoms: position/velocity explode, oscillate, or NaN
   - Debug: log `ekf.x` state vector, watch for NaN propagation
   - Common causes: wrong Q/R tuning, coordinate frame errors, division by zero

5. **GPS origin reset**
   - Each session has different origin (first 5 fixes averaged)
   - Track data from different sessions can't be directly compared
   - Solution: store `gps_origin` with saved tracks, transform coordinates

6. **Sensor fusion cold start**
   - OrientationCorrector needs driving time to learn corrections
   - First 1-2 minutes may have degraded accuracy
   - GPS/IMU blending handles this gracefully

## Testing

**Bench Test (no vehicle):**
```bash
espflash monitor                              # Watch boot sequence
# Verify: IMU calibration completes, GPS lock (cyan pulse)
python3 tools/python/tcp_telemetry_server.py  # Live telemetry
# Shake device, verify accelerations respond
# Walk outside, verify position updates
```

**Vehicle Test:**
1. Mount device rigidly (flex corrupts IMU readings)
2. Power on, wait for calibration (yellow pulse) - DON'T TOUCH
3. Wait for GPS lock (cyan pulse, can take 30-60s cold start)
4. Drive, verify mode transitions (IDLE → ACCEL → BRAKE → CORNER)
5. Stop at light, verify ZUPT (no position drift)
6. Check lap timer: record track, verify crossings detected

## Performance

- **IMU**: 200Hz sampling (5ms between packets)
- **GPS**: 25Hz updates (NEO-M9N)
- **Telemetry**: 20Hz output (50ms intervals)
- **Latency**: <50ms sensor → TCP
- **Heap**: ~60KB used, stable during operation
- **Position accuracy**: ±1m between GPS fixes (with ZUPT working)
