# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

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
- Avoid over-engineering: solve the problem at hand, not hypothetical future problems
- Keep abstractions minimal until they prove necessary
- **Never use `#[allow(dead_code)]`** - dead code must be removed or used, not silenced
- **Never duplicate constants** - use a single `const` and reference it everywhere. Add a unit test to verify defaults use the constant.

**Testing Philosophy:**
- Write meaningful tests that catch real bugs, not boilerplate
- **Every test MUST have assertions** (`assert!`, `assert_eq!`, etc.) - a test without assertions proves nothing
- **Keep tests in sync with code**: When adding new features, add corresponding tests. When removing or changing features, update or remove stale tests.
- Focus on: coordinate transforms, EKF math, protocol parsing, edge cases
- Avoid: trivial getters, obvious constructors, framework code
- Hardware-dependent code is hard to test—isolate pure functions where possible
- Example good tests: `remove_gravity()` with known angles, EKF prediction with synthetic data, NMEA parser with malformed input
- Run tests with: `cargo test -p sensor-fusion -p wt901 -p ublox-gps`

**Code Review:**
After implementing a new feature or making significant changes, perform a code review before considering the work complete. Focus on:
- **Dead code**: Unused variables, unreachable branches, orphaned functions
- **Embedded concerns**: Memory leaks, unbounded allocations, blocking in ISR context
- **Correctness**: Off-by-one errors, unit mismatches (degrees vs radians, m/s vs km/h)
- **Thread safety**: Atomic ordering, mutex contention, race conditions
- **Consistency**: Naming conventions, code style, documentation accuracy
- **Clippy compliance**: Run `cargo clippy -- -D warnings` and fix all issues

## Project Overview

Blackbox is an ESP32-C3 vehicle telemetry system that performs real-time sensor fusion of IMU and GPS data using an Extended Kalman Filter (EKF). It includes a built-in web dashboard for mobile use and supports UDP streaming for data logging during track days, autocross, rally, and vehicle dynamics research.

**Target Hardware:** ESP32-C3 microcontroller
**Sensors:** WT901 9-axis IMU (UART @ 115200), NEO-6M or NEO-M9N GPS (UART)
**Output:** HTTP dashboard (~30 Hz, AP mode) or UDP streaming (20 Hz, Station mode) + MQTT status
**Operating Modes:** Access Point (standalone/mobile) or Station (network integration)
**Cost:** ~$50-100 in parts vs. $1000+ commercial alternatives

**GPS Options:**
- **NEO-6M**: Budget GPS, 5 Hz max, NMEA only (~$15)
- **NEO-M9N**: High-performance GPS, up to 25 Hz, UBX protocol, automotive mode (~$75)

## Build and Development Commands

### Firmware Development
```bash
# Check compilation without building
cargo check

# Build firmware
cargo build
cargo build --release              # Optimized for ESP32 (recommended)

# Format code
cargo fmt

# Run linter (REQUIRED before committing - no warnings allowed)
cargo clippy -- -D warnings

# Run tests (limited - most require hardware)
cargo test --lib --bins

# Flash to ESP32-C3 and monitor serial output
cargo espflash flash --monitor
cargo espflash flash --release --monitor

# Just monitor serial output (after flashing)
espflash monitor

# Build with NEO-M9N GPS at 25 Hz
export GPS_MODEL="m9n"
export GPS_RATE="25"
cargo build --release
```

### Python Tools
```bash
# Install dependencies
pip install -r tools/python/requirements.txt

# Run TCP telemetry receiver (recommended - 20+ Hz)
python3 tools/python/tcp_telemetry_server.py

# Run MQTT receivers
python3 tools/python/mqtt_binary_decoder.py    # Binary protocol
python3 tools/python/mqtt_decoder.py           # JSON (legacy)

# Configure WT901 IMU for 200Hz (run ONCE, saves to EEPROM)
# Disconnect IMU from ESP32, connect to USB-serial adapter
python3 tools/python/configure_wt901.py /dev/ttyUSB0

# Probe WT901 IMU baud rate (verify current config)
python3 tools/python/probe_wt901.py /dev/ttyUSB0

# Check Python syntax
python3 -m py_compile tools/python/*.py
```

### Project Structure
```
blackbox/
├── src/                # Rust firmware
├── tools/python/       # Telemetry receivers
├── docs/              # Additional documentation
├── hardware/          # PCB designs, enclosures (contributions welcome)
├── examples/          # Sample data, integrations
├── .github/           # CI/CD, issue templates
└── *.md               # Documentation (README, FAQ, CLAUDE, etc.)
```

## Architecture Overview

### Coordinate Frames and Transformations

**Critical concept:** The system operates across three coordinate frames. Understanding these is essential before modifying sensor fusion code.

1. **Body Frame** (IMU sensor frame)
   - Raw accelerometer/gyroscope readings from WT901
   - Biases learned during calibration
   - Roll/pitch/yaw angles provided by IMU's onboard processor

2. **Earth Frame** (local horizontal plane, similar to NED)
   - Gravity removed using IMU roll/pitch from body frame
   - Accelerations transformed to horizontal plane
   - Used for EKF prediction step
   - GPS measurements naturally in this frame
   - Functions: `remove_gravity()`, `body_to_earth()` in `src/transforms.rs`

3. **Vehicle Frame** (car-centric, heading-aligned)
   - X-axis: forward (longitudinal)
   - Y-axis: left (lateral)
   - Rotated from earth frame using EKF-estimated yaw
   - Used for mode classification (detect accel/brake/corner)
   - Transformation done in `mode.rs` during classification

**Why three frames?**
- Body: What sensors measure
- Earth: Physics happens here (gravity-free, horizontal dynamics)
- Vehicle: Driver experience (g-forces relative to car orientation)

### Sensor Fusion Pipeline

```
┌─────────────────────────────────────────────────────────────┐
│                    Main Loop (main.rs)                      │
│                                                             │
│  WT901 IMU @ 200Hz          GPS @ 5-25Hz                   │
│  (auto-detect baud)         (NEO-6M or NEO-M9N)            │
│       │                          │                          │
│       ├─► imu.rs            ┌────┴─────┐                   │
│       │   Parse UART        │  gps.rs  │                   │
│       │   Packets           │Parse NMEA│                   │
│       │                     └────┬─────┘                   │
│       ▼                          │                          │
│  transforms.rs ◄─────────────────┘                         │
│  • remove_gravity()          GPS position                  │
│  • body_to_earth()           velocity                      │
│       │                          │                          │
│       └────────┬─────────────────┘                         │
│                ▼                                            │
│           ekf.rs (7-state EKF)                             │
│           • Predict (CTRA/CA model)                        │
│           • Update (GPS, yaw, ZUPT)                        │
│           • Bias estimation                                │
│                │                                            │
│                ▼                                            │
│           mode.rs                                          │
│           Classify: IDLE/ACCEL/BRAKE/CORNER/ACCEL+CORNER/BRAKE+CORNER │
│                │                                            │
│                ▼                                            │
│      binary_telemetry.rs                                   │
│      Pack 67-byte packet                                   │
│                │                                            │
│                ▼                                            │
│      tcp_stream.rs ──► TCP @ 20Hz                         │
│      mqtt.rs ──────────► MQTT (status only)               │
└─────────────────────────────────────────────────────────────┘
```

**Main Loop Structure (`main.rs:187-384`):**
1. Read IMU byte-by-byte, parse WT901 packets
2. When accel packet (0x51): transform to earth frame, run EKF predict
3. When angle packet (0x53): update EKF yaw with magnetometer
4. Read GPS byte-by-byte, parse NMEA sentences
5. When GPS fix valid: check if stationary → ZUPT or GPS update
6. Every 50ms: build telemetry packet, send over TCP
7. Update LED status based on GPS lock

### Extended Kalman Filter (ekf.rs)

**State Vector (7D):**
```rust
x = [x, y, ψ, vx, vy, bax, bay]
     │  │  │  │   │   │    └─ Y-axis accel bias (m/s²)
     │  │  │  │   │   └────── X-axis accel bias (m/s²)
     │  │  │  │   └────────── Velocity Y (m/s, earth frame)
     │  │  │  └────────────── Velocity X (m/s, earth frame)
     │  │  └───────────────── Yaw angle (rad)
     │  └──────────────────── Position Y (m, local)
     └─────────────────────── Position X (m, local)
```

**Motion Models:**
- **CTRA** (Constant Turn Rate & Acceleration): Used when `speed > 2 m/s` and `|wz| > 1e-4`
  - Handles curved motion physics correctly
  - Predicts position using turn rate and velocity
- **CA** (Constant Acceleration): Used when moving slowly or going straight
  - Simple kinematic model: `x += vx*dt + 0.5*ax*dt²`

**Measurement Updates:**
- `update_position(x, y)`: GPS position (R_POS = 20.0 m²)
- `update_velocity(vx, vy)`: GPS velocity (R_VEL = 0.2 (m/s)²)
- `update_speed(speed)`: Scalar GPS speed with adaptive noise
- `update_yaw(yaw)`: Magnetometer yaw (R_YAW = 0.10 rad²)
- `update_bias(ax, ay)`: When stationary, update accel biases
- `zupt()`: Zero-velocity update when stopped

**Tuning Parameters (ekf.rs:8-16):**
```rust
const Q_ACC: f32 = 0.40;    // Process noise: acceleration
const Q_GYRO: f32 = 0.005;  // Process noise: gyro
const Q_BIAS: f32 = 1e-3;   // Process noise: bias evolution
const R_POS: f32 = 20.0;    // Measurement noise: GPS position
const R_VEL: f32 = 0.2;     // Measurement noise: GPS velocity
const R_YAW: f32 = 0.10;    // Measurement noise: magnetometer yaw
```
Higher Q → trust sensors more, model less. Higher R → trust sensors less.

### Stationary Detection and ZUPT

**Why it matters:** IMU drift is catastrophic without ZUPT. When the vehicle is stopped, we know velocity = 0 exactly. This prevents unbounded error growth.

**Detection Logic (`main.rs:422-432`):**
```rust
fn is_stationary(ax, ay, wz, gps_speed, position_speed) -> bool {
    const ACC_THR: f32 = 0.18 * 9.80665;        // 0.18g
    const WZ_THR: f32 = 12.0 * 0.017453293;     // 12°/s
    const GPS_SPEED_THR: f32 = 3.5;             // 3.5 km/h (reported)
    const POS_SPEED_THR: f32 = 5.0;             // 5.0 km/h (position-derived)

    low_inertial && low_speed
}
```

**ZUPT Application (`main.rs:239-252`):**
After 5 consecutive stationary detections:
1. Call `ekf.zupt()` → forces velocity to zero
2. Call `ekf.update_bias()` → learn new bias from current readings
3. Reset mode classifier speed

**Common Issue:** If thresholds are too sensitive, ZUPT triggers while moving → position jumps. If too loose, drift accumulates at red lights.

### GPS Warmup and Local Coordinates

**Problem:** GPS provides lat/lon (degrees), but we need local x/y (meters) for physics.

**Solution (`gps.rs:63-80`):**
1. First 5 valid GPS fixes are averaged → reference point
2. All subsequent positions converted to local meters relative to reference
3. Simple approximation: `dx ≈ (lon - lon_ref) * cos(lat) * 111320`

**Implication:** Restarting the ESP32 resets the origin. Data from different sessions can't be directly compared without transformation.

### Driving Mode Classification (mode.rs)

**Modes:** IDLE, ACCEL, BRAKE, CORNER, ACCEL+CORNER, BRAKE+CORNER

**NEW: Combined States** - Modes use bitflags and can be combined:
- ACCEL + CORNER = 5 (corner exit / trail throttle)
- BRAKE + CORNER = 6 (corner entry / trail braking)
- ACCEL and BRAKE are mutually exclusive
- CORNER is independent and can combine with either

**How it works:**
1. Transform earth-frame acceleration → vehicle frame using current yaw
2. Split into longitudinal (forward/back) and lateral (left/right)
3. EMA filter both lateral accel and yaw rate
4. Independent state detection with hysteresis for each component

**Driving Presets (configurable via dashboard):**

| Preset | Accel | Brake | Lateral | Yaw | Min Speed | Use Case |
|--------|-------|-------|---------|-----|-----------|----------|
| Track | 0.30g | 0.50g | 0.50g | 0.15 rad/s | 3.0 m/s | Racing, autocross |
| Canyon | 0.20g | 0.35g | 0.30g | 0.10 rad/s | 2.5 m/s | Spirited twisty roads |
| City (default) | 0.10g | 0.18g | 0.12g | 0.05 rad/s | 2.0 m/s | Daily driving |
| Highway | 0.08g | 0.15g | 0.10g | 0.04 rad/s | 4.0 m/s | Cruise, subtle inputs |
| Custom | User-defined | | | | | Fine-tuning via sliders |

Exit thresholds are 50% of entry values for hysteresis (prevents oscillation).

**Corner Detection:**
Requires ALL:
- Speed > min_speed
- |Lateral accel| > lat_thr (EMA filtered)
- |Yaw rate| > yaw_thr (EMA filtered)
- Lateral accel and yaw rate same sign (consistent turn direction)

**Why EMA filtering?** Raw accelerometer is noisy. EMA smooths while staying responsive (alpha=0.25 by default).

### Binary Telemetry Protocol (binary_telemetry.rs)

**Packet Structure (67 bytes):**
```c
struct TelemetryPacket {
    u16  header;         // 0xAA55
    u32  timestamp_ms;
    f32  ax, ay, az;     // Corrected accelerations (m/s²)
    f32  wz;             // Yaw rate (rad/s)
    f32  roll, pitch;    // Orientation (rad)
    f32  yaw;            // EKF yaw (rad)
    f32  x, y;           // Position (m)
    f32  vx, vy;         // Velocity (m/s)
    f32  speed_kmh;      // Speed (km/h)
    u8   mode;           // Bitflags: 0=IDLE, 1=ACCEL, 2=BRAKE, 4=CORNER, 5=ACCEL+CORNER, 6=BRAKE+CORNER
    f32  lat, lon;       // GPS (degrees, 0 if invalid)
    u8   gps_valid;      // 0 or 1
    u16  checksum;       // Sum of first 64 bytes
};  // Total: 67 bytes
```

**Why binary?** 67 bytes vs. ~300 bytes JSON. At 20 Hz: 1.3 KB/s vs. 6 KB/s.

**Decoder:** See `tools/python/tcp_telemetry_server.py` for reference implementation.

## Configuration and Calibration

### Network Configuration (config.rs)

**Two Operating Modes:**

**1. Access Point Mode (Default) - Mobile/Standalone Use**
- ESP32 creates WiFi network: `Blackbox` / `blackbox123`
- Fixed IP: `192.168.71.1`
- HTTP dashboard on port 80
- No router required
- No MQTT or UDP - dashboard only

**2. Station Mode - Network Integration**
- ESP32 connects to your WiFi network
- UDP telemetry streaming (20 Hz)
- MQTT status messages
- No HTTP dashboard

**Configuration via Environment Variables:**
```bash
# Access Point mode (default, no env vars needed)
cargo build --release

# Station mode with custom credentials
export WIFI_MODE="station"
export WIFI_SSID="YourNetwork"
export WIFI_PASSWORD="YourPassword"
export MQTT_BROKER="mqtt://192.168.1.100:1883"
export UDP_SERVER="192.168.1.100:9000"
cargo build --release

# GPS configuration (optional)
export GPS_MODEL="m9n"   # "neo6m" (default) or "m9n" for NEO-M9N
export GPS_RATE="25"     # Update rate in Hz (NEO-M9N: 1-25, NEO-6M: 1-5)
cargo build --release
```

**Defaults (config.rs):**
```rust
wifi_mode: AccessPoint
wifi_ssid: "Blackbox"
wifi_password: "blackbox123"
mqtt_broker: "mqtt://192.168.1.100:1883"  // Placeholder - set via env var for Station mode
udp_server: "192.168.1.100:9000"          // Placeholder - set via env var for Station mode
ws_port: 80
```

**Security Note:** Credentials compiled into firmware. Use environment variables to avoid committing secrets. AP mode password is public by design (mobile convenience).

### Hardware Pin Assignments (main.rs:49, 124-141)

```rust
// RGB LED
peripherals.rmt.channel0, peripherals.pins.gpio8

// IMU (WT901) - UART1, auto-detect baud (115200 preferred, falls back to 9600)
TX: GPIO18 → IMU RX
RX: GPIO19 ← IMU TX

// GPS (NEO-6M or NEO-M9N) - UART0
TX: GPIO5  → GPS RX
RX: GPIO4  ← GPS TX
// NEO-6M: 9600 baud, 5 Hz (NMEA only)
// NEO-M9N: 115200 baud, up to 25 Hz (UBX CFG-VALSET configuration)
```

**Why UART0 for GPS?** Console output is disabled in `sdkconfig.defaults` to free UART0. All logging goes through USB-JTAG instead.

**IMU Baud Rate Auto-Detection:**
At boot, firmware tries baud rates in order: 115200, 9600, 38400, 19200. Orange LED blinks indicate which baud is being tested. Green blinks confirm detection. This eliminates manual configuration when using different WT901 firmware versions.

### IMU Calibration (main.rs:387-420)

**Process:**
1. Device must be completely stationary on level surface
2. Collect `CALIB_SAMPLES` (500) accelerometer readings
3. Compute average acceleration → bias
4. Z-axis bias adjusted by -9.80665 m/s² (remove gravity)

**LED Feedback:**
- Yellow pulse: Calibration in progress (10%, 20%, 30%...)
- Duration: ~10 seconds at 200 Hz IMU rate

**Critical:** Any movement during calibration corrupts biases → poor EKF performance.

### IMU Configuration (Optional - Only If Needed)

**Factory default:** WT901 ships at 9600 baud / 10Hz output. Firmware auto-detects the baud rate.

**When to configure:** Check the diagnostics page after flashing. If IMU Rate shows ~10-20 Hz instead of ~200 Hz, run the configuration tool.

**Setup procedure (requires USB-serial adapter ~$5):**
1. Disconnect IMU from ESP32
2. Connect IMU to USB-serial adapter (TX→RX, RX→TX, GND, 5V)
3. Run: `python3 tools/python/configure_wt901.py /dev/ttyUSB0`
4. Settings are saved to IMU's EEPROM (persist across power cycles)
5. Reconnect IMU to ESP32

**What the tool does:**
1. Tries common baud rates (9600, 115200, etc.) to find the IMU
2. Sends unlock command: `FF AA 69 88 B5`
3. Sets baud rate to 115200: `FF AA 04 06 00`
4. Sets output rate to 200Hz: `FF AA 03 0B 00`
5. Saves to EEPROM: `FF AA 00 00 00`

**Rate options:**
- 10Hz, 20Hz, 50Hz: Work at 9600 baud
- 100Hz, 200Hz: Require 115200 baud (auto-configured by tool)

**Why 200Hz?** Higher IMU rate = smoother EKF prediction between GPS updates. 200Hz gives 40 IMU samples between 5Hz GPS updates. The system works at 10Hz but sensor fusion quality is reduced.

### Telemetry Rate (main.rs:35)

```rust
const TELEMETRY_INTERVAL_MS: u32 = 50;  // 20 Hz
```

Change to:
- 25ms → 40 Hz (may be unstable, CPU-limited)
- 100ms → 10 Hz (more stable, lower bandwidth)

IMU always samples at 200 Hz internal, GPS at 5 Hz. Telemetry rate only affects output frequency.

## Module Details

### imu.rs - WT901 UART Protocol

**Packet Format:**
```
[0x55] [TYPE] [DATA(8 bytes)] [CHECKSUM]
```

**Packet Types:**
- 0x51: Accelerometer (ax, ay, az + temp)
- 0x52: Gyroscope (wx, wy, wz + voltage)
- 0x53: Euler angles (roll, pitch, yaw + version)

**Parser State Machine:**
- Searches for header byte 0x55
- Accumulates 11-byte packet
- Verifies checksum (sum of bytes 0-9)
- Parses based on type

**Units:**
- Accel: ±16g range → multiply by 16*9.80665/32768
- Gyro: ±2000°/s → multiply by 2000*π/180/32768
- Angles: ±180° → multiply by 180/32768

### gps.rs - NMEA Parsing

**Supported Sentences:** GPRMC, GNRMC (Recommended Minimum)

**GPRMC Format:**
```
$GPRMC,hhmmss.ss,A,ddmm.mmmm,N,dddmm.mmmm,W,speed,course,DDMMYY,,,*CS
```

**Warmup Logic:**
- First 5 fixes averaged → reference lat/lon
- `is_warmed_up()` returns true after 5 fixes
- Local coordinates only valid after warmup

**Position-Based Speed:**
Calculates speed from position changes (backup when GPS speed unreliable).

### transforms.rs - Coordinate Transformations

**remove_gravity(ax, ay, az, roll, pitch):**
```rust
// Gravity vector in earth frame: [0, 0, -g]
// Rotate by transpose of DCM(roll, pitch) → gravity in body frame
// Subtract from measured acceleration
```
Result: acceleration due to motion only (no gravity component).

**body_to_earth(ax_b, ay_b, az_b, roll, pitch, yaw):**
```rust
// Apply DCM rotation: body → earth
// Uses 3-2-1 Euler angle convention (yaw-pitch-roll)
// Returns (ax_earth, ay_earth) - horizontal plane components
```

**Important:** Roll/pitch from IMU, yaw from EKF. IMU's magnetometer yaw is noisy (buildings, cars), so EKF fuses multiple sources.

### mode.rs - Driving Mode Classifier

**Hybrid Mode Detection** with GPS/IMU blending:
- Longitudinal acceleration (ACCEL/BRAKE) uses **blended GPS + IMU**
- Lateral acceleration (CORNER) uses **filtered IMU** (GPS can't sense lateral)
- GPS-derived acceleration computed from velocity changes at 25Hz
- Blending ratio adapts based on GPS rate and freshness

```
Independent hysteresis detection:

ACCEL:    entry at 0.10g → exit at 0.05g  (from blended lon accel)
BRAKE:    entry at -0.18g → exit at -0.09g (from blended lon accel)
CORNER:   entry at 0.12g lat + 0.05rad/s yaw → exit at 0.06g lat

Combined states:
- ACCEL + CORNER = corner exit (trail throttle)
- BRAKE + CORNER = corner entry (trail braking)
```

**Benefits of Hybrid Detection:**
- Mounting angle independent (GPS measures actual vehicle acceleration)
- No drift (GPS doesn't drift like IMU)
- Graceful fallback when GPS degrades

**All modes require:** speed > min_speed (2.0 m/s / 7.2 km/h)

### filter.rs - Biquad Low-Pass Filter

2nd-order Butterworth IIR filter for vibration removal.

**Purpose:**
- Remove engine vibration (30-100 Hz)
- Remove road texture noise (5-20 Hz)
- Preserve driving dynamics (0-2 Hz)

**Configuration:**
- Sample rate: 20 Hz (telemetry rate)
- Cutoff: 2 Hz
- Q = 0.707 (critically damped, no overshoot)

**Implementation:**
```rust
// Direct Form I biquad
y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
```

### fusion.rs - Sensor Fusion Module

Handles GPS/IMU blending, tilt correction, and continuous calibration.

**Components:**

1. **GpsAcceleration** - Computes longitudinal acceleration from GPS velocity
   - `accel = (speed_new - speed_old) / dt`
   - At 25Hz GPS: dt = 40ms, clean acceleration signal
   - Tracks staleness for fallback to IMU

2. **TiltEstimator** - Learns mounting offset when stopped
   - After 3 seconds stationary, averages earth-frame acceleration
   - This average IS the mounting error (gravity leakage from tilt)
   - Applies correction to all future readings
   - Relearns at every stop (adapts to device repositioning)

3. **GravityEstimator** - Learns gravity offset while driving
   - Detects "steady state": constant speed (±0.5 m/s), low yaw (±5°/s)
   - During steady state, expected acceleration ≈ 0
   - Slowly updates gravity estimate (α=0.02, ~50 second convergence)
   - Essential for track/canyon driving where stops are rare

4. **SensorFusion** - Main processor
   - Applies tilt correction
   - Applies gravity correction
   - Low-pass filters to remove vibration
   - Blends GPS and IMU for longitudinal acceleration

**GPS/IMU Blending Ratios (configurable in FusionConfig):**
```
GPS rate >= 20Hz, fresh:  70% GPS / 30% IMU  (high confidence)
GPS rate 10-20Hz:         50% GPS / 50% IMU  (medium confidence)
GPS rate < 10Hz:          30% GPS / 70% IMU  (low confidence)
GPS stale (>200ms):       0% GPS / 100% IMU  (fallback)
```

These ratios balance responsiveness (~80-100ms latency) with accuracy.
For track use, consider increasing GPS weights for maximum accuracy.

**Data Flow:**
```
GPS (25Hz) → GpsAcceleration → lon_accel_gps
                                    ↓
IMU → remove_gravity → body_to_earth → TiltCorrect → GravityCorrect → Biquad Filter
                                                                            ↓
                                                                    lon_accel_imu
                                                                            ↓
                                                            Blend(gps, imu) → lon_blended
                                                                            ↓
                                                    ┌───────────────────────┴───────────────────────┐
                                                    ↓                                               ↓
                                            mode.rs (classification)                    Dashboard (G-meter)
```

**Dashboard Display:** The G-meter shows the same blended longitudinal and filtered lateral
values that mode classification uses. What you see is what the algorithm sees.

### websocket_server.rs - HTTP Dashboard Server

**Overview:**
Built-in web dashboard for mobile viewing of live telemetry. Runs directly on ESP32 in Access Point mode.

**Features:**
- Mobile-optimized HTML dashboard (single-page, embedded in firmware)
- HTTP polling at ~20 Hz (40ms interval with WiFi overhead compensation)
- G-meter with trail visualization and max values
- Real-time mode detection display
- **Driving presets**: One-tap selection of Track, Canyon, City, Highway, or Custom
- Live settings configuration with validation (Custom mode shows sliders)
- CSV data export for recorded sessions

**Architecture:**
- HTTP server with 7 endpoints: `/`, `/api/telemetry`, `/api/status`, `/api/calibrate`, `/api/settings`, `/api/settings/set`
- Shared state via `Arc<TelemetryServerState>` for thread-safe telemetry updates
- Base64-encoded binary telemetry packets for JSON transport
- Settings changes propagated to main loop via atomic flags

**HTTP Polling Implementation:**
```javascript
// Client-side polling (~30 Hz)
setInterval(async () => {
  const r = await fetch('/api/telemetry');
  const j = await r.json();
  if (j.seq !== lastSeq) {
    // Decode base64, process binary packet
    // Update UI with EMA-filtered speed for smoothness
  }
}, 33);
```

**Why HTTP instead of WebSocket?**
- WebSocket handler's blocking loop monopolized ESP-IDF server threads
- Limited thread pool (4-5 threads) meant one WebSocket = blocked HTTP requests
- HTTP polling eliminates blocking, allows concurrent API calls
- 33ms interval targets ~30 Hz to match ESP32 telemetry update rate

**Preset System:**
- 4 built-in presets (Track, Canyon, City, Highway) with physics-based thresholds
- City preset is the default on boot
- Selecting a preset immediately applies settings to ESP32
- Custom mode reveals all sliders for fine-tuning
- Settings are not persisted across power cycles (no NVS storage yet)

**Settings Validation (Custom mode):**
Dashboard validates threshold ranges before sending to ESP32:
- Accel exit < accel entry
- Brake exit < brake entry
- Lateral exit < lateral entry

**Changed from WebSocket (ap_enabled branch):**
Originally used WebSocket push at 30Hz but this caused thread starvation. Refactored to HTTP polling for reliability.

### diagnostics.rs - System Health Monitoring

**Purpose:**
Real-time statistics for sensor rates, EKF health, GPS status, and system resources. Accessible via `/api/diagnostics` endpoint.

**Data Structures:**
- `SensorRates`: IMU Hz, GPS Hz (valid RMC fixes only), ZUPT rate (per minute), EKF predictions per GPS
- `EkfHealth`: Position/velocity/yaw uncertainty (σ), accelerometer biases
- `GpsHealth`: Fix valid, warmup complete, satellites, HDOP/PDOP
- `SystemHealth`: Free heap, uptime, telemetry sent/failed counts
- `WifiStatus`: Mode (AP/Station), SSID, IP address

**Thread-Safe Design:**
- `AtomicU32` counters for high-frequency IMU/GPS packet counts
- `Mutex<DiagnosticsInner>` for complex data structures
- `snapshot()` returns an immutable copy for HTTP response

**Rate Calculation:**
Uses exponential moving average (α=0.3) for smoother display:
```rust
imu_hz = α * (packets / dt) + (1 - α) * previous_hz
```

**GPS Rate Counting:**
GPS rate counts only valid RMC position fixes using `take_new_fix()`. This prevents over-counting when GPS outputs multiple sentence types (RMC, GGA, GSA) per fix cycle.

**ZUPT Rate:**
Displayed as updates per minute (rolling average), not cumulative count. Typical values: 0-60/min depending on driving pattern.

**Typical Values:**
- IMU: 199-200 Hz (at 115200 baud)
- GPS: 5 Hz (NEO-6M) or 8-25 Hz (NEO-M9N depending on config)
- ZUPT: 0-60/min (depends on stops)
- Free heap: ~60KB (stable during operation)

### udp_stream.rs - UDP Client (Station Mode)

**Features:**
- Disables Nagle's algorithm (`set_nodelay(true)`) for low latency
- 100ms write timeout
- Auto-reconnect on failure (after 100 failed sends)

**Error Handling:**
Counts failures, attempts reconnect periodically. MQTT also available as backup for critical status messages.

### wifi.rs - WiFi Manager

**Supports Two Modes:**

**Access Point Mode (Default):**
1. Initialize WiFi with NVS (persistent storage)
2. Configure as AP (network creator)
3. Create WiFi network with SSID/password
4. Start DHCP server (assigns IPs to clients)
5. Fixed IP: 192.168.71.1
6. Clients connect directly to ESP32

**Station Mode:**
1. Initialize WiFi with NVS
2. Configure as station (client) mode
3. Connect to existing AP with SSID/password
4. Wait for DHCP lease from router
5. Log assigned IP address

**Mode Selection:**
Set at compile time via environment variable:
```bash
export WIFI_MODE="station"  # or "ap" for access point
cargo build --release
```

**2.4 GHz Only:** ESP32 doesn't support 5 GHz WiFi.

### rgb_led.rs - WS2812 LED Control

**Available Colors:** red, green, blue, yellow, cyan, magenta, orange, white

**LED Status Codes (Access Point Mode):**

Boot sequence:
- Magenta (3 blinks): Boot sequence started (AP mode)
- Green (5 blinks): WiFi AP started
- Cyan (3 blinks): HTTP server ready
- Yellow (pulsing): IMU calibrating
- Red (continuous slow blink): WiFi AP failed (critical error)

**LED Status Codes (Station Mode):**

Boot sequence:
- Blue (3 blinks): Boot sequence started (Station mode)
- Green (5 blinks): WiFi connected to network
- Magenta (3 blinks): MQTT connected
- Red (5 fast blinks): MQTT connection failed
- Cyan (3 blinks): UDP socket ready
- Yellow (pulsing): IMU calibrating
- Red (continuous slow blink): WiFi failed at boot (critical error)

**Main Loop (operational, both modes):**
- Cyan (2s pulse): GPS locked, operational
- Yellow (fast blink): Waiting for GPS lock
- Green-white (3 alternating flashes): Settings changed via dashboard
- Orange (3 blinks, repeating): WiFi disconnected (repeats every 5s)
- Red (2 blinks, repeating): MQTT disconnected (Station mode only, repeats every 5s)

**Important:** MQTT status LED (red blinks) only applies to Station mode. In Access Point mode, MQTT is not used.

Useful for debugging without serial monitor.

## Common Pitfalls and Solutions

### 1. Coordinate Frame Confusion
**Problem:** Mixing up which frame accelerations are in.
**Solution:** Always trace the data flow: body → earth (remove gravity) → vehicle (mode classification). Check units: raw IMU is in m/s², earth frame removes gravity, vehicle frame splits into lon/lat.

### 2. ZUPT Threshold Tuning
**Problem:** Too sensitive → triggers while moving (position jumps). Too loose → drift at stoplights.
**Solution:** Test in real vehicle. Watch for position jumps when starting from stop. Adjust `ACC_THR`, `WZ_THR`, `GPS_SPEED_THR` in `is_stationary()`.

### 3. EKF Divergence
**Problem:** Position/velocity estimates explode or oscillate.
**Causes:**
- Wrong Q/R tuning (trust model too much or sensors too little)
- Coordinate frame errors in prediction step
- NaN/Inf propagation from bad math (division by zero)

**Solution:**
- Log `ekf.x` and `ekf.p` to serial, watch for NaN
- Start with default Q/R, change one at a time
- Verify accelerations are reasonable (< 5 m/s² typically)

### 4. Telemetry Packet Size Changes
**Problem:** Adding fields breaks decoder compatibility.
**Solution:** Binary protocol is fixed size. If adding data, either:
- Replace existing unused field
- Add padding to maintain 67 bytes
- Version the protocol (add version field, decoders check it)

### 5. GPS Origin Reset
**Problem:** Data from different sessions has different coordinate origins.
**Solution:** Log reference lat/lon in telemetry or status message. Post-processing can transform all sessions to common origin.

### 6. WiFi Credentials in Firmware
**Problem:** Accidental commit of passwords to public repo.
**Solution:** Use environment variables at build time, or separate config file (gitignored). Example:
```rust
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");
```
Then: `WIFI_SSID=MyNetwork WIFI_PASSWORD=MyPass cargo build`

## ESP-IDF Configuration (sdkconfig.defaults)

**Key Settings:**
```ini
CONFIG_IDF_TARGET="esp32c3"           # Target chip
CONFIG_FREERTOS_HZ=1000               # 1ms tick rate
CONFIG_ESP_CONSOLE_NONE=y             # Disable console (frees UART0 for GPS)
CONFIG_LWIP_MAX_SOCKETS=16            # TCP sockets for WiFi
CONFIG_MQTT_PROTOCOL_311=y            # MQTT v3.1.1
CONFIG_ESP32C3_WIFI_DYNAMIC_TX_BUFFER_NUM=32  # WiFi buffers
```

**Why console disabled?** ESP32-C3 has limited UARTs. UART0 needed for GPS, UART1 for IMU. Logging goes through USB-JTAG instead.

## Testing and Validation

### Bench Testing (no vehicle)
1. Flash firmware, watch serial output
2. Verify IMU calibration completes (stationary)
3. Take device outside, verify GPS lock (cyan LED pulse)
4. Run `tools/python/tcp_telemetry_server.py` on laptop
5. Shake device, verify accelerations respond
6. Walk around, verify position updates

### In-Vehicle Testing
1. Mount rigidly (flex corrupts IMU readings)
2. Power on, wait for calibration (don't touch!)
3. Wait for GPS lock (can take 30-60s cold start)
4. Drive around parking lot, watch telemetry
5. Verify mode transitions (IDLE → ACCEL → IDLE)
6. Check position tracking follows actual path
7. Stop at red light, verify ZUPT prevents drift

### Debugging Tools
```bash
# Monitor serial output
espflash monitor

# Watch telemetry live
python3 tools/python/tcp_telemetry_server.py

# Check GPS sentences raw
# (Temporarily modify gps.rs to print raw NMEA)

# Verify WiFi connection
ping <ESP32_IP>
```

## Performance Characteristics

**Achieved:**
- 200 Hz IMU sampling (5ms between packets)
- 20 Hz telemetry output (50ms intervals)
- <50ms latency sensor → TCP transmission
- ±1m position accuracy between GPS fixes
- Zero drift when stationary (ZUPT working)

**Limitations:**
- GPS only 5 Hz (external sensor limit)
- WiFi range ~30m indoors, ~100m outdoors
- 400KB RAM on ESP32-C3 (monitor heap usage)
- Single-threaded (FreeRTOS but no parallelism used)

**Heap Usage:** ~60KB baseline, stable during operation. Log shows free heap every 5 seconds (`main.rs:192-202`).

## Future Enhancement Areas

See `CHANGELOG.md` for roadmap. Common enhancement requests:
- Additional IMU support (MPU6050, BMI088, LSM6DSO)
- RTK GPS for cm-level accuracy
- CAN bus integration for OBD2 data (RPM, throttle, temps)
- SD card logging (offline data capture)
- BLE streaming for mobile apps
- Web-based configuration interface

Contributions welcome! See `CONTRIBUTING.md`.
