# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Active Wing is an ESP32-C3 vehicle telemetry system that performs real-time sensor fusion of IMU and GPS data using an Extended Kalman Filter (EKF). It streams binary telemetry at 20 Hz over TCP for live data acquisition during track days, autocross, rally, and vehicle dynamics research.

**Target Hardware:** ESP32-C3 microcontroller
**Sensors:** WT901 9-axis IMU (UART), NEO-6M GPS (UART)
**Output:** TCP streaming (20 Hz binary telemetry), MQTT (status messages)
**Cost:** ~$30 in parts vs. $500+ commercial alternatives

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

# Run linter
cargo clippy -- -D warnings

# Run tests (limited - most require hardware)
cargo test --lib --bins

# Flash to ESP32-C3 and monitor serial output
cargo espflash flash --monitor
cargo espflash flash --release --monitor

# Just monitor serial output (after flashing)
espflash monitor
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

# Check Python syntax
python3 -m py_compile tools/python/*.py
```

### Project Structure
```
active_wing/
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
│  WT901 IMU @ 200Hz          NEO-6M GPS @ 5Hz               │
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
│           Classify: IDLE/ACCEL/BRAKE/CORNER               │
│                │                                            │
│                ▼                                            │
│      binary_telemetry.rs                                   │
│      Pack 66-byte packet                                   │
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

**Modes:** IDLE, ACCEL, BRAKE, CORNER

**How it works:**
1. Transform earth-frame acceleration → vehicle frame using current yaw
2. Split into longitudinal (forward/back) and lateral (left/right)
3. EMA filter both lateral accel and yaw rate
4. State machine with hysteresis (different entry/exit thresholds)

**Corner Detection (`mode.rs:117-122`):**
Requires ALL:
- Speed > 2.0 m/s
- |Lateral accel| > 0.20g
- |Yaw rate| > 0.07 rad/s
- Lateral accel and yaw rate same sign (consistent turn direction)

**Why EMA filtering?** Raw accelerometer is noisy. EMA smooths while staying responsive (alpha=0.20 by default).

### Binary Telemetry Protocol (binary_telemetry.rs)

**Packet Structure (66 bytes):**
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
    u8   mode;           // 0=IDLE, 1=ACCEL, 2=BRAKE, 3=CORNER
    f32  lat, lon;       // GPS (degrees, 0 if invalid)
    u8   gps_valid;      // 0 or 1
    u16  checksum;       // Sum of first 64 bytes
};  // Total: 66 bytes
```

**Why binary?** 66 bytes vs. ~300 bytes JSON. At 20 Hz: 1.3 KB/s vs. 6 KB/s.

**Decoder:** See `tools/python/tcp_telemetry_server.py` for reference implementation.

## Configuration and Calibration

### Network Configuration (main.rs:29-33)

**IMPORTANT:** These are hardcoded and must be changed for your environment:

```rust
const WIFI_SSID: &str = "GiraffeWireless";
const WIFI_PASSWORD: &str = "basicchair411";
const MQTT_BROKER: &str = "mqtt://192.168.50.46:1883";
const TCP_SERVER: &str = "192.168.50.46:9000";
```

**Security Note:** WiFi credentials are in plaintext in firmware. Don't commit real credentials to public repos. Consider using environment variables or a secrets file (gitignored).

### Hardware Pin Assignments (main.rs:49, 124-141)

```rust
// RGB LED
peripherals.rmt.channel0, peripherals.pins.gpio8

// IMU (WT901) - UART1, 9600 baud
TX: GPIO18 → IMU RX
RX: GPIO19 ← IMU TX

// GPS (NEO-6M) - UART0, 9600 baud
TX: GPIO5  → GPS RX
RX: GPIO4  ← GPS TX

// GPS configured for 5 Hz update rate (line 143-148)
```

**Why UART0 for GPS?** Console output is disabled in `sdkconfig.defaults` to free UART0. All logging goes through USB-JTAG instead.

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

**State Machine:**
```
IDLE ──┬─→ ACCEL   (if a_lon > 0.21g)
       ├─→ BRAKE   (if a_lon < -0.25g)
       └─→ CORNER  (if |a_lat| > 0.20g && |wz| > 0.07 rad/s)

ACCEL ──→ IDLE     (if a_lon < 0.11g)
BRAKE ──→ IDLE     (if a_lon > -0.12g)
CORNER ─→ IDLE     (if speed < 2 m/s or low lateral forces)
```

**Hysteresis:** Entry threshold > exit threshold prevents oscillation.

**Speed Display Filtering:**
Separate EMA filter (alpha=0.6, faster than mode detection) for smooth speed display. Set to exactly 0 when < 0.5 km/h.

### tcp_stream.rs - TCP Client

**Features:**
- Disables Nagle's algorithm (`set_nodelay(true)`) for low latency
- 100ms write timeout
- Auto-reconnect on failure (after 100 failed sends)

**Error Handling:**
Counts failures, attempts reconnect periodically. MQTT also available as backup for critical status messages.

### wifi.rs - WiFi Manager

**Connection Sequence:**
1. Initialize WiFi with NVS (persistent storage)
2. Configure as station (client) mode
3. Connect to AP with SSID/password
4. Wait for DHCP lease
5. Log IP address

**2.4 GHz Only:** ESP32 doesn't support 5 GHz WiFi.

### rgb_led.rs - WS2812 LED Control

**LED Status Codes:**
- Blue (3 blinks): Boot sequence
- Green (5 blinks): WiFi connected
- Red (slow blink): WiFi failed
- Cyan (3 blinks): TCP connected
- Yellow (pulsing): IMU calibrating
- Cyan (2s pulse): GPS locked, operational
- Yellow (fast blink): Waiting for GPS lock

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
- Add padding to maintain 66 bytes
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
