# Implementation Plan: NEO-6M/7M/8M GPS Support + Diagnostics Page

## Summary

1. Rename `drivers/neo6m` → `drivers/ublox-neo`
2. Add `GpsModel` enum (Neo6M/Neo7M/Neo8M) configurable via GPS_MODEL env var
3. Add UBX CFG-RATE command generation to driver
4. Send UBX command on boot to configure GPS rate
5. Update `warmup_fixes` default from 5 to 10
6. Add diagnostics system (sensor rates, EKF health, system stats, WiFi mode, config)
7. Fix documentation inconsistencies

## Performance Requirements

- Main telemetry path MUST NOT be affected
- Counter increments only (no allocations in hot path)
- Rate calculation once per second only
- Diagnostics uses separate state/mutex from telemetry
- /diagnostics endpoint only active when page is viewed

---

## PHASE 1: Crate Rename (neo6m → ublox-neo)

### 1.1 Directory Rename

- Rename `drivers/neo6m/` → `drivers/ublox-neo/`

### 1.2 Cargo.toml Updates

| File | Change |
|------|--------|
| `drivers/ublox-neo/Cargo.toml` | `name = "ublox-neo"`, add `[lib] name = "ublox_neo"`, update keywords |
| `Cargo.toml` (root) | `"drivers/neo6m"` → `"drivers/ublox-neo"` |
| `sensors/blackbox/Cargo.toml` | `neo6m = {...}` → `ublox-neo = { path = "../../drivers/ublox-neo" }` |

### 1.3 Import Updates

| File | Change |
|------|--------|
| `sensors/blackbox/src/gps.rs` | `pub use neo6m::*` → `pub use ublox_neo::*` |
| `sensors/blackbox/src/system.rs:62` | `&neo6m::GpsFix` → `&ublox_neo::GpsFix` |

### 1.4 Documentation Updates (ALL files with neo6m references)

| File | Lines | Change |
|------|-------|--------|
| `drivers/ublox-neo/README.md` | all | Full rewrite for NEO-6M/7M/8M |
| `drivers/ublox-neo/src/lib.rs:20` | doc comment | Update crate name in example |
| `framework/README.md:232` | 232 | `"neo6m driver"` → `"ublox-neo driver"` |
| `sensors/README.md` | 64, 185 | Update neo6m references to ublox-neo |
| `CONTRIBUTING.md` | 146, 215 | Update neo6m references |
| `docs/SENSOR_TOOLKIT_GUIDE.md` | 25, 426, 558 | Update neo6m references |

### 1.5 Unit Tests

- 4 tests exist in `drivers/neo6m/src/lib.rs:588-618`
- They move with the crate rename - no changes needed
- Run `cargo test -p ublox-neo` to verify after rename

---

## PHASE 2: GPS Configuration

### 2.1 Add GpsModel Enum (sensors/blackbox/src/config.rs)

```rust
/// Supported u-blox NEO GPS modules
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GpsModel {
    Neo6M,   // Max 5 Hz
    Neo7M,   // Max 10 Hz
    Neo8M,   // Max 10 Hz (18 Hz possible in single-GNSS mode, but 10 Hz default for accuracy)
}

impl Default for GpsModel {
    fn default() -> Self {
        // Check environment variable at compile time
        match option_env!("GPS_MODEL") {
            Some(m) => match m.to_lowercase().as_str() {
                "neo6m" | "6m" | "6" => GpsModel::Neo6M,
                "neo7m" | "7m" | "7" => GpsModel::Neo7M,
                "neo8m" | "8m" | "8" => GpsModel::Neo8M,
                _ => GpsModel::Neo7M,  // Default
            },
            None => GpsModel::Neo7M,  // Default
        }
    }
}

impl GpsModel {
    pub fn max_rate_hz(&self) -> u8 {
        match self {
            GpsModel::Neo6M => 5,
            GpsModel::Neo7M => 10,
            GpsModel::Neo8M => 10,  // 18 Hz possible but 10 Hz better for multi-GNSS accuracy
        }
    }

    pub fn measurement_period_ms(&self) -> u16 {
        1000 / self.max_rate_hz() as u16
    }

    pub fn name(&self) -> &'static str {
        match self {
            GpsModel::Neo6M => "NEO-6M",
            GpsModel::Neo7M => "NEO-7M",
            GpsModel::Neo8M => "NEO-8M",
        }
    }
}
```

### 2.2 Update GpsConfig (sensors/blackbox/src/config.rs)

```rust
#[derive(Debug, Clone, Copy)]
pub struct GpsConfig {
    pub model: GpsModel,
    pub warmup_fixes: u8,  // Default: 10 (1 second at 10 Hz)
}

impl Default for GpsConfig {
    fn default() -> Self {
        Self {
            model: GpsModel::default(),  // From GPS_MODEL env var
            warmup_fixes: 10,
        }
    }
}
```

### 2.3 Document GPS_MODEL env var

Add to CLAUDE.md and README.md:

```markdown
### GPS Module Selection

Set at compile time via environment variable:

```bash
# For NEO-6M (5 Hz max)
export GPS_MODEL="neo6m"

# For NEO-7M (10 Hz max) - DEFAULT
export GPS_MODEL="neo7m"

# For NEO-8M (10 Hz default, 18 Hz possible in single-GNSS mode)
export GPS_MODEL="neo8m"

cargo build --release
```
```

### 2.4 Add UBX Module (drivers/ublox-neo/src/lib.rs)

```rust
/// UBX protocol support for u-blox GPS configuration
pub mod ubx {
    /// Generate UBX CFG-RATE command for given measurement period
    /// Returns 14-byte command with checksum
    pub fn cfg_rate_command(measurement_period_ms: u16) -> [u8; 14] {
        let mut cmd = [
            0xB5, 0x62,       // Sync chars
            0x06, 0x08,       // CFG-RATE class/id
            0x06, 0x00,       // Payload length (6 bytes)
            0x00, 0x00,       // measRate (filled below)
            0x01, 0x00,       // navRate: 1 cycle
            0x01, 0x00,       // timeRef: GPS time
            0x00, 0x00,       // Checksum (filled below)
        ];
        cmd[6] = (measurement_period_ms & 0xFF) as u8;
        cmd[7] = (measurement_period_ms >> 8) as u8;
        let (ck_a, ck_b) = fletcher_checksum(&cmd[2..12]);
        cmd[12] = ck_a;
        cmd[13] = ck_b;
        cmd
    }

    fn fletcher_checksum(data: &[u8]) -> (u8, u8) {
        let (mut a, mut b) = (0u8, 0u8);
        for byte in data {
            a = a.wrapping_add(*byte);
            b = b.wrapping_add(a);
        }
        (a, b)
    }
}
```

### 2.5 Send UBX on Init (sensors/blackbox/src/main.rs)

After GPS UART init:

```rust
// Configure GPS update rate via UBX command
let ubx_cmd = ublox_neo::ubx::cfg_rate_command(config.gps.model.measurement_period_ms());
gps_uart.write(&ubx_cmd).ok();
info!(
    "GPS configured: {} @ {} Hz",
    config.gps.model.name(),
    config.gps.model.max_rate_hz()
);
```

---

## PHASE 3: Diagnostics System

### 3.1 Add DiagnosticsState (sensors/blackbox/src/diagnostics.rs) - NEW FILE

```rust
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, Default)]
pub struct SensorRates {
    pub imu_hz: f32,
    pub gps_hz: f32,
    pub imu_expected_hz: f32,   // 200
    pub gps_expected_hz: f32,   // From config
}

#[derive(Debug, Clone, Default)]
pub struct EkfHealth {
    pub position_sigma: f32,
    pub velocity_sigma: f32,
    pub yaw_sigma_deg: f32,
    pub bias_x: f32,
    pub bias_y: f32,
}

#[derive(Debug, Clone, Default)]
pub struct SystemHealth {
    pub free_heap_bytes: u32,
    pub uptime_seconds: u32,
    pub telemetry_sent: u32,
    pub telemetry_failed: u32,
}

#[derive(Debug, Clone, Default)]
pub struct GpsHealth {
    pub model_name: &'static str,      // "NEO-6M", "NEO-7M", "NEO-8M"
    pub configured_rate_hz: u8,
    pub fix_valid: bool,
    pub warmup_complete: bool,
}

#[derive(Debug, Clone, Default)]
pub struct WifiStatus {
    pub mode: &'static str,            // "Access Point" or "Station"
    pub ssid: &'static str,
    pub ip_address: &'static str,      // "192.168.71.1" for AP mode
}

#[derive(Debug, Clone, Default)]
pub struct ConfigSnapshot {
    pub telemetry_rate_hz: u32,
    pub gps_model: &'static str,
    pub gps_warmup_fixes: u8,
}

#[derive(Debug, Clone, Default)]
pub struct DiagnosticsData {
    pub sensor_rates: SensorRates,
    pub ekf_health: EkfHealth,
    pub system_health: SystemHealth,
    pub gps_health: GpsHealth,
    pub wifi_status: WifiStatus,
    pub config: ConfigSnapshot,
    pub imu_temp_celsius: f32,
}

pub struct DiagnosticsState {
    data: Mutex<DiagnosticsData>,
}

impl DiagnosticsState {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            data: Mutex::new(DiagnosticsData::default()),
        })
    }

    pub fn update<F>(&self, f: F) where F: FnOnce(&mut DiagnosticsData) {
        if let Ok(mut data) = self.data.try_lock() {
            f(&mut data);
        }
        // Non-blocking: skip if locked
    }

    pub fn snapshot(&self) -> DiagnosticsData {
        self.data.lock().unwrap().clone()
    }
}
```

### 3.2 Add Rate Counters to SensorManager (sensors/blackbox/src/system.rs)

```rust
pub struct SensorManager {
    // ... existing fields
    imu_packet_count: u32,
    gps_fix_count: u32,
    last_rate_calc_ms: u32,
    pub imu_rate_hz: f32,
    pub gps_rate_hz: f32,
}

impl SensorManager {
    pub fn update_rates(&mut self, now_ms: u32) {
        if now_ms.wrapping_sub(self.last_rate_calc_ms) >= 1000 {
            self.imu_rate_hz = self.imu_packet_count as f32;
            self.gps_rate_hz = self.gps_fix_count as f32;
            self.imu_packet_count = 0;
            self.gps_fix_count = 0;
            self.last_rate_calc_ms = now_ms;
        }
    }
}
```

### 3.3 Add Diagnostics Endpoints (sensors/blackbox/src/websocket_server.rs)

Add endpoints:
- `GET /diagnostics` → HTML diagnostics page (auto-refresh 1s)
- `GET /api/diagnostics` → JSON diagnostics data

### 3.4 Wire Up in Main Loop (sensors/blackbox/src/main.rs)

At startup, populate static config:

```rust
diagnostics_state.update(|d| {
    d.wifi_status.mode = if is_ap_mode { "Access Point" } else { "Station" };
    d.wifi_status.ssid = config.network.wifi_ssid;
    d.wifi_status.ip_address = if is_ap_mode { "192.168.71.1" } else { "DHCP" };
    d.config.telemetry_rate_hz = 1000 / config.telemetry.interval_ms;
    d.config.gps_model = config.gps.model.name();
    d.config.gps_warmup_fixes = config.gps.warmup_fixes;
    d.gps_health.model_name = config.gps.model.name();
    d.gps_health.configured_rate_hz = config.gps.model.max_rate_hz();
});
```

In main loop (once per second):

```rust
sensors.update_rates(now_ms);
diagnostics_state.update(|d| {
    d.sensor_rates.imu_hz = sensors.imu_rate_hz;
    d.sensor_rates.gps_hz = sensors.gps_rate_hz;
    d.sensor_rates.gps_expected_hz = config.gps.model.max_rate_hz() as f32;
    d.sensor_rates.imu_expected_hz = 200.0;
    d.ekf_health.position_sigma = estimator.ekf.position_sigma();
    d.ekf_health.bias_x = estimator.ekf.x[5];
    d.ekf_health.bias_y = estimator.ekf.x[6];
    d.system_health.free_heap_bytes = unsafe { esp_get_free_heap_size() };
    d.system_health.uptime_seconds = now_ms / 1000;
    d.gps_health.fix_valid = sensors.gps_parser.last_fix().valid;
    d.gps_health.warmup_complete = sensors.gps_parser.is_warmed_up();
});
```

### 3.5 Diagnostics HTML Page Layout

```
┌─────────────────────────────────────────────────────────┐
│  BLACKBOX DIAGNOSTICS                      Uptime: 5m  │
├─────────────────────────────────────────────────────────┤
│  CONFIGURATION                                          │
│  WiFi Mode: Access Point    SSID: Blackbox             │
│  IP: 192.168.71.1           Telemetry: 30 Hz           │
│  GPS Model: NEO-7M          Warmup Fixes: 10           │
├─────────────────────────────────────────────────────────┤
│  SENSORS                                                │
│  ┌──────────────────┐  ┌──────────────────┐            │
│  │ IMU              │  │ GPS              │            │
│  │ Rate: 198 Hz ✓   │  │ Rate: 10 Hz ✓    │            │
│  │ Expected: 200 Hz │  │ Expected: 10 Hz  │            │
│  │ Temp: 32°C       │  │ Fix: Valid ✓     │            │
│  └──────────────────┘  │ Warmup: Done ✓   │            │
│                        └──────────────────┘            │
├─────────────────────────────────────────────────────────┤
│  EKF HEALTH                                             │
│  Position σ: 2.3m   Velocity σ: 0.4m/s   Yaw σ: 3°     │
│  Bias X: 0.02 m/s²  Bias Y: -0.01 m/s²                 │
├─────────────────────────────────────────────────────────┤
│  SYSTEM                                                 │
│  Heap: 142 KB free    Packets: 12,450 / 3 failed       │
└─────────────────────────────────────────────────────────┘
```

Green checkmark (✓) when actual rate is within 10% of expected.
Red warning when rate is significantly off.

---

## PHASE 4: Documentation Fixes

### 4.1 CLAUDE.md

| Line | Current | Fix |
|------|---------|-----|
| 24 | `"IMU (50Hz)"` | `"IMU (200Hz)"` |
| 107 | `"NEO-6M GPS @ 5Hz"` | `"u-blox NEO GPS (6M/7M/8M) @ 5-10Hz (configurable)"` |
| 366 | `"TELEMETRY_INTERVAL_MS: u32 = 50"` | `"interval_ms: 33 (30 Hz)"` |
| 373 | `"GPS at 5 Hz"` | `"GPS at 5-10 Hz (model-dependent)"` |
| NEW | - | Add "GPS Module Selection" section with GPS_MODEL env var |
| NEW | - | Add "Diagnostics Page" section |

### 4.2 Main README.md

- Fix `"IMU (50Hz)"` → `"IMU (200Hz)"`
- Update hardware to show NEO-6M/7M/8M support
- Add GPS_MODEL configuration example

### 4.3 drivers/ublox-neo/README.md (Full Rewrite)

Document:
- Supported modules (6M/7M/8M) with rate limits
- NEO-8M 18 Hz note (single-GNSS mode only)
- NMEA parsing
- UBX rate configuration
- Usage examples

---

## PHASE 5: Testing

- [ ] Crate rename compiles: `cargo check`
- [ ] Driver tests pass: `cargo test -p ublox-neo`
- [ ] Framework tests pass: `cargo test -p sensor-fusion`
- [ ] UBX checksum verified against u-center output
- [ ] GPS_MODEL env var works for all 3 values
- [ ] GPS rate configurable (test 5 Hz and 10 Hz)
- [ ] Diagnostics page loads without affecting telemetry rate
- [ ] Diagnostics shows correct WiFi mode and config
- [ ] Rate counters accurate vs expected
- [ ] Main dashboard still 30 Hz responsive
- [ ] Documentation grep for "neo6m" returns 0
- [ ] Documentation grep for "50Hz IMU" returns 0

---

## Files Summary

| Category | Files |
|----------|-------|
| **Renamed** | `drivers/neo6m/` → `drivers/ublox-neo/` |
| **New** | `sensors/blackbox/src/diagnostics.rs` |
| **Modified (code)** | `Cargo.toml` (root), `sensors/blackbox/Cargo.toml`, `sensors/blackbox/src/config.rs`, `sensors/blackbox/src/gps.rs`, `sensors/blackbox/src/system.rs`, `sensors/blackbox/src/main.rs`, `sensors/blackbox/src/websocket_server.rs`, `drivers/ublox-neo/src/lib.rs`, `drivers/ublox-neo/Cargo.toml` |
| **Modified (docs)** | `CLAUDE.md`, `README.md`, `drivers/ublox-neo/README.md`, `framework/README.md`, `sensors/README.md`, `CONTRIBUTING.md`, `docs/SENSOR_TOOLKIT_GUIDE.md` |

---

## Execution Instructions

Start with Phase 1 (crate rename), verify tests pass, then Phase 2 (GPS config), then Phase 3 (diagnostics), then Phase 4 (docs). Verify each phase compiles before proceeding.
