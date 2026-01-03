 Implementation Plan: NEO-6M/7M/8M GPS Support

  Overview

  Add support for u-blox NEO-6M, NEO-7M, and NEO-8M GPS modules with configurable update rates via UBX protocol. Rename driver crate from neo6m to ublox-neo for accuracy.

  ---
  Phase 1: Crate Rename (neo6m → ublox-neo)

  1.1 Directory and Crate Rename

  | Action              | Path                                                       |
  |---------------------|------------------------------------------------------------|
  | Rename directory    | drivers/neo6m/ → drivers/ublox-neo/                        |
  | Update package name | drivers/ublox-neo/Cargo.toml: name = "ublox-neo"           |
  | Update lib name     | drivers/ublox-neo/Cargo.toml: add [lib] name = "ublox_neo" |

  1.2 Update Workspace References

  | File                        | Change                                                                      |
  |-----------------------------|-----------------------------------------------------------------------------|
  | Cargo.toml (root)           | members: "drivers/neo6m" → "drivers/ublox-neo"                              |
  | sensors/blackbox/Cargo.toml | neo6m = { path = "..." } → ublox-neo = { path = "../../drivers/ublox-neo" } |

  1.3 Update Import Statements

  | File                           | Change                                  |
  |--------------------------------|-----------------------------------------|
  | sensors/blackbox/src/gps.rs    | pub use neo6m::* → pub use ublox_neo::* |
  | sensors/blackbox/src/system.rs | &neo6m::GpsFix → &ublox_neo::GpsFix     |

  1.4 Update Documentation References

  | File                        | Change                              |
  |-----------------------------|-------------------------------------|
  | drivers/ublox-neo/README.md | Rewrite for NEO-6M/7M/8M support    |
  | framework/README.md:232     | "neo6m driver" → "ublox-neo driver" |
  | sensors/README.md:64,185    | Update neo6m references             |

  ---
  Phase 2: Add UBX Configuration Support

  2.1 Add GpsModel Enum (sensors/blackbox/src/config.rs)

  /// Supported u-blox NEO GPS modules
  #[derive(Debug, Clone, Copy, PartialEq, Default)]
  pub enum GpsModel {
      Neo6M,   // Max 5 Hz
      #[default]
      Neo7M,   // Max 10 Hz
      Neo8M,   // Max 10 Hz
  }

  impl GpsModel {
      /// Maximum supported update rate for this module
      pub fn max_rate_hz(&self) -> u8 {
          match self {
              GpsModel::Neo6M => 5,
              GpsModel::Neo7M | GpsModel::Neo8M => 10,
          }
      }

      /// Measurement period in milliseconds
      pub fn measurement_period_ms(&self) -> u16 {
          1000 / self.max_rate_hz() as u16
      }
  }

  2.2 Update GpsConfig (sensors/blackbox/src/config.rs)

  #[derive(Debug, Clone, Copy)]
  pub struct GpsConfig {
      pub model: GpsModel,       // NEW: Which GPS module is connected
      pub warmup_fixes: u8,      // Default: 10 (1 second at 10 Hz)
  }

  impl Default for GpsConfig {
      fn default() -> Self {
          Self {
              model: GpsModel::Neo7M,  // Default to newer module
              warmup_fixes: 10,        // 1 second warmup at 10 Hz
          }
      }
  }

  2.3 Add UBX Command Generation (drivers/ublox-neo/src/lib.rs)

  /// UBX protocol support for configuration
  pub mod ubx {
      /// Generate UBX CFG-RATE command for given measurement period
      /// Returns 14-byte command with checksum
      pub fn cfg_rate_command(measurement_period_ms: u16) -> [u8; 14] {
          let mut cmd = [
              0xB5, 0x62,       // Sync chars
              0x06, 0x08,       // CFG-RATE class/id
              0x06, 0x00,       // Payload length (6 bytes)
              0x00, 0x00,       // measRate (placeholder)
              0x01, 0x00,       // navRate: 1 cycle
              0x01, 0x00,       // timeRef: GPS time
              0x00, 0x00,       // Checksum (placeholder)
          ];

          // Set measurement rate (little-endian)
          cmd[6] = (measurement_period_ms & 0xFF) as u8;
          cmd[7] = (measurement_period_ms >> 8) as u8;

          // Calculate checksum (8-bit Fletcher)
          let (ck_a, ck_b) = checksum(&cmd[2..12]);
          cmd[12] = ck_a;
          cmd[13] = ck_b;

          cmd
      }

      fn checksum(data: &[u8]) -> (u8, u8) {
          let mut ck_a: u8 = 0;
          let mut ck_b: u8 = 0;
          for byte in data {
              ck_a = ck_a.wrapping_add(*byte);
              ck_b = ck_b.wrapping_add(ck_a);
          }
          (ck_a, ck_b)
      }
  }

  2.4 Send UBX Command on Init (sensors/blackbox/src/main.rs)

  After GPS UART initialization (~line 237):

  // Configure GPS update rate via UBX command
  let ubx_cmd = ublox_neo::ubx::cfg_rate_command(
      config.gps.model.measurement_period_ms()
  );
  gps_uart.write(&ubx_cmd).ok();
  info!(
      "GPS configured: {:?} @ {} Hz",
      config.gps.model,
      config.gps.model.max_rate_hz()
  );

  ---
  Phase 3: Fix Documentation Inconsistencies

  3.1 CLAUDE.md Fixes

  | Line | Current                                           | Fix                                                  |
  |------|---------------------------------------------------|------------------------------------------------------|
  | 24   | "GPS (5Hz) and IMU (50Hz)"                        | "GPS (5-10Hz configurable) and IMU (200Hz)"          |
  | 107  | "NEO-6M GPS @ 5Hz"                                | "u-blox NEO GPS @ 5-10Hz (configurable)"             |
  | 366  | "const TELEMETRY_INTERVAL_MS: u32 = 50; // 20 Hz" | Remove or update to match actual default (33ms/30Hz) |
  | 373  | "GPS at 5 Hz"                                     | "GPS at 5-10 Hz (configurable)"                      |

  3.2 Main README.md Fixes

  | Location         | Current                    | Fix                                   |
  |------------------|----------------------------|---------------------------------------|
  | Line ~24         | "GPS (5Hz) and IMU (50Hz)" | "GPS (5-10Hz) and IMU (200Hz)"        |
  | Hardware section | "NEO-6M GPS"               | "u-blox NEO GPS (6M/7M/8M supported)" |

  3.3 sensors/blackbox/README.md (if exists)

  Update hardware requirements to show NEO-6M/7M/8M compatibility.

  ---
  Phase 4: Update Driver Documentation

  4.1 drivers/ublox-neo/README.md (Full Rewrite)

  # u-blox NEO GPS Driver

  NMEA parser and UBX configuration for u-blox NEO-series GPS modules.

  ## Supported Modules

  | Module | Max Rate | GNSS Support |
  |--------|----------|--------------|
  | NEO-6M | 5 Hz | GPS |
  | NEO-7M | 10 Hz | GPS + GLONASS |
  | NEO-8M | 10 Hz | GPS + GLONASS + Galileo + BeiDou |

  ## Features

  - NMEA 0183 parsing (GPRMC, GNRMC sentences)
  - UBX protocol for rate configuration
  - Zero-allocation parsing (no_std compatible)
  - Automatic warmup with reference point averaging
  - Position-based speed calculation
  - Local coordinate conversion (lat/lon → meters)

  ## Usage

  ```rust
  use ublox_neo::{NmeaParser, ubx};

  // Create parser with 10-fix warmup
  let mut parser = NmeaParser::with_warmup_fixes(10);

  // Configure GPS to 10 Hz (send via UART)
  let cmd = ubx::cfg_rate_command(100); // 100ms = 10 Hz
  uart.write(&cmd)?;

  // Parse incoming NMEA bytes
  loop {
      if parser.feed_byte(byte) {
          // Complete sentence parsed
          let fix = parser.last_fix();
          if fix.valid {
              println!("Position: {}, {}", fix.lat, fix.lon);
          }
      }
  }

  Update Rate Configuration

  The module defaults to 1 Hz. Use UBX CFG-RATE to configure:

  | Rate  | Period | Command                    |
  |-------|--------|----------------------------|
  | 5 Hz  | 200ms  | ubx::cfg_rate_command(200) |
  | 10 Hz | 100ms  | ubx::cfg_rate_command(100) |


  ---

  ## Phase 5: Testing Checklist

  - [ ] Rename compiles: `cargo check` in workspace
  - [ ] Framework tests pass: `cargo test -p sensor-fusion`
  - [ ] UBX checksum verified against known-good values
  - [ ] NEO-6M: Verify 5 Hz config works
  - [ ] NEO-7M: Verify 10 Hz config works
  - [ ] NEO-8M: Verify 10 Hz config works
  - [ ] Warmup completes in ~1 second at 10 Hz
  - [ ] EKF receives GPS updates at configured rate
  - [ ] Documentation grep for "neo6m" returns 0 results
  - [ ] Documentation grep for "50Hz IMU" returns 0 results

  ---

  ## Files Changed Summary

  | Category | Files |
  |----------|-------|
  | **Renamed** | `drivers/neo6m/` → `drivers/ublox-neo/` |
  | **Modified (code)** | `Cargo.toml` (root), `sensors/blackbox/Cargo.toml`, `sensors/blackbox/src/config.rs`, `sensors/blackbox/src/gps.rs`, `sensors/blackbox/src/system.rs`, `sensors/blackbox/src/main.rs`, `drivers/ublox-neo/src/lib.rs` |
  | **Modified (docs)** | `CLAUDE.md`, `README.md`, `drivers/ublox-neo/README.md`, `framework/README.md`, `sensors/README.md` |

  ---

  ## Follow-up Prompt

  Copy and paste this in your next conversation:

  ---

  I want to implement NEO-6M/7M/8M GPS support with UBX rate configuration. Here's the plan:

  Summary

  1. Rename drivers/neo6m → drivers/ublox-neo
  2. Add GpsModel enum (Neo6M/Neo7M/Neo8M) to config
  3. Add UBX CFG-RATE command generation to driver
  4. Send UBX command on boot to configure GPS rate
  5. Update warmup_fixes default from 5 to 10
  6. Fix documentation inconsistencies

  Key Changes

  Config (sensors/blackbox/src/config.rs)

  - Add GpsModel enum with max_rate_hz() and measurement_period_ms() methods
  - Update GpsConfig to include model: GpsModel field
  - Default to Neo7M with 10 Hz rate and 10 warmup fixes

  Driver (drivers/ublox-neo/src/lib.rs)

  - Add pub mod ubx with cfg_rate_command(period_ms: u16) -> [u8; 14]
  - Include Fletcher checksum calculation

  Main (sensors/blackbox/src/main.rs)

  - After GPS UART init, send UBX rate command based on config

  Renames

  - Directory: drivers/neo6m/ → drivers/ublox-neo/
  - Package: neo6m → ublox-neo
  - Imports: neo6m:: → ublox_neo::

  Documentation Fixes

  - CLAUDE.md: Fix "IMU 50Hz" → "IMU 200Hz", fix telemetry rate to 33ms/30Hz
  - README.md: Fix "IMU 50Hz" → "IMU 200Hz"
  - Update all "NEO-6M" references to "u-blox NEO GPS (6M/7M/8M)"

  Please implement this plan. Start with the crate rename, then add UBX support, then fix documentation.

  ---

  This plan is ready to execute. The prompt at the end is what you give me in a follow-up conversation to begin implementation.

