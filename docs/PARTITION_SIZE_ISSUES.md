# Flash Partition Size Issues

This document tracks the flash partition size constraints and failed attempts to increase capacity.

## Current State

**Date:** 2026-01-26

| Metric | Value |
|--------|-------|
| Flash chip size | 4MB |
| App partition size | 1.5MB (1,536,000 bytes) |
| Current binary size | 1,499,312 bytes |
| Usage | 97.61% |
| Headroom | ~37KB (2.4%) |

## Problem

The ESP32-C3 firmware is approaching the flash partition limit. At 97.6% capacity, there's only ~37KB remaining for new features. This will become a blocker as the codebase grows.

## Root Cause

The project uses ESP-IDF's built-in `partitions_singleapp_large.csv` which allocates only 1.5MB for the application partition, despite having 4MB of flash available.

```csv
# partitions_singleapp_large.csv (ESP-IDF built-in)
nvs,      data, nvs,     ,        0x6000,
phy_init, data, phy,     ,        0x1000,
factory,  app,  factory, ,        1500K,
```

With 4MB flash, we could theoretically use up to ~3.5MB for the app partition (leaving room for NVS, PHY, bootloader).

## Failed Attempts to Increase Partition Size

### Attempt 1: Custom partitions.csv

**What I tried:**
1. Created `sensors/blackbox/partitions.csv` with 2.5MB app partition:
```csv
# Name,   Type, SubType, Offset,  Size,   Flags
nvs,      data, nvs,     0x9000,  0x6000,
phy_init, data, phy,     0xf000,  0x1000,
factory,  app,  factory, 0x10000, 0x280000,
```

2. Modified `sdkconfig.defaults`:
```
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
```

**Why it failed:**
ESP-IDF build system couldn't find the partition file. Error:
```
FileNotFoundError: No such file or directory:
'/home/.../target/.../out/partitions.csv'
```

The build system expects `partitions.csv` in the CMake build output directory, not the source directory. The file path in `CONFIG_PARTITION_TABLE_CUSTOM_FILENAME` is interpreted relative to the ESP-IDF build directory, not the Rust project directory.

### Attempt 2: ESP_IDF_SDKCONFIG_DEFAULTS environment variable

**What I tried:**
Added to `.cargo/config.toml`:
```toml
ESP_IDF_SDKCONFIG_DEFAULTS = { value = "sdkconfig.defaults", relative = true }
```

**Why it failed:**
The `sdkconfig.defaults` was being read, but the custom partition table setting was still not being applied. The generated `sdkconfig` in the build output showed:
```
CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE=y
# CONFIG_PARTITION_TABLE_CUSTOM is not set
```

This suggests the defaults were being overridden somewhere in the ESP-IDF build process, or the Kconfig comment syntax (`# CONFIG_... is not set`) wasn't being parsed correctly.

### Attempt 3: Clearing build cache

**What I tried:**
1. Deleted `target/riscv32imc-esp-espidf/release/build/esp-idf-sys-*/`
2. Deleted the generated `sdkconfig` file
3. Rebuilt from scratch

**Why it failed:**
Even after clearing the cache and rebuilding, the partition table settings reverted to the default `partitions_singleapp_large.csv`. The ESP-IDF build system caches configuration aggressively.

## What Would Actually Work

Based on the failed attempts, here are approaches that should work but weren't fully implemented:

### Option 1: Modify build.rs to copy partition file

Add to `sensors/blackbox/build.rs`:
```rust
use std::fs;
use std::path::Path;

fn main() {
    // Copy partitions.csv to where ESP-IDF expects it
    let out_dir = std::env::var("OUT_DIR").unwrap();
    let src = Path::new("partitions.csv");
    let dst = Path::new(&out_dir).join("partitions.csv");
    if src.exists() {
        fs::copy(src, dst).expect("Failed to copy partitions.csv");
    }

    embuild::espidf::sysenv::output();
    println!("cargo:rustc-check-cfg=cfg(esp_idf_httpd_ws_support)");
}
```

### Option 2: Use ESP_IDF_COMPONENT_CONFDIR

Set environment variable pointing to a directory containing the custom partition table and other configuration files.

### Option 3: CMakeLists.txt approach

Create a `CMakeLists.txt` in the project that explicitly sets the partition table path before the ESP-IDF build runs.

### Option 4: Symlink in build directory

Post-build script that creates a symlink from the ESP-IDF build output directory to the source partition file.

## Workarounds Until Fixed

1. **Optimize binary size:**
   - Enable LTO (Link-Time Optimization) in Cargo.toml
   - Review and remove unused dependencies
   - Compress the HTML dashboard (currently embedded as string)

2. **Monitor usage:**
   - Track binary size in CI
   - Alert when approaching 95% capacity

3. **Feature flags:**
   - Make non-essential features optional via Cargo features
   - Build variants for different use cases

## Impact on Development

- New features are constrained to ~37KB total
- Major features (e.g., SD card logging, BLE support) may not fit
- Each code change should verify binary size doesn't exceed limit

## Related Configuration

Current `sdkconfig.defaults`:
```
CONFIG_IDF_TARGET="esp32c3"
CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE=y
CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y
CONFIG_ESPTOOLPY_FLASHSIZE="4MB"
CONFIG_COMPILER_OPTIMIZATION_SIZE=y
CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_DISABLE=y
```

## References

- ESP-IDF Partition Tables: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html
- esp-idf-sys build configuration: https://github.com/esp-rs/esp-idf-sys
- embuild documentation: https://github.com/esp-rs/embuild
