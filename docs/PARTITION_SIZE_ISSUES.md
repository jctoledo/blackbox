# Flash Partition Configuration

This document describes the custom flash partition configuration for the ESP32-C3 blackbox firmware.

## Current Configuration

| Metric | Value |
|--------|-------|
| Flash chip size | 4MB |
| App partition size | **2.5 MB** (2,621,440 bytes) |
| Current binary size | ~1.5 MB |
| Utilization | ~57% |
| Headroom | ~1.07 MB |

This is a significant improvement over the default 1.5 MB partition which had only 36 KB headroom (97.6% utilization).

## Setup After Cloning

**Important:** After cloning this repo, run the setup script to configure the partition table path:

```bash
./scripts/setup-partition.sh
```

This updates `sdkconfig.defaults` with the correct absolute path for your machine.

### Why is this needed?

The ESP-IDF build system (via embuild/esp-idf-sys) requires an absolute path to the partition table file because:
1. The build runs from `target/.../esp-idf-sys-*/out/`, not the source directory
2. Relative paths are resolved from that build directory, not the repo root
3. There's no way to dynamically compute the path during build (env vars set in build.rs don't propagate to esp-idf-sys)

## Files

- `partitions.csv` - Custom partition table definition (workspace root)
- `sdkconfig.defaults` - ESP-IDF configuration with partition table path (workspace root)
- `scripts/setup-partition.sh` - Setup script for portability

## Partition Layout

```
Address Range          Size     Name        Description
─────────────────────────────────────────────────────────
0x000000 - 0x007FFF    32 KB    Bootloader  ESP-IDF bootloader
0x008000 - 0x008FFF     4 KB    Part.Table  Partition table
0x009000 - 0x00EFFF    24 KB    nvs         Non-volatile storage
0x00F000 - 0x00FFFF     4 KB    phy_init    PHY calibration data
0x010000 - 0x28FFFF   2.5 MB    factory     Application
0x290000 - 0x3FFFFF   1.4 MB    (unused)    Available for future use
```

## Manual Configuration

If the setup script doesn't work, manually edit `sdkconfig.defaults`:

```bash
# Find your repo's absolute path
REPO_PATH=$(pwd)

# Edit sdkconfig.defaults and update this line:
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="/your/absolute/path/to/blackbox/partitions.csv"
```

Then clean and rebuild:
```bash
rm -rf target/riscv32imc-esp-espidf/release/build/esp-idf-sys-*
cargo build --release
```

## Verifying the Configuration

After building, verify the partition table was applied:

```bash
python3 -c "
import struct
with open('target/riscv32imc-esp-espidf/release/partition-table.bin', 'rb') as f:
    data = f.read()
offset = 0
while offset + 32 <= len(data):
    entry = data[offset:offset+32]
    magic = struct.unpack('<H', entry[0:2])[0]
    if magic != 0x50AA: break
    size = struct.unpack('<I', entry[8:12])[0]
    name = entry[12:28].split(b'\x00')[0].decode()
    print(f'{name:12} {size/1024/1024:.2f} MB')
    offset += 32
"
```

Expected output:
```
nvs          0.02 MB
phy_init     0.00 MB
factory      2.50 MB    ← Should be 2.50 MB, not 1.00 or 1.50 MB
```

## Troubleshooting

### Partition still shows 1.0 or 1.5 MB

1. Make sure you ran `./scripts/setup-partition.sh`
2. Clean the ESP-IDF build cache: `rm -rf target/riscv32imc-esp-espidf/release/build/esp-idf-sys-*`
3. Rebuild: `cargo build --release`
4. Check that `sdkconfig.defaults` has the correct absolute path

### "partitions.csv not found" error

The path in `sdkconfig.defaults` is wrong. Run the setup script or manually update the path.
