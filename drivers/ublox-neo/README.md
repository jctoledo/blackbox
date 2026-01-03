# u-blox NEO GPS Driver

Pure Rust NMEA parser and UBX configuration for u-blox NEO GPS receivers (NEO-6M/7M/8M).

## Features

- Zero-allocation NMEA parsing (GPRMC/GNRMC sentences)
- UBX protocol support for GPS rate configuration
- Reference point averaging (warmup)
- Local coordinate conversion (lat/lon → meters)
- Position-based speed calculation
- `no_std` compatible
- No external dependencies

## Supported Modules

| Module | Max Update Rate | Notes |
|--------|----------------|-------|
| **NEO-6M** | 5 Hz | Most common, budget-friendly |
| **NEO-7M** | 10 Hz | Better accuracy, recommended |
| **NEO-8M** | 10 Hz (18 Hz*) | Best accuracy, multi-GNSS |

*NEO-8M can achieve 18 Hz in single-GNSS mode, but 10 Hz is recommended for multi-GNSS accuracy.

## Usage

Add to your `Cargo.toml`:

```toml
[dependencies]
ublox-neo = { path = "path/to/drivers/ublox-neo" }
```

### Basic NMEA Parsing

```rust
use ublox_neo::NmeaParser;

let mut parser = NmeaParser::new();

// Read bytes from UART and feed to parser
loop {
    let byte = uart.read_byte();

    if parser.feed_byte(byte) {
        // New sentence parsed
        let fix = parser.last_fix();

        if fix.valid {
            println!("GPS: {:.6}, {:.6}", fix.lat, fix.lon);
            println!("Speed: {:.1} m/s", fix.speed);
            println!("Course: {:.1}°", fix.course.to_degrees());
            println!("Time: {:02}:{:02}:{:02} UTC", fix.hour, fix.minute, fix.second);
        }
    }
}
```

### Configure GPS Update Rate (UBX)

```rust
use ublox_neo::ubx::cfg_rate_command;

// Configure for 10 Hz (100ms measurement period)
let cmd = cfg_rate_command(100);
uart.write(&cmd);

// Configure for 5 Hz (200ms measurement period)
let cmd = cfg_rate_command(200);
uart.write(&cmd);
```

The `cfg_rate_command()` function generates a complete UBX CFG-RATE command with proper Fletcher checksum.

### With Local Coordinates

```rust
use ublox_neo::NmeaParser;

let mut parser = NmeaParser::new();

// Feed data until warmup complete (averages first N fixes)
while !parser.is_warmed_up() {
    if parser.feed_byte(uart.read_byte()) {
        println!("Warmup: {:.0}%", parser.warmup_progress() * 100.0);
    }
}

println!("Reference point: {:.6}, {:.6}",
         parser.reference().lat, parser.reference().lon);

// Now get local coordinates
loop {
    if parser.feed_byte(uart.read_byte()) {
        if let Some((east, north)) = parser.to_local_coords() {
            println!("Position: ({:.2}m E, {:.2}m N)", east, north);
        }
    }
}
```

### Position-Based Speed

```rust
use ublox_neo::NmeaParser;

let mut parser = NmeaParser::new();
let mut last_timestamp_ms = 0;

loop {
    if parser.feed_byte(uart.read_byte()) {
        let now_ms = get_timestamp_ms();

        // Update position-based speed calculation
        parser.update_position_speed(now_ms, last_timestamp_ms);

        println!("GPS speed: {:.1} m/s", parser.last_fix().speed);
        println!("Position-based speed: {:.1} m/s", parser.position_based_speed());

        last_timestamp_ms = now_ms;
    }
}
```

## UBX Protocol

The driver includes UBX protocol support for configuring u-blox GPS modules.

### CFG-RATE Command

Sets the GPS measurement and navigation rate:

```rust
use ublox_neo::ubx::cfg_rate_command;

// Generate command for desired measurement period
let cmd = cfg_rate_command(100);  // 100ms = 10 Hz
// cmd is a [u8; 14] containing the complete UBX message with checksum
```

Command structure (14 bytes):
```
0xB5 0x62     - Sync characters (μb)
0x06 0x08     - CFG-RATE class and message ID
0x06 0x00     - Payload length (6 bytes)
LL HH         - measRate: measurement period in ms (little-endian)
0x01 0x00     - navRate: 1 navigation solution per measurement
0x01 0x00     - timeRef: GPS time reference
CK_A CK_B     - Fletcher checksum
```

## NMEA Sentences

### GPRMC/GNRMC - Recommended Minimum
```
$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
```

Fields:
- Time: 12:35:19 UTC
- Status: A (active/valid)
- Latitude: 48°07.038' N = 48.1173°
- Longitude: 11°31.000' E = 11.5167°
- Speed: 22.4 knots = 11.54 m/s
- Course: 84.4° true
- Date: 23/03/1994

## Coordinate System

The parser provides two coordinate systems:

### 1. Geographic (WGS84)
- Latitude/Longitude in degrees
- Direct from GPS

### 2. Local ENU (East-North-Up)
- Origin at reference point (averaged from first N fixes)
- East/North in meters
- Useful for navigation and vehicle dynamics

Conversion formulas:
```
north_m = (lat - ref_lat) × 111,320
east_m = (lon - ref_lon) × 111,320 × cos(ref_lat)
```

## No-Std Support

This driver works in `no_std` environments:

```toml
[dependencies]
ublox-neo = { path = "...", default-features = false }
```

Note: Requires `alloc` feature for NMEA sentence parsing (Vec usage).

## Migration from neo6m

If upgrading from the old `neo6m` crate:

```toml
# Old
neo6m = { path = "..." }

# New
ublox-neo = { path = "..." }
```

```rust
// Old
use neo6m::NmeaParser;

// New
use ublox_neo::NmeaParser;
```

The API is fully compatible - only the crate name changed.

## License

MIT OR Apache-2.0
