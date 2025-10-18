# NEO-6M GPS NMEA Parser

Pure Rust parser for NMEA sentences from NEO-6M (and compatible) GPS receivers.

## Features

- ✅ Zero-allocation NMEA parsing
- ✅ Supports GPRMC/GNRMC sentences
- ✅ Reference point averaging (warmup)
- ✅ Local coordinate conversion (lat/lon → meters)
- ✅ Position-based speed calculation
- ✅ `no_std` compatible
- ✅ No external dependencies

## Hardware

This driver works with any GPS receiver that outputs NMEA sentences:
- **NEO-6M** - Popular low-cost GPS module
- **NEO-7M, NEO-8M** - Higher precision variants
- Any NMEA 0183 compatible GPS
- **Baud rate:** Typically 9600 (configurable on module)

## Usage

Add to your `Cargo.toml`:

```toml
[dependencies]
neo6m = { path = "path/to/drivers/neo6m" }
```

### Basic Example

```rust
use neo6m::NmeaParser;

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

### With Local Coordinates

```rust
use neo6m::NmeaParser;

let mut parser = NmeaParser::new();

// Feed data until warmup complete (averages first 5 fixes)
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
use neo6m::NmeaParser;

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

## NMEA Sentences

This parser currently supports:

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
neo6m = { path = "...", default-features = false }
```

Note: Requires `alloc` feature for NMEA sentence parsing (Vec usage).

## License

MIT OR Apache-2.0
