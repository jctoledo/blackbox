# u-blox GPS Driver

Pure Rust driver for u-blox GPS receivers with NMEA parsing and UBX protocol support.

## Supported Hardware

| Module | Max Rate | Protocol | Features |
|--------|----------|----------|----------|
| NEO-6M | 5 Hz | NMEA only | Budget option, basic accuracy |
| NEO-7M | 10 Hz | NMEA + UBX | Improved accuracy |
| NEO-M8N | 10 Hz | NMEA + UBX | Better sensitivity |
| **NEO-M9N** | **25 Hz** | NMEA + UBX | Automotive mode, best accuracy |

Any NMEA 0183 compatible GPS receiver will work for basic functionality.

## Features

- Zero-allocation NMEA parsing (no_std compatible)
- Supports GPRMC/GNRMC (position/velocity), GGA (satellites), GSA (DOP)
- UBX protocol commands for GPS configuration (NEO-M9N)
- Reference point averaging (warmup)
- Local coordinate conversion (lat/lon to meters)
- Position-based speed calculation
- Dynamic platform models (Automotive, Pedestrian, etc.)

## Usage

Add to your `Cargo.toml`:

```toml
[dependencies]
ublox-gps = { path = "path/to/drivers/ublox-gps" }
```

### Basic NMEA Parsing

```rust
use ublox_gps::NmeaParser;

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
            println!("Satellites: {}", fix.satellites);
            println!("HDOP: {:.1}", fix.hdop);
        }
    }
}
```

### With Local Coordinates

```rust
use ublox_gps::NmeaParser;

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

### Configuring NEO-M9N (UBX Protocol)

```rust
use ublox_gps::ubx::{UbxCommands, DynamicModel, generate_init_sequence};

// Generate initialization sequence for 10Hz automotive mode
let mut buffer = [0u8; 128];
let commands = generate_init_sequence(10, 115200, &mut buffer);

// Send baud rate command at current GPS baud (factory: 38400)
let (baud_off, baud_len) = commands[0];
uart.write(&buffer[baud_off..baud_off + baud_len]);
delay_ms(100);

// Switch UART to new baud rate
uart.set_baudrate(115200);
delay_ms(100);

// Send rate + dynamic model command
let (config_off, config_len) = commands[1];
uart.write(&buffer[config_off..config_off + config_len]);
```

### Individual UBX Commands

```rust
use ublox_gps::ubx::{UbxCommands, DynamicModel};

let mut buffer = [0u8; 32];

// Set measurement rate to 25 Hz
let len = UbxCommands::cfg_valset_rate(25, &mut buffer);
uart.write(&buffer[..len]);

// Set automotive dynamic model
let len = UbxCommands::cfg_valset_dynmodel(DynamicModel::Automotive, &mut buffer);
uart.write(&buffer[..len]);

// Set baud rate to 115200
let len = UbxCommands::cfg_valset_baudrate(115200, &mut buffer);
uart.write(&buffer[..len]);
```

## Dynamic Platform Models

The NEO-M9N supports multiple dynamic models optimized for different use cases:

| Model | Use Case | Max Altitude | Max Velocity |
|-------|----------|--------------|--------------|
| Portable | General handheld | 12km | 310 m/s |
| Stationary | Fixed installation | 9km | 10 m/s |
| Pedestrian | Walking/running | 9km | 30 m/s |
| **Automotive** | Vehicle telemetry | 9km | 100 m/s |
| Sea | Marine | 9km | 25 m/s |
| Airborne1g | Light aircraft | 50km | 100 m/s |
| Airborne2g | Aerobatic | 50km | 250 m/s |
| Airborne4g | High-g maneuvers | 50km | 500 m/s |

**Automotive mode** is recommended for vehicle telemetry - it's optimized for road dynamics and provides the best position/velocity accuracy for cars.

## NMEA Sentences

### GPRMC/GNRMC - Recommended Minimum
```
$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
```

Fields: Time, Status (A=valid), Latitude, Longitude, Speed (knots), Course, Date

### GPGGA/GNGGA - Fix Data
```
$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*47
```

Fields: Time, Lat, Lon, Fix Quality, Satellites, HDOP, Altitude

### GPGSA/GNGSA - DOP and Active Satellites
```
$GPGSA,A,3,04,05,06,07,08,09,10,11,12,13,14,15,1.4,0.9,1.1*3D
```

Fields: Mode, Fix Type, Satellite PRNs, PDOP, HDOP, VDOP

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
north_m = (lat - ref_lat) * 111,320
east_m = (lon - ref_lon) * 111,320 * cos(ref_lat)
```

## GpsFix Structure

```rust
pub struct GpsFix {
    pub lat: f64,           // Latitude (degrees, + = North)
    pub lon: f64,           // Longitude (degrees, + = East)
    pub speed: f32,         // Ground speed (m/s)
    pub course: f32,        // Course over ground (radians)
    pub valid: bool,        // Fix validity
    pub hour: u8,           // UTC hour
    pub minute: u8,         // UTC minute
    pub second: u8,         // UTC second
    pub altitude: f32,      // Altitude MSL (meters)
    pub satellites: u8,     // Satellites in use
    pub fix_quality: FixQuality,  // Fix type
    pub hdop: f32,          // Horizontal DOP
    pub pdop: f32,          // Position DOP
    pub vdop: f32,          // Vertical DOP
}
```

## No-Std Support

This driver works in `no_std` environments:

```toml
[dependencies]
ublox-gps = { path = "...", default-features = false }
```

## License

MIT OR Apache-2.0
