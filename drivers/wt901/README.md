# WT901 9-Axis IMU Driver

Pure Rust driver for the WT901 9-axis IMU sensor (accelerometer + gyroscope + magnetometer).

## Features

- ✅ Zero-copy UART packet parsing
- ✅ Automatic checksum verification
- ✅ Bias calibration support
- ✅ `no_std` compatible
- ✅ No external dependencies (except optional `log`)
- ✅ Well-tested

## Hardware

This driver supports the WT901 IMU connected via UART:
- **Baud rate:** 9600 (default)
- **Voltage:** 3.3V - 5V
- **Packet rate:** Up to 200 Hz

## Usage

Add to your `Cargo.toml`:

```toml
[dependencies]
wt901 = { path = "path/to/drivers/wt901" }
```

### Basic Example

```rust
use wt901::{Wt901Parser, PacketType};

let mut parser = Wt901Parser::new();

// Read bytes from UART and feed to parser
loop {
    let byte = uart.read_byte();
    let timestamp_us = get_timestamp_microseconds();

    if let Some(packet_type) = parser.feed_byte(byte, timestamp_us) {
        match packet_type {
            PacketType::Accel => {
                let (ax, ay, az) = parser.get_accel_corrected();
                println!("Accel: {:.2}, {:.2}, {:.2} m/s²", ax, ay, az);
            }
            PacketType::Gyro => {
                let data = parser.data();
                println!("Gyro: {:.2}, {:.2}, {:.2} rad/s",
                         data.wx, data.wy, data.wz);
            }
            PacketType::Angle => {
                let data = parser.data();
                println!("Roll: {:.1}°, Pitch: {:.1}°, Yaw: {:.1}°",
                         data.roll, data.pitch, data.yaw);
            }
        }
    }
}
```

### With Calibration

```rust
use wt901::{Wt901Parser, ImuCalibrator};

let mut parser = Wt901Parser::new();
let mut calibrator = ImuCalibrator::new(500); // 500 samples

// Calibration phase (IMU must be stationary and level)
while !calibrator.is_complete() {
    let byte = uart.read_byte();

    if let Some(PacketType::Accel) = parser.feed_byte(byte, timestamp()) {
        let data = parser.data();
        calibrator.add_sample(data.ax, data.ay, data.az);

        println!("Calibration: {:.0}%", calibrator.progress() * 100.0);
    }
}

// Apply calibration
let bias = calibrator.compute_bias().unwrap();
parser.set_bias(bias);

// Now use calibrated readings
loop {
    // ... normal operation with bias-corrected data
}
```

## Features

### `logging` (optional)

Enable logging for checksum errors:

```toml
[dependencies]
wt901 = { path = "...", features = ["logging"] }
```

## Protocol Details

The WT901 sends 11-byte packets via UART:

```
[0x55] [TYPE] [DATA(8 bytes)] [CHECKSUM]
```

**Packet Types:**
- `0x51` - Accelerometer data
- `0x52` - Gyroscope data
- `0x53` - Euler angles (roll, pitch, yaw)

**Data Ranges:**
- Accelerometer: ±16g
- Gyroscope: ±2000°/s
- Angles: ±180°

## No-Std Support

This driver works in `no_std` environments:

```toml
[dependencies]
wt901 = { path = "...", default-features = false }
```

Note: Calibration requires `alloc` feature (Vec usage).

## License

MIT OR Apache-2.0
