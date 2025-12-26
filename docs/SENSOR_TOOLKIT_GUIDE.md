## Sensor Toolkit Guide

# Adding New Sensors to Blackbox

Blackbox is designed as a **modular sensor fusion platform**. Adding a new sensor is as simple as implementing a trait and registering it. This guide shows you how.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    Distributed Sensor Network                    │
└─────────────────────────────────────────────────────────────────┘

┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│   ESP32 #1   │  │   ESP32 #2   │  │   ESP32 #3   │  │   ESP32 #4   │
│              │  │              │  │              │  │              │
│  WT901 IMU   │  │  NEO-6M GPS  │  │  Wheel Speed │  │   CAN Bus    │
│              │  │              │  │              │  │              │
└──────┬───────┘  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘
       │                 │                 │                 │
       │ MQTT            │ MQTT            │ MQTT            │ MQTT
       │ topic:          │ topic:          │ topic:          │ topic:
       │ imu/wt901       │ gps/neo6m       │ wheels/abs      │ can/obd2
       │                 │                 │                 │
       └─────────────────┴─────────────────┴─────────────────┘
                                 │
                                 ▼
                    ┌─────────────────────────┐
                    │  Fusion Coordinator     │
                    │  (subscribes to all)    │
                    │                         │
                    │  Multi-Sensor EKF       │
                    └────────────┬────────────┘
                                 │
                                 ▼
                         ┌───────────────┐
                         │ Fused State   │
                         │ Published to: │
                         │ car/state     │
                         └───────────────┘
```

---

## Quick Start: Add a New Sensor in 5 Minutes

### Step 1: Define Your Sensor Struct

```rust
// src/sensor_plugins.rs

use crate::sensor_framework::*;

pub struct MySensor {
    // Your sensor-specific fields
    last_reading_us: u64,
    some_calibration_value: f32,
}

impl MySensor {
    pub fn new() -> Self {
        Self {
            last_reading_us: 0,
            some_calibration_value: 0.0,
        }
    }
}
```

### Step 2: Implement the `Sensor` Trait

```rust
impl Sensor for MySensor {
    fn capabilities(&self) -> SensorCapabilities {
        SensorCapabilities {
            id: SensorId(42),  // Pick a unique ID
            name: "My Cool Sensor",
            measurement_types: &[MeasurementType::Custom("my_measurement")],
            update_rate_hz: 100.0,
            latency_ms: 10.0,
            accuracy: 0.5,
            mqtt_topic: "car/sensors/mysensor",
        }
    }

    fn poll(&mut self) -> Option<SensorReading> {
        // Read from your sensor hardware
        // Return None if no new data available

        let now_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };

        // Example: Return a reading
        Some(SensorReading {
            sensor_id: SensorId(42),
            timestamp_us: now_us,
            data: SensorData::Custom(CustomReading {
                data: vec![1, 2, 3, 4],  // Your sensor data
            }),
        })
    }

    fn init(&mut self) -> Result<(), SensorError> {
        // Initialize your sensor hardware
        // Setup GPIO, I2C, SPI, UART, etc.

        log::info!("MySensor initialized!");
        Ok(())
    }

    fn calibrate(&mut self) -> Result<(), SensorError> {
        // Optional: Calibration routine
        // Can return Ok(()) if no calibration needed

        log::info!("MySensor calibrated!");
        Ok(())
    }

    fn is_healthy(&self) -> bool {
        // Optional: Health check
        // Return false if sensor is malfunctioning
        true
    }
}
```

### Step 3: Register Your Sensor

```rust
// In main.rs

let mut registry = SensorRegistry::new();

// Register your sensor
let my_sensor = Box::new(MySensor::new());
registry.register(my_sensor).expect("Failed to register sensor");

// Poll all sensors
loop {
    let readings = registry.poll_all();
    for reading in readings {
        publisher.publish(&reading).ok();
    }
}
```

**That's it!** Your sensor is now integrated into the system.

---

## Real-World Examples

### Example 1: Adding Wheel Speed Sensors

**Hardware:** 4x Hall effect sensors on wheel hubs

```rust
pub struct WheelSpeedSensor {
    fl_count: u32,  // Front-left pulse count
    fr_count: u32,
    rl_count: u32,
    rr_count: u32,
    last_time_us: u64,
    pulses_per_rev: u32,  // Depends on your sensor
    wheel_circumference: f32,  // meters
}

impl WheelSpeedSensor {
    pub fn new() -> Self {
        Self {
            fl_count: 0,
            fr_count: 0,
            rl_count: 0,
            rr_count: 0,
            last_time_us: 0,
            pulses_per_rev: 48,  // Example: 48 pulses per wheel rotation
            wheel_circumference: 1.884,  // Example: 60cm diameter wheel
        }
    }

    // Called from GPIO interrupt handler
    pub fn on_fl_pulse(&mut self) { self.fl_count += 1; }
    pub fn on_fr_pulse(&mut self) { self.fr_count += 1; }
    pub fn on_rl_pulse(&mut self) { self.rl_count += 1; }
    pub fn on_rr_pulse(&mut self) { self.rr_count += 1; }
}

impl Sensor for WheelSpeedSensor {
    fn capabilities(&self) -> SensorCapabilities {
        SensorCapabilities {
            id: SensorId(11),
            name: "ABS Wheel Speed",
            measurement_types: &[MeasurementType::WheelSpeed, MeasurementType::Velocity3D],
            update_rate_hz: 100.0,
            latency_ms: 5.0,
            accuracy: 0.05,  // ±5cm/s
            mqtt_topic: "car/sensors/wheels",
        }
    }

    fn poll(&mut self) -> Option<SensorReading> {
        let now_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };
        let dt = (now_us - self.last_time_us) as f32 * 1e-6;

        if dt < 0.01 {  // Poll at 100Hz max
            return None;
        }

        // Convert pulse counts to speeds
        let fl_speed = (self.fl_count as f32 / self.pulses_per_rev as f32)
                       * self.wheel_circumference / dt;

        // Reset counts
        self.fl_count = 0;
        // ... same for fr, rl, rr

        self.last_time_us = now_us;

        Some(SensorReading {
            sensor_id: SensorId(11),
            timestamp_us: now_us,
            data: SensorData::WheelSpeed(WheelSpeedReading {
                fl: fl_speed,
                fr: 0.0,  // Calculate similarly
                rl: 0.0,
                rr: 0.0,
            }),
        })
    }

    fn init(&mut self) -> Result<(), SensorError> {
        // Setup GPIO interrupts for wheel speed sensors
        // (ESP-IDF specific code omitted for brevity)
        Ok(())
    }
}
```

### Example 2: Adding CAN Bus (OBD2)

**Hardware:** ESP32 with MCP2515 CAN controller

```rust
use esp_idf_hal::can::*;

pub struct CanBusSensor {
    can: CanDriver,
    rpm: u16,
    throttle: u8,
    speed: u8,
}

impl Sensor for CanBusSensor {
    fn capabilities(&self) -> SensorCapabilities {
        SensorCapabilities {
            id: SensorId(10),
            name: "CAN Bus OBD2",
            measurement_types: &[
                MeasurementType::Custom("RPM"),
                MeasurementType::Throttle,
                MeasurementType::Velocity3D,
            ],
            update_rate_hz: 50.0,
            latency_ms: 20.0,
            accuracy: 1.0,
            mqtt_topic: "car/sensors/can",
        }
    }

    fn poll(&mut self) -> Option<SensorReading> {
        // Read CAN messages
        if let Ok(frame) = self.can.receive() {
            match frame.id() {
                0x0C => {  // RPM (PID 0x0C)
                    self.rpm = u16::from_be_bytes([frame.data()[0], frame.data()[1]]);
                }
                0x11 => {  // Throttle (PID 0x11)
                    self.throttle = frame.data()[0];
                }
                0x0D => {  // Vehicle speed (PID 0x0D)
                    self.speed = frame.data()[0];
                }
                _ => {}
            }
        }

        Some(SensorReading {
            sensor_id: SensorId(10),
            timestamp_us: unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 },
            data: SensorData::Can(CanReading {
                can_id: 0x123,
                data: vec![self.throttle, self.speed],
            }),
        })
    }

    fn init(&mut self) -> Result<(), SensorError> {
        // Initialize CAN bus at 500kbps
        // (Implementation details omitted)
        Ok(())
    }
}
```

### Example 3: Adding Lidar for Lane Detection

**Hardware:** Cheap 2D Lidar (e.g., RPLidar A1)

```rust
pub struct LidarSensor {
    // Serial port for lidar
    last_scan: Vec<[f32; 3]>,
}

impl Sensor for LidarSensor {
    fn capabilities(&self) -> SensorCapabilities {
        SensorCapabilities {
            id: SensorId(20),
            name: "RPLidar A1",
            measurement_types: &[MeasurementType::LaneDetection],
            update_rate_hz: 10.0,  // 10Hz scan rate
            latency_ms: 100.0,
            accuracy: 0.1,  // ±10cm
            mqtt_topic: "car/sensors/lidar",
        }
    }

    fn poll(&mut self) -> Option<SensorReading> {
        // Read lidar scan
        // Process points to detect lane markings
        // Return processed data

        Some(SensorReading {
            sensor_id: SensorId(20),
            timestamp_us: unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 },
            data: SensorData::Lidar(LidarReading {
                points: self.last_scan.clone(),
            }),
        })
    }

    fn init(&mut self) -> Result<(), SensorError> {
        // Start lidar motor
        // Begin scanning
        Ok(())
    }
}
```

---

## Sensor Fusion Integration

Once your sensor is publishing data, integrate it into the fusion coordinator:

### Option 1: Local Fusion (Single ESP32)

```rust
// In fusion_coordinator.rs

impl FusionCoordinator {
    pub fn process_reading(&mut self, reading: SensorReading) {
        match reading.data {
            SensorData::WheelSpeed(wheels) => {
                self.process_wheel_speed(wheels, reading.timestamp_us);
            }
            SensorData::Can(can) => {
                self.process_can(can, reading.timestamp_us);
            }
            // Add your sensor here!
            SensorData::Custom(custom) => {
                // Process your custom sensor data
                // Update EKF or other estimators
            }
            _ => {}
        }
    }
}
```

### Option 2: Distributed Fusion (Multiple ESP32s)

Each sensor runs on its own ESP32 and publishes to MQTT:

```rust
// On Sensor ESP32
let mut sensor = MySensor::new();
let mut mqtt = MqttClient::new("mqtt://broker:1883").unwrap();

loop {
    if let Some(reading) = sensor.poll() {
        let json = serde_json::to_string(&reading).unwrap();
        mqtt.publish("car/sensors/mysensor", &json).unwrap();
    }
}
```

```rust
// On Fusion ESP32
let mut coordinator = FusionCoordinator::new();
let mut mqtt = MqttClient::new("mqtt://broker:1883").unwrap();

mqtt.subscribe("car/sensors/#").unwrap();  // Subscribe to all sensors

mqtt.on_message(|topic, payload| {
    let reading: SensorReading = serde_json::from_slice(payload).unwrap();
    coordinator.process_reading(reading);
});
```

---

## Best Practices

### 1. **Sensor IDs**
- Pick unique IDs (1-1000)
- Reserve ranges: IMU (1-9), GPS (10-19), Wheels (20-29), CAN (30-39), etc.

### 2. **Timestamps**
- Always use microsecond timestamps
- Use system time, not sensor time (for synchronization)

### 3. **MQTT Topics**
- Format: `car/sensors/<type>/<model>`
- Examples: `car/sensors/imu/wt901`, `car/sensors/gps/neo6m`

### 4. **Error Handling**
- Sensors should gracefully degrade
- Return `None` from `poll()` if no data available
- Return errors from `init()` if hardware fails

### 5. **Performance**
- Keep `poll()` non-blocking
- Use interrupts for high-frequency sensors
- Buffer data if necessary

---

## Debugging New Sensors

### 1. Test Sensor in Isolation

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_my_sensor() {
        let mut sensor = MySensor::new();
        sensor.init().unwrap();

        // Simulate hardware input
        // Check output
        let reading = sensor.poll();
        assert!(reading.is_some());
    }
}
```

### 2. Enable Logging

```rust
impl Sensor for MySensor {
    fn poll(&mut self) -> Option<SensorReading> {
        log::debug!("MySensor polled");
        // ... your code
        let reading = /* ... */;
        log::info!("MySensor reading: {:?}", reading);
        Some(reading)
    }
}
```

### 3. Use the LogPublisher

```rust
let mut sensor = MySensor::new();
let mut publisher = LogPublisher;

sensor.init().unwrap();

loop {
    if let Some(reading) = sensor.poll() {
        publisher.publish(&reading).unwrap();
    }
}
```

Output:
```
[SENSOR 42] Custom(CustomReading { data: [1, 2, 3, 4] })
```

---

## FAQ

**Q: Can I add a sensor that's not on an ESP32?**
A: Yes! Use MQTT. Any device that can publish MQTT messages can be a sensor (Raspberry Pi, Arduino, phone, etc.)

**Q: How do I handle sensors with different update rates?**
A: The fusion coordinator automatically handles this. Fast sensors (200Hz IMU) and slow sensors (5Hz GPS) work together seamlessly.

**Q: What if my sensor needs calibration?**
A: Implement the `calibrate()` method. Call it from main before starting the main loop.

**Q: Can I add vision sensors (cameras)?**
A: Yes, but you'll need a more powerful processor (ESP32-S3 or Raspberry Pi) for image processing. Publish only processed results (lane detection, object detection) not raw images.

**Q: How do I test sensor fusion without hardware?**
A: Create a `SimulatedSensor` that plays back recorded data or generates synthetic data.

---

## Advanced: Custom Measurement Types

If the built-in `MeasurementType` enum doesn't cover your sensor:

```rust
// In sensor_framework.rs

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MeasurementType {
    // ... existing types ...

    // Add your custom type
    TirePressure,
    SuspensionTravel,
    FuelLevel,

    Custom(&'static str),  // For truly custom measurements
}
```

Then use it in your sensor:

```rust
impl Sensor for TirePressureSensor {
    fn capabilities(&self) -> SensorCapabilities {
        SensorCapabilities {
            id: SensorId(100),
            name: "TPMS",
            measurement_types: &[MeasurementType::TirePressure],
            // ...
        }
    }
}
```

---

## Example: Complete Sensor Plugin

See `src/sensor_plugins.rs` for complete examples:
- `Wt901ImuSensor` - IMU implementation
- `Neo6mGpsSensor` - GPS implementation
- `WheelSpeedSensor` - Wheel speed template
- `CanBusSensor` - CAN bus template

---

## Next Steps

1. Implement your sensor using the `Sensor` trait
2. Test it in isolation with `LogPublisher`
3. Register it with `SensorRegistry`
4. Add fusion logic in `FusionCoordinator::process_reading()`
5. Deploy and publish to MQTT
6. Monitor fused output on `car/fusion/state`

Happy sensing! 🚗💨
