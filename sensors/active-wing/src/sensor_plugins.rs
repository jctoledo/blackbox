/// Concrete sensor implementations using the plugin framework
/// Each sensor is self-contained and publishes to its own MQTT topic

use crate::sensor_framework::*;
use crate::imu::Wt901Parser;
use crate::gps::NmeaParser;

/// WT901 IMU sensor plugin
pub struct Wt901ImuSensor {
    parser: Wt901Parser,
}

impl Wt901ImuSensor {
    pub fn new() -> Self {
        Self {
            parser: Wt901Parser::new(),
        }
    }

    /// Feed UART byte (called from main loop)
    pub fn feed_byte(&mut self, byte: u8, timestamp_us: u64) -> Option<u8> {
        self.parser.feed_byte(byte, timestamp_us)
    }
}

impl Sensor for Wt901ImuSensor {
    fn capabilities(&self) -> SensorCapabilities {
        SensorCapabilities {
            id: SensorId(1),  // IMU = sensor 1
            name: "WT901 IMU",
            measurement_types: &[
                MeasurementType::Acceleration3D,
                MeasurementType::AngularVelocity3D,
                MeasurementType::Orientation,
            ],
            update_rate_hz: 200.0,
            latency_ms: 5.0,
            accuracy: 0.1,  // ±0.1g for accel
            mqtt_topic: "car/sensors/imu/wt901",
        }
    }

    fn poll(&mut self) -> Option<SensorReading> {
        // IMU data is fed via feed_byte(), so we just return current state
        Some(SensorReading {
            sensor_id: SensorId(1),
            timestamp_us: self.parser.data.timestamp_us,
            data: SensorData::Imu(ImuReading {
                accel: [self.parser.data.ax, self.parser.data.ay, self.parser.data.az],
                gyro: [self.parser.data.wx, self.parser.data.wy, self.parser.data.wz],
                mag: None,  // WT901 has magnetometer but we'll add later
                temperature: None,
            }),
        })
    }

    fn init(&mut self) -> Result<(), SensorError> {
        // UART already initialized in main
        Ok(())
    }

    fn calibrate(&mut self) -> Result<(), SensorError> {
        // Calibration handled separately in main
        Ok(())
    }
}

/// NEO-6M GPS sensor plugin
pub struct Neo6mGpsSensor {
    parser: NmeaParser,
    last_update: u64,
}

impl Neo6mGpsSensor {
    pub fn new() -> Self {
        Self {
            parser: NmeaParser::new(),
            last_update: 0,
        }
    }

    /// Feed UART byte (called from main loop)
    pub fn feed_byte(&mut self, byte: u8) -> bool {
        self.parser.feed_byte(byte)
    }

    /// Get mutable reference to parser (for local coords, etc.)
    pub fn parser_mut(&mut self) -> &mut NmeaParser {
        &mut self.parser
    }
}

impl Sensor for Neo6mGpsSensor {
    fn capabilities(&self) -> SensorCapabilities {
        SensorCapabilities {
            id: SensorId(2),  // GPS = sensor 2
            name: "NEO-6M GPS",
            measurement_types: &[
                MeasurementType::Position3D,
                MeasurementType::Velocity3D,
                MeasurementType::Heading,
            ],
            update_rate_hz: 5.0,
            latency_ms: 200.0,  // GPS has ~200ms latency
            accuracy: 2.5,  // ±2.5m CEP
            mqtt_topic: "car/sensors/gps/neo6m",
        }
    }

    fn poll(&mut self) -> Option<SensorReading> {
        if !self.parser.last_fix.valid {
            return None;
        }

        let now_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };

        Some(SensorReading {
            sensor_id: SensorId(2),
            timestamp_us: now_us,
            data: SensorData::Gps(GpsReading {
                lat: self.parser.last_fix.lat,
                lon: self.parser.last_fix.lon,
                alt: None,
                speed: self.parser.last_fix.speed,
                course: self.parser.last_fix.course,
                hdop: None,
                satellites: None,
            }),
        })
    }

    fn init(&mut self) -> Result<(), SensorError> {
        // UART already initialized in main
        Ok(())
    }

    fn is_healthy(&self) -> bool {
        self.parser.is_warmed_up()
    }
}

/// Example: CAN bus sensor plugin (for OBD2 data)
/// This is a template showing how easy it is to add new sensors
pub struct CanBusSensor {
    // Add CAN driver here
    last_reading_us: u64,
}

impl CanBusSensor {
    pub fn new() -> Self {
        Self {
            last_reading_us: 0,
        }
    }
}

impl Sensor for CanBusSensor {
    fn capabilities(&self) -> SensorCapabilities {
        SensorCapabilities {
            id: SensorId(10),  // CAN bus = sensor 10
            name: "CAN Bus (OBD2)",
            measurement_types: &[
                MeasurementType::Throttle,
                MeasurementType::Brake,
                MeasurementType::Gear,
                MeasurementType::Custom("RPM"),
            ],
            update_rate_hz: 50.0,
            latency_ms: 10.0,
            accuracy: 1.0,
            mqtt_topic: "car/sensors/can/obd2",
        }
    }

    fn poll(&mut self) -> Option<SensorReading> {
        // TODO: Read CAN bus
        // For now, return None
        None
    }

    fn init(&mut self) -> Result<(), SensorError> {
        // TODO: Initialize CAN controller
        Ok(())
    }
}

/// Example: Wheel speed sensor plugin
pub struct WheelSpeedSensor {
    last_reading_us: u64,
}

impl WheelSpeedSensor {
    pub fn new() -> Self {
        Self {
            last_reading_us: 0,
        }
    }
}

impl Sensor for WheelSpeedSensor {
    fn capabilities(&self) -> SensorCapabilities {
        SensorCapabilities {
            id: SensorId(11),  // Wheel speed = sensor 11
            name: "ABS Wheel Speed",
            measurement_types: &[MeasurementType::WheelSpeed],
            update_rate_hz: 100.0,
            latency_ms: 5.0,
            accuracy: 0.1,  // ±0.1 rad/s
            mqtt_topic: "car/sensors/wheels/abs",
        }
    }

    fn poll(&mut self) -> Option<SensorReading> {
        // TODO: Read wheel speed sensors via GPIO interrupts
        None
    }

    fn init(&mut self) -> Result<(), SensorError> {
        // TODO: Setup GPIO interrupt handlers
        Ok(())
    }
}

/// Example: Steering angle sensor
pub struct SteeringAngleSensor {
    last_reading_us: u64,
}

impl SteeringAngleSensor {
    pub fn new() -> Self {
        Self {
            last_reading_us: 0,
        }
    }
}

impl Sensor for SteeringAngleSensor {
    fn capabilities(&self) -> SensorCapabilities {
        SensorCapabilities {
            id: SensorId(12),  // Steering = sensor 12
            name: "Steering Angle",
            measurement_types: &[MeasurementType::SteeringAngle],
            update_rate_hz: 100.0,
            latency_ms: 5.0,
            accuracy: 0.5,  // ±0.5 degrees
            mqtt_topic: "car/sensors/steering",
        }
    }

    fn poll(&mut self) -> Option<SensorReading> {
        // TODO: Read steering angle (could be CAN, potentiometer, or rotary encoder)
        None
    }

    fn init(&mut self) -> Result<(), SensorError> {
        Ok(())
    }
}
