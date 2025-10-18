/// Sensor Framework - Plugin architecture for distributed sensor fusion
///
/// Design philosophy:
/// - Each sensor is an independent publisher
/// - Sensors self-describe their capabilities
/// - Fusion broker subscribes to sensor topics
/// - Adding a new sensor = implement trait + register
/// - No coupling between sensors

#[cfg(feature = "mqtt")]
use serde::{Serialize, Deserialize};

/// Unique sensor identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub struct SensorId(pub u16);

/// Types of measurements a sensor can provide
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub enum MeasurementType {
    // Position & Velocity
    Position3D,          // GPS, UWB, SLAM
    Velocity3D,          // GPS, wheel encoders
    Acceleration3D,      // IMU
    AngularVelocity3D,   // Gyro

    // Orientation
    Orientation,         // IMU, magnetometer
    Heading,             // Magnetometer, GPS course

    // Vehicle dynamics
    WheelSpeed,          // ABS sensors
    SteeringAngle,       // Steering sensor
    Throttle,            // CAN bus
    Brake,               // CAN bus
    Gear,                // CAN bus

    // Environmental
    Temperature,
    Pressure,
    Humidity,

    // Vision
    LaneDetection,
    ObjectDetection,

    // Custom
    Custom(&'static str),
}

/// Sensor capabilities and metadata
#[derive(Debug, Clone)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub struct SensorCapabilities {
    pub id: SensorId,
    pub name: &'static str,
    pub measurement_types: &'static [MeasurementType],
    pub update_rate_hz: f32,
    pub latency_ms: f32,
    pub accuracy: f32,  // Application-specific units
    pub mqtt_topic: &'static str,
}

/// Universal sensor reading with timestamp
#[derive(Debug, Clone)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub struct SensorReading {
    pub sensor_id: SensorId,
    pub timestamp_us: u64,
    pub data: SensorData,
}

/// Polymorphic sensor data
#[derive(Debug, Clone)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub enum SensorData {
    Imu(ImuReading),
    Gps(GpsReading),
    WheelSpeed(WheelSpeedReading),
    Can(CanReading),
    Lidar(LidarReading),
    Camera(CameraReading),
    Custom(CustomReading),
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub struct ImuReading {
    pub accel: [f32; 3],      // m/s²
    pub gyro: [f32; 3],       // rad/s
    pub mag: Option<[f32; 3]>,
    pub temperature: Option<f32>,
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub struct GpsReading {
    pub lat: f64,
    pub lon: f64,
    pub alt: Option<f32>,
    pub speed: f32,           // m/s
    pub course: f32,          // radians
    pub hdop: Option<f32>,
    pub satellites: Option<u8>,
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub struct WheelSpeedReading {
    pub fl: f32,  // Front-left (rad/s or m/s)
    pub fr: f32,
    pub rl: f32,
    pub rr: f32,
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub struct CanReading {
    pub can_id: u32,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub struct LidarReading {
    pub points: Vec<[f32; 3]>,  // Simplified
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub struct CameraReading {
    pub frame_id: u64,
    pub detections: Vec<Detection>,
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub struct Detection {
    pub class: String,
    pub confidence: f32,
    pub bbox: [f32; 4],  // x, y, w, h
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub struct CustomReading {
    pub data: Vec<u8>,
}

/// Trait that all sensors must implement
pub trait Sensor {
    /// Get sensor capabilities (called once at registration)
    fn capabilities(&self) -> SensorCapabilities;

    /// Poll for new data (non-blocking)
    /// Returns None if no new data available
    fn poll(&mut self) -> Option<SensorReading>;

    /// Initialize sensor hardware
    fn init(&mut self) -> Result<(), SensorError>;

    /// Calibrate sensor (if applicable)
    fn calibrate(&mut self) -> Result<(), SensorError> {
        Ok(()) // Default: no calibration needed
    }

    /// Health check
    fn is_healthy(&self) -> bool {
        true
    }
}

/// Sensor errors
#[derive(Debug, Clone)]
pub enum SensorError {
    InitFailed(&'static str),
    CalibrationFailed,
    CommunicationError,
    HardwareError,
    Timeout,
}

/// Sensor registry - manages all active sensors
pub struct SensorRegistry {
    sensors: Vec<Box<dyn Sensor>>,
    capabilities: Vec<SensorCapabilities>,
}

impl SensorRegistry {
    pub fn new() -> Self {
        Self {
            sensors: Vec::new(),
            capabilities: Vec::new(),
        }
    }

    /// Register a new sensor
    pub fn register(&mut self, mut sensor: Box<dyn Sensor>) -> Result<SensorId, SensorError> {
        sensor.init()?;
        let caps = sensor.capabilities();
        let id = caps.id;

        self.capabilities.push(caps);
        self.sensors.push(sensor);

        Ok(id)
    }

    /// Poll all sensors and collect readings
    pub fn poll_all(&mut self) -> Vec<SensorReading> {
        self.sensors
            .iter_mut()
            .filter_map(|s| s.poll())
            .collect()
    }

    /// Get capabilities of all registered sensors
    pub fn get_capabilities(&self) -> &[SensorCapabilities] {
        &self.capabilities
    }

    /// Get sensor by ID
    pub fn get_sensor(&mut self, id: SensorId) -> Option<&mut Box<dyn Sensor>> {
        self.sensors
            .iter_mut()
            .find(|s| s.capabilities().id == id)
    }
}

/// Sensor data publisher (for distributed architecture)
pub trait SensorPublisher {
    /// Publish sensor reading to MQTT broker
    fn publish(&mut self, reading: &SensorReading) -> Result<(), PublishError>;

    /// Publish sensor capabilities (on startup)
    fn announce(&mut self, caps: &SensorCapabilities) -> Result<(), PublishError>;
}

#[derive(Debug)]
pub enum PublishError {
    MqttError,
    SerializationError,
}

/// Example: Simple logging publisher (for testing)
pub struct LogPublisher;

impl SensorPublisher for LogPublisher {
    fn publish(&mut self, reading: &SensorReading) -> Result<(), PublishError> {
        println!("[SENSOR {}] {:?}", reading.sensor_id.0, reading.data);
        Ok(())
    }

    fn announce(&mut self, caps: &SensorCapabilities) -> Result<(), PublishError> {
        println!("[ANNOUNCE] Sensor {} ({}) @ {}Hz on topic '{}'",
                 caps.id.0, caps.name, caps.update_rate_hz, caps.mqtt_topic);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct DummySensor {
        id: SensorId,
        counter: u64,
    }

    impl Sensor for DummySensor {
        fn capabilities(&self) -> SensorCapabilities {
            SensorCapabilities {
                id: self.id,
                name: "DummySensor",
                measurement_types: &[MeasurementType::Temperature],
                update_rate_hz: 10.0,
                latency_ms: 1.0,
                accuracy: 0.1,
                mqtt_topic: "sensors/dummy",
            }
        }

        fn poll(&mut self) -> Option<SensorReading> {
            self.counter += 1;
            Some(SensorReading {
                sensor_id: self.id,
                timestamp_us: self.counter * 100_000,
                data: SensorData::Custom(CustomReading { data: vec![42] }),
            })
        }

        fn init(&mut self) -> Result<(), SensorError> {
            Ok(())
        }
    }

    #[test]
    fn test_sensor_registry() {
        let mut registry = SensorRegistry::new();

        let sensor1 = Box::new(DummySensor { id: SensorId(1), counter: 0 });
        let sensor2 = Box::new(DummySensor { id: SensorId(2), counter: 0 });

        registry.register(sensor1).unwrap();
        registry.register(sensor2).unwrap();

        let readings = registry.poll_all();
        assert_eq!(readings.len(), 2);
    }
}
