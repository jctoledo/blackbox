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

/// Trait for extensible sensor data types
/// Implement this trait for any custom sensor data type to integrate with the framework
pub trait SensorDataType: core::fmt::Debug {
    /// Get a type name for this sensor data (used for logging and debugging)
    fn type_name(&self) -> &'static str;

    /// Downcast to concrete type (for type-safe access)
    fn as_any(&self) -> &dyn core::any::Any;

    /// Downcast to mutable concrete type
    fn as_any_mut(&mut self) -> &mut dyn core::any::Any;
}

/// Universal sensor reading with timestamp
/// Now uses trait objects for extensibility (Open/Closed Principle)
#[derive(Debug)]
pub struct SensorReading {
    pub sensor_id: SensorId,
    pub timestamp_us: u64,
    pub data: Box<dyn SensorDataType>,
}

impl SensorReading {
    /// Helper to downcast data to specific type
    pub fn downcast_data<T: 'static>(&self) -> Option<&T> {
        self.data.as_any().downcast_ref::<T>()
    }

    /// Helper to downcast data to specific type (mutable)
    pub fn downcast_data_mut<T: 'static>(&mut self) -> Option<&mut T> {
        self.data.as_any_mut().downcast_mut::<T>()
    }
}

/// Legacy enum for backward compatibility during migration
/// DEPRECATED: Use trait-based SensorDataType instead
#[derive(Debug, Clone)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
#[deprecated(since = "0.2.0", note = "Use SensorDataType trait instead")]
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

impl SensorDataType for ImuReading {
    fn type_name(&self) -> &'static str {
        "ImuReading"
    }

    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn core::any::Any {
        self
    }
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

impl SensorDataType for GpsReading {
    fn type_name(&self) -> &'static str {
        "GpsReading"
    }

    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn core::any::Any {
        self
    }
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub struct WheelSpeedReading {
    pub fl: f32,  // Front-left (rad/s or m/s)
    pub fr: f32,
    pub rl: f32,
    pub rr: f32,
}

impl SensorDataType for WheelSpeedReading {
    fn type_name(&self) -> &'static str {
        "WheelSpeedReading"
    }

    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn core::any::Any {
        self
    }
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub struct CanReading {
    pub can_id: u32,
    pub data: Vec<u8>,
}

impl SensorDataType for CanReading {
    fn type_name(&self) -> &'static str {
        "CanReading"
    }

    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn core::any::Any {
        self
    }
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub struct LidarReading {
    pub points: Vec<[f32; 3]>,  // Simplified
}

impl SensorDataType for LidarReading {
    fn type_name(&self) -> &'static str {
        "LidarReading"
    }

    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn core::any::Any {
        self
    }
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "mqtt", derive(Serialize, Deserialize))]
pub struct CameraReading {
    pub frame_id: u64,
    pub detections: Vec<Detection>,
}

impl SensorDataType for CameraReading {
    fn type_name(&self) -> &'static str {
        "CameraReading"
    }

    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn core::any::Any {
        self
    }
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

impl SensorDataType for CustomReading {
    fn type_name(&self) -> &'static str {
        "CustomReading"
    }

    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn core::any::Any {
        self
    }
}

/// Core trait that all sensors must implement
/// This trait focuses on the essential sensor operations (Interface Segregation Principle)
pub trait Sensor {
    /// Get sensor capabilities (called once at registration)
    fn capabilities(&self) -> SensorCapabilities;

    /// Poll for new data (non-blocking)
    /// Returns None if no new data available
    fn poll(&mut self) -> Option<SensorReading>;

    /// Initialize sensor hardware
    fn init(&mut self) -> Result<(), SensorError>;
}

/// Optional trait for sensors that require calibration
/// Implement this only for sensors that need calibration (Interface Segregation Principle)
pub trait CalibratableSensor: Sensor {
    /// Calibrate sensor (e.g., bias learning for IMU)
    fn calibrate(&mut self) -> Result<(), SensorError>;

    /// Check if calibration is required
    fn needs_calibration(&self) -> bool {
        true
    }
}

/// Optional trait for sensors that support health monitoring
/// Implement this only for sensors with health check capabilities (Interface Segregation Principle)
pub trait HealthMonitoredSensor: Sensor {
    /// Perform health check on sensor
    /// Returns true if sensor is operating normally
    fn is_healthy(&self) -> bool;

    /// Get detailed health status information
    fn health_status(&self) -> &'static str {
        if self.is_healthy() {
            "OK"
        } else {
            "DEGRADED"
        }
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

    /// Calibrate all sensors that support calibration
    /// Returns count of successfully calibrated sensors
    ///
    /// Note: Due to trait object limitations, this is a placeholder.
    /// In practice, calibration should be called on specific sensor instances
    /// that implement CalibratableSensor trait.
    pub fn calibrate_all(&mut self) -> usize {
        // Placeholder - trait objects don't support downcasting to other traits
        // Callers should store CalibratableSensor references separately if needed
        0
    }

    /// Get health status of all sensors
    /// Returns vector of (sensor_id, is_healthy) tuples
    pub fn health_check_all(&self) -> Vec<(SensorId, bool)> {
        // Similar limitation as calibrate_all
        // In practice, health monitoring would be implemented differently
        // or tracked separately
        Vec::new()
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
        println!("[SENSOR {}] {} @ {}us",
                 reading.sensor_id.0,
                 reading.data.type_name(),
                 reading.timestamp_us);
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
                data: Box::new(CustomReading { data: vec![42] }),
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
