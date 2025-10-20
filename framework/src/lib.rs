//! A modular, extensible framework for building custom automotive sensor fusion
//! systems. Build professional-grade telemetry for track days, racing, vehicle
//! dynamics research, and fleet monitoring without vendor lock-in.
//!
//! ## Features
//!
//! - **Plugin Architecture**: Add sensors without modifying existing code
//!   (Open/Closed Principle)
//! - **Sensor Abstraction**: Hardware-independent sensor interface (Dependency
//!   Inversion)
//! - **Extended Kalman Filter**: 7-state sensor fusion (position, velocity,
//!   yaw, biases)
//! - **Coordinate Transforms**: Body ↔ Earth ↔ Vehicle frame conversions
//! - **SOLID Principles**: Designed following best practices for
//!   maintainability
//! - **Distributed Support**: MQTT-based multi-board sensor networks (optional)
//! - **No-std Compatible**: Works on embedded systems without operating system
//!
//! ## Architecture
//!
//! The framework follows **SOLID principles** for maximum extensibility:
//!
//! ```text
//! ┌─────────────────────────────────────────┐
//! │  Sensor Framework (Traits & Registry)   │
//! │  - Sensor (core operations)             │
//! │  - CalibratableSensor (optional)        │
//! │  - HealthMonitoredSensor (optional)     │
//! │  - SensorDataType (extensible data)     │
//! ├─────────────────────────────────────────┤
//! │  Sensor Fusion (EKF, Transforms)        │
//! │  - Pure algorithms, no sensor coupling  │
//! ├─────────────────────────────────────────┤
//! │  Sensor Implementations (Plugins)       │
//! │  - IMU, GPS, CAN, Camera, etc.         │
//! └─────────────────────────────────────────┘
//! ```
//!
//! ## Primary vs Legacy Abstractions
//!
//! **PRIMARY (Recommended)**: Use `sensor_framework` module
//! - Modern trait-based plugin architecture
//! - Extensible via `SensorDataType` trait
//! - Supports any sensor type without framework changes
//! - Interface segregation (optional traits)
//!
//! **LEGACY**: `sensors` module (ImuSensor, GpsSensor)
//! - Provided for backward compatibility
//! - Simpler for basic IMU/GPS-only applications
//! - Consider migrating to `sensor_framework` for new code
//!
//! ## Example Usage
//!
//! ```rust,no_run
//! use motorsport_telemetry::{
//!     ekf::Ekf,
//!     sensor_framework::{GpsReading, ImuReading, Sensor, SensorDataType, SensorRegistry},
//! };
//!
//! // Create sensor registry
//! let mut registry = SensorRegistry::new();
//!
//! // Add your sensors (implement Sensor trait)
//! // registry.register(Box::new(MyImuSensor::new()))?;
//! // registry.register(Box::new(MyGpsSensor::new()))?;
//!
//! // Create Extended Kalman Filter
//! let mut ekf = Ekf::new();
//!
//! // Main fusion loop
//! loop {
//!     // Poll all sensors
//!     let readings = registry.poll_all();
//!
//!     // Process readings and update EKF
//!     for reading in readings {
//!         // Downcast to specific sensor type
//!         if let Some(imu) = reading.downcast_data::<ImuReading>() {
//!             // Process IMU data
//!         } else if let Some(gps) = reading.downcast_data::<GpsReading>() {
//!             // Process GPS data
//!         }
//!     }
//! }
//! ```
//!
//! ## Creating Custom Sensor Types
//!
//! The framework is **open for extension** without modification:
//!
//! ```rust,no_run
//! use motorsport_telemetry::sensor_framework::{SensorDataType, SensorReading};
//!
//! #[derive(Debug)]
//! struct RadarReading {
//!     targets: Vec<(f32, f32, f32)>, // (distance, angle, velocity)
//! }
//!
//! impl SensorDataType for RadarReading {
//!     fn type_name(&self) -> &'static str {
//!         "RadarReading"
//!     }
//!
//!     fn as_any(&self) -> &dyn core::any::Any {
//!         self
//!     }
//!
//!     fn as_any_mut(&mut self) -> &mut dyn core::any::Any {
//!         self
//!     }
//! }
//!
//! // Now RadarReading can be used with SensorReading!
//! // No framework changes required (Open/Closed Principle)
//! ```
//!
//! ## Use Cases
//!
//! - **Track Day Data Logging** - Record lap times, g-forces, and racing lines
//! - **Vehicle Dynamics Research** - Collect data for suspension, brake, and
//!   aero tuning
//! - **Fleet Monitoring** - Track driver behavior and vehicle health
//! - **Autonomous Vehicles** - Sensor fusion for localization and navigation
//! - **Educational Projects** - Learn sensor fusion and embedded programming
//!
//! ## Modules
//!
//! - [`sensor_framework`] - **PRIMARY**: Plugin architecture with SOLID design
//! - [`sensors`] - **LEGACY**: Simplified IMU/GPS abstractions
//! - [`ekf`] - Extended Kalman Filter for sensor fusion
//! - [`transforms`] - Coordinate frame transformations

// Core framework modules
pub mod ekf;
pub mod sensor_framework;
pub mod sensors;
pub mod transforms;

// Re-export commonly used types from sensor_framework
// Re-export EKF
pub use ekf::Ekf;
// Legacy (deprecated) - kept for backward compatibility
#[allow(deprecated)]
pub use sensor_framework::SensorData;
pub use sensor_framework::{
    CalibratableSensor,
    CameraReading,
    CanReading,
    CustomReading,
    Detection,

    GpsReading,
    HealthMonitoredSensor,
    // Sensor data types
    ImuReading,
    LidarReading,
    PublishError,
    // Core traits
    Sensor,
    SensorCapabilities,
    SensorDataType,
    // Errors
    SensorError,
    SensorId,

    SensorPublisher,

    SensorReading,
    // Registry and metadata
    SensorRegistry,
    WheelSpeedReading,
};
