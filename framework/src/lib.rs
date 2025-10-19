//! Open Motorsport Telemetry Framework
//!
//! A modular, extensible framework for building custom automotive sensor fusion systems.
//! Build professional-grade telemetry for track days, racing, vehicle dynamics research,
//! and fleet monitoring without vendor lock-in.
//!
//! ## Features
//!
//! - **Plugin Architecture**: Add sensors without modifying existing code
//! - **Sensor Abstraction**: Hardware-independent sensor interface
//! - **Extended Kalman Filter**: 7-state sensor fusion (position, velocity, yaw, biases)
//! - **Coordinate Transforms**: Body ↔ Earth ↔ Vehicle frame conversions
//! - **Distributed Support**: MQTT-based multi-board sensor networks (optional)
//! - **No-std Compatible**: Works on embedded systems without operating system
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────┐
//! │  Sensor Framework (Traits & Registry)   │
//! ├─────────────────────────────────────────┤
//! │  Sensor Fusion (EKF, Transforms)        │
//! ├─────────────────────────────────────────┤
//! │  Sensor Implementations (Plugins)       │
//! └─────────────────────────────────────────┘
//! ```
//!
//! ## Example Usage
//!
//! ```rust,no_run
//! use motorsport_telemetry::sensor_framework::{SensorRegistry, Sensor};
//! use motorsport_telemetry::ekf::Ekf;
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
//!     // for reading in readings {
//!     //     // Update EKF based on sensor type
//!     // }
//! }
//! ```
//!
//! ## Use Cases
//!
//! - **Track Day Data Logging** - Record lap times, g-forces, and racing lines
//! - **Vehicle Dynamics Research** - Collect data for suspension, brake, and aero tuning
//! - **Fleet Monitoring** - Track driver behavior and vehicle health
//! - **Autonomous Vehicles** - Sensor fusion for localization and navigation
//! - **Educational Projects** - Learn sensor fusion and embedded programming
//!
//! ## Modules
//!
//! - [`sensor_framework`] - Core plugin architecture and sensor traits
//! - [`sensors`] - Basic sensor trait definitions
//! - [`ekf`] - Extended Kalman Filter for sensor fusion
//! - [`transforms`] - Coordinate frame transformations

// Core framework modules
pub mod sensor_framework;
pub mod sensors;
pub mod ekf;
pub mod transforms;

// Re-export commonly used types
pub use sensor_framework::{
    Sensor, SensorRegistry, SensorCapabilities, SensorReading, SensorData, SensorId,
};
pub use ekf::Ekf;
