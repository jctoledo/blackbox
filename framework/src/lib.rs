//! Active Wing Sensor Fusion Framework
//!
//! A modular, extensible framework for multi-sensor fusion in embedded systems.
//!
//! ## Features
//!
//! - **Plugin Architecture**: Add sensors without modifying existing code
//! - **Sensor Abstraction**: Hardware-independent sensor interface
//! - **Extended Kalman Filter**: 7-state sensor fusion (position, velocity, yaw, biases)
//! - **Coordinate Transforms**: Body ↔ Earth ↔ Vehicle frame conversions
//! - **Distributed Support**: MQTT-based multi-board sensor networks (optional)
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
//! use active_wing_framework::sensor_framework::{SensorRegistry, Sensor};
//! use active_wing_framework::ekf::Ekf;
//!
//! // Create sensor registry
//! let mut registry = SensorRegistry::new();
//!
//! // Add sensors (implement Sensor trait)
//! // registry.register(Box::new(MyImuSensor::new()))?;
//! // registry.register(Box::new(MyGpsSensor::new()))?;
//!
//! // Create Extended Kalman Filter
//! let mut ekf = Ekf::new();
//!
//! // Main loop
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
