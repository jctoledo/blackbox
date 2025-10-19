/// IMU sensor adapter for WT901 driver
///
/// This module re-exports the WT901 driver types for use in the application.
// Re-export the driver types
pub use wt901::{ImuBias, ImuCalibrator, PacketType, Wt901Parser};
