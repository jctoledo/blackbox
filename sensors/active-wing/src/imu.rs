/// IMU sensor adapter for WT901 driver
///
/// This module re-exports the WT901 driver and provides application-specific adapters.

// Re-export the driver types
pub use wt901::{ImuBias, ImuCalibrator, ImuData, Wt901Parser, PacketType};

use crate::sensors::{ImuSensor, SensorError};

/// UART-based WT901 IMU sensor (application adapter)
///
/// This wraps the pure `wt901::Wt901Parser` driver and implements
/// the application's `ImuSensor` trait.
pub struct UartWt901 {
    pub parser: Wt901Parser,
}

impl UartWt901 {
    pub fn new() -> Self {
        Self {
            parser: Wt901Parser::new(),
        }
    }

    /// Feed a byte from UART, returns packet type if complete packet parsed
    pub fn feed_byte(&mut self, byte: u8, timestamp_us: u64) -> Option<PacketType> {
        self.parser.feed_byte(byte, timestamp_us)
    }

    /// Set IMU bias from calibration
    pub fn set_bias(&mut self, bias: ImuBias) {
        self.parser.set_bias(bias);
    }

    /// Get raw IMU data
    pub fn data(&self) -> &ImuData {
        self.parser.data()
    }

    /// Get bias-corrected accelerometer readings
    pub fn get_accel_corrected(&self) -> (f32, f32, f32) {
        self.parser.get_accel_corrected()
    }
}

impl Default for UartWt901 {
    fn default() -> Self {
        Self::new()
    }
}

/// Implement the application's sensor trait
impl ImuSensor for UartWt901 {
    fn poll(&mut self) -> Result<Option<ImuData>, SensorError> {
        // Note: Actual polling happens via feed_byte() called from main loop
        // This just returns the current state
        Ok(None)
    }

    fn get_data(&self) -> &ImuData {
        self.parser.data()
    }

    fn get_accel_corrected(&self) -> (f32, f32, f32) {
        self.parser.get_accel_corrected()
    }
}
