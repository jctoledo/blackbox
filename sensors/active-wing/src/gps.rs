/// GPS sensor adapter for NEO-6M driver
///
/// This module re-exports the NEO-6M driver and provides application-specific adapters.

// Re-export the driver types
pub use neo6m::{GpsFix, GpsReference, NmeaParser};

use crate::sensors::{GpsSensor, SensorError};

/// UART-based NMEA GPS receiver (application adapter)
///
/// This wraps the pure `neo6m::NmeaParser` driver and implements
/// the application's `GpsSensor` trait.
pub struct UartNmeaGps {
    pub parser: NmeaParser,
    last_timestamp_ms: u32,
}

impl UartNmeaGps {
    pub fn new() -> Self {
        Self {
            parser: NmeaParser::new(),
            last_timestamp_ms: 0,
        }
    }

    /// Feed a byte from UART, returns true if sentence was completed
    pub fn feed_byte(&mut self, byte: u8) -> bool {
        let sentence_complete = self.parser.feed_byte(byte);

        // Update position-based speed if new sentence and fix is valid
        if sentence_complete && self.parser.last_fix().valid {
            let now_ms = unsafe { (esp_idf_svc::sys::esp_timer_get_time() / 1000) as u32 };
            self.parser
                .update_position_speed(now_ms, self.last_timestamp_ms);
            self.last_timestamp_ms = now_ms;
        }

        sentence_complete
    }

    /// Get reference to last GPS fix
    pub fn last_fix(&self) -> &GpsFix {
        self.parser.last_fix()
    }

    /// Check if GPS warmup is complete
    pub fn is_warmed_up(&self) -> bool {
        self.parser.is_warmed_up()
    }

    /// Get warmup progress (0.0 to 1.0)
    pub fn warmup_progress(&self) -> f32 {
        self.parser.warmup_progress()
    }

    /// Get position-based speed
    pub fn position_based_speed(&self) -> f32 {
        self.parser.position_based_speed()
    }

    /// Convert current position to local coordinates
    pub fn to_local_coords(&self) -> Option<(f32, f32)> {
        self.parser.to_local_coords()
    }

    /// Get velocity in ENU frame
    pub fn get_velocity_enu(&self) -> Option<(f32, f32)> {
        self.parser.get_velocity_enu()
    }
}

impl Default for UartNmeaGps {
    fn default() -> Self {
        Self::new()
    }
}

/// Implement the application's sensor trait
impl GpsSensor for UartNmeaGps {
    fn poll(&mut self) -> Result<bool, SensorError> {
        // Actual polling happens via feed_byte() called from main loop
        Ok(false)
    }

    fn get_fix(&self) -> &GpsFix {
        self.parser.last_fix()
    }

    fn is_ready(&self) -> bool {
        self.parser.is_warmed_up()
    }

    fn to_local_coords(&self) -> Option<(f32, f32)> {
        self.parser.to_local_coords()
    }

    fn get_velocity_enu(&self) -> Option<(f32, f32)> {
        self.parser.get_velocity_enu()
    }

    fn position_based_speed(&self) -> f32 {
        self.parser.position_based_speed()
    }
}
