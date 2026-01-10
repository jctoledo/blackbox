/// GPS sensor adapter
///
/// This module re-exports the GPS driver types for use in the application.
/// Supports both NEO-6M and NEO-M9N modules (same NMEA protocol).
// Re-export the driver types
pub use neo6m::ubx::generate_init_sequence;
pub use neo6m::NmeaParser;
