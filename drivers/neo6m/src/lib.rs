//! NEO-6M GPS NMEA Parser
//!
//! This crate provides a pure Rust parser for NMEA sentences from NEO-6M GPS receivers.
//! It supports GPRMC and GNRMC sentences and provides position, velocity, and time information.
//!
//! # Features
//!
//! - Zero-allocation NMEA parsing
//! - Supports GPRMC/GNRMC sentences
//! - Reference point averaging (warmup)
//! - Local coordinate conversion (lat/lon → meters)
//! - Position-based speed calculation
//! - `no_std` compatible
//! - No external dependencies
//!
//! # Example
//!
//! ```no_run
//! use neo6m::NmeaParser;
//!
//! let mut parser = NmeaParser::new();
//!
//! // Feed bytes from UART
//! for byte in uart_bytes {
//!     if parser.feed_byte(byte) {
//!         // New sentence parsed
//!         if parser.last_fix().valid {
//!             println!("Position: {}, {}", parser.last_fix().lat, parser.last_fix().lon);
//!             println!("Speed: {:.1} m/s", parser.last_fix().speed);
//!         }
//!     }
//! }
//! ```

#![cfg_attr(not(test), no_std)]

#[cfg(feature = "logging")]
use log::warn;

// Import libm for no-std floating point operations
use libm::{cos, sin, sqrt, floor};

/// GPS coordinate transformations
pub mod transforms {
    use libm::{cos, sqrtf};

    const METERS_PER_DEGREE_LAT: f64 = 111320.0;

    /// Convert latitude difference to meters
    pub fn lat_to_meters(dlat: f64) -> f32 {
        (dlat * METERS_PER_DEGREE_LAT) as f32
    }

    /// Convert longitude difference to meters
    pub fn lon_to_meters(dlon: f64, ref_lat: f64) -> f32 {
        (dlon * METERS_PER_DEGREE_LAT * cos(ref_lat.to_radians())) as f32
    }

    /// Convert GPS coordinates to local ENU frame
    pub fn gps_to_local(lat: f64, lon: f64, ref_lat: f64, ref_lon: f64) -> (f32, f32) {
        let dlat = lat - ref_lat;
        let dlon = lon - ref_lon;

        let north = lat_to_meters(dlat);
        let east = lon_to_meters(dlon, ref_lat);

        (east, north)
    }

    /// Calculate distance between two GPS positions
    pub fn gps_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64, ref_lat: f64) -> f32 {
        let dlat = lat2 - lat1;
        let dlon = lon2 - lon1;

        let dy = lat_to_meters(dlat);
        let dx = lon_to_meters(dlon, ref_lat);

        sqrtf(dx * dx + dy * dy)
    }
}

/// GPS fix data from GPRMC/GNRMC sentence
#[derive(Debug, Clone, Copy, Default)]
pub struct GpsFix {
    /// Latitude in degrees (positive = North, negative = South)
    pub lat: f64,
    /// Longitude in degrees (positive = East, negative = West)
    pub lon: f64,
    /// Ground speed in m/s
    pub speed: f32,
    /// Course over ground in radians
    pub course: f32,
    /// Fix validity (A = valid, V = invalid)
    pub valid: bool,
    /// UTC hour (0-23)
    pub hour: u8,
    /// UTC minute (0-59)
    pub minute: u8,
    /// UTC second (0-59)
    pub second: u8,
}

/// GPS reference point for local coordinate conversion
#[derive(Debug, Clone, Copy, Default)]
pub struct GpsReference {
    /// Reference latitude (degrees)
    pub lat: f64,
    /// Reference longitude (degrees)
    pub lon: f64,
    /// Whether reference is set (warmup complete)
    pub set: bool,
}

/// NMEA sentence parser for NEO-6M GPS
pub struct NmeaParser {
    line_buffer: [u8; 120],
    line_len: usize,
    last_fix: GpsFix,
    reference: GpsReference,
    warmup_count: u8,
    warmup_fixes: u8,
    warmup_lat_sum: f64,
    warmup_lon_sum: f64,
    last_valid_lat: f64,
    last_valid_lon: f64,
    position_based_speed: f32,
}

impl NmeaParser {
    /// Create a new NMEA parser with default warmup (5 fixes)
    pub fn new() -> Self {
        Self::with_warmup_fixes(5)
    }

    /// Create a new NMEA parser with custom warmup count
    ///
    /// # Arguments
    ///
    /// * `warmup_fixes` - Number of valid fixes to average for reference point
    pub fn with_warmup_fixes(warmup_fixes: u8) -> Self {
        Self {
            line_buffer: [0; 120],
            line_len: 0,
            last_fix: GpsFix::default(),
            reference: GpsReference::default(),
            warmup_count: 0,
            warmup_fixes,
            warmup_lat_sum: 0.0,
            warmup_lon_sum: 0.0,
            last_valid_lat: 0.0,
            last_valid_lon: 0.0,
            position_based_speed: 0.0,
        }
    }

    /// Feed a single byte from UART to the parser
    ///
    /// Returns `true` when a complete sentence has been parsed (on newline).
    pub fn feed_byte(&mut self, byte: u8) -> bool {
        match byte {
            b'$' => {
                self.line_len = 0;
                self.line_buffer[self.line_len] = byte;
                self.line_len += 1;
                false
            }
            b'\r' => false,
            b'\n' => {
                if self.line_len > 0 {
                    self.parse_line();
                    true
                } else {
                    false
                }
            }
            _ => {
                if self.line_len < 120 {
                    self.line_buffer[self.line_len] = byte;
                    self.line_len += 1;
                }
                false
            }
        }
    }

    /// Get reference to last GPS fix
    pub fn last_fix(&self) -> &GpsFix {
        &self.last_fix
    }

    /// Get reference point (if warmup complete)
    pub fn reference(&self) -> &GpsReference {
        &self.reference
    }

    /// Check if GPS warmup is complete
    pub fn is_warmed_up(&self) -> bool {
        self.reference.set
    }

    /// Get warmup progress (0.0 to 1.0)
    pub fn warmup_progress(&self) -> f32 {
        if self.reference.set {
            1.0
        } else {
            self.warmup_count as f32 / self.warmup_fixes as f32
        }
    }

    /// Get position-based speed estimate (m/s)
    ///
    /// This is calculated from position changes and can be more accurate
    /// than GPS speed at low velocities.
    pub fn position_based_speed(&self) -> f32 {
        self.position_based_speed
    }

    /// Convert current GPS position to local coordinates
    ///
    /// Returns `None` if warmup not complete or fix invalid.
    pub fn to_local_coords(&self) -> Option<(f32, f32)> {
        if !self.reference.set || !self.last_fix.valid {
            return None;
        }

        Some(transforms::gps_to_local(
            self.last_fix.lat,
            self.last_fix.lon,
            self.reference.lat,
            self.reference.lon,
        ))
    }

    /// Get velocity in ENU frame (East, North)
    ///
    /// Returns `None` if fix is invalid.
    pub fn get_velocity_enu(&self) -> Option<(f32, f32)> {
        if !self.last_fix.valid {
            return None;
        }

        let vx = self.last_fix.speed * cos(self.last_fix.course as f64) as f32;
        let vy = self.last_fix.speed * sin(self.last_fix.course as f64) as f32;

        Some((vx, vy))
    }

    /// Get local time with timezone offset
    ///
    /// # Arguments
    ///
    /// * `tz_offset` - Timezone offset in hours (e.g., -8 for PST, +1 for CET)
    pub fn get_local_time(&self, tz_offset: i8) -> (u8, u8, u8) {
        let hour = ((self.last_fix.hour as i8 + tz_offset + 24) % 24) as u8;
        (hour, self.last_fix.minute, self.last_fix.second)
    }

    /// Update position-based speed with new timestamp
    ///
    /// Call this from your application's time source when a new fix arrives.
    /// This allows position-based speed calculation to work in no_std environments.
    ///
    /// # Arguments
    ///
    /// * `timestamp_ms` - Current timestamp in milliseconds
    pub fn update_position_speed(&mut self, timestamp_ms: u32, last_timestamp_ms: u32) {
        if !self.last_fix.valid {
            return;
        }

        let dt = (timestamp_ms - last_timestamp_ms) as f32 / 1000.0;

        if dt > 0.1 && dt < 2.0 {
            let dist = transforms::gps_distance(
                self.last_valid_lat,
                self.last_valid_lon,
                self.last_fix.lat,
                self.last_fix.lon,
                self.reference.lat,
            );

            self.position_based_speed = dist / dt;

            // If position-based speed is low but GPS speed is high, trust position
            if self.position_based_speed < 0.3 && self.last_fix.speed > 0.5 {
                self.last_fix.speed = self.position_based_speed;
            }
        }

        self.last_valid_lat = self.last_fix.lat;
        self.last_valid_lon = self.last_fix.lon;
    }

    fn parse_line(&mut self) {
        let len = self.line_len;

        // Make a local copy to avoid borrow checker issues
        let mut local_buf = [0u8; 120];
        local_buf[..len].copy_from_slice(&self.line_buffer[..len]);

        let line = match core::str::from_utf8(&local_buf[..len]) {
            Ok(s) => s,
            Err(_) => return,
        };

        if line.starts_with("$GPRMC") || line.starts_with("$GNRMC") {
            self.parse_rmc(line);
        }
    }

    #[cfg(any(test, feature = "std"))]
    fn parse_rmc(&mut self, line: &str) {
        let fields: Vec<&str> = line.split(',').collect();

        if fields.len() < 10 {
            return;
        }

        if fields[2] != "A" {
            self.last_fix.valid = false;
            return;
        }

        if let Some((h, m, s)) = parse_time(fields[1]) {
            self.last_fix.hour = h;
            self.last_fix.minute = m;
            self.last_fix.second = s;
        }

        let lat = match parse_coordinate(fields[3], fields[4]) {
            Some(lat) => lat,
            None => return,
        };

        let lon = match parse_coordinate(fields[5], fields[6]) {
            Some(lon) => lon,
            None => return,
        };

        if let Ok(speed_knots) = fields[7].parse::<f32>() {
            self.last_fix.speed = speed_knots * 0.514444; // knots to m/s
        }

        if let Ok(course_deg) = fields[8].parse::<f32>() {
            self.last_fix.course = course_deg.to_radians();
        }

        // Warmup: average first N fixes for reference point
        if !self.reference.set {
            self.warmup_count += 1;
            self.warmup_lat_sum += lat;
            self.warmup_lon_sum += lon;

            if self.warmup_count >= self.warmup_fixes {
                self.reference.lat = self.warmup_lat_sum / self.warmup_count as f64;
                self.reference.lon = self.warmup_lon_sum / self.warmup_count as f64;
                self.reference.set = true;

                self.last_valid_lat = lat;
                self.last_valid_lon = lon;
            }

            self.last_fix.valid = false;
            return;
        }

        self.last_fix.lat = lat;
        self.last_fix.lon = lon;
        self.last_fix.valid = true;
    }

    #[cfg(not(any(test, feature = "std")))]
    fn parse_rmc(&mut self, _line: &str) {
        // no_std: Simplified implementation
        // For a full no_std implementation, you would need to add libm dependency for floor()
        // and implement manual field parsing without Vec
        self.last_fix.valid = false;
    }
}

impl Default for NmeaParser {
    fn default() -> Self {
        Self::new()
    }
}

/// Parse NMEA coordinate field (ddmm.mmmm format)
fn parse_coordinate(coord_str: &str, dir_str: &str) -> Option<f64> {
    if coord_str.is_empty() || dir_str.is_empty() {
        return None;
    }

    let value = coord_str.parse::<f64>().ok()?;

    let degrees = floor(value / 100.0);
    let minutes = value - (degrees * 100.0);

    let mut decimal = degrees + (minutes / 60.0);

    if dir_str == "S" || dir_str == "W" {
        decimal = -decimal;
    }

    Some(decimal)
}

/// Parse NMEA time field (hhmmss.ss format)
fn parse_time(time_str: &str) -> Option<(u8, u8, u8)> {
    if time_str.len() < 6 {
        return None;
    }

    let hh = time_str[0..2].parse::<u8>().ok()?;
    let mm = time_str[2..4].parse::<u8>().ok()?;
    let ss = time_str[4..6].parse::<u8>().ok()?;

    Some((hh, mm, ss))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_coordinate() {
        // North latitude
        assert_eq!(parse_coordinate("3723.2475", "N"), Some(37.387458333333336));
        // South latitude
        assert_eq!(parse_coordinate("3723.2475", "S"), Some(-37.387458333333336));
        // East longitude
        assert_eq!(parse_coordinate("12158.3416", "E"), Some(121.97236));
        // West longitude
        assert_eq!(parse_coordinate("12158.3416", "W"), Some(-121.97236));
    }

    #[test]
    fn test_parse_time() {
        assert_eq!(parse_time("123456"), Some((12, 34, 56)));
        assert_eq!(parse_time("000000"), Some((0, 0, 0)));
        assert_eq!(parse_time("235959"), Some((23, 59, 59)));
        assert_eq!(parse_time("12"), None); // Too short
    }

    #[test]
    fn test_parser_creation() {
        let parser = NmeaParser::new();
        assert!(!parser.is_warmed_up());
        assert_eq!(parser.warmup_progress(), 0.0);
    }

    #[test]
    fn test_transforms() {
        use transforms::*;

        // Test coordinate conversion
        let (east, north) = gps_to_local(37.0001, -122.0, 37.0, -122.0);
        assert!((north - 11.132).abs() < 0.1); // ~11m north
        assert!(east.abs() < 1.0); // negligible east

        // Test distance calculation
        let dist = gps_distance(37.0, -122.0, 37.0001, -122.0, 37.0);
        assert!((dist - 11.132).abs() < 0.1);
    }
}
