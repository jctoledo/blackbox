//! NEO-6M / NEO-M9N GPS Driver
//!
//! This crate provides a pure Rust parser for NMEA sentences from u-blox GPS
//! receivers (NEO-6M, NEO-M9N, etc.). It supports GPRMC/GNRMC/GGA/GSA sentences
//! and provides position, velocity, satellite info, and time information.
//!
//! # Features
//!
//! - Zero-allocation NMEA parsing
//! - Supports GPRMC/GNRMC (position/velocity), GGA (satellites), GSA (DOP)
//! - UBX protocol commands for GPS configuration
//! - Reference point averaging (warmup)
//! - Local coordinate conversion (lat/lon → meters)
//! - Position-based speed calculation
//! - `no_std` compatible
//! - No external dependencies
//!
//! # Example
//!
//! ```ignore
//! use neo6m::NmeaParser;
//!
//! let mut parser = NmeaParser::new();
//!
//! // Feed bytes from UART
//! for byte in uart_bytes {
//!     if parser.feed_byte(byte) {
//!         // New sentence parsed
//!         if parser.last_fix().valid {
//!             println!(
//!                 "Position: {}, {}",
//!                 parser.last_fix().lat,
//!                 parser.last_fix().lon
//!             );
//!             println!("Speed: {:.1} m/s", parser.last_fix().speed);
//!         }
//!     }
//! }
//! ```

#![cfg_attr(not(any(test, feature = "std")), no_std)]

pub mod ubx;

// Import libm for no-std floating point operations
use libm::{cos, floor, sin};
#[cfg(feature = "logging")]
use log::warn;

#[cfg(any(test, feature = "std"))]
extern crate std;
#[cfg(any(test, feature = "std"))]
use std::vec::Vec;

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

/// GPS fix quality indicator
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum FixQuality {
    /// No fix
    #[default]
    NoFix = 0,
    /// Standard GPS fix
    GpsFix = 1,
    /// Differential GPS fix (DGPS)
    DgpsFix = 2,
    /// PPS fix
    PpsFix = 3,
    /// RTK Fixed solution
    RtkFixed = 4,
    /// RTK Float solution
    RtkFloat = 5,
    /// Dead reckoning
    DeadReckoning = 6,
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
    /// Altitude above mean sea level (meters) - from GGA
    pub altitude: f32,
    /// Number of satellites in use - from GGA
    pub satellites: u8,
    /// Fix quality indicator - from GGA
    pub fix_quality: FixQuality,
    /// Horizontal dilution of precision - from GGA/GSA
    pub hdop: f32,
    /// Position dilution of precision - from GSA
    pub pdop: f32,
    /// Vertical dilution of precision - from GSA
    pub vdop: f32,
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
    #[allow(dead_code)] // Used in warmup calculation
    warmup_lat_sum: f64,
    #[allow(dead_code)] // Used in warmup calculation
    warmup_lon_sum: f64,
    last_valid_lat: f64,
    last_valid_lon: f64,
    position_based_speed: f32,
    /// Flag set when a new valid RMC fix is received (cleared by take_new_fix)
    new_fix_available: bool,
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
            new_fix_available: false,
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

    /// Check if a new valid RMC fix is available and clear the flag
    ///
    /// Returns true only once per valid RMC sentence with position data.
    /// Use this for GPS rate calculation instead of checking last_fix().valid.
    pub fn take_new_fix(&mut self) -> bool {
        let available = self.new_fix_available;
        self.new_fix_available = false;
        available
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
    /// Returns `None` if warmup not complete.
    pub fn to_local_coords(&self) -> Option<(f32, f32)> {
        if !self.reference.set {
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
    /// Returns `None` if warmup not complete.
    pub fn get_velocity_enu(&self) -> Option<(f32, f32)> {
        if !self.reference.set {
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
    /// This allows position-based speed calculation to work in no_std
    /// environments.
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
        } else if line.starts_with("$GPGGA") || line.starts_with("$GNGGA") {
            self.parse_gga(line);
        } else if line.starts_with("$GPGSA") || line.starts_with("$GNGSA") {
            self.parse_gsa(line);
        }
    }

    /// Parse GGA sentence for altitude, satellites, fix quality, and HDOP
    /// Format: $GPGGA,hhmmss.ss,lat,N/S,lon,E/W,quality,numSV,hdop,alt,M,sep,M,age,refID*CS
    #[cfg(not(any(test, feature = "std")))]
    fn parse_gga(&mut self, line: &str) {
        let mut field_idx = 0;
        let mut field_start = 0;
        let mut fields: [&str; 15] = [""; 15];

        for (i, &byte) in line.as_bytes().iter().enumerate() {
            if byte == b',' || byte == b'*' || i == line.len() - 1 {
                if field_idx < 15 {
                    let end = if byte == b',' || byte == b'*' {
                        i
                    } else {
                        i + 1
                    };
                    fields[field_idx] = &line[field_start..end];
                    field_idx += 1;
                }
                field_start = i + 1;
            }
        }

        if field_idx < 10 {
            return;
        }

        // Field 6: Fix quality (0=invalid, 1=GPS, 2=DGPS, etc.)
        if let Ok(q) = fields[6].parse::<u8>() {
            self.last_fix.fix_quality = match q {
                0 => FixQuality::NoFix,
                1 => FixQuality::GpsFix,
                2 => FixQuality::DgpsFix,
                3 => FixQuality::PpsFix,
                4 => FixQuality::RtkFixed,
                5 => FixQuality::RtkFloat,
                6 => FixQuality::DeadReckoning,
                _ => FixQuality::NoFix,
            };
        }

        // Field 7: Number of satellites
        if let Ok(sats) = fields[7].parse::<u8>() {
            self.last_fix.satellites = sats;
        }

        // Field 8: HDOP
        if let Ok(hdop) = parse_f32(fields[8]) {
            self.last_fix.hdop = hdop;
        }

        // Field 9: Altitude (meters above mean sea level)
        if let Ok(alt) = parse_f32(fields[9]) {
            self.last_fix.altitude = alt;
        }
    }

    #[cfg(any(test, feature = "std"))]
    fn parse_gga(&mut self, line: &str) {
        let fields: Vec<&str> = line.split([',', '*']).collect();

        if fields.len() < 10 {
            return;
        }

        // Field 6: Fix quality
        if let Ok(q) = fields[6].parse::<u8>() {
            self.last_fix.fix_quality = match q {
                0 => FixQuality::NoFix,
                1 => FixQuality::GpsFix,
                2 => FixQuality::DgpsFix,
                3 => FixQuality::PpsFix,
                4 => FixQuality::RtkFixed,
                5 => FixQuality::RtkFloat,
                6 => FixQuality::DeadReckoning,
                _ => FixQuality::NoFix,
            };
        }

        // Field 7: Number of satellites
        if let Ok(sats) = fields[7].parse::<u8>() {
            self.last_fix.satellites = sats;
        }

        // Field 8: HDOP
        if let Ok(hdop) = fields[8].parse::<f32>() {
            self.last_fix.hdop = hdop;
        }

        // Field 9: Altitude
        if let Ok(alt) = fields[9].parse::<f32>() {
            self.last_fix.altitude = alt;
        }
    }

    /// Parse GSA sentence for DOP values
    /// Format: $GPGSA,mode,fixType,sv1,sv2,...,sv12,pdop,hdop,vdop*CS
    #[cfg(not(any(test, feature = "std")))]
    fn parse_gsa(&mut self, line: &str) {
        let mut field_idx = 0;
        let mut field_start = 0;
        let mut fields: [&str; 18] = [""; 18];

        for (i, &byte) in line.as_bytes().iter().enumerate() {
            if byte == b',' || byte == b'*' || i == line.len() - 1 {
                if field_idx < 18 {
                    let end = if byte == b',' || byte == b'*' {
                        i
                    } else {
                        i + 1
                    };
                    fields[field_idx] = &line[field_start..end];
                    field_idx += 1;
                }
                field_start = i + 1;
            }
        }

        if field_idx < 17 {
            return;
        }

        // Field 15: PDOP
        if let Ok(pdop) = parse_f32(fields[15]) {
            self.last_fix.pdop = pdop;
        }

        // Field 16: HDOP (also in GGA, but GSA is more authoritative)
        if let Ok(hdop) = parse_f32(fields[16]) {
            self.last_fix.hdop = hdop;
        }

        // Field 17: VDOP
        if let Ok(vdop) = parse_f32(fields[17]) {
            self.last_fix.vdop = vdop;
        }
    }

    #[cfg(any(test, feature = "std"))]
    fn parse_gsa(&mut self, line: &str) {
        let fields: Vec<&str> = line.split([',', '*']).collect();

        if fields.len() < 18 {
            return;
        }

        // Field 15: PDOP
        if let Ok(pdop) = fields[15].parse::<f32>() {
            self.last_fix.pdop = pdop;
        }

        // Field 16: HDOP
        if let Ok(hdop) = fields[16].parse::<f32>() {
            self.last_fix.hdop = hdop;
        }

        // Field 17: VDOP
        if let Ok(vdop) = fields[17].parse::<f32>() {
            self.last_fix.vdop = vdop;
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
            self.last_fix.course = course_deg * (core::f32::consts::PI / 180.0);
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
        self.new_fix_available = true;
    }

    #[cfg(not(any(test, feature = "std")))]
    fn parse_rmc(&mut self, line: &str) {
        // Manual field parsing without Vec allocation
        let mut field_idx = 0;
        let mut field_start = 0;
        let mut fields: [&str; 12] = [""; 12];

        for (i, &byte) in line.as_bytes().iter().enumerate() {
            if byte == b',' || i == line.len() - 1 {
                if field_idx < 12 {
                    let end = if byte == b',' { i } else { i + 1 };
                    fields[field_idx] = &line[field_start..end];
                    field_idx += 1;
                }
                field_start = i + 1;
            }
        }

        if field_idx < 10 {
            return;
        }

        // Parse time (field 1)
        if let Some((h, m, s)) = parse_time(fields[1]) {
            self.last_fix.hour = h;
            self.last_fix.minute = m;
            self.last_fix.second = s;
        }

        // Parse latitude (fields 3, 4)
        let lat = match parse_coordinate(fields[3], fields[4]) {
            Some(lat) => lat,
            None => {
                self.last_fix.valid = false;
                return;
            }
        };

        // Parse longitude (fields 5, 6)
        let lon = match parse_coordinate(fields[5], fields[6]) {
            Some(lon) => lon,
            None => {
                self.last_fix.valid = false;
                return;
            }
        };

        // SANITY CHECK: Reject obviously invalid coordinates
        if lat.abs() < 0.0001 && lon.abs() < 0.0001 {
            // GPS hasn't locked yet, coords are ~(0,0)
            self.last_fix.valid = false;
            return;
        }

        // Parse speed (field 7) - knots to m/s
        if let Ok(speed_knots) = parse_f32(fields[7]) {
            self.last_fix.speed = speed_knots * 0.514444;
        }

        // Parse course (field 8) - degrees to radians
        if let Ok(course_deg) = parse_f32(fields[8]) {
            self.last_fix.course = course_deg * (core::f32::consts::PI / 180.0);
        }

        // Store lat/lon even if fix is invalid (for telemetry)
        self.last_fix.lat = lat;
        self.last_fix.lon = lon;

        // Warmup: average first N fixes for reference
        // ONLY accumulate if we have reasonable coordinates
        if !self.reference.set && lat.abs() > 0.01 && lon.abs() > 0.01 {
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
        }

        // Field 2: Status (A = valid, V = invalid)
        if fields[2] == "A" {
            self.last_fix.valid = true;
            self.new_fix_available = true;
        } else {
            self.last_fix.valid = false;
        }
    }
}

impl Default for NmeaParser {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(not(any(test, feature = "std")))]
fn parse_f32(s: &str) -> Result<f32, ()> {
    if s.is_empty() {
        return Err(());
    }

    let mut result = 0.0f32;
    let mut after_decimal = false;
    let mut decimal_divisor = 1.0f32;
    let mut negative = false;

    for &byte in s.as_bytes() {
        match byte {
            b'-' => negative = true,
            b'0'..=b'9' => {
                let digit = (byte - b'0') as f32;
                if after_decimal {
                    decimal_divisor *= 10.0;
                    result += digit / decimal_divisor;
                } else {
                    result = result * 10.0 + digit;
                }
            }
            b'.' => after_decimal = true,
            _ => return Err(()),
        }
    }

    Ok(if negative { -result } else { result })
}

fn parse_coordinate(coord_str: &str, dir_str: &str) -> Option<f64> {
    if coord_str.is_empty() || dir_str.is_empty() {
        return None;
    }

    // Parse f64 manually (no std::parse available in no_std)
    let value = parse_f64(coord_str)?;

    let degrees = floor(value / 100.0);
    let minutes = value - (degrees * 100.0);
    let mut decimal = degrees + (minutes / 60.0);

    if dir_str == "S" || dir_str == "W" {
        decimal = -decimal;
    }

    Some(decimal)
}

// Add f64 parser
#[cfg(not(any(test, feature = "std")))]
fn parse_f64(s: &str) -> Option<f64> {
    if s.is_empty() {
        return None;
    }

    let mut result = 0.0f64;
    let mut after_decimal = false;
    let mut decimal_divisor = 1.0f64;
    let mut negative = false;

    for &byte in s.as_bytes() {
        match byte {
            b'-' => negative = true,
            b'0'..=b'9' => {
                let digit = (byte - b'0') as f64;
                if after_decimal {
                    decimal_divisor *= 10.0;
                    result += digit / decimal_divisor;
                } else {
                    result = result * 10.0 + digit;
                }
            }
            b'.' => after_decimal = true,
            _ => return None,
        }
    }

    Some(if negative { -result } else { result })
}

#[cfg(any(test, feature = "std"))]
fn parse_f64(s: &str) -> Option<f64> {
    s.parse::<f64>().ok()
}

/// Parse NMEA time field (hhmmss.ss format)
#[allow(dead_code)] // Helper function for future use
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
        assert_eq!(
            parse_coordinate("3723.2475", "S"),
            Some(-37.387458333333336)
        );
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
