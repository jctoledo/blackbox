/// NEO-6M GPS NMEA Parser
/// Parses GPRMC/GNRMC sentences for position and velocity


/// GPS fix data
#[derive(Debug, Clone, Copy)]
pub struct GpsFix {
    pub lat: f64,           // degrees
    pub lon: f64,           // degrees
    pub speed: f32,         // m/s
    pub course: f32,        // radians (0 = North, clockwise)
    pub valid: bool,
    
    // Time (UTC)
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

impl Default for GpsFix {
    fn default() -> Self {
        Self {
            lat: 0.0,
            lon: 0.0,
            speed: 0.0,
            course: 0.0,
            valid: false,
            hour: 0,
            minute: 0,
            second: 0,
        }
    }
}

/// Reference point for local coordinate conversion
#[derive(Debug, Clone, Copy)]
pub struct GpsReference {
    pub lat: f64,
    pub lon: f64,
    pub set: bool,
}

impl Default for GpsReference {
    fn default() -> Self {
        Self {
            lat: 0.0,
            lon: 0.0,
            set: false,
        }
    }
}

/// NMEA sentence parser
pub struct NmeaParser {
    line_buffer: Vec<u8>,
    pub last_fix: GpsFix,
    pub reference: GpsReference,
    // GPS warmup state
    warmup_count: u8,
    warmup_lat_sum: f64,
    warmup_lon_sum: f64,
    // Position-based speed calculation
    last_valid_lat: f64,
    last_valid_lon: f64,
    last_valid_time_ms: u32,
    pub position_based_speed: f32,
}

const GPS_WARMUP_FIXES: u8 = 5;

impl NmeaParser {
    pub fn new() -> Self {
        Self {
            line_buffer: Vec::with_capacity(120),
            last_fix: GpsFix::default(),
            reference: GpsReference::default(),
            warmup_count: 0,
            warmup_lat_sum: 0.0,
            warmup_lon_sum: 0.0,
            last_valid_lat: 0.0,
            last_valid_lon: 0.0,
            last_valid_time_ms: 0,
            position_based_speed: 0.0,
        }
    }
    
    /// Feed a byte from GPS UART
    /// Returns true when a complete sentence is ready
    pub fn feed_byte(&mut self, byte: u8) -> bool {
        match byte {
            b'$' => {
                // Start of new sentence
                self.line_buffer.clear();
                self.line_buffer.push(byte);
                false
            }
            b'\r' => {
                // Ignore CR
                false
            }
            b'\n' => {
                // End of sentence - parse it
                if self.line_buffer.len() > 0 {
                    self.parse_line();
                    true
                } else {
                    false
                }
            }
            _ => {
                // Accumulate
                if self.line_buffer.len() < 120 {
                    self.line_buffer.push(byte);
                }
                false
            }
        }
    }
    
    /// Parse accumulated line
    fn parse_line(&mut self) {
        // Convert to string and clone it
        let line = match core::str::from_utf8(&self.line_buffer) {
            Ok(s) => s.to_string(),
            Err(_) => return,
        };
        
        // Check for RMC sentence (Recommended Minimum)
        if line.starts_with("$GPRMC") || line.starts_with("$GNRMC") {
            self.parse_rmc(&line);
        }
    }
    
    /// Parse RMC sentence
    /// Format: $GPRMC,hhmmss.ss,A,ddmm.mmmm,N,dddmm.mmmm,E,speed,course,date,,,*checksum
    fn parse_rmc(&mut self, line: &str) {
        let fields: Vec<&str> = line.split(',').collect();
        
        if fields.len() < 10 {
            return;
        }
        
        // Field 2: Status (A=valid, V=invalid)
        if fields[2] != "A" {
            self.last_fix.valid = false;
            return;
        }
        
        // Field 1: UTC time (hhmmss.ss)
        if let Some((h, m, s)) = parse_time(fields[1]) {
            self.last_fix.hour = h;
            self.last_fix.minute = m;
            self.last_fix.second = s;
        }
        
        // Field 3-4: Latitude (ddmm.mmmm, N/S)
        let lat = match parse_coordinate(fields[3], fields[4]) {
            Some(lat) => lat,
            None => return,
        };
        
        // Field 5-6: Longitude (dddmm.mmmm, E/W)
        let lon = match parse_coordinate(fields[5], fields[6]) {
            Some(lon) => lon,
            None => return,
        };
        
        // Field 7: Speed over ground (knots)
        if let Ok(speed_knots) = fields[7].parse::<f32>() {
            self.last_fix.speed = speed_knots * 0.514444;  // knots to m/s
        }
        
        // Field 8: Course over ground (degrees)
        if let Ok(course_deg) = fields[8].parse::<f32>() {
            self.last_fix.course = course_deg.to_radians();
        }
        
        // GPS warmup: collect first N fixes and average them
        if !self.reference.set {
            self.warmup_count += 1;
            self.warmup_lat_sum += lat;
            self.warmup_lon_sum += lon;
            
            if self.warmup_count >= GPS_WARMUP_FIXES {
                // Average the warmup fixes to get a stable reference
                self.reference.lat = self.warmup_lat_sum / self.warmup_count as f64;
                self.reference.lon = self.warmup_lon_sum / self.warmup_count as f64;
                self.reference.set = true;
                
                // Initialize position tracking
                self.last_valid_lat = lat;
                self.last_valid_lon = lon;
                self.last_valid_time_ms = unsafe { 
                    (esp_idf_svc::sys::esp_timer_get_time() / 1000) as u32 
                };
            }
            
            // Don't update last_fix position until reference is set
            // This prevents phantom velocity readings from bad first fixes
            self.last_fix.valid = false;
            return;
        }
        
        // Calculate position-based speed to detect stopped motion
        let now_ms = unsafe { (esp_idf_svc::sys::esp_timer_get_time() / 1000) as u32 };
        let dt = (now_ms - self.last_valid_time_ms) as f32 / 1000.0;
        
        if dt > 0.1 && dt < 2.0 {  // Reasonable time delta (0.1s to 2s)
            let dlat = lat - self.last_valid_lat;
            let dlon = lon - self.last_valid_lon;
            
            // Convert to meters
            let dy = (dlat * 111320.0) as f32;
            let dx = (dlon * 111320.0 * self.reference.lat.to_radians().cos()) as f32;
            let dist = (dx * dx + dy * dy).sqrt();
            
            self.position_based_speed = dist / dt;
            
            // Override GPS speed if position says we're stopped
            // This handles GPS Doppler lag when coming to a stop
            if self.position_based_speed < 0.3 && self.last_fix.speed > 0.5 {
                self.last_fix.speed = self.position_based_speed;
            }
        }
        
        // Update tracking variables
        self.last_valid_lat = lat;
        self.last_valid_lon = lon;
        self.last_valid_time_ms = now_ms;
        
        // Reference is set, now we can process positions normally
        self.last_fix.lat = lat;
        self.last_fix.lon = lon;
        self.last_fix.valid = true;
    }
    
    /// Convert GPS position to local ENU coordinates (meters)
    /// Returns (east, north) relative to reference point
    pub fn to_local_coords(&self) -> Option<(f32, f32)> {
        if !self.reference.set || !self.last_fix.valid {
            return None;
        }
        
        let dlat = self.last_fix.lat - self.reference.lat;
        let dlon = self.last_fix.lon - self.reference.lon;
        
        // Simple flat-earth approximation (good for < 10 km)
        let north = (dlat * 111320.0) as f32;  // meters
        let east = (dlon * 111320.0 * self.reference.lat.to_radians().cos()) as f32;
        
        Some((east, north))
    }
    
    /// Get velocity in local ENU frame
    pub fn get_velocity_enu(&self) -> Option<(f32, f32)> {
        if !self.last_fix.valid {
            return None;
        }
        
        let vx = self.last_fix.speed * self.last_fix.course.cos();
        let vy = self.last_fix.speed * self.last_fix.course.sin();
        
        Some((vx, vy))
    }
    
    /// Apply timezone offset to UTC time
    #[allow(dead_code)]
    pub fn get_local_time(&self, tz_offset: i8) -> (u8, u8, u8) {
        let hour = ((self.last_fix.hour as i8 + tz_offset + 24) % 24) as u8;
        (hour, self.last_fix.minute, self.last_fix.second)
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
            self.warmup_count as f32 / GPS_WARMUP_FIXES as f32
        }
    }
}

/// Parse NMEA coordinate field (ddmm.mmmm or dddmm.mmmm)
fn parse_coordinate(coord_str: &str, dir_str: &str) -> Option<f64> {
    if coord_str.is_empty() || dir_str.is_empty() {
        return None;
    }
    
    let value = coord_str.parse::<f64>().ok()?;
    
    // Split degrees and minutes
    let degrees = (value / 100.0).floor();
    let minutes = value - (degrees * 100.0);
    
    let mut decimal = degrees + (minutes / 60.0);
    
    // Apply hemisphere
    if dir_str == "S" || dir_str == "W" {
        decimal = -decimal;
    }
    
    Some(decimal)
}

/// Parse NMEA time field (hhmmss.ss)
fn parse_time(time_str: &str) -> Option<(u8, u8, u8)> {
    if time_str.len() < 6 {
        return None;
    }
    
    let hh = time_str[0..2].parse::<u8>().ok()?;
    let mm = time_str[2..4].parse::<u8>().ok()?;
    let ss = time_str[4..6].parse::<u8>().ok()?;
    
    Some((hh, mm, ss))
}

/// Helper: Convert lat/lon difference to meters
#[allow(dead_code)]
pub fn lat_to_meters(dlat: f64) -> f32 {
    (dlat * 111320.0) as f32
}

#[allow(dead_code)]
pub fn lon_to_meters(dlon: f64, ref_lat: f64) -> f32 {
    (dlon * 111320.0 * ref_lat.to_radians().cos()) as f32
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_coordinate_parsing() {
        // Test latitude: 3723.2475,N = 37.387458°N
        let lat = parse_coordinate("3723.2475", "N").unwrap();
        assert!((lat - 37.387458).abs() < 0.001);
        
        // Test longitude: 12158.3416,W = 121.972360°W
        let lon = parse_coordinate("12158.3416", "W").unwrap();
        assert!((lon - (-121.972360)).abs() < 0.001);
    }
    
    #[test]
    fn test_time_parsing() {
        let (h, m, s) = parse_time("123456").unwrap();
        assert_eq!(h, 12);
        assert_eq!(m, 34);
        assert_eq!(s, 56);
    }
}
