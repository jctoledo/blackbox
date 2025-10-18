/// NEO-6M GPS NMEA Parser
/// Parses GPRMC/GNRMC sentences for position and velocity

/// GPS fix data
#[derive(Debug, Clone, Copy)]
pub struct GpsFix {
    pub lat: f64,
    pub lon: f64,
    pub speed: f32,
    pub course: f32,
    pub valid: bool,
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

pub struct NmeaParser {
    line_buffer: [u8; 120],  // Fixed size array
    line_len: usize,
    pub last_fix: GpsFix,
    pub reference: GpsReference,
    warmup_count: u8,
    warmup_lat_sum: f64,
    warmup_lon_sum: f64,
    last_valid_lat: f64,
    last_valid_lon: f64,
    last_valid_time_ms: u32,
    pub position_based_speed: f32,
}

const GPS_WARMUP_FIXES: u8 = 5;

impl NmeaParser {
    pub fn new() -> Self {
        Self {
            line_buffer: [0; 120],
            line_len: 0,
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
    
    pub fn feed_byte(&mut self, byte: u8) -> bool {
        match byte {
            b'$' => {
                self.line_len = 0;
                self.line_buffer[self.line_len] = byte;
                self.line_len += 1;
                false
            }
            b'\r' => {
                false
            }
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
    
    fn parse_line(&mut self) {
        // Copy the line to a local buffer to avoid borrow checker issues
        let mut local_buf = [0u8; 120];
        let len = self.line_len;
        local_buf[..len].copy_from_slice(&self.line_buffer[..len]);
        
        let line = match core::str::from_utf8(&local_buf[..len]) {
            Ok(s) => s,
            Err(_) => return,
        };
        
        if line.starts_with("$GPRMC") || line.starts_with("$GNRMC") {
            self.parse_rmc(line);
        }
    }
    
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
            self.last_fix.speed = speed_knots * 0.514444;
        }
        
        if let Ok(course_deg) = fields[8].parse::<f32>() {
            self.last_fix.course = course_deg.to_radians();
        }
        
        if !self.reference.set {
            self.warmup_count += 1;
            self.warmup_lat_sum += lat;
            self.warmup_lon_sum += lon;
            
            if self.warmup_count >= GPS_WARMUP_FIXES {
                self.reference.lat = self.warmup_lat_sum / self.warmup_count as f64;
                self.reference.lon = self.warmup_lon_sum / self.warmup_count as f64;
                self.reference.set = true;
                
                self.last_valid_lat = lat;
                self.last_valid_lon = lon;
                self.last_valid_time_ms = unsafe { 
                    (esp_idf_svc::sys::esp_timer_get_time() / 1000) as u32 
                };
            }
            
            self.last_fix.valid = false;
            return;
        }
        
        let now_ms = unsafe { (esp_idf_svc::sys::esp_timer_get_time() / 1000) as u32 };
        let dt = (now_ms - self.last_valid_time_ms) as f32 / 1000.0;
        
        if dt > 0.1 && dt < 2.0 {
            let dlat = lat - self.last_valid_lat;
            let dlon = lon - self.last_valid_lon;
            
            let dy = (dlat * 111320.0) as f32;
            let dx = (dlon * 111320.0 * self.reference.lat.to_radians().cos()) as f32;
            let dist = (dx * dx + dy * dy).sqrt();
            
            self.position_based_speed = dist / dt;
            
            if self.position_based_speed < 0.3 && self.last_fix.speed > 0.5 {
                self.last_fix.speed = self.position_based_speed;
            }
        }
        
        self.last_valid_lat = lat;
        self.last_valid_lon = lon;
        self.last_valid_time_ms = now_ms;
        
        self.last_fix.lat = lat;
        self.last_fix.lon = lon;
        self.last_fix.valid = true;
    }
    
    pub fn to_local_coords(&self) -> Option<(f32, f32)> {
        if !self.reference.set || !self.last_fix.valid {
            return None;
        }
        
        let dlat = self.last_fix.lat - self.reference.lat;
        let dlon = self.last_fix.lon - self.reference.lon;
        
        let north = (dlat * 111320.0) as f32;
        let east = (dlon * 111320.0 * self.reference.lat.to_radians().cos()) as f32;
        
        Some((east, north))
    }
    
    pub fn get_velocity_enu(&self) -> Option<(f32, f32)> {
        if !self.last_fix.valid {
            return None;
        }
        
        let vx = self.last_fix.speed * self.last_fix.course.cos();
        let vy = self.last_fix.speed * self.last_fix.course.sin();
        
        Some((vx, vy))
    }
    
    #[allow(dead_code)]
    pub fn get_local_time(&self, tz_offset: i8) -> (u8, u8, u8) {
        let hour = ((self.last_fix.hour as i8 + tz_offset + 24) % 24) as u8;
        (hour, self.last_fix.minute, self.last_fix.second)
    }
    
    pub fn is_warmed_up(&self) -> bool {
        self.reference.set
    }
    
    pub fn warmup_progress(&self) -> f32 {
        if self.reference.set {
            1.0
        } else {
            self.warmup_count as f32 / GPS_WARMUP_FIXES as f32
        }
    }
}

fn parse_coordinate(coord_str: &str, dir_str: &str) -> Option<f64> {
    if coord_str.is_empty() || dir_str.is_empty() {
        return None;
    }
    
    let value = coord_str.parse::<f64>().ok()?;
    
    let degrees = (value / 100.0).floor();
    let minutes = value - (degrees * 100.0);
    
    let mut decimal = degrees + (minutes / 60.0);
    
    if dir_str == "S" || dir_str == "W" {
        decimal = -decimal;
    }
    
    Some(decimal)
}

fn parse_time(time_str: &str) -> Option<(u8, u8, u8)> {
    if time_str.len() < 6 {
        return None;
    }
    
    let hh = time_str[0..2].parse::<u8>().ok()?;
    let mm = time_str[2..4].parse::<u8>().ok()?;
    let ss = time_str[4..6].parse::<u8>().ok()?;
    
    Some((hh, mm, ss))
}

#[allow(dead_code)]
pub fn lat_to_meters(dlat: f64) -> f32 {
    (dlat * 111320.0) as f32
}

#[allow(dead_code)]
pub fn lon_to_meters(dlon: f64, ref_lat: f64) -> f32 {
    (dlon * 111320.0 * ref_lat.to_radians().cos()) as f32
}
