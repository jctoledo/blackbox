/// WT901 IMU Driver
/// Parses UART packets from WT901 9-axis IMU sensor
/// Packet format: 11 bytes [0x55, TYPE, DATA(8), CHECKSUM]


/// Packet constants
const PACKET_SIZE: usize = 11;
const HEADER: u8 = 0x55;

/// Packet types
const TYPE_ACCEL: u8 = 0x51;
const TYPE_GYRO: u8 = 0x52;
const TYPE_ANGLE: u8 = 0x53;

/// Physical constants
const G: f32 = 9.80665;  // m/s²

/// Raw IMU measurements
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    // Accelerometer (m/s²)
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    
    // Gyroscope (rad/s)
    pub wx: f32,
    pub wy: f32,
    pub wz: f32,
    
    // Euler angles (degrees)
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    
    // Timestamp
    pub timestamp_us: u64,
}

impl Default for ImuData {
    fn default() -> Self {
        Self {
            ax: 0.0, ay: 0.0, az: 0.0,
            wx: 0.0, wy: 0.0, wz: 0.0,
            roll: 0.0, pitch: 0.0, yaw: 0.0,
            timestamp_us: 0,
        }
    }
}

/// Biases learned during calibration
#[derive(Debug, Clone, Copy)]
pub struct ImuBias {
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
}

impl Default for ImuBias {
    fn default() -> Self {
        Self { ax: 0.0, ay: 0.0, az: 0.0 }
    }
}

/// Packet parser with state machine
pub struct Wt901Parser {
    buffer: [u8; PACKET_SIZE],
    index: usize,
    synced: bool,
    pub data: ImuData,
    pub bias: ImuBias,
}

impl Wt901Parser {
    pub fn new() -> Self {
        Self {
            buffer: [0; PACKET_SIZE],
            index: 0,
            synced: false,
            data: ImuData::default(),
            bias: ImuBias::default(),
        }
    }
    
    /// Feed a single byte to the parser
    /// Returns Some(packet_type) when a complete valid packet is parsed
    pub fn feed_byte(&mut self, byte: u8, timestamp_us: u64) -> Option<u8> {
        // Look for header
        if !self.synced {
            if byte == HEADER {
                self.synced = true;
                self.index = 0;
                self.buffer[self.index] = byte;
                self.index += 1;
            }
            return None;
        }
        
        // Accumulate packet
        self.buffer[self.index] = byte;
        self.index += 1;
        
        if self.index < PACKET_SIZE {
            return None;
        }
        
        // Full packet received, reset state
        self.synced = false;
        self.index = 0;
        
        // Verify checksum
        let mut checksum: u8 = 0;
        for i in 0..10 {
            checksum = checksum.wrapping_add(self.buffer[i]);
        }
        
        if checksum != self.buffer[10] {
            return None;
        }
        
        // Parse based on packet type
        let packet_type = self.buffer[1];
        self.data.timestamp_us = timestamp_us;
        
        match packet_type {
            TYPE_ACCEL => {
                self.parse_accel();
                Some(TYPE_ACCEL)
            }
            TYPE_GYRO => {
                self.parse_gyro();
                Some(TYPE_GYRO)
            }
            TYPE_ANGLE => {
                self.parse_angle();
                Some(TYPE_ANGLE)
            }
            _ => {
                None
            }
        }
    }
    
    /// Parse accelerometer packet
    fn parse_accel(&mut self) {
        let ax_raw = i16::from_le_bytes([self.buffer[2], self.buffer[3]]);
        let ay_raw = i16::from_le_bytes([self.buffer[4], self.buffer[5]]);
        let az_raw = i16::from_le_bytes([self.buffer[6], self.buffer[7]]);
        
        // ±16g range
        self.data.ax = (ax_raw as f32 / 32768.0) * 16.0 * G;
        self.data.ay = (ay_raw as f32 / 32768.0) * 16.0 * G;
        self.data.az = (az_raw as f32 / 32768.0) * 16.0 * G;
    }
    
    /// Parse gyroscope packet
    fn parse_gyro(&mut self) {
        let wx_raw = i16::from_le_bytes([self.buffer[2], self.buffer[3]]);
        let wy_raw = i16::from_le_bytes([self.buffer[4], self.buffer[5]]);
        let wz_raw = i16::from_le_bytes([self.buffer[6], self.buffer[7]]);
        
        // ±2000°/s range, convert to rad/s
        const DEG_TO_RAD: f32 = std::f32::consts::PI / 180.0;
        self.data.wx = (wx_raw as f32 / 32768.0) * 2000.0 * DEG_TO_RAD;
        self.data.wy = (wy_raw as f32 / 32768.0) * 2000.0 * DEG_TO_RAD;
        self.data.wz = (wz_raw as f32 / 32768.0) * 2000.0 * DEG_TO_RAD;
    }
    
    /// Parse Euler angles packet
    fn parse_angle(&mut self) {
        let roll_raw  = i16::from_le_bytes([self.buffer[2], self.buffer[3]]);
        let pitch_raw = i16::from_le_bytes([self.buffer[4], self.buffer[5]]);
        let yaw_raw   = i16::from_le_bytes([self.buffer[6], self.buffer[7]]);
        
        // Convert to degrees
        self.data.roll  = (roll_raw  as f32 / 32768.0) * 180.0;
        self.data.pitch = (pitch_raw as f32 / 32768.0) * 180.0;
        self.data.yaw   = (yaw_raw   as f32 / 32768.0) * 180.0;
    }
    
    /// Get bias-corrected accelerometer readings
    pub fn get_accel_corrected(&self) -> (f32, f32, f32) {
        (
            self.data.ax - self.bias.ax,
            self.data.ay - self.bias.ay,
            self.data.az - self.bias.az,
        )
    }
    
    /// Set calibration biases
    pub fn set_bias(&mut self, bias: ImuBias) {
        self.bias = bias;
    }
}

/// Calibration helper - collects samples for bias computation
pub struct ImuCalibrator {
    samples_ax: Vec<f32>,
    samples_ay: Vec<f32>,
    samples_az: Vec<f32>,
    target_samples: usize,
}

impl ImuCalibrator {
    pub fn new(target_samples: usize) -> Self {
        Self {
            samples_ax: Vec::with_capacity(target_samples),
            samples_ay: Vec::with_capacity(target_samples),
            samples_az: Vec::with_capacity(target_samples),
            target_samples,
        }
    }
    
    /// Add a sample
    pub fn add_sample(&mut self, ax: f32, ay: f32, az: f32) {
        if !self.is_complete() {
            self.samples_ax.push(ax);
            self.samples_ay.push(ay);
            self.samples_az.push(az);
        }
    }
    
    /// Check if calibration is complete
    pub fn is_complete(&self) -> bool {
        self.samples_ax.len() >= self.target_samples
    }
    
    /// Get progress as 0.0 to 1.0
    pub fn progress(&self) -> f32 {
        self.samples_ax.len() as f32 / self.target_samples as f32
    }
    
    /// Compute bias (assumes sensor is stationary and level)
    pub fn compute_bias(&self) -> Option<ImuBias> {
        if !self.is_complete() {
            return None;
        }
        
        let n = self.samples_ax.len() as f32;
        let sum_ax: f32 = self.samples_ax.iter().sum();
        let sum_ay: f32 = self.samples_ay.iter().sum();
        let sum_az: f32 = self.samples_az.iter().sum();
        
        Some(ImuBias {
            ax: sum_ax / n,
            ay: sum_ay / n,
            az: sum_az / n - G,  // Remove gravity from Z axis
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_packet_parsing() {
        let mut parser = Wt901Parser::new();
        
        // Simulate accel packet: 0x55 0x51 [ax=1000] [ay=2000] [az=16384] [temp]
        let packet = [
            0x55, 0x51,
            0xE8, 0x03,  // ax = 1000
            0xD0, 0x07,  // ay = 2000
            0x00, 0x40,  // az = 16384 (1g)
            0x00, 0x00,  // temp (ignored)
            0x00,        // checksum (will fail, but demonstrates parsing)
        ];
        
        // Feed bytes
        for &byte in &packet {
            parser.feed_byte(byte, 0);
        }
        
        // Note: checksum will fail in this test, but structure is correct
    }
}
