//! WT901 9-Axis IMU UART Driver
//!
//! This crate provides a pure Rust driver for the WT901 9-axis IMU sensor.
//! It parses the UART protocol and provides calibrated sensor readings.
//!
//! # Features
//!
//! - Zero-copy UART packet parsing
//! - Automatic checksum verification
//! - Bias calibration support
//! - `no_std` compatible
//! - No external dependencies
//!
//! # Example
//!
//! ```ignore
//! use wt901::{PacketType, Wt901Parser};
//!
//! let mut parser = Wt901Parser::new();
//!
//! // Feed bytes from UART
//! for byte in uart_bytes {
//!     if let Some(packet_type) = parser.feed_byte(byte, timestamp_us) {
//!         match packet_type {
//!             PacketType::Accel => {
//!                 let (ax, ay, az) = parser.get_accel_corrected();
//!                 println!("Accel: {}, {}, {}", ax, ay, az);
//!             }
//!             PacketType::Gyro => {
//!                 let data = parser.data();
//!                 println!("Gyro: {}, {}, {}", data.wx, data.wy, data.wz);
//!             }
//!             PacketType::Angle => {
//!                 let data = parser.data();
//!                 println!("Angles: {}, {}, {}", data.roll, data.pitch, data.yaw);
//!             }
//!         }
//!     }
//! }
//! ```

#![cfg_attr(not(any(test, feature = "std")), no_std)]

#[cfg(feature = "logging")]
use log::warn;

/// Packet constants
const PACKET_SIZE: usize = 11;
const HEADER: u8 = 0x55;

/// Physical constants
const G: f32 = 9.80665; // m/s²

/// WT901 packet types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PacketType {
    /// Accelerometer data (0x51)
    Accel,
    /// Gyroscope data (0x52)
    Gyro,
    /// Euler angles (0x53)
    Angle,
}

impl PacketType {
    fn from_u8(byte: u8) -> Option<Self> {
        match byte {
            0x51 => Some(PacketType::Accel),
            0x52 => Some(PacketType::Gyro),
            0x53 => Some(PacketType::Angle),
            _ => None,
        }
    }

    #[allow(dead_code)] // Helper function for future use
    fn to_u8(self) -> u8 {
        match self {
            PacketType::Accel => 0x51,
            PacketType::Gyro => 0x52,
            PacketType::Angle => 0x53,
        }
    }
}

/// Raw IMU measurements from WT901
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuData {
    /// Accelerometer X-axis (m/s²)
    pub ax: f32,
    /// Accelerometer Y-axis (m/s²)
    pub ay: f32,
    /// Accelerometer Z-axis (m/s²)
    pub az: f32,

    /// Gyroscope X-axis (rad/s)
    pub wx: f32,
    /// Gyroscope Y-axis (rad/s)
    pub wy: f32,
    /// Gyroscope Z-axis (rad/s)
    pub wz: f32,

    /// Roll angle (degrees)
    pub roll: f32,
    /// Pitch angle (degrees)
    pub pitch: f32,
    /// Yaw angle (degrees)
    pub yaw: f32,

    /// Timestamp in microseconds
    pub timestamp_us: u64,
}

/// Accelerometer biases learned during calibration
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuBias {
    /// X-axis bias (m/s²)
    pub ax: f32,
    /// Y-axis bias (m/s²)
    pub ay: f32,
    /// Z-axis bias (m/s²)
    pub az: f32,
}

/// WT901 UART packet parser with state machine
pub struct Wt901Parser {
    buffer: [u8; PACKET_SIZE],
    index: usize,
    synced: bool,
    data: ImuData,
    bias: ImuBias,
}

impl Wt901Parser {
    /// Create a new parser instance
    pub fn new() -> Self {
        Self {
            buffer: [0; PACKET_SIZE],
            index: 0,
            synced: false,
            data: ImuData::default(),
            bias: ImuBias::default(),
        }
    }

    /// Feed a single byte from UART to the parser
    ///
    /// Returns `Some(PacketType)` when a complete valid packet is parsed.
    ///
    /// # Arguments
    ///
    /// * `byte` - The byte read from UART
    /// * `timestamp_us` - Current timestamp in microseconds
    pub fn feed_byte(&mut self, byte: u8, timestamp_us: u64) -> Option<PacketType> {
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
            #[cfg(feature = "logging")]
            warn!(
                "WT901 checksum failed: expected 0x{:02X}, got 0x{:02X}",
                self.buffer[10], checksum
            );
            return None;
        }

        // Parse based on packet type
        let packet_type = PacketType::from_u8(self.buffer[1])?;
        self.data.timestamp_us = timestamp_us;

        match packet_type {
            PacketType::Accel => {
                self.parse_accel();
            }
            PacketType::Gyro => {
                self.parse_gyro();
            }
            PacketType::Angle => {
                self.parse_angle();
            }
        }

        Some(packet_type)
    }

    /// Get reference to current IMU data
    pub fn data(&self) -> &ImuData {
        &self.data
    }

    /// Get mutable reference to current IMU data
    pub fn data_mut(&mut self) -> &mut ImuData {
        &mut self.data
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

    /// Get current calibration biases
    pub fn bias(&self) -> ImuBias {
        self.bias
    }

    /// Parse accelerometer packet (0x51)
    fn parse_accel(&mut self) {
        let ax_raw = i16::from_le_bytes([self.buffer[2], self.buffer[3]]);
        let ay_raw = i16::from_le_bytes([self.buffer[4], self.buffer[5]]);
        let az_raw = i16::from_le_bytes([self.buffer[6], self.buffer[7]]);

        // ±16g range
        self.data.ax = (ax_raw as f32 / 32768.0) * 16.0 * G;
        self.data.ay = (ay_raw as f32 / 32768.0) * 16.0 * G;
        self.data.az = (az_raw as f32 / 32768.0) * 16.0 * G;
    }

    /// Parse gyroscope packet (0x52)
    fn parse_gyro(&mut self) {
        let wx_raw = i16::from_le_bytes([self.buffer[2], self.buffer[3]]);
        let wy_raw = i16::from_le_bytes([self.buffer[4], self.buffer[5]]);
        let wz_raw = i16::from_le_bytes([self.buffer[6], self.buffer[7]]);

        // ±2000°/s range, convert to rad/s
        const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
        self.data.wx = (wx_raw as f32 / 32768.0) * 2000.0 * DEG_TO_RAD;
        self.data.wy = (wy_raw as f32 / 32768.0) * 2000.0 * DEG_TO_RAD;
        self.data.wz = (wz_raw as f32 / 32768.0) * 2000.0 * DEG_TO_RAD;
    }

    /// Parse Euler angles packet (0x53)
    fn parse_angle(&mut self) {
        let roll_raw = i16::from_le_bytes([self.buffer[2], self.buffer[3]]);
        let pitch_raw = i16::from_le_bytes([self.buffer[4], self.buffer[5]]);
        let yaw_raw = i16::from_le_bytes([self.buffer[6], self.buffer[7]]);

        // Convert to degrees
        self.data.roll = (roll_raw as f32 / 32768.0) * 180.0;
        self.data.pitch = (pitch_raw as f32 / 32768.0) * 180.0;
        self.data.yaw = (yaw_raw as f32 / 32768.0) * 180.0;
    }
}

impl Default for Wt901Parser {
    fn default() -> Self {
        Self::new()
    }
}

/// Calibration helper - collects samples for bias computation
///
/// Requires `alloc` feature (enabled by default in std environments)
#[cfg(any(feature = "std", test))]
use std::vec::Vec;

#[cfg(any(feature = "std", test))]
pub struct ImuCalibrator {
    samples_ax: Vec<f32>,
    samples_ay: Vec<f32>,
    samples_az: Vec<f32>,
    target_samples: usize,
}

#[cfg(any(feature = "std", test))]
impl ImuCalibrator {
    /// Create a new calibrator
    ///
    /// # Arguments
    ///
    /// * `target_samples` - Number of samples to collect before computing bias
    pub fn new(target_samples: usize) -> Self {
        Self {
            samples_ax: Vec::with_capacity(target_samples),
            samples_ay: Vec::with_capacity(target_samples),
            samples_az: Vec::with_capacity(target_samples),
            target_samples,
        }
    }

    /// Add a sample to the calibration
    ///
    /// Samples should be taken while the IMU is stationary and level.
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

    /// Get calibration progress as a value from 0.0 to 1.0
    pub fn progress(&self) -> f32 {
        self.samples_ax.len() as f32 / self.target_samples as f32
    }

    /// Compute the bias from collected samples
    ///
    /// Returns `None` if calibration is not complete.
    /// Assumes the sensor is stationary and level (gravity on Z-axis).
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
            az: sum_az / n - G, // Remove gravity from Z axis
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_packet_type_conversion() {
        assert_eq!(PacketType::from_u8(0x51), Some(PacketType::Accel));
        assert_eq!(PacketType::from_u8(0x52), Some(PacketType::Gyro));
        assert_eq!(PacketType::from_u8(0x53), Some(PacketType::Angle));
        assert_eq!(PacketType::from_u8(0xFF), None);

        assert_eq!(PacketType::Accel.to_u8(), 0x51);
        assert_eq!(PacketType::Gyro.to_u8(), 0x52);
        assert_eq!(PacketType::Angle.to_u8(), 0x53);
    }

    #[test]
    fn test_parser_creation() {
        let parser = Wt901Parser::new();
        let data = parser.data();
        assert_eq!(data.ax, 0.0);
        assert_eq!(data.timestamp_us, 0);
    }

    #[test]
    fn test_calibrator() {
        let mut cal = ImuCalibrator::new(10);
        assert!(!cal.is_complete());
        assert_eq!(cal.progress(), 0.0);

        for i in 0..5 {
            cal.add_sample(0.1, 0.2, G + 0.3);
            assert_eq!(cal.progress(), (i + 1) as f32 / 10.0);
        }

        assert!(!cal.is_complete());
        assert!(cal.compute_bias().is_none());

        for _ in 0..5 {
            cal.add_sample(0.1, 0.2, G + 0.3);
        }

        assert!(cal.is_complete());
        let bias = cal.compute_bias().unwrap();
        assert!((bias.ax - 0.1).abs() < 0.01);
        assert!((bias.ay - 0.2).abs() < 0.01);
        assert!((bias.az - 0.3).abs() < 0.01);
    }
}
