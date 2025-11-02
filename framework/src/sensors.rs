/// Sensor abstraction layer for hardware independence
/// Enables testing, simulation, and support for multiple sensor types
/// IMU sensor data (accelerometer + gyroscope + magnetometer)
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuData {
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    pub wx: f32,
    pub wy: f32,
    pub wz: f32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

/// GPS fix data
#[derive(Debug, Clone, Copy)]
pub struct GpsFix {
    pub valid: bool,
    pub lat: f64,
    pub lon: f64,
    pub alt: f32,
    pub speed: f32,
    pub course: f32,
    pub hdop: f32,
    pub satellites: u8,
}

impl Default for GpsFix {
    fn default() -> Self {
        Self {
            valid: false,
            lat: 0.0,
            lon: 0.0,
            alt: 0.0,
            speed: 0.0,
            course: 0.0,
            hdop: 99.9,
            satellites: 0,
        }
    }
}

/// Errors that can occur when reading sensors
#[derive(Debug, Clone, Copy)]
pub enum SensorError {
    IoError,
    ParseError,
    NotReady,
    Timeout,
}

/// Abstraction for IMU sensors (accelerometer + gyroscope + magnetometer)
/// Implementations: WT901, MPU6050, BMI088, etc.
pub trait ImuSensor {
    /// Poll for new IMU data
    /// Returns None if no new data available
    fn poll(&mut self) -> Result<Option<ImuData>, SensorError>;

    /// Get current IMU state (last reading)
    fn get_data(&self) -> &ImuData;

    /// Get bias-corrected accelerometer readings
    fn get_accel_corrected(&self) -> (f32, f32, f32);
}

/// Core abstraction for GPS receivers (Interface Segregation Principle)
/// Implementations: NEO-6M (NMEA), u-blox (UBX), etc.
/// This trait focuses on essential GPS reading operations only
pub trait GpsSensor {
    /// Poll for new GPS data
    /// Returns true if new fix was processed
    fn poll(&mut self) -> Result<bool, SensorError>;

    /// Get current GPS fix
    fn get_fix(&self) -> &GpsFix;

    /// Check if GPS is warmed up and ready
    fn is_ready(&self) -> bool;
}

/// Optional trait for GPS coordinate transformations
/// Implement this for GPS sensors that maintain a local reference frame
pub trait GpsCoordinateTransform: GpsSensor {
    /// Get local coordinates relative to reference point
    /// Returns None if reference is not set or GPS not ready
    fn to_local_coords(&self) -> Option<(f32, f32)>;

    /// Get velocity in ENU (East-North-Up) frame
    /// Returns None if velocity data not available
    fn get_velocity_enu(&self) -> Option<(f32, f32)>;
}

/// Optional trait for GPS-derived speed estimation
/// Implement this for sensors that can estimate speed from position changes
pub trait GpsSpeedEstimator: GpsSensor {
    /// Get position-based speed estimate (m/s)
    /// Calculated from successive position measurements
    fn position_based_speed(&self) -> f32;
}

/// Combined sensor reading for state estimation
#[derive(Debug, Clone, Copy)]
pub struct SensorReading {
    pub timestamp_us: u64,
    pub imu: ImuData,
    pub gps: GpsFix,
    pub gps_updated: bool,
}
