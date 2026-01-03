//! Mount calibration for determining device orientation relative to vehicle
//!
//! The device can be mounted in any horizontal orientation. This module
//! determines the yaw offset between the device's forward axis and the
//! vehicle's forward axis using a simple brake calibration maneuver.
//!
//! The insight: when braking hard in a straight line, the acceleration
//! vector points directly backward relative to the vehicle. The angle
//! of this vector in the device's body frame *is* the mounting offset.
//!
//! No GPS required. No magnetometer. No EKF. Just physics.

#![allow(dead_code)] // Some methods reserved for future use

use core::f32::consts::PI;

/// Calibration configuration thresholds
#[derive(Debug, Clone, Copy)]
pub struct CalibrationConfig {
    /// Minimum deceleration to detect braking (m/s²)
    pub min_decel: f32,
    /// Maximum yaw rate during braking to ensure straight line (rad/s)
    pub max_yaw_rate: f32,
    /// Minimum speed before we start looking for braking (m/s)
    pub min_speed: f32,
    /// Duration to collect samples during braking (seconds)
    pub sample_duration: f32,
    /// Maximum angle variance in samples (radians)
    pub max_angle_variance: f32,
    /// Timeout waiting for motion (seconds)
    pub motion_timeout: f32,
    /// Timeout waiting for brake (seconds)
    pub brake_timeout: f32,
}

impl Default for CalibrationConfig {
    fn default() -> Self {
        Self {
            min_decel: 2.5,            // ~0.25g - moderate braking
            max_yaw_rate: 0.15,        // ~8.6°/s - reasonably straight
            min_speed: 3.0,            // ~11 km/h - clearly moving forward
            sample_duration: 0.5,      // Half second of samples
            max_angle_variance: 0.175, // ~10° - samples should agree
            motion_timeout: 30.0,      // 30 seconds to start moving
            brake_timeout: 30.0,       // 30 seconds to brake after moving
        }
    }
}

/// Persistent mount calibration data
#[derive(Debug, Clone, Copy)]
pub struct MountCalibration {
    /// Yaw offset from device frame to vehicle frame (radians)
    /// To convert device heading to vehicle heading: vehicle_yaw = device_yaw - offset
    pub yaw_offset: f32,
    /// Whether calibration has been performed
    pub is_calibrated: bool,
}

impl Default for MountCalibration {
    fn default() -> Self {
        Self {
            yaw_offset: 0.0,
            is_calibrated: false,
        }
    }
}

impl MountCalibration {
    /// Create a new calibration with a known offset
    pub fn new(yaw_offset: f32) -> Self {
        Self {
            yaw_offset: normalize_angle(yaw_offset),
            is_calibrated: true,
        }
    }

    /// Apply the calibration offset to a device-frame yaw to get vehicle-frame yaw
    pub fn device_to_vehicle_yaw(&self, device_yaw: f32) -> f32 {
        if self.is_calibrated {
            normalize_angle(device_yaw - self.yaw_offset)
        } else {
            device_yaw
        }
    }

    /// Apply the calibration to rotate a 2D vector from device frame to vehicle frame
    pub fn device_to_vehicle_accel(&self, ax_device: f32, ay_device: f32) -> (f32, f32) {
        if !self.is_calibrated {
            return (ax_device, ay_device);
        }

        let cos_off = self.yaw_offset.cos();
        let sin_off = self.yaw_offset.sin();

        // Rotate by negative offset (device → vehicle)
        let ax_vehicle = ax_device * cos_off + ay_device * sin_off;
        let ay_vehicle = -ax_device * sin_off + ay_device * cos_off;

        (ax_vehicle, ay_vehicle)
    }
}

/// Calibration state machine states
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CalibrationState {
    /// Not calibrating, waiting for trigger
    Idle,
    /// Waiting for vehicle to start moving
    WaitingForMotion,
    /// Vehicle moving, waiting for hard brake
    WaitingForBrake,
    /// Collecting acceleration samples during braking
    Collecting,
    /// Calibration completed successfully
    Complete,
    /// Calibration failed
    Failed,
}

/// Result of calibration attempt
#[derive(Debug, Clone)]
pub enum CalibrationResult {
    /// Still in progress
    InProgress(CalibrationState),
    /// Completed successfully with computed offset
    Success(MountCalibration),
    /// Failed with reason
    Failure(CalibrationError),
}

/// Calibration failure reasons
#[derive(Debug, Clone, Copy)]
pub enum CalibrationError {
    /// Timed out waiting for vehicle motion
    MotionTimeout,
    /// Timed out waiting for braking
    BrakeTimeout,
    /// Samples were too inconsistent (user turned while braking?)
    InconsistentSamples,
    /// Not enough valid samples collected
    InsufficientSamples,
    /// Calibration was cancelled
    Cancelled,
}

/// Collected sample during braking
#[derive(Debug, Clone, Copy)]
struct BrakeSample {
    /// Body-frame X acceleration (gravity removed)
    ax: f32,
    /// Body-frame Y acceleration (gravity removed)
    ay: f32,
    /// Timestamp in milliseconds
    timestamp_ms: u32,
}

/// The calibration engine
pub struct MountCalibrator {
    config: CalibrationConfig,
    state: CalibrationState,
    state_start_ms: u32,
    samples: [BrakeSample; 64],
    sample_count: usize,
    computed_offset: Option<f32>,
}

impl MountCalibrator {
    pub fn new(config: CalibrationConfig) -> Self {
        Self {
            config,
            state: CalibrationState::Idle,
            state_start_ms: 0,
            samples: [BrakeSample {
                ax: 0.0,
                ay: 0.0,
                timestamp_ms: 0,
            }; 64],
            sample_count: 0,
            computed_offset: None,
        }
    }

    /// Get current calibration state
    pub fn state(&self) -> CalibrationState {
        self.state
    }

    /// Check if calibration is in progress
    pub fn is_active(&self) -> bool {
        matches!(
            self.state,
            CalibrationState::WaitingForMotion
                | CalibrationState::WaitingForBrake
                | CalibrationState::Collecting
        )
    }

    /// Trigger calibration to begin
    pub fn trigger(&mut self, now_ms: u32) {
        self.state = CalibrationState::WaitingForMotion;
        self.state_start_ms = now_ms;
        self.sample_count = 0;
        self.computed_offset = None;
    }

    /// Cancel calibration in progress
    pub fn cancel(&mut self) {
        self.state = CalibrationState::Failed;
        self.sample_count = 0;
    }

    /// Reset to idle state
    pub fn reset(&mut self) {
        self.state = CalibrationState::Idle;
        self.sample_count = 0;
        self.computed_offset = None;
    }

    /// Update calibration state machine
    ///
    /// # Arguments
    /// * `ax_body` - X acceleration in body frame, gravity removed (m/s²)
    /// * `ay_body` - Y acceleration in body frame, gravity removed (m/s²)
    /// * `wz` - Yaw rate (rad/s)
    /// * `speed` - Vehicle speed from GPS or EKF (m/s)
    /// * `now_ms` - Current timestamp in milliseconds
    ///
    /// # Returns
    /// Current calibration result
    pub fn update(
        &mut self,
        ax_body: f32,
        ay_body: f32,
        wz: f32,
        speed: f32,
        now_ms: u32,
    ) -> CalibrationResult {
        let elapsed_ms = now_ms.wrapping_sub(self.state_start_ms);
        let elapsed_s = elapsed_ms as f32 / 1000.0;

        match self.state {
            CalibrationState::Idle => CalibrationResult::InProgress(self.state),

            CalibrationState::WaitingForMotion => {
                // Check timeout
                if elapsed_s > self.config.motion_timeout {
                    self.state = CalibrationState::Failed;
                    return CalibrationResult::Failure(CalibrationError::MotionTimeout);
                }

                // Check if we have sufficient forward motion
                if speed > self.config.min_speed {
                    self.state = CalibrationState::WaitingForBrake;
                    self.state_start_ms = now_ms;
                }

                CalibrationResult::InProgress(self.state)
            }

            CalibrationState::WaitingForBrake => {
                // Check timeout
                if elapsed_s > self.config.brake_timeout {
                    self.state = CalibrationState::Failed;
                    return CalibrationResult::Failure(CalibrationError::BrakeTimeout);
                }

                // Compute total horizontal acceleration magnitude
                let accel_mag = (ax_body * ax_body + ay_body * ay_body).sqrt();

                // Check for braking: high decel, low yaw rate, still moving
                let is_decelerating = accel_mag > self.config.min_decel;
                let is_straight = wz.abs() < self.config.max_yaw_rate;
                let still_moving = speed > self.config.min_speed * 0.5;

                if is_decelerating && is_straight && still_moving {
                    self.state = CalibrationState::Collecting;
                    self.state_start_ms = now_ms;
                    self.sample_count = 0;
                }

                CalibrationResult::InProgress(self.state)
            }

            CalibrationState::Collecting => {
                let accel_mag = (ax_body * ax_body + ay_body * ay_body).sqrt();
                let is_still_braking = accel_mag > self.config.min_decel * 0.7; // Slight hysteresis
                let is_straight = wz.abs() < self.config.max_yaw_rate;

                // If braking stopped or turned, check if we have enough samples
                if !is_still_braking || !is_straight {
                    return self.finalize_calibration();
                }

                // Collect sample
                if self.sample_count < self.samples.len() {
                    self.samples[self.sample_count] = BrakeSample {
                        ax: ax_body,
                        ay: ay_body,
                        timestamp_ms: now_ms,
                    };
                    self.sample_count += 1;
                }

                // Check if we have enough duration
                if elapsed_s >= self.config.sample_duration {
                    return self.finalize_calibration();
                }

                CalibrationResult::InProgress(self.state)
            }

            CalibrationState::Complete => {
                if let Some(offset) = self.computed_offset {
                    CalibrationResult::Success(MountCalibration::new(offset))
                } else {
                    CalibrationResult::InProgress(self.state)
                }
            }

            CalibrationState::Failed => CalibrationResult::Failure(CalibrationError::Cancelled),
        }
    }

    /// Compute the calibration from collected samples
    fn finalize_calibration(&mut self) -> CalibrationResult {
        if self.sample_count < 5 {
            self.state = CalibrationState::Failed;
            return CalibrationResult::Failure(CalibrationError::InsufficientSamples);
        }

        // Compute mean acceleration vector
        let mut sum_ax = 0.0;
        let mut sum_ay = 0.0;

        for i in 0..self.sample_count {
            sum_ax += self.samples[i].ax;
            sum_ay += self.samples[i].ay;
        }

        let mean_ax = sum_ax / self.sample_count as f32;
        let mean_ay = sum_ay / self.sample_count as f32;

        // Compute angle of mean vector
        let mean_angle = mean_ay.atan2(mean_ax);

        // Check consistency: all samples should point roughly the same direction
        let mut max_deviation = 0.0f32;
        for i in 0..self.sample_count {
            let sample_angle = self.samples[i].ay.atan2(self.samples[i].ax);
            let deviation = angle_difference(sample_angle, mean_angle).abs();
            max_deviation = max_deviation.max(deviation);
        }

        if max_deviation > self.config.max_angle_variance {
            self.state = CalibrationState::Failed;
            return CalibrationResult::Failure(CalibrationError::InconsistentSamples);
        }

        // The acceleration during braking points BACKWARD relative to vehicle forward
        // So the mounting offset is the brake angle plus 180 degrees
        let yaw_offset = normalize_angle(mean_angle + PI);

        self.computed_offset = Some(yaw_offset);
        self.state = CalibrationState::Complete;

        CalibrationResult::Success(MountCalibration::new(yaw_offset))
    }

    /// Get computed offset if calibration is complete
    pub fn get_offset(&self) -> Option<f32> {
        self.computed_offset
    }
}

/// Normalize angle to [-π, π]
fn normalize_angle(mut angle: f32) -> f32 {
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}

/// Compute shortest angular difference between two angles
fn angle_difference(a: f32, b: f32) -> f32 {
    normalize_angle(a - b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 1e-6);
        assert!((normalize_angle(PI) - PI).abs() < 1e-6);
        assert!((normalize_angle(-PI) - (-PI)).abs() < 1e-6);
        assert!((normalize_angle(2.0 * PI) - 0.0).abs() < 1e-6);
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-6);
    }

    #[test]
    fn test_calibration_no_offset() {
        // Device mounted aligned with vehicle
        // Braking produces acceleration in -X direction (backward)
        let cal = MountCalibration::new(0.0);
        let (vx, vy) = cal.device_to_vehicle_accel(-3.0, 0.0);
        assert!((vx - (-3.0)).abs() < 1e-6);
        assert!(vy.abs() < 1e-6);
    }

    #[test]
    fn test_calibration_90_degree_offset() {
        // Device mounted rotated 90 degrees clockwise (offset = -π/2)
        // Device X points to vehicle right, Device Y points to vehicle backward
        // In device frame, braking (vehicle -X) appears as +Y (device Y = -vehicle X)
        // Should transform to -X in vehicle frame
        let cal = MountCalibration::new(-PI / 2.0);
        let (vx, vy) = cal.device_to_vehicle_accel(0.0, 3.0);
        assert!((vx - (-3.0)).abs() < 0.01);
        assert!(vy.abs() < 0.01);
    }

    #[test]
    fn test_offset_from_braking_aligned() {
        // Device aligned: braking shows as -X in device frame
        let brake_ax = -3.0;
        let brake_ay = 0.0;
        let brake_angle = brake_ay.atan2(brake_ax); // Should be PI (pointing backward)
        let offset = normalize_angle(brake_angle + PI); // Should be ~0
        assert!(offset.abs() < 0.01);
    }

    #[test]
    fn test_offset_from_braking_rotated() {
        // Device rotated 45 degrees: braking shows as (-2.12, -2.12) in device frame
        let brake_ax = -2.12;
        let brake_ay = -2.12;
        let brake_angle = brake_ay.atan2(brake_ax); // Should be -3π/4
        let offset = normalize_angle(brake_angle + PI); // Should be π/4 (45 degrees)
        assert!((offset - PI / 4.0).abs() < 0.1);
    }
}
