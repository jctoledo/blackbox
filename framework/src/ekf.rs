/// Extended Kalman Filter for vehicle state estimation
/// 7-state: [x, y, ψ, vx, vy, bax, bay]
///
/// Features:
/// - CTRA (Constant Turn Rate and Acceleration) motion model at speed
/// - CA (Constant Acceleration) model when slow/straight
/// - Innovation gating to reject outlier measurements (prevents position explosion)
/// - Diagonal covariance approximation for embedded efficiency
use core::f32::consts::PI;

const N: usize = 7; // State dimension

/// Chi-squared thresholds for innovation gating (1 DOF)
/// These values determine how many standard deviations a measurement can be
/// before it's rejected as an outlier.
///
/// At 99.5% confidence: chi2 = 7.88 (rejects 0.5% of valid measurements)
/// At 99% confidence: chi2 = 6.63 (rejects 1% of valid measurements)
/// At 95% confidence: chi2 = 3.84 (rejects 5% of valid measurements)
const CHI2_GATE_POSITION: f32 = 16.0; // ~4 sigma - reject extreme outliers only
const CHI2_GATE_VELOCITY: f32 = 9.0; // ~3 sigma - moderately strict
const CHI2_GATE_YAW: f32 = 12.0; // ~3.5 sigma - account for magnetic interference

/// EKF tuning configuration
/// Allows runtime adjustment of filter parameters (Open/Closed Principle)
#[derive(Debug, Clone, Copy)]
pub struct EkfConfig {
    // Process noise parameters
    pub q_acc: f32,  // (m/s²)² - Acceleration process noise
    pub q_gyro: f32, // (rad/s)² - Gyro process noise
    pub q_bias: f32, // (m/s²)² - Bias evolution noise

    // Measurement noise parameters
    pub r_pos: f32, // (m)² - GPS position measurement noise
    pub r_vel: f32, // (m/s)² - GPS velocity measurement noise
    pub r_yaw: f32, // (rad)² - Magnetometer yaw measurement noise

    // Model parameters
    pub min_speed: f32, // m/s - Minimum speed for CTRA model
}

impl Default for EkfConfig {
    fn default() -> Self {
        Self {
            q_acc: 0.40,
            q_gyro: 0.005,
            q_bias: 1e-3,
            r_pos: 20.0,
            r_vel: 0.2,
            r_yaw: 0.10,
            min_speed: 2.0,
        }
    }
}

/// Result of a measurement update, indicating whether it was accepted or rejected
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UpdateResult {
    /// Measurement was accepted and state was updated
    Accepted,
    /// Measurement was rejected due to innovation gating (outlier)
    Rejected,
}

/// Minimum position sigma floor (meters)
/// EKF can't know position better than GPS accuracy (~2-3m for M9N)
const POS_SIGMA_FLOOR: f32 = 2.5;

/// EKF state and covariance
pub struct Ekf {
    /// State: [x, y, ψ, vx, vy, bax, bay]
    pub x: [f32; N],

    /// Covariance matrix (diagonal approximation for efficiency)
    pub p: [f32; N],

    /// Filter configuration
    pub config: EkfConfig,

    /// Innovation gating statistics (for diagnostics)
    pub rejected_position: u32,
    pub rejected_velocity: u32,
    pub rejected_yaw: u32,
    pub accepted_position: u32,
    pub accepted_velocity: u32,
    pub accepted_yaw: u32,

    // Issue 3: Innovation tracking for uncertainty estimation
    /// Sum of normalized innovation squared (NIS) for position updates
    nis_position_sum: f32,
    /// Count of position updates for NIS average
    nis_position_count: u32,
    /// Last accepted GPS position X (for discrepancy check)
    last_gps_x: f32,
    /// Last accepted GPS position Y (for discrepancy check)
    last_gps_y: f32,
    /// Whether we have a valid last GPS position
    has_last_gps: bool,

    // Issue 4: Yaw stabilization when stationary
    /// Last valid heading before stopping (radians, aligned to GPS course)
    last_moving_yaw: f32,
    /// Whether we have a valid stored yaw from when moving
    has_last_moving_yaw: bool,
}

impl Ekf {
    pub fn new() -> Self {
        Self::with_config(EkfConfig::default())
    }

    pub fn with_config(config: EkfConfig) -> Self {
        Self {
            x: [0.0; N],
            p: [100.0, 100.0, 100.0, 100.0, 100.0, 1.0, 1.0],
            config,
            rejected_position: 0,
            rejected_velocity: 0,
            rejected_yaw: 0,
            accepted_position: 0,
            accepted_velocity: 0,
            accepted_yaw: 0,
            // Issue 3: Innovation tracking
            nis_position_sum: 0.0,
            nis_position_count: 0,
            last_gps_x: 0.0,
            last_gps_y: 0.0,
            has_last_gps: false,
            // Issue 4: Yaw stabilization
            last_moving_yaw: 0.0,
            has_last_moving_yaw: false,
        }
    }

    /// Get innovation gating statistics (accepted, rejected) for position
    pub fn position_gate_stats(&self) -> (u32, u32) {
        (self.accepted_position, self.rejected_position)
    }

    /// Get innovation gating statistics (accepted, rejected) for velocity
    pub fn velocity_gate_stats(&self) -> (u32, u32) {
        (self.accepted_velocity, self.rejected_velocity)
    }

    /// Get innovation gating statistics (accepted, rejected) for yaw
    pub fn yaw_gate_stats(&self) -> (u32, u32) {
        (self.accepted_yaw, self.rejected_yaw)
    }

    /// Reset gating statistics (call periodically for rate calculation)
    pub fn reset_gate_stats(&mut self) {
        self.rejected_position = 0;
        self.rejected_velocity = 0;
        self.rejected_yaw = 0;
        self.accepted_position = 0;
        self.accepted_velocity = 0;
        self.accepted_yaw = 0;
    }

    /// Get position (x, y)
    pub fn position(&self) -> (f32, f32) {
        (self.x[0], self.x[1])
    }

    /// Get yaw (ψ)
    pub fn yaw(&self) -> f32 {
        self.x[2]
    }

    /// Get velocity (vx, vy)
    pub fn velocity(&self) -> (f32, f32) {
        (self.x[3], self.x[4])
    }

    /// Get speed (magnitude of velocity)
    pub fn speed(&self) -> f32 {
        (self.x[3] * self.x[3] + self.x[4] * self.x[4]).sqrt()
    }

    // ============== Issue 3: Effective Uncertainty Estimation ==============

    /// Get average normalized innovation squared (NIS) for position
    ///
    /// NIS should average ~2.0 for 2D position if filter is consistent.
    /// Values significantly higher indicate model/measurement mismatch.
    ///
    /// Returns 2.0 (ideal) if no updates yet.
    pub fn get_nis_position_avg(&self) -> f32 {
        if self.nis_position_count > 0 {
            self.nis_position_sum / self.nis_position_count as f32
        } else {
            2.0 // Expected value for 2 DOF
        }
    }

    /// Reset NIS statistics (call periodically to track recent performance)
    pub fn reset_nis_stats(&mut self) {
        self.nis_position_sum = 0.0;
        self.nis_position_count = 0;
    }

    /// Get distance between EKF position and last GPS measurement (meters)
    ///
    /// Large discrepancy indicates EKF may have drifted from reality.
    /// Returns 0.0 if no GPS position stored yet.
    pub fn get_gps_discrepancy(&self) -> f32 {
        if self.has_last_gps {
            let dx = self.x[0] - self.last_gps_x;
            let dy = self.x[1] - self.last_gps_y;
            (dx * dx + dy * dy).sqrt()
        } else {
            0.0
        }
    }

    /// Get effective position sigma that accounts for uncertainty beyond covariance
    ///
    /// The raw covariance P may not reflect true uncertainty when:
    /// - Innovation statistics indicate model mismatch (NIS >> expected)
    /// - EKF has drifted far from GPS measurements
    ///
    /// This method returns an adjusted sigma that:
    /// 1. Starts with sqrt(P[0] + P[1]) as base
    /// 2. Inflates based on NIS average (if significantly above expected)
    /// 3. Inflates based on GPS discrepancy
    /// 4. Applies minimum floor (GPS can't be more accurate than ~2-3m)
    pub fn get_effective_pos_sigma(&self) -> f32 {
        // Base sigma from covariance
        let base_sigma = (self.p[0] + self.p[1]).sqrt();

        // NIS inflation factor: if NIS is higher than expected, uncertainty is understated
        // Expected NIS for 2 DOF is ~2.0. If it's 4.0, our uncertainty is 2x understated.
        let nis_avg = self.get_nis_position_avg();
        let nis_inflation = if nis_avg > 2.0 {
            (nis_avg / 2.0).sqrt() // sqrt to convert variance ratio to sigma ratio
        } else {
            1.0
        };

        // GPS discrepancy inflation: if EKF is far from GPS, uncertainty is understated
        let discrepancy = self.get_gps_discrepancy();
        let discrepancy_inflation = if discrepancy > 5.0 {
            // If discrepancy > 5m, add it to sigma (conservative)
            1.0 + (discrepancy - 5.0) / 10.0
        } else {
            1.0
        };

        // Combine inflations (multiplicative)
        let inflated_sigma = base_sigma * nis_inflation * discrepancy_inflation;

        // Apply floor: EKF can't know position better than GPS (~2-3m for M9N)
        inflated_sigma.max(POS_SIGMA_FLOOR)
    }

    // ============== Issue 4: Yaw Stabilization When Stationary ==============

    /// Store current yaw as the last known heading while moving
    ///
    /// Call this while vehicle is moving (before it stops) to remember
    /// the heading for stabilization when stationary.
    pub fn store_moving_yaw(&mut self, yaw: f32) {
        self.last_moving_yaw = yaw;
        self.has_last_moving_yaw = true;
    }

    /// Get the last stored yaw from when the vehicle was moving
    ///
    /// Returns None if no yaw has been stored yet.
    pub fn get_last_moving_yaw(&self) -> Option<f32> {
        if self.has_last_moving_yaw {
            Some(self.last_moving_yaw)
        } else {
            None
        }
    }

    /// Lock yaw when stationary - reduces yaw uncertainty to prevent
    /// magnetometer noise from corrupting heading while parked
    ///
    /// Similar to lock_position(), this makes the EKF resistant to
    /// noisy magnetometer updates when we're confident about heading.
    ///
    /// With P_yaw = 0.001 and R_yaw = 0.10, Kalman gain K ≈ 0.01,
    /// so magnetometer updates only affect yaw by ~1% instead of ~50%.
    pub fn lock_yaw(&mut self) {
        self.p[2] = 0.001; // Very confident in current yaw
    }

    /// Update yaw with soft constraint (higher measurement noise)
    ///
    /// Use this when stationary to allow slow yaw updates without
    /// letting magnetometer noise cause rapid drift.
    ///
    /// # Arguments
    /// * `z_yaw` - Yaw measurement in radians
    /// * `r_yaw_factor` - Multiplier for measurement noise (>1 = less trust)
    pub fn update_yaw_soft(&mut self, z_yaw: f32, r_yaw_factor: f32) -> UpdateResult {
        // Wrap innovation to [-π, π]
        let dy = wrap_angle(z_yaw - self.x[2]);

        // Use inflated measurement noise
        let r_yaw = self.config.r_yaw * r_yaw_factor;
        let s = self.p[2] + r_yaw;

        // Mahalanobis distance squared (1 DOF)
        let mahal_sq = (dy * dy) / s;

        // Chi-squared gate (same as regular update_yaw)
        if mahal_sq > CHI2_GATE_YAW {
            self.rejected_yaw += 1;
            return UpdateResult::Rejected;
        }

        // Apply Kalman update with reduced gain due to higher R
        let k = self.p[2] / s;
        self.x[2] += k * dy;
        self.x[2] = wrap_angle(self.x[2]);
        self.p[2] *= 1.0 - k;

        self.accepted_yaw += 1;
        UpdateResult::Accepted
    }

    /// Get accelerometer biases (bax, bay)
    #[allow(dead_code)]
    pub fn biases(&self) -> (f32, f32) {
        (self.x[5], self.x[6])
    }

    /// Prediction step using CTRA or constant acceleration model
    pub fn predict(&mut self, ax_earth: f32, ay_earth: f32, wz: f32, dt: f32) {
        let speed = self.speed();
        let use_ctra = speed > self.config.min_speed;

        // Copy values to avoid borrowing issues
        let mut x = self.x[0];
        let mut y = self.x[1];
        let mut psi = self.x[2];
        let mut vx = self.x[3];
        let mut vy = self.x[4];
        let bax = self.x[5];
        let bay = self.x[6];

        // Predict position
        if use_ctra && wz.abs() > 1e-4 {
            // CTRA model (Constant Turn Rate and Acceleration)
            let sin_d = (psi + wz * dt).sin() - psi.sin();
            let cos_d = (psi + wz * dt).cos() - psi.cos();
            x += sin_d / wz * vx + cos_d / wz * vy;
            y += -cos_d / wz * vx + sin_d / wz * vy;
        } else {
            // Constant acceleration model
            x += vx * dt + 0.5 * (ax_earth - bax) * dt * dt;
            y += vy * dt + 0.5 * (ay_earth - bay) * dt * dt;
        }

        // Predict yaw
        psi += wz * dt;
        psi = wrap_angle(psi);

        // Predict velocity
        vx += (ax_earth - bax) * dt;
        vy += (ay_earth - bay) * dt;

        // Write back to state
        self.x[0] = x;
        self.x[1] = y;
        self.x[2] = psi;
        self.x[3] = vx;
        self.x[4] = vy;
        // Biases remain constant (self.x[5] and self.x[6] unchanged)

        // Update covariance (diagonal approximation)
        let qx = 0.25 * self.config.q_acc * dt * dt;
        let qv = self.config.q_acc * dt * dt;
        self.p[0] += qx;
        self.p[1] += qx;
        self.p[2] += self.config.q_gyro * dt * dt;
        self.p[3] += qv;
        self.p[4] += qv;
        self.p[5] += self.config.q_bias * dt + 1e-6;
        self.p[6] += self.config.q_bias * dt + 1e-6;
    }

    /// Update with GPS position measurement
    ///
    /// Uses innovation gating (Mahalanobis distance test) to reject outliers.
    /// This prevents corrupt GPS data from causing position explosion.
    ///
    /// Returns UpdateResult::Accepted if measurement was used, Rejected if gated.
    pub fn update_position(&mut self, z_x: f32, z_y: f32) -> UpdateResult {
        // Compute innovations (measurement residuals)
        let y_x = z_x - self.x[0];
        let y_y = z_y - self.x[1];

        // Innovation covariances (diagonal)
        let s_x = self.p[0] + self.config.r_pos;
        let s_y = self.p[1] + self.config.r_pos;

        // Mahalanobis distance squared (sum for 2D position)
        // For diagonal covariance: d² = y_x²/s_x + y_y²/s_y
        let mahal_sq = (y_x * y_x) / s_x + (y_y * y_y) / s_y;

        // Chi-squared gate: reject if measurement is too far from prediction
        // For 2 DOF at 99.9% confidence, chi2 ≈ 13.8
        // Using sum of individual 1-DOF tests with CHI2_GATE_POSITION each
        if mahal_sq > CHI2_GATE_POSITION * 2.0 {
            self.rejected_position += 1;
            return UpdateResult::Rejected;
        }

        // Issue 3: Track normalized innovation squared (NIS) for uncertainty estimation
        // NIS = innovation² / S, should average ~1.0 if filter is consistent
        // High NIS average indicates model/measurement mismatch
        self.nis_position_sum += mahal_sq;
        self.nis_position_count += 1;

        // Store GPS position for discrepancy checking
        self.last_gps_x = z_x;
        self.last_gps_y = z_y;
        self.has_last_gps = true;

        // Measurement passed gating - apply standard Kalman update
        let k_x = self.p[0] / s_x;
        self.x[0] += k_x * y_x;
        self.p[0] *= 1.0 - k_x;

        let k_y = self.p[1] / s_y;
        self.x[1] += k_y * y_y;
        self.p[1] *= 1.0 - k_y;

        self.accepted_position += 1;
        UpdateResult::Accepted
    }

    /// Update with GPS velocity measurement
    ///
    /// Uses innovation gating to reject outliers (e.g., corrupt GPS data).
    ///
    /// Returns UpdateResult::Accepted if measurement was used, Rejected if gated.
    pub fn update_velocity(&mut self, z_vx: f32, z_vy: f32) -> UpdateResult {
        // Compute innovations
        let y_vx = z_vx - self.x[3];
        let y_vy = z_vy - self.x[4];

        // Innovation covariances
        let s_x = self.p[3] + self.config.r_vel;
        let s_y = self.p[4] + self.config.r_vel;

        // Mahalanobis distance squared (2D velocity)
        let mahal_sq = (y_vx * y_vx) / s_x + (y_vy * y_vy) / s_y;

        // Chi-squared gate for 2 DOF
        if mahal_sq > CHI2_GATE_VELOCITY * 2.0 {
            self.rejected_velocity += 1;
            return UpdateResult::Rejected;
        }

        // Apply Kalman update
        let k_x = self.p[3] / s_x;
        self.x[3] += k_x * y_vx;
        self.p[3] *= 1.0 - k_x;

        let k_y = self.p[4] / s_y;
        self.x[4] += k_y * y_vy;
        self.p[4] *= 1.0 - k_y;

        self.accepted_velocity += 1;
        UpdateResult::Accepted
    }

    /// Update with scalar speed measurement
    ///
    /// Uses innovation gating to reject outlier speed measurements.
    /// Note: Speed is always non-negative, so outliers are typically
    /// from corrupt GPS data showing impossibly high speeds.
    pub fn update_speed(&mut self, z_speed: f32) -> UpdateResult {
        let r_spd = if z_speed < 0.3 { 0.20 } else { 0.04 }; // Adaptive R

        let v_est = self.speed().max(0.05); // Avoid division by zero

        // Jacobian H = [vx/v, vy/v]
        let h_x = self.x[3] / v_est;
        let h_y = self.x[4] / v_est;

        // Innovation covariance
        let s = h_x * h_x * self.p[3] + h_y * h_y * self.p[4] + r_spd;

        // Innovation
        let y = z_speed - v_est;

        // Mahalanobis distance squared (1 DOF for scalar speed)
        let mahal_sq = (y * y) / s;

        // Gate using velocity threshold (speed is derived from velocity)
        if mahal_sq > CHI2_GATE_VELOCITY {
            // Don't increment rejected_velocity here - it's a different measurement
            return UpdateResult::Rejected;
        }

        // Kalman gains
        let k_x = self.p[3] * h_x / s;
        let k_y = self.p[4] * h_y / s;

        // Update
        self.x[3] += k_x * y;
        self.x[4] += k_y * y;
        self.p[3] -= k_x * h_x * self.p[3];
        self.p[4] -= k_y * h_y * self.p[4];

        UpdateResult::Accepted
    }

    /// Update with IMU yaw measurement (from magnetometer)
    ///
    /// Uses innovation gating to reject outliers (e.g., magnetic interference).
    /// Note: Innovation is wrapped to [-π, π] before gating.
    ///
    /// Returns UpdateResult::Accepted if measurement was used, Rejected if gated.
    pub fn update_yaw(&mut self, z_yaw: f32) -> UpdateResult {
        // Wrap innovation to [-π, π]
        let dy = wrap_angle(z_yaw - self.x[2]);

        // Innovation covariance
        let s = self.p[2] + self.config.r_yaw;

        // Mahalanobis distance squared (1 DOF)
        let mahal_sq = (dy * dy) / s;

        // Chi-squared gate
        if mahal_sq > CHI2_GATE_YAW {
            self.rejected_yaw += 1;
            return UpdateResult::Rejected;
        }

        // Apply Kalman update
        let k = self.p[2] / s;
        self.x[2] += k * dy;
        self.x[2] = wrap_angle(self.x[2]);
        self.p[2] *= 1.0 - k;

        self.accepted_yaw += 1;
        UpdateResult::Accepted
    }

    /// Update accelerometer biases when stationary
    pub fn update_bias(&mut self, z_ax: f32, z_ay: f32) {
        const R_BIAS: f32 = 0.05 * 0.05; // (0.05 m/s²)²

        // BAX update
        let s_x = self.p[5] + R_BIAS;
        let k_x = self.p[5] / s_x;
        self.x[5] += k_x * (z_ax - self.x[5]);
        self.p[5] *= 1.0 - k_x;

        // BAY update
        let s_y = self.p[6] + R_BIAS;
        let k_y = self.p[6] / s_y;
        self.x[6] += k_y * (z_ay - self.x[6]);
        self.p[6] *= 1.0 - k_y;
    }

    /// Zero-velocity update (ZUPT) - force velocity to zero
    ///
    /// ZUPT bypasses innovation gating because we have high confidence
    /// the vehicle is stationary (verified by multiple sensors).
    /// If the EKF has drifted, ZUPT is the mechanism to recover.
    pub fn zupt(&mut self) {
        // Force velocity to zero without gating
        // This is intentional - ZUPT is a recovery mechanism
        self.x[3] = 0.0;
        self.x[4] = 0.0;

        // Reduce velocity covariance to reflect high confidence
        // Using very small values (but not zero to maintain numerical stability)
        self.p[3] = 0.01;
        self.p[4] = 0.01;
    }

    /// Lock position when stationary - reduces position uncertainty to make EKF
    /// resistant to GPS noise. This prevents phantom movement while parked.
    ///
    /// With P_pos = 0.01 and R_pos = 20.0, Kalman gain K = 0.01/20.01 ≈ 0.0005,
    /// so GPS noise only affects position by 0.05% instead of 45%.
    pub fn lock_position(&mut self) {
        self.p[0] = 0.01;
        self.p[1] = 0.01;
    }

    /// Skip IMU prediction (use when stationary to prevent drift)
    /// Call this instead of predict() when vehicle is known to be stationary.
    pub fn skip_prediction(&mut self) {
        // Do nothing - position and velocity stay constant
        // Covariance grows very slightly to allow eventual GPS correction
        self.p[0] += 1e-6;
        self.p[1] += 1e-6;
    }
}

impl Default for Ekf {
    fn default() -> Self {
        Self::new()
    }
}

/// Wrap angle to [-π, π]
fn wrap_angle(angle: f32) -> f32 {
    let mut a = angle;
    while a > PI {
        a -= 2.0 * PI;
    }
    while a < -PI {
        a += 2.0 * PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ekf_init() {
        let ekf = Ekf::new();
        assert_eq!(ekf.position(), (0.0, 0.0));
        assert_eq!(ekf.speed(), 0.0);
    }

    #[test]
    fn test_wrap_angle() {
        assert!((wrap_angle(0.0) - 0.0).abs() < 0.001);
        assert!((wrap_angle(4.0 * PI) - 0.0).abs() < 0.001);
        assert!((wrap_angle(PI + 0.1) - (-PI + 0.1)).abs() < 0.001);
    }

    // ========== Innovation Gating Tests ==========

    #[test]
    fn test_position_gating_accepts_reasonable_measurement() {
        let mut ekf = Ekf::new();
        // Initialize at origin with some uncertainty
        ekf.x[0] = 0.0;
        ekf.x[1] = 0.0;
        ekf.p[0] = 10.0; // 10m² variance = ~3m std dev
        ekf.p[1] = 10.0;

        // Measurement within ~3 sigma (sqrt(10+20) = 5.5m std dev)
        // 5m is within reasonable range
        let result = ekf.update_position(5.0, 5.0);

        assert_eq!(result, UpdateResult::Accepted);
        assert!(ekf.x[0] > 0.0); // State should have moved toward measurement
        assert!(ekf.x[1] > 0.0);
        assert_eq!(ekf.accepted_position, 1);
        assert_eq!(ekf.rejected_position, 0);
    }

    #[test]
    fn test_position_gating_rejects_extreme_outlier() {
        let mut ekf = Ekf::new();
        // Initialize at origin with tight uncertainty
        ekf.x[0] = 0.0;
        ekf.x[1] = 0.0;
        ekf.p[0] = 1.0; // 1m² variance = 1m std dev
        ekf.p[1] = 1.0;

        // Extreme outlier: 1000m away when EKF thinks we're at origin with 1m uncertainty
        // Mahalanobis: (1000² / (1+20)) = 47619 >> threshold
        let result = ekf.update_position(1000.0, 1000.0);

        assert_eq!(result, UpdateResult::Rejected);
        // State should NOT have moved
        assert!((ekf.x[0] - 0.0).abs() < 0.001);
        assert!((ekf.x[1] - 0.0).abs() < 0.001);
        assert_eq!(ekf.accepted_position, 0);
        assert_eq!(ekf.rejected_position, 1);
    }

    #[test]
    fn test_position_gating_accepts_after_covariance_grows() {
        let mut ekf = Ekf::new();
        ekf.x[0] = 0.0;
        ekf.x[1] = 0.0;
        ekf.p[0] = 1.0;
        ekf.p[1] = 1.0;

        // First: 100m measurement rejected when uncertainty is low
        let result1 = ekf.update_position(100.0, 100.0);
        assert_eq!(result1, UpdateResult::Rejected);

        // Simulate many prediction steps that grow covariance
        // qx = 0.25 * q_acc * dt² = 0.25 * 0.40 * 0.01 = 0.001 per step
        // After 1000 steps: P grows by ~1.0
        for _ in 0..1000 {
            ekf.predict(0.0, 0.0, 0.0, 0.1);
        }

        // Now covariance should be larger (not huge, but growing)
        assert!(
            ekf.p[0] > 1.5,
            "Covariance should grow: {}",
            ekf.p[0]
        );

        // The key insight: covariance growth allows larger innovations
        // With very high uncertainty, even extreme measurements get accepted
        // Let's verify by setting high uncertainty manually
        ekf.p[0] = 10000.0; // Very uncertain
        ekf.p[1] = 10000.0;
        let result2 = ekf.update_position(100.0, 100.0);
        assert_eq!(
            result2,
            UpdateResult::Accepted,
            "Should accept when uncertainty is high"
        );
    }

    #[test]
    fn test_velocity_gating_accepts_reasonable_measurement() {
        let mut ekf = Ekf::new();
        ekf.x[3] = 10.0; // Moving at 10 m/s
        ekf.x[4] = 0.0;
        ekf.p[3] = 1.0;
        ekf.p[4] = 1.0;

        // Small velocity change is reasonable
        let result = ekf.update_velocity(11.0, 1.0);

        assert_eq!(result, UpdateResult::Accepted);
        assert_eq!(ekf.accepted_velocity, 1);
    }

    #[test]
    fn test_velocity_gating_rejects_impossible_jump() {
        let mut ekf = Ekf::new();
        ekf.x[3] = 10.0; // Moving at 10 m/s
        ekf.x[4] = 0.0;
        ekf.p[3] = 0.1; // Very confident in velocity
        ekf.p[4] = 0.1;

        // Jump from 10 m/s to 100 m/s is physically impossible
        // Mahalanobis: (90² / (0.1+0.2)) = 27000 >> threshold
        let result = ekf.update_velocity(100.0, 0.0);

        assert_eq!(result, UpdateResult::Rejected);
        // Velocity should NOT have changed
        assert!((ekf.x[3] - 10.0).abs() < 0.1);
        assert_eq!(ekf.rejected_velocity, 1);
    }

    #[test]
    fn test_yaw_gating_accepts_small_correction() {
        let mut ekf = Ekf::new();
        ekf.x[2] = 0.0; // Facing east
        ekf.p[2] = 0.1; // ~18° std dev

        // Small yaw correction (10°)
        let result = ekf.update_yaw(0.17); // ~10°

        assert_eq!(result, UpdateResult::Accepted);
        assert!(ekf.x[2] > 0.0); // Yaw moved toward measurement
    }

    #[test]
    fn test_yaw_gating_rejects_magnetic_spike() {
        let mut ekf = Ekf::new();
        ekf.x[2] = 0.0;
        ekf.p[2] = 0.01; // Very confident

        // Sudden 180° flip (magnetic interference near metal)
        let result = ekf.update_yaw(PI);

        assert_eq!(result, UpdateResult::Rejected);
        // Yaw should NOT have flipped
        assert!(ekf.x[2].abs() < 0.1);
        assert_eq!(ekf.rejected_yaw, 1);
    }

    #[test]
    fn test_zupt_bypasses_gating() {
        let mut ekf = Ekf::new();
        // EKF thinks we're moving fast
        ekf.x[3] = 50.0; // 50 m/s
        ekf.x[4] = 50.0;
        ekf.p[3] = 0.01; // Very confident (incorrectly)
        ekf.p[4] = 0.01;

        // ZUPT should FORCE velocity to zero regardless of gating
        ekf.zupt();

        assert!((ekf.x[3] - 0.0).abs() < 0.001);
        assert!((ekf.x[4] - 0.0).abs() < 0.001);
        // Covariance should be reset to small value
        assert!(ekf.p[3] < 0.1);
        assert!(ekf.p[4] < 0.1);
    }

    #[test]
    fn test_gating_stats_reset() {
        let mut ekf = Ekf::new();
        ekf.p[0] = 0.1;
        ekf.p[1] = 0.1;

        // Generate some rejections
        ekf.update_position(1000.0, 1000.0);
        ekf.update_position(1000.0, 1000.0);
        assert_eq!(ekf.rejected_position, 2);

        // Reset stats
        ekf.reset_gate_stats();
        assert_eq!(ekf.rejected_position, 0);
        assert_eq!(ekf.accepted_position, 0);
    }

    #[test]
    fn test_speed_gating_rejects_impossible_speed() {
        let mut ekf = Ekf::new();
        ekf.x[3] = 10.0;
        ekf.x[4] = 0.0;
        ekf.p[3] = 0.1;
        ekf.p[4] = 0.1;

        // Current speed is ~10 m/s, claiming 200 m/s is impossible
        let result = ekf.update_speed(200.0);

        assert_eq!(result, UpdateResult::Rejected);
        // Speed should NOT have changed much
        assert!(ekf.speed() < 15.0);
    }

    // ========== Issue 3: Effective Uncertainty Tests ==========

    #[test]
    fn test_nis_tracking() {
        let mut ekf = Ekf::new();
        ekf.x[0] = 0.0;
        ekf.x[1] = 0.0;
        ekf.p[0] = 10.0;
        ekf.p[1] = 10.0;

        // Initial NIS average should be 2.0 (expected for 2 DOF)
        assert!((ekf.get_nis_position_avg() - 2.0).abs() < 0.01);

        // Accept a measurement - NIS should be tracked
        ekf.update_position(2.0, 2.0);
        assert_eq!(ekf.nis_position_count, 1);

        // NIS for this measurement: (2² + 2²) / (10+20) / 2 per dim
        // Should be small since measurement is close
        assert!(ekf.get_nis_position_avg() < 1.0);
    }

    #[test]
    fn test_nis_reset() {
        let mut ekf = Ekf::new();
        ekf.p[0] = 10.0;
        ekf.p[1] = 10.0;

        ekf.update_position(1.0, 1.0);
        ekf.update_position(2.0, 2.0);
        assert_eq!(ekf.nis_position_count, 2);

        ekf.reset_nis_stats();
        assert_eq!(ekf.nis_position_count, 0);
        assert!((ekf.get_nis_position_avg() - 2.0).abs() < 0.01); // Back to default
    }

    #[test]
    fn test_gps_discrepancy() {
        let mut ekf = Ekf::new();
        ekf.x[0] = 0.0;
        ekf.x[1] = 0.0;
        ekf.p[0] = 100.0;
        ekf.p[1] = 100.0;

        // No GPS yet - discrepancy should be 0
        assert!((ekf.get_gps_discrepancy() - 0.0).abs() < 0.01);

        // Accept GPS at (10, 0)
        ekf.update_position(10.0, 0.0);

        // EKF should have moved toward GPS, but discrepancy should be low
        assert!(ekf.get_gps_discrepancy() < 2.0);

        // Now simulate EKF drift: manually set EKF position far from last GPS
        ekf.x[0] = 50.0;
        ekf.x[1] = 0.0;

        // Discrepancy should now be ~40m (50 - 10)
        let discrepancy = ekf.get_gps_discrepancy();
        assert!(
            discrepancy > 35.0 && discrepancy < 45.0,
            "Discrepancy should be ~40m, got {}",
            discrepancy
        );
    }

    #[test]
    fn test_effective_pos_sigma_floor() {
        let mut ekf = Ekf::new();
        // Very confident EKF (low covariance)
        ekf.p[0] = 0.01;
        ekf.p[1] = 0.01;

        // Even with low covariance, effective sigma should have floor
        let sigma = ekf.get_effective_pos_sigma();
        assert!(
            sigma >= POS_SIGMA_FLOOR,
            "Sigma {} should be >= floor {}",
            sigma,
            POS_SIGMA_FLOOR
        );
    }

    #[test]
    fn test_effective_pos_sigma_inflates_with_discrepancy() {
        let mut ekf = Ekf::new();
        ekf.p[0] = 10.0;
        ekf.p[1] = 10.0;

        // Accept GPS at origin
        ekf.update_position(0.0, 0.0);
        let sigma_no_discrepancy = ekf.get_effective_pos_sigma();

        // Simulate drift: move EKF far from last GPS
        ekf.x[0] = 20.0;
        ekf.x[1] = 0.0;

        let sigma_with_discrepancy = ekf.get_effective_pos_sigma();
        assert!(
            sigma_with_discrepancy > sigma_no_discrepancy,
            "Sigma should inflate with discrepancy: {} > {}",
            sigma_with_discrepancy,
            sigma_no_discrepancy
        );
    }

    // ========== Issue 4: Yaw Stabilization Tests ==========

    #[test]
    fn test_store_moving_yaw() {
        let mut ekf = Ekf::new();

        // Initially no stored yaw
        assert!(ekf.get_last_moving_yaw().is_none());

        // Store yaw
        ekf.store_moving_yaw(1.5);
        assert!(ekf.get_last_moving_yaw().is_some());
        assert!((ekf.get_last_moving_yaw().unwrap() - 1.5).abs() < 0.01);

        // Update stored yaw
        ekf.store_moving_yaw(2.0);
        assert!((ekf.get_last_moving_yaw().unwrap() - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_lock_yaw_reduces_sensitivity() {
        let mut ekf = Ekf::new();
        ekf.x[2] = 0.5; // Current yaw
        ekf.p[2] = 0.1; // Normal uncertainty

        // Normal yaw update
        let yaw_before = ekf.x[2];
        ekf.update_yaw(1.0);
        let change_normal = (ekf.x[2] - yaw_before).abs();

        // Reset
        ekf.x[2] = 0.5;
        ekf.p[2] = 0.1;

        // Lock yaw then update
        ekf.lock_yaw();
        let yaw_before_locked = ekf.x[2];
        ekf.update_yaw(1.0);
        let change_locked = (ekf.x[2] - yaw_before_locked).abs();

        // Locked update should have much smaller effect
        assert!(
            change_locked < change_normal * 0.1,
            "Locked change {} should be << normal change {}",
            change_locked,
            change_normal
        );
    }

    #[test]
    fn test_update_yaw_soft() {
        let mut ekf = Ekf::new();
        ekf.x[2] = 0.0;
        ekf.p[2] = 0.1;

        // Soft update with 10x noise factor
        let yaw_before = ekf.x[2];
        ekf.update_yaw_soft(0.5, 10.0);
        let change_soft = (ekf.x[2] - yaw_before).abs();

        // Reset
        ekf.x[2] = 0.0;
        ekf.p[2] = 0.1;

        // Normal update
        let yaw_before_normal = ekf.x[2];
        ekf.update_yaw(0.5);
        let change_normal = (ekf.x[2] - yaw_before_normal).abs();

        // Soft update should have smaller effect
        assert!(
            change_soft < change_normal,
            "Soft change {} should be < normal change {}",
            change_soft,
            change_normal
        );
    }
}
