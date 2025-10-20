/// Extended Kalman Filter for vehicle state estimation
/// 7-state: [x, y, ψ, vx, vy, bax, bay]
use core::f32::consts::PI;

const N: usize = 7; // State dimension

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

/// EKF state and covariance
pub struct Ekf {
    /// State: [x, y, ψ, vx, vy, bax, bay]
    pub x: [f32; N],

    /// Covariance matrix (diagonal approximation for efficiency)
    pub p: [f32; N],

    /// Filter configuration
    pub config: EkfConfig,
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
        }
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
    pub fn update_position(&mut self, z_x: f32, z_y: f32) {
        // X position update
        let s_x = self.p[0] + self.config.r_pos;
        let k_x = self.p[0] / s_x;
        self.x[0] += k_x * (z_x - self.x[0]);
        self.p[0] *= 1.0 - k_x;

        // Y position update
        let s_y = self.p[1] + self.config.r_pos;
        let k_y = self.p[1] / s_y;
        self.x[1] += k_y * (z_y - self.x[1]);
        self.p[1] *= 1.0 - k_y;
    }

    /// Update with GPS velocity measurement
    pub fn update_velocity(&mut self, z_vx: f32, z_vy: f32) {
        // VX update
        let s_x = self.p[3] + self.config.r_vel;
        let k_x = self.p[3] / s_x;
        self.x[3] += k_x * (z_vx - self.x[3]);
        self.p[3] *= 1.0 - k_x;

        // VY update
        let s_y = self.p[4] + self.config.r_vel;
        let k_y = self.p[4] / s_y;
        self.x[4] += k_y * (z_vy - self.x[4]);
        self.p[4] *= 1.0 - k_y;
    }

    /// Update with scalar speed measurement
    pub fn update_speed(&mut self, z_speed: f32) {
        let r_spd = if z_speed < 0.3 { 0.20 } else { 0.04 }; // Adaptive R

        let v_est = self.speed().max(0.05); // Avoid division by zero

        // Jacobian H = [vx/v, vy/v]
        let h_x = self.x[3] / v_est;
        let h_y = self.x[4] / v_est;

        // Innovation covariance
        let s = h_x * h_x * self.p[3] + h_y * h_y * self.p[4] + r_spd;

        // Kalman gains
        let k_x = self.p[3] * h_x / s;
        let k_y = self.p[4] * h_y / s;

        // Innovation
        let y = z_speed - v_est;

        // Update
        self.x[3] += k_x * y;
        self.x[4] += k_y * y;
        self.p[3] -= k_x * h_x * self.p[3];
        self.p[4] -= k_y * h_y * self.p[4];
    }

    /// Update with IMU yaw measurement (from magnetometer)
    pub fn update_yaw(&mut self, z_yaw: f32) {
        // Wrap innovation to [-π, π]
        let dy = wrap_angle(z_yaw - self.x[2]);

        let s = self.p[2] + self.config.r_yaw;
        let k = self.p[2] / s;

        self.x[2] += k * dy;
        self.x[2] = wrap_angle(self.x[2]);
        self.p[2] *= 1.0 - k;
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
    pub fn zupt(&mut self) {
        self.update_velocity(0.0, 0.0);
        self.update_speed(0.0);
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
}
