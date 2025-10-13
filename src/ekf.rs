/// Extended Kalman Filter for vehicle state estimation
/// 7-state: [x, y, ψ, vx, vy, bax, bay]

use core::f32::consts::PI;

const N: usize = 7; // State dimension

// Process noise
const Q_ACC: f32 = 0.40;   // (m/s²)²
const Q_GYRO: f32 = 0.005; // (rad/s)²
const Q_BIAS: f32 = 1e-3;  // (m/s²)²

// Measurement noise
const R_POS: f32 = 20.0;   // (m)²
const R_VEL: f32 = 0.2;    // (m/s)²
const R_YAW: f32 = 0.10;   // (rad)²

/// EKF state and covariance
pub struct Ekf {
    /// State: [x, y, ψ, vx, vy, bax, bay]
    pub x: [f32; N],
    
    /// Covariance matrix (diagonal approximation for efficiency)
    pub p: [f32; N],
    
    /// Minimum speed for CTRA model (m/s)
    pub min_speed: f32,
}

impl Ekf {
    pub fn new() -> Self {
        Self {
            x: [0.0; N],
            p: [100.0, 100.0, 100.0, 100.0, 100.0, 1.0, 1.0],
            min_speed: 2.0, // ~7 km/h
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
        let use_ctra = speed > self.min_speed;
        
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
        let qx = 0.25 * Q_ACC * dt * dt;
        let qv = Q_ACC * dt * dt;
        self.p[0] += qx;
        self.p[1] += qx;
        self.p[2] += Q_GYRO * dt * dt;
        self.p[3] += qv;
        self.p[4] += qv;
        self.p[5] += Q_BIAS * dt + 1e-6;
        self.p[6] += Q_BIAS * dt + 1e-6;
    }
    
    /// Update with GPS position measurement
    pub fn update_position(&mut self, z_x: f32, z_y: f32) {
        // X position update
        let s_x = self.p[0] + R_POS;
        let k_x = self.p[0] / s_x;
        self.x[0] += k_x * (z_x - self.x[0]);
        self.p[0] *= 1.0 - k_x;
        
        // Y position update
        let s_y = self.p[1] + R_POS;
        let k_y = self.p[1] / s_y;
        self.x[1] += k_y * (z_y - self.x[1]);
        self.p[1] *= 1.0 - k_y;
    }
    
    /// Update with GPS velocity measurement
    pub fn update_velocity(&mut self, z_vx: f32, z_vy: f32) {
        // VX update
        let s_x = self.p[3] + R_VEL;
        let k_x = self.p[3] / s_x;
        self.x[3] += k_x * (z_vx - self.x[3]);
        self.p[3] *= 1.0 - k_x;
        
        // VY update
        let s_y = self.p[4] + R_VEL;
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
        
        let s = self.p[2] + R_YAW;
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
