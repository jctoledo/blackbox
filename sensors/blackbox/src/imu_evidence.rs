//! Application IMU adapter: packet freshness and frame contracts live together.
//! Host-receive ages do not establish synchronized sensor measurement epochs.
use sensor_fusion::transforms::{body_angular_rates, remove_gravity, AngularRates};
use wt901::ImuData;

pub const MAX_IMU_AGE_MS: u32 = 200;

pub fn packet_age_ms(now_ms: u32, packet_us: &[u64]) -> Option<u32> {
    packet_us.iter().try_fold(0, |oldest, &at_us| {
        if at_us == 0 {
            return None;
        }
        let age = now_ms.wrapping_sub((at_us / 1000) as u32);
        (age <= MAX_IMU_AGE_MS).then_some(oldest.max(age))
    })
}

/// Angular evidence does not depend on accelerometer quality.
pub fn angular_rates(data: &ImuData, now_ms: u32, gyro_angle_us: [u64; 2]) -> Option<AngularRates> {
    packet_age_ms(now_ms, &gyro_angle_us)?;
    body_angular_rates(data.wx, data.wy, data.wz, data.roll, data.pitch)
}

#[derive(Clone, Copy, Debug)]
pub struct MotionEvidence {
    pub rates: AngularRates,
    /// Linear acceleration in sensor body axes, with predicted gravity removed.
    pub linear_body: [f32; 3],
    pub age_ms: u32,
}

impl MotionEvidence {
    /// Stationarity uses vector norms, invariant under mounting rotation.
    pub fn stationary_candidate(&self, speed_mps: f32) -> bool {
        sensor_fusion::stationary::stationary_candidate_3d(
            self.linear_body,
            self.rates.body_norm,
            speed_mps,
        )
    }
}

/// Order of packet times is acceleration, gyro, attitude. Bias is removed by the parser.
pub fn motion_evidence(
    data: &ImuData,
    corrected_accel: (f32, f32, f32),
    now_ms: u32,
    packet_us: [u64; 3],
) -> Option<MotionEvidence> {
    let age_ms = packet_age_ms(now_ms, &packet_us)?;
    let rates = angular_rates(data, now_ms, [packet_us[1], packet_us[2]])?;
    let (ax, ay, az) = corrected_accel;
    if ![ax, ay, az].iter().all(|v| v.is_finite()) {
        return None;
    }
    let (x, y, z) = remove_gravity(ax, ay, az, data.roll, data.pitch);
    Some(MotionEvidence {
        rates,
        linear_body: [x, y, z],
        age_ms,
    })
}
