//! Stationary gating uses SI units, like the sensor drivers.
const G: f32 = 9.80665;
const MAX_SPEED_MPS: f32 = 3.5 / 3.6;

/// Candidate only; callers must also require a fresh fix and sustained evidence.
pub fn stationary_candidate(ax: f32, ay: f32, wz: f32, speed_mps: f32) -> bool {
    [ax, ay, wz, speed_mps].iter().all(|v| v.is_finite())
        && ax.abs() < 0.18 * G
        && ay.abs() < 0.18 * G
        && wz.abs() < 12.0_f32.to_radians()
        && (0.0..MAX_SPEED_MPS).contains(&speed_mps)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn slow_driving_is_not_stationary() {
        assert!(!stationary_candidate(0.0, 0.0, 0.0, 2.0)); // 7.2km/h
        assert!(stationary_candidate(0.01, -0.01, 0.0, 0.0));
        assert!(!stationary_candidate(0.0, 0.0, 0.0, f32::NAN));
    }
}
