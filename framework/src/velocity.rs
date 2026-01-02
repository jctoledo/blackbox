//! Velocity estimation and source selection
//!
//! This module provides pure functions for selecting the best velocity
//! estimate from multiple sources (EKF, GPS, etc.).

/// Velocity selection threshold (m/s)
/// Below this, EKF velocity is considered unreliable and GPS is preferred
pub const VELOCITY_THRESHOLD: f32 = 0.1;

/// Select best velocity from available sources
///
/// Returns a consistent (vx, vy, speed) tuple from a single source:
/// - Prefers EKF velocity when magnitude exceeds threshold
/// - Falls back to GPS velocity if EKF is unavailable or below threshold
/// - Returns zeros if no valid source
///
/// # Arguments
/// * `ekf_velocity` - Velocity from EKF as (vx, vy), or None
/// * `gps_velocity` - Velocity from GPS as (vx, vy), or None
///
/// # Returns
/// * `(vx, vy, speed)` - Consistent velocity tuple from same source
///
/// # Example
/// ```
/// use sensor_fusion::velocity::select_best_velocity;
///
/// // EKF velocity above threshold - uses EKF
/// let (vx, vy, speed) = select_best_velocity(Some((3.0, 4.0)), Some((1.0, 0.0)));
/// assert_eq!((vx, vy), (3.0, 4.0));
/// assert!((speed - 5.0).abs() < 0.001);
///
/// // EKF velocity below threshold - falls back to GPS
/// let (vx, vy, speed) = select_best_velocity(Some((0.05, 0.05)), Some((1.0, 0.0)));
/// assert_eq!((vx, vy), (1.0, 0.0));
/// ```
pub fn select_best_velocity(
    ekf_velocity: Option<(f32, f32)>,
    gps_velocity: Option<(f32, f32)>,
) -> (f32, f32, f32) {
    // Try EKF first
    if let Some((vx, vy)) = ekf_velocity {
        let speed = (vx * vx + vy * vy).sqrt();
        if speed > VELOCITY_THRESHOLD {
            return (vx, vy, speed);
        }
    }
    // Fallback to GPS
    if let Some((vx, vy)) = gps_velocity {
        let speed = (vx * vx + vy * vy).sqrt();
        return (vx, vy, speed);
    }
    // No valid source
    (0.0, 0.0, 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_prefers_ekf_above_threshold() {
        // EKF velocity (3, 4) has magnitude 5.0, well above 0.1 threshold
        let ekf = Some((3.0, 4.0));
        let gps = Some((1.0, 0.0));

        let (vx, vy, speed) = select_best_velocity(ekf, gps);

        assert_eq!((vx, vy), (3.0, 4.0));
        assert!((speed - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_falls_back_to_gps_below_threshold() {
        // EKF velocity (0.05, 0.05) has magnitude ~0.07, below 0.1 threshold
        let ekf = Some((0.05, 0.05));
        let gps = Some((1.0, 0.0));

        let (vx, vy, speed) = select_best_velocity(ekf, gps);

        assert_eq!((vx, vy), (1.0, 0.0));
        assert!((speed - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_uses_gps_when_no_ekf() {
        let gps = Some((2.0, 0.0));

        let (vx, vy, speed) = select_best_velocity(None, gps);

        assert_eq!((vx, vy), (2.0, 0.0));
        assert!((speed - 2.0).abs() < 0.001);
    }

    #[test]
    fn test_returns_zeros_when_no_source() {
        let (vx, vy, speed) = select_best_velocity(None, None);

        assert_eq!((vx, vy, speed), (0.0, 0.0, 0.0));
    }

    #[test]
    fn test_returns_zeros_when_ekf_below_threshold_no_gps() {
        // EKF below threshold, no GPS fallback
        let ekf = Some((0.01, 0.01));

        let (vx, vy, speed) = select_best_velocity(ekf, None);

        assert_eq!((vx, vy, speed), (0.0, 0.0, 0.0));
    }

    #[test]
    fn test_components_consistent_with_speed() {
        // Verify that returned speed matches sqrt(vx² + vy²)
        let ekf = Some((3.0, 4.0));

        let (vx, vy, speed) = select_best_velocity(ekf, None);
        let computed_speed = (vx * vx + vy * vy).sqrt();

        assert!((speed - computed_speed).abs() < 0.001);
    }

    #[test]
    fn test_at_exact_threshold_uses_gps() {
        // EKF velocity exactly at threshold (0.1) should fall back to GPS
        // since condition is > 0.1, not >= 0.1
        let ekf = Some((0.1, 0.0)); // magnitude = 0.1
        let gps = Some((1.0, 1.0));

        let (vx, vy, _) = select_best_velocity(ekf, gps);

        // Should use GPS since EKF is not strictly greater than threshold
        assert_eq!((vx, vy), (1.0, 1.0));
    }

    #[test]
    fn test_just_above_threshold_uses_ekf() {
        // EKF velocity just above threshold should use EKF
        let ekf = Some((0.11, 0.0)); // magnitude = 0.11 > 0.1
        let gps = Some((1.0, 1.0));

        let (vx, vy, _) = select_best_velocity(ekf, gps);

        assert_eq!((vx, vy), (0.11, 0.0));
    }
}
