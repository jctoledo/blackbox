//! Velocity estimation and source selection
//!
//! This module provides pure functions for selecting the best velocity
//! estimate from multiple sources (EKF, GPS, etc.).

/// Velocity selection threshold (m/s)
/// Below this, an EKF-only fallback is suppressed as near-zero drift
pub const VELOCITY_THRESHOLD: f32 = 0.1;

/// Select a consistent (east m/s, north m/s, magnitude m/s) tuple.
///
/// Prefer fresh receiver velocity, including a valid zero, over an unvalidated
/// estimator. The caller must omit GPS when invalid, stale or not warmed up.
/// Fall back to a finite EKF velocity above 0.1 m/s, otherwise return zero.
/// Both sources are limited to 100 m/s, matching the application GPS bound.
pub fn select_best_velocity(
    ekf_velocity: Option<(f32, f32)>,
    gps_velocity: Option<(f32, f32)>,
) -> (f32, f32, f32) {
    let bounded = |(vx, vy): (f32, f32)| {
        let speed = (vx * vx + vy * vy).sqrt();
        (speed.is_finite() && speed < 100.0).then_some((vx, vy, speed))
    };
    if let Some(gps) = gps_velocity.and_then(bounded) {
        return gps;
    }
    if let Some(ekf) = ekf_velocity.and_then(bounded) {
        if ekf.2 > VELOCITY_THRESHOLD {
            return ekf;
        }
    }
    (0.0, 0.0, 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gps_components_and_magnitude_share_one_source() {
        assert_eq!(
            select_best_velocity(Some((8.0, 6.0)), Some((3.0, 4.0))),
            (3.0, 4.0, 5.0)
        );
    }

    #[test]
    fn gps_zero_and_low_speeds_beat_estimator_drift() {
        for gps in [(0.0, 0.0), (0.01, 0.02), (0.1, 0.0)] {
            let result = select_best_velocity(Some((3.0, 4.0)), Some(gps));
            assert_eq!((result.0, result.1), gps);
            assert_eq!(result.2, (gps.0 * gps.0 + gps.1 * gps.1).sqrt());
        }
    }

    #[test]
    fn unavailable_gps_allows_bounded_estimator_fallback() {
        assert_eq!(
            select_best_velocity(Some((3.0, 4.0)), None),
            (3.0, 4.0, 5.0)
        );
        assert_eq!(
            select_best_velocity(Some((0.11, 0.0)), None),
            (0.11, 0.0, 0.11)
        );
        assert_eq!(
            select_best_velocity(Some((0.1, 0.0)), None),
            (0.0, 0.0, 0.0)
        );
    }

    #[test]
    fn invalid_or_extreme_sources_cannot_publish_speed() {
        for bad in [
            (f32::NAN, 0.0),
            (0.0, f32::INFINITY),
            (100.0, 0.0),
            (80.0, 80.0),
        ] {
            assert_eq!(
                select_best_velocity(Some((3.0, 4.0)), Some(bad)),
                (3.0, 4.0, 5.0)
            );
            assert_eq!(select_best_velocity(Some(bad), None), (0.0, 0.0, 0.0));
            assert_eq!(select_best_velocity(Some(bad), Some(bad)), (0.0, 0.0, 0.0));
        }
        assert_eq!(select_best_velocity(None, None), (0.0, 0.0, 0.0));
        assert_eq!(
            select_best_velocity(None, Some((0.0, 2.0))),
            (0.0, 2.0, 2.0)
        );
        assert_eq!(
            select_best_velocity(None, Some((99.0, 0.0))),
            (99.0, 0.0, 99.0)
        );
    }
}
