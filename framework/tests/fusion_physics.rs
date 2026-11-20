use sensor_fusion::ekf::Ekf;
use sensor_fusion::fusion::GpsAcceleration;

#[test]
fn earth_frame_velocity_is_not_rotated_a_second_time_in_a_turn() {
    let mut ekf = Ekf::new();
    ekf.x[2] = std::f32::consts::FRAC_PI_2;
    ekf.x[3] = 0.0;
    ekf.x[4] = 10.0;
    // Northbound at 10m/s, gentle left turn. Over 0.1s expect ~1m north.
    ekf.predict(-0.1, 0.0, 0.01, 0.1);
    let (east, north) = ekf.position();
    assert!(east.abs() < 0.01, "Spurious lateral motion: {east}");
    assert!((north - 1.0).abs() < 0.01, "Lost forward motion: {north}");
}

#[test]
fn invalid_gps_sample_interval_does_not_refresh_old_acceleration() {
    let mut gps = GpsAcceleration::new();
    gps.update(10.0, 1.0);
    assert!(gps.update(10.2, 1.1).is_some());
    // A duplicate timestamp must invalidate the derivative, not resurrect the old value.
    assert!(gps.update(30.0, 1.1).is_none());
    assert!(gps.get_accel().is_none());
    assert!(!gps.is_fresh(0.4));
}

#[test]
fn nonfinite_gps_speed_cannot_poison_fusion() {
    let mut gps = GpsAcceleration::new();
    gps.update(10.0, 1.0);
    assert!(gps.update(f32::NAN, 1.1).is_none());
    assert!(gps.get_accel().is_none());
}

#[test]
fn orientation_feedback_removes_persistent_pitch_and_roll_error() {
    use sensor_fusion::fusion::OrientationCorrector;
    use sensor_fusion::transforms::{body_to_earth, remove_gravity};
    for (pitch, roll, lon, lat) in [(3.0, 0.0, 3.0, 0.0), (0.0, 3.0, 0.0, 5.0)] {
        let mut corrector = OrientationCorrector::new();
        let mut residual = 0.0;
        for _ in 0..500 {
            let (p, r) = corrector.correct(pitch, roll);
            let (ax, ay, az) = remove_gravity(lon, lat, 9.80665, r, p);
            let (x, y) = body_to_earth(ax, ay, az, r, p, 0.0);
            corrector.update(x, lon, y, lat, 15.0, true, 0.033);
            residual = (x - lon).abs() + (y - lat).abs();
        }
        assert!(
            residual < 0.01,
            "Correction left {residual} m/s² error for pitch={pitch}, roll={roll}"
        );
        if lat > 0.0 {
            assert!(corrector.get_roll_confidence() > 0.9);
        }
    }
}
