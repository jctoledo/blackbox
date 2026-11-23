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

#[test]
fn prediction_propagates_velocity_bias_and_bias_walk_with_correct_units() {
    let mut ekf = Ekf::new();
    ekf.p = [1.0, 2.0, 0.1, 4.0, 9.0, 0.2, 0.3];
    ekf.predict(0.0, 0.0, 0.0, 0.5);
    // Independent analytic F P F^T + Q for x += vx*dt - bax*dt²/2.
    // Q includes the continuous bias random walk integrated through position/velocity.
    assert!(
        (ekf.p[0] - (1.0 + 4.0 * 0.25 + 0.25 * 0.6 * 0.0625 + 0.001 * 0.03125 / 20.0)).abs() < 1e-6
    );
    assert!(
        (ekf.p[1] - (2.0 + 9.0 * 0.25 + 0.25 * 0.7 * 0.0625 + 0.001 * 0.03125 / 20.0)).abs() < 1e-6
    );
    assert!((ekf.p[3] - (4.15 + 0.001 * 0.125 / 3.0)).abs() < 1e-6);
    assert!((ekf.p[4] - (9.175 + 0.001 * 0.125 / 3.0)).abs() < 1e-6);
}

#[test]
fn corrupt_measurements_do_not_poison_the_estimator() {
    use sensor_fusion::ekf::UpdateResult;
    let mut ekf = Ekf::new();
    for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        assert_eq!(ekf.update_position(bad, 0.0), UpdateResult::Rejected);
        assert_eq!(ekf.update_yaw(bad), UpdateResult::Rejected);
        assert_eq!(ekf.update_yaw_soft(0.0, bad), UpdateResult::Rejected);
        ekf.update_velocity(0.0, bad);
        ekf.update_speed(bad);
        ekf.update_bias(bad, 0.0);
        assert_eq!(ekf.x, [0.0; 7]);
        assert!(ekf.p.iter().all(|v| v.is_finite() && *v >= 0.0));
    }
    ekf.update_velocity(12.0, 0.0);
    assert!(
        ekf.speed() > 11.0,
        "Valid velocity must still recover after bad input"
    );
}

#[test]
fn cruise_bias_converges_to_full_offset_instead_of_half() {
    use sensor_fusion::fusion::OrientationCorrector;
    for offset in [-1.0, 0.8] {
        let mut corrector = OrientationCorrector::new();
        for _ in 0..10000 {
            corrector.update(offset, 0.0, 0.0, 0.0, 15.0, true, 0.033);
        }
        assert!((corrector.get_cruise_bias() - offset).abs() < 0.01);
        corrector.update(f32::NAN, 0.0, 0.0, 0.0, 15.0, true, 0.033);
        assert!((corrector.get_cruise_bias() - offset).abs() < 0.01);
    }
}

#[test]
fn mount_projection_preserves_vehicle_forces_for_all_horizontal_angles() {
    use sensor_fusion::fusion::{FusionConfig, SensorFusion};
    for degrees in [-179.0f32, -90.0, -45.0, 0.0, 45.0, 90.0, 179.0] {
        let angle = degrees.to_radians();
        for (lon, lat) in [(-3.0, 0.0), (3.0, 0.0), (0.0, 4.0), (2.0, -2.0)] {
            let mut f = SensorFusion::new(FusionConfig::default());
            f.set_mount_yaw(angle);
            let x = angle.cos() * lon - angle.sin() * lat;
            let y = angle.sin() * lon + angle.cos() * lat;
            f.process_imu(x, y, 9.80665, 0.0, 0.0, 1.2, 15.0, 0.0, 0.033, false);
            assert!((f.get_lon_imu_raw() - lon).abs() < 1e-5, "angle {degrees}");
            assert!((f.get_lat_display() - lat).abs() < 1e-5, "angle {degrees}");
        }
    }
}
