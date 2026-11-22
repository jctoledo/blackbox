use sensor_fusion::{Ekf, FusionConfig, HeadingAligner, SensorFusion};
use std::f32::consts::{FRAC_PI_2, PI};

fn angle_error(actual: f32, expected: f32) -> f32 {
    (actual - expected)
        .sin()
        .atan2((actual - expected).cos())
        .abs()
}

#[test]
fn unlearned_heading_converts_basis_without_claiming_alignment() {
    let aligner = HeadingAligner::new();
    for (yaw, course) in [
        (0.0, FRAC_PI_2),
        (FRAC_PI_2, 0.0),
        (PI, -FRAC_PI_2),
        (-FRAC_PI_2, PI),
    ] {
        assert!(angle_error(aligner.get_aligned_heading(yaw), course) < 1e-6);
        assert!(!aligner.is_valid());
    }
}

#[test]
fn learned_offset_keeps_turn_sense_when_low_speed_prevents_relearning() {
    let mut aligner = HeadingAligner::new();
    aligner.update_gps_course(0.0);
    assert!(aligner.update(FRAC_PI_2 + 0.7, 15.0, 0.05));
    for step in 0..=100 {
        let turn = step as f32 * 0.05;
        let yaw = FRAC_PI_2 + 0.7 + turn;
        assert!(!aligner.update(yaw, 2.0, 0.05));
        assert!(angle_error(aligner.get_aligned_heading(yaw), -turn) < 1e-5);
    }
}

#[test]
fn aligned_heading_follows_cardinal_turns_in_compass_sense() {
    for (offset, direction) in [(0.0, 1.0), (0.7, -1.0), (-0.7, 1.0)] {
        let mut ekf = Ekf::new();
        ekf.x[2] = FRAC_PI_2 + offset;
        let mut fusion = SensorFusion::new(FusionConfig::default());
        fusion.process_gps_heading(0.0); // North, clockwise GPS convention.
        fusion.process_imu(
            0.0,
            0.0,
            9.80665,
            0.0,
            0.0,
            ekf.yaw(),
            15.0,
            0.0,
            0.05,
            false,
        );
        for step in 1..=256 {
            let rate = direction * PI / 4.0;
            let turn = step as f32 * rate * 0.05;
            ekf.predict(0.0, 0.0, rate, 0.05);
            let course = -turn; // Positive earth-Z turns counterclockwise.
            fusion.process_gps_heading(course.sin().atan2(course.cos()));
            fusion.process_imu(
                0.0,
                0.0,
                9.80665,
                0.0,
                0.0,
                ekf.yaw(),
                15.0,
                rate,
                0.05,
                false,
            );
            assert!(angle_error(fusion.get_aligned_heading(), course) < 0.001,
                "opposite-sense compass heading: offset={offset}, step={step}, actual={}, expected={course}", fusion.get_aligned_heading());
        }
    }
}
