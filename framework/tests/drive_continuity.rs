use sensor_fusion::fusion::OrientationCorrector;
use sensor_fusion::{
    ekf::{Ekf, UpdateResult},
    fusion::GpsAcceleration,
    position_recovery::{PositionRecovery, ReceiverEpoch},
};

#[test]
fn saturated_orientation_is_not_high_confidence() {
    let mut c = OrientationCorrector::new();
    for _ in 0..1000 {
        c.update(10.0, 1.0, 0.0, 0.0, 15.0, true, 0.033);
    }
    assert!(c.get_pitch_correction_deg().abs() > 14.0);
    assert!(c.get_pitch_confidence() < 0.1);
}

#[test]
fn receiver_derivative_is_precise_late_in_the_day_and_resets_at_boundaries() {
    let mut gps = GpsAcceleration::new();
    let origin = 83_000_000_000;
    assert_eq!(gps.update_epoch(10.0, Some(origin), Some(231126)), None);
    for i in 1..=100 {
        let a = gps
            .update_epoch(
                10.0 + i as f32 * 0.04,
                Some(origin + i * 40_000),
                Some(231126),
            )
            .unwrap();
        assert!((a - 1.0).abs() < 0.001);
    }
    assert_eq!(gps.update_epoch(15.0, Some(1), Some(241126)), None);
    assert_eq!(gps.update_epoch(15.0, Some(1), Some(241126)), None);
    assert_eq!(gps.update_epoch(15.0, None, None), None);
    assert_eq!(gps.update_epoch(15.0, Some(40_001), Some(241126)), None);
    assert_eq!(gps.update_epoch(f32::NAN, Some(80_001), Some(241126)), None);
    assert!(gps.get_accel().is_none());
}

#[test]
fn gated_position_recovers_only_after_sustained_coherent_fixes() {
    let mut recovery = PositionRecovery::default();
    let mut ekf = Ekf::new();
    ekf.reset_covariance([1.0; 7]);
    // An isolated jump never immediately resets the estimator.
    assert_eq!(ekf.update_position(500.0, 0.0), UpdateResult::Rejected);
    assert!(!recovery.observe([500.0, 0.0], [10.0, 0.0], epoch(1_000_000)));
    assert!(!recovery.observe([900.0, 0.0], [10.0, 0.0], epoch(1_040_000)));
    // A real returning trajectory must establish its own collection.
    for i in 0..=13 {
        let x = 500.0 + i as f32 * 0.4;
        let ready = recovery.observe([x, 0.0], [10.0, 0.0], epoch(2_000_000 + i * 40_000));
        assert_eq!(ready, i == 13);
        if ready {
            ekf.reacquire_position(x, 0.0);
            assert_eq!(ekf.update_position(x, 0.0), UpdateResult::Accepted);
        }
    }
    recovery.reset();
    for i in 0..20 {
        assert!(!recovery.observe([500.0, 0.0], [0.0, 0.0], epoch(i * 400_000)));
    }
    assert!(!recovery.observe([f32::NAN, 0.0], [0.0, 0.0], epoch(10_000_000)));
}

#[test]
fn short_stalls_and_long_coasts_preserve_elapsed_displacement() {
    let mut ekf = Ekf::new();
    ekf.x[3] = 10.0;
    for dt in [0.005, 0.07, 0.12] {
        ekf.predict(0.0, 0.0, 0.0, dt);
    }
    assert!((ekf.position().0 - 1.95).abs() < 1e-5);
    ekf.x[5] = 0.3;
    ekf.coast(2.0);
    assert!((ekf.position().0 - 21.95).abs() < 1e-4);
    assert!((ekf.x[3] - 10.0).abs() < 1e-5);
    assert!(ekf.covariance().iter().flatten().all(|v| v.is_finite()));
}

#[test]
fn orientation_learning_scales_with_time_not_call_count() {
    fn learn(dt: f32) -> f32 {
        let mut c = OrientationCorrector::new();
        for _ in 0..(3.0 / dt) as usize {
            let residual = -0.05 - c.get_pitch_correction_deg().to_radians();
            c.update(3.0 + 9.80665 * residual, 3.0, 0.0, 0.0, 15.0, true, dt);
        }
        c.get_pitch_correction_deg()
    }
    assert!((learn(0.01) - learn(0.05)).abs() < 0.01);
}

fn epoch(us: u64) -> ReceiverEpoch {
    ReceiverEpoch {
        host_us: us,
        utc_us: us,
        date: 231126,
        sequence: (us / 40000) as u32 + 1,
    }
}

#[test]
fn receiver_clock_boundaries_cannot_reacquire_position() {
    for mode in 0..3 {
        let mut recovery = PositionRecovery::default();
        for i in 0..50 {
            let mut e = epoch(1_000_000 + i * 40_000);
            match mode {
                0 => e.utc_us = 1_000_000,
                1 => e.utc_us = 9_000_000 - i * 40_000,
                _ => e.date += i as u32,
            }
            assert!(!recovery.observe([500.0, 0.0], [0.0, 0.0], e));
        }
    }
}
