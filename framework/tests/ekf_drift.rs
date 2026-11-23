use sensor_fusion::ekf::{Ekf, EkfConfig};

#[test]
fn constant_bias_uncertainty_accumulates_over_elapsed_time() {
    let mut filter = Ekf::with_config(EkfConfig {
        q_acc: 0.0,
        q_gyro: 0.0,
        q_bias: 0.0,
        ..EkfConfig::default()
    });
    filter.p = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0];
    for _ in 0..200 {
        filter.predict(0.0, 0.0, 0.0, 0.005);
    }
    // A single uncertain but persistent bias: dv=-b*T, dx=-b*T^2/2.
    assert!(
        (filter.p[3] - 1.0).abs() < 0.0001,
        "velocity variance {}",
        filter.p[3]
    );
    assert!(
        (filter.p[0] - 0.25).abs() < 0.0001,
        "position variance {}",
        filter.p[0]
    );
}

#[test]
fn moving_gps_observations_learn_persistent_earth_acceleration_bias() {
    for gps_every in [8, 40] {
        // 25 Hz and 5 Hz, with 200 Hz prediction.
        let mut filter = Ekf::new();
        filter.x[3] = 5.0;
        filter.x[4] = 2.0;
        for tick in 1..=12_000 {
            filter.predict(0.6, -0.35, 0.0, 0.005);
            if tick % gps_every == 0 {
                filter.update_velocity(5.0, 2.0);
            }
        }
        let (vx, vy) = filter.velocity();
        let (bx, by) = filter.biases();
        assert!(
            (vx - 5.0).abs() < 0.05 && (vy - 2.0).abs() < 0.05,
            "velocity {vx},{vy}, GPS period {gps_every}"
        );
        assert!(
            (bx - 0.6).abs() < 0.02 && (by + 0.35).abs() < 0.02,
            "bias {bx},{by}"
        );
    }
}

#[test]
fn prediction_splitting_preserves_persistent_bias_variance() {
    let run = |hz: usize| {
        let mut filter = Ekf::with_config(EkfConfig {
            q_acc: 0.0,
            q_gyro: 0.0,
            q_bias: 0.0,
            ..EkfConfig::default()
        });
        filter.p = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0];
        for _ in 0..hz * 10 {
            filter.predict(0.0, 0.0, 0.0, 1.0 / hz as f32)
        }
        filter.p
    };
    let a = run(50);
    let b = run(200);
    assert!((a[3] - 100.0).abs() < 0.02 && (b[3] - 100.0).abs() < 0.02);
    assert!((a[0] - 2500.0).abs() < 0.5 && (b[0] - 2500.0).abs() < 0.5);
}
