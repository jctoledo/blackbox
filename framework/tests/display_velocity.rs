use sensor_fusion::velocity::select_best_velocity;

#[test]
fn fresh_receiver_speed_beats_an_estimator_disagreement() {
    // Synthetic disagreement: source selection must use the fresh receiver.
    let ekf_ms = 10.0_f32;
    let gps_ms = 5.0_f32;
    let (vx, vy, speed) = select_best_velocity(Some((ekf_ms, 0.0)), Some((gps_ms, 0.0)));
    assert!((speed * 3.6 - 18.0).abs() < 0.0001);
    assert_eq!((vx, vy), (gps_ms, 0.0));
}

#[test]
fn valid_receiver_zero_overrides_estimator_drift() {
    assert_eq!(
        select_best_velocity(Some((2.0, 1.0)), Some((0.0, 0.0))),
        (0.0, 0.0, 0.0)
    );
}
