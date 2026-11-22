use blackbox_host_tests::imu_bias_collection::BiasCollection;
use wt901::ImuData;

fn stationary() -> ImuData {
    ImuData {
        az: 9.80665,
        ..Default::default()
    }
}
fn fresh(c: &mut BiasCollection, t: u64) -> Option<bool> {
    c.observe_accel(t, &stationary(), t, t)
}

#[test]
fn recovered_fresh_gyro_and_angle_do_not_bridge_acceleration_silence() {
    let mut c = BiasCollection::new(1_000_000);
    for i in 0..149 {
        assert_eq!(fresh(&mut c, 1_000_000 + i * 10_000), Some(true));
    }
    assert!(!c.is_complete());
    assert_eq!(fresh(&mut c, 12_480_000), Some(true));
    assert!(
        !c.is_complete(),
        "149 old samples plus one recovered sample completed calibration"
    );
    for i in 1..149 {
        fresh(&mut c, 12_480_000 + i * 10_000);
        assert!(!c.is_complete());
    }
    fresh(&mut c, 13_970_000);
    assert!(c.is_complete());
    assert!(c.compute_bias().is_some());
}

#[test]
fn spacing_and_acceleration_gap_boundaries_use_receive_microseconds() {
    for start in [1_000_000, u32::MAX as u64 * 1000 + 1_000_000] {
        let mut c = BiasCollection::new(start);
        fresh(&mut c, start);
        for offset in [0, 1, 9_999] {
            assert_eq!(fresh(&mut c, start + offset), None);
        }
        assert_eq!(fresh(&mut c, start + 10_000), Some(true));
        assert_eq!(c.progress(), 2.0 / 150.0);
        assert_eq!(fresh(&mut c, start - 1), None);
        assert_eq!(c.progress(), 2.0 / 150.0);
    }
    for gap in [200_000, 200_001] {
        let mut c = BiasCollection::new(1_000_000);
        for i in 0..149 {
            fresh(&mut c, 1_000_000 + i * 10_000);
        }
        fresh(&mut c, 2_480_000 + gap);
        assert_eq!(c.is_complete(), gap == 200_000);
        if gap > 200_000 {
            assert_eq!(c.progress(), 1.0 / 150.0);
        }
    }
    // The gap is between acceleration packets, including packets omitted by
    // the 100 Hz sampling bound, rather than between selected bias samples.
    let mut c = BiasCollection::new(1_000_000);
    fresh(&mut c, 1_000_000);
    assert_eq!(fresh(&mut c, 1_009_999), None);
    fresh(&mut c, 1_209_999);
    assert_eq!(c.progress(), 2.0 / 150.0);
}

#[test]
fn stale_missing_invalid_and_moving_samples_require_a_full_fresh_collection() {
    for kind in 0..9 {
        let mut c = BiasCollection::new(1_000_000);
        for i in 0..149 {
            fresh(&mut c, 1_000_000 + i * 10_000);
        }
        let t = 2_490_000;
        let mut d = stationary();
        let (mut angle, mut gyro) = (t, t);
        match kind {
            0 => angle = t - 200_001,
            1 => gyro = t - 200_001,
            2 => angle = 0,
            3 => gyro = 0,
            4 => d.ax = f32::NAN,
            5 => d.wy = f32::INFINITY,
            6 => d.az += 2.0,
            7 => d.wx = 0.04,
            _ => d.roll = 21.0,
        }
        assert_eq!(
            c.observe_accel(t, &d, angle, gyro),
            Some(false),
            "kind {kind}"
        );
        assert_eq!(c.progress(), 0.0);
        for i in 1..150 {
            fresh(&mut c, t + i * 10_000);
            assert!(!c.is_complete());
        }
        fresh(&mut c, t + 1_500_000);
        assert!(c.is_complete());
        let bias = c.compute_bias().unwrap();
        assert_eq!((bias.ax, bias.ay, bias.az), (0.0, 0.0, 0.0));
    }
    let mut c = BiasCollection::new(1_000_000);
    assert_eq!(
        c.observe_accel(1_000_000, &stationary(), 800_000, 800_000),
        Some(true)
    );
}

#[test]
fn interruptions_do_not_extend_the_overall_deadline_or_return_a_bias() {
    let start = 1_000_000;
    let mut c = BiasCollection::new(start);
    for i in 0..149 {
        fresh(&mut c, start + i * 10_000);
    }
    // A late recovery resets collection but does not restart the 30 s clock.
    fresh(&mut c, start + 29_000_000);
    assert!(!c.expired(start + 30_000_000));
    assert!(c.expired(start + 30_000_001));
    assert_eq!(fresh(&mut c, start + 30_000_001), None);
    assert!(!c.is_complete());
    assert!(c.compute_bias().is_none());
    // Existing installed bias belongs to the parser, not the failed collection.
    let mut parser = wt901::Wt901Parser::new();
    parser.set_bias(wt901::ImuBias {
        ax: 0.1,
        ay: -0.2,
        az: 0.3,
    });
    if let Some(bias) = c.compute_bias() {
        parser.set_bias(bias);
    }
    let saved = parser.bias();
    assert_eq!((saved.ax, saved.ay, saved.az), (0.1, -0.2, 0.3));
}
