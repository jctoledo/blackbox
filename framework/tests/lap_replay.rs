use sensor_fusion::{LapTimer, TimingLine};
use std::f64::consts::{PI, TAU};

// A physical trajectory exercises lat/lon projection, all crossing headings,
// variable GPS sample spacing, and interpolated lap results together.
#[test]
fn circular_track_replay_is_independent_of_heading_and_latitude() {
    for latitude in [-55.0_f64, 0.0, 37.0, 65.0] {
        for heading in 0..8 {
            let angle = heading as f64 * TAU / 8.0;
            let radius = 50.0;
            let speed = 10.0;
            let longitude = -122.0;
            let gps = |x: f64, y: f64| {
                (
                    latitude + y / 111320.0,
                    longitude + x / (111320.0 * latitude.to_radians().cos()),
                )
            };
            let a = gps(38.0 * angle.cos(), 38.0 * angle.sin());
            let b = gps(62.0 * angle.cos(), 62.0 * angle.sin());
            let mut timer = LapTimer::new();
            timer.configure_loop(TimingLine::new(
                a.0,
                a.1,
                b.0,
                b.1,
                (angle + PI / 2.0) as f32,
            ));
            let mut timestamp = 0_u32;
            let mut i = 0;
            while (timestamp as f64 / 1000.0) * speed / radius < 2.0 * TAU + 0.2 {
                let phase = angle - 0.1 + timestamp as f64 / 1000.0 * speed / radius;
                let position = gps(radius * phase.cos(), radius * phase.sin());
                timer.update(
                    position.0,
                    position.1,
                    ((-speed * phase.sin()) as f32, (speed * phase.cos()) as f32),
                    timestamp,
                    speed as f32,
                );
                timestamp += [40, 80, 120, 60, 200][i % 5];
                i += 1;
            }
            assert_eq!(
                timer.lap_count(),
                2,
                "latitude={latitude}, heading={heading}"
            );
            let expected = TAU * radius / speed * 1000.0;
            assert!((timer.last_lap_ms().unwrap() as f64 - expected).abs() < 3.0);
            assert!((timer.best_lap_ms().unwrap() as f64 - expected).abs() < 3.0);
        }
    }
}

#[test]
fn invalid_fix_breaks_the_crossing_segment() {
    let mut timer = LapTimer::new();
    timer.configure_loop(TimingLine::new(
        37.0,
        -122.0001,
        37.0,
        -121.9999,
        std::f32::consts::FRAC_PI_2,
    ));
    timer.update(36.99999, -122.0, (0.0, 10.0), 1000, 10.0);
    timer.invalidate_fix();
    assert_eq!(timer.update(37.00001, -122.0, (0.0, 10.0), 1200, 10.0), 0);
    assert_eq!(timer.current_lap_ms(), 0);
    assert_eq!(timer.lap_count(), 0);
}

#[test]
fn twenty_mile_stage_replay_finishes_once_at_the_interpolated_crossing() {
    let latitude = 37.0;
    let longitude = -122.0;
    let distance = 32_186.88;
    let speed = 25.0;
    let line = |north: f64| {
        TimingLine::new(
            latitude + north / 111320.0,
            longitude - 0.0001,
            latitude + north / 111320.0,
            longitude + 0.0001,
            (PI / 2.0) as f32,
        )
    };
    let mut timer = LapTimer::new();
    timer.configure_point_to_point(line(0.0), line(distance));
    let mut timestamp = 0_u32;
    while timestamp < 1_300_000 {
        let north = -5.0 + speed * timestamp as f64 / 1000.0;
        timer.update(
            latitude + north / 111320.0,
            longitude,
            (0.0, speed as f32),
            timestamp,
            speed as f32,
        );
        timestamp += 200;
    }
    assert_eq!(timer.lap_count(), 1);
    let expected = (distance / speed * 1000.0_f64).round() as u32;
    assert!(timer.last_lap_ms().unwrap().abs_diff(expected) <= 2);
    assert_eq!(timer.current_lap_ms(), 0);
}

#[test]
fn stage_never_fabricates_a_finish_across_a_gps_outage() {
    let line = |latitude| TimingLine::new(latitude, -0.0001, latitude, 0.0001, (PI / 2.0) as f32);
    let mut timer = LapTimer::new();
    timer.configure_point_to_point(line(0.0), line(0.001));
    timer.update(-0.00001, 0.0, (0.0, 10.0), 1000, 10.0);
    timer.update(0.00001, 0.0, (0.0, 10.0), 1200, 10.0);
    timer.update(0.0009, 0.0, (0.0, 10.0), 10_000, 10.0);
    timer.invalidate_fix();
    timer.update(0.0011, 0.0, (0.0, 10.0), 15_000, 10.0);
    assert_eq!(timer.lap_count(), 0);
    assert_eq!(timer.last_lap_ms(), None);
}
