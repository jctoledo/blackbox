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
