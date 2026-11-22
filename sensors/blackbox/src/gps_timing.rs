//! Bounded receiver evidence for timing, independent of latest-fix display state.
//! Times are host sentence-completion times, not receiver epochs or PPS times.

use sensor_fusion::LapTimer;
use ublox_gps::GpsFix;

const CAPACITY: usize = 16;
const MAX_AGE_US: u64 = 1_500_000;
// Broad gross-jump guard, not a receiver accuracy estimate. Preserve normal
// GNSS jitter and the application's existing 100 m/s supported speed ceiling.
const POSITION_MARGIN_M: f64 = 25.0;
const MAX_SPEED_MPS: f64 = 100.0;

#[derive(Clone, Copy)]
struct Evidence {
    fix: GpsFix,
    host_us: u64,
}

pub struct GpsTimingQueue {
    samples: [Option<Evidence>; CAPACITY],
    len: usize,
    discontinuity: bool,
    previous: Option<Evidence>,
}

impl GpsTimingQueue {
    pub fn new() -> Self {
        Self {
            samples: [None; CAPACITY],
            len: 0,
            discontinuity: false,
            previous: None,
        }
    }

    /// Preserve invalid RMC evidence too: it must break consecutive-fix timing.
    pub fn push(&mut self, fix: GpsFix, host_us: u64) {
        if self.len == CAPACITY {
            // Lost evidence must never join the old trajectory to the retained tail.
            self.samples.fill(None);
            self.len = 0;
            self.discontinuity = true;
        }
        self.samples[self.len] = Some(Evidence { fix, host_us });
        self.len += 1;
    }

    pub fn replay(&mut self, timer: &mut LapTimer, now_us: u64) -> u8 {
        if self.discontinuity {
            timer.invalidate_fix();
            self.previous = None;
            self.discontinuity = false;
        }
        let mut flags = 0;
        for sample in self.samples[..self.len].iter_mut() {
            let Evidence { fix, host_us } = sample.take().expect("queued evidence");
            let fresh = now_us
                .checked_sub(host_us)
                .is_some_and(|age| age <= MAX_AGE_US);
            if !fresh
                || !fix.valid
                || !fix.lat.is_finite()
                || !fix.lon.is_finite()
                || fix.lat.abs() > 90.0
                || fix.lon.abs() > 180.0
                || !fix.speed.is_finite()
                || !(0.0..=100.0).contains(&fix.speed)
                || !fix.course.is_finite()
                || !(0.0..=core::f32::consts::TAU).contains(&fix.course)
            {
                timer.invalidate_fix();
                self.previous = None;
                continue;
            }
            if self.previous.is_some_and(|previous| {
                // Millisecond timer cannot interpolate collapsed parse timestamps.
                if host_us / 1000 <= previous.host_us / 1000 {
                    return true;
                }
                let elapsed_us = host_us - previous.host_us;
                if elapsed_us > MAX_AGE_US {
                    return true;
                }
                let north = (fix.lat - previous.fix.lat) * 111320.0;
                let mean_lat = (fix.lat + previous.fix.lat) * 0.5;
                let east = (fix.lon - previous.fix.lon) * 111320.0 * mean_lat.to_radians().cos();
                let limit = POSITION_MARGIN_M + MAX_SPEED_MPS * elapsed_us as f64 * 1e-6;
                north * north + east * east > limit * limit
            }) {
                timer.invalidate_fix();
            }
            // A rejected segment never crosses a line; its endpoint starts a new
            // trajectory so ordinary consecutive evidence can recover promptly.
            self.previous = Some(Evidence { fix, host_us });
            let velocity = (fix.speed * fix.course.sin(), fix.speed * fix.course.cos());
            flags |= timer.update(
                fix.lat,
                fix.lon,
                velocity,
                (host_us / 1000) as u32,
                fix.speed,
            );
        }
        self.len = 0;
        flags
    }
}

impl Default for GpsTimingQueue {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sensor_fusion::{LapTimerState, TimingLine, CROSSED_START};

    fn timer() -> LapTimer {
        let mut timer = LapTimer::new();
        timer.configure_loop(TimingLine::new(
            0.0,
            -0.001,
            0.0,
            0.001,
            core::f32::consts::FRAC_PI_2,
        ));
        timer
    }

    fn fix(lat: f64) -> GpsFix {
        GpsFix {
            lat,
            lon: 0.0,
            speed: 10.0,
            course: 0.0,
            valid: true,
            ..GpsFix::default()
        }
    }

    #[test]
    fn intermediate_crossing_survives_batched_return_to_original_side() {
        let mut queue = GpsTimingQueue::new();
        let mut timer = timer();
        queue.push(fix(-0.0001), 1_000_000);
        queue.push(fix(0.0001), 1_040_000);
        queue.push(fix(-0.0001), 1_080_000);
        assert_ne!(queue.replay(&mut timer, 1_080_000) & CROSSED_START, 0);
        assert_eq!(timer.state(), LapTimerState::Timing);
        assert_eq!(queue.replay(&mut timer, 1_100_000), 0);
    }

    #[test]
    fn invalid_between_valid_fixes_breaks_crossing_even_in_one_batch() {
        let mut queue = GpsTimingQueue::new();
        let mut timer = timer();
        queue.push(fix(-0.0001), 1_000_000);
        queue.push(
            GpsFix {
                valid: false,
                ..fix(0.0)
            },
            1_020_000,
        );
        queue.push(fix(0.0001), 1_040_000);
        assert_eq!(queue.replay(&mut timer, 1_040_000), 0);
        assert_eq!(timer.state(), LapTimerState::Armed);
    }

    #[test]
    fn gross_position_jump_cannot_start_timing() {
        let mut queue = GpsTimingQueue::new();
        let mut timer = timer();
        // A 200 m segment in 40 ms implies 5,000 m/s, despite a valid 10 m/s RMC.
        queue.push(fix(-100.0 / 111320.0), 1_000_000);
        queue.push(fix(100.0 / 111320.0), 1_040_000);
        assert_eq!(queue.replay(&mut timer, 1_040_000), 0);
        assert_eq!(timer.state(), LapTimerState::Armed);
        // Returning near the line is also a rejected jump, not a bridge to the
        // old approach. A subsequent ordinary crossing recovers normally.
        queue.push(fix(-1.0 / 111320.0), 1_080_000);
        assert_eq!(queue.replay(&mut timer, 1_080_000), 0);
        queue.push(fix(1.0 / 111320.0), 1_120_000);
        assert_ne!(queue.replay(&mut timer, 1_120_000) & CROSSED_START, 0);
    }

    #[test]
    fn gross_position_jump_cannot_finish_a_stage() {
        let mut queue = GpsTimingQueue::new();
        let mut timer = LapTimer::new();
        let line = |lat| TimingLine::new(lat, -0.001, lat, 0.001, core::f32::consts::FRAC_PI_2);
        timer.configure_point_to_point(line(0.0), line(0.01));
        queue.push(fix(-1.0 / 111320.0), 1_000_000);
        queue.push(fix(1.0 / 111320.0), 1_040_000);
        assert_ne!(queue.replay(&mut timer, 1_040_000) & CROSSED_START, 0);
        queue.push(fix(0.01 - 100.0 / 111320.0), 12_000_000);
        queue.push(fix(0.01 + 100.0 / 111320.0), 12_040_000);
        assert_eq!(queue.replay(&mut timer, 12_040_000), 0);
        assert_eq!(timer.state(), LapTimerState::Timing);
        assert_eq!(timer.lap_count(), 0);
        queue.push(fix(0.01 - 1.0 / 111320.0), 12_080_000);
        assert_eq!(queue.replay(&mut timer, 12_080_000), 0);
        queue.push(fix(0.01 + 1.0 / 111320.0), 12_120_000);
        assert_ne!(
            queue.replay(&mut timer, 12_120_000) & sensor_fusion::NEW_LAP,
            0
        );
        assert_eq!(timer.lap_count(), 1);
    }

    #[test]
    fn gross_jump_bound_preserves_supported_speed_and_longitude_projection() {
        for (latitude, interval_us) in [(0.0, 40_000), (60.0, 200_000)] {
            let mut queue = GpsTimingQueue::new();
            let mut timer = LapTimer::new();
            let cos_lat = (latitude as f64).to_radians().cos();
            timer.configure_loop(TimingLine::new(
                latitude - 0.001,
                0.0,
                latitude + 0.001,
                0.0,
                0.0,
            ));
            let distance = 100.0 * interval_us as f64 * 1e-6;
            for (east, host_us) in [
                (-distance / 2.0, 1_000_000),
                (distance / 2.0, 1_000_000 + interval_us),
            ] {
                queue.push(
                    GpsFix {
                        lat: latitude,
                        lon: east / (111320.0 * cos_lat),
                        speed: 100.0,
                        course: core::f32::consts::FRAC_PI_2,
                        valid: true,
                        ..GpsFix::default()
                    },
                    host_us,
                );
            }
            assert_ne!(
                queue.replay(&mut timer, 1_000_000 + interval_us) & CROSSED_START,
                0
            );
        }
    }

    #[test]
    fn overflow_breaks_old_trajectory_before_retained_tail() {
        let mut queue = GpsTimingQueue::new();
        let mut timer = timer();
        queue.push(fix(-0.0001), 1_000_000);
        queue.replay(&mut timer, 1_000_000);
        for n in 0..=CAPACITY {
            queue.push(fix(0.0001), 1_040_000 + n as u64 * 40_000);
        }
        assert_eq!(queue.replay(&mut timer, 1_800_000), 0);
        assert_eq!(timer.state(), LapTimerState::Armed);
    }

    #[test]
    fn collapsed_backwards_stale_future_and_nonfinite_evidence_cannot_bridge() {
        for (middle, middle_us, end_us, now_us) in [
            (fix(0.0001), 1_000_999, 1_000_999, 1_000_999),
            (fix(0.0001), 900_000, 900_000, 1_000_000),
            (fix(0.0001), 2_600_000, 2_600_000, 2_600_000),
            (fix(0.0001), 1_040_000, 1_040_000, 3_000_000),
            (fix(0.0001), 1_040_000, 1_040_000, 1_020_000),
            (
                GpsFix {
                    speed: f32::NAN,
                    ..fix(0.0)
                },
                1_020_000,
                1_040_000,
                1_040_000,
            ),
        ] {
            let mut queue = GpsTimingQueue::new();
            let mut timer = timer();
            queue.push(fix(-0.0001), 1_000_000);
            queue.replay(&mut timer, 1_000_000);
            queue.push(middle, middle_us);
            if end_us != middle_us {
                queue.push(fix(0.0001), end_us);
            }
            assert_eq!(queue.replay(&mut timer, now_us), 0);
            assert_eq!(timer.state(), LapTimerState::Armed);
        }
    }

    #[test]
    fn diagonal_course_preserves_forward_and_reverse_crossing_directions() {
        for (course, start, end, expected_crossing) in [
            (
                3.0 * core::f32::consts::FRAC_PI_4,
                (0.000001, -0.000001),
                (-0.000001, 0.000001),
                true,
            ),
            (
                7.0 * core::f32::consts::FRAC_PI_4,
                (-0.000001, 0.000001),
                (0.000001, -0.000001),
                false,
            ),
        ] {
            let mut timer = LapTimer::new();
            timer.configure_loop(TimingLine::new(
                0.0,
                -0.001,
                0.0,
                0.001,
                -core::f32::consts::FRAC_PI_4,
            ));
            let mut queue = GpsTimingQueue::new();
            for (position, host_us) in [(start, 1_000_000), (end, 1_040_000)] {
                queue.push(
                    GpsFix {
                        lat: position.0,
                        lon: position.1,
                        speed: 10.0,
                        course,
                        valid: true,
                        ..GpsFix::default()
                    },
                    host_us,
                );
            }
            assert_eq!(
                queue.replay(&mut timer, 1_040_000) & CROSSED_START != 0,
                expected_crossing
            );
        }
    }
}
