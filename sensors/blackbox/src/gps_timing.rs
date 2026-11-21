//! Bounded receiver evidence for timing, independent of latest-fix display state.
//! Times are host sentence-completion times, not receiver epochs or PPS times.

use sensor_fusion::LapTimer;
use ublox_gps::GpsFix;

const CAPACITY: usize = 16;
const MAX_AGE_US: u64 = 1_500_000;

#[derive(Clone, Copy)]
struct Evidence {
    fix: GpsFix,
    host_us: u64,
}

pub struct GpsTimingQueue {
    samples: [Option<Evidence>; CAPACITY],
    len: usize,
    discontinuity: bool,
    previous_us: Option<u64>,
}

impl GpsTimingQueue {
    pub fn new() -> Self {
        Self {
            samples: [None; CAPACITY],
            len: 0,
            discontinuity: false,
            previous_us: None,
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
            self.previous_us = None;
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
                self.previous_us = None;
                continue;
            }
            if self.previous_us.is_some_and(|previous| {
                // Millisecond timer cannot interpolate collapsed parse timestamps.
                host_us / 1000 <= previous / 1000 || host_us - previous > MAX_AGE_US
            }) {
                timer.invalidate_fix();
            }
            self.previous_us = Some(host_us);
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
