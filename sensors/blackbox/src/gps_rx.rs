//! Drain policy shared with host tests: never commit a trajectory across UART loss.
use crate::diagnostics::GpsTransportHealth;

#[derive(Debug)]
pub struct ReadError;
pub trait Receiver {
    fn read(&mut self, out: &mut [u8]) -> Result<usize, ReadError>;
    fn observe_loss(&mut self, counters: &mut GpsTransportHealth) -> bool;
    fn discard(&mut self) -> Result<(), ReadError>;
}
pub struct Drain {
    pub counters: GpsTransportHealth,
    pub loss: bool,
}

/// The caller must invalidate all parser/timing evidence if `loss` is true,
/// before replaying any of the consumed fixes. Raw parsed messages may be logged.
pub fn drain(rx: &mut impl Receiver, mut consume: impl FnMut(&[u8])) -> Drain {
    let mut result = Drain {
        counters: GpsTransportHealth::default(),
        loss: false,
    };
    if rx.observe_loss(&mut result.counters) {
        result.loss = true;
    } else {
        let mut buffer = [0u8; 256];
        // Yield even if a faulty source continuously floods the UART.
        while result.counters.rx_bytes < 8192 {
            match rx.read(&mut buffer) {
                Ok(0) => break,
                Ok(n) => {
                    result.counters.rx_bytes += n as u32;
                    consume(&buffer[..n]);
                }
                Err(_) => {
                    result.counters.read_errors += 1;
                    result.loss = true;
                    break;
                }
            }
        }
        // Events raised during this drain must be handled before timing replay.
        result.loss |= rx.observe_loss(&mut result.counters);
    }
    if result.loss && rx.discard().is_err() {
        result.counters.read_errors += 1;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gps_timing::GpsTimingQueue;
    use sensor_fusion::{LapTimer, LapTimerState, TimingLine};
    use ublox_gps::GpsFix;
    struct Fake {
        before: bool,
        after: bool,
        reads: usize,
        discarded: bool,
    }
    impl Receiver for Fake {
        fn read(&mut self, out: &mut [u8]) -> Result<usize, ReadError> {
            self.reads += 1;
            if self.reads > 1 {
                return Ok(0);
            }
            out[..2].copy_from_slice(&[1, 2]);
            Ok(2)
        }
        fn observe_loss(&mut self, counters: &mut GpsTransportHealth) -> bool {
            let loss = self.before || self.after && self.reads > 0;
            if loss {
                counters.fifo_overflows += 1;
            }
            loss
        }
        fn discard(&mut self) -> Result<(), ReadError> {
            self.discarded = true;
            Ok(())
        }
    }
    #[test]
    fn buffered_fixes_on_both_sides_of_loss_never_make_a_crossing() {
        for before in [false, true] {
            let mut rx = Fake {
                before,
                after: !before,
                reads: 0,
                discarded: false,
            };
            let mut queue = GpsTimingQueue::new();
            let mut timer = LapTimer::new();
            timer.configure_loop(TimingLine::new(
                0.,
                -0.001,
                0.,
                0.001,
                core::f32::consts::FRAC_PI_2,
            ));
            let outcome = drain(&mut rx, |bytes| {
                for byte in bytes {
                    queue.push(
                        GpsFix {
                            lat: if *byte == 1 { -0.0001 } else { 0.0001 },
                            lon: 0.,
                            valid: true,
                            speed: 10.,
                            course: 0.,
                            ..Default::default()
                        },
                        1_000_000 + *byte as u64 * 40_000,
                    );
                }
            });
            if outcome.loss {
                queue.invalidate();
            }
            assert!(outcome.loss);
            assert!(rx.discarded);
            assert_eq!(queue.replay(&mut timer, 1_080_000), 0);
            assert_eq!(timer.state(), LapTimerState::Armed);
            if before {
                assert_eq!(rx.reads, 0);
            }
        }
    }
    #[test]
    fn empty_reads_are_not_errors_or_loss() {
        let mut rx = Fake {
            before: false,
            after: false,
            reads: 1,
            discarded: false,
        };
        let outcome = drain(&mut rx, |_| panic!("no bytes expected"));
        assert!(!outcome.loss);
        assert_eq!(outcome.counters.read_errors, 0);
        assert!(!rx.discarded);
    }
}
