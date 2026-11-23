//! Application boundary between receiver ingestion and latest-only estimation.

use crate::diagnostics::DiagnosticsState;
use ublox_gps::NmeaParser;

/// Retain boundaries from every parsed RMC even when estimation consumes only
/// the final fix of a drain. Consumers reset before accepting that final fix.
#[derive(Default)]
pub struct Continuity {
    previous: Option<ublox_gps::RmcTiming>,
    lost: bool,
}
impl Continuity {
    pub fn invalidate(&mut self) {
        self.previous = None;
        self.lost = true;
    }
    pub fn observe(&mut self, timing: ublox_gps::RmcTiming, valid: bool) {
        let Some(utc) = timing.utc_us.filter(|utc| *utc < 86_401_000_000) else {
            self.invalidate();
            return;
        };
        if !valid || !timing.status_active || timing.date_ddmmyy.is_none() {
            self.invalidate();
            return;
        }
        if self.previous.is_some_and(|p| {
            timing.sequence != p.sequence.wrapping_add(1)
                || timing.date_ddmmyy != p.date_ddmmyy
                || p.utc_us
                    .is_none_or(|old| utc <= old || utc - old > 1_500_000)
        }) {
            self.lost = true;
        }
        self.previous = Some(timing);
    }
    pub fn take_discontinuity(&mut self) -> bool {
        core::mem::take(&mut self.lost)
    }
}

/// Count each valid post-warmup RMC when parsed, before another sentence can
/// replace its evidence. Timing and logging retain the same per-RMC events.
pub fn feed_byte(parser: &mut NmeaParser, byte: u8, diagnostics: &DiagnosticsState) -> bool {
    let previous_sequence = parser.rmc_timing().sequence;
    let updated = parser.feed_byte(byte);
    if parser.rmc_timing().sequence != previous_sequence && parser.last_fix().valid {
        diagnostics.record_gps_fix();
    }
    updated
}

/// Estimation still consumes only the latest-fix flag after draining the UART.
pub fn take_latest_fix(parser: &mut NmeaParser) -> bool {
    parser.take_new_fix()
}

#[cfg(test)]
mod tests {
    use super::*;

    const VALID: &str = "GNRMC,123456.04,A,3723.2475,N,12158.3416,W,10.0,90.0,201126,,,A";
    const INVALID: &str = "GNRMC,123456.08,V,,,,,,,201126,,,N";
    const GGA: &str = "GNGGA,123456.04,3723.2475,N,12158.3416,W,1,08,0.9,10.0,M,0.0,M,,";
    const GSA: &str = "GNGSA,A,3,01,02,03,04,05,06,07,08,,,,,1.8,1.0,1.5";

    fn sentence(parser: &mut NmeaParser, diagnostics: &DiagnosticsState, body: &str) {
        let checksum = body.bytes().fold(0u8, |sum, byte| sum ^ byte);
        for byte in format!("${body}*{checksum:02X}\r\n").bytes() {
            feed_byte(parser, byte, diagnostics);
        }
    }

    fn warmed() -> (NmeaParser, DiagnosticsState) {
        let mut parser = NmeaParser::with_warmup_fixes(1);
        let diagnostics = DiagnosticsState::new();
        sentence(&mut parser, &diagnostics, VALID);
        assert!(parser.is_warmed_up());
        assert!(!take_latest_fix(&mut parser));
        assert_eq!(diagnostics.gps_fix_count(), 0);
        (parser, diagnostics)
    }

    #[test]
    fn invalid_then_valid_in_one_drain_preserves_a_boundary_for_latest_consumers() {
        let (mut parser, diagnostics) = warmed();
        let mut continuity = Continuity::default();
        for body in [VALID, INVALID, VALID] {
            sentence(&mut parser, &diagnostics, body);
            continuity.observe(parser.rmc_timing(), parser.last_fix().valid);
        }
        assert!(take_latest_fix(&mut parser));
        assert!(continuity.take_discontinuity());
        let mut acceleration = sensor_fusion::fusion::GpsAcceleration::new();
        acceleration.update_epoch(10.0, Some(1_000_000), Some(201126));
        acceleration.update_epoch(11.0, Some(1_040_000), Some(201126));
        // Same reset contract used by main before it feeds the retained final fix.
        acceleration.update_epoch(f32::NAN, None, None);
        assert_eq!(
            acceleration.update_epoch(12.0, Some(1_080_000), Some(201126)),
            None
        );
        assert!(!continuity.take_discontinuity());
    }

    #[test]
    fn every_valid_rmc_in_one_uart_drain_counts_once() {
        let (mut parser, diagnostics) = warmed();
        for _ in 0..3 {
            sentence(&mut parser, &diagnostics, VALID);
        }
        assert!(take_latest_fix(&mut parser));
        assert_eq!(diagnostics.gps_fix_count(), 3);
        assert!(!take_latest_fix(&mut parser));
        assert_eq!(diagnostics.gps_fix_count(), 3);
    }

    #[test]
    fn invalid_tail_does_not_erase_delivered_valid_fixes() {
        for bodies in [[VALID, VALID, INVALID], [VALID, INVALID, VALID]] {
            let (mut parser, diagnostics) = warmed();
            for body in bodies {
                sentence(&mut parser, &diagnostics, body);
            }
            assert_eq!(take_latest_fix(&mut parser), bodies[2] == VALID);
            assert_eq!(diagnostics.gps_fix_count(), 2);
            assert!(!take_latest_fix(&mut parser));
            assert_eq!(diagnostics.gps_fix_count(), 2);
        }
    }

    #[test]
    fn uart_loss_discards_partial_sentence_and_requires_a_new_valid_fix() {
        let (mut parser, diagnostics) = warmed();
        sentence(&mut parser, &diagnostics, VALID);
        for byte in b"$GNRMC,123" {
            feed_byte(&mut parser, *byte, &diagnostics);
        }
        parser.invalidate_stream();
        assert!(!parser.last_fix().valid);
        assert!(!take_latest_fix(&mut parser));
        sentence(&mut parser, &diagnostics, VALID);
        assert!(take_latest_fix(&mut parser));
        let sequence = parser.rmc_timing().sequence;
        for byte in b"$GNRMC,123456.04,A,3723.2475,N,12158.3416,W,10.0,90.0,201126,,,A*00\r\n" {
            feed_byte(&mut parser, *byte, &diagnostics);
        }
        assert_eq!(parser.checksum_rejections(), 1);
        assert_eq!(parser.rmc_timing().sequence, sequence);
    }

    #[test]
    fn warmup_non_rmc_and_bad_checksum_never_add_fixes() {
        let mut parser = NmeaParser::with_warmup_fixes(3);
        let diagnostics = DiagnosticsState::new();
        for _ in 0..3 {
            sentence(&mut parser, &diagnostics, VALID);
            assert!(!take_latest_fix(&mut parser));
        }
        for body in [GGA, GSA, INVALID] {
            sentence(&mut parser, &diagnostics, body);
        }
        for byte in b"$GNRMC,123456.04,A,3723.2475,N,12158.3416,W,10.0,90.0,201126,,,A*00\r\n" {
            feed_byte(&mut parser, *byte, &diagnostics);
        }
        assert!(!take_latest_fix(&mut parser));
        assert_eq!(diagnostics.gps_fix_count(), 0);
        sentence(&mut parser, &diagnostics, VALID);
        assert!(take_latest_fix(&mut parser));
        assert_eq!(diagnostics.gps_fix_count(), 1);
        // Non-RMC sentences must not recount the retained valid position.
        for body in [GGA, GSA] {
            sentence(&mut parser, &diagnostics, body);
        }
        assert!(!take_latest_fix(&mut parser));
        assert_eq!(diagnostics.gps_fix_count(), 1);
    }
}
