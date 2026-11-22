//! Application boundary between receiver ingestion and latest-only estimation.

use crate::diagnostics::DiagnosticsState;
use ublox_gps::NmeaParser;

/// Parse every byte; timing and logging retain the parser's per-RMC events.
pub fn feed_byte(parser: &mut NmeaParser, byte: u8, _diagnostics: &DiagnosticsState) -> bool {
    parser.feed_byte(byte)
}

/// Consume the latest-fix flag once after draining the UART.
pub fn take_latest_fix(parser: &mut NmeaParser, diagnostics: &DiagnosticsState) -> bool {
    let available = parser.take_new_fix();
    if available {
        diagnostics.record_gps_fix();
    }
    available
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
        assert!(!take_latest_fix(&mut parser, &diagnostics));
        assert_eq!(diagnostics.gps_fix_count(), 0);
        (parser, diagnostics)
    }

    #[test]
    fn every_valid_rmc_in_one_uart_drain_counts_once() {
        let (mut parser, diagnostics) = warmed();
        for _ in 0..3 {
            sentence(&mut parser, &diagnostics, VALID);
        }
        assert!(take_latest_fix(&mut parser, &diagnostics));
        assert_eq!(diagnostics.gps_fix_count(), 3);
        assert!(!take_latest_fix(&mut parser, &diagnostics));
        assert_eq!(diagnostics.gps_fix_count(), 3);
    }

    #[test]
    fn invalid_tail_does_not_erase_delivered_valid_fixes() {
        for bodies in [[VALID, VALID, INVALID], [VALID, INVALID, VALID]] {
            let (mut parser, diagnostics) = warmed();
            for body in bodies {
                sentence(&mut parser, &diagnostics, body);
            }
            assert_eq!(
                take_latest_fix(&mut parser, &diagnostics),
                bodies[2] == VALID
            );
            assert_eq!(diagnostics.gps_fix_count(), 2);
            assert!(!take_latest_fix(&mut parser, &diagnostics));
            assert_eq!(diagnostics.gps_fix_count(), 2);
        }
    }

    #[test]
    fn warmup_non_rmc_and_bad_checksum_never_add_fixes() {
        let mut parser = NmeaParser::with_warmup_fixes(3);
        let diagnostics = DiagnosticsState::new();
        for _ in 0..3 {
            sentence(&mut parser, &diagnostics, VALID);
            assert!(!take_latest_fix(&mut parser, &diagnostics));
        }
        for body in [GGA, GSA, INVALID] {
            sentence(&mut parser, &diagnostics, body);
        }
        for byte in b"$GNRMC,123456.04,A,3723.2475,N,12158.3416,W,10.0,90.0,201126,,,A*00\r\n" {
            feed_byte(&mut parser, *byte, &diagnostics);
        }
        assert!(!take_latest_fix(&mut parser, &diagnostics));
        assert_eq!(diagnostics.gps_fix_count(), 0);
        sentence(&mut parser, &diagnostics, VALID);
        assert!(take_latest_fix(&mut parser, &diagnostics));
        assert_eq!(diagnostics.gps_fix_count(), 1);
        // Non-RMC sentences must not recount the retained valid position.
        for body in [GGA, GSA] {
            sentence(&mut parser, &diagnostics, body);
        }
        assert!(!take_latest_fix(&mut parser, &diagnostics));
        assert_eq!(diagnostics.gps_fix_count(), 1);
    }
}
