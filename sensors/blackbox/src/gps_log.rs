//! Bounded evidence, captured at completed RMC parsing (not first UART bit/PPS).
use std::collections::VecDeque;
use ublox_gps::{GpsFix, RmcTiming};
pub const GPS_LOG_CAPACITY: usize = 64;
pub const GPS_HISTORY_CAPACITY: usize = 512;
#[derive(Clone, Copy, Debug, serde::Serialize)]
pub struct GpsEvent {
    pub seq: u64,
    pub host_us: u64,
    pub utc_us: Option<u64>,
    pub date_ddmmyy: Option<u32>,
    pub fractional_digits: Option<u8>,
    pub active: bool,
    pub accepted: bool,
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    pub speed_mps: Option<f32>,
    pub course_rad: Option<f32>,
}
#[derive(Clone, serde::Serialize)]
pub struct GpsLogSnapshot {
    pub version: u8,
    pub simulated: bool,
    pub boot_id: u32,
    pub capacity: usize,
    pub latest_seq: u64,
    pub events: Vec<GpsEvent>,
    pub has_more: bool,
}
pub struct GpsLog {
    boot_id: u32,
    sequence: u64,
    events: VecDeque<GpsEvent>,
}
impl GpsLog {
    pub fn new(boot_id: u32) -> Self {
        Self {
            boot_id,
            sequence: 0,
            events: VecDeque::with_capacity(GPS_HISTORY_CAPACITY),
        }
    }
    pub fn push(&mut self, timing: RmcTiming, fix: &GpsFix, host_us: u64) {
        self.sequence += 1;
        let event = GpsEvent {
            seq: self.sequence,
            host_us,
            utc_us: timing.utc_us,
            date_ddmmyy: timing.date_ddmmyy,
            fractional_digits: timing.fractional_digits,
            active: timing.status_active,
            accepted: fix.valid,
            lat: fix.valid.then_some(fix.lat),
            lon: fix.valid.then_some(fix.lon),
            speed_mps: fix.valid.then_some(fix.speed),
            course_rad: fix.valid.then_some(fix.course),
        };
        if self.events.len() == GPS_HISTORY_CAPACITY {
            self.events.pop_front();
        }
        self.events.push_back(event);
    }
    pub fn latest(&self) -> (u32, Option<GpsEvent>) {
        (self.boot_id, self.events.back().copied())
    }
    pub fn snapshot_since(&self, cursor: Option<(u32, u64)>) -> GpsLogSnapshot {
        let after = cursor
            .filter(|(boot, seq)| *boot == self.boot_id && *seq <= self.sequence)
            .map_or(
                self.sequence.saturating_sub(GPS_LOG_CAPACITY as u64),
                |(_, seq)| seq,
            );
        let events: Vec<_> = self
            .events
            .iter()
            .filter(|e| e.seq > after)
            .take(GPS_LOG_CAPACITY)
            .copied()
            .collect();
        let latest_seq = events.last().map_or(self.sequence, |e| e.seq);
        GpsLogSnapshot {
            version: 1,
            simulated: false,
            boot_id: self.boot_id,
            capacity: GPS_LOG_CAPACITY,
            latest_seq,
            events,
            has_more: latest_seq < self.sequence,
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn pagination_recovers_a_phone_stall_without_unbounded_responses() {
        let mut log = GpsLog::new(7);
        for i in 0..400 {
            log.push(RmcTiming::default(), &GpsFix::default(), i * 40_000);
        }
        let mut cursor = 10;
        let mut count = 0;
        loop {
            let page = log.snapshot_since(Some((7, cursor)));
            assert!(page.events.len() <= 64);
            assert_eq!(page.events[0].seq, cursor + 1);
            count += page.events.len();
            cursor = page.latest_seq;
            if !page.has_more {
                break;
            }
        }
        assert_eq!(count, 390);
        assert_eq!(log.snapshot_since(Some((8, 10))).events[0].seq, 337);
        for i in 400..600 {
            log.push(RmcTiming::default(), &GpsFix::default(), i * 40_000);
        }
        assert_eq!(log.snapshot_since(Some((7, 10))).events[0].seq, 89);
        assert!(core::mem::size_of::<GpsEvent>() * GPS_HISTORY_CAPACITY <= 65_536);
    }
    #[test]
    fn bounded_log_keeps_order_and_invalid_epochs_without_stale_coordinates() {
        let mut log = GpsLog::new(7);
        let mut fix = GpsFix {
            valid: true,
            lat: 12.,
            lon: 34.,
            ..Default::default()
        };
        for i in 0..70 {
            log.push(
                RmcTiming {
                    utc_us: Some(i * 40000),
                    ..Default::default()
                },
                &fix,
                1_000_000 + i * 40000,
            );
        }
        fix.valid = false;
        log.push(RmcTiming::default(), &fix, 9_000_000);
        let s = log.snapshot_since(None);
        assert_eq!(s.events.len(), 64);
        assert_eq!(s.events[0].seq, 8);
        assert_eq!(s.latest_seq, 71);
        let last = s.events.last().unwrap();
        assert!(!last.accepted);
        assert_eq!(last.lat, None);
        assert_eq!(last.utc_us, None);
        assert!(s.events.windows(2).all(|w| w[1].seq == w[0].seq + 1));
        let json = serde_json::to_value(s).unwrap();
        assert!(json["events"][63]["lat"].is_null());
    }
    #[test]
    fn duplicate_receiver_epochs_and_host_times_remain_evidence() {
        let mut log = GpsLog::new(8);
        let t = RmcTiming {
            utc_us: Some(123456),
            ..Default::default()
        };
        log.push(t, &GpsFix::default(), 1);
        log.push(t, &GpsFix::default(), 1);
        assert_eq!(log.snapshot_since(None).events.len(), 2);
        assert_eq!(log.latest().1.unwrap().seq, 2);
    }
}
