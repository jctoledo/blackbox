//! Bounded evidence, captured at completed RMC parsing (not first UART bit/PPS).
use std::collections::VecDeque;
use ublox_gps::{GpsFix, RmcTiming};
pub const GPS_LOG_CAPACITY: usize = 64;
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
            events: VecDeque::with_capacity(GPS_LOG_CAPACITY),
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
        if self.events.len() == GPS_LOG_CAPACITY {
            self.events.pop_front();
        }
        self.events.push_back(event);
    }
    pub fn latest(&self) -> (u32, Option<GpsEvent>) {
        (self.boot_id, self.events.back().copied())
    }
    pub fn snapshot(&self) -> GpsLogSnapshot {
        GpsLogSnapshot {
            version: 1,
            simulated: false,
            boot_id: self.boot_id,
            capacity: GPS_LOG_CAPACITY,
            latest_seq: self.sequence,
            events: self.events.iter().copied().collect(),
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
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
        let s = log.snapshot();
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
        assert_eq!(log.snapshot().events.len(), 2);
        assert_eq!(log.latest().1.unwrap().seq, 2);
    }
}
