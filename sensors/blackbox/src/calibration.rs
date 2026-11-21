//! Experimental horizontal mounting calibration, revising PR #13's brake-vector idea.
//! A signed GPS speed derivative is required: acceleration magnitude alone cannot
//! distinguish forward acceleration from braking. Host receive times are not PPS.
use core::f32::consts::PI;
use serde::Serialize;

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize)]
pub struct MountCalibration {
    /// Vehicle forward measured counterclockwise from levelled device X.
    /// Vehicle vectors = R(-offset) device vectors. Device yaw is NOT modified.
    pub yaw_offset: f32,
    pub is_calibrated: bool,
}
impl MountCalibration {
    pub fn from_offset(offset: f32) -> Option<Self> {
        if !offset.is_finite() {
            return None;
        }
        Some(Self {
            yaw_offset: wrap(offset),
            is_calibrated: true,
        })
    }
    pub fn encode(self) -> [u8; 16] {
        let mut b = [0; 16];
        b[..4].copy_from_slice(b"BMC1");
        b[4] = u8::from(self.is_calibrated);
        b[8..12].copy_from_slice(&self.yaw_offset.to_le_bytes());
        let crc = checksum(&b[..12]);
        b[12..].copy_from_slice(&crc.to_le_bytes());
        b
    }
    pub fn decode(b: &[u8]) -> Option<Self> {
        if b.len() != 16
            || &b[..4] != b"BMC1"
            || b[4] > 1
            || b[5..8] != [0; 3]
            || checksum(&b[..12]) != u32::from_le_bytes(b[12..16].try_into().ok()?)
        {
            return None;
        }
        let angle = f32::from_le_bytes(b[8..12].try_into().ok()?);
        if !angle.is_finite() || angle.abs() > PI || (b[4] == 0 && angle != 0.0) {
            return None;
        }
        Some(Self {
            yaw_offset: angle,
            is_calibrated: b[4] == 1,
        })
    }
}
fn checksum(bytes: &[u8]) -> u32 {
    let mut crc = !0u32;
    for b in bytes {
        crc ^= *b as u32;
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb88320 & (0u32.wrapping_sub(crc & 1)));
        }
    }
    !crc
}
pub fn wrap(angle: f32) -> f32 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub at_ms: u32,
    pub speed_mps: f32,
    pub course_rad: f32,
    /// Gravity removed, roll/pitch levelled, yaw left at zero.
    pub ax: f32,
    pub ay: f32,
    pub yaw_rate: f32,
    pub imu_age_ms: u32,
    pub roll_deg: f32,
    pub pitch_deg: f32,
}
impl Sample {
    fn valid(self) -> bool {
        [
            self.speed_mps,
            self.course_rad,
            self.ax,
            self.ay,
            self.yaw_rate,
            self.roll_deg,
            self.pitch_deg,
        ]
        .iter()
        .all(|v| v.is_finite())
            && (0.0..=100.0).contains(&self.speed_mps)
            && self.imu_age_ms <= 60
            && self.roll_deg.abs() <= 10.0
            && self.pitch_deg.abs() <= 10.0
            && self.ax.abs() <= 20.0
            && self.ay.abs() <= 20.0
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct CalibrationStatus {
    pub state: &'static str,
    pub message: &'static str,
    pub saved: MountCalibration,
    pub candidate_degrees: Option<f32>,
    pub samples: usize,
    pub parked: bool,
    pub storage_available: bool,
}

/// Bounded collection at fresh GPS-fix cadence (5–25 Hz), not repeated IMU ticks.
/// The previous saved calibration survives all unsuccessful attempts.
pub struct MountCalibrator {
    pub status: CalibrationStatus,
    started_ms: u32,
    last: Option<Sample>,
    first_brake_ms: u32,
    angles: [f32; 64],
    course: f32,
    candidate: Option<MountCalibration>,
}
impl MountCalibrator {
    pub fn new(saved: MountCalibration, storage_available: bool) -> Self {
        Self {
            status: CalibrationStatus {
                state: "idle",
                message: "Park to start. Horizontal mounting only; field validation required.",
                saved,
                candidate_degrees: None,
                samples: 0,
                parked: false,
                storage_available,
            },
            started_ms: 0,
            last: None,
            first_brake_ms: 0,
            angles: [0.0; 64],
            course: 0.0,
            candidate: None,
        }
    }
    pub fn active(&self) -> bool {
        matches!(self.status.state, "waiting" | "collecting")
    }
    fn fail(&mut self, message: &'static str) {
        self.status.state = "failed";
        self.status.message = message;
        self.candidate = None;
        self.status.candidate_degrees = None;
        self.last = None;
    }
    pub fn command(&mut self, command: &str, now_ms: u32) {
        match command {
            "cancel" => self.fail("Cancelled. Saved calibration unchanged."),
            "start" if self.status.parked && self.status.storage_available && !self.active() => {
                self.started_ms=now_ms;self.last=None;self.candidate=None;
                self.status.candidate_degrees=None;self.status.samples=0;self.status.state="waiting";
                self.status.message="At a controlled site, drive forward then brake steadily in a straight line. Do not operate the screen while moving.";
            }
            "start" => self.status.message="Start requires a fresh stationary GPS fix and available storage; stop any active attempt first.",
            _ => self.status.message="Unknown calibration command.",
        }
    }
    /// Persistence completes BEFORE the new value becomes active. A failed write
    /// leaves both the active calibration and review candidate available for retry.
    pub fn save_with(
        &mut self,
        clear: bool,
        persist: impl FnOnce(&[u8]) -> Result<(), ()>,
    ) -> bool {
        if !self.status.parked || self.active() || !self.status.storage_available {
            self.status.message =
                "Park with a fresh GPS fix and stop collection before saving or clearing.";
            return false;
        }
        let next = if clear {
            MountCalibration::default()
        } else if let Some(c) = self.candidate {
            c
        } else {
            self.status.message = "No candidate to save.";
            return false;
        };
        if persist(&next.encode()).is_err() {
            self.status.message =
                "Save failed. Previous calibration remains active; retry while parked.";
            return false;
        }
        self.status.saved = next;
        self.candidate = None;
        self.status.candidate_degrees = None;
        self.status.state = "saved";
        self.status.message = if clear {
            "Mount offset cleared."
        } else {
            "Saved. Verify acceleration/braking and left/right signs before relying on forces."
        };
        true
    }
    pub fn tick(&mut self, now_ms: u32) {
        if self.active() && now_ms.wrapping_sub(self.started_ms) >= 60000 {
            self.fail("Timed out. Saved calibration unchanged.");
        } else if self.status.state == "collecting"
            && self
                .last
                .is_some_and(|s| now_ms.wrapping_sub(s.at_ms) > 300)
        {
            self.fail("GPS gap during collection. Retry with clear reception.");
        }
    }
    pub fn invalidate(&mut self) {
        self.last = None;
        if self.status.state == "collecting" {
            self.fail("Invalid or stale sensor data. Saved calibration unchanged.");
        }
    }
    pub fn sample(&mut self, s: Sample) {
        self.tick(s.at_ms);
        if !self.active() {
            return;
        }
        if !s.valid() {
            self.invalidate();
            return;
        }
        let previous = self.last.replace(s);
        let Some(p) = previous else {
            return;
        };
        let gap = s.at_ms.wrapping_sub(p.at_ms);
        // Duplicate and backward timestamps never count as new evidence.
        if !(20..=300).contains(&gap) {
            if self.status.state == "collecting" {
                self.fail("Non-contiguous GPS samples. Retry.");
            }
            return;
        }
        let decel = (s.speed_mps - p.speed_mps) / (gap as f32 * 0.001);
        let magnitude = s.ax.hypot(s.ay);
        let straight = s.yaw_rate.abs() < 0.08
            && p.yaw_rate.abs() < 0.08
            && wrap(s.course_rad - p.course_rad).abs() < 0.035;
        let braking = (-10.0..=-1.5).contains(&decel)
            && magnitude >= 1.5
            && (magnitude + decel).abs() <= 1.2f32.max(-decel * 0.5)
            && p.speed_mps >= 3.0
            && s.speed_mps >= 2.0
            && straight;
        if !braking {
            if self.status.state == "collecting" {
                self.fail(
                    "Braking was not continuous and straight. Retry; saved calibration unchanged.",
                );
            }
            return;
        }
        if self.status.state == "waiting" {
            self.status.state = "collecting";
            self.status.message = "Collecting signed braking evidence…";
            self.first_brake_ms = s.at_ms;
            self.course = s.course_rad;
            self.status.samples = 0;
        }
        if wrap(s.course_rad - self.course).abs() > 0.0524 {
            self.fail("Course changed during collection. Retry straight-line braking.");
            return;
        }
        if self.status.samples == self.angles.len() {
            self.fail("Collection capacity reached before duration requirement.");
            return;
        }
        self.angles[self.status.samples] = s.ay.atan2(s.ax);
        self.status.samples += 1;
        if s.at_ms.wrapping_sub(self.first_brake_ms) < 600 {
            return;
        }
        if self.status.samples < 4 {
            self.fail("Insufficient independent GPS samples.");
            return;
        }
        let angles = &self.angles[..self.status.samples];
        let mean = angles
            .iter()
            .map(|a| a.sin())
            .sum::<f32>()
            .atan2(angles.iter().map(|a| a.cos()).sum::<f32>());
        if angles.iter().any(|a| wrap(*a - mean).abs() > 0.175) {
            self.fail("Brake directions disagree. Check mounting and reception.");
            return;
        }
        self.candidate = MountCalibration::from_offset(wrap(mean + PI));
        self.status.candidate_degrees = self.candidate.map(|c| c.yaw_offset.to_degrees());
        self.status.state = "review";
        self.status.message="Candidate ready. Park, review, then save. Forward travel and level mounting must be confirmed by the operator.";
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn sample(t: u32, speed: f32, angle: f32) -> Sample {
        Sample {
            at_ms: t,
            speed_mps: speed,
            course_rad: 1.0,
            ax: -3.0 * angle.cos(),
            ay: -3.0 * angle.sin(),
            yaw_rate: 0.0,
            imu_age_ms: 5,
            roll_deg: 0.0,
            pitch_deg: 0.0,
        }
    }
    fn armed(t: u32) -> MountCalibrator {
        let mut c = MountCalibrator::new(MountCalibration::from_offset(0.4).unwrap(), true);
        c.status.parked = true;
        c.command("start", t);
        c
    }
    fn collect(c: &mut MountCalibrator, start: u32, step: u32, angle: f32) {
        for i in 0..=1000 / step {
            let t = i * step;
            c.sample(sample(
                start.wrapping_add(t),
                15.0 - 3.0 * t as f32 / 1000.0,
                angle,
            ));
        }
    }
    #[test]
    fn rotated_braking_at_both_receiver_rates_and_clock_wrap() {
        for step in [40, 200] {
            for start in [0, u32::MAX - 300] {
                for angle in [0.0, PI / 2.0, -PI / 2.0, PI - 0.01, -PI + 0.01, 0.7] {
                    let mut c = armed(start);
                    collect(&mut c, start, step, angle);
                    assert_eq!(c.status.state, "review");
                    assert!(wrap(c.candidate.unwrap().yaw_offset - angle).abs() < 1e-5);
                    assert_eq!(
                        c.status.saved.yaw_offset,
                        MountCalibration::from_offset(0.4).unwrap().yaw_offset
                    );
                }
            }
        }
    }
    #[test]
    fn acceleration_cruising_and_low_speed_never_calibrate() {
        for accel in [0.0, 3.0] {
            let mut c = armed(0);
            for i in 0..100 {
                let t = i * 40;
                c.sample(sample(t, 10.0 + accel * t as f32 / 1000.0, 0.0));
            }
            assert_eq!(c.status.state, "waiting");
        }
        let mut c = armed(0);
        for i in 0..20 {
            c.sample(sample(i * 40, 2.0 - i as f32 * 0.12, 0.0));
        }
        assert!(c.candidate.is_none());
    }
    #[test]
    fn corrupt_stale_turning_and_discontinuous_data_cannot_complete() {
        for kind in 0..10 {
            let mut c = armed(0);
            c.sample(sample(0, 15.0, 0.0));
            c.sample(sample(40, 14.88, 0.0));
            let mut s = sample(80, 14.76, 0.0);
            match kind {
                0 => s.ax = f32::NAN,
                1 => s.speed_mps = f32::INFINITY,
                2 => s.imu_age_ms = 61,
                3 => s.yaw_rate = 0.2,
                4 => s.roll_deg = 11.0,
                5 => s.course_rad += 0.2,
                6 => s.at_ms = 40,
                7 => s.at_ms = 20,
                8 => s.speed_mps = 14.99,
                _ => s.ax = 12.0,
            }
            c.sample(s);
            assert_eq!(c.status.state, "failed", "kind {kind}");
            assert!(c.candidate.is_none());
        }
    }
    #[test]
    fn insufficient_duration_cannot_finalize_on_interruption() {
        let mut c = armed(0);
        for i in 0..8 {
            c.sample(sample(i * 40, 15.0 - i as f32 * 0.12, 0.0));
        }
        c.sample(sample(320, 14.16, 0.0));
        assert_eq!(c.status.state, "failed");
    }
    #[test]
    fn timeouts_cancel_and_failure_reasons_are_sticky() {
        let mut c = armed(0);
        c.tick(60000);
        let message = c.status.message;
        c.tick(70000);
        assert_eq!(c.status.message, message);
        c.command("start", 70000);
        c.sample(sample(70000, 15.0, 0.0));
        c.sample(sample(70040, 14.88, 0.0));
        c.tick(70341);
        assert_eq!(c.status.state, "failed");
        c.command("start", 80000);
        c.command("cancel", 80001);
        assert_eq!(c.status.state, "failed");
        assert!(c.candidate.is_none());
    }
    #[test]
    fn storage_failure_keeps_old_calibration_and_allows_retry() {
        let mut c = armed(0);
        collect(&mut c, 0, 40, 1.0);
        let old = c.status.saved;
        assert!(!c.save_with(false, |_| Err(())));
        assert_eq!(c.status.saved, old);
        assert!(c.candidate.is_some());
        c.status.parked = false;
        assert!(!c.save_with(false, |_| panic!("moving must not write")));
        c.status.parked = true;
        assert!(c.save_with(false, |b| {
            assert!(MountCalibration::decode(b).unwrap().is_calibrated);
            Ok(())
        }));
        assert!((c.status.saved.yaw_offset - 1.0).abs() < 1e-5);
        assert!(c.save_with(true, |_| Ok(())));
        assert!(!c.status.saved.is_calibrated);
    }
    #[test]
    fn record_codec_rejects_corruption_old_format_and_nonfinite_offsets() {
        assert!(MountCalibration::from_offset(f32::INFINITY).is_none());
        for c in [
            MountCalibration::default(),
            MountCalibration::from_offset(-PI).unwrap(),
        ] {
            let b = c.encode();
            assert_eq!(MountCalibration::decode(&b), Some(c));
            for n in 0..16 {
                let mut corrupt = b;
                corrupt[n] ^= 1;
                assert!(MountCalibration::decode(&corrupt).is_none());
            }
        }
        assert!(MountCalibration::decode(&[0; 4]).is_none());
        for angle in [f32::NAN, f32::INFINITY, 4.0] {
            let c = MountCalibration {
                yaw_offset: angle,
                is_calibrated: true,
            };
            assert!(MountCalibration::decode(&c.encode()).is_none());
        }
    }
    #[test]
    fn inconsistent_braking_vectors_are_rejected() {
        let mut c = armed(0);
        for i in 0..25 {
            c.sample(sample(
                i * 40,
                15.0 - i as f32 * 0.12,
                if i % 2 == 0 { 0.3 } else { -0.3 },
            ));
        }
        assert_eq!(c.status.state, "failed");
    }
}
