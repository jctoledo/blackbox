//! Stationary bias collection at application receive-clock cadence.
//! The caller installs a returned bias only after successful completion.
use wt901::{ImuBias, ImuCalibrator, ImuData};

pub const CALIB_SAMPLES: usize = 150;
const MIN_SAMPLE_US: u64 = 10_000;
const MAX_ACCEL_GAP_US: u64 = 200_000;
const DEADLINE_US: u64 = 30_000_000;

pub struct BiasCollection {
    calibrator: ImuCalibrator,
    started_us: u64,
    last_sample_us: u64,
    last_accel_us: Option<u64>,
}

impl BiasCollection {
    pub fn new(started_us: u64) -> Self {
        Self {
            calibrator: ImuCalibrator::new(CALIB_SAMPLES),
            started_us,
            last_sample_us: 0,
            last_accel_us: None,
        }
    }

    pub fn expired(&self, now_us: u64) -> bool {
        now_us.saturating_sub(self.started_us) > DEADLINE_US
    }

    /// None means minimum sample spacing has not elapsed; false resets progress.
    /// Packet times use the same monotonic host receive clock as now_us.
    pub fn observe_accel(
        &mut self,
        now_us: u64,
        data: &ImuData,
        angle_us: u64,
        gyro_us: u64,
    ) -> Option<bool> {
        if self.expired(now_us) {
            return None;
        }
        // Fresh gyro/attitude on recovery must not revive pre-outage samples.
        // Track every acceleration packet, including those skipped by spacing.
        if self
            .last_accel_us
            .is_some_and(|at| now_us.saturating_sub(at) > MAX_ACCEL_GAP_US)
        {
            self.calibrator.reset();
        }
        if self.last_accel_us.is_none_or(|at| now_us >= at) {
            self.last_accel_us = Some(now_us);
        }
        // Bound selected samples to 100 Hz: about 1.5 s of stationary evidence.
        if now_us.saturating_sub(self.last_sample_us) < MIN_SAMPLE_US {
            return None;
        }
        self.last_sample_us = now_us;
        let age = |at| {
            if at == 0 {
                u64::MAX
            } else {
                now_us.saturating_sub(at)
            }
        };
        Some(
            self.calibrator
                .add_stationary_sample(data, age(angle_us), age(gyro_us)),
        )
    }

    pub fn is_complete(&self) -> bool {
        self.calibrator.is_complete()
    }

    pub fn progress(&self) -> f32 {
        self.calibrator.progress()
    }

    pub fn compute_bias(&self) -> Option<ImuBias> {
        self.calibrator.compute_bias()
    }
}
