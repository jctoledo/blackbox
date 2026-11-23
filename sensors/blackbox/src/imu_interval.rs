//! One elapsed interval for a drained UART batch; packet arrival count is not time.
pub fn elapsed_seconds(previous_us: u64, now_us: u64, accel_count: u32) -> Option<f32> {
    (accel_count > 0 && now_us > previous_us).then(|| (now_us - previous_us) as f32 * 1e-6)
}

/// Prediction advances every loop before GPS correction, including without IMU
/// arrivals. Acquisition freshness timestamps must never double as this clock.
#[derive(Default)]
pub struct PredictionClock(Option<u64>);
impl PredictionClock {
    pub fn advance(&mut self, now_us: u64, correction_pending: bool) -> Option<f32> {
        let Some(previous) = self.0 else {
            self.0 = Some(now_us);
            return None;
        };
        if now_us <= previous {
            self.0 = Some(now_us);
            return None;
        }
        // Avoid increasing prediction work/noise cadence to the busy-loop rate.
        // Flush a shorter interval before any GPS correction.
        if !correction_pending && now_us - previous < 5_000 {
            return None;
        }
        self.0 = Some(now_us);
        elapsed_seconds(previous, now_us, 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn batching_and_short_stalls_preserve_all_elapsed_time() {
        let mut previous = 1_000_000;
        let mut total = 0.0;
        for (now, count) in [(1_005_000, 1), (1_075_000, 14), (1_195_000, 24)] {
            total += elapsed_seconds(previous, now, count).unwrap();
            previous = now;
        }
        assert!((total - 0.195).abs() < 1e-6);
        assert_eq!(elapsed_seconds(previous, previous, 1), None);
        assert_eq!(elapsed_seconds(previous, previous + 5_000, 0), None);
    }

    #[test]
    fn ongoing_gps_corrections_during_imu_outage_are_not_counted_twice() {
        let mut clock = PredictionClock::default();
        let mut ekf = sensor_fusion::ekf::Ekf::new();
        ekf.x[3] = 10.0;
        assert_eq!(clock.advance(1_000_000, true), None);
        for i in 1..=50 {
            let dt = clock.advance(1_000_000 + i * 40_000, true).unwrap();
            ekf.coast(dt);
            ekf.update_position(i as f32 * 0.4, 0.0);
            ekf.update_velocity(10.0, 0.0);
        }
        let resumed_dt = clock.advance(3_040_000, false).unwrap();
        assert!((resumed_dt - 0.04).abs() < 1e-6);
        ekf.predict(0.0, 0.0, 0.0, resumed_dt);
        assert!((ekf.position().0 - 20.4).abs() < 0.001);
    }
}
