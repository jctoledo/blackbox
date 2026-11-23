//! One elapsed interval for a drained UART batch; packet arrival count is not time.
pub fn elapsed_seconds(previous_us: u64, now_us: u64, accel_count: u32) -> Option<f32> {
    (accel_count > 0 && now_us > previous_us).then(|| (now_us - previous_us) as f32 * 1e-6)
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
}
