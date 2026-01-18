//! Signal filtering utilities
//!
//! Contains Biquad IIR filter implementation for vibration removal.
//! Note: Currently unused in production - kept for potential future use
//! and to verify the math with unit tests.

/// Biquad IIR low-pass filter for vibration removal
///
/// Implements a 2nd-order Butterworth low-pass filter optimized for
/// removing engine/road vibration (10-100Hz) while preserving
/// driving dynamics (0-3Hz).
///
/// Design: Butterworth (maximally flat passband, Q = 0.707)
/// Typical cutoff: 4Hz at 200Hz sample rate
#[derive(Clone)]
pub struct BiquadFilter {
    // Filter coefficients (normalized)
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,

    // Filter state (previous samples)
    x1: f32, // x[n-1]
    x2: f32, // x[n-2]
    y1: f32, // y[n-1]
    y2: f32, // y[n-2]
}

impl BiquadFilter {
    /// Create a new Butterworth low-pass filter
    ///
    /// # Arguments
    /// * `cutoff_hz` - Cutoff frequency in Hz (e.g., 4.0)
    /// * `sample_rate_hz` - Sample rate in Hz (e.g., 200.0)
    ///
    /// # Panics
    /// Panics if cutoff >= sample_rate/2 (Nyquist limit)
    pub fn new_lowpass(cutoff_hz: f32, sample_rate_hz: f32) -> Self {
        assert!(
            cutoff_hz < sample_rate_hz / 2.0,
            "Cutoff must be below Nyquist frequency"
        );

        // Butterworth Q factor for critically damped response
        let q = core::f32::consts::FRAC_1_SQRT_2; // 0.7071...

        // Pre-warp the cutoff frequency for bilinear transform
        let omega = 2.0 * core::f32::consts::PI * cutoff_hz / sample_rate_hz;
        let sin_omega = omega.sin();
        let cos_omega = omega.cos();
        let alpha = sin_omega / (2.0 * q);

        // Compute coefficients (lowpass)
        let b0 = (1.0 - cos_omega) / 2.0;
        let b1 = 1.0 - cos_omega;
        let b2 = (1.0 - cos_omega) / 2.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_omega;
        let a2 = 1.0 - alpha;

        // Normalize by a0
        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1: a1 / a0,
            a2: a2 / a0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    /// Process a single sample through the filter
    ///
    /// # Arguments
    /// * `input` - Input sample
    ///
    /// # Returns
    /// Filtered output sample
    pub fn process(&mut self, input: f32) -> f32 {
        // Direct Form I implementation
        // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
        let output = self.b0 * input + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;

        // Update state
        self.x2 = self.x1;
        self.x1 = input;
        self.y2 = self.y1;
        self.y1 = output;

        output
    }

    /// Initialize filter state to a steady value
    ///
    /// Use this to avoid startup transients when you know
    /// the initial value (e.g., after calibration shows ~0)
    pub fn reset_to(&mut self, value: f32) {
        self.x1 = value;
        self.x2 = value;
        self.y1 = value;
        self.y2 = value;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_RATE: f32 = 200.0;
    const CUTOFF: f32 = 4.0;

    #[test]
    fn test_filter_creation() {
        let filter = BiquadFilter::new_lowpass(CUTOFF, SAMPLE_RATE);
        assert!(filter.b0 > 0.0 && filter.b0 < 1.0);
        assert!(filter.b1 > 0.0);
        assert!(filter.b2 > 0.0 && filter.b2 < 1.0);
    }

    #[test]
    #[should_panic(expected = "Cutoff must be below Nyquist")]
    fn test_filter_nyquist_limit() {
        let _ = BiquadFilter::new_lowpass(100.0, SAMPLE_RATE);
    }

    #[test]
    fn test_dc_passthrough() {
        let mut filter = BiquadFilter::new_lowpass(CUTOFF, SAMPLE_RATE);

        for _ in 0..100 {
            filter.process(1.0);
        }

        let output = filter.process(1.0);
        assert!(
            (output - 1.0).abs() < 0.01,
            "DC should pass through: got {}",
            output
        );
    }

    #[test]
    fn test_high_frequency_attenuation() {
        let mut filter = BiquadFilter::new_lowpass(CUTOFF, SAMPLE_RATE);

        let freq = 50.0;
        let mut max_output: f32 = 0.0;

        for i in 0..1000 {
            let t = i as f32 / SAMPLE_RATE;
            let input = (2.0 * core::f32::consts::PI * freq * t).sin();
            let output = filter.process(input);

            if i > 500 {
                max_output = max_output.max(output.abs());
            }
        }

        assert!(
            max_output < 0.05,
            "50Hz should be attenuated: got peak {}",
            max_output
        );
    }

    #[test]
    fn test_low_frequency_passthrough() {
        let mut filter = BiquadFilter::new_lowpass(CUTOFF, SAMPLE_RATE);

        let freq = 1.0;
        let mut max_output: f32 = 0.0;

        for i in 0..2000 {
            let t = i as f32 / SAMPLE_RATE;
            let input = (2.0 * core::f32::consts::PI * freq * t).sin();
            let output = filter.process(input);

            if i > 1000 {
                max_output = max_output.max(output.abs());
            }
        }

        assert!(
            max_output > 0.9,
            "1Hz should pass through: got peak {}",
            max_output
        );
    }

    #[test]
    fn test_step_response_minimal_overshoot() {
        // Butterworth filters have minimal overshoot (Q = 0.707)
        // In practice, slight overshoot ~5% is acceptable
        let mut filter = BiquadFilter::new_lowpass(CUTOFF, SAMPLE_RATE);

        let mut max_output: f32 = 0.0;

        for _ in 0..500 {
            let output = filter.process(1.0);
            max_output = max_output.max(output);
        }

        assert!(
            max_output <= 1.10,
            "Should have minimal overshoot (<10%): got max {}",
            max_output
        );
    }

    #[test]
    fn test_reset_to_value() {
        let mut filter = BiquadFilter::new_lowpass(CUTOFF, SAMPLE_RATE);

        filter.reset_to(0.5);

        let output = filter.process(0.5);
        assert!(
            (output - 0.5).abs() < 0.01,
            "After reset_to, should output steady value: got {}",
            output
        );
    }
}
