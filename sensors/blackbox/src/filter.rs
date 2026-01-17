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

    // Test with IMU-rate parameters (200Hz sampling, 4Hz cutoff)
    const SAMPLE_RATE: f32 = 200.0;
    const CUTOFF: f32 = 4.0;

    // Also test with telemetry-rate parameters (20Hz sampling, 2Hz cutoff)
    const TELEM_RATE: f32 = 20.0;
    const TELEM_CUTOFF: f32 = 2.0;

    #[test]
    fn test_filter_creation() {
        let filter = BiquadFilter::new_lowpass(CUTOFF, SAMPLE_RATE);
        // Verify coefficients are reasonable
        assert!(filter.b0 > 0.0 && filter.b0 < 1.0);
        assert!(filter.b1 > 0.0);
        assert!(filter.b2 > 0.0 && filter.b2 < 1.0);
    }

    #[test]
    #[should_panic(expected = "Cutoff must be below Nyquist")]
    fn test_filter_nyquist_limit() {
        // Should panic: cutoff >= sample_rate/2
        let _ = BiquadFilter::new_lowpass(100.0, SAMPLE_RATE);
    }

    #[test]
    fn test_dc_passthrough() {
        // DC signal (0 Hz) should pass through unchanged
        let mut filter = BiquadFilter::new_lowpass(CUTOFF, SAMPLE_RATE);

        // Feed constant value, let filter settle
        for _ in 0..100 {
            filter.process(1.0);
        }

        // After settling, output should be ~1.0
        let output = filter.process(1.0);
        assert!(
            (output - 1.0).abs() < 0.01,
            "DC should pass through: got {}",
            output
        );
    }

    #[test]
    fn test_high_frequency_attenuation() {
        // 50Hz signal should be heavily attenuated (4Hz cutoff)
        let mut filter = BiquadFilter::new_lowpass(CUTOFF, SAMPLE_RATE);

        // Generate 50Hz sine wave
        let freq = 50.0;
        let mut max_output: f32 = 0.0;

        // Let filter settle, then measure
        for i in 0..1000 {
            let t = i as f32 / SAMPLE_RATE;
            let input = (2.0 * core::f32::consts::PI * freq * t).sin();
            let output = filter.process(input);

            if i > 500 {
                // After settling
                max_output = max_output.max(output.abs());
            }
        }

        // At 50Hz with 4Hz cutoff, attenuation should be significant
        // 50Hz is ~3.6 octaves above 4Hz, expect -40dB or more
        // That's amplitude ratio of 0.01 or less
        assert!(
            max_output < 0.05,
            "50Hz should be attenuated: got peak {}",
            max_output
        );
    }

    #[test]
    fn test_low_frequency_passthrough() {
        // 1Hz signal should pass through mostly unchanged
        let mut filter = BiquadFilter::new_lowpass(CUTOFF, SAMPLE_RATE);

        let freq = 1.0;
        let mut max_output: f32 = 0.0;

        for i in 0..2000 {
            // Need longer for 1Hz signal
            let t = i as f32 / SAMPLE_RATE;
            let input = (2.0 * core::f32::consts::PI * freq * t).sin();
            let output = filter.process(input);

            if i > 1000 {
                max_output = max_output.max(output.abs());
            }
        }

        // 1Hz is well below 4Hz cutoff, should pass with minimal attenuation
        assert!(
            max_output > 0.9,
            "1Hz should pass through: got peak {}",
            max_output
        );
    }

    #[test]
    fn test_step_response_no_overshoot() {
        // Butterworth should have no overshoot (critically damped)
        let mut filter = BiquadFilter::new_lowpass(CUTOFF, SAMPLE_RATE);

        let mut max_output: f32 = 0.0;

        // Step from 0 to 1
        for _ in 0..500 {
            let output = filter.process(1.0);
            max_output = max_output.max(output);
        }

        // Should never exceed 1.0 (no overshoot)
        assert!(
            max_output <= 1.01,
            "Should not overshoot: got max {}",
            max_output
        );
    }

    #[test]
    fn test_reset_to_value() {
        let mut filter = BiquadFilter::new_lowpass(CUTOFF, SAMPLE_RATE);

        // Reset to steady state of 0.5
        filter.reset_to(0.5);

        // Immediately processing 0.5 should output ~0.5
        let output = filter.process(0.5);
        assert!(
            (output - 0.5).abs() < 0.01,
            "After reset_to, should output steady value: got {}",
            output
        );
    }

    #[test]
    fn test_vibration_simulation() {
        // Simulate realistic scenario: driving signal + engine vibration
        let mut filter = BiquadFilter::new_lowpass(CUTOFF, SAMPLE_RATE);

        let driving_freq = 0.5; // 0.5 Hz - slow braking
        let vibration_freq = 60.0; // 60 Hz - engine vibration
        let driving_amplitude = 0.3; // 0.3g braking
        let vibration_amplitude = 0.05; // 0.05g vibration

        let mut driving_energy = 0.0;
        let mut vibration_energy = 0.0;
        let mut output_energy = 0.0;

        for i in 0..4000 {
            // 20 seconds
            let t = i as f32 / SAMPLE_RATE;

            // Input: driving signal + vibration noise
            let driving = driving_amplitude * (2.0 * core::f32::consts::PI * driving_freq * t).sin();
            let vibration =
                vibration_amplitude * (2.0 * core::f32::consts::PI * vibration_freq * t).sin();
            let input = driving + vibration;

            let output = filter.process(input);

            if i > 1000 {
                // After settling
                driving_energy += driving * driving;
                vibration_energy += vibration * vibration;
                output_energy += output * output;
            }
        }

        // Output energy should be close to driving energy (vibration removed)
        let ratio = output_energy / driving_energy;
        assert!(
            ratio > 0.8 && ratio < 1.2,
            "Filter should preserve driving signal: energy ratio = {}",
            ratio
        );

        // Output should have much less energy than input (vibration removed)
        let input_energy = driving_energy + vibration_energy;
        assert!(
            output_energy < input_energy * 0.95,
            "Filter should remove vibration energy"
        );
    }

    #[test]
    fn test_telemetry_rate_filter() {
        // Test at telemetry rate (20Hz sampling, 2Hz cutoff)
        // This is the actual configuration used in main.rs
        let mut filter = BiquadFilter::new_lowpass(TELEM_CUTOFF, TELEM_RATE);

        // DC should pass through
        for _ in 0..50 {
            filter.process(1.0);
        }
        let dc_output = filter.process(1.0);
        assert!(
            (dc_output - 1.0).abs() < 0.02,
            "DC should pass at telemetry rate: got {}",
            dc_output
        );

        // Reset and test low frequency (0.5 Hz driving input)
        let mut filter = BiquadFilter::new_lowpass(TELEM_CUTOFF, TELEM_RATE);
        let freq = 0.5; // 0.5 Hz - typical braking/accel
        let mut max_output: f32 = 0.0;

        for i in 0..200 {
            // 10 seconds at 20Hz
            let t = i as f32 / TELEM_RATE;
            let input = (2.0 * core::f32::consts::PI * freq * t).sin();
            let output = filter.process(input);

            if i > 100 {
                max_output = max_output.max(output.abs());
            }
        }

        // 0.5 Hz is well below 2Hz cutoff, should pass through
        assert!(
            max_output > 0.85,
            "0.5Hz should pass at 20Hz/2Hz filter: got {}",
            max_output
        );

        // Test high frequency rejection (8Hz - at Nyquist, should be attenuated)
        let mut filter = BiquadFilter::new_lowpass(TELEM_CUTOFF, TELEM_RATE);
        let freq = 8.0;
        let mut max_output: f32 = 0.0;

        for i in 0..200 {
            let t = i as f32 / TELEM_RATE;
            let input = (2.0 * core::f32::consts::PI * freq * t).sin();
            let output = filter.process(input);

            if i > 100 {
                max_output = max_output.max(output.abs());
            }
        }

        // 8Hz is well above 2Hz cutoff at 20Hz sample rate
        assert!(
            max_output < 0.2,
            "8Hz should be attenuated at 20Hz/2Hz filter: got {}",
            max_output
        );
    }
}

