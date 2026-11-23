use sensor_fusion::ekf::{Ekf, EkfConfig, UpdateResult};

// Independent dense f64 reference. Production uses two f32 3x3 blocks and Joseph
// updates; this reference propagates 7x7 and uses the Schur covariance update.
struct Reference {
    x: [f64; 7],
    p: [[f64; 7]; 7],
}
impl Reference {
    fn new(ekf: &Ekf) -> Self {
        Self {
            x: ekf.x.map(f64::from),
            p: ekf.covariance().map(|r| r.map(f64::from)),
        }
    }
    fn predict(&mut self, a: [f64; 2], wz: f64, dt: f64, c: EkfConfig) {
        let mut f = [[0.0; 7]; 7];
        for (i, row) in f.iter_mut().enumerate() {
            row[i] = 1.0;
        }
        for [pos, vel, bias] in [[0, 3, 5], [1, 4, 6]] {
            f[pos][vel] = dt;
            f[pos][bias] = -dt * dt / 2.0;
            f[vel][bias] = -dt;
        }
        let mut next = [0.0; 7];
        for i in 0..7 {
            for j in 0..7 {
                next[i] += f[i][j] * self.x[j];
            }
        }
        for (axis, [pos, vel]) in [[0, 3], [1, 4]].iter().enumerate() {
            next[*pos] += a[axis] * dt * dt / 2.0;
            next[*vel] += a[axis] * dt;
        }
        next[2] += wz * dt;
        self.x = next;
        let mut p = [[0.0; 7]; 7];
        for i in 0..7 {
            for j in 0..7 {
                for k in 0..7 {
                    for l in 0..7 {
                        p[i][j] += f[i][k] * self.p[k][l] * f[j][l];
                    }
                }
            }
        }
        for [pos, vel, bias] in [[0, 3, 5], [1, 4, 6]] {
            let g = [(pos, dt * dt / 2.0), (vel, dt)];
            for (i, gi) in g {
                for (j, gj) in g {
                    p[i][j] += f64::from(c.q_acc) * gi * gj;
                }
            }
            // Continuous noise integral from the impulse response [-t²/2,-t,1].
            let indices = [pos, vel, bias];
            let coeff = [-0.5, -1.0, 1.0];
            let power = [2, 1, 0];
            for i in 0..3 {
                for j in 0..3 {
                    let exponent = power[i] + power[j] + 1;
                    p[indices[i]][indices[j]] +=
                        f64::from(c.q_bias) * coeff[i] * coeff[j] * dt.powi(exponent)
                            / f64::from(exponent);
                }
            }
        }
        p[2][2] += f64::from(c.q_gyro) * dt * dt;
        self.p = p;
    }
    fn observe(&mut self, index: usize, z: f64, r: f64) {
        let s = self.p[index][index] + r;
        let residual = z - self.x[index];
        let old = self.p;
        for i in 0..7 {
            let k = old[i][index] / s;
            self.x[i] += k * residual;
            for j in 0..7 {
                self.p[i][j] = old[i][j] - k * old[index][j];
            }
        }
    }
    fn compare(&self, ekf: &Ekf) {
        let p = ekf.covariance();
        for i in 0..7 {
            assert!(
                (f64::from(ekf.x[i]) - self.x[i]).abs() < 0.003,
                "state {i}: {} vs {}",
                ekf.x[i],
                self.x[i]
            );
            for j in 0..7 {
                let tolerance = 0.0003 * (1.0 + self.p[i][j].abs());
                assert!(
                    (f64::from(p[i][j]) - self.p[i][j]).abs() < tolerance,
                    "P[{i},{j}]: {} vs {}",
                    p[i][j],
                    self.p[i][j]
                );
            }
        }
    }
}

#[test]
fn irregular_predictions_and_measurements_match_dense_reference() {
    let mut ekf = Ekf::new();
    ekf.x = [0.0, 0.0, 0.2, 4.0, -1.0, 0.0, 0.0];
    let mut reference = Reference::new(&ekf);
    let mut t = 0.0;
    for tick in 0..1800 {
        let dt = [0.003f32, 0.005, 0.011, 0.017, 0.04][tick % 5];
        t += f64::from(dt);
        let a = [
            (t * 0.2).sin() as f32 * 0.4 + 0.2,
            (t * 0.3).cos() as f32 * 0.2 - 0.1,
        ];
        ekf.predict(a[0], a[1], 0.01, dt);
        reference.predict(
            a.map(f64::from),
            f64::from(0.01f32),
            f64::from(dt),
            ekf.config,
        );
        if tick % 8 == 0 {
            let v = [4.0 + (t * 0.2).sin() as f32, -1.0 + (t * 0.3).sin() as f32];
            ekf.update_velocity(v[0], v[1]);
            reference.observe(3, f64::from(v[0]), f64::from(ekf.config.r_vel));
            reference.observe(4, f64::from(v[1]), f64::from(ekf.config.r_vel));
        }
        if tick % 13 == 0 {
            let pos = [(4.0 * t) as f32, (-t) as f32];
            assert_eq!(ekf.update_position(pos[0], pos[1]), UpdateResult::Accepted);
            reference.observe(0, f64::from(pos[0]), f64::from(ekf.config.r_pos));
            reference.observe(1, f64::from(pos[1]), f64::from(ekf.config.r_pos));
        }
        if tick % 31 == 0 {
            let yaw = 0.2f32 + (t as f32) * 0.01;
            assert_eq!(ekf.update_yaw(yaw), UpdateResult::Accepted);
            reference.observe(2, f64::from(yaw), f64::from(ekf.config.r_yaw));
        }
        if tick % 101 == 0 {
            ekf.update_bias(0.2, -0.1);
            reference.observe(5, f64::from(0.2f32), f64::from(0.05f32 * 0.05f32));
            reference.observe(6, f64::from(-0.1f32), f64::from(0.05f32 * 0.05f32));
        }
        reference.compare(&ekf);
    }
}

fn assert_psd(ekf: &Ekf) {
    let p = ekf.covariance();
    for [a, b, c] in [[0, 3, 5], [1, 4, 6]] {
        assert!([a, b, c]
            .iter()
            .all(|i| p[*i][*i].is_finite() && p[*i][*i] >= 0.0));
        for i in [a, b, c] {
            if p[i][i] == 0.0 {
                for j in [a, b, c] {
                    assert_eq!(p[i][j], 0.0);
                }
            }
        }
        if [a, b, c].iter().all(|i| p[*i][*i] > 0.0) {
            let correlation = |i: usize, j: usize| {
                f64::from(p[i][j]) / (f64::from(p[i][i]) * f64::from(p[j][j])).sqrt()
            };
            let x = correlation(a, b);
            let y = correlation(a, c);
            let z = correlation(b, c);
            assert!(x.abs() <= 1.00001 && y.abs() <= 1.00001 && z.abs() <= 1.00001);
            assert!(1.0 + 2.0 * x * y * z - x * x - y * y - z * z >= -0.0001);
        }
    }
    for i in 0..7 {
        for j in 0..7 {
            assert_eq!(p[i][j], p[j][i]);
        }
    }
}

#[test]
fn covariance_stays_symmetric_psd_through_turns_stops_and_resets() {
    let mut ekf = Ekf::new();
    for tick in 1..=20000 {
        let t = tick as f32 * 0.005;
        let angle = t * 0.2;
        ekf.predict(-angle.sin() + 0.6, angle.cos() - 0.35, 0.2, 0.005);
        if tick % 8 == 0 {
            ekf.update_velocity(5.0 * angle.cos(), 5.0 * angle.sin());
        }
        if tick % 101 == 0 {
            ekf.update_speed(5.0);
        }
        if tick % 2000 == 0 {
            ekf.zupt();
            ekf.lock_position();
            ekf.lock_yaw();
            ekf.update_bias(0.6, -0.35);
        }
        assert_psd(&ekf);
    }
    let before = ekf.covariance();
    assert!(!ekf.reset_covariance([f32::NAN; 7]));
    assert_eq!(before, ekf.covariance());
    assert!(ekf.reset_covariance([1.0; 7]));
    let p = ekf.covariance();
    for i in 0..7 {
        for j in 0..7 {
            assert_eq!(p[i][j], if i == j { 1.0 } else { 0.0 });
        }
    }
}

#[test]
fn gps_recovery_learns_changed_bias_after_parking_and_a_dropout() {
    let mut ekf = Ekf::new();
    for _ in 0..200 {
        ekf.zupt();
        ekf.lock_position();
        ekf.update_bias(0.0, 0.0);
    }
    for tick in 1..=16000 {
        ekf.predict(0.7, -0.2, 0.0, 0.005);
        // Ten seconds with no GPS after the first 20 seconds.
        if tick % 8 == 0 && !(4000..6000).contains(&tick) {
            ekf.update_velocity(6.0, 0.0);
        }
        assert_psd(&ekf);
    }
    assert!((ekf.velocity().0 - 6.0).abs() < 0.05 && ekf.velocity().1.abs() < 0.05);
    assert!((ekf.biases().0 - 0.7).abs() < 0.02 && (ekf.biases().1 + 0.2).abs() < 0.02);
}

#[test]
fn exact_measurements_and_invalid_noise_do_not_poison_coupled_state() {
    let mut ekf = Ekf::with_config(EkfConfig {
        r_vel: 0.0,
        ..EkfConfig::default()
    });
    ekf.predict(0.6, -0.35, 0.0, 0.02);
    ekf.update_velocity(5.0, 2.0);
    assert_psd(&ekf);
    let state = ekf.x;
    let covariance = ekf.covariance();
    ekf.update_velocity(5.0, 2.0);
    assert_eq!(ekf.x, state);
    assert_eq!(ekf.covariance(), covariance);
    for bad in [f32::NAN, f32::INFINITY, -1.0] {
        ekf.config.q_bias = bad;
        ekf.predict(1.0, 2.0, 0.0, 0.01);
        assert_eq!(ekf.x, state);
        assert_eq!(ekf.covariance(), covariance);
        ekf.config.r_vel = bad;
        ekf.update_velocity(3.0, 4.0);
        assert_eq!(ekf.x, state);
        assert_eq!(ekf.covariance(), covariance);
    }
}

#[test]
fn coupled_filter_recovers_from_one_bounded_velocity_outlier() {
    let mut ekf = Ekf::new();
    ekf.x[3] = 5.0;
    for tick in 1..=12000 {
        ekf.predict(0.6, -0.35, 0.0, 0.005);
        if tick == 6000 {
            ekf.update_velocity(90.0, 0.0);
        } else if tick % 8 == 0 {
            ekf.update_velocity(5.0, 0.0);
        }
        assert_psd(&ekf);
    }
    assert!((ekf.velocity().0 - 5.0).abs() < 0.05 && ekf.velocity().1.abs() < 0.05);
    assert!((ekf.biases().0 - 0.6).abs() < 0.02 && (ekf.biases().1 + 0.35).abs() < 0.02);
}

#[test]
fn rejected_position_and_corrupt_measurements_preserve_coupled_state() {
    let mut ekf = Ekf::new();
    for _ in 0..100 {
        ekf.predict(0.2, -0.1, 0.0, 0.01);
        ekf.update_velocity(5.0, 2.0);
    }
    let x = ekf.x;
    let covariance = ekf.covariance();
    let nis = ekf.get_nis_position_avg();
    assert_eq!(ekf.update_position(1e6, -1e6), UpdateResult::Rejected);
    for bad in [f32::NAN, f32::INFINITY] {
        ekf.update_velocity(3.0, bad);
        ekf.update_bias(bad, 0.0);
        ekf.update_position(0.0, bad);
    }
    assert_eq!(ekf.x, x);
    assert_eq!(ekf.covariance(), covariance);
    assert_eq!(ekf.get_nis_position_avg(), nis);
}

#[test]
fn vector_gps_recovers_position_and_velocity_after_unmodeled_outage_error() {
    let mut ekf = Ekf::new();
    for _ in 0..200 {
        ekf.zupt();
        ekf.lock_position();
        ekf.update_bias(0.0, 0.0);
    }
    for tick in 1..=16000 {
        let dropout = (4000..6000).contains(&tick);
        // Unknown acceleration error appears only during the ten-second outage.
        ekf.predict(if dropout { 1.7 } else { 0.7 }, -0.2, 0.0, 0.005);
        if tick % 8 == 0 && !dropout {
            ekf.update_position(6.0 * tick as f32 * 0.005, 0.0);
            ekf.update_velocity(6.0, 0.0);
        }
        assert_psd(&ekf);
    }
    assert!((ekf.velocity().0 - 6.0).abs() < 0.05 && ekf.velocity().1.abs() < 0.05);
    assert!((ekf.position().0 - 480.0).abs() < 0.2 && ekf.position().1.abs() < 0.2);
    assert_eq!(ekf.rejected_velocity, 0);
}
