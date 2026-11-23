//! Fixed-size covariance algebra for one [position, velocity, earth bias] axis.
//! No allocation or tuning changes. The caller owns the corresponding state.

pub(super) type Covariance = [[f32; 3]; 3];

fn multiply(a: Covariance, b: Covariance) -> Covariance {
    let mut result = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for (k, row) in b.iter().enumerate() {
                result[i][j] += a[i][k] * row[j];
            }
        }
    }
    result
}

fn transpose(a: Covariance) -> Covariance {
    core::array::from_fn(|i| core::array::from_fn(|j| a[j][i]))
}

fn symmetric(mut p: Covariance) -> Covariance {
    for (i, j) in [(0, 1), (0, 2), (1, 2)] {
        let mean = (p[i][j] + p[j][i]) * 0.5;
        p[i][j] = mean;
        p[j][i] = mean;
    }
    p
}

pub(super) fn predict(p: Covariance, dt: f32, q_acc: f32, q_bias: f32) -> Covariance {
    let dt2 = dt * dt;
    let dt3 = dt2 * dt;
    let dt4 = dt3 * dt;
    let dt5 = dt4 * dt;
    let f = [[1.0, dt, -0.5 * dt2], [0.0, 1.0, -dt], [0.0, 0.0, 1.0]];
    let mut next = multiply(multiply(f, p), transpose(f));
    // Held acceleration-sample variance: q_acc * G G^T, G=[dt²/2,dt,0].
    let g = [0.5 * dt2, dt, 0.0];
    for i in 0..3 {
        for j in 0..3 {
            next[i][j] += q_acc * g[i] * g[j];
        }
    }
    // Exact integral for continuous bias random walk through p'=v, v'=a-b.
    let bias_noise = [
        [dt5 / 20.0, dt4 / 8.0, -dt3 / 6.0],
        [dt4 / 8.0, dt3 / 3.0, -dt2 / 2.0],
        [-dt3 / 6.0, -dt2 / 2.0, dt],
    ];
    for i in 0..3 {
        for j in 0..3 {
            next[i][j] += q_bias * bias_noise[i][j];
        }
    }
    symmetric(next)
}

/// Scalar observation, returning its covariance and gain. Joseph form retains
/// symmetry and avoids the subtractive P-KHP update near a strong observation.
pub(super) fn observe(p: Covariance, h: [f32; 3], noise: f32) -> Option<(Covariance, [f32; 3])> {
    if !noise.is_finite() || noise < 0.0 {
        return None;
    }
    let ph: [f32; 3] = core::array::from_fn(|i| (0..3).map(|j| p[i][j] * h[j]).sum());
    let innovation_variance = noise + (0..3).map(|i| h[i] * ph[i]).sum::<f32>();
    if !innovation_variance.is_finite() || innovation_variance <= 0.0 {
        return None;
    }
    let gain = ph.map(|v| v / innovation_variance);
    let a = core::array::from_fn(|i| {
        core::array::from_fn(|j| (if i == j { 1.0 } else { 0.0 }) - gain[i] * h[j])
    });
    let mut next = multiply(multiply(a, p), transpose(a));
    for i in 0..3 {
        for j in 0..3 {
            next[i][j] += gain[i] * noise * gain[j];
        }
    }
    let next = symmetric(next);
    if next.iter().flatten().any(|v| !v.is_finite()) || (0..3).any(|i| next[i][i] < 0.0) {
        return None;
    }
    Some((next, gain))
}
