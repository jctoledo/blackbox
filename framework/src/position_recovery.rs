//! Reacquisition after position gating, using a sequence of coherent GPS fixes.
//! This is a continuity heuristic, not independent receiver accuracy evidence.

#[derive(Default)]
pub struct PositionRecovery {
    previous: Option<([f32; 2], [f32; 2], u64)>,
    started_us: u64,
    count: u32,
}

impl PositionRecovery {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Only call for rejected, valid fixes. A gap or inconsistent step restarts
    /// collection; at least eight fixes spanning half a second are required.
    pub fn observe(&mut self, position: [f32; 2], velocity: [f32; 2], at_us: u64) -> bool {
        if position
            .iter()
            .chain(velocity.iter())
            .any(|v| !v.is_finite())
            || velocity[0].hypot(velocity[1]) >= 100.0
        {
            self.reset();
            return false;
        }
        let consistent = self.previous.is_some_and(|(p, v, at)| {
            let dt = at_us.saturating_sub(at) as f32 * 1e-6;
            let expected = [
                (v[0] + velocity[0]) * dt * 0.5,
                (v[1] + velocity[1]) * dt * 0.5,
            ];
            (0.01..=0.3).contains(&dt)
                && (position[0] - p[0] - expected[0]).hypot(position[1] - p[1] - expected[1]) <= 5.0
        });
        if !consistent {
            self.started_us = at_us;
            self.count = 0;
        }
        self.previous = Some((position, velocity, at_us));
        self.count += 1;
        self.count >= 8 && at_us.saturating_sub(self.started_us) >= 500_000
    }
}
