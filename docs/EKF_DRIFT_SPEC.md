# EKF drift follow-up

The drive review found disagreement between the original estimator-backed speed
and fresh receiver velocity. The exported dashboard rows contain displayed force
values and aligned heading, not the raw 200 Hz IMU/attitude inputs or the complete
filter state. They cannot reproduce the original firmware run exactly.

## Acceptance and testing decisions

- Diagnose a specific estimator defect before changing tuning. Reproduce it in
  synthetic host regressions on lat, including the effect of splitting prediction
  intervals and a persistent earth-frame acceleration error with fresh GPS.
- Retain position/velocity/bias correlations within each ENU axis so repeated
  prediction steps propagate persistent bias uncertainty coherently and receiver
  observations can correct the learned earth-frame bias while moving.
- Keep the existing process/measurement tuning values, ENU units and state order.
  Document the covariance structure precisely, including omitted yaw/axis coupling.
  Use numerically stable measurement updates and consistent covariance resets.
- Verify against an independent dense-matrix reference, analytic covariance,
  positive-semidefinite/symmetry checks, 5/25 Hz GPS, irregular prediction intervals,
  dropouts/recovery, corrupt inputs, stationary transitions and sustained turns.
- Use the latest drive's receiver timeline privately as an additional stress input.
  Clearly label injected/model-derived acceleration; it is not a raw-IMU replay or
  independent physical ground truth. Compare before/after under identical inputs.
  Do not publish original coordinates, timestamps or diagnostic series.
- Preserve rc.18 fresh-GPS display selection, raw-GPS lap crossings, telemetry v4,
  recording/backup schemas, startup GPS configuration and calibration behavior.
- Run full host/browser/Python/protocol and both receiver builds on lat, obtain
  independent pinned Spec/Standards review, merge and publish a tested release.
  Verify the phone installer selects the new M9N image/version/checksum.

This work establishes software behavior under the stated model. It does not
validate sensor mounting, yaw semantics, physical speed/timing accuracy, covariance
confidence coverage or universal drift-free operation.
