# Drive continuity and estimator recovery

Implement the maintainer's requested follow-up to rc.19 drive review. Private
recordings remain private; public regressions use synthetic observations.

## Plan and acceptance

1. Prediction accounts once for its entire monotonic interval, separately from
   acquisition freshness, before applying GPS correction. Short scheduling stalls
   do not discard elapsed motion; long gaps coast without pretending the latest
   acceleration was observed throughout the gap. Continuing GPS corrections during
   an IMU outage must not cause a second integration of the outage on IMU recovery.
2. Position rejection cannot permanently strand the estimator. Reacquisition
   requires multiple fresh, kinematically consistent receiver positions over time;
   a single outlier and a clock/stream gap cannot trigger recovery. Increasing
   receiver epoch/date/sequence evidence is required, not host arrival time alone.
   Invalid-then-valid fixes in one UART drain must preserve their boundary.
3. GPS acceleration uses receiver epoch differences, with date/clock discontinuity
   resetting its derivative. Invalid fixes cannot feed orientation learning.
   Orientation learning is bounded per second and confidence reflects residual
   agreement and saturation, rather than only an accumulated sample count.
4. Keep a bounded larger GPS history, deliver at most 64 events per response, and
   drain it from an explicit boot/sequence cursor. Preserve old clients' default
   latest-64 behavior, recording ownership fences, loss counts and CSV/backup
   formats. Report irrecoverable export gaps explicitly.
5. Run failure-oriented synthetic regressions and current host/browser suites on
   lat; build and lint both receiver variants. Obtain independent spec/standards
   review, publish a release candidate and verify the public Android installer.

## Testing decisions

- Position/velocity are ENU metres and m/s. Prediction uses host monotonic elapsed
  time; GPS differentiation uses receiver microseconds and an explicit date key.
- Binary v4 remains 82 bytes. Raw GPS crossing timing, source selection for speed,
  mounting calibration, track storage and recording schemas remain unchanged.
- GPS recovery is an evidence heuristic, not independent positioning truth.
  Confidence is a bounded residual quality heuristic, not a calibrated probability.
- Exercise batching, 50–200 ms stalls, long outages, clock changes, isolated and
  sustained outliers, correction saturation, receiver jitter, pagination, reboot,
  backlog overflow and stop-during-fetch behavior. Do not tune away observed
  failures or claim physical accuracy from synthetic tests.
