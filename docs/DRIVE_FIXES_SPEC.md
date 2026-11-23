# rc.17 drive follow-up

The drive review identified browser collection pauses during native dialogs and
an EKF-first speed policy that can override available fresh receiver velocity.
Exact diagnostic values and original exports remain in private validation inputs.

## Acceptance and testing decisions

- Prefer finite, physically bounded fresh GPS ENU velocity over EKF for telemetry
  and the scalar speed supplied to the force display. A valid GPS zero must beat
  EKF drift. Keep components and magnitude from one source; use EKF only when GPS
  is unavailable/stale/invalid, with current browser stale-data suppression intact.
  Do not tune estimator covariance or claim GPS is physical ground truth.
- Regress the observed speed-selection case and nonfinite, stationary, threshold,
  bounds and absent-source cases on lat. No raw coordinates, timestamps or original
  user exports may enter the repository.
- Remove native blocking prompt/confirm/alert calls from the dashboard's app flows.
  Keep confirmation for destructive actions, explicit cancellation, safe default,
  keyboard navigation and focus return. Recording/timing/polling must continue while
  waiting for input. The browser's beforeunload protection remains intentional.
- Await decisions at their callers and recheck affected route/timing ownership after
  waiting. Cancel and repeated submissions must not delete a capture, stop a newly
  selected timing owner, duplicate a save or arm stale geometry.
- Preserve transactional storage/recovery and existing exported schemas. Do not
  imply phone-suspension durability: the firmware GPS evidence ring is bounded,
  and existing missing-event markers must remain visible.
- Reproduce representative failures before fixes where feasible; run full browser,
  host, protocol, no_std and both receiver builds on lat. Review new dialogs in
  portrait/compact landscape/light/dark, including a dialog held open longer than
  the 2.56 s GPS buffer while receiver collection continues.
- Independently review a pinned final diff. Update the generated demo, relevant
  screenshots, documentation and changelog; publish a tested M9N release and verify
  the phone installer's version and checksum.

This work fixes speed source selection and browser collection interruptions. It
does not validate physical speed/force accuracy, repair the underlying EKF's
estimation error, or establish independent lap-timing accuracy. Keep those hardware
and R&D issues open.
