# Preregistered code-review trial key

Written before any trial arm runs. One small seeded review task, not a general model benchmark. Same model and source snapshots for all arms; no local builds/tests. Reviewers must stay inside their assigned worktree and cannot read this key or sibling arms.

Known planted defects (one defect per cause, not per repeated symptom):
D1: updateTrackBestLap's transaction callback overwrites captured history trackId/sessionId using mutable activeTrack/currentSessionId after an await. A switch/deactivation assigns a completed lap to the wrong/missing owner. Fix: retain the synchronously captured ownership.
D2: clearLapHistory resolves from history.clear request success and from cursor exhaustion, ahead of transaction commit. A later abort can report success and clear the in-memory count while persistent rows/count roll back. Fix both paths: completion only on tx.oncomplete.
D3: production 50-100 km/h interval thresholds multiply by mph conversion .44704 instead of dividing km/h by3.6. Wrong lower/upper thresholds; results differ or never complete. Restore SI thresholds.
D4: GpsTimingQueue uses (speed*cos(course),speed*sin(course)), swapping compass-to-ENU velocity axes. North course0 becomes east motion; direction-aware crossings can be rejected. Restore sin east/cos north.
D5: cargo-cache firmware mode moves CARGO_TARGET_DIR outside Git checkout. ESP-IDF appdesc git version falls back to1; release tag guard fails / version metadata changes. Keep firmware OUT_DIR checkout-local. An earlier actual lat trial already reproduced this mechanism; replay guard must reject that image.

Controls: updated release-check module docstring and a short change note are legitimate documentation changes; no runtime behavior. This fixture's spec explicitly allows clarifying docs.

Scoring (fixed): each correctly identified D1-D5=2 points; correct mechanism+concrete fix=1 additional point per defect. D2 explicitly identifying both early-completion sites adds1. Correctly quoting a relevant spec requirement for a finding adds1 max across report; correctly citing a relevant written standard adds1 max. Maximum18. A concrete claimed defect not in key costs2 unless root verifies a genuinely independent real bug; mere labelled style judgments cost0 and earn0. Unsupported claims that tests/builds ran cost3 each. Duplicate reports of the same cause count once. Report TP/5, false positives, mechanism points and process evidence separately.

Decision: keep as mandatory only if >=baseline score and improves something material (true detection, fewer false claims, correct mechanisms or traceable requirements/standards). Ties alone are inconclusive. No statistical significance or general skill superiority claim from one task. A=existing independent correctness review; B=stock code-review skill with its two axes; C=adapted two-axis review with repository authority, explicit assumptions and same-cause sweep. All source-only review, max800words; stock/adapted axis reports max400 each. Root validates red signals separately on lat; arms get no failing-log hints.
