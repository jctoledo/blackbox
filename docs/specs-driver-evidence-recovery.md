## Problem Statement
Remaining hardware/R&D issues still have software prerequisites: startup bias can accept non-contiguous evidence, delivered GPS-rate diagnostics lose batched fixes, captured tracks can become unreachable after save failure, and the driver UI can misstate recording/evidence scope or current GPS collection health.

## Solution
Fix reproducible software failures while retaining original data and honest limits. Drivers see unavailable GPS evidence directly, understand which measurements belong to the page versus a recording, and can recover/export a captured route when browser storage fails. Hardware/R&D issues remain open for their actual evidence.

## User Stories
1. As a driver, I need calibration to require fresh contiguous stationary samples after a sensor outage.
2. As a driver, I need previous saved bias preserved when calibration fails.
3. As a tester, I need delivered GPS-rate diagnostics to count every valid post-warmup RMC in a UART batch exactly once.
4. As a driver recording a route, I need an invalid early finish to retain a working Continue route action.
5. As a driver, I need a failed track save to retain the completed route, show an actionable error, and allow retry or a portable JSON download without another storage write.
6. As a driver, I need saved versus armed versus arming-failed states distinguished, without duplicated tracks or unintended timing transitions.
7. As a driver, I need a reference save failure to be visible and never represented as a saved reference.
8. As a driver using Runs, I need the status to stop claiming collection when the receiver log is unavailable, stale or waiting for messages, even while ordinary telemetry remains connected.
9. As a driver, I need accepted results retained through temporary HTTP failures; missing evidence must be judged by the existing sequence/time gates, not guessed from transport failures alone.
10. As a user, I need page maxima/peaks and the half-second averaging window identified visibly on touch screens.
11. As a user, I need compact portrait/landscape screens, fresh demo assets and website guidance consistent with the actual application.

## Implementation Decisions
- Keep physical validation in #73/#55/#79/#153 and other R&D issues open. No new accuracy, mounting, F1, slope, standing-start or background-recording claims.
- Calibration resets candidate collection after an acceleration-evidence gap exceeding the existing 200 ms freshness window, including recovery with fresh angle/gyro packets. Retain minimum sample spacing, full fresh sample count, 30-second overall deadline and failure-preserves-bias behavior. Clock/sample collection seam must exercise the actual application gating in host tests.
- Count each valid post-warmup RMC at the ingest boundary; do not derive delivered rate from a consumed boolean or change latest-only EKF assimilation, raw timing queue, logging or configured-rate labels.
- Track capture keeps stable ownership and one pending finalized identity until save commits. Invalid finish must not consume capture. Failed saves keep retry, continue where valid, explicit discard and portable version 2 track JSON recovery; originals are never overwritten. Guard duplicate clicks and async owner changes. A committed save followed by failed arming is a different state: retain saved identity and retry arming without saving again. Existing explicit 'Save track & arm' intent may remain, with truthful success/error copy. Release completed recorder ownership after safe completion so nearby-track suggestions are not permanently suppressed. Warn before abandoning unsaved geometry; do not require a storage write to export recovery.
- Reference write failures use an existing visible warning or scoped track error, preserving prior stored reference and preventing false success. No speculative redesign of reference selection math.
- Receiver-log UI health is recording-owner-bound and monotonic. Distinguish transport unavailable, connected/waiting, and stale event delivery. Keep latest accepted results; do not fabricate an invalid run solely from an HTTP failure when retained receiver evidence may subsequently be complete. Existing sequence/time/quality gates remain authoritative on recovery.
- Identify page measurements explicitly rather than resetting or relabeling them as recording-owned measurements. Keep the driving layout glanceable and do not reintroduce long explanatory paragraphs in the Peaks instrument. The recording-backed Drive panel retains its existing semantics.
- Add a labeled synthetic demo control to pause GPS logging while ordinary telemetry continues for reproducible visual review. Existing simulation labels stay visible where required; device UI gets no test-only controls.
- Independent implementation lanes: GPS accounting; calibration continuity; track persistence/recovery; driver health/copy/layout. Workers own isolated branches/worktrees. Dependencies start only after prerequisites merge. Root refreshes final generated website/demo screenshots after functional merges.

## Testing Decisions
Use existing host/browser test seams at the highest observable application behavior. Reproduce failures against baseline before fixing where feasible. Calibration covers 149 old samples, a long silence, then one fresh sample (must not complete), continuous fresh recovery, stale/invalid/moving evidence, spacing/clock boundaries and timeout/bias retention. GPS covers multiple valid RMCs, mixed valid/invalid, warmup, non-RMC input and repeated drains, preserving latest-only estimation. Track tests cover invalid finish+continue, transaction quota/abort, retry same ID, no duplicate clicks, export after storage failure, save success+arm failure, cancellation/owner races and failed reference writes. UI tests cover live ordinary telemetry plus failed/empty/stale GPS log, owner changes, recovery and retained results. Do not add tests that merely repeat CSS or copy implementation. Inspect rendered 390×844, 667×375, 844×390 and 844×320 views; capture actual generated UI assets at native proportions. All tests/builds/docs/demo generation and image encoding run on lat. Independent pinned Spec/Standards review precedes substantial merges; run relevant checks and full final firmware CI on lat. Physical sunlight/readability, mounting, receiver cadence/accuracy and native-phone storage acceptance remain human evidence.

## Out of Scope
Hardware ports/schematics, automatic arbitrary-mount calibration, full EKF redesign, autonomous device recording, cloud OAuth/providers, standing-start/quarter-mile timing, learned coaching and fabricated road-test evidence.
