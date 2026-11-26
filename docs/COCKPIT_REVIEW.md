# Cockpit UI review — 2026-11-25

Three independent agent reviews covered motorsport cockpit hierarchy, typography/numeric alignment, and interaction/editorial design. This is a software design review, not a claim of review by a human race-car designer or of physical in-car acceptance.

## Findings and implementation

| Finding | Implemented response |
| --- | --- |
| Browser “Page time” competed with lap/recording duration | Moved browser age into Diagnostics; Max occupies the speed footer rail. |
| Centered delta changed width when trend arrows appeared | Removed decorative trend arrows; used JetBrains Mono, tabular digits and a reserved numeric width. |
| Primary values competed with repeated explanatory prose | Reduced completion banner to Last/Best plus its exact time; shortened missing-reference and connection states; hid maneuver classification outside Analysis. |
| Main controls wrapped into multiple portrait rows | Kept Tracks / Record / Saved data; moved Analysis, Drive view and Full screen into the menu. |
| Runs results were small relative to speed and timing | Enlarged Last, retained a smaller Best, and shared the instrument header with the six-option selector. |
| Drive health descriptions could push metrics into scrolling content | Kept primary metrics fixed and displayed concise health/coverage states; detailed explanations remain in titles and saved evidence. |
| Padding treatment was inconsistent | Applied 20% opacity to speed padding and leading integer zeros in time values, including dynamically rendered history/durations. Fractional precision, signs, text content and exports remain unchanged. |
| Instrument picker could hide its keyboard focus | Returned focus on selection/Escape and added outside/focus-leave dismissal. |
| Simultaneous warnings could bury an unsaved-data error | Ordered recording/storage notices before connection notices; retained explicit persistent warning text. |

The two visual reviewers inspected the resulting portrait, landscape, Runs and six-option picker captures. No blocking design defect remained. Their final polish findings—portrait Max spacing and the longest compact interval label—were included in the final checks.

## Validation and limits

- Browser inspection confirmed the actual rendered speed glyphs use the embedded custom JetBrains Mono font; both bundled font faces loaded.
- Speed width and unit position remained fixed across 000, 009, 035, 099, 100 and 199. Delta trends no longer modify its displayed numeric string.
- Reviewed 390×844, 667×375, 844×390 and 844×320 in both themes, including expanded Controls, long time strings and the interval selector. Keyboard focus, multiple warnings, GPS-log loss and the longest metric label also passed. Long elapsed values were checked against the delta column, not only the card boundary. Fractional zeros remained opaque after repeated observer updates.
- Receiver-time tests cover 5 Hz and 25 Hz standing launches, creeping, gaps, course/clock boundaries, partial distance crossings, provenance and saved-evidence reconstruction. [Timing conventions and research](research/PERFORMANCE_RUNS.md), [implemented run contract](RUNS.md).
- The integrated current-main checkout passed all 408 JavaScript checks on lat before the final documentation and persistence projection polish. Final exact-head CI is required before merging; earlier firmware checks at an older baseline are not evidence for this shipping commit.
- Final independent source review identified compact countdown overflow, long replay-title collisions, replay-switch loading races and mobile chart labels. These were addressed with compact bounded dialogs, reserved/truncated titles, transport locking and responsive SVG axes. Site/browser captures cover homepage, embedded player, guides and the existing real-drive chart at phone and desktop widths.
- Five-second capture browser checks confirm stationary enrollment, cancellation, fresh receiver evidence, and automatic route-recording start. The start-line center is covered by a regression assertion. Position averaging reduces jitter, not systematic GPS bias.
- Stored SIM circuit/rolling/standing examples use the same read-only chunk reader as saved recordings. Public screenshots are synthetic. The private recent-recording review uses only an approved relative force/speed projection and is excluded from the website and repository.


Desktop emulation cannot establish sunlight/glare readability, real phone behavior, physical launch accuracy or parity with external timers. These edits do not flash a connected device or publish a release.
