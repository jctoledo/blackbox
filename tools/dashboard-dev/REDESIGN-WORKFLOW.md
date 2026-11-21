# Dashboard design review workflow

1. Read [the current design contract](../../CLAUDE-design.md). Edit `sensors/blackbox/web/dashboard.html`; avoid maintaining a second production UI.
2. Use a `codex/` branch for agent work. Keep all builds, automated tests, font embedding and demo generation on lat. The CI workflow uses `blackbox-lat` and is dispatched after review of the branch.
3. Generate `docs/demo.html` with `scripts/build_demo.py`. This copies the actual dashboard and replaces device transport with synthetic data. Never present it as measured performance.
4. Review portrait (390×844), compact landscape (667×375), wide landscape (844×390), and short landscape (844×320). Check long names/times, missing reference, stale connection, recording, light/dark themes and keyboard focus.
5. Confirm rotation changes only layout. Dashboard must escape the cluster, portrait must restore controls, and recording/timing must continue. Keep warnings and recording state visible.
6. Refresh `docs/dashboard-screenshot.png` and `docs/dashboard-landscape.png` from the generated demo after visual acceptance. Update website copy when behavior changes.
7. Run host regressions and both GPS firmware builds on lat. Record what was tested; physical mounting, sunlight, glare and glance readability remain hardware checks.

The adjacent `index.html` is a historical simulator. Its neumorphic styling, localStorage recording and replay controls are not the current product contract.
