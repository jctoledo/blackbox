# Historical dashboard simulator

`index.html` preserves an earlier visual experiment. It is **not the production dashboard**, does not reproduce current storage/backup/timing behavior, and must not be copied into firmware.

The maintained UI is [`sensors/blackbox/web/dashboard.html`](../../sensors/blackbox/web/dashboard.html). Firmware embeds that file with `include_str!`. The public demo is generated from the same source by [`scripts/build_demo.py`](../../scripts/build_demo.py), with explicitly synthetic telemetry and isolated demo databases.

For current work, edit the production source, generate the demo on lat, serve the repository through a static HTTP server, and review `docs/demo.html`. See [the design review workflow](REDESIGN-WORKFLOW.md). Old simulator CSV replay is a prototype, not a shipped Blackbox replay feature.
