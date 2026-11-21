# Blackbox dashboard design contract

The device UI is an instrument for drivers. Prioritize glanceable speed, elapsed time, reference delta and clear timing/connection state. Setup and detailed analysis belong to parked use.

## Source and modes

Edit `sensors/blackbox/web/dashboard.html`. `scripts/build_demo.py` generates the public demo from that same file. `tools/dashboard-dev/index.html` is historical and must not replace production.

- Normal portrait: timing first, speed second, explicit Tracks / Record session / Saved data controls.
- Drive view: hides editing controls and force analysis while retaining status and warnings.
- Landscape cluster: automatically activates on landscape viewports at least 600 px wide and no more than 600 px high, unless Analysis or the explicit Dashboard escape is selected. A selectable instrument occupies the left panel; timing/reference occupies the right when a track is selected. With no track, the right panel shows recording-backed Drive readiness, duration, partial GPS distance and health; see `docs/DRIVE.md` for ownership, clocks and recovery. Orientation never starts or stops recording/timing.
- Analysis: optional force visualization for parked review; not the default driving view.

## Typography and styling

Inter is the interface font; JetBrains Mono is the numeric/technical font. Latin WOFF2 subsets are bundled into the firmware HTML and served locally by the website. Keep their OFL licenses. Run `scripts/embed_dashboard_fonts.py --check` on lat after changes. Website display headlines may use a distinct marketing face; instrument numbers must match the demo and device.

Use the existing CSS variables for light/dark themes. Lime marks active timing, amber caution, red faults/recording, with text so color is never the only signal. Keep controls at least 44 px high where practical; avoid decorative animation in the driving cluster. Use consistent existing SVG icons rather than mixing emoji, arbitrary Unicode icons and new icon libraries.

## Data presentation

- Distinguish elapsed time, last completed time, historical best and the loaded reference.
- Display only validated values. Missing is a dash/message, not zero.
- Hide live delta/progress when reference matching is unavailable. Route percentage means matched reference distance, not a completion guarantee.
- Keep full saved-track names visible in management lists. The compact driving header may truncate a name; setup must retain the full name.
- Preserve stale-data warnings, recording state, save failures and pending-data recovery.
- No turn callouts, apex precision, drag times or slope correction until implemented and validated.
- Prefer tabular digits and stable bounds. Test long elapsed values and three-digit speed without overlap.

## Review and assets

Follow [the UI review workflow](tools/dashboard-dev/REDESIGN-WORKFLOW.md). All builds/tests/generation run on lat. Update portrait and landscape screenshots and website descriptions with UI changes. Synthetic demo data must stay explicitly labeled. Physical in-car readability and accuracy require driving evidence; attractive graphics do not establish either.
