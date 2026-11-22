# Real drive replay

`docs/real-drive.html` is the maintained website source for a replay derived from
the maintainer's rc.17 M9N recording and saved track export. The homepage links
to it separately from the synthetic dashboard demo. It is an export analysis,
not a firmware/dashboard feature.

Acceptance:

- Play both recorded trajectories at 4× or scrub elapsed lap seconds. Lap selection
  controls both route visibility and the GPS speed chart.
- Show stored lap times of 81.135 and 75.912 seconds, a 5.223 second difference.
  Stop each marker at its own lap endpoint and stop playback at the final endpoint.
- Use equal metre scales with north to the right and west up. Coordinates retain
  the real route shape but omit the geographic origin, absolute timestamps and
  raw exports. Do not publish the original CSVs or track JSON.
- Explain approximate reconstructed replay boundaries, host receipt times,
  interpolated positions/speeds, elapsed-time comparison and lack of an independent
  timing reference. Do not claim physical accuracy from update intervals.
- Support mobile widths, keyboard controls and reduced motion. No automatic
  playback; reduced-motion visitors retain the slider without the Play button.
- Route playback must work if the pinned external speed-chart module is unavailable;
  show an explicit chart failure with online reload guidance.
- Retain website navigation, canonical metadata and sitemap coverage. No dependency
  on the conversation host or access to recording databases.

Data is embedded as `data-drive-data`: two arrays of elapsed seconds, east metres,
north metres and receiver speed in km/h, plus relative centerline and timing line.
Every second retained GPS event is used for replay (approximately 12.5 Hz); chart
rows interpolate at 0.25 seconds with exact lap endpoints. Stored lap times are
reported separately from this approximate reconstruction. The captured receiver
epochs in both completed laps have uninterrupted 40 ms intervals. That evidence
does not establish physical timing accuracy or continuous logging outside those laps.

The original inline visualization was exported on lat with the Visualize renderer,
then adapted to the existing GitHub Pages project. The public page uses local Inter
and JetBrains Mono fonts and the version-pinned Visualize chart runtime. Update
the dataset and explanatory text together. Keep the source data private and recheck
provenance and coordinate fields before replacing the embedded example.

Validation runs on lat: build the docs, run `scripts/check_docs_site.py`, then use
Chrome to check playback, selection, endpoint behavior, reduced motion, chart
failure, and mobile overflow. The normal docs deployment publishes the static page.
