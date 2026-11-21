# Website media

`circuit-hero.mp4` and `circuit-poster.jpg` are adapted from **Aerial view footage of motor racing track** by **K** (Pexels profile `@kelly`), selected by the project owner:

- Source: https://www.pexels.com/video/aerial-view-footage-of-motor-racing-track-9737823/
- Download: https://videos.pexels.com/video-files/9737823/9737823-uhd_2560_1440_24fps.mp4
- License: https://www.pexels.com/license/ (checked 2026-11-20)

Pexels permits website use and modification without mandatory attribution. This media is covered by that license, not the software's MIT license. It is decorative circuit footage, not a Blackbox measurement or an endorsement by the depicted participants, venue or creator. It is an aerial view, not a driver's onboard camera.

Encoded on lat with FFmpeg: source seconds 3–21, resize 1280×720, 24 fps, saturation 0.45, contrast 1.03, brightness −0.02, fade first/last second, remove audio, H.264 CRF 28 and fast-start MP4. Poster is at 7 seconds in the adaptation. A dark CSS overlay keeps text legible. The old city/tunnel assets were removed.

Reduced-motion and data-saving preferences retain a still image. Playback pauses offscreen or in hidden tabs. The footer's Display preferences exposes a manual pause; the hero has no controls or stock-footage links. Both source and poster are hosted with the site.

## Instrument fonts

Inter and JetBrains Mono Latin variable WOFF2 subsets are stored in `fonts/`, with OFL license texts and exact source URLs/byte counts in `fonts/sources.json`. The website serves these locally. `scripts/embed_dashboard_fonts.py` embeds the identical files into the device HTML, which therefore needs no internet font request. Preserve the licenses when redistributing.
