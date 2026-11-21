# Build and publishing scripts

Run builds, tests and generated-asset work on lat. CI is explicitly configured for Blackbox's two GPS variants; adding a new sensor application needs workspace, target and workflow review.

- `embed_dashboard_fonts.py`: embeds the licensed Latin WOFF2 files into production HTML; `--check` verifies that the committed block matches the font files.
- `build_demo.py`: generates `docs/demo.html` from production HTML and `docs/demo.js`; demo data and storage are synthetic/isolated.
- `build_docs.py`: renders maintained public guides and sitemap. Requires Python Markdown 3.7 and also generates the demo.
- `generate_flasher_config.py`: reads `sensors/*/project.json`, writes project metadata to stdout and (unless `CI=true`) `docs/projects-config.js`. `VERSION` and `GITHUB_REPOSITORY` select release links. Actual flash downloads use Pages-hosted firmware and manifest verification.
- `setup-partition.sh`: historical helper that rewrites root sdkconfig with a machine-specific absolute path. The current flash runner and release packager explicitly pass the root `partitions.csv`; do not run this helper for the standard workflow.

Release discovery scans `project.json`, but the current build step targets ESP32-C3 and applies 6M/M9N variants. A discovered S3 application will not automatically gain a valid build. Never reuse the C3 RISC-V target for an Xtensa S3 project.

The workflows are the authoritative packaging instructions. Regeneration is not proof of a physical USB flash/boot; see `docs/HARDWARE_VALIDATION.md`.
