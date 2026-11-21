# Blackbox build infrastructure

All maintainer builds, automated tests, website generation and media encoding run on **lat**. Workflow commands and toolchain pins remain authoritative in `.github/workflows/`; this note explains the persistence choices and constraints.

## Runner and trust

The dedicated `blackbox-lat` runner runs in its existing container, limited to four CPUs and 8 GiB RAM, with no host directories or Docker socket mounted. Main/tag builds and maintainer-dispatched reviewed branches use it. Unreviewed fork code must not run automatically on this persistent runner.

One runner serializes its jobs. Another runner in a separate container gets separate storage. Sharing this cache between concurrent runners would require an additional locking design covering build and packaging; adding a cross-workflow concurrency group can also replace pending runs, so it is not a harmless substitute.

## What persists between jobs

`scripts/cargo-cache.sh` selects a Blackbox-only namespace inside the container's home directory. Configuration identity includes tracked Cargo manifests/lockfile, Cargo configuration, SDK defaults, partitions and relevant build overrides. Host output also includes the compiler identity. Environment values contribute to digests; credentials and network settings are not printed as cache metadata.

SDK/tool installations and host-test build artifacts persist. Whole configuration groups are retained/pruned together, with bounded host build keys inside them. Cache deletion is limited to this generated namespace. A new container or an evicted configuration legitimately starts cold.

Clean checkout remains enabled. Firmware output uses the checkout's `target` directory and is removed by the next clean checkout. This keeps generated native project files inside the repository ancestry ESP-IDF uses to determine the application version. `CARGO_WORKSPACE_DIR` is explicit so workspace SDK defaults remain discoverable.

## Why firmware target output stays in checkout

An external Cargo target trial compiled and packaged correctly but changed the ESP-IDF application version: generated native project files could no longer find the firmware repository's Git tags. A directory key containing a tag cannot fix that lookup. The published rc.10 image was built using the original pipeline and was unaffected.

The adopted design preserves SDK/tool downloads while retaining normal firmware target placement. Full persistent firmware target reuse would need a separately validated way to preserve clean source state, Git-derived version metadata, configuration and packaging paths. Fast repeats alone are insufficient evidence.

## Validation

Validation runs on lat and covers shell syntax, key stability/invalidation, bounded group retention, environment propagation failures, actual host tests, both receiver Clippy/release builds, generated SDK defaults and image packaging.

For firmware metadata, compare a clean tagged checkout and a clean untagged checkout against their actual Git-described versions. Check the merged ESP32-C3 header, application header/descriptor, partition placement, size and SHA-256. The internal ESP-IDF project name can be `libespidf`; compare the version and target metadata rather than assuming the project-name field equals the repository name.

Performance observations belong with the exact command, receiver variant, cache state and runner configuration. An empty target with an already installed SDK is different from a run that includes first-time downloads. Neither implies a universal build duration.

## Observations on lat, 2026-11-21

Firmware source `5ec5b47`, four-CPU/8-GiB container, pinned nightly/ESP-IDF configuration:

| Check | State | Observed elapsed |
| --- | --- | --- |
| Rust host regression packages | Empty host target | 8.001 s |
| Same host regressions | Repeated host target | 0.208 s |
| 6M Clippy + release build | Fresh checkout-local target, installed SDK | 104 s |
| M9N Clippy + release build | Fresh checkout-local target, installed SDK | 105 s |
| Untagged 6M Clippy + release build | Fresh checkout-local target, installed SDK | 105 s |

These are this container's observations, not a build-time guarantee. Firmware compilation remains in every clean job; the reusable SDK/tool installation was approximately 3.4 GiB. An earlier external-firmware-target trial was rejected after its application-version lookup changed. Its fast repeat times are not a result of the adopted design.

Both receiver builds retained the expected SDK defaults, compile-time receiver settings, C3 headers and partition layout. Tagged and untagged app descriptors matched their actual Git descriptions. The release workflow additionally checks the packaged application version against the requested tag before upload using `scripts/check_release_image.py`; it fails rather than publishing a mislabeled image.
