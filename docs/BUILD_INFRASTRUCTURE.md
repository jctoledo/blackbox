# Blackbox build infrastructure

All maintainer builds, automated tests, website generation and media encoding run on **lat**. Workflow commands and toolchain pins remain authoritative in `.github/workflows/`; this note explains the persistence choices and constraints.

## Runner and trust

The dedicated `blackbox-lat` runner runs in its existing container, limited to four CPUs and 8 GiB RAM, with no host directories or Docker socket mounted. Main/tag builds and maintainer-dispatched reviewed branches use it. Unreviewed fork code must not run automatically on this persistent runner.

One runner serializes its jobs. Another runner in a separate container gets separate storage. Sharing this cache between concurrent runners would require an additional locking design covering build and packaging; adding a cross-workflow concurrency group can also replace pending runs, so it is not a harmless substitute.

## Reviewed PR controller

`.github/workflows/reviewed-pr.yml` accepts a PR number and full reviewed head SHA only through an explicit dispatch from `main`. The trusted controller refuses a non-main dispatch, then uses GitHub's PR API to require an open same-repository PR targeting main and an equal current head. No PR open/update event schedules code. The same guard runs again immediately before immutable checkout in reusable `ci.yml`; checkout credentials are disabled and `HEAD` is checked against the reviewed SHA. Main push and ordinary manual branch behavior are preserved. The [contribution guide](../CONTRIBUTING.md#reviewed-pr-validation) gives exact dispatch commands.

The wrapper and guard/status helper come from the dispatch's main commit. Its authorization and final reporting jobs receive `statuses: write`; the reused job that executes reviewed source retains only read permissions. Existing validation commands, runner resources and cache policy are reused. This permission split limits the build token; it does not make a persistent runner safe for arbitrary untrusted code. Only dispatch code the maintainer has reviewed.

`lat/reviewed-ci` is a distinct commit status against the exact SHA, with a run-attempt URL. Pending is published after authorization; final success requires the complete reusable job to succeed and the PR still to match. Failure maps to failure; skipped/cancelled/other conclusions and a moved/closed PR map to error. A reporter API failure cannot publish success. GitHub does not surface ordinary branch `workflow_dispatch` jobs as native PR checks; see [primary-source research](research/PR_CI.md).

Workflow concurrency serializes runs per PR with `cancel-in-progress: false`. An older run finishes reporting before a newer run starts publishing; exact-SHA statuses never transfer to a new head. GitHub may replace a waiting concurrency run, which has not yet authorized or published pending. Reusable CI uses a run-specific concurrency group for reviewed calls so main/ordinary branch cancellation cannot cancel them. Avoid force-cancelling the controller: its `always()` reporter attempts to publish after validation failure/cancellation, but runner loss, force cancellation, checkout failure or GitHub API failure can leave a pending status. A pending/missing status is not a pass. Recover by inspecting the linked run and dispatching the still-reviewed current head again. No branch-protection configuration is changed here.

Controller tests exercise API data and status-state behavior on lat. Full CI and the live stale-head/visible-status exercises require workflow runs; the first deployed controller must be verified on main before claiming end-to-end PR visibility. The dedicated runner still serializes jobs; no extra runner or hosted minutes are added.

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

## Artifact availability

CI waits for its uploaded protocol fixture to appear in the run's REST index, then downloads it and compares the bytes with the source fixture. Index visibility does not guarantee download availability: run `44960323851` passed the builds and regressions but received a download-endpoint 404 afterward.

The official download action now gets at most three attempts, each limited to two minutes, with a three-second delay after a failed attempt. Retries inspect the step's `outcome`, which preserves failure before `continue-on-error` changes its conclusion. A successful attempt skips the remaining attempts; the third attempt is fatal, and the byte-for-byte round-trip check remains required. This only tolerates brief service failures; missing artifacts, permission failures and corrupt data still fail CI. It adds no runner, token permissions or third-party action. See [GitHub's step outcome semantics](https://docs.github.com/en/actions/reference/workflows-and-actions/contexts#steps-context).

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

## Reviewed-PR acceptance, 2026-11-21

The first live negative exercise deliberately dispatched PR #147 with an obsolete reviewed SHA. [Run 44974417467](https://github.com/jctoledo/blackbox/actions/runs/44974417467) failed authorization with the expected head mismatch error; validation and reporting were skipped, so no PR source ran. This intentional failure checks the trust contract.

The positive exercise pinned `9ee9e212aa9212110cc8c6cfd453e28cabd73c7d`. [Run 44975026545](https://github.com/jctoledo/blackbox/actions/runs/44975026545) published pending, ran the full existing CI and published success. Both the commit-status API and `gh pr checks 147` exposed `lat/reviewed-ci` with that run's attempt URL. GitHub's job API reported `blackbox-lat` for authorization, validation and reporting. The resulting main merge `6861161` has the identical tested source tree. This verifies one real successful dispatch and one stale-head refusal; cancellation and runner-loss reporting remain best effort as described above.
