# Reviewed PR validation on lat

Research date: 2026-11-21. Read-only investigation; no builds, tests, runner changes, workflow dispatches or repository mutations performed.

## Current repository facts

`CLAUDE.md`, `docs/BUILD_INFRASTRUCTURE.md`, and `docs/process/workflow.md` require all builds/tests on lat and restrict its persistent runner to trusted main/tags or maintainer-reviewed dispatches. `.github/workflows/ci.yml` has only `push` to `main` and `workflow_dispatch`; its single `validate` job uses `blackbox-lat`, four Cargo jobs, and per-ref cancellation. Missing automatic same-repository PR CI is therefore explained by the trigger configuration, not evidence of a missing runner. One existing runner serializes jobs; increasing runner count is unnecessary for this trigger problem.

## Important GitHub behavior

- **A branch dispatch is not a native PR check.** GitHub explicitly documents that `workflow_dispatch` jobs on a PR head branch do not appear in the PR checks section or satisfy branch-ruleset required checks. Latest-commit SHA matters; stale checks cannot cover newer code. Skipped jobs can count as success, so a skipped trust guard is not evidence that validation ran. [GitHub: Troubleshooting required status checks](https://docs.github.com/en/pull-requests/how-tos/merge-and-close-pull-requests/troubleshooting-required-status-checks#checks-from-some-workflow-jobs-are-not-evaluated)
- Dispatch is available through UI/CLI/API and requires repository write access. `gh workflow run ci.yml --ref BRANCH` selects another branch; the workflow must exist on the default branch. [GitHub: Manually running a workflow](https://docs.github.com/en/actions/how-tos/manage-workflow-runs/manually-run-a-workflow)
- `workflow_dispatch` uses the selected branch/tag's tip SHA. Review events use the PR merge ref. `pull_request_target` uses the default/base context and its default-branch SHA; GitHub warns against executing untrusted PR code through that event. Review submission can trigger jobs conditioned on approved state. [GitHub: Workflow events](https://docs.github.com/en/actions/reference/workflows-and-actions/events-that-trigger-workflows)
- Pull request authors cannot approve their own PRs. Review approval alone therefore does not handle a sole maintainer's own PRs. [GitHub: Approving PRs](https://docs.github.com/en/pull-requests/how-tos/review-pull-requests/approving-a-pull-request-with-required-reviews)
- Public-repository reviews are not necessarily maintainer reviews: by default any user can submit an approving review. An approval trigger needs explicit reviewer authorization, not merely `review.state == approved`. [GitHub: Managing pull request reviews](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/managing-repository-settings/managing-pull-request-reviews-in-your-repository)
- Persistent self-hosted runners can retain compromise from untrusted code. Fork workflow approval settings address compute authorization and are not a security boundary for such runners. [GitHub: Secure use](https://docs.github.com/en/actions/reference/security/secure-use#hardening-for-self-hosted-runners), [GitHub: Workflow approval policy warning](https://docs.github.com/en/organizations/managing-organization-settings/disabling-or-limiting-github-actions-for-your-organization#limiting-the-use-of-self-hosted-runners)

## Recommendation (design inference)

**If manual reviewed validation is sufficient:** document the existing dispatch procedure, record exact reviewed SHA and resulting run URL, and verify run `head_sha` and current PR head before merging. This is the smallest change, but must not be advertised as fixing PR Checks visibility. A post-dispatch SHA comparison detects a race only after the selected tip may already have executed: it is not sufficient to enforce the trust rule when branches can advance concurrently.

**If visible PR merge-box validation is wanted:** retain maintainer initiation and add an exact-commit dispatch contract plus a commit-status result. Dispatch the trusted workflow on `main`, passing PR number and the full reviewed SHA. Before executing PR code, validate the PR/repository/head association using trusted code, then checkout only the supplied full SHA with credentials disabled. Do not resolve the mutable branch for checkout. Recheck current head to avoid presenting obsolete results as current. This avoids new automatic PR execution and handles the maintainer's own PRs.

Publish a distinct status such as `lat/reviewed-ci` against that exact SHA: pending before execution, then success only after all required validation succeeds, otherwise failure/error, with the Actions run URL as `target_url`. GitHub documents that commit statuses appear on PRs containing the commit. This is a visible PR status, **not** the richer Checks tab. The reporting helper or trusted reporting step needs status-write permission; existing workflow permissions are read-only. Keep reporting code trusted, use a distinct context to avoid check/status name collisions, and never infer success merely from dispatch acceptance. [GitHub: Commit statuses](https://docs.github.com/en/rest/commits/statuses), [GitHub: Status types](https://docs.github.com/en/pull-requests/reference/status-checks)

A trusted local coordinator can publish the status after watching the run without running builds locally; this keeps the tested code's job token read-only. Alternatively a trusted workflow reporting step can publish it, with deliberately scoped permissions. A dispatch helper alone, without pinning and explicit status publication, solves neither the race nor PR visibility.

## Why not the apparent simpler automatic gates?

- Same-repository origin establishes where a branch lives, not that its newest commit was reviewed. Automatically running every same-repository PR violates the stated stronger rule.
- `pull_request_review: submitted` cannot cover authors' own PRs, needs reviewer authorization and review-commit equality, and should not accidentally test a newer mutable head.
- A persistent `reviewed` label combined with `synchronize` silently approves later pushes. Label-only execution is better, but the approval must name the reviewed SHA and checkout must use that immutable SHA; merely reading the current head at label time leaves a review-to-label race.
- A trust `if:` in a normal PR workflow is not an independent trust boundary when PR authors can change that workflow. A default-branch controller can guard execution, but then its exact-SHA and reporting contract must be designed. `pull_request_target` plus PR checkout is sufficiently subtle that it is not the minimal default here.

## Acceptance and independent review seams

Review the trust contract separately from build-command reuse. Preserve current build commands, cache policy, runner label and resource limits.

1. Unreviewed fork open/update and unreviewed same-repository open/update schedule no PR code on lat.
2. Maintainer's own reviewed PR can be explicitly validated.
3. Wrong repository, closed PR, mismatched SHA, moved head and malformed SHA fail before executing PR code. A head movement during queueing never causes checkout of new code.
4. Checked-out `HEAD`, recorded reviewed SHA, and status target SHA match exactly; newer commits do not inherit success.
5. Pending, success, failure, timeout and cancellation report honestly. A failed/absent reporter leaves no false green; never overwrite another concurrent attempt's newer status with an older result.
6. Observe the intended PR merge-box status and run link on a reviewed harmless PR; separately confirm whether rich Checks-tab display is actually required before claiming completion.
7. Existing main CI still runs on lat. No GitHub-hosted runner or extra persistent runner is introduced.

These acceptance checks require controlled lat/GitHub validation later; this research does not claim they have run.

## Implemented design and operational constraints

The implementation uses a main-dispatched wrapper, a trusted Python API guard/status helper, and `workflow_call` inputs to reuse `ci.yml` with an immutable checkout. A same-repository reusable-workflow reference (`./.github/workflows/ci.yml`) resolves at the caller's commit; reusable workflows cannot elevate the caller's token permissions. [GitHub: Reusing workflows](https://docs.github.com/en/actions/how-tos/reuse-automations/reuse-workflows)

Per-PR workflow concurrency uses `cancel-in-progress: false`. GitHub permits one running and one pending member per group and can replace an existing pending member; ordering is not guaranteed. Here, a waiting member has not published pending or executed PR source, while the active member reports before another starts. The called CI gets a run-specific group to avoid the existing main/branch cancellation policy affecting reviewed runs. [GitHub: Control concurrency](https://docs.github.com/en/actions/how-tos/write-workflows/choose-when-workflows-run/control-workflow-concurrency)

An `always()` final job attempts reporting after failed/cancelled validation. This cannot guarantee reporting after runner loss, force cancellation or GitHub/API failure; unresolved pending remains unsuccessful evidence. [GitHub: Workflow cancellation](https://docs.github.com/en/actions/reference/workflow-cancellation-reference), [GitHub: Status check functions](https://docs.github.com/en/actions/reference/workflows-and-actions/expressions#status-check-functions)
