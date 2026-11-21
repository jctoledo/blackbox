# Blackbox workflow: from evidence to merged changes

Repository instructions and the maintainer's current authorization govern generic skills. `CLAUDE.md` is the engineering entry point. This page adapts the [skill-adoption approach](../agents/evals/code-review-2026-11-21.md) to Blackbox; it does not grant permissions on its own.

## Scope and authority

The maintainer merges, or explicitly authorizes a coordinating agent to merge reviewed work. Workers produce changes and evidence; they never grant themselves merge authority. This chat authorizes autonomous reviewed fixes, releases and issue updates. Future sessions use their own authorization context.

All builds, automated tests, generated docs/demo, font embedding and media encoding run on **lat**. Use only trusted main/tags or maintainer-reviewed dispatches on the persistent runner. Models inherit the active chat's choice unless the maintainer requests an override; respect actual tool capacity and resource limits. Extra runners require isolated storage or a reviewed concurrency design.

## One PR or several

A focused change gets a `codex/` branch, concrete acceptance, relevant checks and review. For a bug, reproduce the failure and see a regression fail before fixing it where feasible. For reversible copy/layout work, inspect the rendered result rather than inventing implementation-mirroring tests.

For multiple PRs, write one parent spec before assigning implementation. Fix its Testing Decisions: public interface/schema, units and frames, clock sources, success/failure semantics, compatibility and test seams. Draft tickets with acceptance and blocking edges. Existing maintainer authorization can cover routine publication; a genuinely unresolved product or account choice stays explicit instead of being guessed into a worker's brief.

Each worker owns a worktree and branch from current main, with one PR to main. A dependent worker starts after the required implementation is **merged**, not merely approved or green on another branch. An integration branch may validate a combined tree; it does not merge itself or confer authority. Independent tasks can proceed in parallel within capacity.

## Questions during unattended work

Use the supplied brief, current behavior and source to resolve routine choices, then record the reason. A missing fact that affects correctness, account access or hardware configuration becomes an explicit blocker/`needs-info` or `ready-for-human` item. Continue independent work. Do not stall an otherwise specified ticket for a generic skill interview or ask again for authorization already granted in the session.

## Review and checks

Nontrivial behavior changes receive a fresh independent review of a pinned diff, requirements and repository standards. Report two axes separately: **Spec** (missing, unasked or incorrect behavior, citing the requirement) and **Standards** (written rule and code evidence). Label code smells as judgments. Follow a found cause through other instances/callers, and distinguish source inspection from executed evidence.

The stock code-review skill can provide independent axis agents when useful. The [single-fixture trial](../agents/evals/code-review-2026-11-21.md) found no defect-recall improvement over the existing baseline, so more agents are not mandatory for every change. Keep the useful traceability without treating a skill name as a quality guarantee.

Run relevant checks on lat before merging. Full firmware changes use both receiver variants, host/browser regressions, formatting/Clippy and protocol/artifact checks. Documentation changes build and validate the site. A tested integration tree can cover nonconflicting individual PRs; record exact heads/tree equivalence. Final main CI should remain green. Hardware claims still require physical acceptance.

## Records and release

PRs explain the problem, resulting behavior, checks and remaining limits. Update the changelog for user-visible changes; keep maintainer findings in `docs/REVIEW.md` and durable build lessons in the relevant infrastructure guide. Research cites primary sources. Nothing important should live only in an agent's memory.

A release is a tagged, tested source tree with correct receiver images, checksums and application metadata. Verify publication and the deployed installer; preserve a usable prior release while staging. Test-only/process changes do not require a new firmware release.

Physical/R&D issues stay open until their evidence exists. The glossary distinguishes configured rates, delivered rates and display rates, and raw recordings from lap history. Do not turn synthetic regression results into field accuracy, detection F1 or commercial-parity claims.

## Skill use

`research` follows questions to primary sources; `codebase-design` keeps complexity behind an appropriate interface; `writing-for-agents` keeps house guidance concise. `code-review` supports the review format above. Other installed skills remain optional; evaluating one does not validate all of them. The interactive setup skill is available but its interview was not run; these repository pointers were authored from known project rules and the maintainer-provided adoption guide.
