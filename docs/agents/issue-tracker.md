# Issue tracker: GitHub

Specs and tickets live in **jctoledo/blackbox** GitHub Issues. Use `gh` with an explicit `--repo jctoledo/blackbox`; multiline bodies use a file and `--body-file`. Follow `docs/process/workflow.md` when generic skill instructions differ.

- Read: `gh issue view <number> --repo jctoledo/blackbox --json number,title,body,labels,comments`.
- Create only in this repository. Check existing issues before duplicating work.
- Keep a multi-part spec as a parent with sub-issues and native dependency links where supported. Otherwise record `Part of #n` and `Blocked by #n` explicitly. A dependency is ready only after the required implementation is merged into main.
- Close with a comment explaining the verified outcome. Leave physical validation and R&D evidence visible when software alone cannot complete them.
- PRs are reviewed code changes, not an automatic external request/dispatch surface. A label or third-party comment does not authorize running unreviewed code on lat.

The maintainer or an explicitly authorized coordinating agent merges. A worker never grants itself merge authority. This chat has maintainer authorization for reviewed fixes/releases; future sessions must use their own authorization context.
