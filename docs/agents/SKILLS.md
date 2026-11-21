# Maintainer agent skills

Installed from [mattpocock/skills](https://github.com/mattpocock/skills) at commit `abeeaf95660ba9e8b7c753eb38f441ca707a8823` in the maintainer's `~/.codex/skills`. These are user-level installations, not vendored runtime dependencies; another contributor must install them separately. The upstream instructions and their supporting files remain in the installed directories.

| Skill | Use in Blackbox |
| --- | --- |
| research | Follow a technical question to primary sources and save cited findings under `docs/research/` |
| code-review | Review a pinned diff against repository standards and the originating requirements separately |
| codebase-design | Evaluate module interfaces and isolate complexity when restructuring code |
| writing-for-agents | Keep engineering instructions concise and link to task-specific references |
| setup-matt-pocock-skills | Available for an interactive future setup of tracker/domain conventions; its setup interview has not been run |

For this discovery pass, research produced `docs/research/DISCOVERY.md`; writing-for-agents informed the maintainer pointers in `CLAUDE.md`; code-review is applied before merge. Codebase-design informed the shared cache helper: one small interface owns cache selection, while firmware outputs retain the existing build/version behavior. Installation does not certify the project or replace the lat checks and hardware acceptance plan.

GitHub Issues in `jctoledo/blackbox` are the existing tracker. Session authorization and repository instructions continue to govern builds, external posts, merges and claims. Launch drafts are reviewable source material; publishing them to communities is a separate maintainer action.
