## Large backup implementation spec — issue #138

### Problem Statement
A single long recording can exceed the version 1 backup's 100 MiB limit. Increasing that limit would still materialize the entire archive multiple times, potentially crashing a phone. Existing recordings, tracks and unsaved samples must survive every failure.

### Solution
Offer a bounded multipart backup on ordinary HTTP as the default export. The user explicitly downloads one numbered part at a time and downloads the final manifest last. Restore accepts the manifest and all its parts, checks the complete archive, and adds new copies. Version 1 backups and legacy track JSON remain importable.

### User Stories
1. Back up one recording or all saved data without a whole-recording row array or whole-archive Blob.
2. Keep an unsaved tail recoverable when browser storage is full; exporting must not require a storage write.
3. See preparation, part progress, cancellation and exactly which files must be kept together.
4. Know that a requested download is not proof that a file was saved.
5. Detect missing, mixed, duplicated, reordered or damaged part contents before destination writes. File selection order may vary; manifest order is authoritative.
6. Cancel export without changing source data; incomplete files have no valid final manifest.
7. Restore long recordings as one recording with metadata, Drive evidence, tracks, references and history preserved/remapped.
8. Preserve originals through validation, quota, write, cancellation, collision and partial restore failures.
9. Retry an interrupted restore without duplicating an already committed database group.
10. Continue using existing version 1 and legacy track backups.

### Implementation Decisions
- Version 2 multipart JSON, at most 8 MiB UTF-8 per file including envelope; bounded fragment/part construction. Manifest binds archive identity, order, exact bytes, CRC checksums and collection counts. CRC detects accidental corruption, not authenticity.
- Explicit documented archive/record/part limits guard hostile input. Fragment oversized logical records; no arbitrary split into independent recording sessions. Memory scales with a part, the largest bounded source record, bounded metadata and captured tail, never total sample count. Do not claim a fixed total heap or phone capacity.
- Two-pass export: fixed tail/metadata overlay and source fences; compare canonical record counts/digests across the passes. Release each part after advancing. Publish a manifest only after the complete second pass matches. Source mutation fails the archive rather than producing a falsely complete backup. No mandatory export staging writes.
- Stage and validate imports in a separate IndexedDB database. Keep existing database versions unchanged for compatibility with older firmware dashboards. No production writes until complete structural, link, checksum, count and incremental Drive evidence validation succeeds.
- Promote each destination database group using add-only transactions with bounded staged reads. A destination transaction must remain active through request callbacks; validate the keepalive/paging behavior and abort semantics explicitly. Never await files inside a destination transaction. If this cannot be made demonstrably reliable, record a reviewed architectural revision rather than silently weakening atomicity.
- Persist fresh-ID mapping before promotion. Mark imported root rows with restore-job identity in the same atomic destination transaction. Recover an uncertain commit by checking expected IDs/job markers; mixed/conflicting results stop safely. Never guess success, overwrite originals or silently resurrect externally removed copies. Track and recording databases remain separate commits, with explicit partial completion and retry.
- Persistent validated staging can resume after reload; incomplete staging needs reselected files or explicit discard. Cancellation aborts the active destination group, preserves a previously committed group, and never deletes original or restored production data.
- Stream Drive replay and post-restore summaries from cursors. Preserve exact initial GPS fence, observed-row evidence, duration and rejection semantics.
- UI stays stopped/unarmed after restore, exposes progress/cancel/retry/discard of staging only, and explains temporary quota use. No automatic recording/resumption. Existing backup idle/tail/metadata race guarantees continue.

### Testing Decisions
Test through the production backup module and fake IndexedDB, using existing backup/Drive regression seams. Verify equivalent small archives and a single-session >100 MiB round trip on lat; adversarial missing/corrupt/mixed/duplicate/reordered fragments, limits, cancellation in every phase, source mutation, quota/transaction/collision failures, retries and uncertain commits. Assert original contents and unsaved tail remain identical. Include real File-like byte reads and exact UTF-8 bounds, not only mocked object passing. Produce 20/60/120 minute synthetic payload/time/process-memory observations on lat; process memory is not phone capacity. Run full relevant CI on lat and independent Spec/Standards review before merge. Android/iPhone download, picker, quota and reload acceptance remains a clear human checklist, not inferred from Node tests.

### Out of Scope
Cloud accounts/providers, background/device-side recording, authentication/encryption, physical sensor claims, standing-start runs and unrelated UI changes.

### Further Notes
The CSV bounded source scan already merged in rc.13. This work completes the restorable-backup portion. No original deletion follows download or restore. References: W3C File API and IndexedDB specifications; the ordinary HTTP path must not depend on File System Access availability.
