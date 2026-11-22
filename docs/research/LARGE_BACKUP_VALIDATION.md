# Large backup validation on lat

Issue: [#138](https://github.com/jctoledo/blackbox/issues/138). Acceptance: [implementation spec](../specs-138-large-backups.md).

The deterministic workload in `scripts/large_backup_fixture.cjs` uses production-shaped numeric/null telemetry rows and GPS evidence. It supplies 30 telemetry rows and 25 receiver rows per simulated second, saved in five-second chunks. Those are fixture inputs, not measured delivered rates. GPS coordinates, speeds, metadata and timing are synthetic; they establish no physical accuracy or run-detection performance.

The fixture's serialized chunk payload sizes on lat are:

| Duration | Rows | Chunks | JSON bytes | MiB |
| --- | ---: | ---: | ---: | ---: |
| 20 min | 66,000 | 240 | 34,953,986 | 33.335 |
| 60 min | 198,000 | 720 | 105,415,335 | 100.532 |
| 120 min | 396,000 | 1,440 | 211,226,790 | 201.442 |

These numbers include serialized chunk fields and their row arrays, before backup envelopes, JSON fragment escaping, manifests, filesystem overhead, metadata, browser storage indexes and temporary transaction space. The precise threshold depends on real row values and sampling rates. They should not be read as a storage-capacity promise.

## Reproduce

Run from a trusted checkout on lat with the locked Node dependencies installed:

```sh
node scripts/benchmark_large_backup.cjs 20
node scripts/benchmark_large_backup.cjs 60
node scripts/benchmark_large_backup.cjs 120
```

Run each case in a fresh process. The benchmark writes exported files to a temporary directory, reads them back through the production file interface in reversed selection order, stages them, opens a fresh JavaScript context, restores as new copies, and compares cursored SHA-256 hashes of every row and chunk index. It checks the retained original, Drive replay, session count and a second-context retry without duplicates. Temporary files are removed after the case. A checksum/hash comparison is content evidence, not a physical sensor-accuracy claim.

The output records duration per phase and sampled Node RSS/heap. `fake-indexeddb` retains source, staging and destination records in JavaScript memory. Thus process memory includes the fixture and multiple database copies; it does **not** isolate the exporter's working set, model native IndexedDB disk use, or establish a phone limit. Sampled peaks can miss brief allocations. The bounded-memory claim also requires source review: one current file, one bounded logical record, bounded root metadata and the captured tail, with no retained whole-recording row/output collection.

## Observed round trips

All three cases passed: source retained, exact cursor hashes equal after restore, Drive replay equal, and a fresh-context retry added no duplicates. The source module was `c0edfaa` for 20/60 minutes and `9099c1a` for 120 minutes; the latter also bounds each retained Drive anchor to six scalars. The later modal-dismissal guard changes no archive or calculation behavior. Node was 22.14.0 with fake-indexeddb 6.2.7 inside the dedicated 4-CPU/8-GiB lat container. These are single observations under ordinary host load, not a controlled performance guarantee.

| Fixture | Files including manifest | Largest file, bytes | Prepare + export, s | Read + validate, s | Restore after reload, s | Sampled peak Node RSS, bytes |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 20 min | 6 | 8,337,692 | 6.14 | 5.09 | 2.27 | 418,775,040 |
| 60 min | 16 | 8,337,691 | 17.39 | 14.62 | 6.30 | 905,179,136 |
| 120 min | 30 | 8,337,695 | 34.52 | 30.21 | 12.82 | 1,370,771,456 |

The largest file in every case is below 8,388,608 bytes (8 MiB). Both the 60-minute and 120-minute **single recordings** exceed the old 100 MiB limit. The 120-minute file set totals 234,753,157 bytes after JSON envelopes/escaping. The original and restored 396,000-row payloads share SHA-256 `b747cb3dbe991d2bf34eb40fe93183e4ca684f6024ec56c9822480ecb51b3d55` (chunk index + row content, excluding remapped session identity).

Raw phase/memory observations: [20 min](data/backup-20min-c0edfaa.json), [60 min](data/backup-60min-c0edfaa.json), [120 min](data/backup-120min-9099c1a.json). Their RSS growth reflects fake IndexedDB retaining multiple copies of the growing fixture in Node; it must not be presented as the browser exporter's required heap. Native IndexedDB and actual phones remain separate acceptance.

## Phone acceptance

Keep the original browser and data intact. Use a disposable destination with free storage and follow the [Android/iPhone checklist](../HARDWARE_VALIDATION.md#drive-recording-and-backup-acceptance-on-both-phones). Actual download handling, multiple-file selection, browser quota, foreground cancellation and reload recovery must be observed on both phones. This software benchmark does not stand in for those checks.
