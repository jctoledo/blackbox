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

## Phone acceptance

Keep the original browser and data intact. Use a disposable destination with free storage and follow the [Android/iPhone checklist](../HARDWARE_VALIDATION.md#drive-recording-and-backup-acceptance-on-both-phones). Actual download handling, multiple-file selection, browser quota, foreground cancellation and reload recovery must be observed on both phones. This software benchmark does not stand in for those checks.
