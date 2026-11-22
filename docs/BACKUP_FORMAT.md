# Multipart backup format, version 2

This document describes the implementation added for [#138](https://github.com/jctoledo/blackbox/issues/138). The [user guide](BACKUP.md) explains downloads and recovery. This version is distinct from legacy **track export version 2**. Version 1 full-backup imports remain supported.

## Files and checksums

Each file is UTF-8 JSON. The producer emits numbered parts followed by a manifest. Every envelope has `format: "blackbox-backup"`, `version: 2`, `kind` (`part` or `manifest`), `archiveId` and `checksum`. The envelope checksum uses the existing `crc32-utf16le:` algorithm: JSON-stringify the envelope without its `checksum` field, then feed the low and high bytes of each UTF-16 code unit to reflected CRC-32 (polynomial `0xedb88320`, initial/final XOR `0xffffffff`). JSON property order therefore matters for that checksum. CRC detects accidental damage, not authenticity.

A part also has a zero-based `index` and a nonempty `fragments` array. The manifest has `createdAt`, ordered `parts`, `counts`, `samples`, `records`, `sourceDigest` and informational `limits`. Each part descriptor binds its index, suggested filename, exact byte length, semantic envelope checksum and **`fileChecksum` over the complete emitted UTF-8 bytes**, including the checksum field. The byte checksum is `crc32-utf8:` with the same CRC-32 polynomial/initial/final settings. Import uses raw bytes and fatal UTF-8 decoding. Even an equal-length whitespace edit to a part must fail its file checksum. Renaming a file does not alter its archive identity; actual part identity comes from its checked contents and manifest index.

No downloaded part becomes a complete backup by itself. The manifest is offered only after the source's second pass agrees with the first pass. Missing, extra, duplicate, mixed or damaged parts cannot produce a ready restore. Selection order is irrelevant; the manifest determines processing order. The manifest checksum checks its normalized envelope; the exact-byte descriptors bind the part files.

## Records and fragments

Collections occur in this order: `sessions`, `tracks`, `chunks`, `references`, `history`. Each logical record is one JSON-serialized portable row from the corresponding database store. A fragment has:

- `record`: one-based logical record ordinal across the archive;
- `collection`: one of the collection names above;
- `fragment`: zero-based sequence within that record;
- `last`: whether the record ends here;
- `text`: a consecutive substring of the logical record's JSON text.

Fragments from one record remain consecutive, even across files. The importer requires a record to start at fragment zero, consumes each ordinal exactly once in order, joins the bounded record, parses it and validates it. A recording's chunk rows retain the original session ownership and chunk indices. Parts do not become separate recordings. References use the existing numeric float32 distance/time encoding; restore reconstructs their ArrayBuffers.

`counts` names all five collections. `samples` counts all chunk rows, including receiver evidence. `records` is the sum of the collection counts. `sourceDigest` is an incremental UTF-16LE CRC over, for every record in export order, `collection + "\n" + recordJsonText + "\n"`. It must match both source passes and the reconstructed import, with matching counts. The importer additionally validates IDs, parent links, sample values, geometry, reference order and incremental Drive replay. File integrity alone does not make invalid data acceptable.

## Bounds and source consistency

A file is at most 8 MiB; producer manifests are at most 1 MiB. A logical record is at most 16 MiB serialized UTF-8. Text fragments are at most 262,144 UTF-16 code units, and exact escaped envelope bytes are counted when packing a part. At most 4,096 parts, 1,000,000 records and 10,000 sessions/tracks each are supported. These limits are guards, not claims about phone capacity.

The source scan captures the pending tail before database awaits, freezes recording metadata overlays after pending lookups, and records per-store high-water keys. Each pass cursor-reads bounded records through those fences. A matching committed chunk key deduplicates the captured tail. Neither pass consumes the buffer or requires staging writes, so quota-full export remains possible. Existing-row changes between passes prevent the final manifest; appends beyond the captured fences are outside that archive. Across the two source databases this is a verified export set, not a cross-database atomic snapshot.

The working set includes a current part, the largest logical record, bounded IDs/Drive state, metadata and the captured tail. IndexedDB itself clones returned records. Neither a whole archive nor a whole recording's row array is retained. Legacy single-file import retains its old full-file behavior and 100 MiB limit.

## Import staging and destination transactions

A separate staging database stores checked records and a job ledger. Job creation atomically enforces a 100-job limit; the interface never hides excess jobs by evicting them. The existing recording and track database versions remain unchanged. Incomplete staging cannot be promoted. Fully validated staging contains per-collection digests; promotion checks counts and digests again so deleted or changed staging aborts the active group.

Fresh ID mappings are persisted before destination writes. Promotion processes children first and adds marked roots last, using `add`, never `put`, in one transaction per destination database. Bounded staged reads publish the next row; only destination request callbacks queue destination writes. A pending destination request keeps that transaction alive until EOF and all validation checks succeed. This follows the [IndexedDB transaction lifecycle](https://w3c.github.io/IndexedDB/#transaction-lifecycle); it does not guarantee background execution on a suspended phone.

Each group has a transactional claim and attempt identity. State transitions reread the current ledger, and an aborted competing attempt cannot downgrade committed evidence. Root markers bind job/archive/group/root-count identity. If every expected root matches, recovery skips that committed group. Partial/conflicting markers stop. If a previous attempt lost its terminal outcome and no roots remain, recovery stops as uncertain: rollback and commit-followed-by-external-deletion cannot be distinguished safely. Known aborts can retry the same mapping. Records and tracks are two separate commits, so partial completion and post-commit bookkeeping failures remain explicit.

Discard removes only the selected staging job and its temporary records. It never deletes original data or committed destination copies. Refer to the production regressions and [lat workload results](research/LARGE_BACKUP_VALIDATION.md) for verified behavior and remaining phone acceptance.
