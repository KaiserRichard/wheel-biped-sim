# P3 analysis provenance

Imported on 17 November 2026 as a file snapshot, without private Git history. The source package is identified as `p3_logsysid`; it is a first-party analysis candidate prepared with Claude and Codex assistance. The owner's authorship/licence confirmation remains a release-review item; no new third-party licence grant is inferred here. Existing human/upstream notices and the repository's MIT text are retained. No copied upstream firmware implementation is included in this slice.

`analyses/logsysid/SOURCE_MANIFEST.json` records original package SHA-256 values, separately calculated released-file hashes, and opaque original-reference digests. The original manifest and unbundled references are withheld. Hashes establish lineage only; they do not make withheld files reproducible or clear their rights. The supplied simulation CSV is unchanged; its units, column order and unknown historical simulation seed are documented. Rehearsal random seeds remain explicit in `rehearsal.py`.

Changes remove personal paths, account routing and named sensor references; keep generic IMU sensor wording; stream verification logs instead of bundling operational output; and regenerate the synthetic report and receipt from the released code. The table in the batch proposal omits `wbrlog/studio.py`, but the audit's machine manifest includes it and the importer/tests require it. It is included with its original digest.

All published recovery results are synthetic. No real-robot log was supplied or fitted. Hardware acceptance remains OPEN. No MATLAB execution or P2/CAD input import is claimed. No new media are included; existing public media retain their unresolved rights/review limits.
