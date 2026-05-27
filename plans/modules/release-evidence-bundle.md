# Module Goal

生成面向交付的 evidence bundle，包含 static closeout、delivery gate、readiness summary、live smoke、Web delivery record 和文档索引，并提供验收包自校验工具。

# Ownership

- Release/readiness tooling under `tools/`
- Canonical release evidence collection in `tools/collect_release_evidence.py`
- Static evidence closeout artifacts
- Delivery gate and Web delivery records
- Documentation index generation
- Bundle validation tests

# Inputs and Outputs

Inputs:

- Static closeout JSON.
- Delivery acceptance gate JSON.
- Release/readiness summary JSON.
- Optional live smoke JSON.
- Optional Web delivery record JSON.
- Optional Phase 3 control/communication simulation closeout JSON.
- Documentation paths for operator/developer handoff.

Outputs:

- Evidence bundle directory or archive.
- Bundle index JSON with artifact type, path, checksum or size metadata, evidence level and residual risk.
- Bundle self-validation report.

# Contract Checklist

- Public surface this module exposes: `tools/build_dynamic_godot_release_evidence_bundle.py`, `tools/validate_dynamic_godot_release_evidence_bundle.py`, bundle index schema, self-validation output.
- Optional collector surface this module exposes: `tools/collect_release_evidence.py --control-comm-closeout-source`.
- Inputs this module accepts: evidence artifact paths and docs index inputs.
- Outputs this module produces: bundle index, copied artifacts, validation report.
- Shared types/schemas/config touched: release/readiness summary, delivery gate, static closeout.
- Optional shared artifact touched: `control_comm_simulation_closeout.v1` as `control_comm_closeout`.
- Backward compatibility requirements: existing evidence tools keep their output formats; bundle consumes them without rewriting.
- Integration tests required: ready bundle, missing artifact, invalid artifact and optional live/Web evidence cases.

# Local Context

Static closeout and readiness summary already exist. This module packages those artifacts for delivery and prevents handoff from depending on scattered local files.

# Non-Goals

- Do not invent a new acceptance level.
- Do not rerun Godot or regenerate evidence during bundle validation unless explicitly requested.
- Do not require live/Web evidence for a static-only bundle.

# Tasks

- [x] Define evidence bundle index schema.
- [x] Add bundle generation CLI that copies/references required artifacts.
- [x] Add self-validation CLI for required artifact presence and schema consistency.
- [x] Include docs index entries for static, live, Web and readiness workflows.
- [x] Support static-only bundles and stronger bundles with live/Web evidence.
- [x] Add tests for ready, incomplete and malformed bundles.
- [x] Document release handoff workflow.
- [x] Support optional Phase 3 control/communication closeout evidence without making it required.
- [x] Let canonical release evidence collection copy an existing Phase 3 control/communication closeout.
- [x] Document and test the canonical collector output path as a release bundle `--control-comm-closeout` input.
- [x] Include the Phase 3 control/communication guide as optional bundle documentation when `control_comm_closeout` is bundled.

# Risks and Mitigations

- Risk: Bundle self-validation report drifts from the current bundle index.
  Mitigation: Require the final index to reference an in-bundle validation report and verify its status, error self-consistency, bundle root, required contract lists, counts and evidence level against the current validation result.

- Risk: Bundle validation duplicates existing gate logic.
  Mitigation: Validate artifact presence and consistency, then delegate delivery gate shape checks to the shared `delivery_acceptance_gate.v1` contract.

- Risk: Bundle archives stale artifacts.
  Mitigation: Record artifact paths, sizes/checksums, source timestamps and bundled-file timestamps in the index; validate index bundle root, generated timestamp, readiness status and residual risks against the current bundle.

- Risk: Bundle index marks required evidence as optional or optional handoff notes as required.
  Mitigation: Validate each artifact and documentation `required` flag against the canonical required key/role lists.

# Validation

```powershell
py -3.12 -m pytest tests\test_dynamic_godot_robot_generation.py -q
py -3.12 -m pytest tests\test_workflow_contracts.py -q
py -3.12 -m pytest -m "not live" --collect-only -q
```

# Completion Criteria

- A delivery bundle can be generated from existing evidence artifacts.
- The bundle self-validator reports ready, incomplete or invalid with actionable errors.
- Static-only, Godot-load and full Godot-verified evidence levels are represented without ambiguity.

# Notes

- Start with directory output before adding archive packaging if that keeps validation simpler.
- Implemented directory output only. Archive packaging remains out of scope until a delivery target requires it.
- Validation evidence: `py -3.12 -m py_compile tools\build_dynamic_godot_release_evidence_bundle.py tools\validate_dynamic_godot_release_evidence_bundle.py`; `py -3.12 -m pytest tests\test_dynamic_godot_robot_generation.py -q -k "release_evidence_bundle"`; `py -3.12 -m pytest tests\test_dynamic_godot_robot_generation.py -q`; `py -3.12 -m pytest tests\test_workflow_contracts.py -q`; `py -3.12 -m pytest -m "not live" --collect-only -q`.
- 2026-05-20: Optional bundled live smoke artifacts are now validated for `dynamic_godot_live_verification_profile.v1` metadata and wrapper retry field consistency when present.
- 2026-05-20: Optional bundled Web delivery records are now validated for `web_godot_delivery/godot_load` gate metadata and static node-tree manifest evidence consistency when present.
- 2026-05-20: Bundle validation now rejects absolute or escaping `bundle_path` values so artifacts and docs remain self-contained under the bundle root.
- 2026-05-20: Bundle validation now rejects duplicate artifact `key` values and documentation `role` values to avoid ambiguous or overwritten evidence entries.
- 2026-05-20: Bundle entries now record timezone-aware `source_modified_at` and `bundle_modified_at`; validation checks timestamp shape and bundled file mtime consistency.
- 2026-05-20: Bundle validation now delegates bundled delivery gates and Web delivery record gates to the shared `delivery_acceptance_gate.v1` contract.
- 2026-05-20: Bundle validation now checks the final index `validation_report` path and ensures the stored report status plus `validation_status` match the current validation result.
- 2026-05-20: Bundle validation now checks the stored validation report snapshot for status/error self-consistency, artifact count, documentation count and evidence level drift.
- 2026-05-20: Bundle validation now checks the stored validation report snapshot for bundle root and required artifact/documentation role contract-list drift.
- 2026-05-20: Bundle validation now checks index metadata drift for `bundle_root`, timezone-aware `generated_at`, `readiness_status` and `residual_risks`.
- 2026-05-20: Bundle validation now checks artifact and documentation `required` flags against the canonical required key/role lists.
- 2026-05-24: PR Ubuntu 3.10 CI exposed that `datetime.UTC` is not available before Python 3.11. Bundle builder and validator now use `timezone.utc` to preserve the documented Python 3.10 compatibility matrix.
- 2026-05-26: Bundle builder now accepts optional `--control-comm-closeout`; validator checks provided `control_comm_closeout` artifacts for healthy non-live closeout semantics while preserving existing required artifact keys.
- 2026-05-26: Canonical release evidence collector now accepts `--control-comm-closeout-source` and copies existing closeout evidence under `control_communication/` without generating or upgrading live hardware claims.
- 2026-05-26: Release handoff docs now show the direct collector output path `test_env/release_evidence/control_communication/control_comm_simulation_closeout.json` as the bundle `--control-comm-closeout` input, guarded by active-path tests.
- 2026-05-26: Bundles that include `control_comm_closeout` now automatically include optional `control_comm_workflow` documentation from `docs/guides/CONTROL_COMMUNICATION_SIMULATION.md` unless the caller supplies an explicit `--doc` set.
- 2026-05-26: Added regression coverage that explicit `--doc` entries remain caller-controlled when `control_comm_closeout` is present, preserving compatibility for curated handoff bundles.

# Drift Check

Before implementation, verify bundle fields preserve existing release/readiness semantics and do not redefine acceptance levels.
