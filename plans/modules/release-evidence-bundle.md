# Module Goal

生成面向交付的 evidence bundle，包含 static closeout、delivery gate、readiness summary、live smoke、Web delivery record 和文档索引，并提供验收包自校验工具。

# Ownership

- Release/readiness tooling under `tools/`
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
- Documentation paths for operator/developer handoff.

Outputs:

- Evidence bundle directory or archive.
- Bundle index JSON with artifact type, path, checksum or size metadata, evidence level and residual risk.
- Bundle self-validation report.

# Contract Checklist

- Public surface this module exposes: `tools/build_dynamic_godot_release_evidence_bundle.py`, `tools/validate_dynamic_godot_release_evidence_bundle.py`, bundle index schema, self-validation output.
- Inputs this module accepts: evidence artifact paths and docs index inputs.
- Outputs this module produces: bundle index, copied artifacts, validation report.
- Shared types/schemas/config touched: release/readiness summary, delivery gate, static closeout.
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

# Risks and Mitigations

- Risk: Bundle validation duplicates existing gate logic.
  Mitigation: Validate artifact presence and consistency, then delegate artifact-specific validation to existing contracts where possible.

- Risk: Bundle archives stale artifacts.
  Mitigation: Record artifact paths, sizes/checksums and source timestamps in the index.

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

# Drift Check

Before implementation, verify bundle fields preserve existing release/readiness semantics and do not redefine acceptance levels.
