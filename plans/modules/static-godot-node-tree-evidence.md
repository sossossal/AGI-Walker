# Module Goal

Provide one reusable static Godot node-tree evidence command that emits all static artifacts needed for CI and release triage: combined report, delivery gate, manifest sidecars, validation summary and compact closeout report.

# Ownership

- `tools/build_static_godot_node_tree_evidence.py`
- `tools/build_dynamic_robot_generation_report.py`
- `tools/validate_delivery_acceptance_gate.py`
- `.github/workflows/ci.yml`
- `tests/test_dynamic_godot_robot_generation.py`
- `docs/guides/DYNAMIC_GODOT_ROBOT_GENERATION.md`
- `docs/guides/DYNAMIC_GODOT_ROBOT_GENERATION_FUTURE_PLAN.md`

# Inputs and Outputs

Inputs:

- Robot JSON fixtures or user-provided robot config paths.
- Existing static report builder and delivery gate validator.

Outputs:

- `report.json`
- `gate.json`
- `node_tree_manifests/*.node_tree_manifest.json`
- `validation_summary.json`
- `static_godot_node_tree_evidence_closeout.json`

# Contract Checklist

- Public surface this module exposes: `tools/build_static_godot_node_tree_evidence.py`.
- Inputs this module accepts: one or more robot JSON configs, output root, optional closeout path.
- Outputs this module produces: machine-readable closeout report and referenced artifacts.
- Shared types/schemas/config touched: delivery gate summary counts and sidecar validation summary.
- Backward compatibility requirements: existing report builder and gate validator behavior must remain unchanged.
- Integration tests required: CLI wrapper creates all artifacts and CI workflow references the wrapper.

# Local Context

The static manifest gate already exists as two commands: report generation with `--static-node-tree-manifest-dir`, followed by strict sidecar validation. This module should wrap those existing commands, not duplicate manifest or gate validation logic.

# Non-Goals

- No live Godot execution in this module.
- No new robot schema format.
- No replacement of existing report/gate tools.

# Tasks

- [x] Add wrapper CLI for golden static evidence generation.
- [x] Write closeout JSON with artifact paths, status, sidecar counts and residual risk.
- [x] Update CI to use the wrapper.
- [x] Update docs with the single command.
- [x] Add tests for wrapper output and workflow reference.
- [x] Add schema/manifest compatibility matrix.
- [x] Add reusable negative fixtures for duplicate part ids, duplicate connection names and root drift.

# Risks and Mitigations

- Risk: Wrapper hides underlying command failures.
  Mitigation: Store report/gate/validator return codes and stderr/stdout previews in closeout JSON.

- Risk: Closeout report overclaims readiness.
  Mitigation: Include explicit `live_godot_smoke_run=false` and `acceptance_level=static_only`.

# Validation

```powershell
py -3.12 -m py_compile tools\build_static_godot_node_tree_evidence.py
py -3.12 -m pytest tests\test_dynamic_godot_robot_generation.py::test_static_godot_node_tree_evidence_tool_writes_closeout -q
```

# Completion Criteria

- Wrapper command exits 0 for golden fixtures.
- Closeout report status is `success` and points to existing artifacts.
- Sidecar validation summary has zero invalid sidecars, zero validation errors and zero path mismatches.
- CI workflow invokes the wrapper.

# Notes

- Keep implementation as a thin orchestration layer over existing tools.
- 2026-05-18: Added `tools/build_static_godot_node_tree_evidence.py`; validation still delegates to existing report builder and gate validator.
- 2026-05-18: Added wrapper closeout test and CI workflow reference test.
- 2026-05-18: Added explicit dynamic Godot compatibility matrix and reusable negative fixtures for duplicate part ids, duplicate connection names and manifest root drift.

# Drift Check

Before and after implementation, verify this module still matches `PROJECT_PLAN.md` cross-module contracts and does not move runtime/live Godot work into the static evidence module.
