# Goal

Add a compact release/readiness summary for dynamic Godot robot generation that states which evidence level is currently proven: `static_only`, `godot_load_verified`, `godot_verified`, or `incomplete`.

# Scope

In scope:

- Read existing JSON evidence artifacts without rerunning Godot.
- Recognize static closeout reports and delivery acceptance gates embedded in reports, Web delivery records, or compact gate artifacts.
- Pick the strongest proven level and record residual risk for missing stronger evidence.
- Add CLI tests and documentation.

Out of scope:

- Running live Godot smoke.
- Changing the existing delivery gate contract.
- Replacing project-wide release readiness tooling.

# Interfaces and Contracts

- CLI: `tools/build_dynamic_godot_release_readiness.py`.
- Release-check version source: root `VERSION`, which must match `pyproject.toml` `[project].version`.
- Output summary version: `dynamic_godot_release_readiness_summary.v1`.
- Output artifact type: `dynamic_godot_release_readiness_summary`.
- Recognized evidence:
  - static closeout with `acceptance_level=static_only` and `status=success`
  - any `delivery_acceptance_gate.v1` with `complete=true` and `passed=true`
  - Web delivery record containing `delivery_acceptance_gate`
- Level precedence:
  - `godot_verified`
  - `godot_load_verified`
  - `static_only`
  - `incomplete`

# Implementation Tasks

- [x] Add release/readiness summary CLI.
- [x] Add tests for static-only, Web load verified, full Godot verified, and blocked evidence.
- [x] Define golden static fixture set as fixed pair, biped, and quadruped.
- [x] Add manual live smoke checklist with exact artifact paths and expected summary fields.
- [x] Document acceptance levels and proof commands for static-only, Godot-load verified, and Godot verified evidence.
- [x] Update project plan and documentation.
- [x] Run targeted non-live validation.

# Validation

```powershell
py -3.12 -m py_compile tools\build_dynamic_godot_release_readiness.py
py -3.12 -m pytest tests\test_dynamic_godot_robot_generation.py -q
py -3.12 -m pytest -m "not live" --collect-only -q
```

# Progress Notes

- 2026-05-18: Created module plan because `PROJECT_PLAN.md` next step is release/readiness summary.
- 2026-05-18: Added `tools/build_dynamic_godot_release_readiness.py` to summarize existing closeout/gate/Web delivery evidence into the strongest proven level.
- 2026-05-18: Validated with py_compile, dynamic Godot generation tests, workflow contract tests, and non-live collect-only.
- 2026-05-19: Updated static evidence defaults and CI command so release/readiness static coverage includes fixed pair, biped, and quadruped.
- 2026-05-19: Documented manual live smoke artifact paths and required `godot_verified` report/gate/smoke/readiness fields without making live smoke mandatory.
- 2026-05-19: Added acceptance-level proof table for `static_only`, `godot_load_verified`, and `godot_verified`.
- 2026-07-09: Added root `VERSION` release metadata closeout and regression coverage that keeps it aligned with `pyproject.toml`.
