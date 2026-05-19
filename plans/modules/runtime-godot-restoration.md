# Module Goal

让 live Godot smoke 产出的运行时节点树证据能和静态 `godot_node_tree_manifest.v1` 做同字段对比，逐步证明 JSON 机械结构从模型生成、参数设置到运动模拟的还原质量。

# Ownership

- `tools/run_dynamic_godot_robot_smoke.py`
- `agi_walker/core/api/robot_schema.py`
- `godot_project/scripts/robot_assembler.gd`
- `godot_project/scripts/generated_robot_controller.gd`
- `godot_project/scripts/tcp_server.gd`
- `tests/test_dynamic_godot_robot_generation.py`
- `docs/guides/DYNAMIC_GODOT_ROBOT_GENERATION.md`

# Interfaces and Contracts

## Inputs and Outputs

- Inputs consumed: normalized robot JSON, static `godot_node_tree_manifest.v1`, Godot load result `part_nodes` / `joint_nodes`, step telemetry `body_states` / `joint_states`.
- Outputs produced: runtime smoke `node_tree_manifest.static_manifest_version`, `node_tree_manifest.static_manifest_comparison`, mismatch counts, mismatch kind counts and preview mismatches.
- Runtime physical fields compared when available: `mass`, `collision_parameters` and `mesh_parameters`.
- Integration point: static manifest evidence remains the PR-level default; live Godot smoke consumes the same field names when it is run manually or in scheduled CI.

## Contract Checklist

- Public surface this module exposes: smoke report JSON fields under `node_tree_manifest`.
- Inputs this module accepts: static manifest payload plus runtime `part_nodes` and `joint_nodes` mappings.
- Outputs this module produces: comparison summary with `complete`, `mismatch_count`, `mismatch_kind_counts`, `mismatches`, `part_count`, `joint_count`, `restored_part_count`, `restored_joint_count`, `tolerance`.
- Shared types/schemas/config touched: `godot_node_tree_manifest.v1` and static mismatch kind names where applicable.
- Backward compatibility requirements: no new default failure unless existing smoke flags already fail; no live Godot requirement for non-live CI.
- Integration tests required: function-level non-live tests plus smoke report shape tests.

# Local Context

- `robot_assembler.gd` already returns runtime `part_nodes` and `joint_nodes` in load result / schema assembly summary.
- `run_dynamic_godot_robot_smoke.py` already builds a runtime `node_tree_manifest` and supports strict node-tree flags.
- `robot_schema.py` owns static manifest generation and validation; comparison logic should live there so report/gate/runtime code can share it.

# Non-Goals

- Do not make live Godot smoke mandatory in every PR.
- Do not rewrite Godot physics assembly.
- Do not replace the current JSON robot config format.
- Do not add Web/session evidence preservation in this module.

# Tasks

- [x] Add shared static-vs-runtime manifest comparison helper.
- [x] Wire runtime smoke report to include static manifest version and comparison summary.
- [x] Add non-live tests for exact match, missing runtime node, class mismatch and tolerance-sensitive vector mismatch.
- [x] Compare mass/collision/mesh fields where Godot readback is available.
- [x] Update docs with the new runtime comparison report fields.
- [x] Run targeted tests and non-live collect validation.

# Risks and Mitigations

- Risk: runtime comparison duplicates existing smoke checks.
  Mitigation: Keep the helper focused on static manifest field comparison and leave motion/telemetry gates in the smoke tool.
- Risk: applied parameter shape differs between static and runtime.
  Mitigation: Compare static expected `applied_parameters` against runtime `applied_parameters.runtime` when present.
- Risk: live Godot behavior changes accidentally.
  Mitigation: Keep failure behavior unchanged unless existing strict flags already detect the same issue.

# Validation

```powershell
py -3.12 -m py_compile tools\run_dynamic_godot_robot_smoke.py agi_walker\core\api\robot_schema.py
py -3.12 -m pytest tests\test_dynamic_godot_robot_generation.py -q
py -3.12 -m pytest -m "not live" --collect-only -q
```

# Completion Criteria

- Runtime smoke report includes a static manifest version and comparison summary.
- Shared helper reports missing, unexpected and value mismatch cases with stable counts.
- Tests cover comparison success and representative drift cases without requiring Godot.

# Notes

- Live Godot execution remains a follow-up validation step when a Godot executable is available.
- 2026-05-18: Added non-live static-vs-runtime comparison contract; live Godot execution was not run in this step.
- 2026-05-18: Extended static-vs-runtime comparison to include nested collision and mesh parameter readback plus mass.

# Drift Check

Before and after implementation, verify this module still matches `PROJECT_PLAN.md`: static evidence remains mandatory, runtime restoration comparison is the next step, and Web/session preservation stays out of scope.
