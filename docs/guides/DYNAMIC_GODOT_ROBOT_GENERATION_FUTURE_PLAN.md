# Dynamic Godot Robot Generation Future Plan

# Goal

把“从 JSON 自动生成 Godot 机械节点树，并还原部件完整机械机构，从模型生成到参数设置，再到运动模拟”的能力推进到可持续交付状态：

- 静态层：JSON、normalized config、static `godot_node_tree_manifest.v1`、delivery gate 和 CI 门禁保持一致。
- 运行层：Godot 实例化出的节点树、物理节点、关节参数、控制器读回与静态 manifest 能互相校验。
- 验收层：静态 CI、可选 live Godot smoke、报告摘要和文档形成同一条闭环，不依赖人工解释。

# Scope

In scope:

- `agi_walker/core/api/robot_schema.py` 的机器人 schema、normalizer、manifest builder 和 manifest validator。
- `tools/validate_robot_config_for_godot.py`、`tools/build_dynamic_robot_generation_report.py`、`tools/validate_delivery_acceptance_gate.py` 的 CLI 验收链。
- `godot_project/scripts/robot_assembler.gd`、`generated_robot_controller.gd`、`tcp_server.gd` 的 Godot runtime 生成、读回和 smoke 接口。
- `tests/test_dynamic_godot_robot_generation.py`、`tests/test_workflow_contracts.py`、fixture JSON、CI workflow 和本指南文档。

Out of scope for the next static-focused increments:

- 不默认要求真实 Godot 可执行文件或图形环境。
- 不把 live smoke 变成所有 PR 的必跑步骤。
- 不引入新的机器人描述格式来替代当前 JSON schema。
- 不重写 Godot 物理建模架构，除非静态/运行契约已经证明现有结构无法表达。

# Existing Context

当前已经完成：

- JSON normalizer 能生成 Godot-ready config。
- 静态 `godot_node_tree_manifest.v1` 能描述计划中的 `robot_node`、controller、part nodes、joint nodes、path maps、endpoint completeness 和 joint parameters。
- Manifest 自校验已覆盖结构、计数、完整性布尔、missing endpoints、part/joint node 基础字段、duplicate lookup key、root drift 和 path map mismatch kind。
- `validate_robot_config_for_godot.py` 能生成并校验 manifest sidecar。
- `build_dynamic_robot_generation_report.py` 已把静态 manifest 诊断汇总进 report、batch summary、robot summary 和 delivery gate summary counts。
- `validate_delivery_acceptance_gate.py` 能扫描 sidecar 目录，默认跳过普通 gate 校验，同时记录 valid/invalid、validation error、path incomplete、path map mismatch 和 kind counts；严格模式可 fail closed。
- CI 已新增静态 Godot node-tree manifest gate，生成 fixture sidecar 后做严格目录验收。
- 本文原始 Future Plan 范围已完成；后续 Phase 2 扩展已迁移到根级 `PROJECT_PLAN.md` 和 `plans/modules/`，并完成 live Godot 验证产品化、运行时机械行为 evidence、Web evidence dashboard、schema 1.5 演进和 release evidence bundle。

# Architecture Principles

- Contract first: 新字段先进入 schema/report/gate 契约，再接 Godot runtime 或 CI。
- Static before live: 所有能静态证明的拓扑、路径、参数和完整性先在 manifest 中闭合，live smoke 只验证运行时还原。
- One evidence chain: config、manifest、report、gate summary、CI artifact 使用同一组字段名和计数含义。
- Backward compatible by default: sidecar 在普通目录扫描中仍保持 skipped；严格失败只由显式 flag 或 CI profile 触发。
- Drift visible: path map、root、duplicate key、missing endpoint、parameter readback 等漂移必须进入结构化 preview，而不是只进文本日志。

# Module Index

| Module | Subplan | Responsibility | Dependencies | Status |
| --- | --- | --- | --- | --- |
| Static schema and manifest | 本文 `Tasks` / `Static schema`；`PROJECT_PLAN.md` / `plans/modules/static-godot-node-tree-evidence.md` | JSON normalize、manifest build/validate、path map mismatch | Fixtures, report builder | complete |
| Report and acceptance gate | 本文 `Tasks` / `Report and gate`；`PROJECT_PLAN.md` / `plans/modules/static-godot-node-tree-evidence.md` | CLI 输出、summary counts、sidecar strict gate、CI evidence | Static manifest contract | complete |
| Godot runtime restoration | 本文 `Tasks` / `Runtime restoration`；`PROJECT_PLAN.md` / `plans/modules/runtime-godot-restoration.md` | Godot 节点树实例化、runtime manifest、参数读回、motion smoke | Static manifest, Godot scripts | complete |
| Web/session integration | 本文 `Tasks` / `Web integration`；`PROJECT_PLAN.md` / `plans/modules/web-session-godot-evidence.md` | Web workflow 加载、session bridge evidence、operator-facing artifact flow | Runtime smoke, report builder | complete |
| Release evidence and docs | 本文 `Tasks` / `Release evidence`；`PROJECT_PLAN.md` / `plans/modules/release-evidence-readiness.md` | 黄金样例、runbook、CI artifact retention、manual/live checklist | All modules | complete |
| Phase 2 productization | `PROJECT_PLAN.md` / `plans/modules/` | live profile、mechanical behavior evidence、Web dashboard、schema 1.5、release evidence bundle | All modules | complete |

# Cross-Module Contracts

- Static manifest version: `godot_node_tree_manifest.v1`。
- Static manifest required identity fields: `manifest_version`, `robot_name`, `robot_node`, `controller_node`。
- Static manifest topology fields: `parts_count`, `joints_count`, `parameterized_joints`, `part_nodes`, `joint_nodes`, `part_node_paths`, `joint_node_paths`。
- Static manifest completeness fields: `complete`, `parameters_complete`, `endpoint_paths_complete`, `path_maps_complete`, missing endpoint lists and details.
- Path map mismatch kinds: `missing`, `unexpected`, `value_mismatch`, `duplicate`, `root_mismatch`。
- Report/gate summary counts must keep scalar count fields non-negative and map count sums equal to their aggregate mismatch count.
- Runtime manifest must eventually map back to static manifest by `part_id`, `connection_name`, `body_node`, `joint_node`, `node_a`, `node_b`, `origin`, `axis`, and `applied_parameters`。

# Integration Plan

1. Keep the current static CI gate as the mandatory PR-level guard.
2. Add a reusable golden static evidence command that emits report, gate, sidecar directory, validation summary, and closeout report from the same fixture set.
3. Extend live Godot smoke to compare runtime node-tree evidence against static manifest fields, starting with node existence and class checks.
4. Add parameter readback and fixed-lock checks to runtime manifest comparison.
5. Promote live Godot smoke to manual/scheduled CI only, with artifacts retained for inspection.
6. Wire Web/session delivery reports to reference the same static/live evidence fields.
7. Add release/readiness summary that states which level is proven: static-only, Godot-load verified, or full mechanical motion verified.

# Tasks

Static schema:

- [x] Add static `godot_node_tree_manifest.v1` builder and self-validator.
- [x] Validate counts, completeness flags, missing endpoint consistency, duplicate lookup keys and root drift.
- [x] Emit path map mismatch previews and mismatch kind counts.
- [x] Add a small compatibility matrix for schema versions and accepted manifest versions.
- [x] Add negative fixtures for duplicate part ids, duplicate connection names and root drift as reusable JSON artifacts, not only inline mutations.

Report and gate:

- [x] Add `node_tree_manifest_errors` and path map mismatch output to config validator.
- [x] Add static manifest diagnostics to report, batch summary, robot summary and delivery gate summary counts.
- [x] Add sidecar directory scan with valid/invalid, mismatch count and kind-count aggregation.
- [x] Add CI static sidecar gate with fail-closed options and exact zero-drift assertions.
- [x] Add a single documented `golden static evidence` command block that writes all expected artifacts into one directory.
- [x] Add a compact machine-readable closeout report for static evidence readiness.

Runtime restoration:

- [x] Extend live smoke runtime `node_tree_manifest` to include explicit source static manifest version and comparison result.
- [x] Compare runtime part/joint node existence against static `part_nodes` and `joint_nodes`.
- [x] Compare runtime node classes against static `body_class` and `joint_class`.
- [x] Compare runtime transforms, origins and axes within `--node-tree-tolerance`.
- [x] Compare mass/collision/mesh fields where Godot readback is available.
- [x] Compare fixed joint lock state and parameter application.
- [x] Add runtime mismatch kind counts aligned with static mismatch naming where possible.

Web integration:

- [x] Ensure workflow Godot load artifacts preserve static manifest output paths.
- [x] Add Web/session delivery summary fields for static manifest valid/invalid and runtime restoration level.
- [x] Add route/API tests that prove Web Godot sync does not drop manifest evidence.
- [x] Keep browser/manual validation separate from non-live CI.

Release evidence:

- [x] Define golden fixture set for biped, quadruped and fixed pair static evidence.
- [x] Add release/readiness report section: static manifest gate, sidecar gate, live Godot smoke status and residual risk.
- [x] Add manual live smoke checklist that references exact artifact paths and expected summary fields.
- [x] Document acceptance levels and which commands prove each level.

# Risks and Mitigations

- Risk: Static manifest becomes too strict and rejects old artifacts.
  Mitigation: Keep default sidecar scan skipped; use strict failure only through explicit flags and CI profile.

- Risk: Runtime Godot evidence diverges from static manifest field names.
  Mitigation: Reuse `part_id`, `connection_name`, node path and parameter field names from static manifest.

- Risk: Live Godot smoke is flaky on CI.
  Mitigation: Keep live smoke manual/scheduled, keep static gate as PR-level default, and retain artifacts for diagnosis.

- Risk: Report/gate field growth becomes hard to validate.
  Mitigation: Every new summary count must be in workflow contract schema, tests and docs in the same change.

- Risk: JSON fixture coverage is too narrow.
  Mitigation: Keep fixed pair, biped and quadruped as golden paths; add negative fixtures for malformed topology and manifest drift.

# Validation

Default non-live validation:

```powershell
py -3.12 -m py_compile tools\validate_robot_config_for_godot.py tools\validate_delivery_acceptance_gate.py tools\build_dynamic_robot_generation_report.py agi_walker\core\api\robot_schema.py agi_walker\core\api\workflow_contracts.py
py -3.12 -m pytest tests\test_dynamic_godot_robot_generation.py -q
py -3.12 -m pytest tests\test_workflow_contracts.py -q
py -3.12 -m pytest -m "not live" --collect-only -q
```

Static golden evidence command:

```powershell
py -3.12 tools\build_static_godot_node_tree_evidence.py tests\fixtures\robot_dynamic_fixed_pair.json tests\fixtures\robot_dynamic_biped.json tests\fixtures\robot_dynamic_quadruped.json --output-root test_env\static_godot_node_tree_manifest_ci --manifest-dir test_env\static_godot_node_tree_manifests
```

Live validation, when Godot is available:

```powershell
py -3.12 tools\build_dynamic_robot_generation_report.py tests\fixtures\robot_dynamic_biped.json --run-godot-smoke --fail-on-full-node-tree-restoration --fail-on-parameter-mismatch --output test_env\dynamic_godot_biped_live_report.json --gate-output test_env\dynamic_godot_biped_live_gate.json
```

# Acceptance Criteria

- Static CI gate produces sidecars, validates them strictly, and uploads report/gate/summary artifacts.
- Report, gate and workflow contract expose the same manifest diagnostic counts.
- A malformed sidecar cannot pass strict mode just because `manifest_version` is correct.
- Runtime smoke can prove at least one fixture reaches Godot load verified with node existence and class checks.
- Full mechanical restoration gate includes parameter readback, transform/axis checks, fixed-lock checks and motion evidence.
- Documentation states which command proves each acceptance level.

# Decision Log

- 2026-05-18: Keep static sidecar scan backward compatible by default; strict behavior is opt-in through flags and CI profile.
- 2026-05-18: Treat static manifest evidence as the PR-level default; live Godot smoke remains manual/scheduled until runtime flakiness and environment dependencies are controlled.
- 2026-05-18: Keep this future plan under `docs/guides/` to follow the repository's existing planning convention instead of introducing a root-level `PROJECT_PLAN.md`.
- 2026-05-18: Web/session delivery now preserves generated static manifest evidence in `static_node_tree_manifest_evidence` and source-specific gate summary counts.
- 2026-05-18: Dynamic Godot release/readiness summary now reports the strongest proven level from existing closeout, gate, report, or Web delivery evidence.
- 2026-05-18: Static schema compatibility matrix and reusable negative fixtures now cover accepted robot schema versions, manifest version, duplicate ids and root drift.
- 2026-05-18: Static-vs-runtime manifest comparison now includes mass, collision parameter and mesh parameter readback where Godot load mappings provide it.
- 2026-05-18: Web workflow artifact metadata now preserves known static manifest sidecar output paths through Godot delivery evidence and gate summary counts.
- 2026-05-18: Non-live static Godot CI remains separate from browser/manual validation; browser evidence is explicit release evidence, not a static gate dependency.
- 2026-05-19: Golden static evidence fixture set is fixed pair, biped, and quadruped for release/readiness static coverage.
- 2026-05-19: Manual live smoke checklist now names exact report, gate, smoke, and readiness artifact paths plus expected `godot_verified` fields.
- 2026-05-19: Acceptance levels now document proof commands and required fields for `static_only`, `godot_load_verified`, and `godot_verified`.
- 2026-05-19: Future Plan status synchronized with root `PROJECT_PLAN.md`; all original modules are complete, and Phase 2 productization follow-up work is tracked and complete in `PROJECT_PLAN.md` plus `plans/modules/`.

# Change Control

- Deferred: mandatory live Godot smoke on every PR.
- Deferred: replacing JSON with a new robot description format.
- Deferred: broad Godot physics architecture rewrite.
- Approved next focus: golden static evidence closeout, then runtime manifest comparison, then Web/session evidence preservation.
- Approved Web/session evidence increment: preserve static manifest validity/completeness/path-map evidence through Web Godot load records without requiring live motion smoke.
- Approved release/readiness increment: summarize existing evidence into `static_only`, `godot_load_verified`, or `godot_verified` without making live Godot mandatory.
- Closed: original Future Plan implementation scope is complete; subsequent dynamic Godot productization scope is represented by root `PROJECT_PLAN.md` and module subplans.
