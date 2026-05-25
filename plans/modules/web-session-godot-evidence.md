# Goal

Preserve static Godot node-tree manifest evidence through Web/session Godot delivery so UI-triggered loads carry the same contract language as CLI reports and delivery gates.

# Scope

In scope:

- Build static node-tree manifest evidence from the workflow robot config artifact before Web Godot delivery records are serialized.
- Include manifest validation, completeness, output-path presence, and path-map mismatch counts in Web delivery records.
- Extend `web_godot_delivery` acceptance gate summary counts to accept the static manifest evidence fields.
- Add tests for successful session-bridge delivery and contract compatibility.

Out of scope:

- Running live Godot smoke from the Web workflow.
- Changing the robot JSON schema or session bridge transport protocol.
- Adding a full UI redesign for evidence display.

# Interfaces and Contracts

- Source: `web_godot_delivery`.
- Verification scope: `godot_load`.
- Static manifest version: `godot_node_tree_manifest.v1`.
- Web delivery record field: `static_node_tree_manifest_evidence`.
- Gate summary fields reused from existing static manifest contract:
  - `static_node_tree_manifest_count`
  - `static_node_tree_manifest_valid_count`
  - `static_node_tree_manifest_invalid_count`
  - `static_node_tree_manifest_error_count`
  - `static_node_tree_manifest_output_count`
  - `static_node_tree_manifest_path_map_mismatch_count`
  - `static_node_tree_manifest_path_map_mismatch_kind_counts`
  - `static_node_tree_complete_count`
  - `static_node_tree_incomplete_count`
  - `static_node_tree_endpoint_paths_complete_count`
  - `static_node_tree_endpoint_paths_incomplete_count`
  - `static_node_tree_parameters_complete_count`
  - `static_node_tree_parameters_incomplete_count`

# Implementation Tasks

- [x] Add Web delivery static manifest evidence helper.
- [x] Attach evidence to success and failure delivery records.
- [x] Extend `web_godot_delivery` source summary contract fields.
- [x] Preserve upstream static manifest sidecar output paths on Godot-loadable workflow artifacts.
- [x] Keep browser/manual validation separate from non-live CI.
- [x] Update Web/session integration tests.
- [x] Update project plan and documentation.
- [x] Run targeted non-live validation.

# Validation

```powershell
py -3.12 -m py_compile web_panel\workflows_api.py agi_walker\core\api\workflow_contracts.py
py -3.12 -m pytest tests\test_web_panel_integration_routes.py -q
py -3.12 -m pytest tests\test_workflow_contracts.py -q
py -3.12 -m pytest tests\test_dynamic_godot_robot_generation.py -q
py -3.12 -m pytest -m "not live" --collect-only -q
```

# Progress Notes

- 2026-05-18: Created module plan because the Web/session evidence module was listed in `PROJECT_PLAN.md` but had no subplan.
- 2026-05-18: Added Web delivery `static_node_tree_manifest_evidence` and source-specific gate summary counts for manifest validity, completeness, output coverage and path-map mismatches.
- 2026-05-18: Validated with py_compile, Web integration tests, workflow contract tests, dynamic Godot generation tests, and non-live collect-only.
- 2026-05-18: Added artifact metadata pass-through for `static_node_tree_manifest_output` / `node_tree_manifest_output`, plus sibling sidecar discovery for `*.node_tree_manifest.json`.
- 2026-05-18: Added CI contract coverage that keeps Web browser/manual validation commands out of the static Godot node-tree gate.
