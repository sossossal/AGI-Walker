# Module Goal

在 Web 操作界面中展示 static/load/live 三档 evidence、manifest mismatch 和残余风险，并支持用户从 artifact 一键进入 Godot load 验证或查看 readiness summary。

# Ownership

- `web_panel/workflows_api.py`
- `web_panel/static/workflows.html`
- Web workflow artifact metadata and Godot load/sync routes
- `tests/test_web_panel_integration_routes.py`

# Inputs and Outputs

Inputs:

- Workflow artifacts and Godot-loadable generated robot outputs.
- Static manifest evidence, delivery gate and release/readiness summary JSON.

Outputs:

- Web-facing evidence summary with level, completeness, mismatch counts, residual risk and artifact links.
- User actions for Godot load verification and readiness summary inspection.

# Contract Checklist

- Public surface this module exposes: Web API response fields, UI labels/states, artifact action routes.
- Inputs this module accepts: artifact metadata, delivery records, readiness summaries.
- Outputs this module produces: static/load/live evidence panels and action results.
- Shared types/schemas/config touched: Web delivery gate evidence fields and release/readiness fields.
- Backward compatibility requirements: existing workflow routes and artifact downloads remain compatible.
- Integration tests required: API payload tests, route tests, static UI contract tests.

# Local Context

Web/session evidence already preserves static manifest output paths through Godot delivery evidence. The next increment should make those fields visible and actionable for operators.

# Non-Goals

- Do not require live Godot for the Web UI to render evidence.
- Do not replace CLI release/readiness tooling.

# Tasks

- [x] Define Web evidence summary payload for static/load/live levels.
- [x] Add API support for manifest mismatch counts and residual risk display.
- [x] Add UI state for `static_only`, `godot_load_verified`, `godot_verified` and incomplete evidence.
- [x] Add artifact action for Godot load verification when available.
- [x] Add artifact action or link for readiness summary generation/view.
- [x] Add integration tests proving Web flows preserve and expose evidence fields.
- [x] Update operator-facing documentation.

# Risks and Mitigations

- Risk: UI labels diverge from CLI acceptance levels.
  Mitigation: Reuse exact level names and summary fields from release/readiness contracts.

- Risk: Web actions depend on unavailable Godot runtime.
  Mitigation: Surface disabled/actionable states with explicit residual risk and retry hints.

# Validation

```powershell
py -3.12 -m pytest tests\test_web_panel_integration_routes.py -q
py -3.12 -m pytest tests\test_dynamic_godot_robot_generation.py -q
```

# Completion Criteria

- Web users can inspect evidence level, mismatch counts and residual risk.
- Godot load/readiness artifact actions are discoverable and tested.
- Existing Web workflow routes remain compatible.

# Notes

- Keep UI text concise and evidence-driven; avoid duplicating long docs inside the app.
- 2026-05-19: Added `web_godot_evidence_summary.v1` to Web run
  responses and Godot load/sync responses. It summarizes static/load/live
  levels, manifest mismatch counts, residual risks and Godot load/readiness
  actions. Added `/api/workflows/runs/{run_id}/godot-readiness-summary` for a
  run-scoped `dynamic_godot_release_readiness_summary.v1` view.

# Drift Check

Before implementation, verify Web payload fields match root plan contracts and release/readiness summary names.

After implementation, the new Web payload reuses the root plan level names
`static_only`, `godot_load_verified`, `godot_verified`, and `incomplete`, and
does not require live Godot for static evidence display.
