# Module Goal

Provide one project-level automation entrypoint that starts from frontend-style inputs, exercises the main Web/Godot software paths, runs the local Godot demonstration evidence, and writes a detailed acceptance report.

# Ownership

- `tools/run_frontend_godot_project_acceptance.py`
- Frontend-style input fixture embedded in the runner until a dedicated UI fixture is needed
- Generated reports under `test_env/frontend_godot_project_acceptance/`
- Existing `biped_robot/` Godot demo and local acceptance artifacts

# Inputs and Outputs

Inputs:

- Frontend-equivalent values: robot description command, task CRUD fields, Godot connection host/port, simulation physics inputs, target speed, robot template.
- Existing Web Panel API routes through FastAPI `TestClient`.
- Existing `biped_robot/tools/run_local_acceptance.py` local Godot/software evidence.

Outputs:

- `frontend_godot_project_acceptance_report.v1` JSON report.
- Route-level evidence for frontend input mapping, Web API responses, Godot control calls, workflow discovery and biped Godot coverage.
- Residual-risk section for hardware, ROS2 and full browser/manual validation.

# Contract Checklist

- Public surface exposed: `tools/run_frontend_godot_project_acceptance.py` CLI.
- Inputs accepted: optional `--output-root` and `--skip-biped-acceptance`.
- Outputs produced: JSON report with schema, status, frontend inputs, checks, artifacts, residual risks.
- Backward compatibility: does not change existing Web API behavior, Godot project behavior, or biped acceptance contracts.
- Security: no real hardware commands are sent; Godot controller is mocked for frontend route acceptance.

# Tasks

- [x] Implement frontend-input automation runner.
- [x] Capture route checks for task CRUD, command parsing, Godot control, workflow discovery.
- [x] Invoke biped local Godot acceptance unless explicitly skipped.
- [x] Write detailed report with artifacts and residual risks.
- [x] Run syntax check and runner smoke.

# Notes

- 2026-06-11: Added `tools/run_frontend_godot_project_acceptance.py`. It maps frontend-style inputs to Web Panel route checks, safely mocks Web Godot control routes, invokes `biped_robot` local Godot acceptance, and writes `frontend_godot_project_acceptance_report.v1`.
- 2026-06-11: Godot 4.5.1 headless required a workspace-local `--log-file` path to avoid crashing on the default `user://logs` path in this environment.

# Risks and Mitigations

- Risk: TestClient route coverage is mistaken for a real browser session.
  Mitigation: Report `browser_runtime=not_run` explicitly and keep route evidence separate from browser/manual evidence.

- Risk: Mocked Godot control is mistaken for live Godot process control.
  Mitigation: Web route Godot control stays mocked; live/headless Godot evidence comes only from `biped_robot` acceptance artifacts.

# Validation

```powershell
py -3.12 -m py_compile tools\run_frontend_godot_project_acceptance.py
py -3.12 tools\run_frontend_godot_project_acceptance.py
```

# Completion Criteria

- The runner produces a detailed report from frontend-style input.
- Main Web route checks pass.
- Existing biped Godot demonstration/coverage evidence is invoked and referenced.
- Missing browser, hardware and ROS2 evidence are documented rather than silently accepted.
