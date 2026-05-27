# Module Goal

Add a reusable non-live CI evidence profile for Phase 3 control/communication simulation so pull requests and scheduled runs can retain generated simulation artifacts and closeout evidence without requiring Godot, Zenoh, external simulators or hardware.

# Ownership

- `.github/workflows/ci.yml`
- `tests/test_active_path_references.py`
- `tools/run_control_comm_simulation.py`
- `tools/build_control_comm_simulation_closeout.py`
- `docs/guides/CONTROL_COMMUNICATION_SIMULATION.md`

# Inputs and Outputs

Inputs:

- Existing deterministic control/communication simulation CLI.
- Existing closeout CLI over `control_comm_simulation_report.json`.

Outputs:

- CI artifact directory under `test_env/control_comm_simulation_ci`.
- Uploaded GitHub Actions artifact named `control-communication-simulation-artifacts`.
- Workflow-shape tests that keep the CI profile non-live.

# Contract Checklist

- Public surface this module exposes: GitHub Actions job `control-communication-simulation-evidence`.
- Inputs this module accepts: repository checkout and Python runtime only.
- Outputs this module produces: retained non-live report, traces, Godot log preview, closeout and validation output.
- Shared types/schemas/config touched: CI workflow and existing Phase 3 simulation/closeout artifact contracts.
- Backward compatibility requirements: no change to default simulation report schema, closeout schema or live hardware gate semantics.
- Integration tests required: workflow static-reference test plus existing simulation/closeout tests.

# Local Context

- `tools/run_control_comm_simulation.py` already writes validated deterministic artifacts.
- `tools/build_control_comm_simulation_closeout.py` already self-validates closeout and artifact integrity metadata.
- Existing CI has a static Godot manifest artifact job and keeps live Godot/manual checks separate.

# Non-Goals

- Do not install or run Godot.
- Do not open a real Zenoh session.
- Do not start EtherCAT/CAN/TSN hardware or simulator runtimes.
- Do not make this profile depend on Docker or security scanner jobs.

# Tasks

- [x] Add module plan and root-plan CI profile scope.
- [x] Add GitHub Actions job that generates control/communication simulation artifacts.
- [x] Add GitHub Actions step that builds the closeout artifact.
- [x] Upload retained non-live evidence artifacts.
- [x] Add workflow-shape test for commands, artifact name and non-live boundaries.
- [x] Update guide with CI profile behavior.

# Risks and Mitigations

- Risk: CI artifact is misread as live transport or hardware validation.
  Mitigation: Job name and docs use non-live simulation wording, and closeout preserves live hardware blockers.

- Risk: CI profile drifts from local commands.
  Mitigation: Workflow-shape test locks the runner, closeout command, output root and artifact name.

# Validation

```powershell
py -3.12 -m py_compile tools\run_control_comm_simulation.py tools\build_control_comm_simulation_closeout.py agi_walker\core\simulation\control_comm_simulation.py
py -3.12 tools\run_control_comm_simulation.py --output-root test_env\control_comm_simulation_ci
py -3.12 tools\build_control_comm_simulation_closeout.py --report test_env\control_comm_simulation_ci\control_comm_simulation_report.json --output test_env\control_comm_simulation_ci\control_comm_simulation_closeout.json
py -3.12 -m pytest tests\test_control_communication_simulation.py tests\test_godot_control_comm_simulation_artifacts.py tests\test_active_path_references.py -q
```

# Completion Criteria

- CI has a default non-live control/communication evidence job after smoke tests.
- CI uploads retained artifacts even when a later validation step fails.
- Tests prove the CI profile invokes both simulation and closeout commands.
- Tests prove the CI job does not call the Godot replay runner or live hardware tooling.

# Notes

- 2026-05-26: Created to make Phase 3 evidence reusable by CI without changing live validation requirements.

# Drift Check

This module must remain an evidence-retention CI profile over deterministic non-live simulation artifacts. Any live runtime, hardware, Docker or external simulator requirement belongs in a separate opt-in profile.
