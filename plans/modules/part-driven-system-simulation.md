# Module Goal

Create a part-parameter-driven full software simulation closeout for the biped robot that aggregates component inventory, actuator response, mountain motion, Godot IO, communication, public-data replay and hardware validation options into one report.

# Ownership

- `biped_robot/tools/run_part_driven_system_simulation.py`
- `biped_robot/tools/validate_part_driven_system_simulation.py`
- Generated artifacts under `biped_robot/test_env/`
- Biped coverage, validation and documentation entries that reference the system-level report

# Inputs and Outputs

Inputs:

- `biped_robot/config/robot.json`
- `biped_robot/config/actuators.json`
- Latest local simulation reports under `biped_robot/test_env/`
- `biped_robot/contracts/hardware_interface.json`

Outputs:

- `part_driven_system_simulation_report.v1`
- `part_driven_system_simulation_trace.jsonl`
- Validation result from `validate_part_driven_system_simulation.py`
- Full coverage domain entry `part_driven_system_simulation`

# Contract Checklist

- Public surface exposed: `biped_robot/tools/run_part_driven_system_simulation.py` CLI.
- Inputs accepted: optional `--report-output` and `--trace-output`.
- Outputs produced: system report JSON and trace JSONL.
- Backward compatibility: no changes to hardware, ROS2, serial/CAN or existing Godot contracts.
- Integration tests required: workspace validation, folder-scoped local acceptance and frontend-to-Godot project acceptance.

# Local Context

The biped folder already generates deterministic motion, actuator, communication, Godot IO, public replay, component parameter and hardware gap reports. This module closes the system-level software loop by validating that those artifacts are mutually present and consistent enough for a software-only readiness claim.

# Non-Goals

- Do not implement higher-fidelity rigid-body physics in this pass.
- Do not connect to hardware or runtime ROS2.
- Do not mark real motor transport, serial/CAN or ROS2 as passed.

# Tasks

- [x] Add system-level report and trace generator.
- [x] Add independent system report and trace validator.
- [x] Add coverage and retention requirements.
- [x] Update biped workspace validation, local acceptance and documentation.
- [x] Run targeted and project-level validation.

# Risks and Mitigations

- Risk: Aggregated report hides a failed lower-level artifact.
  Mitigation: System report checks each required source report status and fails if any required software artifact is missing or failed.

- Risk: Hardware option text is mistaken for hardware validation.
  Mitigation: Hardware validation options explicitly keep external runtime status blocked and require separate live evidence.

# Validation

```powershell
D:\actions-tools\Python312\python.exe -m py_compile biped_robot\tools\run_part_driven_system_simulation.py
D:\actions-tools\Python312\python.exe biped_robot\tools\run_part_driven_system_simulation.py
D:\actions-tools\Python312\python.exe biped_robot\tools\validate_part_driven_system_simulation.py
D:\actions-tools\Python312\python.exe biped_robot\tools\validate_biped_workspace.py
D:\actions-tools\Python312\python.exe biped_robot\tools\run_local_acceptance.py
D:\actions-tools\Python312\python.exe tools\run_frontend_godot_project_acceptance.py
```

# Completion Criteria

- System report status is `passed`.
- Full coverage marks `part_driven_system_simulation` as covered.
- Retention manifest contains the system report and trace.
- Real-world coverage remains blocked without external runtime/hardware.

# Drift Check

This module stays aligned with the master plan because it strengthens software simulation readiness while preserving the existing simulation-before-hardware boundary.
