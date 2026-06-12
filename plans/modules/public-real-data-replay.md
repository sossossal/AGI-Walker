# Module Goal

Add a project-level public real-data replay evidence layer that strengthens Godot, biped, actuator, contact and communication simulation without claiming real hardware validation.

# Ownership

- `biped_robot/config/public_real_data_sources.json`
- `biped_robot/fixtures/public_real_data_replay_fixture.json`
- `biped_robot/tools/run_public_real_data_replay.py`
- Generated artifacts under `biped_robot/test_env/`
- Biped local validation, coverage requirements and documentation entries that reference this replay evidence

# Inputs and Outputs

Inputs:

- Public dataset source metadata with URL, role, citation and offline sample status.
- Small offline normalized replay fixture rows.
- Existing biped robot, actuator, Godot IO and communication contracts.

Outputs:

- `public_real_data_replay_report.v1`
- `public_real_data_replay_trace.jsonl`
- Full coverage domain entry `public_real_data_replay`
- Retention manifest entries for replay report and trace artifacts

# Contract Checklist

- Public surface exposed: `biped_robot/tools/run_public_real_data_replay.py` CLI.
- Inputs accepted: `--sources`, `--fixture`, `--trace-output`, `--report-output`.
- Outputs produced: report JSON and replay trace JSONL.
- Shared configs touched: biped coverage requirements and workspace validation contract.
- Backward compatibility: existing Godot, communication, actuator and hardware contracts remain unchanged.
- Integration tests required: biped workspace validation and folder-scoped local acceptance.

# Local Context

The biped folder already has deterministic Godot IO, communication, actuator physics, contact stability and full coverage reporting. This module adds a replay evidence domain over public dataset-shaped samples, but real hardware, serial/CAN and ROS2 runtime gates remain blocked by the hardware gap report.

# Non-Goals

- Do not download large public datasets during default validation.
- Do not touch real hardware, ROS2, serial/CAN or driver modules.
- Do not upgrade public dataset replay into live hardware validation.

# Tasks

- [x] Add public source metadata and offline replay fixture.
- [x] Add replay CLI that maps legged robot, ROS2-like and CAN-like rows into local evidence domains.
- [x] Include replay artifacts in retention and full coverage checks.
- [x] Update biped docs and root change control.
- [x] Run local validation and acceptance.

# Risks and Mitigations

- Risk: Public sample replay is mistaken for live hardware evidence.
  Mitigation: Report fields explicitly mark `public_dataset_replay`, `simulated_transport` and `real_hardware_not_run`.

- Risk: Network availability makes default tests flaky.
  Mitigation: Default validation uses only offline normalized samples and source metadata.

# Validation

```powershell
D:\actions-tools\Python312\python.exe -m py_compile biped_robot\tools\run_public_real_data_replay.py
D:\actions-tools\Python312\python.exe biped_robot\tools\run_public_real_data_replay.py
D:\actions-tools\Python312\python.exe biped_robot\tools\validate_biped_workspace.py
D:\actions-tools\Python312\python.exe biped_robot\tools\run_local_acceptance.py
```

# Completion Criteria

- Replay report status is `passed`.
- Replay trace JSONL exists and is retained.
- Full coverage shows `public_real_data_replay` covered.
- Real-world coverage remains blocked without external runtime/hardware.

# Drift Check

This module remains aligned with the master plan because it adds software evidence only and preserves the explicit hardware/ROS2 blockers.
