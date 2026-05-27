# Module Goal

Close the Phase 3 control/communication simulation evidence loop with a reusable closeout artifact that summarizes local non-live simulation proof, retained artifacts, validation errors and external blockers for live hardware.

# Ownership

- `tools/build_control_comm_simulation_closeout.py`
- `tests/test_control_communication_simulation.py`
- `docs/guides/CONTROL_COMMUNICATION_SIMULATION.md`
- Existing `control_comm_simulation_report.v1` and generated artifact contracts.

# Inputs and Outputs

Inputs:

- `control_comm_simulation_report.json`
- Artifact paths referenced by the report: Godot log preview, Zenoh mapping/trace, EtherCAT model trace, motor/joint trace, simulator adapter boundary and live hardware migration gate.

Outputs:

- `control_comm_simulation_closeout.v1`
- Machine-readable status, artifact validation errors, artifact size/hash metadata, closeout self-validation errors, evidence level, live hardware blockers and residual risks.

# Contract Checklist

- Public surface this module exposes: `tools/build_control_comm_simulation_closeout.py` CLI and `control_comm_simulation_closeout.v1` JSON.
- Inputs this module accepts: existing simulation report path.
- Outputs this module produces: self-validating closeout JSON suitable for release evidence collection.
- Shared types/schemas/config touched: Phase 3 control/communication simulation report and artifact contracts.
- Backward compatibility requirements: no changes to existing simulation report or artifact field meanings.
- Integration tests required: successful closeout from generated artifacts and blocked closeout when report validation fails.

# Local Context

- `tools/run_control_comm_simulation.py` already generates all local non-live artifacts and validates them.
- Live CAN/EtherCAT/TSN evidence remains blocked by `live_hardware_migration_gate.v1`.
- This closeout summarizes evidence; it must not weaken the hardware gate.

# Non-Goals

- Do not connect to real CAN, EtherCAT, TSN, Zenoh, Gazebo, MuJoCo, Isaac Sim or live Godot.
- Do not change control loop, bus, fieldbus or motor/joint model behavior.
- Do not mark real hardware, external simulator or real transport validation as passed.

# Tasks

- [x] Add module plan and root-plan closeout scope.
- [x] Add closeout CLI for existing simulation report artifacts.
- [x] Add tests for successful closeout and validation failure.
- [x] Update user-facing guide with the closeout command.
- [x] Add closeout artifact self-validation.
- [x] Add retained artifact size and SHA-256 integrity metadata.

# Risks and Mitigations

- Risk: Closeout is mistaken for live hardware acceptance.
  Mitigation: Closeout uses `evidence_level=non_live_simulation` and carries live hardware blockers forward.

- Risk: Closeout duplicates simulation generation logic.
  Mitigation: Closeout only reads the report and validates referenced artifacts.

# Validation

```powershell
py -3.12 -m py_compile tools\build_control_comm_simulation_closeout.py
py -3.12 -m pytest tests\test_control_communication_simulation.py -q
```

# Completion Criteria

- A generated simulation report can be converted into `control_comm_simulation_closeout.v1`.
- Missing or malformed report/artifact evidence blocks the closeout.
- Malformed closeout JSON is rejected by `validate_control_comm_simulation_closeout`.
- Present artifacts record `size_bytes` and `sha256`; missing artifacts keep those fields null.
- Live hardware blockers remain visible in the closeout.

# Notes

- 2026-05-26: Created as the next plan after the approved non-live Phase 3 simulation scope was completed.
- 2026-05-26: Added closeout self-validation for status, evidence level, artifact error accounting and live hardware release gate status.
- 2026-05-26: Added artifact integrity metadata to closeout results so retained evidence can be checked for replacement or truncation.

# Drift Check

This module must remain a closeout/evidence layer over existing Phase 3 artifacts and must not become a live hardware validation path.
