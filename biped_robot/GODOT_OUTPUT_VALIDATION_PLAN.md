# Godot Output Validation Plan

## Goal

Validate whether the Godot-side software simulation meets the current test criteria by checking generated output logs and reports.

## Scope

In scope:

- Godot input contract: `config/godot_io_input.json`
- Godot telemetry: `test_env/godot_io_telemetry.jsonl`
- Godot summary report: `test_env/godot_io_report.json`
- Communication report and events derived from Godot output
- Local acceptance report and retention manifest

Out of scope:

- Repository-wide regression testing
- Real hardware, serial, CAN, ROS2, or external communication runtime
- Files outside `biped_robot/`

## Acceptance Criteria

- Godot IO report status is `passed`.
- Godot telemetry has at least the configured minimum row count.
- All configured Godot commands are applied.
- Required telemetry fields are present in every row.
- Roll and pitch remain within configured thresholds.
- Communication simulation derived from Godot output passes delivery, ACK, latency, jitter, and loss gates.
- Generated Godot and communication artifacts are present in the retention manifest.

## Validation Commands

```powershell
py -3.12 biped_robot\tools\run_local_acceptance.py
py -3.12 biped_robot\tools\validate_godot_io.py
```

## Evidence Files

- `test_env/godot_io_report.json`
- `test_env/godot_io_telemetry.jsonl`
- `test_env/communication_report.json`
- `test_env/communication_events.jsonl`
- `test_env/local_acceptance_report.json`
- `test_env/retention_manifest.json`

## Decision Rule

The Godot output log validation is accepted only if all relevant local acceptance, Godot IO, and communication checks pass. Missing hardware checks remain documented risk, not a Godot-output failure.
