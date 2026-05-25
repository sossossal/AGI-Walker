# Module Goal

规划机器人 schema 1.5 的可选扩展字段，为 actuator、sensor、joint limits、controller tuning 和 material/physics presets 提供向后兼容的表达方式。

# Ownership

- `agi_walker/core/api/robot_schema.py`
- `tests/fixtures/robot_dynamic_*.json`
- `tests/test_dynamic_godot_robot_generation.py`
- Schema compatibility documentation

# Inputs and Outputs

Inputs:

- Existing schema versions 1.1, 1.2, 1.3, 1.4 and 1.5.
- Current static manifest and runtime restoration contracts.
- Phase 2 runtime behavior evidence needs.

Outputs:

- Schema 1.5 compatibility contract.
- Optional field definitions and validation for actuator, sensor, joint limits, controller tuning and material/physics presets.
- Migration and compatibility tests proving older schemas still load.

# Contract Checklist

- Public surface this module exposes: schema version support, optional field names, normalizer output shape.
- Inputs this module accepts: old and new JSON robot configs.
- Outputs this module produces: normalized config and manifest-ready fields for schema 1.5 optional metadata.
- Shared types/schemas/config touched: robot config schema, manifest builder, Godot-ready output.
- Backward compatibility requirements: versions 1.1 through 1.4 remain accepted while 1.5 is current.
- Integration tests required: compatibility matrix, migration fixtures, negative malformed field tests.

# Local Context

Current emitted schema version is 1.5 and accepted versions are 1.1 through 1.5. Phase 2 behavior evidence needs richer optional mechanical metadata but must not force old configs to migrate immediately.

# Non-Goals

- Do not replace the JSON robot config format.
- Do not make schema 1.5 fields required before compatibility tests and docs are complete.
- Do not remove support for older accepted schema versions.

# Tasks

- [x] Define schema 1.5 additive optional field contract.
- [x] Add actuator field draft and validation expectations.
- [x] Add sensor field draft and validation expectations.
- [x] Add joint limits field draft and runtime mapping expectations.
- [x] Add controller tuning field draft and defaulting behavior.
- [x] Add material/physics preset draft and manifest/runtime mapping expectations.
- [x] Add compatibility and migration tests for versions 1.1 through 1.5.
- [x] Update schema/version documentation.

# Risks and Mitigations

- Risk: New fields duplicate existing parameter mechanisms.
  Mitigation: Map each field to a clear owner and normalized output before implementation.

- Risk: Optional fields produce inconsistent runtime behavior.
  Mitigation: Default explicitly and add fixture tests for absent, partial and malformed fields.

# Validation

```powershell
py -3.12 -m py_compile agi_walker\core\api\robot_schema.py
py -3.12 -m pytest tests\test_dynamic_godot_robot_generation.py -q
py -3.12 -m pytest tests\test_workflow_contracts.py -q
py -3.12 -m pytest -m "not live" --collect-only -q
```

# Completion Criteria

- Schema 1.5 optional fields are documented and tested.
- Older schema versions remain accepted.
- Runtime behavior modules can consume new fields without hidden assumptions.

# Notes

- Treat schema 1.5 as a contract planning and compatibility task before broad runtime use.
- Schema 1.5 optional field contract is additive: connection-level `actuator`, `sensor`/`sensors`, extended `limits.effort`/`limits.velocity`, connection-level `controller`, and part-level `material`/`physics`.
- Validation evidence: `py -3.12 -m py_compile agi_walker\core\api\robot_schema.py`; `py -3.12 -m pytest tests\test_dynamic_godot_robot_generation.py -q`; `py -3.12 -m pytest tests\test_workflow_contracts.py -q`; `py -3.12 -m pytest -m "not live" --collect-only -q`.

# Drift Check

Before implementation, verify the schema plan remains additive and matches root plan compatibility requirements.
