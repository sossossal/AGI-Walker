# Module Goal

把 `godot_verified` 从结构和参数还原推进到可解释的机械行为 evidence，覆盖关节限位、力矩/速度响应、重心/稳定性、接触状态和 step-by-step motion trace。

# Ownership

- `godot_project/scripts/robot_assembler.gd`
- `godot_project/scripts/generated_robot_controller.gd`
- `tools/run_dynamic_godot_robot_smoke.py`
- `tools/build_dynamic_robot_generation_report.py`
- `agi_walker/core/api/robot_schema.py` runtime comparison helpers

# Inputs and Outputs

Inputs:

- Static manifest and normalized robot config.
- Runtime Godot node-tree/readback mappings.
- Motion smoke steps, tolerances and fixture expectations.

Outputs:

- Mechanical behavior evidence section with units, tolerances and per-step trace.
- Mismatch kind counts aligned with existing static/runtime mismatch conventions where possible.
- Gate summary fields that distinguish structure restoration from behavior explanation.

# Contract Checklist

- Public surface this module exposes: mechanical behavior evidence JSON fields and report/gate summary counts.
- Inputs this module accepts: joint limits, command profile, smoke steps, tolerance values, fixture-specific expectations.
- Outputs this module produces: joint limit checks, torque/velocity response, center of mass, contact state and step trace.
- Shared types/schemas/config touched: static manifest, smoke report, delivery gate, workflow contracts.
- Backward compatibility requirements: behavior evidence is additive and must not break existing `godot_verified` consumers until contract migration is documented.
- Integration tests required: fixture-based behavior summaries, mismatch classification tests, workflow contract tests.

# Local Context

Runtime restoration already compares node existence, classes, transforms, axes, mass/collision/mesh, fixed lock and parameters. This module adds behavior-level proof on top of that base.

# Non-Goals

- Do not build a full physics validation framework before the evidence schema is stable.
- Do not enforce new behavior metrics as release blocking until fixture expectations and tolerances are documented.

# Tasks

- [x] Define mechanical behavior evidence schema with units and tolerances.
- [x] Add joint limit readback/check evidence.
- [x] Add torque and velocity response evidence for controlled smoke steps.
- [x] Add center of mass and stability summary where runtime data is available.
- [x] Add contact state evidence for feet/support parts where available.
- [x] Add step-by-step motion trace with bounded size and artifact path support.
- [x] Add report/gate/workflow contract tests for behavior evidence fields.
- [x] Add structured threshold failure details for joint-limit diagnostics.
- [x] Exclude fixed joints from hinge-style joint-limit violation checks.

# Risks and Mitigations

- Risk: Godot physics readings vary by engine version or timestep.
  Mitigation: Start with tolerant summaries and record environment/timestep metadata before strict gates.

- Risk: Trace artifacts become too large.
  Mitigation: Bound inline trace size and write full traces as referenced artifacts.

# Validation

```powershell
py -3.12 -m py_compile tools\run_dynamic_godot_robot_smoke.py tools\build_dynamic_robot_generation_report.py agi_walker\core\api\robot_schema.py
py -3.12 -m pytest tests\test_dynamic_godot_robot_generation.py -q
py -3.12 -m pytest tests\test_workflow_contracts.py -q
```

# Completion Criteria

- Behavior evidence schema is documented and tested.
- `godot_verified` can include mechanical behavior proof without losing existing structure/parameter evidence.
- Missing runtime behavior data is reported as structured residual risk.

# Notes

- Coordinate with schema 1.5 before making actuator, sensor or joint-limit fields required.
- 2026-05-19: Added additive `dynamic_godot_mechanical_behavior_evidence.v1`
  to smoke reports and report summaries. It currently covers documented units,
  joint-limit evidence, torque/velocity-response evidence and bounded inline
  step trace. Center-of-mass and contact-state telemetry remain structured
  residual risks until runtime readback exists.
- 2026-05-19: Added opt-in `--mechanical-trace-output` support for full
  `dynamic_godot_mechanical_behavior_trace.v1` artifacts while keeping the
  default smoke report compact.
- 2026-05-19: Added center-of-mass evidence from runtime `body_states`
  `position`, `linear_velocity` and `mass`, including stability summary fields
  for COM height range, horizontal displacement and COM speed.
- 2026-05-19: Added contact-state evidence extraction when runtime telemetry
  explicitly reports contact fields such as `contact`, `contacts`,
  `contact_count`, `on_floor`, `ground_contact`, `support_contact` or
  `contact_points`; missing contact telemetry remains a structured residual
  risk.
- 2026-05-19: Enabled generated `RigidBody3D` contact monitoring and added
  `contact_count` plus collider names to generated controller `body_states`.
  Explicit zero-contact readings now count as available contact-state telemetry
  for live mechanical behavior evidence.
- 2026-05-19: Added additive report, delivery gate and workflow contract
  coverage for `mechanical_behavior_*` summary counts so behavior evidence
  presence, completeness, residual risks, threshold failures, COM/contact
  availability and trace artifact output are machine-readable without making
  them release blocking by default.
- 2026-05-21: Added `threshold_failure_details` and detailed joint-limit
  violation records with joint name, relative angle, lower/upper limits and
  margin values. This closes the prior behavior-evidence gap where live smoke
  could report a `joint_limit_violation` without enough diagnostic detail.
- 2026-05-21: Fixed false-positive joint-limit telemetry for fixed joints.
  `GeneratedRobotController._joint_limit_state` now returns `not_applicable`
  for `joint_type=fixed`, so fixed-lock validation remains covered by the
  fixed-lock gate while hinge-style angular limit checks only apply to movable
  joints. Re-running the mountain humanoid live Godot gate produced
  `mechanical_behavior_complete_count=1`,
  `mechanical_behavior_threshold_failure_count=0` and
  `mechanical_behavior_residual_risk_count=0`.

# Drift Check

Before implementation, verify behavior evidence still maps to `PROJECT_PLAN.md` contracts and does not silently redefine existing acceptance levels.
