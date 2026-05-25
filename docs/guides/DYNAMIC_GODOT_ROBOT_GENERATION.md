# Dynamic Godot Robot Generation

This guide describes the JSON-to-Godot path for generating a mechanical node tree
at runtime.

Future work is tracked in
[`DYNAMIC_GODOT_ROBOT_GENERATION_FUTURE_PLAN.md`](DYNAMIC_GODOT_ROBOT_GENERATION_FUTURE_PLAN.md).

## Flow

```text
RobotBuilder/template JSON
  -> normalize_robot_config_for_godot()
  -> Web Panel/session bridge load_robot
  -> godot_project/scripts/tcp_server.gd
  -> RobotAssembler.build_from_config()
  -> GeneratedRobotController
```

## Canonical JSON Fields

The runtime still accepts the existing `name`, `parts`, `connections`, and
`metadata` fields. For dynamic Godot assembly, each part and connection should
also carry explicit mechanical fields:

```json
{
  "name": "dynamic_biped",
  "parts": [
    {
      "id": "torso_1",
      "type": "torso",
      "shape": "box",
      "params": {
        "mass": 5.0,
        "size": [0.3, 0.2, 0.5],
        "position": [0.0, 1.2, 0.0],
        "rotation": [0.0, 0.0, 0.0]
      }
    }
  ],
  "connections": [
    {
      "name": "hip_left",
      "from": "torso_1",
      "to": "thigh_left",
      "joint_type": "hinge",
      "origin": [-0.15, 1.0, 0.0],
      "axis": [0.0, 0.0, 1.0],
      "limits": {"lower": -1.2, "upper": 1.2},
      "motor": {"enabled": true, "target_velocity": 0.0, "max_impulse": 120.0},
      "dynamics": {"damping": 0.1, "friction": 0.01}
    }
  ]
}
```

## Supported Runtime Mapping

Parts:

- `box` -> `RigidBody3D` + `BoxShape3D` + `BoxMesh`
- `capsule` -> `RigidBody3D` + `CapsuleShape3D` + `CapsuleMesh`
- `cylinder` -> `RigidBody3D` + `CylinderShape3D` + `CylinderMesh`
- `sphere` -> `RigidBody3D` + `SphereShape3D` + `SphereMesh`

Connections:

- `hinge` or `revolute` -> `HingeJoint3D`
- `slider` or `prismatic` -> `SliderJoint3D`
- `fixed` -> `Generic6DOFJoint3D` with locked linear and angular axes

Parameter mappings:

- Hinge limits are applied to `PARAM_LIMIT_LOWER`, `PARAM_LIMIT_UPPER`, and
  `FLAG_USE_LIMIT`.
- Hinge motors are applied to `FLAG_ENABLE_MOTOR`,
  `PARAM_MOTOR_TARGET_VELOCITY`, and `PARAM_MOTOR_MAX_IMPULSE`.
- `dynamics` is preserved in parameter readback diagnostics. Damping is reported
  as requested but not written to a Godot solver parameter until this project has
  a stable joint-type-specific mapping for the current Godot version.
- Slider limits are applied to `PARAM_LINEAR_LIMIT_LOWER` and
  `PARAM_LINEAR_LIMIT_UPPER`. Slider motor control is reported as unsupported
  until a stable motor mapping is added for that joint type.
- Fixed joints enable linear and angular limits on X/Y/Z and set every lower and
  upper limit to `0.0`. Runtime readback records `fixed_lock_applied`,
  per-axis linear/angular limit flags, and per-axis lower/upper values.
- `normalize_robot_config_for_godot()` normalizes fixed-joint `limits` to
  `{"lower": 0.0, "upper": 0.0}`, and the validator rejects non-zero fixed
  limits in Godot-ready configs.

The Python helper `normalize_robot_config_for_godot()` keeps older templates
usable by filling default shapes, positions, axes, limits, motors, and dynamics.
The validator `validate_godot_robot_config()` checks the assembly-ready shape.
The current normalized mechanical schema version is `1.5`; it rejects movable
`hinge`/`slider` joints with a zero axis, rejects limits where `lower > upper`,
requires `motor.max_impulse` to be non-negative, and requires any provided
`dynamics.damping` or `dynamics.friction` value to be non-negative. It also
rejects self-connections where a joint's `from` and `to` reference the same
part, duplicate child endpoints, and disconnected multi-part configs. Multi-part
robots must define exactly one root part so the Godot assembly can restore one
complete mechanical node tree. Every part must be reachable from that root, and
directed connection cycles are rejected before Godot launch.
`build_mechanical_topology_summary()` and
`tools/validate_robot_config_for_godot.py` expose the same topology as
machine-readable JSON, including `root_part`, `reachable_parts_count`,
`disconnected_parts`, `unreachable_parts`, `duplicate_child_endpoints`, `cycle`,
and `complete_tree`. The combined report builder also stores this object at
`static.topology_summary`, so static normalization reports carry the same
mechanical-tree evidence as the standalone validator.
`build_godot_node_tree_manifest()` also emits the expected Godot node tree before
Godot is launched. The standalone validator returns this as
`node_tree_manifest`, while the combined report stores it at
`static.node_tree_manifest`. The manifest records the robot node, generated
controller node, each planned `RigidBody3D`/collision/mesh path, each planned
joint node, endpoint paths, origin, axis, and the joint parameters expected to be
read back from Godot. It also includes `part_node_paths` and `joint_node_paths`
lookup maps keyed by source part id and connection name, so consumers can
restore JSON parts and connections to their planned Godot paths without scanning
the arrays. `path_maps_complete` records whether those lookup maps exactly match
the planned part and joint node paths. This gives CI a static audit artifact for
JSON-to-node-tree restoration even when live Godot smoke is skipped.
It also records `endpoint_paths_complete`, `missing_endpoint_part_ids`, and
`complete`, so a connection that references an undeclared part is visible in the
node-tree plan as well as in validation errors.
`validate_godot_node_tree_manifest()` checks the emitted manifest itself:
manifest identity fields, counts, `part_nodes`, `joint_nodes`, completion flags,
missing endpoint aggregates, planned Godot paths/classes, transforms, and
applied parameter objects. Manifest `part_id` and `connection_name` values must
also be unique so sidecar lookup maps cannot silently collapse duplicate nodes.
The manifest root is also self-consistent: `robot_node` must equal
`robot_name`, `controller_node` must be the generated controller under that
root, and planned part/joint paths must stay under the same root. Path-map
mismatch errors include the exact part or connection key and field whose planned
Godot path differs. The standalone
validator reports these checks as `node_tree_manifest_errors`, also emits
structured `node_tree_manifest_path_map_mismatches` entries with
`map`/`key`/`field`/`expected`/`actual`, and returns a non-zero exit code if
generated static manifest evidence is internally inconsistent.
Batch reports aggregate this static plan as `static_node_tree_manifest_count`,
`static_node_tree_manifest_valid_count`,
`static_node_tree_manifest_invalid_count`,
`static_node_tree_manifest_error_count`,
`static_node_tree_manifest_path_map_mismatch_count`,
`static_node_tree_manifest_path_map_mismatch_kind_counts`,
`static_node_tree_parts_planned_count`, `static_node_tree_joints_planned_count`,
`static_node_tree_parameterized_joints_count`,
`static_node_tree_complete_count`, `static_node_tree_incomplete_count`,
`static_node_tree_endpoint_paths_complete_count`,
`static_node_tree_endpoint_paths_incomplete_count`,
`static_node_tree_missing_endpoint_connections_count`,
`static_node_tree_missing_endpoint_parts_count`,
`static_node_tree_parameters_complete_count`, and
`static_node_tree_parameters_incomplete_count`. When standalone static manifests
are written, the gate also exposes `static_node_tree_manifest_output_count` for
CI assertions about archived manifest artifact coverage. The delivery acceptance gate
mirrors those scalar counts in `summary_counts`, so CI can enforce the static
node-tree plan with `--expect-summary-count` or `--expect-summary-value` before a
live Godot runner is available. `batch_summary` also includes
`static_node_tree_missing_endpoint_part_ids`, a deduplicated list of missing
endpoint IDs for direct triage, plus
`static_node_tree_missing_endpoint_connection_names` to identify the connections
that reference those missing endpoints. Each manifest and robot summary can also
include `static_node_tree_missing_endpoint_details`, whose entries name the
connection, endpoint field (`from` or `to`), and missing part id.

The dynamic Godot generation compatibility matrix is intentionally small and
explicit. `ROBOT_MECHANICAL_SCHEMA_VERSION` is `1.5`, while the normalizer
continues to accept robot config schema versions `1.1`, `1.2`, `1.3`, `1.4`,
and `1.5` for this generation path. The static Godot node-tree manifest
currently accepts `godot_node_tree_manifest.v1`. Code that needs to discover
this contract can call `godot_robot_generation_compatibility_matrix()` in
`agi_walker.core.api.robot_schema`; tests keep that matrix aligned with the
constants and docs.

Schema `1.5` is additive. Older configs do not need new fields, and the
normalizer preserves any provided optional metadata for downstream runtime
evidence. Part nodes may include optional `material` and `physics` objects.
Connections may include optional `actuator`, `sensor`, `sensors`, extended
`limits.effort`, `limits.velocity`, and `controller` tuning objects. These
fields are validated when present and are copied into the static node-tree
manifest as manifest-ready metadata, but they are not required for older
fixtures or release gates.

Negative regression fixtures live under `tests/fixtures/` for reusable contract
checks: `robot_dynamic_duplicate_part_ids.json`,
`robot_dynamic_duplicate_connection_names.json`, and
`robot_dynamic_root_drift.node_tree_manifest.json`. They cover duplicate part
ids, duplicate connection names, and manifest root drift without relying only on
inline test mutations.
Use `tools/build_dynamic_robot_generation_report.py --static-node-tree-manifest-dir`
to archive each static node-tree manifest as a separate JSON artifact while still
keeping the same data in the combined report. Each per-input report records the
written path as `static_node_tree_manifest_output`; batch robot summaries expose
the same path under `batch_summary.robots[*].static_node_tree_manifest_output`
for quick artifact lookup. Batch summaries also include
`static_node_tree_manifest_outputs` and
`static_node_tree_manifest_output_count` so CI can enumerate the standalone
manifest artifacts without walking each robot summary. Batch summaries and
per-robot summaries also expose
`static_node_tree_manifest_path_map_mismatches`, capped for preview, so a report
artifact can show whether drift came from missing paths, unexpected paths,
value mismatches, duplicate lookup keys, or root drift before the standalone
sidecar gate is run.
Add `--require-static-node-tree-manifest-output` when report generation itself
should fail unless every input writes a standalone static node-tree manifest
artifact. Use it with `--static-node-tree-manifest-dir`; if the directory option
is omitted, the delivery gate exits non-zero with reason code
`missing_static_node_tree_manifest_output`.
Add `--fail-on-static-node-tree-incomplete` when CI should also fail if the
static node-tree manifest itself is incomplete before live Godot smoke. This
sets `acceptance_requirements.static_node_tree_complete` and reports
`static_node_tree_incomplete` when the manifest has unresolved endpoint paths or
missing applied joint parameters.
It also validates `control.mode` and `control.joints`: each configured control
joint must match a normalized connection name, each entry must be an object, and
known controller gains such as `kp` and `kd` must be numeric. Invalid control
targets are rejected before Godot launch instead of being silently ignored by
the generated controller.
Web workflow artifact delivery calls the normalizer before sending a config to
legacy Godot or the session bridge, so existing workflow outputs can still be
loaded without rewriting the original artifact on disk.

Legacy compound `type: "leg"` parts are expanded during normalization when they
carry upper/lower dimensions such as `thigh_length` + `shin_length` or
`upper_length` + `lower_length`. One compound leg becomes:

- upper segment: `<leg_id>_upper`
- lower segment: `<leg_id>_lower`
- hip connection: original parent -> upper segment
- knee connection: upper segment -> lower segment

The upper and lower segment centers are inferred from the original hip
connection origin. If the hip is at `[x, y, z]`, the upper center is placed at
`[x, y - upper_length / 2, z]`, and the lower center is placed at
`[x, y - upper_length - lower_length / 2, z]`. This gives legacy compound
templates a concrete spatial body layout before Godot creates `RigidBody3D`
nodes.

This means the built-in `biped_basic` template becomes 5 bodies and 4 joints,
while `quadruped_dog` becomes 9 bodies and 8 joints for dynamic Godot loading.

## Runtime Control

`GeneratedRobotController` is attached under the generated robot root. It exposes:

- `apply_action(action)` for array or dictionary joint velocity targets
- `apply_instruction_steps(steps)` for instruction-console commands
- `reset_pose()`
- `get_sensor_data()`

When a generated joint has `motor.target_velocity` in JSON, the controller uses
that value as the initial joint target. Later `step` actions can override targets
by array order or by joint name dictionary.

Generated telemetry includes aggregate counts plus `body_states` and
`joint_states`. Body state entries report position, rotation, linear/angular
velocity, and mass for each generated `RigidBody3D`. Joint state entries report
the generated joint class, target velocity, endpoint paths, joint origin, axis,
estimated relative angle, endpoint distance, and per-endpoint body pose/velocity
metadata. They also include a `limits` object derived from the source JSON
connection limits, with lower/upper bounds, remaining margins, and a violation
flag. `applied_parameters` records both the source JSON parameter blocks and the
runtime parameters read back from the generated Godot joint, so smoke tests and
UI diagnostics can confirm that limits, motors, and damping were applied to the
actual node tree. The relative angle is derived from endpoint body orientation
projected onto the plane perpendicular to the generated joint axis, so it is a
runtime diagnostic estimate rather than a replacement for native constraint
solver state.

`tcp_server.gd` now tries dynamic assembly first when `load_robot` includes a
non-empty `parts` list. It falls back to legacy `load_from_dict()` only when the
assembler cannot be used.

The `load_robot` response and generated schema metadata include an
`assembly_summary` with:

- `parts_created` and `joints_created`
- `expected_parts`, `expected_joints`, `parts_complete`, `joints_complete`,
  `parameters_complete`, and overall `complete`
- `part_nodes`: source part id -> generated body, collision, and mesh node info
- `joint_nodes`: source connection -> generated joint node and endpoint paths
- `failed_connections` and `warnings`

This mapping is intended for Web Panel diagnostics and for verifying that a JSON
mechanical definition was materialized into the expected Godot node tree.
Workflow Godot delivery records persist the same data as `assembly_summary`,
plus a compact `assembly_mapping_summary` with part/joint mapping counts and
warning/failure counts for UI status surfaces. The compact summary also carries
expected part/joint counts plus `complete`, `parts_complete`,
`joints_complete`, and `parameters_complete`, so the Web UI can show restoration
status without requiring a headless smoke run.
The workflow Web UI renders this under the Godot delivery card as the dynamic
node-tree section, so a run detail can show whether the JSON parts and
connections became the expected Godot bodies, joints, and mappings.
`complete` is only true when expected parts, expected joints, and joint parameter
readback are all complete; parameter loss is therefore treated as an incomplete
mechanical restoration instead of a cosmetic warning. The smoke-free restoration
score uses the same three-part denominator, so missing joint parameter readback
also lowers the score instead of leaving a fully restored-looking value.
When live smoke data is available, the same section can also show endpoint
diagnostics such as maximum endpoint distance, average endpoint distance,
missing endpoint count, and the farthest joint. It can also show relative-angle
diagnostics such as maximum absolute angle, average absolute angle, configured
angle limit, and the largest-angle joint. When live joint limit telemetry is
available, the UI also shows configured limit count, violation count, minimum
remaining margin, and the worst-margin joint. Parameter application diagnostics
show how many generated joints have runtime parameter readback, applied limits,
motor support, enabled motors, configured dynamics, and applied damping.
Control diagnostics show how many joints currently have non-zero target velocity,
the maximum/average absolute target velocity, and the largest-target joint.
Multi-step smoke diagnostics also report how many steps ran, maximum and
average body displacement between the first and last sampled step, maximum final
linear speed, and the fastest body.
Optional motion gates can fail a smoke run when maximum displacement is below a
minimum threshold or final linear speed exceeds a maximum threshold.
Action-sequence diagnostics report how many actions were sent, how many unique
actions appeared, and how many transitions occurred between consecutive steps.
Mechanical restoration diagnostics combine part mapping, joint mapping, telemetry
coverage, parameter readback, and simulation sampling into
`mechanical_restoration_summary`. A complete restoration means every normalized
JSON part and connection has a Godot node mapping, telemetry is present for all
generated bodies and joints, all joints have parameter readback, and at least one
simulation step was sampled.

## Validation Tool

Use the repository tool to validate or materialize a Godot-ready copy of any
robot config:

```powershell
python tools/validate_robot_config_for_godot.py tests/fixtures/robot_dynamic_biped.json
python tools/validate_robot_config_for_godot.py configs/tutorial_01_biped.json --write-normalized test_env/tutorial_01_biped.godot.json
python tools/validate_robot_config_for_godot.py configs/tutorial_01_biped.json --write-node-tree-manifest test_env/tutorial_01_biped.node_tree_manifest.json
python tools/validate_robot_config_for_godot.py test_env/tutorial_01_biped.node_tree_manifest.json --input-kind node-tree-manifest
python tools/validate_robot_config_for_godot.py agi_walker/skills/robot-modeling/assets/templates/quadruped_dog.json --write-normalized test_env/quadruped_dog.godot.json
```

The tool reports part and connection counts, validation errors, and the optional
normalized output path. Use `--write-node-tree-manifest` when CI or a review
step needs a standalone static Godot node-tree manifest artifact. The command
still returns the same manifest on stdout as `node_tree_manifest`, and adds
`node_tree_manifest_output` when the artifact is written successfully. Use
`--input-kind node-tree-manifest` or `--input-kind auto` to validate an existing
standalone manifest sidecar directly; this returns `node_tree_manifest_errors`
without normalizing a robot config first.

## Headless Smoke

When Godot is available, run a live headless check that launches the demo scene,
sends `load_robot`, reads schema, and runs one `step`:

```powershell
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --output test_env/dynamic_godot_biped_smoke.json
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_quadruped.json --port 19091 --output test_env/dynamic_godot_quadruped_smoke.json
python tools/run_dynamic_godot_robot_smoke.py test_env/quadruped_dog.godot.json --port 19092 --output test_env/dynamic_godot_builtin_quadruped_smoke.json
python tools/run_dynamic_godot_robot_smoke.py test_env/biped_basic.godot.json --max-endpoint-distance 0.6 --output test_env/dynamic_godot_biped_smoke.json
python tools/run_dynamic_godot_robot_smoke.py test_env/biped_basic.godot.json --max-relative-angle 1.57 --output test_env/dynamic_godot_biped_angle_smoke.json
python tools/run_dynamic_godot_robot_smoke.py test_env/biped_basic.godot.json --fail-on-joint-limit-violation --output test_env/dynamic_godot_biped_limits_smoke.json
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --action-json "{\"hip_left\": 0.35, \"knee_left\": -0.2}" --output test_env/dynamic_godot_biped_action_smoke.json
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --action-json "{\"hip_left\": 0.35, \"knee_left\": -0.2}" --fail-on-action-target-mismatch --parameter-tolerance 0.0001 --output test_env/dynamic_godot_biped_action_target_smoke.json
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --action-json "{\"hip_left\": 0.35, \"knee_left\": -0.2}" --steps 5 --step-delay-seconds 0.02 --output test_env/dynamic_godot_biped_multistep_smoke.json
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --action-json "{\"hip_left\": 0.35, \"knee_left\": -0.2}" --steps 5 --min-body-displacement 0.01 --max-linear-speed 5.0 --output test_env/dynamic_godot_biped_motion_gate_smoke.json
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --action-sequence-json "[{\"hip_left\": 0.2, \"knee_left\": -0.1}, {\"hip_left\": -0.2, \"knee_left\": 0.1}]" --steps 4 --output test_env/dynamic_godot_biped_sequence_smoke.json
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --steps 3 --fail-on-incomplete-restoration --min-restoration-score 1.0 --output test_env/dynamic_godot_biped_restoration_gate_smoke.json
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --steps 3 --fail-on-parameter-mismatch --parameter-tolerance 0.0001 --output test_env/dynamic_godot_biped_parameter_consistency_smoke.json
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --steps 3 --fail-on-control-mismatch --parameter-tolerance 0.0001 --output test_env/dynamic_godot_biped_control_consistency_smoke.json
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --steps 3 --fail-on-full-mechanical-restoration --node-tree-tolerance 0.0001 --parameter-tolerance 0.0001 --output test_env/dynamic_godot_biped_full_mechanical_gate_smoke.json
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --steps 3 --fail-on-full-node-tree-restoration --node-tree-tolerance 0.0001 --output test_env/dynamic_godot_biped_full_node_tree_gate_smoke.json
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --steps 3 --fail-on-incomplete-node-tree --output test_env/dynamic_godot_biped_node_tree_gate_smoke.json
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --steps 3 --fail-on-node-tree-class-mismatch --output test_env/dynamic_godot_biped_node_tree_class_gate_smoke.json
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --steps 3 --fail-on-node-tree-missing-parameters --output test_env/dynamic_godot_biped_node_tree_parameter_gate_smoke.json
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --steps 3 --fail-on-node-tree-transform-mismatch --node-tree-tolerance 0.0001 --output test_env/dynamic_godot_biped_node_tree_transform_gate_smoke.json
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --steps 3 --fail-on-node-tree-physical-mismatch --node-tree-tolerance 0.0001 --output test_env/dynamic_godot_biped_node_tree_physical_gate_smoke.json
```

The report fails closed when Godot does not return the expected part count,
joint count, dynamic schema marker, mapping summary, body/joint telemetry, or
step telemetry. Joint telemetry also checks that both endpoint body states can
be resolved for each generated joint.
The smoke report summarizes endpoint distances in `joint_endpoint_summary` so
CI and the Web UI can flag stretched or unresolved joints without inspecting the
full per-joint payload.
The smoke report also emits `node_tree_manifest`, a source-to-runtime manifest
that lists every normalized JSON part and connection, whether it was restored,
its generated Godot node path/class, shape or joint class, transform, endpoint
paths, and missing part/connection names. The Web panel surfaces the manifest
completion state as `Tree Complete`, `Tree Parts`, `Tree Joints`, `Missing
Parts`, and `Missing Joints`.
The same smoke section now includes `static_manifest_version` and
`static_manifest_comparison`. That comparison checks the static
`godot_node_tree_manifest.v1` part/joint paths, classes, transforms, axes, mass,
collision parameters, mesh parameters and runtime-applied joint parameters
against the Godot load mappings, then reports
`complete`, `mismatch_count`, `mismatch_kind_counts`, and preview `mismatches`.
This is non-live-testable contract evidence; it does not make live Godot smoke a
default CI requirement.
Use `--fail-on-incomplete-node-tree` to fail when the manifest reports missing
parts or connections, even if lower-level load counts appear to match.
Use `--fail-on-full-node-tree-restoration` as the CI default when you need the
whole restored Godot node tree to match the JSON. It enables the node-tree
missing-node, class, missing-parameter, transform, physical-parameter, and
fixed-lock failure gates together while keeping the individual flags available
for narrower diagnostics. Smoke reports record the effective gate configuration
as `node_tree_gate_summary`, including `checks`, `enabled_checks`,
`enabled_count`, and `full_node_tree_restoration_required`.
Use `--fail-on-full-mechanical-restoration` when the run must prove the full
JSON-to-Godot mechanical restoration chain. It enables mechanical restoration
completeness, JSON joint parameter readback, `control.joints` readback, and the
full node-tree restoration gate together. Smoke reports record the effective
configuration as `mechanical_gate_summary`, including `checks`,
`enabled_checks`, `enabled_count`, and
`full_mechanical_restoration_required`.
Use `--fail-on-node-tree-class-mismatch` to fail when the generated Godot node
classes do not match the JSON shape or joint type, such as a capsule part not
producing `CapsuleShape3D`/`CapsuleMesh` or a slider connection not producing
`SliderJoint3D`. The manifest records expected and actual classes plus
`class_mismatch_count`.
Use `--fail-on-node-tree-missing-parameters` to fail when a restored joint does
not carry `applied_parameters`; the manifest records
`missing_parameter_connection_names`, `parameter_missing_count`, and
`parameters_complete`.
For fixed joints, the manifest also records `fixed_lock_checked_count`,
`fixed_lock_mismatch_count`, `fixed_lock_mismatches`, and
`fixed_locks_complete`; each joint entry includes `fixed_lock_checked`,
`fixed_lock_complete`, and `fixed_lock_mismatch_count`.
The Web panel renders these manifest-level smoke results as `Tree Fixed Locks`
and `Tree Fixed Mismatch`, with the first `fixed_lock_mismatches` records shown
next to the node-tree warnings.
Use `--fail-on-node-tree-fixed-lock-mismatch` to make the smoke runner fail
specifically when restored fixed-joint lock parameters are not complete.
The batch report also carries these manifest-level values as
`node_tree_fixed_lock_checked_count` and
`node_tree_fixed_lock_mismatch_count`, plus
`node_tree_fixed_locks_complete_count` and
`node_tree_fixed_locks_incomplete_count` for direct completion checks. The
`node_tree_mismatch_preview` includes the first `fixed_lock_mismatches` records
alongside class, transform, and physical mismatch previews.
Use `--fail-on-node-tree-transform-mismatch` to fail when generated part
`position`/`rotation` or joint `origin`/`axis` differs from the normalized JSON.
The manifest records `transform_mismatches`, `transform_mismatch_count`,
`transforms_complete`, and the selected `tolerance`; tune the tolerance with
`--node-tree-tolerance`.
Use `--fail-on-node-tree-physical-mismatch` to fail when generated part mass,
collision shape parameters, or mesh parameters differ from normalized JSON.
The manifest records expected physical parameters, runtime collision/mesh
parameters, `physical_mismatches`, `physical_mismatch_count`, and
`physical_complete`.
Use `--max-endpoint-distance` to make the smoke fail when any joint endpoint
distance exceeds a chosen threshold. The summary records the threshold and
`threshold_exceeded` status.
Use `--max-relative-angle` to fail when any absolute estimated joint angle
exceeds a chosen radian threshold. The smoke report records this in
`joint_angle_summary`.
Use `--fail-on-joint-limit-violation` to fail when any generated joint reports a
relative angle outside the lower/upper limits inherited from the JSON
connection. The smoke report records configured limit count, violation count,
minimum margin, and worst-margin joint in `joint_limit_summary`.
The smoke report also records `joint_parameter_summary`, which counts generated
joint parameter readback, applied limit parameters, motor-capable joints,
enabled motors, configured dynamics, and applied damping.
`joint_parameter_consistency_summary` compares source JSON joint parameters
against Godot runtime readback for limits, motor enablement, and motor impulse.
`motor.target_velocity` is reported in telemetry but is not treated as a static
consistency mismatch because actions can override it during a step.
For fixed joints, the same consistency summary also checks `fixed_lock_applied`
plus per-axis linear/angular limit flags and lower/upper values. The summary
reports `fixed_lock_checked_count` and `fixed_lock_mismatch_count`, and
`--fail-on-parameter-mismatch` fails if a fixed joint is not locked on every
axis.
Use `--parameter-tolerance` to tune the absolute numeric tolerance used by this
comparison; the selected tolerance is recorded in the summary and surfaced in
the Web workflow panel.
When mismatches exist, the summary includes the first mismatch records with the
joint name, field path, expected JSON value, and runtime value; the Web workflow
panel renders the first few records next to the mismatch count.
`mechanical_restoration_summary` reports restored part/joint counts, telemetry
counts, parameterized joint count, a 0-1 score, and a `complete` boolean for
quick acceptance checks.
Use `--fail-on-incomplete-restoration` to fail when `complete` is false, and
`--min-restoration-score` to fail when the restoration score drops below a chosen
threshold.
Use `--action-json` to send a custom action to the Godot `step` command. The
report records it as `action_sent` and summarizes target velocity propagation in
`joint_control_summary`.
Use `--min-nonzero-action-targets` to fail when the final runtime telemetry has
too few nonzero `target_velocity` joints. This catches smoke runs that only
perform static zero-action readback.
`action_target_consistency_summary` compares the last action payload against
runtime `joint_states[*].target_velocity`. Use
`--fail-on-action-target-mismatch` to fail when a commanded joint target was not
applied; it uses `--parameter-tolerance` for numeric comparisons.
`action_sequence_target_consistency_summary` compares every payload sent through
`--action-sequence-json` against the matching step telemetry. Use
`--fail-on-action-sequence-target-mismatch` to fail when any step in the command
sequence does not appear in `joint_states[*].target_velocity`.
Both action summaries also record `unknown_target_count` and `unknown_targets`
when a named action references a joint that was not generated, or a list action
contains more entries than available joints. Use
`--fail-on-unknown-action-target` to fail those cases explicitly.
They also record `invalid_target_count` and `invalid_targets` when an action
payload names an existing joint but gives a non-numeric target value. Use
`--fail-on-invalid-action-target` to fail malformed action values instead of
silently ignoring them.
`action_target_coverage_summary` reports how much of the generated joint set was
actually targeted by the action or action sequence. Use
`--min-action-target-coverage` with a 0-1 ratio to fail runs that only exercise a
small subset of generated joints.
`control_action_coverage_summary` applies the same idea only to JSON
`control.joints` entries that were generated in Godot. Use
`--min-control-action-coverage` to ensure configured control joints are exercised
by the action payloads.
`joint_control_consistency_summary` compares JSON `control.joints` entries,
such as `kp` and `kd`, against control metadata attached to generated Godot
joints. Use `--fail-on-control-mismatch` to make missing or mismatched control
parameters fail the smoke run; it uses `--parameter-tolerance` for numeric
comparisons. Report summaries expose `control_configured_count`,
`control_readback_checked_count`, and `control_readback_missing_count` so CI can
distinguish missing controller metadata from ordinary numeric mismatches.
Gate validation can make those counts blocking with
`--expect-control-configured-count`,
`--expect-control-readback-checked-count`,
`--expect-control-readback-missing-count`, or the shortcut
`--fail-on-control-readback-missing`. When that shortcut or the expected-count
flags are used in text mode, the summary includes
`control_readback=configured:N,checked:N,missing:N`. A Godot-verified delivery
acceptance gate is also incomplete when any configured control entry is missing
readback metadata, and the stable reason code is `control_readback_missing`.
Use `--steps` with optional `--step-delay-seconds` to run repeated step commands
and record a compact `simulation_summary` from the first and final sampled
telemetry.
Use `--min-body-displacement` and `--max-linear-speed` to turn
`simulation_summary` into a motion gate. The report records
`displacement_under_min` and `speed_threshold_exceeded`, and the smoke exits
non-zero when either gate fails.
Use `--min-joint-angle-delta` to verify the generated joints actually move
during the sampled simulation. This records `joint_motion_summary` with
`max_abs_relative_angle_delta`, `average_abs_relative_angle_delta`, and
`angle_delta_under_min`.
Use `--min-joint-angle-range` when oscillating joints may return close to their
initial angle; it checks the full sampled peak-to-peak `relative_angle` range
and records `max_abs_relative_angle_range` plus `angle_range_under_min`.
Use `--min-moving-joint-coverage` with `--joint-motion-epsilon` to require a
minimum ratio of measured joints whose angle range exceeds the epsilon. This
prevents a smoke run from passing only because one joint moved while the rest
stayed effectively static.
Use `--min-commanded-joint-response-coverage` to require the valid joints named
by action payloads to exceed the same motion epsilon, which catches a controller
that accepts commands but leaves some commanded joints static.
Use `--action-sequence-json` to send different actions across step commands.
When the sequence is shorter than `--steps`, the last action is repeated. The
report records `action_sequence_summary`, `first_action_sent`, and
`last_action_sent`.
Use `--min-action-transitions` to fail when adjacent action payloads do not
change enough times across the smoke sequence.
Use `--min-action-transition-delta` when the action payload must change by a
minimum numeric amount, not just differ structurally. The smoke records
`max_numeric_transition_delta`, `average_numeric_transition_delta`, and
`transition_delta_under_min`.
Use `--fail-on-incomplete-restoration` and `--min-restoration-score` to make the
mechanical restoration summary enforceable in CI.
Use `--fail-on-parameter-mismatch` to fail when Godot runtime readback does not
match source JSON joint parameters.

## Diagnostic Report

Use the combined report builder when you want one artifact that contains static
normalization, contract counts, optional normalized JSON, and optional live Godot
smoke output:

```powershell
python tools/build_dynamic_robot_generation_report.py agi_walker/skills/robot-modeling/assets/templates/biped_basic.json --output test_env/biped_dynamic_report.json
python tools/build_dynamic_robot_generation_report.py agi_walker/skills/robot-modeling/assets/templates/biped_basic.json --output test_env/biped_dynamic_report.json --static-node-tree-manifest-dir test_env/node_tree_manifests
python tools/build_dynamic_robot_generation_report.py test_env/biped_basic.godot.json --run-godot-smoke --port 19170 --output test_env/biped_dynamic_live_report.json
python tools/build_dynamic_robot_generation_report.py agi_walker/skills/robot-modeling/assets/templates/biped_basic.json agi_walker/skills/robot-modeling/assets/templates/quadruped_dog.json --output test_env/dynamic_template_batch_report.json
python tools/build_dynamic_robot_generation_report.py agi_walker/skills/robot-modeling/assets/templates/biped_basic.json agi_walker/skills/robot-modeling/assets/templates/quadruped_dog.json --run-godot-smoke --max-endpoint-distance 0.6 --output test_env/dynamic_template_batch_live_report.json
python tools/build_dynamic_robot_generation_report.py agi_walker/skills/robot-modeling/assets/templates/biped_basic.json --run-godot-smoke --max-relative-angle 1.57 --output test_env/biped_dynamic_angle_report.json
python tools/build_dynamic_robot_generation_report.py agi_walker/skills/robot-modeling/assets/templates/biped_basic.json --run-godot-smoke --fail-on-joint-limit-violation --output test_env/biped_dynamic_limit_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-json "{\"hip_left\": 0.35, \"knee_left\": -0.2}" --output test_env/biped_dynamic_action_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-json "{\"hip_left\": 0.35, \"knee_left\": -0.2}" --fail-on-action-target-mismatch --parameter-tolerance 0.0001 --output test_env/biped_dynamic_action_target_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-json "{\"hip_left\": 0.35, \"missing_joint\": 0.2}" --fail-on-unknown-action-target --output test_env/biped_dynamic_unknown_action_target_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-json "{\"hip_left\": \"fast\"}" --fail-on-invalid-action-target --output test_env/biped_dynamic_invalid_action_target_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-json "{\"hip_left\": 0.35}" --min-action-target-coverage 1.0 --output test_env/biped_dynamic_action_coverage_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-json "{\"hip_left\": 0.35}" --min-control-action-coverage 1.0 --output test_env/biped_dynamic_control_action_coverage_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-json "{\"hip_left\": 0.0, \"knee_left\": 0.0}" --min-nonzero-action-targets 1 --output test_env/biped_dynamic_nonzero_action_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-json "{\"hip_left\": 0.35, \"knee_left\": -0.2}" --steps 5 --step-delay-seconds 0.02 --output test_env/biped_dynamic_multistep_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-json "{\"hip_left\": 0.35, \"knee_left\": -0.2}" --steps 5 --min-body-displacement 0.01 --max-linear-speed 5.0 --output test_env/biped_dynamic_motion_gate_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-json "{\"hip_left\": 0.35, \"knee_left\": -0.2}" --steps 5 --min-joint-angle-delta 0.001 --output test_env/biped_dynamic_joint_motion_gate_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-sequence-json "[{\"hip_left\": 0.3}, {\"hip_left\": -0.3}, {\"hip_left\": 0.3}]" --steps 3 --min-joint-angle-range 0.001 --output test_env/biped_dynamic_joint_range_gate_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-json "{\"hip_left\": 0.35}" --steps 5 --joint-motion-epsilon 0.000001 --min-moving-joint-coverage 1.0 --output test_env/biped_dynamic_joint_motion_coverage_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-json "{\"hip_left\": 0.35, \"knee_left\": -0.2}" --steps 5 --joint-motion-epsilon 0.000001 --min-commanded-joint-response-coverage 1.0 --output test_env/biped_dynamic_commanded_response_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-sequence-json "[{\"hip_left\": 0.2, \"knee_left\": -0.1}, {\"hip_left\": -0.2, \"knee_left\": 0.1}]" --steps 4 --output test_env/biped_dynamic_sequence_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-sequence-json "[{\"hip_left\": 0.2}, {\"hip_left\": 0.2}]" --steps 2 --min-action-transitions 1 --output test_env/biped_dynamic_action_transition_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-sequence-json "[{\"hip_left\": 0.2}, {\"hip_left\": 0.201}]" --steps 2 --min-action-transition-delta 0.05 --output test_env/biped_dynamic_action_transition_delta_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --action-sequence-json "[{\"hip_left\": 0.2, \"knee_left\": -0.1}, {\"hip_left\": -0.2, \"knee_left\": 0.1}]" --steps 2 --fail-on-action-sequence-target-mismatch --parameter-tolerance 0.0001 --output test_env/biped_dynamic_sequence_target_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --steps 3 --fail-on-incomplete-restoration --min-restoration-score 1.0 --output test_env/biped_dynamic_restoration_gate_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --steps 3 --fail-on-parameter-mismatch --parameter-tolerance 0.0001 --output test_env/biped_dynamic_parameter_consistency_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --steps 3 --fail-on-control-mismatch --parameter-tolerance 0.0001 --output test_env/biped_dynamic_control_consistency_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --full-mechanical-restoration-acceptance --steps 3 --node-tree-tolerance 0.0001 --parameter-tolerance 0.0001 --output test_env/biped_dynamic_full_mechanical_acceptance_report.json --gate-output test_env/biped_dynamic_full_mechanical_acceptance_gate.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --steps 3 --fail-on-full-mechanical-restoration --node-tree-tolerance 0.0001 --parameter-tolerance 0.0001 --output test_env/biped_dynamic_full_mechanical_gate_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --steps 3 --fail-on-full-node-tree-restoration --node-tree-tolerance 0.0001 --output test_env/biped_dynamic_full_node_tree_gate_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --steps 3 --fail-on-incomplete-node-tree --output test_env/biped_dynamic_node_tree_gate_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --steps 3 --fail-on-node-tree-class-mismatch --output test_env/biped_dynamic_node_tree_class_gate_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --steps 3 --fail-on-node-tree-missing-parameters --output test_env/biped_dynamic_node_tree_parameter_gate_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --steps 3 --fail-on-node-tree-transform-mismatch --node-tree-tolerance 0.0001 --output test_env/biped_dynamic_node_tree_transform_gate_report.json
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --run-godot-smoke --steps 3 --fail-on-node-tree-physical-mismatch --node-tree-tolerance 0.0001 --output test_env/biped_dynamic_node_tree_physical_gate_report.json
```

Without `--run-godot-smoke`, the report does not launch Godot and only verifies
the Python normalization/validation contract. With `--run-godot-smoke`, the
report also includes live `load_robot`, schema, step telemetry, and mapping
counts from the generated Godot node tree.
The main report stores only smoke stdout/stderr line counts, tails, and a
compact `godot_smoke.report_summary`; the full live smoke payload stays in the
separate `--smoke-output` file referenced by `godot_smoke.report_path`.
The compact summary also includes `node_tree_mismatch_preview`, which keeps the
first class mismatches, transform mismatches, physical mismatches, and missing
parameterized joint names near the CI-friendly summary without duplicating the
entire smoke output.
`delivery_contract_preview` mirrors the delivery-facing status for both static
and live runs. In live smoke mode it carries the Godot load result's
`expected_parts`, `expected_joints`, `complete`, `parts_complete`,
`joints_complete`, `parameterized_joints`, and `parameters_complete` fields
alongside the created/mapped counts.
If validation fails, the report exits non-zero and `--run-godot-smoke` is skipped
with `godot_smoke_skipped_reason`, so invalid JSON does not launch Godot.
Use `--require-godot-verified-acceptance` in CI to return a non-zero exit code
unless every input reaches full Godot-verified delivery acceptance. Single-input
reports expose the same decision under `delivery_acceptance_summary`; batch
reports expose it under `batch_summary`.
Use `--require-full-mechanical-restoration-gate` when CI must also prove that
each input was run with the full mechanical restoration gate enabled. This
requirement is recorded on `delivery_acceptance_gate` as
`requires_full_mechanical_restoration_gate`; missing coverage adds the stable
reason code `missing_full_mechanical_restoration_gate`.
Use `--full-mechanical-restoration-acceptance` as the shortest CI entrypoint
for the complete JSON-to-Godot mechanical acceptance path. It enables
`--run-godot-smoke`, `--fail-on-full-mechanical-restoration`,
`--require-godot-verified-acceptance`, and
`--require-full-mechanical-restoration-gate`. Reports and gate artifacts mark
this preset with `acceptance_profile=full_mechanical_restoration`; hand-composed
CLI runs use `acceptance_profile=custom`, while Web direct-load gates use
`acceptance_profile=web_godot_load`.
When the preset starts a smoke run, the report tool clears the requested smoke
output path before launch and records `godot_smoke.report_written`; failed runs
therefore cannot pass by reusing a stale smoke JSON from an earlier successful
run. The delivery gate also records `smoke_report_written_count`,
`smoke_report_missing_count`, and `smoke_report_read_error_count`; a launched
smoke run that does not write a fresh report adds the stable reason code
`missing_godot_smoke_report`; a smoke run that writes unreadable JSON increments
`smoke_report_read_error_count` and adds the stable reason code
`invalid_godot_smoke_report`. Gate validation can make those counts blocking
with `--expect-smoke-report-written-count`,
`--expect-smoke-report-missing-count`, and
`--expect-smoke-report-read-error-count`. Use
`--fail-on-smoke-report-missing` and `--fail-on-smoke-report-read-error` as
shortcuts when any missing or unreadable smoke report should fail validation;
when enabled, text summaries include
`smoke_reports=written:N,missing:N,read_error:N` even for successful
single-input validations.
Use `--gate-output path/to/gate.json` to write only the compact
`delivery_acceptance_gate` object as a separate artifact. The gate artifact is
written before the process returns a non-zero gate exit code. The same flag works
for single-input and batch reports; the artifact contains only the top-level gate
object, not the static report, per-robot reports, or batch payload. Validation
failures also write this gate artifact, so CI can parse `reason_codes` and
`reason_details` even when Godot smoke was skipped before launch.
Use `python tools/validate_delivery_acceptance_gate.py path/to/gate-or-report.json`
to validate an existing gate artifact without re-running normalization or Godot
smoke. The validator accepts either a compact `--gate-output` artifact or a full
report containing top-level `delivery_acceptance_gate`; its JSON output includes
`gate_source_path` as `.` for compact gate files or `delivery_acceptance_gate`
for full reports. Each result also includes `complete_required_summary_fields`
and `complete_required_summary_fields_count`, derived from the gate `source` and
`verification_scope`, so CI can inspect the exact counters needed for a complete
delivery without printing the full schema first. Summary JSON output also
includes `complete_required_summary_fields_by_source_scope`,
`complete_required_summary_fields_source_scope_count`, and
`complete_required_summary_fields_source_scopes`, aggregating those
requirements across every validated gate while also exposing a stable
`source/scope` list for CI assertions. The validator exits `0` for a valid
gate shape and `1` when the artifact does not satisfy the shared
`delivery_acceptance_gate.v1` contract.
Use `python tools/validate_delivery_acceptance_gate.py --print-contract-schema`
to print `DELIVERY_ACCEPTANCE_GATE_SCHEMA` as JSON without supplying an input
artifact; CI can use this to discover allowed metadata, reason-code, requirement,
and summary-count fields before validating generated reports. The printed schema
also includes `complete_required_summary_fields_by_source_scope`, so external
Godot runners can derive the exact `summary_counts` keys required for a complete
CLI smoke-motion gate or Web Godot-load gate without duplicating Python constants.
It also exposes `summary_value_paths`, the complete set of scalar fields and
`map.key` paths accepted by `--expect-summary-value`, plus
`summary_value_paths_by_source` so CI can choose only the paths emitted by a
specific gate source.
Unreadable inputs, malformed JSON, and directory scans that do not find any
`*.json` files are also returned as structured `status: "error"` results with
an `errors` list, so CI can parse stdout without depending on Python tracebacks
or argparse stderr.
Add `--require-passed` when CI should also fail valid gate artifacts whose
`passed` field is not `true`. Add `--require-required` when release CI should
also fail valid gate artifacts whose own `required` field is not `true`, even if
the gate would otherwise pass as an informational artifact. Add
`--require-complete` when CI should reject gates whose `complete` field is not
`true`, even when they are valid and non-blocking. Add
`--require-full-mechanical-restoration-gate` when CI should reject gates whose
`requires_full_mechanical_restoration_gate` field is not `true`, proving the
complete mechanical restoration gate was actually required. Validator output
includes `required`, `complete`, `requires_full_mechanical_restoration_gate`,
`passed`, `reason_codes`, `reasons_count`, `enabled_requirements`, and compact
`summary_counts` so CI logs can show the active gate profile, failure class, and
affected scale without opening the full report. It also includes
`affected_inputs`, `affected_inputs_count`, and `affected_inputs_truncated`,
aggregated from `reason_details[].inputs`, so CI can show which robot JSON files
triggered the gate. Text detail lines include `full_mechanical_gate=true`,
`full_mechanical_gate=false`, or `full_mechanical_gate=unknown` for each input.
Pass multiple gate or report paths to validate them in one invocation. JSON mode
then returns aggregate `inputs_count`, `success_count`, `error_count`, and
per-input `results`; text mode emits one aggregate summary line followed by one
stable detail line per input. The aggregate summary also reports
`requires_full_mechanical_restoration_gate_true_count`,
`requires_full_mechanical_restoration_gate_false_count`, and
`requires_full_mechanical_restoration_gate_unknown_count`, so multi-robot CI can
confirm how many artifacts actually ran under the complete mechanical
restoration gate. Use `--expect-full-mechanical-gate-true-count`,
`--expect-full-mechanical-gate-false-count`, and
`--expect-full-mechanical-gate-unknown-count` to make those counts blocking CI
assertions. Summary artifacts also include `full_mechanical_gate_true_inputs`,
`full_mechanical_gate_false_inputs`, and `full_mechanical_gate_unknown_inputs`
so the same CI job can point to the exact reports that did or did not enable
the complete mechanical restoration gate. Use the matching
`--expect-full-mechanical-gate-*-input`, `--allow-full-mechanical-gate-*-input`,
and `--forbid-full-mechanical-gate-*-input` options when CI needs exact input
allowlists or blocklists for those three states. Use
`--fail-on-full-mechanical-gate-false` and
`--fail-on-full-mechanical-gate-unknown` as shortcuts when any false or unknown
coverage should fail the whole validation run. The validator records those
shortcuts as `fail_on_full_mechanical_gate_false` and
`fail_on_full_mechanical_gate_unknown` in summary/full JSON output and text
summaries when enabled, including successful single-input validations where no
false or unknown full-mechanical-gate inputs were found.
Add `--output path/to/validation.json` to write the full validation result as a
JSON artifact while keeping stdout in either JSON or text mode. If the full
output file cannot be written, the validator exits with `1` and reports
`unable to write output` in structured stdout aggregate errors.
Add `--summary-output path/to/gate-summary.json` when CI needs a compact
machine-readable file independent of stdout formatting. The summary file uses
`delivery_acceptance_gate_validation_summary.v1` and includes aggregate status,
input counts, skipped/error counts, aggregate errors, reason codes,
`sources`, `verification_scopes`, `acceptance_profiles`, `affected_inputs`,
`failed_inputs`, `skipped_inputs`, and `skipped_reasons`. If the summary file
cannot be written, the validator exits with `1` and reports
`unable to write summary output` in the structured stdout aggregate errors.
The shared contract schema exposes that value as `validation_summary_version`,
so CI can verify both gate artifacts and validation-summary artifacts from the
same schema printout. It also exposes `validation_summary_required_fields`,
covering the compact summary's stable identity, status, input counts, result
counts, and aggregate error list. Shared workflow contracts provide
`validate_delivery_acceptance_validation_summary(payload)` for tools that need
to validate compact validation-summary artifacts directly. The CLI can also
validate those artifacts with
`python tools/validate_delivery_acceptance_gate.py path/to/gate-summary.json --validate-validation-summary`;
use `--ignore-non-gate` in that mode when scanning directories that may contain
other JSON files. Summary-validation mode reports aggregate `summary_versions`
and `validation_summary_statuses` in summary/full JSON output so CI can confirm
which compact-summary versions and statuses were inspected. Use
`--expect-summary-version`, `--expect-validation-summary-status`,
`--expect-summary-versions-count`, `--expect-validation-summary-statuses-count`,
`--allow-summary-version`, `--allow-validation-summary-status`,
`--forbid-summary-version`, and `--forbid-validation-summary-status` to turn
those metadata checks into explicit CI failures. Summary/full JSON output records
the matching `expected_*`, `missing_expected_*`, `allowed_*`, `unexpected_*`,
`forbidden_*`, and `present_forbidden_*` audit lists for these summary metadata
constraints. The contract schema lists these
summary-validation aggregate fields under `validation_summary_metadata_fields`
and their JSON value shapes under `validation_summary_metadata_field_types`.
It lists the constraint audit fields under
`validation_summary_constraint_fields` and their JSON value shapes under
`validation_summary_constraint_field_types`, including nullable count
expectations and list-valued expected/missing/allowed/unexpected/forbidden
sets.
When present, `validate_delivery_acceptance_validation_summary(payload)` checks
those metadata previews, counts, truncation flags, nullable constraint counts,
and constraint audit lists for consistency.
The validator also runs this shared summary contract check on its own generated
compact summary before writing `--summary-output`, `--output-shape summary`, or
summary-only JSON stdout; a failure is reported as
`validation summary contract invalid`.
When both `--output` and `--summary-output` are provided, the primary output is
attempted first; the summary artifact includes `unable to write output` if that
artifact could not be written. By default `--output-shape auto` writes the full
validation result payload, but writes the same compact summary shape as JSON
stdout when `--summary-only` is set. Use `--output-shape full` to keep a full
result artifact while stdout is summary-only, or `--output-shape summary` to
write compact summary artifacts without changing stdout. Any explicitly supplied
`--output-shape` value requires `--output` and fails during argument parsing
when no output artifact path is provided. If the summary
artifact cannot be written after the primary output succeeds, the primary
output is rewritten with `unable to write summary output` in its aggregate
errors. The two output paths
must be different; using the same resolved path fails during argument parsing so
the compact summary cannot silently overwrite the full result. Output artifact
paths also cannot be explicit validation inputs. When an output artifact path is
inside a directory being scanned, that exact artifact path is excluded from the
expanded input list so stale validation artifacts do not pollute the next scan.
When exclusions happen, aggregate stdout and `--summary-output` include
`expanded_inputs_count`, `excluded_output_artifact_inputs`, and
`excluded_output_artifact_inputs_count`; text summaries include
`expanded_inputs=N` and `excluded_output_artifacts=N`.
Directory inputs are expanded as first-level `*.json`, so CI can point the
validator at an artifact directory instead of enumerating every gate file. Add
`--ignore-non-gate` when that directory also contains unrelated JSON metadata;
those files are reported as `skipped` and do not fail the aggregate validation
as long as at least one gate or full report is validated. If every input is
skipped, the validator fails closed with
`no delivery_acceptance_gate artifacts or reports found`; add `--allow-empty`
only for intentionally optional artifact scans. Add `--fail-on-skipped` when CI
should still skip non-gate JSON for reporting, but fail the aggregate result if
any skipped inputs were present.
Add `--fail-on-passed-false` when CI should fail if any structurally valid gate
reported `passed=false`, while preserving each input's own validation status.
Add `--fail-on-passed-unknown` when missing or non-boolean `passed` values
should fail the aggregate result even in optional scans. In text mode, either
flag also prints `passed=true:N,false:N,unknown:N` on the summary line.
Use `--show-passed-counts` with `--format text` when CI logs should show the
same `passed` state counts without adding a failure condition or count
expectation; for a single successful input this also emits the aggregate summary
line before the detail line.
Use `--show-diagnostics` with `--format text` when CI should emit one complete
diagnostic summary line without listing every individual display flag. It
combines passed-state counts, metadata, reason codes, input previews, and
skipped reasons, but does not add any failure condition.
Add `--summary-only` when CI should suppress per-input detail. In text mode it
keeps only the aggregate summary line; in JSON mode it prints the compact
validation summary instead of the full result payload. Combine it with
`--show-diagnostics` and `--format text` for a single-line delivery gate report
that still includes the main diagnostic previews.
Use `--preview-limit N` to control how many unique aggregate preview values are
included in JSON and text summaries. Counts still report the full unique totals,
and `*_truncated` flags tell CI when a preview list was shortened. Text
summaries print `preview_limit=N` so CI logs explain why preview lists are
shorter than their counts. This keeps large directory scans readable while
preserving machine-checkable totals. Expectations, allowlists, and forbidden
value checks still evaluate the full aggregate value set even when previews are
truncated or hidden with `--preview-limit 0`. JSON summaries also include
`previews_truncated` and `truncated_previews`, and text summaries print
`truncated_previews=...` when any aggregate preview list was shortened. Add
`--fail-on-truncated-previews` when CI should fail closed if any preview list is
shortened by the configured limit. Use `--expect-truncated-previews-count`,
`--expect-truncated-preview`, `--allow-truncated-preview`, and
`--forbid-truncated-preview` when CI should assert exactly how many and which
aggregate preview fields were shortened, for example to permit a large
`skipped_inputs` scan while forbidding truncation of `reason_codes` or release
metadata previews.
Use `--show-metadata` with `--format text` when CI logs should show the
aggregate `contract_versions`, `levels`, `sources`, `verification_scopes`,
`acceptance_profiles`, `enabled_requirements`, and
`complete_required_summary_fields` without adding a metadata failure condition.
It also prints `complete_required_summary_fields_source_scope_count` when those
requirements are present. Like `--show-passed-counts`, this emits the aggregate
summary line even for a single successful input.
Use `--show-reason-codes` with `--format text` when CI logs should show
aggregate `reason_codes` without adding a reason-code expectation or
allow/forbid condition; this also emits the aggregate summary line for a single
successful input.
Use `--show-inputs` with `--format text` when CI logs should show aggregate
`input_paths`, `affected_inputs`, `failed_inputs`, and `skipped_inputs`
previews without opening the JSON summary. This is useful for
artifact-directory scans where one summary line should point directly at the
generated report or ignored metadata file that needs attention. Use
`--expect-input`, `--allow-input`, and `--forbid-input` when CI should assert
the exact post-exclusion JSON files scanned by the gate before they are
classified as successful, invalid, skipped, or passed-state inputs. Use
`--expect-skipped-input`,
`--allow-skipped-input`, and `--forbid-skipped-input` when a directory scan
should only ignore a known set of metadata or sidecar JSON files. Use
`--expect-failed-input`, `--allow-failed-input`, and `--forbid-failed-input`
when CI should fail closed on any unexpected invalid or failing input file. Use
`--expect-affected-input`, `--allow-affected-input`, and
`--forbid-affected-input` when CI should assert which robot JSON inputs were
actually covered by the delivery gate. Use `--expect-passed-unknown-input`,
`--allow-passed-unknown-input`, and `--forbid-passed-unknown-input` when CI
should reject unexpected inputs whose `passed` state is missing or non-boolean.
Use `--expect-passed-true-input`, `--allow-passed-true-input`,
`--forbid-passed-true-input`, `--expect-passed-false-input`,
`--allow-passed-false-input`, and `--forbid-passed-false-input` when CI should
assert exactly which gate artifacts passed or explicitly failed.
Use `--show-skipped-reasons` with `--format text` when directory scans should
also show why ignored inputs were skipped, such as non-gate JSON metadata found
beside delivery artifacts.
Use `--expect-expanded-inputs-count N`,
`--expect-excluded-output-artifacts-count N`, `--expect-inputs-count N`,
`--expect-success-count N`, `--expect-error-count N`, or
`--expect-skipped-count N` when CI knows how many artifacts should be scanned,
excluded, and land in each aggregate status.
Use repeatable `--expect-summary-count FIELD=N` when CI needs to assert any
scalar `delivery_acceptance_gate.summary_counts` field that is part of the
shared contract, including newer motion, coverage, restoration, readback, or
provenance counters. Unknown fields are rejected during argument parsing so a
typo cannot silently become an always-zero assertion. Summary JSON includes
`expected_summary_counts`, `actual_expected_summary_counts`, and
`mismatched_expected_summary_counts`; text summaries print
`expected_summary_counts=field:N` and
`mismatched_expected_summary_counts=field:expected/actual`.
For static JSON-to-node-tree CI, useful assertions include
`--expect-summary-count static_node_tree_manifest_count=1`,
`--expect-summary-count static_node_tree_manifest_output_count=1`,
`--expect-summary-count static_node_tree_parts_planned_count=5`,
`--expect-summary-count static_node_tree_joints_planned_count=4`,
`--expect-summary-count static_node_tree_missing_endpoint_connections_count=0`,
and
`--expect-summary-count static_node_tree_parameters_complete_count=1`.
For example, generate a report with `--static-node-tree-manifest-dir`, write a
gate artifact with `--gate-output`, then run
`tools/validate_delivery_acceptance_gate.py --expect-summary-count static_node_tree_manifest_output_count=1`
against that gate artifact to prove the standalone node-tree manifest was
archived.
When the gate validator scans an artifact directory, JSON files whose
`manifest_version` is `godot_node_tree_manifest.v1` are treated as skipped
sidecar artifacts with skip reason `static Godot node-tree manifest sidecar`, so
co-locating gate, report, and manifest outputs does not make the directory scan
fail. Use `--expect-skipped-count`,
`--expect-skipped-input <path-to-manifest>`, and
`--expect-skipped-reason "static Godot node-tree manifest sidecar"` when CI
should also prove that the sidecar manifest was present in the scanned artifact
directory. The skipped input result includes `node_tree_manifest_summary` with
the manifest version, robot name, planned part/joint counts, completion flags,
lookup-map counts, `path_maps_complete`, and missing endpoint counts for quick
triage without reopening the sidecar JSON.
Directory validation summaries also aggregate these sidecars as
`node_tree_manifest_sidecar_count`,
`node_tree_manifest_sidecar_complete_count`,
`node_tree_manifest_sidecar_incomplete_count`,
`node_tree_manifest_sidecar_valid_count`,
`node_tree_manifest_sidecar_invalid_count`,
`node_tree_manifest_sidecar_validation_error_count`,
`node_tree_manifest_sidecar_path_incomplete_count`,
`node_tree_manifest_sidecar_path_map_mismatch_count`,
`node_tree_manifest_sidecar_parts_planned_count`,
`node_tree_manifest_sidecar_joints_planned_count`,
`node_tree_manifest_sidecar_part_path_count`,
`node_tree_manifest_sidecar_joint_path_count`, and
`node_tree_manifest_sidecars`. These fields are part of the validation summary
schema metadata, so a written summary artifact can be validated again and will
fail if the sidecar counts or preview shape are malformed.
With `--format text --show-diagnostics`, the validator prints the same aggregate
as
`node_tree_sidecars=count:N,complete:N,incomplete:N,valid:N,invalid:N,validation_errors:N,path_incomplete:N,path_mismatches:N,parts:N,joints:N,part_paths:N,joint_paths:N`.
Newer summaries include the same legacy fields plus
`path_mismatch_kinds:KIND:N+...` between `path_mismatches` and `parts`.
Written validation summaries also check that aggregate and preview path counts
do not exceed their planned part/joint counts, catching malformed summary
artifacts during a second validation pass. When the sidecar preview is not
truncated, the aggregate complete/valid/planned/path counts must also equal the
sum of the preview entries, and `path_incomplete` must equal the number of
preview entries whose `path_maps_complete` field is `false` when present, or
whose path-map counts are below their planned part/joint counts otherwise.
For manifest sidecars, `path_incomplete` follows the manifest's own
`path_maps_complete=false` flag when present; a sidecar that claims
`path_maps_complete=true` while its maps do not exactly match planned nodes is
reported through `invalid` by structural self-validation. Skipped sidecar
results include full `node_tree_manifest_path_map_mismatches`; aggregate
`node_tree_manifest_sidecars` previews include the mismatch count and truncated
preview entries for quick triage. The top-level
`node_tree_manifest_sidecar_path_map_mismatch_count` sums all sidecar mismatch
counts so CI can assert zero map drift without reading every preview entry.
Validation summary self-checks require these preview entries to be objects with
`map`, `key`, and `kind`, require the mismatch count to cover the preview
length, and require the top-level mismatch count to equal the preview sum when
the sidecar preview is not truncated. Mismatch `kind` values include
`missing`, `unexpected`, `value_mismatch`, `duplicate`, and `root_mismatch`;
duplicate entries flag repeated planned lookup keys such as `part_id` or
`connection_name` before they can collapse the generated path maps.
`root_mismatch` entries flag planned part/joint node paths that are internally
consistent with the sidecar maps but no longer live under the manifest's
declared `robot_node`. The gate summary also emits
`node_tree_manifest_sidecar_path_map_mismatch_kind_counts`, an exact aggregate
count by mismatch kind, so CI logs can distinguish missing paths from duplicate
keys or root drift without expanding every sidecar preview.
Use `--fail-on-node-tree-manifest-sidecar-incomplete` when CI should fail if
any scanned static Godot node-tree manifest sidecar reports `complete=false`.
Use `--fail-on-invalid-node-tree-manifest-sidecar` when CI should fail if any
sidecar manifest fails structural self-validation even though it is still
skipped as a sidecar artifact during normal directory scans.
Use `--fail-on-node-tree-manifest-sidecar-validation-error` when CI should fail
on the exact aggregate self-validation error count rather than only the invalid
sidecar count.
Use `--fail-on-node-tree-manifest-sidecar-path-incomplete` when CI should fail if
a static manifest sidecar does not provide part/joint path maps for every
planned part/joint.
Use `--fail-on-node-tree-manifest-sidecar-path-map-mismatch` when CI should fail
if a static manifest sidecar includes lookup paths that do not match the planned
Godot node tree. These fail-closed switches are also exposed as validation
summary policy booleans, and validation summary self-checks reject malformed
non-boolean policy values.
Use `--expect-node-tree-manifest-sidecar-path-map-mismatch-count 0` when CI
should assert the exact aggregate mismatch count while still keeping the default
sidecar skip behavior. The expected value is persisted as a validation summary
constraint field and must be a non-negative integer or `null` during summary
self-validation.
The same exact-count pattern is available for static sidecar discovery and
state counters with
`--expect-node-tree-manifest-sidecar-count`,
`--expect-node-tree-manifest-sidecar-complete-count`,
`--expect-node-tree-manifest-sidecar-incomplete-count`,
`--expect-node-tree-manifest-sidecar-valid-count`,
`--expect-node-tree-manifest-sidecar-invalid-count`,
`--expect-node-tree-manifest-sidecar-validation-error-count`, and
`--expect-node-tree-manifest-sidecar-path-incomplete-count`. Planned and
resolved path totals can be asserted with
`--expect-node-tree-manifest-sidecar-parts-planned-count`,
`--expect-node-tree-manifest-sidecar-joints-planned-count`,
`--expect-node-tree-manifest-sidecar-part-path-count`, and
`--expect-node-tree-manifest-sidecar-joint-path-count`.
Use repeatable
`--expect-node-tree-manifest-sidecar-path-map-mismatch-kind KIND=N` to assert
exact per-kind mismatch totals, for example `root_mismatch=0` in CI profiles
that should fail on any root drift while still preserving the default sidecar
skip behavior.
The CI workflow runs the same static loop before live Godot smoke: generate
standalone sidecars from known fixtures, then scan the sidecar directory with
fail-closed manifest validation and exact zero-drift assertions. The equivalent
local command is:

```powershell
python tools/build_static_godot_node_tree_evidence.py tests/fixtures/robot_dynamic_fixed_pair.json tests/fixtures/robot_dynamic_biped.json tests/fixtures/robot_dynamic_quadruped.json --output-root test_env/static_godot_node_tree_manifest_ci --manifest-dir test_env/static_godot_node_tree_manifests
```

The command writes `report.json`, `gate.json`,
`validation_summary.json`, `static_godot_node_tree_evidence_closeout.json`,
and the standalone manifest sidecars. The closeout report records
`live_godot_smoke_run=false` so static evidence is not mistaken for runtime
motion verification.
The golden static fixture set is fixed pair, biped, and quadruped. Those three
inputs cover a locked fixed joint, a two-leg articulated topology, and a
four-leg articulated topology before any optional live Godot smoke is run.

Use `tools/build_dynamic_godot_release_readiness.py` to summarize existing
evidence artifacts without rerunning Godot:

```powershell
python tools/build_dynamic_godot_release_readiness.py test_env/static_godot_node_tree_manifest_ci/static_godot_node_tree_evidence_closeout.json --output test_env/dynamic_godot_release_readiness.json
```

The summary artifact uses
`dynamic_godot_release_readiness_summary.v1` and reports `proven_level` as the
strongest supplied evidence level: `static_only`, `godot_load_verified`, or
`godot_verified`. It accepts static closeout JSON, compact
`delivery_acceptance_gate.v1` artifacts, reports containing
`delivery_acceptance_gate`, and Web delivery records containing the same gate.
If only static evidence is supplied, the summary remains ready at `static_only`
and records residual risk for missing Web/session load and full live motion
evidence. A complete Web `godot_load` gate raises the level to
`godot_load_verified`; a complete smoke-motion gate raises it to
`godot_verified`.

Acceptance levels and proof commands:

| Level | What it proves | Command that proves it | Required evidence fields |
| --- | --- | --- | --- |
| `static_only` | JSON normalization, static `godot_node_tree_manifest.v1`, standalone sidecars, strict sidecar validation, and closeout evidence are complete for the golden fixture set. | `python tools/build_static_godot_node_tree_evidence.py tests/fixtures/robot_dynamic_fixed_pair.json tests/fixtures/robot_dynamic_biped.json tests/fixtures/robot_dynamic_quadruped.json --output-root test_env/static_godot_node_tree_manifest_ci --manifest-dir test_env/static_godot_node_tree_manifests` then `python tools/build_dynamic_godot_release_readiness.py test_env/static_godot_node_tree_manifest_ci/static_godot_node_tree_evidence_closeout.json --output test_env/dynamic_godot_release_readiness.json` | `static_godot_node_tree_evidence_closeout.json.status=success`, `acceptance_level=static_only`, `live_godot_smoke_run=false`, readiness `proven_level=static_only`, `proven_level_rank=1`. |
| `godot_load_verified` | A Web/session Godot load gate accepted the artifact and returned a complete Godot load assembly summary, without proving smoke-motion behavior. | Load a Godot-loadable workflow artifact through `/api/workflows/runs/{run_id}/artifacts/{artifact_index}/godot-load` or `/api/workflows/runs/{run_id}/godot-sync`, archive the returned Web delivery record, then run `python tools/build_dynamic_godot_release_readiness.py <web_delivery_record.json> --output test_env/dynamic_godot_release_readiness_load.json`. | Web delivery `delivery_acceptance_gate.source=web_godot_delivery`, `verification_scope=godot_load`, `level=godot_load_verified`, `passed=true`, `complete=true`, readiness `proven_level=godot_load_verified`, `proven_level_rank=2`. |
| `godot_verified` | Full live Godot smoke-motion acceptance is proven, including static-vs-runtime node-tree comparison, restoration, parameter readback, fixed-lock checks, and motion gate evidence. | `python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --full-mechanical-restoration-acceptance --steps 3 --node-tree-tolerance 0.0001 --parameter-tolerance 0.0001 --smoke-output test_env/dynamic_godot_biped_live_smoke.json --output test_env/dynamic_godot_biped_live_report.json --gate-output test_env/dynamic_godot_biped_live_gate.json` then `python tools/build_dynamic_godot_release_readiness.py test_env/dynamic_godot_biped_live_gate.json --output test_env/dynamic_godot_release_readiness_live.json`. | Gate `source=dynamic_godot_generation_report`, `verification_scope=godot_smoke_motion`, `level=godot_verified`, `passed=true`, `complete=true`, readiness `proven_level=godot_verified`, `proven_level_rank=3`. |

`incomplete` is not an acceptance level. It means the supplied evidence was
missing, malformed, blocked, or weaker than the requested proof level.

Use `tools/build_dynamic_godot_release_evidence_bundle.py` to package the
handoff artifacts after readiness is built:

```powershell
python tools/build_dynamic_godot_release_evidence_bundle.py --static-closeout test_env/static_godot_node_tree_manifest_ci/static_godot_node_tree_evidence_closeout.json --delivery-gate test_env/static_godot_node_tree_manifest_ci/gate.json --readiness-summary test_env/dynamic_godot_release_readiness.json --output-root test_env/dynamic_godot_release_evidence_bundle
python tools/validate_dynamic_godot_release_evidence_bundle.py test_env/dynamic_godot_release_evidence_bundle/bundle_index.json --output test_env/dynamic_godot_release_evidence_bundle/bundle_validation.json
```

The bundle index uses `dynamic_godot_release_evidence_bundle.v1`. It copies the
static closeout, delivery gate, readiness summary, optional live smoke, optional
Web delivery record, and documentation index entries into one directory. Each
artifact and documentation entry records its source path, bundle path, byte size,
SHA-256 checksum, `source_modified_at`, and `bundle_modified_at`. Artifact keys
and documentation roles must be unique. The `required` flag must match the
canonical required artifact key and documentation role lists. Bundle paths must
be non-empty paths relative to the bundle root and may not resolve outside that
root, so the delivered directory remains self-contained. Static-only bundles are
valid when the readiness summary proves `static_only`; stronger bundles can add
`--live-smoke` and `--web-delivery-record` while preserving the same readiness
semantics. The
self-validator writes
`dynamic_godot_release_evidence_bundle_validation.v1` and fails if required
artifacts are missing, checksums drift, readiness status is not `ready`, or the
bundle level does not match `readiness_summary.proven_level`. It also checks
timestamp shape, verifies `bundle_modified_at` against the bundled file mtime,
and checks index `bundle_root`, timezone-aware `generated_at`,
`readiness_status`, and `residual_risks` against the current bundle and
readiness summary.
Final bundle validation requires `validation_report` to point to an in-bundle
validation report and checks that both the index `validation_status` and the
stored report status match the current validation result. It also checks that
the stored validation report is internally status/error consistent and that its
bundle root, required artifact list, required documentation role list, artifact
count, documentation count and evidence level still match the current bundle
index.
Bundled delivery gates, including gates inside Web delivery records, are
validated with the shared `delivery_acceptance_gate.v1` contract.
When `--live-smoke`
is included, validation also checks that the smoke artifact carries
`dynamic_godot_live_verification_profile.v1` metadata and internally consistent
wrapper retry fields such as `wrapper_attempts`, `wrapper_attempt_count`, and
`attempts_recorded`. When `--web-delivery-record` is included, validation checks
that the record carries a `web_godot_delivery` / `godot_load` acceptance gate and
consistent static node-tree manifest evidence.

Manual live smoke checklist, when a Godot executable is available:

```powershell
python tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json --full-mechanical-restoration-acceptance --live-profile local --live-artifact-root test_env/dynamic_godot_live/local --steps 3 --node-tree-tolerance 0.0001 --parameter-tolerance 0.0001 --smoke-output test_env/dynamic_godot_biped_live_smoke.json --output test_env/dynamic_godot_biped_live_report.json --gate-output test_env/dynamic_godot_biped_live_gate.json
python tools/build_dynamic_godot_release_readiness.py test_env/dynamic_godot_biped_live_gate.json --output test_env/dynamic_godot_release_readiness_live.json
```

Before running a live profile on a new machine, use the non-launching discovery
check:

```powershell
python tools/run_dynamic_godot_robot_smoke.py tests/fixtures/robot_dynamic_biped.json --dry-run-discovery --live-profile local --live-artifact-root test_env/dynamic_godot_live/local --output test_env/dynamic_godot_live/local/discovery.json
```

The discovery report and every live smoke report include
`live_verification.profile_version=dynamic_godot_live_verification_profile.v1`,
`profile_name`, `environment_mode`, `godot_executable`, `failure_category`,
`artifact_retention`, and `flaky_policy`. Missing executables are classified as
`missing_godot_executable` and produce a structured smoke report instead of
depending on a stale or absent output file. Launch failures and TCP startup
timeouts are also written as structured smoke reports with
`failure_category=godot_launch_failure` or `godot_tcp_timeout` and a
`failure_detail` object; other unexpected command/runtime failures use
`godot_runtime_failure`.

Live smoke reports also include additive
`mechanical_behavior_evidence.evidence_version=dynamic_godot_mechanical_behavior_evidence.v1`.
This section records documented units, joint-limit evidence,
torque/velocity-response evidence, and a bounded inline step-by-step trace. Use
`--mechanical-trace-output path/to/trace.json` on
`tools/run_dynamic_godot_robot_smoke.py` or
`tools/build_dynamic_robot_generation_report.py` to write the full trace as a
separate `dynamic_godot_mechanical_behavior_trace.v1` artifact and record its
path in `step_trace_evidence.artifact_path`. When `body_states` include
`position`, `linear_velocity`, and `mass`, the same section computes
`center_of_mass_evidence` from mass-weighted runtime telemetry and includes a
stability summary with COM height range, horizontal displacement, and COM speed.
When `body_states` or top-level step telemetry include explicit contact fields
such as `contact`, `contacts`, `contact_count`, `on_floor`, `ground_contact`,
`support_contact`, or `contact_points`, the report also fills
`contact_state_evidence` with support parts and current contacts. Generated
Godot bodies enable contact monitoring and report `contact_count` plus collider
names in `body_states`, so an explicit zero-contact reading is treated as
available telemetry, not as missing evidence. If support or contact telemetry is
not present, contact state remains a structured residual risk:
`contact_state_runtime_readback_missing`. If body telemetry is unavailable or
incomplete, center of mass is also reported as
`center_of_mass_runtime_readback_missing`. Existing `godot_verified` gates keep
their current compatibility while the behavior contract becomes inspectable.

CI also exposes an opt-in `dynamic-godot-live-verification` job on
`workflow_dispatch` and `schedule`. It never runs on default pull-request CI.
The job always uploads `dynamic-godot-live-verification-artifacts` containing
the discovery report. If `GODOT_EXECUTABLE` is configured and points to an
existing executable, the same job runs the full `godot_verified` report/gate
chain and writes the live smoke, report, gate, and readiness artifacts under
`test_env/dynamic_godot_live/<profile>/`. Scheduled runs use the `scheduled_ci`
profile; manual runs use `manual_ci`. The report builder auto-selects a free
localhost TCP port by default for each live smoke invocation and records
`godot_smoke.attempts`; if the first auto-port launch fails with
`godot_tcp_timeout`, or fails before writing a smoke report with
`Godot TCP server did not respond on port`, it retries once with a newly
selected port. Pass `--port <port>` only when a fixed port is required for a
local diagnostic session; fixed-port runs do not auto-retry. Report-level live
evidence updates `godot_smoke.live_verification.flaky_policy.attempts_recorded`,
and, when a wrapper retry occurs, `max_attempts` and `classification` so a
successful retry is visible as `passed_after_retry` instead of `not_retried`.
After a wrapper retry, the final retained smoke JSON is synchronized with the
same `live_verification.flaky_policy` fields so archived smoke artifacts and
the generation report do not disagree. The retained smoke JSON also records
`wrapper_attempt_count`, `wrapper_retried`, and `wrapper_attempts` under
`live_verification.flaky_policy` so retry diagnostics remain available when the
smoke artifact is inspected by itself.

Archive these exact artifacts:

- `test_env/dynamic_godot_biped_live_report.json`
- `test_env/dynamic_godot_biped_live_gate.json`
- `test_env/dynamic_godot_biped_live_smoke.json`
- `test_env/dynamic_godot_release_readiness_live.json`

The live report must show `delivery_acceptance_gate.level=godot_verified`,
`delivery_acceptance_gate.passed=true`, `delivery_acceptance_gate.complete=true`,
and `delivery_acceptance_gate.summary_counts.live_smoke_count=1`. The smoke
report must include `node_tree_manifest.static_manifest_version`,
`node_tree_manifest.static_manifest_comparison.complete=true`, and
`node_tree_manifest.static_manifest_comparison.mismatch_count=0`. The readiness
summary must show `summary_version=dynamic_godot_release_readiness_summary.v1`,
`status=ready`, `proven_level=godot_verified`, and
`proven_level_rank=3`. If any of those fields is absent or weaker, archive the
artifacts as diagnostic evidence and keep the release at the strongest lower
level proven by `tools/build_dynamic_godot_release_readiness.py`.

Use repeatable `--expect-summary-value PATH=N` when CI wants one unified syntax
for scalar counts and check-map counts. `PATH` may be a scalar field such as
`delivery_static_only_count` or a map path such as
`node_tree_gate_check_counts.fixed_lock_mismatch`. Summary JSON includes
`expected_summary_values`, `actual_expected_summary_values`, and
`mismatched_expected_summary_values`; text summaries print the same paths as
`expected_summary_values=path:N`.
Add repeatable `--expect-summary-value-source SOURCE` when CI wants those
`PATH` assertions checked against a specific gate source before validation runs.
This catches source-scope mistakes such as using CLI-only smoke readback
counters for `web_godot_delivery`. Summary JSON and full-output JSON record the
selected source filters under `expected_summary_value_sources`; text summaries
print `expected_summary_value_sources=source`. When at least one gate is
validated, the same source must appear in the aggregate gate sources; missing
matches are reported as `missing_expected_summary_value_sources`. When source
filters are present, `actual_expected_summary_values` is calculated only from
matching gate sources, so mixed CLI/Web artifact directories cannot cross-feed
the same summary path. The summary also reports
`summary_value_source_matched_count`, `summary_value_source_excluded_count`, and
`summary_value_source_excluded_sources` to make the filtered aggregation scope
auditable. Use `--expect-summary-value-source-matched-count N` and
`--expect-summary-value-source-excluded-count N` when CI needs to fail unless
the source-filtered aggregation covered exactly the intended number of gates.
Use repeatable `--expect-summary-value-source-excluded-source SOURCE` when CI
also needs to prove a specific source was excluded from that aggregation. Use
`--allow-summary-value-source-excluded-source SOURCE` and
`--forbid-summary-value-source-excluded-source SOURCE` to constrain that
excluded-source set. Use `--allow-summary-value-source-matched-source SOURCE`
and `--forbid-summary-value-source-matched-source SOURCE` to constrain the
sources that are allowed to enter the filtered aggregation. These matched and
excluded source constraints require at least one
`--expect-summary-value-source`, so CI cannot accidentally apply them to an
unfiltered summary-value aggregation. The contract schema exposes the related
summary/full-output fields under `summary_value_source_filter_fields` for CI
generators that need to discover those audit keys, and
`summary_value_source_filter_field_types` records whether each audit key is a
list, count, nullable count, or object payload.
Use `--expect-passed-true-count N`, `--expect-passed-false-count N`, and
`--expect-passed-unknown-count N` when CI also needs to distinguish gates that
are structurally valid from gates whose own `passed` field is true, false, or
missing/non-boolean. This is useful when a pipeline validates artifacts without
using `--require-passed`, but still wants to prove how many delivery gates
actually accepted the generated Godot mechanical restoration.
The compact summary also includes `passed_true_inputs`, `passed_false_inputs`,
and `passed_unknown_inputs` with matching counts and truncation flags, so CI can
link a failed aggregate directly to the gate artifact paths that reported each
`passed` state.
These count arguments must be non-negative integers; invalid values fail during
argument parsing with `value must be an integer >= 0`.
Count mismatches fail the aggregate result with
`expected expanded_inputs_count ...`,
`expected excluded_output_artifacts_count ...`, `expected inputs_count ...`,
`expected success_count ...`, `expected error_count ...`,
`expected passed_true_count ...`, `expected passed_false_count ...`,
`expected passed_unknown_count ...`, `expected skipped_count ...`, or
`expected reason_codes_count ...`, `expected contract_versions_count ...`,
`expected levels_count ...`, `expected sources_count ...`,
`expected verification_scopes_count ...`,
`expected acceptance_profiles_count ...`, or
`expected enabled_requirements_count ...`, or
`expected complete_required_summary_fields_source_scope_count ...`,
`expected complete_required_summary_fields_source_scope ... was not found`, or
`expected affected_inputs_count ...`, `expected failed_inputs_count ...`,
`expected skipped_inputs_count ...`, or `expected skipped_reasons_count ...`,
and the expected values are copied into
`--summary-output`. Supplying any `--expect-*` option also forces stdout to use
the aggregate result shape, even for a single successful input, so CI can read
the actual and expected counts from the same fields every time. In
`--format text`, supplied count expectations are also printed as
`expected_expanded_inputs_count=...`,
`expected_excluded_output_artifacts_count=...`,
`expected_inputs_count=...`, `expected_success_count=...`,
`expected_error_count=...`, `expected_passed_true_count=...`,
`expected_passed_false_count=...`, `expected_passed_unknown_count=...`, and
`expected_skipped_count=...`, `expected_reason_codes_count=...`,
`expected_contract_versions_count=...`, `expected_levels_count=...`,
`expected_sources_count=...`,
`expected_verification_scopes_count=...`, and
`expected_acceptance_profiles_count=...`,
`expected_enabled_requirements_count=...`, and
`expected_complete_required_summary_fields_source_scope_count=...`,
`expected_complete_required_summary_fields_source_scopes=...`,
`missing_expected_complete_required_summary_fields_source_scopes=...`, and
`expected_affected_inputs_count=...`, `expected_failed_inputs_count=...`,
`expected_skipped_inputs_count=...`, and
`expected_skipped_reasons_count=...` on the summary line. Use
`--expect-reason-codes-count N` when CI needs to fail on any new or missing
aggregate reason code without listing the expected code names. Use
`--expect-sources-count N`, `--expect-verification-scopes-count N`, and
`--expect-acceptance-profiles-count N` when CI needs to catch metadata drift
without listing the exact metadata values. Use
`--expect-affected-inputs-count N` when CI needs to assert how many source
inputs or reports the gate evidence references. Use
`--expect-failed-inputs-count N`, `--expect-skipped-inputs-count N`, and
`--expect-skipped-reasons-count N` when CI needs to assert how many artifacts
landed in the failed or skipped input previews, and how many distinct skip
causes were reported.
Use repeatable
`--expect-complete-required-summary-fields-source-scope SOURCE/SCOPE` when CI
must prove that a specific gate source and verification scope contributed a
complete-delivery required-summary-fields contract. This catches regressions
where the aggregate count is still correct but the Godot CLI smoke-motion gate
or Web Godot-load gate disappeared from the validated artifact set. Values must
contain exactly one `/` with non-empty source and scope segments; malformed
values fail during argument parsing with
`value must be formatted as non-empty source/scope`. Formatted values must also
appear in the shared schema's
`complete_required_summary_fields_source_scopes`; unsupported values fail before
artifact validation and print the supported `source/scope` list.
Use repeatable
`--allow-complete-required-summary-fields-source-scope SOURCE/SCOPE` when CI must
also fail if any other complete-delivery required-summary-fields source/scope
appears in the validated artifact set. JSON and text summaries then include
`allowed_complete_required_summary_fields_source_scopes` and
`unexpected_complete_required_summary_fields_source_scopes`.
Use repeatable
`--forbid-complete-required-summary-fields-source-scope SOURCE/SCOPE` when CI must
fail if a known complete-delivery source/scope appears at all. Summaries then
include `forbidden_complete_required_summary_fields_source_scopes` and
`present_forbidden_complete_required_summary_fields_source_scopes`.
When any passed-state expectation is supplied, the text summary also includes
`passed=true:N,false:N,unknown:N`.
Use repeatable `--expect-reason-code CODE` when CI also needs to prove that a
specific delivery gate reason was present. Missing reason codes fail the
aggregate result with `expected reason_code ... was not found`, and both
`expected_reason_codes` and `missing_expected_reason_codes` are included in
JSON stdout and `--summary-output`. In `--format text`, the summary line also
includes `expected_reason_codes=...` and
`missing_expected_reason_codes=...` when reason-code expectations were supplied.
Use repeatable `--expect-contract-version VALUE`, `--expect-level VALUE`,
`--expect-source VALUE`, `--expect-verification-scope VALUE`,
`--expect-acceptance-profile VALUE`, and `--expect-enabled-requirement VALUE`
when CI needs to prove that the scan contains the intended gate contract
version, delivery acceptance level, source, verification scope, acceptance
profile, or enabled acceptance requirement. Missing values fail the aggregate
result with `expected contract_version ...`, `expected level ...`,
`expected source ...`, `expected verification_scope ...`,
`expected acceptance_profile ...`, or `expected enabled_requirement ...`, and
JSON stdout plus `--summary-output` include both the expected and missing lists.
In `--format text`, the summary line prints the expected and missing metadata
lists whenever these expectations are supplied.
Use repeatable `--allow-source VALUE`, `--allow-verification-scope VALUE`, and
`--allow-acceptance-profile VALUE`, `--allow-enabled-requirement VALUE`, and
`--allow-contract-version VALUE`, and `--allow-level VALUE` when CI needs to
reject artifacts from any contract version, delivery acceptance level, source,
verification scope, acceptance profile, or enabled acceptance requirement
outside an explicit allowlist. Unexpected values fail the aggregate result and
are reported as `unexpected_contract_versions`, `unexpected_levels`,
`unexpected_sources`, `unexpected_verification_scopes`,
`unexpected_acceptance_profiles`, or `unexpected_enabled_requirements` in JSON
stdout, `--summary-output`, and text summary lines.
Use repeatable `--forbid-source VALUE`, `--forbid-verification-scope VALUE`,
`--forbid-acceptance-profile VALUE`, `--forbid-enabled-requirement VALUE`, and
`--forbid-contract-version VALUE`, and `--forbid-level VALUE` when CI needs to
reject known-bad metadata, contract versions, delivery acceptance levels, or
acceptance requirements without maintaining a full allowlist. Present forbidden
values fail the aggregate result and are reported as
`present_forbidden_contract_versions`, `present_forbidden_levels`,
`present_forbidden_sources`, `present_forbidden_verification_scopes`,
`present_forbidden_acceptance_profiles`, or
`present_forbidden_enabled_requirements`.
Use repeatable `--allow-reason-code CODE` when CI needs to fail on new or
unexpected gate reasons. Any aggregate `reason_codes` entry outside the allowlist
fails with `unexpected reason_code ... was found`; JSON output and
`--summary-output` include `allowed_reason_codes` and `unexpected_reason_codes`,
and text summary lines include `allowed_reason_codes=...` and
`unexpected_reason_codes=...`.
Use repeatable `--forbid-reason-code CODE` when CI only needs to reject known
bad gate reasons without maintaining a full allowlist. Present forbidden values
fail with `forbidden reason_code ... was found`; JSON output and
`--summary-output` include `forbidden_reason_codes` and
`present_forbidden_reason_codes`, and text summary lines include the same values.
Use repeatable `--expect-skipped-reason TEXT`, `--allow-skipped-reason TEXT`,
and `--forbid-skipped-reason TEXT` when directory scans must prove why inputs
were skipped, reject unexpected skip causes, or block a known-bad skip cause.
These constraints use the same aggregate contract as reason codes: failures are
reported as `expected skipped_reason ... was not found`,
`unexpected skipped_reason ... was found`, or
`forbidden skipped_reason ... was found`, and JSON/text summaries include the
expected, missing, allowed, unexpected, forbidden, and present-forbidden skipped
reason lists.
Repeated reason-code constraints are normalized by first occurrence, so duplicate
CLI flags do not duplicate summary entries or aggregate errors. Empty
reason-code constraint values fail during argument parsing with
`value must be non-empty`.
Add `--recursive` only when the artifact directory is intentionally nested.
Use `--format text` for stable log lines instead of JSON output; skipped detail
lines include `skip_reason=...` so CI logs show why an input was ignored. Gate
detail lines also print `complete_required_summary_fields=...` and
`complete_required_summary_fields_count=...` when the gate source/scope maps to
known complete-delivery counters. Summary text also prints
`complete_required_summary_fields_source_scopes=...` when those aggregate
source/scope contracts are present.
All report modes also expose a top-level `delivery_acceptance_gate` with
`contract_version`, `source`, `verification_scope`, `required`,
`requires_full_mechanical_restoration_gate`, `acceptance_profile`,
`acceptance_requirements`, `passed`, `exit_code`, `level`,
`complete`, `reasons`, `reason_codes`, and
`reason_details`. The gate also includes `summary_counts`, a compact set of
aggregate counters for `inputs_count`, `success_count`, `error_count`,
`live_smoke_count`, delivery provenance counts, completion counts, parameter
incompletion count, parameter-level fixed-joint lock checked/mismatch counts,
node-tree fixed-joint lock checked/mismatch counts, and total smoke failure
reasons. CLI gate `summary_counts` also carries node-tree gate audit fields:
`node_tree_gate_enabled_count`, `node_tree_full_restoration_required_count`,
`node_tree_full_restoration_not_required_count`, and
`node_tree_gate_check_counts`, plus mechanical gate audit fields:
`mechanical_gate_enabled_count`, `full_mechanical_restoration_required_count`,
`full_mechanical_restoration_not_required_count`, and
`mechanical_gate_check_counts`. The current gate contract version is
`delivery_acceptance_gate.v1`. The shared workflow contract module exposes
`validate_delivery_acceptance_gate(payload)` for CI and tests that need to
assert the gate artifact shape before reading reason codes or counts. It also
exposes `DELIVERY_ACCEPTANCE_GATE_SCHEMA`, a machine-readable schema summary
containing required fields, allowed `source`, `verification_scope`,
`acceptance_profile`, `level`, and `reason_code` values, requirement fields,
summary count field names, and the valid `source`/scope/profile/level
combinations as `source_scope_pairs`, `source_profile_values`, and
`scope_level_values`. It also groups reason codes and summary fields by source
as `reason_code_values_by_source` and `summary_fields_by_source`, and groups
requirements that may be enabled per source as
`enabled_requirement_values_by_source`, so consumers can distinguish
smoke-report gate fields from Web load delivery fields without hard-coding
source-specific lists. The CLI
also runs this validator before writing `--output` or `--gate-output`; an
internal gate contract error exits with code `2` and a stderr line beginning
with `delivery_acceptance_gate contract invalid`. CLI
reports use `source=dynamic_godot_report_cli` and
`verification_scope=godot_smoke_motion`, meaning the gate can include Godot
smoke, assembly, parameter, and motion checks. `acceptance_requirements` records
the effective requirements enabled by the selected profile, including whether
Godot smoke, Godot-verified acceptance, the full mechanical restoration gate,
and the smoke-level full mechanical restoration gate were required. It also
records the expanded live gate requirements implied by the smoke runner, such as
mechanical restoration completeness, joint parameter readback, control readback,
full node-tree restoration, node class matching, applied joint parameters,
transform matching, physical property matching, fixed-joint lock matching,
motion gates, action target gates, and restoration score gates. CI should use that object as the
canonical explanation when the process exit code is non-zero even though JSON
normalization itself succeeded. `reasons` remains human-readable, while
`reason_codes` and `reason_details` provide stable machine-readable values such as
`no_inputs`, `missing_godot_smoke`, `missing_godot_smoke_report`,
`invalid_godot_smoke_report`, `static_only`, `robot_errors`,
`unknown_delivery_provenance`, `missing_dynamic_generation`,
`incomplete_delivery`, `incomplete_parts`, `incomplete_joints`,
`incomplete_joint_parameters`, `incomplete_restoration`,
`control_readback_missing`, `missing_full_mechanical_restoration_gate`, and
`smoke_failure_reasons`. Web delivery gates can also emit
`godot_delivery_failed` and `missing_godot_assembly_summary`; fixed-joint
readback failures use `fixed_lock_mismatch` for Web load delivery and
`node_tree_fixed_lock_mismatch` for smoke-restored node-tree mismatches.
Each `reason_details` entry includes `code`, `count`, `message`, `inputs`,
`inputs_count`, and `inputs_truncated`. `inputs` lists the affected robot config
paths, capped at the first 10 entries; `inputs_count` preserves the full affected
count when the list is truncated. The contract validator rejects any
`reason_details` fields outside this fixed `reason_detail_fields` set, and
requires
`reason_codes` entries to be non-empty strings, to use one of the stable reason
codes listed above, to be allowed for the gate `source` according to
`reason_code_values_by_source`, and to match `reason_details[*].code` in both
directions and in the same order. `reason_codes` entries and
`reason_details[*].code` values must be
unique, so each stable blocker appears once with one aggregate count and input
preview. It also applies the same source-specific code check to each
`reason_details[*].code`. The human-readable `reasons` list must contain unique
entries, and it must also match the unique `reason_details[*].message` values in
both directions and in the same order, so compact logs and structured
diagnostics cannot drift apart.
It requires each previewed input to be a non-empty
string, unique within its detail entry, `inputs_count` to be at least the
preview length, and `inputs_count` to equal the preview length when
`inputs_truncated` is false. It also requires
`reason_details[*].count >= inputs_count`, so a reason cannot claim fewer
occurrences than affected inputs.
The result fields are checked for consistency: `passed` must match
`exit_code == 0`, `reasons` entries must be non-empty strings, and a non-zero
`exit_code` must carry non-empty `reasons`, `reason_codes`, and `reason_details`.
If `required` is true and `complete` is false, or if
`requires_full_mechanical_restoration_gate` is true and `complete` is false,
`exit_code` must be non-zero. When `complete` is true, `passed` must be true
and `exit_code` must be zero; these complete-result expectations are also
published in the schema as `complete_result_requirements`.
A gate with `complete: true` must not carry `reasons`, `reason_codes`, or
`reason_details`, and `summary_counts.inputs_count` must be greater than zero.
`complete` is also tied to `level` through
`complete_level_by_scope`: `godot_smoke_motion` gates use `godot_verified` for
complete restoration, while `godot_load` gates use `godot_load_verified`.
Those verified levels require `complete: true`, and `complete: true` cannot use
`static_only` or `incomplete`.
Gate requirements are also cross-checked: `requires_full_mechanical_restoration_gate`
requires `acceptance_requirements.full_mechanical_restoration_gate: true`, the
`full_mechanical_restoration` profile requires both that requirement and
`requires_full_mechanical_restoration_gate: true`, and
`verification_scope: godot_load` requires `acceptance_requirements.godot_load: true`.
Any requirement set to `true` must also be allowed for the gate `source`
according to `enabled_requirement_values_by_source`; Web load gates may enable
`godot_load`, mechanical restoration completeness, joint parameter readback,
and fixed-lock readback checks, while CLI smoke gates may enable the smoke and
restoration requirements.
The metadata fields are enumerated by contract: `source` is either
`dynamic_godot_report_cli` or `web_godot_delivery`; `verification_scope` is
either `godot_smoke_motion` or `godot_load`; `acceptance_profile` is `custom`,
`full_mechanical_restoration`, or `web_godot_load`; and `level` is
`godot_verified`, `static_only`, `incomplete`, or `godot_load_verified`.
CLI gates must pair `dynamic_godot_report_cli` with `godot_smoke_motion`, while
Web delivery gates must pair `web_godot_delivery` with `godot_load`. CLI gates
may use `custom` or `full_mechanical_restoration` profiles; Web delivery gates
must use `web_godot_load`. The `full_mechanical_restoration` profile also
requires `acceptance_requirements.full_mechanical_restoration_gate: true`.
`godot_smoke_motion` gates may report `godot_verified`, `static_only`, or
`incomplete` levels; `godot_load` gates may report `godot_load_verified` or
`incomplete`.
When the gate returns a non-zero exit code, the tool also writes one stable
stderr line beginning with `delivery_acceptance_gate failed`, including
`contract_version`, `source`, `verification_scope`, `acceptance_profile`, and
compact enabled `requirements`, plus compact
`counts=inputs:N,errors:N,live:N,verified:N,static:N,fixed:N/N,treefixed:N/N,treefixedok:N/N,treegate:N/N,mechgate:N/N,failures:N`
diagnostics. `treefixedok:N/N` shows node-tree fixed-lock
complete/incomplete counts when present; `treegate:N/N` shows total enabled
node-tree gate checks versus robots requiring full node-tree restoration.
`mechgate:N/N` shows total enabled mechanical gate checks versus robots
requiring full mechanical restoration. It also includes
`topology=complete:X/Y,incomplete:Z,disconnected:N,unreachable:N,duplicates:N,cycles:N,roots:...`
from the static topology summary, then ends with the comma-separated
`reason_codes` plus an `affected_inputs` preview, so CI logs can show the
blocker, gate type, aggregate counts, topology status, and the first affected
configs without opening the JSON report artifact. The stderr preview is
intentionally compact; use
`delivery_acceptance_gate.reason_details[].inputs` for the full structured
artifact data.
When node-tree or mechanical check maps are present, CLI logs and validator text
output also include `checks=tree[...]|mech[...]`; for example
`checks=tree[fixed_lock_mismatch:1]|mech[joint_parameter_readback:1,mechanical_restoration:1]`.
This mirrors the Web panel's Gate Checks field and makes the exact enabled
check evidence visible in CI logs without opening the JSON artifact.
Validator JSON output keeps the same evidence as structured data:
single-input results include compact `summary_counts.node_tree_gate_check_counts`
and `summary_counts.mechanical_gate_check_counts`, while `--summary-only` JSON
aggregates them as top-level `node_tree_gate_check_counts` and
`mechanical_gate_check_counts`.
Use `--expect-node-tree-gate-check-count CHECK=N` and
`--expect-mechanical-gate-check-count CHECK=N` when CI needs those structured
check maps to become blocking. The validator rejects unknown check keys and
records both `expected_*_gate_check_counts` and `mismatched_expected_*_gate_check_counts`
in summary JSON so failures identify the exact missing or unexpected check.
Text summaries print the same expectation as `expected_checks=tree[...]|mech[...]`
and print `mismatched_expected_checks=...` with `expected/actual` pairs when a
check count is wrong.
In text summary mode, `--show-diagnostics` / metadata output also includes the
same compact `checks=...` field, using `checks=none` when no enabled check map
has a positive count.
When multiple inputs are provided, the output contains `batch_summary` plus one
report per robot. Batch live smoke uses incrementing ports starting at `--port`.
For live batch runs, `batch_summary` also aggregates gate outcomes across all
robots: `live_smoke_count`, `delivery_godot_verified_count`,
`delivery_static_only_count`, `delivery_unverified_count`,
`delivery_dynamic_generation_count`, `delivery_acceptance_complete`,
`delivery_acceptance_level`, `delivery_acceptance_reasons`,
`delivery_complete_count`, `delivery_incomplete_count`,
`delivery_parameters_incomplete_count`, total
`parameter_mismatch_count`, `fixed_lock_checked_count`,
`fixed_lock_mismatch_count`, `node_tree_fixed_lock_checked_count`,
`node_tree_fixed_lock_mismatch_count`,
`node_tree_fixed_locks_complete_count`,
`node_tree_fixed_locks_incomplete_count`,
`parameter_consistency_complete_count`, `control_mismatch_count`,
`control_configured_count`, `control_readback_checked_count`,
`control_readback_missing_count`,
`control_consistency_complete_count`, `action_target_mismatch_count`,
`nonzero_action_targets_under_min_count`,
`unknown_action_target_count`,
`invalid_action_target_count`,
`action_target_consistency_complete_count`,
`action_sequence_target_mismatch_count`,
`action_sequence_target_consistency_complete_count`,
`action_target_coverage_under_min_count`,
`action_target_coverage_complete_count`,
`control_action_coverage_under_min_count`,
`control_action_coverage_complete_count`,
`joint_angle_delta_under_min_count`,
`joint_angle_range_under_min_count`,
`moving_joint_coverage_under_min_count`,
`commanded_joint_response_under_min_count`,
`commanded_static_joint_count`,
`action_transitions_under_min_count`,
`action_transition_delta_under_min_count`,
`restoration_complete_count`, `restoration_incomplete_count`,
`mechanical_gate_enabled_count`,
`full_mechanical_restoration_required_count`,
`full_mechanical_restoration_not_required_count`, and
`mechanical_gate_check_counts`. Node tree
gate status is summarized with
`node_tree_complete_count`, `node_tree_incomplete_count`,
`node_tree_missing_parts_count`, `node_tree_missing_joints_count`, and
`node_tree_class_mismatch_count`, `node_tree_parameter_missing_count`, and
`node_tree_transform_mismatch_count`, and
`node_tree_physical_mismatch_count`. Batch summaries also aggregate effective
node-tree gate configuration as `node_tree_gate_enabled_count`,
`node_tree_full_restoration_required_count`,
`node_tree_full_restoration_not_required_count`, and
`node_tree_gate_check_counts`, so CI can distinguish robots that were run with
the full restoration gate from robots that only enabled a subset of checks.
Static topology evidence is also aggregated as
`static_topology_complete_count`, `static_topology_incomplete_count`,
`static_topology_disconnected_parts_count`,
`static_topology_unreachable_parts_count`,
`static_topology_duplicate_child_endpoint_count`,
`static_topology_cycle_count`, and `static_topology_root_parts`. Each
`batch_summary.robots[]` entry includes the corresponding
`topology_root_part`, `topology_complete_tree`,
`topology_reachable_parts_count`, `topology_disconnected_parts`,
`topology_unreachable_parts`, `topology_duplicate_child_endpoints`, and
`topology_cycle` preview for per-robot triage.
The delivery acceptance gate also mirrors the scalar `static_topology_*_count`
values inside `summary_counts`, so `validate_delivery_acceptance_gate.py` can
enforce them with `--expect-summary-count` or `--expect-summary-value`.
The delivery gate contract validates its known `summary_counts` values as
non-negative integers, and validates both `node_tree_gate_check_counts` and
`mechanical_gate_check_counts` as objects whose values are non-negative
integers. Their object keys are also enumerated by
`summary_count_map_key_values`: node-tree checks may only use
`incomplete_node_tree`, `class_mismatch`, `missing_parameters`,
`transform_mismatch`, `physical_mismatch`, and `fixed_lock_mismatch`;
mechanical checks may only use `mechanical_restoration`,
`joint_parameter_readback`, `control_parameter_readback`, and
`full_node_tree_restoration`. The schema also exposes
`summary_value_paths`, a flattened list of scalar summary counters and
check-map paths such as `node_tree_gate_check_counts.fixed_lock_mismatch` for
CI jobs that use `--expect-summary-value`, and `summary_value_paths_by_source`
for generating source-specific constraints. For example, Web Godot-load gates
do not expose CLI-only smoke readback counters such as
`control_readback_missing_count`. The validator can enforce the same source
filter with `--expect-summary-value-source SOURCE`.
The schema also exposes
`requirement_summary_map_counts`, mapping each requirement field to the summary
map key that must equal `inputs_count` when that requirement is enabled on a
complete CLI smoke-motion gate. `requirement_summary_map_counts_by_source_scope`
also exposes that mapping per valid source/scope pair, with an empty map for
sources that do not enforce requirement-to-check-count evidence. `complete_summary_counts`
lists the generic summary counters that must equal `inputs_count` or zero when `complete` is true,
`complete_summary_counts_by_source` filters those rules to fields legal for each
gate source, `scope_complete_summary_counts` lists the same rules that only
apply to a specific verification scope, and
`complete_summary_counts_by_source_scope` pre-merges both views for each valid
source/scope pair. `complete_required_summary_fields_by_source_scope` flattens
those per-source/per-scope rules into the exact `summary_counts` keys that must
be present for a complete delivery gate, and
`complete_required_summary_fields_source_scopes` lists the valid `source/scope`
pairs with those complete-delivery field contracts. Total consistency is exposed as
`summary_count_sum_rules`, `summary_count_lte_rules`, and
`summary_map_sum_rules`, so external CI can derive the same aggregate checks
without duplicating Python implementation details. Unknown `summary_counts` keys
are rejected, and
known keys must be allowed for the gate `source` according to
`summary_fields_by_source`; for example, Web load gates cannot claim CLI
smoke-only control readback counts.
Core totals must also be internally consistent:
`success_count + error_count == inputs_count`,
`delivery_godot_verified_count + delivery_static_only_count + delivery_unverified_count == inputs_count`,
`delivery_complete_count + delivery_incomplete_count == inputs_count`, and
`live_smoke_count <= inputs_count`. The values in `node_tree_gate_check_counts`
must sum to `node_tree_gate_enabled_count`, and the values in
`mechanical_gate_check_counts` must sum to `mechanical_gate_enabled_count`.
When `complete` is true, the required summary counters must be present and must
reflect a clean completion: `success_count` and `delivery_complete_count` equal `inputs_count`,
`delivery_godot_verified_count` and `delivery_dynamic_generation_count` equal
`inputs_count`, while `error_count`, `delivery_incomplete_count`,
`delivery_parameters_incomplete_count`, `delivery_unverified_count`,
`delivery_static_only_count`, `control_readback_missing_count`,
`failure_reasons_count`, `fixed_lock_mismatch_count`,
`node_tree_fixed_lock_mismatch_count`, and
`node_tree_fixed_locks_incomplete_count` are zero. For `godot_smoke_motion`
gates, `live_smoke_count` and
`smoke_report_written_count` also equal `inputs_count`, while
`smoke_report_missing_count` and `smoke_report_read_error_count` are zero. If
a CLI smoke-motion requirement enables a node-tree or mechanical gate check, the
matching key in `node_tree_gate_check_counts` or `mechanical_gate_check_counts`
must also equal `inputs_count`, proving every input ran that required check.
Each robot entry mirrors the key gate state with `godot_smoke_returncode`,
`delivery_source`, `delivery_dynamic_robot_generation`,
`delivery_complete`, `delivery_expected_parts`, `delivery_parts_complete`,
`delivery_expected_joints`, `delivery_failed_joints`,
`delivery_joints_complete`, `delivery_parameterized_joints`,
`delivery_parameters_complete`, `delivery_part_nodes_count`,
`delivery_joint_nodes_count`,
`parameter_mismatch_count`, `parameter_consistency_complete`,
`fixed_lock_checked_count`, `fixed_lock_mismatch_count`,
`control_mismatch_count`, `control_configured_count`,
`control_readback_checked_count`, `control_readback_missing_count`,
`control_consistency_complete`,
`nonzero_action_target_count`, `nonzero_action_targets_under_min`,
`action_target_mismatch_count`, `action_target_consistency_complete`,
`unknown_action_target_count`,
`invalid_action_target_count`,
`action_sequence_target_mismatch_count`,
`action_sequence_target_consistency_complete`,
`action_target_coverage_ratio`, `action_target_coverage_under_min`,
`action_target_coverage_complete`,
`control_action_coverage_ratio`, `control_action_coverage_under_min`,
`control_action_coverage_complete`,
`joint_angle_delta_max`, `joint_angle_delta_under_min`,
`joint_angle_range_max`, `joint_angle_range_under_min`,
`moving_joint_coverage_ratio`, `moving_joint_coverage_under_min`,
`commanded_joint_response_ratio`, `commanded_joint_response_under_min`,
`commanded_static_joints`, `commanded_joint_response_details`,
`action_transition_count`, `action_transitions_under_min`,
`action_transition_delta_max`, `action_transition_delta_under_min`,
`restoration_complete`, `restoration_score`,
`mechanical_gate_enabled_count`, `full_mechanical_restoration_required`,
`mechanical_gate_enabled_checks`, `node_tree_complete`,
`node_tree_gate_enabled_count`, `node_tree_full_restoration_required`,
`node_tree_gate_enabled_checks`,
`node_tree_missing_parts_count`, `node_tree_missing_joints_count`, and
`node_tree_class_mismatch_count`, `node_tree_parameter_missing_count`, and
`node_tree_transform_mismatch_count`, and
`node_tree_physical_mismatch_count`.
Failed or skipped batch entries also expose
`failure_reasons`, and the batch root records `failure_reasons_count`, so CI
logs can show validation errors, skipped smoke reasons, and smoke gate failures
without parsing stdout.
For delivery restoration failures, the batch root includes
`delivery_failure_robots`, a compact preview of robots whose delivery is
incomplete or whose delivered joints are not fully parameterized.
These records keep `delivery_source` so CI can distinguish
`static_normalization` from an actual `godot_smoke` load, and live smoke runs
also carry delivered part and joint node counts.
At the batch root, `delivery_godot_verified_count` counts robots loaded through
Godot, `delivery_static_only_count` counts robots validated only by schema
normalization, and `delivery_unverified_count` flags missing or unknown
delivery provenance.
`delivery_acceptance_complete` is true only when every input is dynamically
generated, loaded through Godot smoke, fully delivered, fully parameterized, and
free of smoke failure reasons. `delivery_acceptance_level` is `godot_verified`,
`static_only`, or `incomplete`; `delivery_acceptance_reasons` lists the blocking
conditions when the batch has not reached full Godot-verified acceptance.
`delivery_acceptance_reason_codes` and `delivery_acceptance_reason_details`
mirror those blockers in a stable structured form with affected input paths.
For commanded response failures, the batch root also includes
`commanded_response_failure_robots`, a compact preview of the affected robots,
static commanded joints, failed response detail records, and failure reasons.
The Web workflow panel renders commanded response failures with the static
joint names plus the first response detail records, including commanded target,
observed angle range, and `responded` status.
When a workflow artifact is sent to Godot from the Web panel, the delivery
record also stores `assembly_mapping_summary`, `assembly_restoration_summary`,
and `assembly_fixed_lock_summary` derived directly from the Godot load result.
This gives the UI a smoke-free restoration and fixed-joint lock status for
normal interactive syncs, while deeper motion and action gates remain in the
headless smoke report.
The Web delivery record also stores `static_node_tree_manifest_evidence`,
generated from the normalized workflow artifact before transport. This evidence
records `godot_node_tree_manifest.v1`, manifest validity, validation errors,
completeness flags, part/joint counts, output-path presence when an upstream
manifest sidecar is known, and path-map mismatch counts/kind counts. The Web
`delivery_acceptance_gate.summary_counts` preserves the same static manifest
fields used by CLI gates, including `static_node_tree_manifest_valid_count`,
`static_node_tree_manifest_invalid_count`,
`static_node_tree_manifest_error_count`,
`static_node_tree_manifest_output_count`,
`static_node_tree_manifest_path_map_mismatch_count`, and
`static_node_tree_manifest_path_map_mismatch_kind_counts`.
The same Web delivery record now includes `delivery_acceptance_gate`. Its
`contract_version` is `delivery_acceptance_gate.v1`, `source` is
`web_godot_delivery`, and `verification_scope` is `godot_load`.
The Web direct-load gate is also validated with
`validate_delivery_acceptance_gate(payload)` before it is returned; an internal
contract error raises `delivery_acceptance_gate contract invalid` instead of
returning a malformed gate object.
It uses the same `summary_counts` field shape as CLI gates; for Web direct-load
delivery, `live_smoke_count` remains `0` because this scope verifies Godot load
acceptance rather than the headless smoke-motion gate. Web direct-load gates
also use the same expanded `acceptance_requirements` shape as CLI gates, with
`godot_load=true` and the direct-load restoration requirements for mechanical
restoration completeness, joint parameter readback, and fixed-joint lock
matching enabled; smoke-only motion and action gates remain `false`.
Web direct-load gates
populate `fixed_lock_checked_count` and `fixed_lock_mismatch_count` from
`assembly_fixed_lock_summary` when the Godot load result includes fixed joint
readback. Any non-zero `fixed_lock_mismatch_count` makes the Web load gate
incomplete and adds the `fixed_lock_mismatch` reason code.
An invalid or incomplete Web static manifest evidence block also makes the Web
load gate incomplete and adds `static_node_tree_incomplete`, so malformed
artifact-to-Godot evidence cannot be hidden behind a successful transport call.
The Web gate `summary_counts` also records the enabled direct-load check maps:
`mechanical_gate_check_counts` includes mechanical restoration and joint
parameter readback, while `node_tree_gate_check_counts` includes the fixed-lock
match check. A complete `godot_load_verified` gate must report those enabled
checks for the input, so the requirement map and the evidence counters cannot
silently drift apart.
The `godot_load_verified` level means the artifact was accepted by Godot and
the returned assembly summary is complete; it is intentionally weaker than the
CLI `godot_verified` level, which also requires headless smoke and motion gates.
Workflow run responses also expose `godot_evidence_summary` for the operator
panel. Its `summary_version` is `web_godot_evidence_summary.v1`; `level` is one
of `static_only`, `godot_load_verified`, `godot_verified`, or `incomplete`; and
`levels` records the boolean proof state for the static/load/live tiers. The
summary also carries `static.manifest_mismatch_count`,
`static.manifest_mismatch_kind_counts`, `residual_risks`, and action metadata
for `godot_load`, recommended `godot_sync`, and
`/api/workflows/runs/{run_id}/godot-readiness-summary`. The readiness endpoint
returns a `dynamic_godot_release_readiness_summary.v1` payload for the selected
run without rerunning Godot, so operators can inspect the strongest currently
proven level from the Web UI.
The Web panel renders this load gate with compact gate counts, reason codes, and
affected inputs above the assembly counts. It also renders
the enabled check maps as `tree[...]` and `mech[...]`, so operators can see the
specific node-tree and mechanical checks that contributed to the load gate
without opening the raw JSON. The same panel renders
`assembly_fixed_lock_summary` as Load Fixed Locks / Load Fixed Mismatch / Load
Fixed Match, so direct-load fixed-joint lock problems are visible even without a
headless smoke report. If delivery fails before Godot returns an assembly
summary, the panel still renders the load gate and then shows the empty
assembly-report state, so failure reasons remain visible in the run detail.

Browser/manual validation remains separate from the non-live dynamic Godot CI
path. The static CI gate runs `tools/build_static_godot_node_tree_evidence.py`
and related JSON contract checks only; it must not require Playwright,
Chromium, a running Web Panel, or a human-filled browser validation report.
When a release needs browser evidence, run the explicit Web browser checklist in
`docs/guides/WEB_BROWSER_MANUAL_VALIDATION_CHECKLIST_20260426.md` and archive
the outputs from `tools/run_web_browser_playwright_smoke.py`,
`tools/build_web_browser_manual_validation_report.py`,
`tools/build_web_browser_validation_closeout.py`, and
`tools/build_web_browser_validation_evidence_pack.py` as release evidence.

## Current Limits

- Meshes are primitive shapes, not CAD-quality assets.
- Joint axes are approximated by orienting the joint node; complex multi-axis
  joints need a richer schema.
- `fixed` joints use a locked `Generic6DOFJoint3D`; this restores fixed
  constraints for generated robots but is still not a dedicated fixed-joint node.
- Relative joint angle is diagnostic telemetry estimated from endpoint body
  orientation. It is suitable for smoke checks and UI warnings, but it is not a
  native solver readback.
