import json
import importlib.util
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests" / "fixtures" / "robot_dynamic_biped.json"
QUADRUPED_FIXTURE = ROOT / "tests" / "fixtures" / "robot_dynamic_quadruped.json"
FIXED_PAIR_FIXTURE = ROOT / "tests" / "fixtures" / "robot_dynamic_fixed_pair.json"
DUPLICATE_PART_IDS_FIXTURE = (
    ROOT / "tests" / "fixtures" / "robot_dynamic_duplicate_part_ids.json"
)
DUPLICATE_CONNECTION_NAMES_FIXTURE = (
    ROOT / "tests" / "fixtures" / "robot_dynamic_duplicate_connection_names.json"
)
ROOT_DRIFT_MANIFEST_FIXTURE = (
    ROOT / "tests" / "fixtures" / "robot_dynamic_root_drift.node_tree_manifest.json"
)
BIPED_TEMPLATE = (
    ROOT
    / "agi_walker"
    / "skills"
    / "robot-modeling"
    / "assets"
    / "templates"
    / "biped_basic.json"
)
QUADRUPED_TEMPLATE = (
    ROOT
    / "agi_walker"
    / "skills"
    / "robot-modeling"
    / "assets"
    / "templates"
    / "quadruped_dog.json"
)
ROBOT_SCHEMA = ROOT / "agi_walker" / "core" / "api" / "robot_schema.py"
WORKFLOW_CONTRACTS = ROOT / "agi_walker" / "core" / "api" / "workflow_contracts.py"
WORKFLOWS_API = ROOT / "web_panel" / "workflows_api.py"
WORKFLOWS_HTML = ROOT / "web_panel" / "static" / "workflows.html"
VALIDATOR_TOOL = ROOT / "tools" / "validate_robot_config_for_godot.py"
GATE_VALIDATOR_TOOL = ROOT / "tools" / "validate_delivery_acceptance_gate.py"
SMOKE_TOOL = ROOT / "tools" / "run_dynamic_godot_robot_smoke.py"
REPORT_TOOL = ROOT / "tools" / "build_dynamic_robot_generation_report.py"
STATIC_EVIDENCE_TOOL = ROOT / "tools" / "build_static_godot_node_tree_evidence.py"
RELEASE_READINESS_TOOL = ROOT / "tools" / "build_dynamic_godot_release_readiness.py"
RELEASE_EVIDENCE_BUNDLE_TOOL = (
    ROOT / "tools" / "build_dynamic_godot_release_evidence_bundle.py"
)
RELEASE_EVIDENCE_BUNDLE_VALIDATOR_TOOL = (
    ROOT / "tools" / "validate_dynamic_godot_release_evidence_bundle.py"
)
ASSEMBLER = ROOT / "godot_project" / "scripts" / "robot_assembler.gd"
CONTROLLER = ROOT / "godot_project" / "scripts" / "generated_robot_controller.gd"
TCP_SERVER = ROOT / "godot_project" / "scripts" / "tcp_server.gd"
CI_WORKFLOW = ROOT / ".github" / "workflows" / "ci.yml"


def _acceptance_args(**overrides: object) -> SimpleNamespace:
    values = {
        "run_godot_smoke": False,
        "require_godot_verified_acceptance": False,
        "require_full_mechanical_restoration_gate": False,
        "fail_on_full_mechanical_restoration": False,
        "fail_on_incomplete_restoration": False,
        "fail_on_parameter_mismatch": False,
        "fail_on_control_mismatch": False,
        "fail_on_full_node_tree_restoration": False,
        "fail_on_incomplete_node_tree": False,
        "fail_on_node_tree_class_mismatch": False,
        "fail_on_node_tree_missing_parameters": False,
        "fail_on_node_tree_transform_mismatch": False,
        "fail_on_node_tree_physical_mismatch": False,
        "fail_on_node_tree_fixed_lock_mismatch": False,
        "fail_on_joint_limit_violation": False,
        "min_body_displacement": None,
        "max_linear_speed": None,
        "min_joint_angle_delta": None,
        "min_joint_angle_range": None,
        "min_moving_joint_coverage": None,
        "min_commanded_joint_response_coverage": None,
        "fail_on_action_target_mismatch": False,
        "fail_on_action_sequence_target_mismatch": False,
        "fail_on_unknown_action_target": False,
        "fail_on_invalid_action_target": False,
        "min_action_target_coverage": None,
        "min_control_action_coverage": None,
        "min_nonzero_action_targets": None,
        "min_action_transitions": None,
        "min_action_transition_delta": None,
        "min_restoration_score": None,
    }
    values.update(overrides)
    return SimpleNamespace(**values)

spec = importlib.util.spec_from_file_location("robot_schema", ROBOT_SCHEMA)
robot_schema = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(robot_schema)

contracts_spec = importlib.util.spec_from_file_location(
    "workflow_contracts", WORKFLOW_CONTRACTS
)
workflow_contracts = importlib.util.module_from_spec(contracts_spec)
assert contracts_spec.loader is not None
contracts_spec.loader.exec_module(workflow_contracts)


def _acceptance_requirements(**overrides: bool) -> dict[str, bool]:
    return workflow_contracts.build_delivery_acceptance_requirements(**overrides)


def test_dynamic_biped_fixture_matches_godot_robot_contract() -> None:
    payload = json.loads(FIXTURE.read_text(encoding="utf-8"))

    assert robot_schema.validate_godot_robot_config(payload) == []


def test_dynamic_quadruped_fixture_matches_godot_robot_contract() -> None:
    payload = json.loads(QUADRUPED_FIXTURE.read_text(encoding="utf-8"))

    assert robot_schema.validate_godot_robot_config(payload) == []


def test_dynamic_fixed_pair_fixture_matches_godot_robot_contract() -> None:
    payload = json.loads(FIXED_PAIR_FIXTURE.read_text(encoding="utf-8"))

    assert robot_schema.validate_godot_robot_config(payload) == []
    assert payload["connections"][0]["joint_type"] == "fixed"
    assert payload["connections"][0]["limits"] == {"lower": 0.0, "upper": 0.0}


def test_fixed_joint_normalization_forces_zero_limits() -> None:
    payload = {
        "name": "fixed_limit_defaults",
        "parts": [
            {"id": "base", "type": "body", "shape": "box", "params": {"mass": 1.0}},
            {"id": "tool", "type": "body", "shape": "box", "params": {"mass": 1.0}},
        ],
        "connections": [
            {
                "name": "base_tool",
                "from": "base",
                "to": "tool",
                "joint_type": "fixed",
                "limits": {"lower": -1.0, "upper": 1.0},
            }
        ],
    }

    normalized = robot_schema.normalize_robot_config_for_godot(payload)

    assert normalized["connections"][0]["limits"] == {"lower": 0.0, "upper": 0.0}
    assert robot_schema.validate_godot_robot_config(normalized) == []
    invalid = json.loads(json.dumps(normalized))
    invalid["connections"][0]["limits"] = {"lower": -0.1, "upper": 0.0}
    assert robot_schema.validate_godot_robot_config(invalid) == [
        "connections[1].limits.lower must be 0.0 for fixed joints"
    ]


def test_robot_schema_rejects_mechanically_invalid_joint_parameters() -> None:
    payload = {
        "name": "invalid_joint_parameters",
        "parts": [
            {"id": "base", "type": "body", "shape": "box", "params": {"mass": 1.0}},
            {"id": "link", "type": "body", "shape": "box", "params": {"mass": 1.0}},
        ],
        "connections": [
            {
                "name": "bad_hinge",
                "from": "base",
                "to": "link",
                "joint_type": "hinge",
                "origin": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 0.0],
                "limits": {"lower": 1.0, "upper": -1.0},
                "motor": {
                    "enabled": True,
                    "target_velocity": 0.0,
                    "max_impulse": -1.0,
                },
                "dynamics": {"damping": -0.1, "friction": -0.01},
            }
        ],
    }
    normalized = robot_schema.normalize_robot_config_for_godot(payload)

    assert normalized["schema_version"] == robot_schema.ROBOT_MECHANICAL_SCHEMA_VERSION
    assert robot_schema.validate_godot_robot_config(normalized) == [
        "connections[1].axis must be non-zero for movable joints",
        "connections[1].limits.lower must be <= limits.upper",
        "connections[1].motor.max_impulse must be a non-negative number",
        "connections[1].dynamics.damping must be a non-negative number",
        "connections[1].dynamics.friction must be a non-negative number",
    ]


def test_robot_schema_rejects_self_connections() -> None:
    payload = {
        "name": "self_connection",
        "parts": [
            {"id": "base", "type": "body", "shape": "box", "params": {"mass": 1.0}},
        ],
        "connections": [
            {
                "name": "base_self",
                "from": "base",
                "to": "base",
                "joint_type": "fixed",
            }
        ],
    }

    normalized = robot_schema.normalize_robot_config_for_godot(payload)

    assert robot_schema.validate_godot_robot_config(normalized) == [
        "connections[1].from and .to must reference different parts"
    ]


def test_robot_schema_rejects_disconnected_mechanical_trees() -> None:
    payload = {
        "name": "disconnected_tree",
        "parts": [
            {"id": "base", "type": "body", "shape": "box", "params": {"mass": 1.0}},
            {"id": "link", "type": "body", "shape": "box", "params": {"mass": 1.0}},
            {"id": "tool", "type": "body", "shape": "box", "params": {"mass": 1.0}},
        ],
        "connections": [
            {
                "name": "base_link",
                "from": "base",
                "to": "link",
                "joint_type": "fixed",
            }
        ],
    }

    normalized = robot_schema.normalize_robot_config_for_godot(payload)

    assert robot_schema.validate_godot_robot_config(normalized) == [
        "connections must connect every part; disconnected parts: tool",
        "connections must define exactly one root part; roots: base, tool",
    ]


def test_robot_schema_rejects_duplicate_child_endpoints() -> None:
    payload = {
        "name": "duplicate_child",
        "parts": [
            {"id": "base_a", "type": "body", "shape": "box", "params": {"mass": 1.0}},
            {"id": "base_b", "type": "body", "shape": "box", "params": {"mass": 1.0}},
            {"id": "link", "type": "body", "shape": "box", "params": {"mass": 1.0}},
        ],
        "connections": [
            {
                "name": "base_a_link",
                "from": "base_a",
                "to": "link",
                "joint_type": "fixed",
            },
            {
                "name": "base_b_link",
                "from": "base_b",
                "to": "link",
                "joint_type": "fixed",
            },
        ],
    }

    normalized = robot_schema.normalize_robot_config_for_godot(payload)

    assert robot_schema.validate_godot_robot_config(normalized) == [
        "connections[2].to duplicates child endpoint 'link'; already connected by base_a_link",
        "connections must define exactly one root part; roots: base_a, base_b",
    ]


def test_robot_schema_rejects_unreachable_cycles() -> None:
    payload = {
        "name": "unreachable_cycle",
        "parts": [
            {"id": "root", "type": "body", "shape": "box", "params": {"mass": 1.0}},
            {"id": "branch", "type": "body", "shape": "box", "params": {"mass": 1.0}},
            {"id": "cycle_a", "type": "body", "shape": "box", "params": {"mass": 1.0}},
            {"id": "cycle_b", "type": "body", "shape": "box", "params": {"mass": 1.0}},
        ],
        "connections": [
            {
                "name": "root_branch",
                "from": "root",
                "to": "branch",
                "joint_type": "fixed",
            },
            {
                "name": "cycle_a_b",
                "from": "cycle_a",
                "to": "cycle_b",
                "joint_type": "fixed",
            },
            {
                "name": "cycle_b_a",
                "from": "cycle_b",
                "to": "cycle_a",
                "joint_type": "fixed",
            },
        ],
    }

    normalized = robot_schema.normalize_robot_config_for_godot(payload)

    assert robot_schema.validate_godot_robot_config(normalized) == [
        "connections must be reachable from root 'root'; unreachable parts: cycle_a, cycle_b",
        "connections must not contain directed cycles: cycle_a -> cycle_b -> cycle_a",
    ]
    topology = robot_schema.build_mechanical_topology_summary(normalized)
    assert topology["root_part"] == "root"
    assert topology["reachable_parts_count"] == 2
    assert topology["unreachable_parts"] == ["cycle_a", "cycle_b"]
    assert topology["cycle"] == ["cycle_a", "cycle_b", "cycle_a"]
    assert topology["complete_tree"] is False


def test_legacy_robot_config_is_normalized_for_godot_assembly() -> None:
    legacy_payload = {
        "name": "legacy_biped",
        "parts": [
            {"id": "torso_1", "type": "torso", "params": {"height": 0.5}},
            {
                "id": "leg_left_1",
                "type": "leg",
                "params": {"thigh_length": 0.3, "shin_length": 0.3, "mass": 1.0},
            },
        ],
        "connections": [
            {
                "from": "torso_1",
                "to": "leg_left_1",
                "joint_type": "revolute",
                "offset": [-0.15, -0.25, 0.0],
            }
        ],
    }

    normalized = robot_schema.normalize_robot_config_for_godot(legacy_payload)

    assert normalized["parts"][0]["shape"] == "box"
    assert normalized["parts"][1]["shape"] == "capsule"
    assert len(normalized["parts"]) == 3
    assert len(normalized["connections"]) == 2
    assert normalized["connections"][0]["to"] == "leg_left_1_upper"
    assert normalized["connections"][1]["name"] == "knee_leg_left_1"
    assert normalized["connections"][0]["joint_type"] == "hinge"
    assert normalized["connections"][0]["origin"] == [-0.15, -0.25, 0.0]
    parts_by_id = {part["id"]: part for part in normalized["parts"]}
    assert parts_by_id["leg_left_1_upper"]["params"]["position"] == [
        -0.15,
        -0.4,
        0.0,
    ]
    assert parts_by_id["leg_left_1_lower"]["params"]["position"] == [
        -0.15,
        -0.7,
        0.0,
    ]
    assert robot_schema.validate_godot_robot_config(normalized) == []


def test_builtin_templates_expand_compound_legs_for_dynamic_godot() -> None:
    biped = json.loads(BIPED_TEMPLATE.read_text(encoding="utf-8"))
    quadruped = json.loads(QUADRUPED_TEMPLATE.read_text(encoding="utf-8"))

    normalized_biped = robot_schema.normalize_robot_config_for_godot(biped)
    normalized_quadruped = robot_schema.normalize_robot_config_for_godot(quadruped)

    assert len(normalized_biped["parts"]) == 5
    assert len(normalized_biped["connections"]) == 4
    biped_parts = {part["id"]: part for part in normalized_biped["parts"]}
    assert "leg_left_1_upper" in biped_parts
    assert biped_parts["leg_left_1_upper"]["params"]["position"] == [
        -0.15,
        -0.4,
        0.0,
    ]
    assert biped_parts["leg_left_1_lower"]["params"]["position"] == [
        -0.15,
        -0.7,
        0.0,
    ]
    biped_joint_names = {conn["name"] for conn in normalized_biped["connections"]}
    assert "knee_leg_left_1" in biped_joint_names
    assert len(normalized_quadruped["parts"]) == 9
    assert len(normalized_quadruped["connections"]) == 8
    quadruped_parts = {part["id"]: part for part in normalized_quadruped["parts"]}
    assert quadruped_parts["leg_front_left_1_upper"]["params"]["position"] == [
        0.0,
        -0.125,
        0.0,
    ]
    assert quadruped_parts["leg_front_left_1_lower"]["params"]["position"] == [
        0.0,
        -0.375,
        0.0,
    ]
    assert robot_schema.validate_godot_robot_config(normalized_biped) == []
    assert robot_schema.validate_godot_robot_config(normalized_quadruped) == []


def test_workflow_contract_delegates_to_godot_robot_contract() -> None:
    content = WORKFLOW_CONTRACTS.read_text(encoding="utf-8")
    schema_content = ROBOT_SCHEMA.read_text(encoding="utf-8")

    assert "normalize_robot_config_for_godot" in content
    assert "validate_godot_robot_config" in content
    assert "build_godot_node_tree_manifest" in schema_content
    assert "GODOT_NODE_TREE_MANIFEST_VERSION" in schema_content
    assert '"endpoint_paths_complete": endpoint_paths_complete' in schema_content
    assert '"missing_endpoint_part_ids": missing_endpoint_part_ids' in schema_content
    assert (
        '"missing_endpoint_connection_names": missing_endpoint_connection_names'
        in schema_content
    )
    assert '"missing_endpoint_details": missing_endpoint_details' in schema_content
    assert '"missing_endpoint_fields": [' in schema_content
    assert '"endpoint_parts_exist": not missing_endpoint_part_ids' in schema_content
    assert "node_tree_manifest = robot_schema.build_godot_node_tree_manifest" in (
        VALIDATOR_TOOL.read_text(encoding="utf-8")
    )
    assert "--write-node-tree-manifest" in VALIDATOR_TOOL.read_text(encoding="utf-8")
    assert "--input-kind" in VALIDATOR_TOOL.read_text(encoding="utf-8")
    assert '"node_tree_manifest_output"' in VALIDATOR_TOOL.read_text(encoding="utf-8")
    assert "node_tree_manifest = robot_schema.build_godot_node_tree_manifest" in (
        REPORT_TOOL.read_text(encoding="utf-8")
    )
    assert "--static-node-tree-manifest-dir" in REPORT_TOOL.read_text(encoding="utf-8")
    assert "static_node_tree_manifest_output" in REPORT_TOOL.read_text(
        encoding="utf-8"
    )
    assert '"static_node_tree_manifest_output": report.get(' in REPORT_TOOL.read_text(
        encoding="utf-8"
    )
    assert '"static_node_tree_manifest_outputs": static_node_tree_manifest_outputs' in (
        REPORT_TOOL.read_text(encoding="utf-8")
    )
    assert '"godot_dynamic_required"' in content
    assert 'DEFAULT_FIXED_LIMITS = {"lower": 0.0, "upper": 0.0}' in schema_content
    assert "def _validate_fixed_limits" in schema_content
    assert "def _validate_control" in schema_content
    assert "def _validate_dynamics" in schema_content
    assert "def _is_nonzero_vector3" in schema_content
    assert "ROBOT_MECHANICAL_SCHEMA_VERSION = \"1.5\"" in schema_content
    assert "SUPPORTED_ROBOT_MECHANICAL_SCHEMA_VERSIONS" in schema_content
    assert "SUPPORTED_GODOT_NODE_TREE_MANIFEST_VERSIONS" in schema_content
    assert "def godot_robot_generation_compatibility_matrix" in schema_content
    assert "def _default_limits_for_joint_type" in schema_content


def test_godot_robot_generation_compatibility_matrix_is_explicit() -> None:
    spec = importlib.util.spec_from_file_location("robot_schema", ROBOT_SCHEMA)
    robot_schema = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(robot_schema)

    matrix = robot_schema.godot_robot_generation_compatibility_matrix()

    assert matrix == {
        "robot_mechanical_schema_current": "1.5",
        "robot_mechanical_schema_versions": ["1.1", "1.2", "1.3", "1.4", "1.5"],
        "godot_node_tree_manifest_current": "godot_node_tree_manifest.v1",
        "godot_node_tree_manifest_versions": ["godot_node_tree_manifest.v1"],
        "schema_1_5_optional_fields": {
            "parts": ["material", "physics"],
            "connections": [
                "actuator",
                "sensor",
                "sensors",
                "limits.effort",
                "limits.velocity",
                "controller",
            ],
        },
    }


def test_schema_1_5_optional_mechanical_fields_are_additive() -> None:
    payload = {
        "schema_version": "1.5",
        "name": "schema_1_5_robot",
        "parts": [
            {
                "id": "base",
                "type": "body",
                "shape": "box",
                "params": {"mass": 2.0},
                "material": {"preset": "rubber", "friction": 0.8},
                "physics": {"preset": "stable", "linear_damping": 0.1},
            },
            {"id": "link", "type": "body", "shape": "box", "params": {"mass": 1.0}},
        ],
        "connections": [
            {
                "name": "base_link",
                "from": "base",
                "to": "link",
                "joint_type": "hinge",
                "limits": {
                    "lower": -1.0,
                    "upper": 1.0,
                    "effort": 12.0,
                    "velocity": 3.0,
                },
                "actuator": {"type": "servo", "max_torque": 12.0},
                "sensor": {"type": "encoder", "rate_hz": 120.0},
                "sensors": [{"type": "torque", "noise_stddev": 0.01}],
                "controller": {"kp": 30.0, "ki": 0.0, "kd": 2.0},
            }
        ],
    }

    normalized = robot_schema.normalize_robot_config_for_godot(payload)
    manifest = robot_schema.build_godot_node_tree_manifest(normalized)

    assert normalized["schema_version"] == "1.5"
    assert robot_schema.validate_godot_robot_config(normalized) == []
    assert robot_schema.validate_godot_node_tree_manifest(manifest) == []
    assert manifest["part_nodes"][0]["material"] == {"preset": "rubber", "friction": 0.8}
    applied = manifest["joint_nodes"][0]["applied_parameters"]
    assert applied["limit_effort"] == 12.0
    assert applied["limit_velocity"] == 3.0
    assert applied["actuator"] == {"type": "servo", "max_torque": 12.0}
    assert applied["sensor"] == {"type": "encoder", "rate_hz": 120.0}
    assert applied["sensors"] == [{"type": "torque", "noise_stddev": 0.01}]
    assert applied["controller"] == {"kp": 30.0, "ki": 0.0, "kd": 2.0}


def test_schema_1_5_accepts_older_versions_and_rejects_bad_optional_fields() -> None:
    for schema_version in ["1.1", "1.2", "1.3", "1.4", "1.5"]:
        payload = json.loads(FIXED_PAIR_FIXTURE.read_text(encoding="utf-8"))
        payload["schema_version"] = schema_version
        assert robot_schema.validate_godot_robot_config(payload) == []

    invalid = robot_schema.normalize_robot_config_for_godot(
        {
            "schema_version": "1.5",
            "name": "invalid_schema_1_5_fields",
            "parts": [
                {
                    "id": "base",
                    "type": "body",
                    "shape": "box",
                    "params": {"mass": 1.0},
                    "material": {"preset": "", "friction": -0.1},
                    "physics": {"linear_damping": -0.1},
                },
                {"id": "link", "type": "body", "shape": "box", "params": {"mass": 1.0}},
            ],
            "connections": [
                {
                    "name": "base_link",
                    "from": "base",
                    "to": "link",
                    "joint_type": "hinge",
                    "limits": {
                        "lower": -1.0,
                        "upper": 1.0,
                        "effort": -1.0,
                        "velocity": -1.0,
                    },
                    "actuator": {"type": "", "max_torque": -1.0},
                    "sensor": {"type": "", "rate_hz": -1.0},
                    "sensors": [{"type": "torque", "noise_stddev": -0.1}],
                    "controller": {"kp": -1.0, "target_velocity": "fast"},
                }
            ],
        }
    )

    assert robot_schema.validate_godot_robot_config(invalid) == [
        "parts[1].material.preset must be a non-empty string",
        "parts[1].material.friction must be a non-negative number",
        "parts[1].physics.linear_damping must be a non-negative number",
        "connections[1].limits.effort must be a non-negative number",
        "connections[1].limits.velocity must be a non-negative number",
        "connections[1].actuator.type must be a non-empty string",
        "connections[1].actuator.max_torque must be a non-negative number",
        "connections[1].sensor.type must be a non-empty string",
        "connections[1].sensor.rate_hz must be a non-negative number",
        "connections[1].sensors[1].noise_stddev must be a non-negative number",
        "connections[1].controller.kp must be a non-negative number",
        "connections[1].controller.target_velocity must be a number",
    ]

    invalid["schema_version"] = "2.0"
    assert robot_schema.validate_godot_robot_config(invalid)[0] == (
        "schema_version must be one of ['1.1', '1.2', '1.3', '1.4', '1.5']"
    )


def test_web_panel_godot_delivery_normalizes_artifacts_before_transport() -> None:
    content = WORKFLOWS_API.read_text(encoding="utf-8")
    load_function = content[content.index("def _load_robot_config_from_artifact") :]
    load_function = load_function[
        : load_function.index("def _normalize_transport_mode")
    ]

    assert "from agi_walker.core.api.robot_schema import" in content
    assert "normalize_robot_config_for_godot" in load_function
    assert "return normalize_robot_config_for_godot(payload)" in load_function


def test_web_panel_godot_delivery_persists_assembly_mapping_summary() -> None:
    content = WORKFLOWS_API.read_text(encoding="utf-8")

    assert '"assembly_summary": assembly_summary' in content
    assert "mapping_summary = _build_assembly_mapping_summary(assembly_summary)" in content
    assert (
        "restoration_summary = _build_assembly_restoration_summary(assembly_summary)"
        in content
    )
    assert "fixed_lock_summary = _build_assembly_fixed_lock_summary(assembly_summary)" in content
    assert (
        "static_manifest_evidence = _build_static_node_tree_manifest_evidence"
        in content
    )
    assert '"assembly_mapping_summary": mapping_summary' in content
    assert '"assembly_restoration_summary": restoration_summary' in content
    assert '"assembly_fixed_lock_summary": fixed_lock_summary' in content
    assert '"static_node_tree_manifest_evidence": static_manifest_evidence' in content
    assert '"delivery_acceptance_gate": _build_godot_delivery_acceptance_gate' in content
    assert "fixed_lock_summary=fixed_lock_summary" in content
    assert "def _extract_godot_assembly_summary" in content
    assert "def _build_assembly_mapping_summary" in content
    assert "def _build_assembly_restoration_summary" in content
    assert "def _build_assembly_fixed_lock_summary" in content
    assert "def _build_static_node_tree_manifest_evidence" in content
    assert "def _append_fixed_lock_mismatch" in content
    assert "def _build_godot_delivery_acceptance_gate" in content
    assert '"complete": assembly_summary.get("complete")' in content
    assert '"expected_parts": assembly_summary.get("expected_parts")' in content
    assert '"parts_complete": assembly_summary.get("parts_complete")' in content
    assert '"expected_joints": assembly_summary.get("expected_joints")' in content
    assert '"failed_joints": failed_joints' in content
    assert '"joints_complete": assembly_summary.get("joints_complete")' in content
    assert '"parameterized_joints": assembly_summary.get("parameterized_joints")' in content
    assert '"parameters_complete": assembly_summary.get("parameters_complete")' in content
    assert '"part_nodes_count": len(part_nodes)' in content
    assert '"joint_nodes_count": len(joint_nodes)' in content
    assert 'assembly_summary.get("expected_parts")' in content
    assert 'assembly_summary.get("expected_joints")' in content
    assert 'assembly_summary.get("parts_complete")' in content
    assert 'assembly_summary.get("joints_complete")' in content
    assert 'assembly_summary.get("parameters_complete")' in content
    assert "def _bool_or_default" in content
    assert "def _count_parameterized_joint_nodes" in content
    assert '"source": "assembly_summary"' in content
    assert "expected_parts + expected_joints + expected_joints" in content
    assert "restored_parts + restored_joints + parameterized_joints" in content
    assert '"complete": (' in content
    assert "and parameters_complete" in content
    assert '"schema_meta": schema_meta' in content
    assert '"level": level' in content
    assert '"godot_load_verified"' in content
    assert "fixed_lock_mismatch_count = _int_or_default(" in content
    assert "and fixed_lock_mismatch_count == 0" in content
    assert '"fixed_lock_mismatch"' in content
    assert "fixed joint lock parameter(s) did not match runtime readback" in content
    assert GATE_VALIDATOR_TOOL.exists()
    assert "DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION" in content
    assert "build_delivery_acceptance_requirements" in content
    assert 'WEB_GODOT_DELIVERY_GATE_SOURCE = "web_godot_delivery"' in content
    assert 'WEB_GODOT_DELIVERY_GATE_SCOPE = "godot_load"' in content
    assert "def _web_godot_load_acceptance_requirements" in content
    assert "godot_load=True" in content
    assert "mechanical_restoration_complete=True" in content
    assert "joint_parameter_readback=True" in content
    assert "node_tree_fixed_lock_match=True" in content
    assert '"contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION' in content
    assert '"source": WEB_GODOT_DELIVERY_GATE_SOURCE' in content
    assert '"verification_scope": WEB_GODOT_DELIVERY_GATE_SCOPE' in content
    assert '"acceptance_requirements": _web_godot_load_acceptance_requirements()' in content
    assert "gate_contract_errors = validate_delivery_acceptance_gate(gate)" in content
    assert "delivery_acceptance_gate contract invalid:" in content
    assert '"summary_counts": {' in content
    assert '"inputs_count": 1' in content
    assert '"fixed_lock_checked_count": fixed_lock_counts.get("checked_count", 0)' in content
    assert '"fixed_lock_mismatch_count": fixed_lock_counts.get("mismatch_count", 0)' in content
    assert '"static_node_tree_manifest_count": static_manifest_count' in content
    assert '"static_node_tree_manifest_valid_count": (' in content
    assert '"static_node_tree_manifest_path_map_mismatch_kind_counts": (' in content
    assert '"node_tree_fixed_lock_checked_count": 0' in content
    assert '"node_tree_fixed_lock_mismatch_count": 0' in content
    assert '"node_tree_fixed_locks_complete_count": 0' in content
    assert '"node_tree_fixed_locks_incomplete_count": 0' in content
    assert '"node_tree_gate_enabled_count": 1' in content
    assert '"node_tree_full_restoration_required_count": 0' in content
    assert '"node_tree_full_restoration_not_required_count": 1' in content
    assert '"node_tree_gate_check_counts": {' in content
    assert '"fixed_lock_mismatch": 1' in content
    assert '"mechanical_gate_enabled_count": 2' in content
    assert '"mechanical_restoration": 1' in content
    assert '"joint_parameter_readback": 1' in content
    assert '"checked_count": checked_count' in content
    assert '"mismatch_count": len(mismatches)' in content
    assert '"delivery_godot_verified_count": 1 if complete else 0' in content
    assert '"delivery_unverified_count": 0 if dynamic_generation else 1' in content
    assert '"failure_reasons_count": len(details)' in content


def test_web_panel_workflows_ui_renders_dynamic_assembly_summary() -> None:
    content = WORKFLOWS_HTML.read_text(encoding="utf-8")

    assert "function renderGodotAssemblyPanel(godotDelivery)" in content
    assert "function renderGodotAcceptanceGate(acceptanceGate)" in content
    assert "function renderGodotEvidenceSummary(summary)" in content
    assert "run.godot_evidence_summary" in content
    assert "Godot Evidence" in content
    assert "staticEvidence.manifest_mismatch_count" in content
    assert "staticEvidence.manifest_mismatch_kind_counts" in content
    assert "actions.readiness_summary?.url" in content
    assert "Recommended Sync" in content
    assert "Residual risk:" in content
    assert "assembly_mapping_summary" in content
    assert "assembly_summary" in content
    assert "assembly_fixed_lock_summary" in content
    assert "动态节点树" in content
    assert "Assembly Complete" in content
    assert "Load Gate" in content
    assert "Scope" in content
    assert "Profile" in content
    assert "Requirements" in content
    assert "Version" in content
    assert "acceptanceGate.contract_version" in content
    assert "acceptanceGate.verification_scope" in content
    assert "acceptanceGate.acceptance_profile" in content
    assert "acceptanceGate.acceptance_requirements" in content
    assert "const requirementText = Object.entries(requirements)" in content
    assert "Accepted" in content
    assert "Gate Counts" in content
    assert "Gate Checks" in content
    assert "function formatGateCheckCounts(prefix, checkCounts)" in content
    assert "summaryCounts.node_tree_gate_check_counts" in content
    assert "summaryCounts.mechanical_gate_check_counts" in content
    assert "formatGateCheckCounts('tree', summaryCounts.node_tree_gate_check_counts)" in content
    assert "formatGateCheckCounts('mech', summaryCounts.mechanical_gate_check_counts)" in content
    assert "return `${prefix}[${entries.join(', ')}]`" in content
    assert "const summaryCounts = acceptanceGate.summary_counts || {}" in content
    assert "summaryCounts.delivery_godot_verified_count" in content
    assert "summaryCounts.fixed_lock_checked_count" in content
    assert "summaryCounts.fixed_lock_mismatch_count" in content
    assert "summaryCounts.node_tree_fixed_lock_checked_count" in content
    assert "summaryCounts.node_tree_fixed_lock_mismatch_count" in content
    assert "summaryCounts.node_tree_fixed_locks_complete_count" in content
    assert "summaryCounts.node_tree_fixed_locks_incomplete_count" in content
    assert "summaryCounts.node_tree_gate_enabled_count" in content
    assert "summaryCounts.node_tree_full_restoration_required_count" in content
    assert "treefixed:" in content
    assert "treefixedok:" in content
    assert "treegate:" in content
    assert "summaryCounts.failure_reasons_count" in content
    assert "Reason Codes" in content
    assert "Affected Inputs" in content
    assert "delivery_acceptance_gate" in content
    assert "const hasAssemblyReport = Boolean" in content
    assert "if (!hasAssemblyReport && !hasAcceptanceGate)" in content
    assert "const acceptanceGatePanel = renderGodotAcceptanceGate(acceptanceGate)" in content
    assert "if (!hasAssemblyReport)" in content
    assert "Part Map" in content
    assert "Joint Map" in content
    assert "Param Joints" in content
    assert "mapping.expected_parts" in content
    assert "mapping.expected_joints" in content
    assert "mapping.parameters_complete" in content
    assert "Max Endpoint" in content
    assert "Avg Endpoint" in content
    assert "Threshold" in content
    assert "Missing Endpoints" in content
    assert "Farthest Joint" in content
    assert "Max Angle" in content
    assert "Avg Angle" in content
    assert "Angle Limit" in content
    assert "Largest Angle" in content
    assert "Limit Joints" in content
    assert "Violations" in content
    assert "Worst Joint" in content
    assert "joint_limit_summary" in content
    assert "Parameterized" in content
    assert "Limit Params" in content
    assert "Motor Enabled" in content
    assert "Damping Applied" in content
    assert "joint_parameter_summary" in content
    assert "Param Checked" in content
    assert "Param Mismatch" in content
    assert "Fixed Locks" in content
    assert "Fixed Mismatch" in content
    assert "Load Fixed Locks" in content
    assert "Load Fixed Mismatch" in content
    assert "Load Fixed Match" in content
    assert "const assemblyFixedLockSummary = godotDelivery?.assembly_fixed_lock_summary || {}" in content
    assert "fixed_lock_checked_count" in content
    assert "fixed_lock_mismatch_count" in content
    assert "Param Tolerance" in content
    assert "renderParameterMismatchList" in content
    assert "renderNodeTreeMismatchList" in content
    assert "expected ${escapeHtml(expected)} / runtime ${escapeHtml(actual)}" in content
    assert "delta ${formatMetric(item.max_delta)}" in content
    assert "请查看 smoke JSON 报告" in content
    assert "joint_parameter_consistency_summary" in content
    assert "node_tree_manifest" in content
    assert "Tree Complete" in content
    assert "Missing Parts" in content
    assert "Missing Joints" in content
    assert "Class Mismatch" in content
    assert "Missing Params" in content
    assert "Tree Fixed Locks" in content
    assert "Tree Fixed Mismatch" in content
    assert "nodeTreeFixedLockMismatch" in content
    assert "fixed_lock_mismatches" in content
    assert "Transform Mismatch" in content
    assert "Physical Mismatch" in content
    assert "class_mismatches" in content
    assert "transform_mismatches" in content
    assert "physical_mismatches" in content
    assert "Targeted" in content
    assert "Max Target" in content
    assert "Nonzero Gate" in content
    assert "Largest Target" in content
    assert "joint_control_summary" in content
    assert "Control Checked" in content
    assert "Control Mismatch" in content
    assert "joint_control_consistency_summary" in content
    assert "Action Targets" in content
    assert "Target Mismatch" in content
    assert "Unknown Targets" in content
    assert "Invalid Targets" in content
    assert "action_target_consistency_summary" in content
    assert "Seq Targets" in content
    assert "Seq Mismatch" in content
    assert "Seq Unknown" in content
    assert "Seq Invalid" in content
    assert "action_sequence_target_consistency_summary" in content
    assert "Action Coverage" in content
    assert "Covered Joints" in content
    assert "Missing Targets" in content
    assert "action_target_coverage_summary" in content
    assert "Control Coverage" in content
    assert "Covered Controls" in content
    assert "Untargeted Controls" in content
    assert "control_action_coverage_summary" in content
    assert "function formatUnknownActionTargets(items)" in content
    assert "Steps Run" in content
    assert "Max Move" in content
    assert "Max Speed" in content
    assert "Min Move Gate" in content
    assert "Max Speed Gate" in content
    assert "displacement_under_min" in content
    assert "speed_threshold_exceeded" in content
    assert "simulation_summary" in content
    assert "Joint Motion" in content
    assert "Max Joint Delta" in content
    assert "Joint Delta Gate" in content
    assert "Max Joint Range" in content
    assert "Joint Range Gate" in content
    assert "Motion Coverage" in content
    assert "Motion Gate" in content
    assert "Command Response" in content
    assert "Response Gate" in content
    assert "未响应的 action 关节" in content
    assert "function renderCommandedJointResponseDetails(jointMotionSummary)" in content
    assert "commanded_joint_response_details" in content
    assert "responded" in content
    assert "joint_motion_summary" in content
    assert "angle_delta_under_min" in content
    assert "angle_range_under_min" in content
    assert "moving_joint_coverage_under_min" in content
    assert "commanded_joint_response_under_min" in content
    assert "commanded_static_joints" in content
    assert "Action Steps" in content
    assert "Unique Actions" in content
    assert "Transitions" in content
    assert "Transition Gate" in content
    assert "Max Delta" in content
    assert "Avg Delta" in content
    assert "Delta Gate" in content
    assert "transition_delta_under_min" in content
    assert "action_sequence_summary" in content
    assert "Restored Parts" in content
    assert "Restored Joints" in content
    assert "Restore Score" in content
    assert "Restore Gate" in content
    assert "score_under_min" in content
    assert "mechanical_restoration_summary" in content
    assert "assembly_restoration_summary" in content
    assert "function formatPercent(value)" in content
    assert "threshold_exceeded" in content
    assert "function formatMetric(value)" in content
    assert "failed_connections" in content
    assert "warnings" in content


def test_godot_assembler_materializes_bodies_joints_and_controller() -> None:
    content = ASSEMBLER.read_text(encoding="utf-8")

    assert "class_name RobotAssembler" in content
    assert "RigidBody3D.new()" in content
    assert "CollisionShape3D.new()" in content
    assert "MeshInstance3D.new()" in content
    assert "HingeJoint3D.new()" in content
    assert "SliderJoint3D.new()" in content
    assert "controller.configure(robot" in content
    assert '"expected_parts": expected_parts' in content
    assert '"parts_created": bodies.size()' in content
    assert "var parts_complete = bodies.size() == expected_parts" in content
    assert '"parts_complete": parts_complete' in content
    assert '"expected_joints": expected_joints' in content
    assert '"joints_created": joint_summary["created"]' in content
    assert '"failed_joints": failed_joint_count' in content
    assert (
        'var joints_complete = joint_summary["created"] == expected_joints '
        'and failed_joint_count == 0' in content
    )
    assert '"joints_complete": joints_complete' in content
    assert '"parameterized_joints": parameterized_joints' in content
    assert 'var parameters_complete = parameterized_joints == joint_summary["created"]' in content
    assert '"parameters_complete": parameters_complete' in content
    assert "var assembly_complete = parts_complete and joints_complete and parameters_complete" in content
    assert '"complete": assembly_complete' in content
    assert '"part_nodes": body_summary["part_nodes"]' in content
    assert '"joint_nodes": joint_summary["joint_nodes"]' in content
    assert "func _valid_part_count(parts: Array) -> int" in content
    assert "func _valid_connection_count(connections: Array) -> int" in content
    assert "func _parameterized_joint_count(joint_nodes: Array) -> int" in content
    assert 'joint.set_meta("applied_parameters", _joint_applied_params(connection, joint))' in content
    assert '"applied_parameters": joint.get_meta("applied_parameters", {})' in content
    assert "func _joint_applied_params(connection: Dictionary, joint: Joint3D) -> Dictionary" in content
    assert "func _apply_fixed_params(joint: Generic6DOFJoint3D) -> void" in content
    assert "Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT" in content
    assert "Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT" in content
    assert "Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT" in content
    assert "Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT" in content
    assert "Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT" in content
    assert "Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT" in content
    assert "func _generic_axis_flags(joint: Generic6DOFJoint3D, flag: int) -> Dictionary" in content
    assert "func _generic_axis_params(joint: Generic6DOFJoint3D, param: int) -> Dictionary" in content
    assert '"fixed_lock_applied": true' in content
    assert '"dynamics_configured": dynamics is Dictionary and not dynamics.is_empty()' in content
    assert '"damping_applied": false' in content
    assert "func _body_mapping(part: Dictionary, body: RigidBody3D)" in content
    assert "func _joint_mapping(connection: Dictionary, joint: Joint3D)" in content
    assert '"rotation": [body.rotation.x, body.rotation.y, body.rotation.z]' in content
    assert '"axis": connection.get("axis", [0.0, 1.0, 0.0])' in content


def test_generated_robot_controller_exposes_motion_and_telemetry_hooks() -> None:
    content = CONTROLLER.read_text(encoding="utf-8")

    assert "class_name GeneratedRobotController" in content
    assert "func apply_action(action)" in content
    assert "func apply_instruction_steps(steps: Array)" in content
    assert "func reset_pose()" in content
    assert "func get_sensor_data() -> Dictionary" in content
    assert "PARAM_MOTOR_TARGET_VELOCITY" in content
    assert '"body_states": _body_states(bodies)' in content
    assert '"joint_states": _joint_states()' in content
    assert "func _body_states(bodies: Array) -> Dictionary" in content
    assert "func _joint_states() -> Dictionary" in content
    assert '"endpoint_distance": _joint_endpoint_distance(joint)' in content
    assert '"relative_angle": relative_angle' in content
    assert '"limits": _joint_limit_state(relative_angle, connection_config)' in content
    assert '"applied_parameters": _applied_parameters_with_runtime(joint)' in content
    assert '"control_parameters": joint.get_meta("control", {}).duplicate(true)' in content
    assert '"body_a": _joint_endpoint_state(joint.node_a)' in content
    assert '"body_b": _joint_endpoint_state(joint.node_b)' in content
    assert "joint_targets[node.name] = _initial_joint_target(node)" in content
    assert "func _initial_joint_target(joint: Joint3D) -> float" in content
    assert "func _applied_parameters_with_runtime(joint: Joint3D) -> Dictionary" in content
    assert (
        "func _joint_limit_state(relative_angle: float, connection_config: Dictionary) -> Dictionary"
        in content
    )
    assert '"violation": relative_angle < lower or relative_angle > upper' in content
    assert "func _joint_endpoint_state(endpoint_path: NodePath) -> Dictionary" in content
    assert "func _joint_relative_angle(joint: Joint3D) -> float" in content
    assert "func _project_on_plane(value: Vector3, normal: Vector3) -> Vector3" in content
    assert "func _resolve_node_path(path: NodePath) -> Node" in content


def test_tcp_server_load_robot_uses_dynamic_assembler_before_fallback() -> None:
    content = TCP_SERVER.read_text(encoding="utf-8")
    dynamic_load = content.index("robot_assembler.build_from_config")
    fallback_load = content.index("robot_node.load_from_dict")

    assert "preload(\"res://scripts/robot_assembler.gd\")" in content
    assert dynamic_load < fallback_load
    assert "generated_controller.apply_action(action)" in content
    assert "generated_controller.apply_instruction_steps(steps)" in content
    assert '"dynamic_robot_generation": true' in content
    assert '"body_states": {"type": "dict"}' in content
    assert '"joint_states": {"type": "dict"}' in content


def test_validator_tool_writes_normalized_godot_config(tmp_path: Path) -> None:
    output_path = tmp_path / "normalized.json"
    manifest_output_path = tmp_path / "node_tree_manifest.json"

    result = subprocess.run(
        [
            sys.executable,
            str(VALIDATOR_TOOL),
            str(FIXTURE),
            "--write-normalized",
            str(output_path),
            "--write-node-tree-manifest",
            str(manifest_output_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    payload = json.loads(result.stdout)
    normalized = json.loads(output_path.read_text(encoding="utf-8"))
    manifest_output = json.loads(manifest_output_path.read_text(encoding="utf-8"))
    assert payload["status"] == "success"
    assert payload["input_kind"] == "robot"
    assert payload["normalized_output"] == str(output_path)
    assert payload["node_tree_manifest_output"] == str(manifest_output_path)
    assert payload["node_tree_manifest_errors"] == []
    assert payload["node_tree_manifest_path_map_mismatches"] == []
    assert payload["schema_version"] == robot_schema.ROBOT_MECHANICAL_SCHEMA_VERSION
    assert payload["parts_count"] == 3
    assert payload["topology_summary"] == {
        "schema_version": robot_schema.ROBOT_MECHANICAL_SCHEMA_VERSION,
        "parts_count": 3,
        "connections_count": 2,
        "root_parts": ["torso_1"],
        "root_part": "torso_1",
        "connected_parts_count": 3,
        "reachable_parts_count": 3,
        "disconnected_parts": [],
        "unreachable_parts": [],
        "duplicate_child_endpoints": [],
        "cycle": [],
        "complete_tree": True,
    }
    assert payload["node_tree_manifest"] == {
        "manifest_version": "godot_node_tree_manifest.v1",
        "schema_version": robot_schema.ROBOT_MECHANICAL_SCHEMA_VERSION,
        "robot_name": "dynamic_biped",
        "robot_node": "dynamic_biped",
        "controller_node": "dynamic_biped/GeneratedRobotController",
        "parts_count": 3,
        "joints_count": 2,
        "parameterized_joints": 2,
        "parameters_complete": True,
        "endpoint_paths_complete": True,
        "path_maps_complete": True,
        "missing_endpoint_part_ids": [],
        "missing_endpoint_connection_names": [],
        "missing_endpoint_details": [],
        "complete": True,
        "part_node_paths": {
            "torso_1": {
                "body_node": "dynamic_biped/torso_1",
                "collision_node": "dynamic_biped/torso_1/Collision",
                "mesh_node": "dynamic_biped/torso_1/Mesh",
            },
            "thigh_left": {
                "body_node": "dynamic_biped/thigh_left",
                "collision_node": "dynamic_biped/thigh_left/Collision",
                "mesh_node": "dynamic_biped/thigh_left/Mesh",
            },
            "shin_left": {
                "body_node": "dynamic_biped/shin_left",
                "collision_node": "dynamic_biped/shin_left/Collision",
                "mesh_node": "dynamic_biped/shin_left/Mesh",
            },
        },
        "joint_node_paths": {
            "hip_left": {
                "joint_node": "dynamic_biped/hip_left",
                "node_a": "dynamic_biped/torso_1",
                "node_b": "dynamic_biped/thigh_left",
            },
            "knee_left": {
                "joint_node": "dynamic_biped/knee_left",
                "node_a": "dynamic_biped/thigh_left",
                "node_b": "dynamic_biped/shin_left",
            },
        },
        "part_nodes": [
            {
                "part_id": "torso_1",
                "part_type": "torso",
                "shape": "box",
                "body_node": "dynamic_biped/torso_1",
                "body_class": "RigidBody3D",
                "collision_node": "dynamic_biped/torso_1/Collision",
                "collision_shape": "BoxShape3D",
                "collision_parameters": {"size": [0.3, 0.2, 0.5]},
                "mesh_node": "dynamic_biped/torso_1/Mesh",
                "mesh_type": "BoxMesh",
                "mesh_parameters": {"size": [0.3, 0.2, 0.5]},
                "mass": 5.0,
                "position": [0.0, 1.2, 0.0],
                "rotation": [0.0, 0.0, 0.0],
            },
            {
                "part_id": "thigh_left",
                "part_type": "thigh",
                "shape": "capsule",
                "body_node": "dynamic_biped/thigh_left",
                "body_class": "RigidBody3D",
                "collision_node": "dynamic_biped/thigh_left/Collision",
                "collision_shape": "CapsuleShape3D",
                "collision_parameters": {"radius": 0.04, "height": 0.3},
                "mesh_node": "dynamic_biped/thigh_left/Mesh",
                "mesh_type": "CapsuleMesh",
                "mesh_parameters": {"radius": 0.04, "height": 0.3},
                "mass": 1.0,
                "position": [-0.15, 0.85, 0.0],
                "rotation": [0.0, 0.0, 0.0],
            },
            {
                "part_id": "shin_left",
                "part_type": "shin",
                "shape": "capsule",
                "body_node": "dynamic_biped/shin_left",
                "body_class": "RigidBody3D",
                "collision_node": "dynamic_biped/shin_left/Collision",
                "collision_shape": "CapsuleShape3D",
                "collision_parameters": {"radius": 0.035, "height": 0.3},
                "mesh_node": "dynamic_biped/shin_left/Mesh",
                "mesh_type": "CapsuleMesh",
                "mesh_parameters": {"radius": 0.035, "height": 0.3},
                "mass": 0.8,
                "position": [-0.15, 0.5, 0.0],
                "rotation": [0.0, 0.0, 0.0],
            },
        ],
        "joint_nodes": [
            {
                "connection_name": "hip_left",
                "joint_node": "dynamic_biped/hip_left",
                "joint_class": "HingeJoint3D",
                "joint_type": "hinge",
                "from": "torso_1",
                "to": "thigh_left",
                "endpoint_parts_exist": True,
                "missing_endpoint_part_ids": [],
                "missing_endpoint_fields": [],
                "missing_endpoint_details": [],
                "node_a": "dynamic_biped/torso_1",
                "node_b": "dynamic_biped/thigh_left",
                "origin": [-0.15, 1.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "applied_parameters": {
                    "limit_enabled": True,
                    "limit_lower": -1.2,
                    "limit_upper": 1.2,
                    "motor_enabled": True,
                    "motor_target_velocity": 0.0,
                    "motor_max_impulse": 120.0,
                },
            },
            {
                "connection_name": "knee_left",
                "joint_node": "dynamic_biped/knee_left",
                "joint_class": "HingeJoint3D",
                "joint_type": "hinge",
                "from": "thigh_left",
                "to": "shin_left",
                "endpoint_parts_exist": True,
                "missing_endpoint_part_ids": [],
                "missing_endpoint_fields": [],
                "missing_endpoint_details": [],
                "node_a": "dynamic_biped/thigh_left",
                "node_b": "dynamic_biped/shin_left",
                "origin": [-0.15, 0.68, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "applied_parameters": {
                    "limit_enabled": True,
                    "limit_lower": -1.5,
                    "limit_upper": 0.2,
                    "motor_enabled": True,
                    "motor_target_velocity": 0.0,
                    "motor_max_impulse": 90.0,
                },
            },
        ],
    }
    assert manifest_output == payload["node_tree_manifest"]
    assert normalized["parts"][0]["shape"] == "box"
    assert robot_schema.validate_godot_robot_config(normalized) == []
    assert robot_schema.validate_godot_node_tree_manifest(manifest_output) == []


def _runtime_joint_nodes_for_manifest(manifest: dict[str, object]) -> list[dict[str, object]]:
    joints = json.loads(json.dumps(manifest["joint_nodes"]))
    for joint in joints:
        joint["applied_parameters"] = {
            "runtime": joint["applied_parameters"],
            "source": {},
        }
    return joints


def test_godot_node_tree_manifest_runtime_comparison_accepts_exact_mapping() -> None:
    normalized = robot_schema.normalize_robot_config_for_godot(
        json.loads(FIXED_PAIR_FIXTURE.read_text(encoding="utf-8"))
    )
    manifest = robot_schema.build_godot_node_tree_manifest(normalized)

    summary = robot_schema.compare_godot_node_tree_manifest_to_runtime(
        manifest,
        json.loads(json.dumps(manifest["part_nodes"])),
        _runtime_joint_nodes_for_manifest(manifest),
    )

    assert summary["static_manifest_version"] == "godot_node_tree_manifest.v1"
    assert summary["complete"] is True
    assert summary["mismatch_count"] == 0
    assert summary["mismatch_kind_counts"] == {}


def test_godot_node_tree_manifest_runtime_comparison_accepts_runtime_root_prefix() -> None:
    normalized = robot_schema.normalize_robot_config_for_godot(
        json.loads(FIXED_PAIR_FIXTURE.read_text(encoding="utf-8"))
    )
    manifest = robot_schema.build_godot_node_tree_manifest(normalized)
    runtime_parts = json.loads(json.dumps(manifest["part_nodes"]))
    runtime_joints = _runtime_joint_nodes_for_manifest(manifest)
    for part in runtime_parts:
        for field in ["body_node", "collision_node", "mesh_node"]:
            part[field] = f"/root/RLServer/{part[field]}"
    for joint in runtime_joints:
        for field in ["joint_node", "node_a", "node_b"]:
            joint[field] = f"/root/RLServer/{joint[field]}"

    summary = robot_schema.compare_godot_node_tree_manifest_to_runtime(
        manifest,
        runtime_parts,
        runtime_joints,
    )

    assert summary["complete"] is True
    assert summary["mismatch_count"] == 0
    assert summary["mismatch_kind_counts"] == {}


def test_godot_node_tree_manifest_runtime_comparison_reports_missing_node() -> None:
    normalized = robot_schema.normalize_robot_config_for_godot(
        json.loads(FIXED_PAIR_FIXTURE.read_text(encoding="utf-8"))
    )
    manifest = robot_schema.build_godot_node_tree_manifest(normalized)

    summary = robot_schema.compare_godot_node_tree_manifest_to_runtime(
        manifest,
        [],
        _runtime_joint_nodes_for_manifest(manifest),
    )

    assert summary["complete"] is False
    assert summary["mismatch_kind_counts"]["missing"] == manifest["parts_count"]
    assert summary["mismatches"][0]["map"] == "part_nodes"


def test_godot_node_tree_manifest_runtime_comparison_reports_class_mismatch() -> None:
    normalized = robot_schema.normalize_robot_config_for_godot(
        json.loads(FIXED_PAIR_FIXTURE.read_text(encoding="utf-8"))
    )
    manifest = robot_schema.build_godot_node_tree_manifest(normalized)
    runtime_parts = json.loads(json.dumps(manifest["part_nodes"]))
    runtime_parts[0]["body_class"] = "Node3D"

    summary = robot_schema.compare_godot_node_tree_manifest_to_runtime(
        manifest,
        runtime_parts,
        _runtime_joint_nodes_for_manifest(manifest),
    )

    assert summary["mismatch_count"] == 1
    assert summary["mismatches"][0]["field"] == "body_class"
    assert summary["mismatches"][0]["kind"] == "value_mismatch"


def test_godot_node_tree_manifest_runtime_comparison_reports_physical_parameter_mismatch() -> None:
    normalized = robot_schema.normalize_robot_config_for_godot(
        json.loads(FIXED_PAIR_FIXTURE.read_text(encoding="utf-8"))
    )
    manifest = robot_schema.build_godot_node_tree_manifest(normalized)
    runtime_parts = json.loads(json.dumps(manifest["part_nodes"]))
    runtime_parts[0]["mass"] += 0.5
    runtime_parts[0]["collision_parameters"]["size"][0] += 0.25
    runtime_parts[0]["mesh_parameters"]["size"][1] += 0.25

    summary = robot_schema.compare_godot_node_tree_manifest_to_runtime(
        manifest,
        runtime_parts,
        _runtime_joint_nodes_for_manifest(manifest),
    )

    assert summary["complete"] is False
    assert summary["mismatch_count"] == 3
    assert summary["mismatch_kind_counts"] == {"value_mismatch": 3}
    assert [mismatch["field"] for mismatch in summary["mismatches"]] == [
        "collision_parameters.size",
        "mesh_parameters.size",
        "mass",
    ]


def test_godot_node_tree_manifest_runtime_comparison_respects_tolerance() -> None:
    normalized = robot_schema.normalize_robot_config_for_godot(
        json.loads(FIXED_PAIR_FIXTURE.read_text(encoding="utf-8"))
    )
    manifest = robot_schema.build_godot_node_tree_manifest(normalized)
    runtime_parts = json.loads(json.dumps(manifest["part_nodes"]))
    runtime_parts[0]["position"][0] += 0.01

    loose = robot_schema.compare_godot_node_tree_manifest_to_runtime(
        manifest,
        runtime_parts,
        _runtime_joint_nodes_for_manifest(manifest),
        tolerance=0.02,
    )
    strict = robot_schema.compare_godot_node_tree_manifest_to_runtime(
        manifest,
        runtime_parts,
        _runtime_joint_nodes_for_manifest(manifest),
        tolerance=0.001,
    )

    assert loose["complete"] is True
    assert strict["complete"] is False
    assert strict["mismatches"][0]["field"] == "position"
    assert strict["mismatches"][0]["max_delta"] > 0.001


def test_godot_node_tree_manifest_validator_reports_inconsistent_shapes() -> None:
    payload = json.loads(FIXTURE.read_text(encoding="utf-8"))
    normalized = robot_schema.normalize_robot_config_for_godot(payload)
    manifest = robot_schema.build_godot_node_tree_manifest(normalized)

    assert robot_schema.validate_godot_node_tree_manifest(manifest) == []

    broken = json.loads(json.dumps(manifest))
    broken["parts_count"] = 99
    broken["joints_count"] = 99
    broken["complete"] = True
    broken["endpoint_paths_complete"] = False
    broken["path_maps_complete"] = "bad"
    broken["missing_endpoint_part_ids"] = ["ghost"]
    broken["missing_endpoint_connection_names"] = ["ghost_joint"]
    broken["missing_endpoint_details"] = [
        {"connection_name": "ghost_joint", "field": "to", "part_id": "ghost"}
    ]
    broken["part_nodes"] = "bad"
    broken["joint_nodes"] = "bad"

    errors = robot_schema.validate_godot_node_tree_manifest(broken)

    assert "node_tree_manifest.parts_count must equal 0" in errors
    assert "node_tree_manifest.joints_count must equal 0" in errors
    assert "node_tree_manifest.part_nodes must be a list" in errors
    assert "node_tree_manifest.joint_nodes must be a list" in errors
    assert (
        "node_tree_manifest.complete must equal parameters_complete and "
        "endpoint_paths_complete"
    ) in errors
    assert (
        "node_tree_manifest.endpoint_paths_complete must match missing endpoint part ids"
        in errors
    )
    assert "node_tree_manifest.path_maps_complete must be a boolean" in errors
    assert (
        "node_tree_manifest.missing_endpoint_part_ids must match joint missing endpoints"
        in errors
    )
    assert (
        "node_tree_manifest.missing_endpoint_connection_names must match joint missing endpoints"
        in errors
    )
    assert (
        "node_tree_manifest.missing_endpoint_details must match joint missing endpoints"
        in errors
    )


def test_godot_node_tree_manifest_validator_reports_node_field_errors() -> None:
    payload = json.loads(FIXED_PAIR_FIXTURE.read_text(encoding="utf-8"))
    normalized = robot_schema.normalize_robot_config_for_godot(payload)
    manifest = robot_schema.build_godot_node_tree_manifest(normalized)
    broken = json.loads(json.dumps(manifest))
    broken["part_nodes"][0]["body_class"] = "Node3D"
    broken["part_nodes"][0]["mass"] = -1.0
    broken["part_nodes"][0]["position"] = [0.0, 0.0]
    broken["joint_nodes"][0]["joint_node"] = ""
    broken["joint_nodes"][0]["origin"] = [0.0]
    broken["joint_nodes"][0]["applied_parameters"] = "bad"
    broken["part_node_paths"]["base"]["body_node"] = "wrong/path"
    broken["joint_node_paths"]["base_payload_fixed"]["node_a"] = "wrong/path"
    broken["path_maps_complete"] = True

    errors = robot_schema.validate_godot_node_tree_manifest(broken)

    assert "node_tree_manifest.part_nodes[1].body_class must be 'RigidBody3D'" in errors
    assert "node_tree_manifest.part_nodes[1].mass must be a non-negative number" in errors
    assert (
        "node_tree_manifest.part_nodes[1].position must be a 3-number sequence"
        in errors
    )
    assert (
        "node_tree_manifest.joint_nodes[1].joint_node must be a non-empty string"
        in errors
    )
    assert (
        "node_tree_manifest.joint_nodes[1].origin must be a 3-number sequence"
        in errors
    )
    assert (
        "node_tree_manifest.joint_nodes[1].applied_parameters must be an object"
        in errors
    )
    assert "node_tree_manifest.part_node_paths must match part_nodes" in errors
    assert "node_tree_manifest.joint_node_paths must match joint_nodes" in errors
    assert (
        "node_tree_manifest.part_node_paths.base.body_node must equal "
        "'dynamic_fixed_pair/base'"
    ) in errors
    assert (
        "node_tree_manifest.joint_node_paths.base_payload_fixed is unexpected"
    ) in errors
    assert (
        "node_tree_manifest.path_maps_complete must match path map counts" in errors
    )
    assert robot_schema.build_godot_node_tree_manifest_path_map_mismatches(
        broken
    ) == [
        {
            "map": "part_node_paths",
            "key": "base",
            "field": "body_node",
            "kind": "value_mismatch",
            "expected": "dynamic_fixed_pair/base",
            "actual": "wrong/path",
        },
        {"map": "joint_node_paths", "key": "base_payload_fixed", "kind": "unexpected"},
    ]


def test_godot_node_tree_manifest_validator_rejects_duplicate_node_keys() -> None:
    payload = json.loads(FIXED_PAIR_FIXTURE.read_text(encoding="utf-8"))
    normalized = robot_schema.normalize_robot_config_for_godot(payload)
    manifest = robot_schema.build_godot_node_tree_manifest(normalized)
    broken = json.loads(json.dumps(manifest))
    broken["part_nodes"].append(json.loads(json.dumps(broken["part_nodes"][0])))
    broken["joint_nodes"].append(json.loads(json.dumps(broken["joint_nodes"][0])))
    broken["parts_count"] = len(broken["part_nodes"])
    broken["joints_count"] = len(broken["joint_nodes"])
    broken["parameterized_joints"] = len(broken["joint_nodes"])
    broken["path_maps_complete"] = True

    errors = robot_schema.validate_godot_node_tree_manifest(broken)

    assert "node_tree_manifest.part_nodes.part_id duplicates 'base'" in errors
    assert (
        "node_tree_manifest.joint_nodes.connection_name duplicates "
        "'base_payload_fixed'"
    ) in errors
    assert (
        "node_tree_manifest.path_maps_complete must match path map counts" in errors
    )
    assert robot_schema.build_godot_node_tree_manifest_path_map_mismatches(
        broken
    ) == [
        {
            "map": "part_node_paths",
            "key": "base",
            "field": "part_id",
            "kind": "duplicate",
        },
        {
            "map": "joint_node_paths",
            "key": "base_payload_fixed",
            "field": "connection_name",
            "kind": "duplicate",
        },
    ]


def test_godot_node_tree_manifest_validator_rejects_root_path_drift() -> None:
    payload = json.loads(FIXED_PAIR_FIXTURE.read_text(encoding="utf-8"))
    normalized = robot_schema.normalize_robot_config_for_godot(payload)
    manifest = robot_schema.build_godot_node_tree_manifest(normalized)
    broken = json.loads(json.dumps(manifest))
    broken["controller_node"] = "dynamic_fixed_pair/Controller"
    broken["part_nodes"][0]["body_node"] = "wrong_robot/base"
    broken["part_nodes"][0]["collision_node"] = "wrong_robot/base/Collision"
    broken["part_nodes"][0]["mesh_node"] = "wrong_robot/base/Mesh"
    broken["joint_nodes"][0]["joint_node"] = "wrong_robot/base_payload_fixed"
    broken["joint_nodes"][0]["node_a"] = "wrong_robot/base"
    broken["joint_nodes"][0]["node_b"] = "wrong_robot/payload"
    broken["part_node_paths"]["base"] = {
        "body_node": "wrong_robot/base",
        "collision_node": "wrong_robot/base/Collision",
        "mesh_node": "wrong_robot/base/Mesh",
    }
    broken["joint_node_paths"]["base_payload_fixed"] = {
        "joint_node": "wrong_robot/base_payload_fixed",
        "node_a": "wrong_robot/base",
        "node_b": "wrong_robot/payload",
    }

    errors = robot_schema.validate_godot_node_tree_manifest(broken)

    assert (
        "node_tree_manifest.controller_node must equal "
        "'dynamic_fixed_pair/GeneratedRobotController'"
    ) in errors
    assert (
        "node_tree_manifest.part_nodes[1].body_node must equal "
        "'dynamic_fixed_pair/base'"
    ) in errors
    assert (
        "node_tree_manifest.part_nodes[1].collision_node must equal "
        "'dynamic_fixed_pair/base/Collision'"
    ) in errors
    assert (
        "node_tree_manifest.part_nodes[1].mesh_node must equal "
        "'dynamic_fixed_pair/base/Mesh'"
    )
    assert (
        "node_tree_manifest.joint_nodes[1].joint_node must equal "
        "'dynamic_fixed_pair/base_payload_fixed'"
    ) in errors
    assert (
        "node_tree_manifest.joint_nodes[1].node_a must equal "
        "'dynamic_fixed_pair/base'"
    )
    assert (
        "node_tree_manifest.joint_nodes[1].node_b must equal "
        "'dynamic_fixed_pair/payload'"
    ) in errors

    root_broken = json.loads(json.dumps(manifest))
    root_broken["robot_node"] = "wrong_robot"

    assert (
        "node_tree_manifest.robot_node must equal robot_name"
        in robot_schema.validate_godot_node_tree_manifest(root_broken)
    )
    assert robot_schema.build_godot_node_tree_manifest_path_map_mismatches(
        broken
    ) == [
        {
            "map": "part_nodes",
            "key": "base",
            "field": "body_node",
            "kind": "root_mismatch",
            "expected": "dynamic_fixed_pair/base",
            "actual": "wrong_robot/base",
        },
        {
            "map": "part_nodes",
            "key": "base",
            "field": "collision_node",
            "kind": "root_mismatch",
            "expected": "dynamic_fixed_pair/base/Collision",
            "actual": "wrong_robot/base/Collision",
        },
        {
            "map": "part_nodes",
            "key": "base",
            "field": "mesh_node",
            "kind": "root_mismatch",
            "expected": "dynamic_fixed_pair/base/Mesh",
            "actual": "wrong_robot/base/Mesh",
        },
        {
            "map": "joint_nodes",
            "key": "base_payload_fixed",
            "field": "joint_node",
            "kind": "root_mismatch",
            "expected": "dynamic_fixed_pair/base_payload_fixed",
            "actual": "wrong_robot/base_payload_fixed",
        },
        {
            "map": "joint_nodes",
            "key": "base_payload_fixed",
            "field": "node_a",
            "kind": "root_mismatch",
            "expected": "dynamic_fixed_pair/base",
            "actual": "wrong_robot/base",
        },
        {
            "map": "joint_nodes",
            "key": "base_payload_fixed",
            "field": "node_b",
            "kind": "root_mismatch",
            "expected": "dynamic_fixed_pair/payload",
            "actual": "wrong_robot/payload",
        },
    ]


def test_negative_robot_schema_fixtures_cover_duplicate_ids() -> None:
    duplicate_parts = json.loads(DUPLICATE_PART_IDS_FIXTURE.read_text(encoding="utf-8"))
    duplicate_connections = json.loads(
        DUPLICATE_CONNECTION_NAMES_FIXTURE.read_text(encoding="utf-8")
    )

    duplicate_part_errors = robot_schema.validate_godot_robot_config(
        robot_schema.normalize_robot_config_for_godot(duplicate_parts)
    )
    duplicate_connection_errors = robot_schema.validate_godot_robot_config(
        robot_schema.normalize_robot_config_for_godot(duplicate_connections)
    )

    assert "parts[2].id duplicates 'base'" in duplicate_part_errors
    assert (
        "connections[2].name duplicates 'base_payload'" in duplicate_connection_errors
    )


def test_negative_manifest_fixture_covers_root_drift() -> None:
    manifest = json.loads(ROOT_DRIFT_MANIFEST_FIXTURE.read_text(encoding="utf-8"))

    errors = robot_schema.validate_godot_node_tree_manifest(manifest)
    mismatches = robot_schema.build_godot_node_tree_manifest_path_map_mismatches(
        manifest
    )

    assert "node_tree_manifest.part_nodes[1].body_node must equal 'root_drift/base'" in errors
    assert {
        "map": "part_nodes",
        "key": "base",
        "field": "body_node",
        "kind": "root_mismatch",
        "expected": "root_drift/base",
        "actual": "OtherRoot/base",
    } in mismatches


def test_godot_node_tree_manifest_validator_reports_path_map_field_mismatches() -> None:
    payload = json.loads(FIXED_PAIR_FIXTURE.read_text(encoding="utf-8"))
    normalized = robot_schema.normalize_robot_config_for_godot(payload)
    manifest = robot_schema.build_godot_node_tree_manifest(normalized)
    broken = json.loads(json.dumps(manifest))
    broken["joint_node_paths"]["base_payload_fixed"]["node_a"] = "wrong/path"
    broken["path_maps_complete"] = True

    errors = robot_schema.validate_godot_node_tree_manifest(broken)

    assert "node_tree_manifest.joint_node_paths must match joint_nodes" in errors
    assert (
        "node_tree_manifest.joint_node_paths.base_payload_fixed.node_a must equal "
        "'dynamic_fixed_pair/base'"
    ) in errors
    assert (
        "node_tree_manifest.path_maps_complete must match path map counts" in errors
    )
    assert robot_schema.build_godot_node_tree_manifest_path_map_mismatches(
        broken
    ) == [
        {
            "map": "joint_node_paths",
            "key": "base_payload_fixed",
            "field": "node_a",
            "kind": "value_mismatch",
            "expected": "dynamic_fixed_pair/base",
            "actual": "wrong/path",
        }
    ]


def test_validator_tool_accepts_standalone_node_tree_manifest(tmp_path: Path) -> None:
    manifest_path = tmp_path / "robot.node_tree_manifest.json"
    payload = json.loads(FIXED_PAIR_FIXTURE.read_text(encoding="utf-8"))
    normalized = robot_schema.normalize_robot_config_for_godot(payload)
    manifest = robot_schema.build_godot_node_tree_manifest(normalized)
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    result = subprocess.run(
        [
            sys.executable,
            str(VALIDATOR_TOOL),
            str(manifest_path),
            "--input-kind",
            "auto",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    result_payload = json.loads(result.stdout)
    assert result_payload["status"] == "success"
    assert result_payload["input_kind"] == "node-tree-manifest"
    assert result_payload["manifest_version"] == "godot_node_tree_manifest.v1"
    assert result_payload["robot_name"] == manifest["robot_name"]
    assert result_payload["node_tree_manifest"] == manifest
    assert result_payload["node_tree_manifest_errors"] == []
    assert result_payload["node_tree_manifest_path_map_mismatches"] == []

    manifest["parts_count"] = 99
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    invalid_result = subprocess.run(
        [
            sys.executable,
            str(VALIDATOR_TOOL),
            str(manifest_path),
            "--input-kind",
            "node-tree-manifest",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert invalid_result.returncode == 1
    invalid_payload = json.loads(invalid_result.stdout)
    assert invalid_payload["status"] == "error"
    assert invalid_payload["input_kind"] == "node-tree-manifest"
    assert invalid_payload["node_tree_manifest_path_map_mismatches"] == []
    assert "node_tree_manifest.parts_count must equal 2" in invalid_payload[
        "node_tree_manifest_errors"
    ]


def test_validator_tool_rejects_unrestorable_control_joints(tmp_path: Path) -> None:
    invalid_path = tmp_path / "invalid_control.json"
    invalid_payload = json.loads(FIXTURE.read_text(encoding="utf-8"))
    invalid_payload["control"] = {
        "mode": "",
        "joints": {
            "missing_joint": {"kp": 50.0, "kd": 5.0},
            "hip_left": {"kp": "fast", "kd": 5.0},
            "knee_left": ["bad"],
        },
    }
    invalid_path.write_text(json.dumps(invalid_payload), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(VALIDATOR_TOOL), str(invalid_path)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    payload = json.loads(result.stdout)
    assert payload["status"] == "error"
    assert payload["errors"] == [
        "control.mode must be a non-empty string when present",
        "control.joints.missing_joint references unknown connection 'missing_joint'",
        "control.joints.hip_left.kp must be a number",
        "control.joints.knee_left must be an object",
    ]


def test_headless_smoke_tool_exercises_load_schema_and_step() -> None:
    content = SMOKE_TOOL.read_text(encoding="utf-8")

    assert '"type": "load_robot"' in content
    assert '"type": "get_schema"' in content
    assert '"type": "step"' in content
    assert "dynamic_robot_generation" in content
    assert "parts_created" in content
    assert "joints_created" in content
    assert "part_nodes mapping count" in content
    assert "joint_nodes mapping count" in content
    assert "body_states does not match" in content
    assert "joint_states does not match" in content
    assert "joint_states endpoint telemetry is incomplete" in content
    assert "def _joint_endpoint_summary" in content
    assert '"joint_endpoint_summary": endpoint_summary' in content
    assert "--max-endpoint-distance" in content
    assert "--max-relative-angle" in content
    assert "--min-body-displacement" in content
    assert "--max-linear-speed" in content
    assert "--min-joint-angle-delta" in content
    assert "--min-joint-angle-range" in content
    assert "--min-moving-joint-coverage" in content
    assert "--min-commanded-joint-response-coverage" in content
    assert "--joint-motion-epsilon" in content
    assert "--min-action-target-coverage" in content
    assert "--min-control-action-coverage" in content
    assert "--min-nonzero-action-targets" in content
    assert "--min-action-transitions" in content
    assert "--min-action-transition-delta" in content
    assert "threshold_exceeded" in content
    assert "def _joint_angle_summary" in content
    assert '"joint_angle_summary": angle_summary' in content
    assert "def _joint_limit_summary" in content
    assert '"joint_limit_summary": limit_summary' in content
    assert "def _joint_parameter_summary" in content
    assert '"joint_parameter_summary": parameter_summary' in content
    assert "def _joint_parameter_consistency_summary" in content
    assert "def _compare_fixed_joint_lock" in content
    assert "def _compare_required_joint_parameter" in content
    assert '"fixed_lock_checked_count": fixed_lock_checked_count' in content
    assert '"fixed_lock_mismatch_count": sum(' in content
    assert 'field="fixed.lock_applied"' in content
    assert 'field=f"fixed.{group_name}.{axis}"' in content
    assert '"joint_parameter_consistency_summary": parameter_consistency_summary' in content
    assert "joint parameter mismatch detected" in content
    assert "--fail-on-parameter-mismatch" in content
    assert "--fail-on-control-mismatch" in content
    assert "--fail-on-full-mechanical-restoration" in content
    assert "--fail-on-action-target-mismatch" in content
    assert "--fail-on-action-sequence-target-mismatch" in content
    assert "--fail-on-unknown-action-target" in content
    assert "--fail-on-invalid-action-target" in content
    assert "--parameter-tolerance" in content
    assert '"tolerance": tolerance' in content
    assert '"dynamics_configured_count": 0' in content
    assert "def _joint_control_summary" in content
    assert '"joint_control_summary": control_summary' in content
    assert "nonzero action targets below minimum" in content
    assert "def _joint_control_consistency_summary" in content
    assert '"joint_control_consistency_summary": control_consistency_summary' in content
    assert "def _action_target_consistency_summary" in content
    assert "def _action_target_plan" in content
    assert "def _effective_invalid_action_target_count" in content
    assert '"action_target_consistency_summary": action_target_consistency_summary' in content
    assert "def _action_sequence_target_consistency_summary" in content
    assert (
        '"action_sequence_target_consistency_summary": '
        "action_sequence_target_consistency_summary"
    ) in content
    assert "def _action_target_coverage_summary" in content
    assert "def _control_action_coverage_summary" in content
    assert '"action_target_coverage_summary": action_target_coverage_summary' in content
    assert '"control_action_coverage_summary": control_action_coverage_summary' in content
    assert "def _simulation_summary" in content
    assert '"simulation_summary": simulation_summary' in content
    assert "def _joint_motion_summary" in content
    assert '"joint_motion_summary": joint_motion_summary' in content
    assert "MECHANICAL_BEHAVIOR_EVIDENCE_VERSION" in content
    assert "MECHANICAL_BEHAVIOR_TRACE_VERSION" in content
    assert "--mechanical-trace-output" in content
    assert "def _mechanical_behavior_evidence" in content
    assert "def _write_mechanical_trace_artifact" in content
    assert '"mechanical_behavior_evidence": mechanical_behavior_evidence' in content
    assert "def _commanded_action_targets" in content
    assert '"steps_run": len(step_results)' in content
    assert '"displacement_under_min": False' in content
    assert '"speed_threshold_exceeded": False' in content
    assert "body displacement below minimum" in content
    assert "linear speed threshold exceeded" in content
    assert "joint angle delta below minimum" in content
    assert "joint angle range below minimum" in content
    assert "moving joint coverage below minimum" in content
    assert "commanded joint response coverage below minimum" in content
    assert "action target coverage below minimum" in content
    assert "control action coverage below minimum" in content
    assert "def _action_sequence_summary" in content
    assert '"action_sequence_summary": action_sequence_summary' in content
    assert "action transitions below minimum" in content
    assert "action transition delta below minimum" in content
    assert "def _mechanical_restoration_summary" in content
    assert '"mechanical_restoration_summary": restoration_summary' in content
    assert '"min_restoration_score_threshold": min_restoration_score' in content
    assert '"score_under_min": (' in content
    assert "mechanical restoration is incomplete" in content
    assert "mechanical restoration score below minimum" in content
    assert '"complete": passed == total' in content
    assert "def _action_for_step" in content
    assert "--action-json" in content
    assert "--action-sequence-json" in content
    assert "--steps" in content
    assert "--step-delay-seconds" in content
    assert '"action_sent": action' in content
    assert '"first_action_sent": actions[0] if actions else action' in content
    assert '"last_action_sent": actions[-1] if actions else action' in content
    assert "--fail-on-joint-limit-violation" in content
    assert "--fail-on-incomplete-restoration" in content
    assert "--min-restoration-score" in content
    assert "--fail-on-incomplete-node-tree" in content
    assert "--fail-on-full-node-tree-restoration" in content
    assert "--fail-on-node-tree-class-mismatch" in content
    assert "--fail-on-node-tree-missing-parameters" in content
    assert "--fail-on-node-tree-transform-mismatch" in content
    assert "--fail-on-node-tree-physical-mismatch" in content
    assert "--fail-on-node-tree-fixed-lock-mismatch" in content
    assert "--node-tree-tolerance" in content
    assert "node tree class mismatch detected" in content
    assert "node tree joint parameters missing" in content
    assert "node tree transform mismatch detected" in content
    assert "node tree physical parameter mismatch detected" in content
    assert "node tree fixed joint lock mismatch detected" in content
    assert "joint control parameter mismatch detected" in content
    assert "action target velocity mismatch detected" in content
    assert "action sequence target velocity mismatch detected" in content
    assert "unknown action target detected" in content
    assert "invalid action target detected" in content
    assert "node tree restoration is incomplete" in content
    assert "args.fail_on_full_node_tree_restoration" in content
    assert "args.fail_on_full_mechanical_restoration" in content
    assert "joint limit violation detected" in content
    assert 'state.get("relative_angle")' in content
    assert "def _joint_endpoint_telemetry_is_complete" in content
    assert 'state.get("limits")' in content
    assert 'state.get("applied_parameters")' in content
    assert '"first_joint_state": _first_mapping_value(joint_states)' in content
    assert '"mapping_summary"' in content
    assert "def _node_tree_manifest" in content
    assert "compare_godot_node_tree_manifest_to_runtime" in content
    assert '"static_manifest_version"' in content
    assert '"static_manifest_comparison"' in content
    assert "def _node_tree_gate_summary" in content
    assert "def _mechanical_gate_summary" in content
    assert '"mechanical_gate_summary": mechanical_gate_summary' in content
    assert '"node_tree_gate_summary": node_tree_gate_summary' in content
    assert '"node_tree_manifest": node_tree_manifest' in content
    assert '"missing_part_ids": missing_part_ids' in content
    assert '"missing_connection_names": missing_connection_names' in content
    assert '"fixed_lock_checked_count": fixed_lock_checked_count' in content
    assert '"fixed_lock_mismatch_count": len(fixed_lock_mismatches)' in content
    assert '"fixed_lock_mismatches": fixed_lock_mismatches[:20]' in content
    assert '"fixed_locks_complete": not fixed_lock_mismatches' in content
    assert '"fixed_lock_complete": fixed_lock_complete' in content


def test_parameter_consistency_summary_respects_tolerance() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    joint_states = {
        "hip_left": {
            "applied_parameters": {
                "source": {
                    "limits": {"lower": -1.2, "upper": 1.2},
                    "motor": {"enabled": True, "max_impulse": 120.0},
                },
                "runtime": {
                    "limit_lower": -1.20005,
                    "limit_upper": 1.2,
                    "motor_enabled": True,
                    "motor_max_impulse": 120.0,
                },
            }
        }
    }

    loose = smoke_tool._joint_parameter_consistency_summary(
        joint_states,
        tolerance=1e-4,
    )
    strict = smoke_tool._joint_parameter_consistency_summary(
        joint_states,
        tolerance=1e-6,
    )

    assert loose["mismatch_count"] == 0
    assert loose["tolerance"] == 1e-4
    assert strict["mismatch_count"] == 1
    assert strict["mismatches"][0]["field"] == "limits.lower"
    assert strict["tolerance"] == 1e-6


def test_parameter_consistency_summary_checks_fixed_joint_lock() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    locked = smoke_tool._joint_parameter_consistency_summary(
        {
            "base_payload_fixed": {
                "applied_parameters": {
                    "source": {
                        "limits": {"lower": 0.0, "upper": 0.0},
                        "motor": {
                            "enabled": False,
                            "target_velocity": 0.0,
                            "max_impulse": 0.0,
                        },
                    },
                    "runtime": {
                        "fixed_approximation": True,
                        "fixed_lock_applied": True,
                        "limit_lower": 0.0,
                        "limit_upper": 0.0,
                        "motor_enabled": False,
                        "motor_max_impulse": 0.0,
                        "linear_limit_enabled": {"x": True, "y": True, "z": True},
                        "linear_lower": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "linear_upper": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "angular_limit_enabled": {"x": True, "y": True, "z": True},
                        "angular_lower": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "angular_upper": {"x": 0.0, "y": 0.0, "z": 0.0},
                    },
                }
            }
        }
    )
    broken = smoke_tool._joint_parameter_consistency_summary(
        {
            "base_payload_fixed": {
                "applied_parameters": {
                    "source": {
                        "limits": {"lower": 0.0, "upper": 0.0},
                        "motor": {
                            "enabled": False,
                            "target_velocity": 0.0,
                            "max_impulse": 0.0,
                        },
                    },
                    "runtime": {
                        "fixed_approximation": True,
                        "fixed_lock_applied": False,
                        "limit_lower": 0.0,
                        "limit_upper": 0.0,
                        "motor_enabled": False,
                        "motor_max_impulse": 0.0,
                        "linear_limit_enabled": {"x": True, "y": False, "z": True},
                        "linear_lower": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "linear_upper": {"x": 0.0, "y": 0.1, "z": 0.0},
                        "angular_limit_enabled": {"x": True, "y": True, "z": True},
                        "angular_lower": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "angular_upper": {"x": 0.0, "y": 0.0, "z": 0.0},
                    },
                }
            }
        },
        tolerance=1e-4,
    )

    assert locked["fixed_lock_checked_count"] == 1
    assert locked["fixed_lock_mismatch_count"] == 0
    assert locked["complete"] is True
    assert broken["fixed_lock_checked_count"] == 1
    assert broken["fixed_lock_mismatch_count"] == 3
    assert broken["complete"] is False
    assert {item["field"] for item in broken["mismatches"]} == {
        "fixed.lock_applied",
        "fixed.linear_limit_enabled.y",
        "fixed.linear_upper.y",
    }


def test_control_consistency_summary_compares_configured_gains() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    summary = smoke_tool._joint_control_consistency_summary(
        {
            "control": {
                "joints": {
                    "hip_left": {"kp": 50.0, "kd": 5.0},
                    "knee_left": {"kp": 45.0, "kd": 4.0},
                }
            }
        },
        {
            "hip_left": {"control_parameters": {"kp": 50.0, "kd": 5.0}},
            "knee_left": {"control_parameters": {"kp": 40.0, "kd": 4.0}},
        },
        tolerance=1e-4,
    )

    assert summary["configured_count"] == 2
    assert summary["checked_count"] == 2
    assert summary["missing_count"] == 0
    assert summary["mismatch_count"] == 1
    assert summary["mismatches"][0]["field"] == "control.kp"
    assert summary["complete"] is False


def test_mechanical_behavior_evidence_records_units_risks_and_trace() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    evidence = smoke_tool._mechanical_behavior_evidence(
        limit_summary={
            "joint_count": 1,
            "checked_count": 1,
            "violation_count": 0,
            "violations": [],
        },
        control_summary={"targeted_count": 1, "max_abs_target_velocity": 2.0},
        action_target_coverage_summary={"coverage_ratio": 1.0},
        control_action_coverage_summary={"coverage_ratio": 1.0},
        simulation_summary={"max_body_displacement": 0.2},
        joint_motion_summary={
            "commanded_joint_count": 1,
            "commanded_moving_joint_count": 1,
            "commanded_joint_response_ratio": 1.0,
            "min_commanded_joint_response_coverage_threshold": 0.5,
            "commanded_joint_response_under_min": False,
            "joint_motion_epsilon": 0.001,
            "max_abs_relative_angle_range": 0.4,
            "commanded_joint_response_details": [
                {"joint": "hip", "angle_range": 0.4, "responded": True}
            ],
        },
        step_results=[
            {
                "body_count": 1,
                "joint_count": 1,
                "body_states": {"torso": {}},
                "joint_states": {"hip": {"relative_angle": 0.1}},
                "reward": 0.0,
                "done": False,
            }
        ],
        actions=[{"hip": 2.0}],
    )

    assert evidence["evidence_version"] == (
        "dynamic_godot_mechanical_behavior_evidence.v1"
    )
    assert evidence["units"]["joint_angle"] == "radians"
    assert evidence["joint_limit_evidence"]["checked_count"] == 1
    assert evidence["torque_velocity_response_evidence"][
        "commanded_joint_response_ratio"
    ] == 1.0
    assert evidence["center_of_mass_evidence"]["available"] is False
    assert "center_of_mass_runtime_readback_missing" in evidence["residual_risks"]
    assert evidence["step_trace_evidence"]["inline_step_count"] == 1
    assert evidence["step_trace_evidence"]["trace"][0]["action"] == {"hip": 2.0}


def test_mechanical_behavior_evidence_computes_center_of_mass() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    evidence = smoke_tool._mechanical_behavior_evidence(
        limit_summary={"violation_count": 0, "violations": []},
        control_summary={},
        action_target_coverage_summary={},
        control_action_coverage_summary={},
        simulation_summary={"max_body_displacement": 0.2},
        joint_motion_summary={},
        step_results=[
            {
                "body_states": {
                    "torso": {
                        "position": [0.0, 1.0, 0.0],
                        "linear_velocity": [0.0, 0.0, 0.0],
                        "mass": 3.0,
                    },
                    "leg": {
                        "position": [1.0, 0.0, 0.0],
                        "linear_velocity": [2.0, 0.0, 0.0],
                        "mass": 1.0,
                    },
                }
            },
            {
                "body_states": {
                    "torso": {
                        "position": [0.0, 1.2, 0.0],
                        "linear_velocity": [0.0, 0.4, 0.0],
                        "mass": 3.0,
                    },
                    "leg": {
                        "position": [1.0, 0.2, 0.0],
                        "linear_velocity": [2.0, 0.0, 0.0],
                        "mass": 1.0,
                    },
                }
            },
        ],
        actions=[],
    )
    com = evidence["center_of_mass_evidence"]

    assert "center_of_mass" in evidence["available_sections"]
    assert "center_of_mass" not in evidence["missing_sections"]
    assert "center_of_mass_runtime_readback_missing" not in evidence["residual_risks"]
    assert com["available"] is True
    assert com["source"] == "body_states_mass_position"
    assert com["current_center_of_mass"] == [0.25, 0.95, 0.0]
    assert com["current_center_of_mass_velocity"] == [0.5, 0.30000000000000004, 0.0]
    assert com["total_mass"] == 4.0
    assert com["stability_summary"]["first_center_of_mass"] == [0.25, 0.75, 0.0]
    assert com["stability_summary"]["height_range"] == 0.19999999999999996
    assert com["motion_summary"] == {"max_body_displacement": 0.2}


def test_mechanical_behavior_evidence_extracts_contact_state_when_reported() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    evidence = smoke_tool._mechanical_behavior_evidence(
        limit_summary={"violation_count": 0, "violations": []},
        control_summary={},
        action_target_coverage_summary={},
        control_action_coverage_summary={},
        simulation_summary={},
        joint_motion_summary={},
        step_results=[
            {
                "body_states": {
                    "left_foot": {
                        "position": [0.0, 0.0, 0.0],
                        "linear_velocity": [0.0, 0.0, 0.0],
                        "mass": 1.0,
                        "ground_contact": True,
                    },
                    "right_foot": {
                        "position": [1.0, 0.0, 0.0],
                        "linear_velocity": [0.0, 0.0, 0.0],
                        "mass": 1.0,
                        "contact_count": 0,
                    },
                },
                "contacts": {
                    "tail": {"contact_points": [[0.0, 0.0, 0.0]]}
                },
            }
        ],
        actions=[],
    )
    contact = evidence["contact_state_evidence"]

    assert "contact_state" in evidence["available_sections"]
    assert "contact_state" not in evidence["missing_sections"]
    assert "contact_state_runtime_readback_missing" not in evidence["residual_risks"]
    assert contact["available"] is True
    assert contact["source"] == "runtime_contact_telemetry"
    assert contact["support_part_count"] == 2
    assert contact["support_parts"] == ["left_foot", "tail"]
    assert contact["current_contacts"][0]["source_fields"] == ["ground_contact"]
    assert contact["current_contacts"][1]["contact_count"] == 0
    assert contact["current_contacts"][2]["part_id"] == "tail"


def test_mechanical_trace_artifact_writes_full_step_trace(tmp_path: Path) -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    trace_path = tmp_path / "trace.json"
    written_path = smoke_tool._write_mechanical_trace_artifact(
        robot_name="trace_bot",
        step_results=[
            {
                "body_count": 1,
                "joint_count": 1,
                "body_states": {"torso": {}},
                "joint_states": {"hip": {"relative_angle": 0.1}},
            },
            {
                "body_count": 1,
                "joint_count": 1,
                "body_states": {"torso": {}},
                "joint_states": {"hip": {"relative_angle": 0.2}},
            },
        ],
        actions=[{"hip": 0.1}, {"hip": 0.2}],
        output=trace_path,
    )
    payload = json.loads(trace_path.read_text(encoding="utf-8"))

    assert written_path == str(trace_path)
    assert payload["artifact_version"] == "dynamic_godot_mechanical_behavior_trace.v1"
    assert payload["evidence_version"] == (
        "dynamic_godot_mechanical_behavior_evidence.v1"
    )
    assert payload["robot_name"] == "trace_bot"
    assert payload["steps_run"] == 2
    assert [item["action"] for item in payload["trace"]] == [
        {"hip": 0.1},
        {"hip": 0.2},
    ]


def test_mechanical_behavior_evidence_reports_threshold_failures() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    evidence = smoke_tool._mechanical_behavior_evidence(
        limit_summary={"violation_count": 1, "violations": [{"joint": "hip"}]},
        control_summary={},
        action_target_coverage_summary={"coverage_under_min": True},
        control_action_coverage_summary={"coverage_under_min": True},
        simulation_summary={},
        joint_motion_summary={"commanded_joint_response_under_min": True},
        step_results=[],
        actions=[],
    )

    assert evidence["complete"] is False
    assert evidence["threshold_failures"] == [
        "joint_limit_violation",
        "commanded_joint_response_under_min",
        "action_target_coverage_under_min",
        "control_action_coverage_under_min",
    ]


def test_nonzero_action_target_gate_adds_error() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    result = smoke_tool._build_result(
        robot_config={
            "name": "zero_action",
            "parts": [{"id": "torso", "type": "torso", "shape": "box"}],
            "connections": [],
        },
        load_result={"status": "success", "parts_created": 1, "joints_created": 0},
        schema_result={"meta": {"dynamic_robot_generation": True}},
        step_result={
            "body_count": 1,
            "joint_count": 0,
            "body_states": {"torso": {}},
            "joint_states": {"hip_left": {"target_velocity": 0.0}},
        },
        step_results=[],
        max_endpoint_distance=None,
        max_relative_angle=None,
        min_body_displacement=None,
        max_linear_speed=None,
        min_joint_angle_delta=None,
        min_joint_angle_range=None,
        min_moving_joint_coverage=None,
        min_commanded_joint_response_coverage=None,
        joint_motion_epsilon=0.0,
        min_action_target_coverage=None,
        min_control_action_coverage=None,
        min_nonzero_action_targets=1,
        min_action_transitions=None,
        min_action_transition_delta=None,
        fail_on_joint_limit_violation=False,
        fail_on_incomplete_restoration=False,
        min_restoration_score=None,
        fail_on_parameter_mismatch=False,
        fail_on_control_mismatch=False,
        fail_on_action_target_mismatch=False,
        fail_on_action_sequence_target_mismatch=False,
        fail_on_unknown_action_target=False,
        fail_on_invalid_action_target=False,
        fail_on_incomplete_node_tree=False,
        fail_on_node_tree_class_mismatch=False,
        fail_on_node_tree_missing_parameters=False,
        fail_on_node_tree_transform_mismatch=False,
        fail_on_node_tree_physical_mismatch=False,
        fail_on_node_tree_fixed_lock_mismatch=False,
        node_tree_tolerance=1e-4,
        parameter_tolerance=1e-4,
        action={},
        actions=[],
        stdout="",
        stderr="",
    )

    assert result["status"] == "error"
    assert result["step_summary"]["joint_control_summary"]["targeted_count"] == 0
    assert result["step_summary"]["joint_control_summary"]["nonzero_targets_under_min"] is True
    assert any("nonzero action targets below minimum" in item for item in result["errors"])


def test_action_transition_gate_adds_error() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    result = smoke_tool._build_result(
        robot_config={
            "name": "static_sequence",
            "parts": [{"id": "torso", "type": "torso", "shape": "box"}],
            "connections": [],
        },
        load_result={"status": "success", "parts_created": 1, "joints_created": 0},
        schema_result={"meta": {"dynamic_robot_generation": True}},
        step_result={
            "body_count": 1,
            "joint_count": 0,
            "body_states": {"torso": {}},
            "joint_states": {},
        },
        step_results=[],
        max_endpoint_distance=None,
        max_relative_angle=None,
        min_body_displacement=None,
        max_linear_speed=None,
        min_joint_angle_delta=None,
        min_joint_angle_range=None,
        min_moving_joint_coverage=None,
        min_commanded_joint_response_coverage=None,
        joint_motion_epsilon=0.0,
        min_action_target_coverage=None,
        min_control_action_coverage=None,
        min_nonzero_action_targets=None,
        min_action_transitions=1,
        min_action_transition_delta=None,
        fail_on_joint_limit_violation=False,
        fail_on_incomplete_restoration=False,
        min_restoration_score=None,
        fail_on_parameter_mismatch=False,
        fail_on_control_mismatch=False,
        fail_on_action_target_mismatch=False,
        fail_on_action_sequence_target_mismatch=False,
        fail_on_unknown_action_target=False,
        fail_on_invalid_action_target=False,
        fail_on_incomplete_node_tree=False,
        fail_on_node_tree_class_mismatch=False,
        fail_on_node_tree_missing_parameters=False,
        fail_on_node_tree_transform_mismatch=False,
        fail_on_node_tree_physical_mismatch=False,
        fail_on_node_tree_fixed_lock_mismatch=False,
        node_tree_tolerance=1e-4,
        parameter_tolerance=1e-4,
        action={"hip_left": 0.1},
        actions=[{"hip_left": 0.1}, {"hip_left": 0.1}],
        stdout="",
        stderr="",
    )

    summary = result["step_summary"]["action_sequence_summary"]
    assert result["status"] == "error"
    assert summary["transition_count"] == 0
    assert summary["transitions_under_min"] is True
    assert any("action transitions below minimum" in item for item in result["errors"])


def test_action_sequence_summary_reports_numeric_transition_delta() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    summary = smoke_tool._action_sequence_summary(
        [{"hip_left": 0.2}, {"hip_left": 0.201}, {"hip_left": 0.205}],
        min_transitions=1,
        min_transition_delta=0.05,
    )

    assert summary["transition_count"] == 2
    assert summary["transitions_under_min"] is False
    assert abs(summary["max_numeric_transition_delta"] - 0.004) < 1e-12
    assert abs(summary["average_numeric_transition_delta"] - 0.0025) < 1e-12
    assert summary["transition_delta_under_min"] is True


def test_joint_motion_summary_reports_relative_angle_delta() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    summary = smoke_tool._joint_motion_summary(
        [
            {
                "joint_states": {
                    "hip_left": {"relative_angle": 0.1},
                    "knee_left": {"relative_angle": -0.2},
                }
            },
            {
                "joint_states": {
                    "hip_left": {"relative_angle": 0.14},
                    "knee_left": {"relative_angle": -0.205},
                }
            },
            {
                "joint_states": {
                    "hip_left": {"relative_angle": 0.1},
                    "knee_left": {"relative_angle": -0.1},
                }
            },
        ],
        min_joint_angle_delta=0.05,
        min_joint_angle_range=0.08,
        min_moving_joint_coverage=1.0,
        min_commanded_joint_response_coverage=1.0,
        joint_motion_epsilon=0.05,
        actions=[{"hip_left": 0.2}, {"knee_left": -0.2}],
    )

    assert summary["joint_count"] == 2
    assert summary["measured_joint_count"] == 2
    assert summary["moving_joint_count"] == 1
    assert summary["moving_joint_coverage_ratio"] == 0.5
    assert summary["moving_joint_coverage_under_min"] is True
    assert summary["commanded_joint_count"] == 2
    assert summary["commanded_moving_joint_count"] == 1
    assert summary["commanded_joint_response_ratio"] == 0.5
    assert summary["commanded_joint_response_under_min"] is True
    assert summary["commanded_static_joints"] == ["hip_left"]
    details = summary["commanded_joint_response_details"]
    assert [item["joint"] for item in details] == ["hip_left", "knee_left"]
    assert [item["responded"] for item in details] == [False, True]
    assert [item["max_abs_target"] for item in details] == [0.2, 0.2]
    assert abs(details[0]["angle_range"] - 0.04) < 1e-12
    assert abs(details[1]["angle_range"] - 0.105) < 1e-12
    assert summary["joint_motion_epsilon"] == 0.05
    assert summary["largest_moving_joint"] == "knee_left"
    assert summary["largest_range_joint"] == "knee_left"
    assert abs(summary["max_abs_relative_angle_delta"] - 0.1) < 1e-12
    assert abs(summary["max_abs_relative_angle_range"] - 0.105) < 1e-12
    assert summary["angle_delta_under_min"] is False
    assert summary["angle_range_under_min"] is False

    static_summary = smoke_tool._joint_motion_summary(
        [
            {"joint_states": {"hip_left": {"relative_angle": 0.1}}},
            {"joint_states": {"hip_left": {"relative_angle": 0.1}}},
        ],
        min_joint_angle_delta=None,
        min_joint_angle_range=None,
        min_moving_joint_coverage=None,
        min_commanded_joint_response_coverage=None,
        joint_motion_epsilon=0.0,
        actions=[{"hip_left": 0.2}],
    )
    assert static_summary["moving_joint_count"] == 0
    assert static_summary["moving_joint_coverage_ratio"] == 0.0
    assert static_summary["commanded_joint_response_ratio"] == 0.0
    assert static_summary["commanded_joint_response_details"][0]["responded"] is False
    assert static_summary["largest_moving_joint"] is None
    assert static_summary["largest_range_joint"] is None


def test_action_target_consistency_summary_compares_last_action() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    summary = smoke_tool._action_target_consistency_summary(
        {
            "hip_left": {"target_velocity": 0.35},
            "knee_left": {"target_velocity": 0.0},
        },
        {"hip_left": 0.35, "knee_left": -0.2},
        tolerance=1e-4,
    )

    assert summary["expected_count"] == 2
    assert summary["checked_count"] == 2
    assert summary["mismatch_count"] == 1
    assert summary["mismatches"][0]["joint"] == "knee_left"
    assert summary["mismatches"][0]["field"] == "target_velocity"
    assert summary["complete"] is False


def test_action_target_consistency_summary_reports_unknown_targets() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    summary = smoke_tool._action_target_consistency_summary(
        {"hip_left": {"target_velocity": 0.35}},
        {"hip_left": 0.35, "missing_joint": 0.2},
        tolerance=1e-4,
    )
    list_summary = smoke_tool._action_target_consistency_summary(
        {"hip_left": {"target_velocity": 0.35}},
        [0.35, 0.2],
        tolerance=1e-4,
    )

    assert summary["expected_count"] == 1
    assert summary["unknown_target_count"] == 1
    assert summary["unknown_targets"][0]["joint"] == "missing_joint"
    assert summary["complete"] is False
    assert list_summary["unknown_target_count"] == 1
    assert list_summary["unknown_targets"][0]["index"] == 1


def test_action_target_consistency_summary_reports_invalid_targets() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    summary = smoke_tool._action_target_consistency_summary(
        {"hip_left": {"target_velocity": 0.35}},
        {"hip_left": "fast"},
        tolerance=1e-4,
    )
    list_summary = smoke_tool._action_target_consistency_summary(
        {"hip_left": {"target_velocity": 0.35}},
        ["fast"],
        tolerance=1e-4,
    )

    assert summary["expected_count"] == 0
    assert summary["invalid_target_count"] == 1
    assert summary["invalid_targets"][0]["joint"] == "hip_left"
    assert summary["invalid_targets"][0]["target_type"] == "str"
    assert summary["complete"] is False
    assert list_summary["invalid_target_count"] == 1
    assert list_summary["invalid_targets"][0]["index"] == 0


def test_action_sequence_target_consistency_summary_compares_each_step() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    summary = smoke_tool._action_sequence_target_consistency_summary(
        [
            {
                "joint_states": {
                    "hip_left": {"target_velocity": 0.2},
                    "knee_left": {"target_velocity": -0.1},
                }
            },
            {
                "joint_states": {
                    "hip_left": {"target_velocity": -0.2},
                    "knee_left": {"target_velocity": 0.0},
                }
            },
        ],
        [
            {"hip_left": 0.2, "knee_left": -0.1},
            {"hip_left": -0.2, "knee_left": 0.1},
        ],
        fallback_action={},
        tolerance=1e-4,
    )

    assert summary["steps"] == 2
    assert summary["expected_count"] == 4
    assert summary["checked_count"] == 4
    assert summary["mismatch_count"] == 1
    assert summary["mismatches"][0]["step_index"] == 1
    assert summary["mismatches"][0]["joint"] == "knee_left"
    assert summary["complete"] is False


def test_action_sequence_target_consistency_summary_reports_unknown_targets() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    summary = smoke_tool._action_sequence_target_consistency_summary(
        [{"joint_states": {"hip_left": {"target_velocity": 0.2}}}],
        [{"hip_left": 0.2, "ankle_left": 0.1}],
        fallback_action={},
        tolerance=1e-4,
    )

    assert summary["expected_count"] == 1
    assert summary["unknown_target_count"] == 1
    assert summary["unknown_targets"][0]["step_index"] == 0
    assert summary["unknown_targets"][0]["joint"] == "ankle_left"
    assert summary["complete"] is False


def test_action_sequence_target_consistency_summary_reports_invalid_targets() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    summary = smoke_tool._action_sequence_target_consistency_summary(
        [{"joint_states": {"hip_left": {"target_velocity": 0.2}}}],
        [{"hip_left": "fast"}],
        fallback_action={},
        tolerance=1e-4,
    )

    assert summary["expected_count"] == 0
    assert summary["invalid_target_count"] == 1
    assert summary["invalid_targets"][0]["step_index"] == 0
    assert summary["invalid_targets"][0]["joint"] == "hip_left"
    assert summary["complete"] is False


def test_action_target_coverage_summary_reports_missing_targets() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    summary = smoke_tool._action_target_coverage_summary(
        {
            "hip_left": {"target_velocity": 0.2},
            "knee_left": {"target_velocity": 0.0},
            "ankle_left": {"target_velocity": 0.0},
        },
        [{"hip_left": 0.2}, {"knee_left": -0.1, "missing_joint": 0.3}],
        fallback_action={},
        min_coverage=0.8,
    )

    assert summary["joint_count"] == 3
    assert summary["covered_joint_count"] == 2
    assert summary["missing_target_count"] == 1
    assert summary["missing_target_joints"] == ["ankle_left"]
    assert summary["target_command_count"] == 2
    assert summary["unknown_target_count"] == 1
    assert summary["invalid_target_count"] == 0
    assert abs(summary["coverage_ratio"] - (2 / 3)) < 1e-12
    assert summary["coverage_under_min"] is True
    assert summary["complete"] is False


def test_control_action_coverage_summary_reports_untargeted_controls() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    summary = smoke_tool._control_action_coverage_summary(
        {
            "control": {
                "joints": {
                    "hip_left": {"kp": 50.0},
                    "knee_left": {"kp": 40.0},
                    "ankle_left": {"kp": 20.0},
                }
            }
        },
        {
            "hip_left": {"target_velocity": 0.2},
            "knee_left": {"target_velocity": 0.0},
        },
        [{"hip_left": 0.2}],
        fallback_action={},
        min_coverage=1.0,
    )

    assert summary["configured_count"] == 3
    assert summary["generated_configured_count"] == 2
    assert summary["covered_control_count"] == 1
    assert summary["missing_action_count"] == 1
    assert summary["missing_action_joints"] == ["knee_left"]
    assert summary["missing_generated_count"] == 1
    assert summary["missing_generated_joints"] == ["ankle_left"]
    assert summary["coverage_ratio"] == 0.5
    assert summary["coverage_under_min"] is True
    assert summary["complete"] is False


def test_node_tree_manifest_tracks_restored_and_missing_nodes() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    manifest = smoke_tool._node_tree_manifest(
        robot_config={
            "name": "manifest_test",
            "parts": [
                {"id": "torso", "type": "torso", "shape": "box"},
                {"id": "leg", "type": "leg", "shape": "capsule"},
            ],
            "connections": [
                {
                    "name": "hip",
                    "from": "torso",
                    "to": "leg",
                    "joint_type": "hinge",
                }
            ],
        },
        part_nodes=[
            {
                "part_id": "torso",
                "body_node": "/robot/torso",
                "body_class": "RigidBody3D",
                "collision_shape": "BoxShape3D",
                "collision_parameters": {"size": [0.2, 0.2, 0.2]},
                "mesh_type": "BoxMesh",
                "mesh_parameters": {"size": [0.2, 0.2, 0.2]},
                "mass": 5.0,
                "position": [0.0, 1.0, 0.0],
                "rotation": [0.0, 0.0, 0.0],
            }
        ],
        joint_nodes=[],
    )

    assert manifest["robot_name"] == "manifest_test"
    assert manifest["complete"] is False
    assert manifest["parts_complete"] is False
    assert manifest["joints_complete"] is False
    assert manifest["missing_part_ids"] == ["leg"]
    assert manifest["missing_connection_names"] == ["hip"]
    assert manifest["class_mismatch_count"] == 0
    assert manifest["classes_complete"] is True
    assert manifest["parameter_missing_count"] == 0
    assert manifest["parameters_complete"] is True
    assert manifest["missing_parameter_connection_names"] == []
    assert manifest["fixed_lock_checked_count"] == 0
    assert manifest["fixed_lock_mismatch_count"] == 0
    assert manifest["fixed_locks_complete"] is True
    assert manifest["fixed_lock_mismatches"] == []
    assert manifest["transform_mismatch_count"] == 0
    assert manifest["transforms_complete"] is True
    assert manifest["physical_mismatch_count"] == 1
    assert manifest["physical_complete"] is False
    assert manifest["parts"][0]["expected_collision_shape"] == "BoxShape3D"
    assert manifest["parts"][0]["expected_mesh_type"] == "BoxMesh"
    assert manifest["joints"][0]["expected_joint_class"] == "HingeJoint3D"
    assert manifest["joints"][0]["fixed_lock_checked"] is False
    assert manifest["joints"][0]["fixed_lock_complete"] is None
    assert manifest["joints"][0]["fixed_lock_mismatch_count"] == 0
    assert manifest["parts"][0]["restored"] is True
    assert manifest["parts"][1]["restored"] is False
    assert manifest["joints"][0]["restored"] is False


def test_node_tree_manifest_reports_fixed_lock_state() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    robot_config = {
        "name": "fixed_manifest",
        "parts": [
            {"id": "base", "type": "body", "shape": "box"},
            {"id": "payload", "type": "body", "shape": "box"},
        ],
        "connections": [
            {
                "name": "fixed_good",
                "from": "base",
                "to": "payload",
                "joint_type": "fixed",
                "origin": [0.0, 0.0, 0.0],
                "axis": [1.0, 0.0, 0.0],
            },
            {
                "name": "fixed_bad",
                "from": "base",
                "to": "payload",
                "joint_type": "fixed",
                "origin": [0.0, 0.0, 0.0],
                "axis": [1.0, 0.0, 0.0],
            },
        ],
    }
    base_joint = {
        "joint_class": "Generic6DOFJoint3D",
        "origin": [0.0, 0.0, 0.0],
        "axis": [1.0, 0.0, 0.0],
        "applied_parameters": {
            "runtime": {
                "fixed_approximation": True,
                "fixed_lock_applied": True,
                "linear_limit_enabled": {"x": True, "y": True, "z": True},
                "linear_lower": {"x": 0.0, "y": 0.0, "z": 0.0},
                "linear_upper": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular_limit_enabled": {"x": True, "y": True, "z": True},
                "angular_lower": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular_upper": {"x": 0.0, "y": 0.0, "z": 0.0},
            }
        },
    }
    bad_joint = json.loads(json.dumps(base_joint))
    bad_joint["applied_parameters"]["runtime"]["fixed_lock_applied"] = False
    bad_joint["applied_parameters"]["runtime"]["linear_upper"]["y"] = 0.25

    manifest = smoke_tool._node_tree_manifest(
        robot_config=robot_config,
        part_nodes=[
            {
                "part_id": "base",
                "body_class": "RigidBody3D",
                "collision_shape": "BoxShape3D",
                "mesh_type": "BoxMesh",
                "collision_parameters": {"size": [0.2, 0.2, 0.2]},
                "mesh_parameters": {"size": [0.2, 0.2, 0.2]},
                "mass": 1.0,
                "position": [0.0, 0.0, 0.0],
                "rotation": [0.0, 0.0, 0.0],
            },
            {
                "part_id": "payload",
                "body_class": "RigidBody3D",
                "collision_shape": "BoxShape3D",
                "mesh_type": "BoxMesh",
                "collision_parameters": {"size": [0.2, 0.2, 0.2]},
                "mesh_parameters": {"size": [0.2, 0.2, 0.2]},
                "mass": 1.0,
                "position": [0.0, 0.0, 0.0],
                "rotation": [0.0, 0.0, 0.0],
            },
        ],
        joint_nodes=[
            {"connection_name": "fixed_good", **base_joint},
            {"connection_name": "fixed_bad", **bad_joint},
        ],
    )

    assert manifest["complete"] is False
    assert manifest["fixed_lock_checked_count"] == 2
    assert manifest["fixed_lock_mismatch_count"] == 2
    assert manifest["fixed_locks_complete"] is False
    assert manifest["joints"][0]["fixed_lock_complete"] is True
    assert manifest["joints"][1]["fixed_lock_complete"] is False
    assert manifest["joints"][1]["fixed_lock_mismatch_count"] == 2
    assert {item["field"] for item in manifest["fixed_lock_mismatches"]} == {
        "fixed.lock_applied",
        "fixed.linear_upper.y",
    }


def test_incomplete_node_tree_gate_adds_error() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    result = smoke_tool._build_result(
        robot_config={
            "name": "missing_tree",
            "parts": [{"id": "torso", "type": "torso", "shape": "box"}],
            "connections": [
                {
                    "name": "missing_joint",
                    "from": "torso",
                    "to": "leg",
                    "joint_type": "hinge",
                }
            ],
        },
        load_result={
            "status": "success",
            "parts_created": 1,
            "joints_created": 1,
            "part_nodes": [
                {
                    "part_id": "torso",
                    "body_node": "/robot/torso",
                    "body_class": "RigidBody3D",
                    "collision_shape": "BoxShape3D",
                    "mesh_type": "BoxMesh",
                }
            ],
            "joint_nodes": [],
        },
        schema_result={"meta": {"dynamic_robot_generation": True}},
        step_result={"body_count": 1, "joint_count": 1, "body_states": {}, "joint_states": {}},
        step_results=[],
        max_endpoint_distance=None,
        max_relative_angle=None,
        min_body_displacement=None,
        max_linear_speed=None,
        min_joint_angle_delta=None,
        min_joint_angle_range=None,
        min_moving_joint_coverage=None,
        min_commanded_joint_response_coverage=None,
        joint_motion_epsilon=0.0,
        min_action_target_coverage=None,
        min_control_action_coverage=None,
        min_nonzero_action_targets=None,
        min_action_transitions=None,
        min_action_transition_delta=None,
        fail_on_joint_limit_violation=False,
        fail_on_incomplete_restoration=False,
        min_restoration_score=None,
        fail_on_parameter_mismatch=False,
        fail_on_control_mismatch=False,
        fail_on_action_target_mismatch=False,
        fail_on_action_sequence_target_mismatch=False,
        fail_on_unknown_action_target=False,
        fail_on_invalid_action_target=False,
        fail_on_incomplete_node_tree=True,
        fail_on_node_tree_class_mismatch=False,
        fail_on_node_tree_missing_parameters=False,
        fail_on_node_tree_transform_mismatch=False,
        fail_on_node_tree_physical_mismatch=False,
        fail_on_node_tree_fixed_lock_mismatch=False,
        node_tree_tolerance=1e-4,
        parameter_tolerance=1e-4,
        action=[0.0],
        actions=[],
        stdout="",
        stderr="",
    )

    assert result["status"] == "error"
    assert result["node_tree_manifest"]["complete"] is False
    assert result["node_tree_gate_summary"] == {
        "checks": {
            "incomplete_node_tree": True,
            "class_mismatch": False,
            "missing_parameters": False,
            "transform_mismatch": False,
            "physical_mismatch": False,
            "fixed_lock_mismatch": False,
        },
        "enabled_checks": ["incomplete_node_tree"],
        "enabled_count": 1,
        "full_node_tree_restoration_required": False,
    }
    assert any("node tree restoration is incomplete" in item for item in result["errors"])


def test_node_tree_gate_summary_marks_full_restoration_gate() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    summary = smoke_tool._node_tree_gate_summary(
        fail_on_incomplete_node_tree=True,
        fail_on_node_tree_class_mismatch=True,
        fail_on_node_tree_missing_parameters=True,
        fail_on_node_tree_transform_mismatch=True,
        fail_on_node_tree_physical_mismatch=True,
        fail_on_node_tree_fixed_lock_mismatch=True,
    )

    assert summary["enabled_count"] == 6
    assert summary["full_node_tree_restoration_required"] is True
    assert summary["enabled_checks"] == [
        "incomplete_node_tree",
        "class_mismatch",
        "missing_parameters",
        "transform_mismatch",
        "physical_mismatch",
        "fixed_lock_mismatch",
    ]


def test_node_tree_manifest_detects_class_mismatch() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    manifest = smoke_tool._node_tree_manifest(
        robot_config={
            "name": "class_mismatch",
            "parts": [{"id": "leg", "type": "shin", "shape": "capsule"}],
            "connections": [
                {
                    "name": "slider_joint",
                    "from": "leg",
                    "to": "leg",
                    "joint_type": "slider",
                }
            ],
        },
        part_nodes=[
            {
                "part_id": "leg",
                "body_node": "/robot/leg",
                "body_class": "RigidBody3D",
                "collision_shape": "BoxShape3D",
                "mesh_type": "BoxMesh",
            }
        ],
        joint_nodes=[
            {
                "connection_name": "slider_joint",
                "joint_node": "/robot/slider_joint",
                "joint_class": "HingeJoint3D",
            }
        ],
    )

    assert manifest["complete"] is False
    assert manifest["classes_complete"] is False
    assert manifest["class_mismatch_count"] == 3
    assert {item["field"] for item in manifest["class_mismatches"]} == {
        "collision_shape",
        "mesh_type",
        "joint_class",
    }


def test_node_tree_missing_parameter_gate_adds_error() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    result = smoke_tool._build_result(
        robot_config={
            "name": "missing_params",
            "parts": [{"id": "torso", "type": "torso", "shape": "box"}],
            "connections": [
                {"name": "fixed_joint", "from": "torso", "to": "torso", "joint_type": "fixed"}
            ],
        },
        load_result={
            "status": "success",
            "parts_created": 1,
            "joints_created": 1,
            "part_nodes": [
                {
                    "part_id": "torso",
                    "body_node": "/robot/torso",
                    "body_class": "RigidBody3D",
                    "collision_shape": "BoxShape3D",
                    "mesh_type": "BoxMesh",
                }
            ],
            "joint_nodes": [
                {
                    "connection_name": "fixed_joint",
                    "joint_node": "/robot/fixed_joint",
                    "joint_class": "Generic6DOFJoint3D",
                }
            ],
        },
        schema_result={"meta": {"dynamic_robot_generation": True}},
        step_result={"body_count": 1, "joint_count": 1, "body_states": {}, "joint_states": {}},
        step_results=[],
        max_endpoint_distance=None,
        max_relative_angle=None,
        min_body_displacement=None,
        max_linear_speed=None,
        min_joint_angle_delta=None,
        min_joint_angle_range=None,
        min_moving_joint_coverage=None,
        min_commanded_joint_response_coverage=None,
        joint_motion_epsilon=0.0,
        min_action_target_coverage=None,
        min_control_action_coverage=None,
        min_nonzero_action_targets=None,
        min_action_transitions=None,
        min_action_transition_delta=None,
        fail_on_joint_limit_violation=False,
        fail_on_incomplete_restoration=False,
        min_restoration_score=None,
        fail_on_parameter_mismatch=False,
        fail_on_control_mismatch=False,
        fail_on_action_target_mismatch=False,
        fail_on_action_sequence_target_mismatch=False,
        fail_on_unknown_action_target=False,
        fail_on_invalid_action_target=False,
        fail_on_incomplete_node_tree=False,
        fail_on_node_tree_class_mismatch=False,
        fail_on_node_tree_missing_parameters=True,
        fail_on_node_tree_transform_mismatch=False,
        fail_on_node_tree_physical_mismatch=False,
        fail_on_node_tree_fixed_lock_mismatch=False,
        node_tree_tolerance=1e-4,
        parameter_tolerance=1e-4,
        action=[0.0],
        actions=[],
        stdout="",
        stderr="",
    )

    assert result["status"] == "error"
    assert result["node_tree_manifest"]["parameter_missing_count"] == 1
    assert any("node tree joint parameters missing" in item for item in result["errors"])


def test_node_tree_transform_mismatch_gate_adds_error() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    result = smoke_tool._build_result(
        robot_config={
            "name": "transform_mismatch",
            "parts": [
                {
                    "id": "torso",
                    "type": "torso",
                    "shape": "box",
                    "params": {
                        "position": [0.0, 1.0, 0.0],
                        "rotation": [0.0, 0.0, 0.0],
                    },
                }
            ],
            "connections": [
                {
                    "name": "fixed_joint",
                    "from": "torso",
                    "to": "torso",
                    "joint_type": "fixed",
                    "origin": [0.0, 1.0, 0.0],
                    "axis": [0.0, 0.0, 1.0],
                }
            ],
        },
        load_result={
            "status": "success",
            "parts_created": 1,
            "joints_created": 1,
            "part_nodes": [
                {
                    "part_id": "torso",
                    "body_node": "/robot/torso",
                    "body_class": "RigidBody3D",
                    "collision_shape": "BoxShape3D",
                    "mesh_type": "BoxMesh",
                    "position": [0.0, 1.2, 0.0],
                    "rotation": [0.0, 0.0, 0.0],
                }
            ],
            "joint_nodes": [
                {
                    "connection_name": "fixed_joint",
                    "joint_node": "/robot/fixed_joint",
                    "joint_class": "Generic6DOFJoint3D",
                    "origin": [0.0, 1.0, 0.0],
                    "axis": [0.0, 1.0, 0.0],
                    "applied_parameters": {"source": {}, "runtime": {}},
                }
            ],
        },
        schema_result={"meta": {"dynamic_robot_generation": True}},
        step_result={"body_count": 1, "joint_count": 1, "body_states": {}, "joint_states": {}},
        step_results=[],
        max_endpoint_distance=None,
        max_relative_angle=None,
        min_body_displacement=None,
        max_linear_speed=None,
        min_joint_angle_delta=None,
        min_joint_angle_range=None,
        min_moving_joint_coverage=None,
        min_commanded_joint_response_coverage=None,
        joint_motion_epsilon=0.0,
        min_action_target_coverage=None,
        min_control_action_coverage=None,
        min_nonzero_action_targets=None,
        min_action_transitions=None,
        min_action_transition_delta=None,
        fail_on_joint_limit_violation=False,
        fail_on_incomplete_restoration=False,
        min_restoration_score=None,
        fail_on_parameter_mismatch=False,
        fail_on_control_mismatch=False,
        fail_on_action_target_mismatch=False,
        fail_on_action_sequence_target_mismatch=False,
        fail_on_unknown_action_target=False,
        fail_on_invalid_action_target=False,
        fail_on_incomplete_node_tree=False,
        fail_on_node_tree_class_mismatch=False,
        fail_on_node_tree_missing_parameters=False,
        fail_on_node_tree_transform_mismatch=True,
        fail_on_node_tree_physical_mismatch=False,
        fail_on_node_tree_fixed_lock_mismatch=False,
        node_tree_tolerance=1e-4,
        parameter_tolerance=1e-4,
        action=[0.0],
        actions=[],
        stdout="",
        stderr="",
    )

    manifest = result["node_tree_manifest"]
    assert result["status"] == "error"
    assert manifest["transform_mismatch_count"] == 2
    assert manifest["transforms_complete"] is False
    assert {item["field"] for item in manifest["transform_mismatches"]} == {
        "position",
        "axis",
    }
    assert any("node tree transform mismatch detected" in item for item in result["errors"])


def test_node_tree_physical_mismatch_gate_adds_error() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    result = smoke_tool._build_result(
        robot_config={
            "name": "physical_mismatch",
            "parts": [
                {
                    "id": "torso",
                    "type": "torso",
                    "shape": "box",
                    "params": {
                        "mass": 5.0,
                        "size": [0.3, 0.2, 0.5],
                        "position": [0.0, 1.0, 0.0],
                        "rotation": [0.0, 0.0, 0.0],
                    },
                }
            ],
            "connections": [],
        },
        load_result={
            "status": "success",
            "parts_created": 1,
            "joints_created": 0,
            "part_nodes": [
                {
                    "part_id": "torso",
                    "body_node": "/robot/torso",
                    "body_class": "RigidBody3D",
                    "collision_shape": "BoxShape3D",
                    "collision_parameters": {"size": [0.2, 0.2, 0.2]},
                    "mesh_type": "BoxMesh",
                    "mesh_parameters": {"size": [0.2, 0.2, 0.2]},
                    "mass": 4.0,
                    "position": [0.0, 1.0, 0.0],
                    "rotation": [0.0, 0.0, 0.0],
                }
            ],
            "joint_nodes": [],
        },
        schema_result={"meta": {"dynamic_robot_generation": True}},
        step_result={"body_count": 1, "joint_count": 0, "body_states": {}, "joint_states": {}},
        step_results=[],
        max_endpoint_distance=None,
        max_relative_angle=None,
        min_body_displacement=None,
        max_linear_speed=None,
        min_joint_angle_delta=None,
        min_joint_angle_range=None,
        min_moving_joint_coverage=None,
        min_commanded_joint_response_coverage=None,
        joint_motion_epsilon=0.0,
        min_action_target_coverage=None,
        min_control_action_coverage=None,
        min_nonzero_action_targets=None,
        min_action_transitions=None,
        min_action_transition_delta=None,
        fail_on_joint_limit_violation=False,
        fail_on_incomplete_restoration=False,
        min_restoration_score=None,
        fail_on_parameter_mismatch=False,
        fail_on_control_mismatch=False,
        fail_on_action_target_mismatch=False,
        fail_on_action_sequence_target_mismatch=False,
        fail_on_unknown_action_target=False,
        fail_on_invalid_action_target=False,
        fail_on_incomplete_node_tree=False,
        fail_on_node_tree_class_mismatch=False,
        fail_on_node_tree_missing_parameters=False,
        fail_on_node_tree_transform_mismatch=False,
        fail_on_node_tree_physical_mismatch=True,
        fail_on_node_tree_fixed_lock_mismatch=False,
        node_tree_tolerance=1e-4,
        parameter_tolerance=1e-4,
        action=[0.0],
        actions=[],
        stdout="",
        stderr="",
    )

    manifest = result["node_tree_manifest"]
    assert result["status"] == "error"
    assert manifest["physical_mismatch_count"] == 3
    assert manifest["physical_complete"] is False
    assert {item["field"] for item in manifest["physical_mismatches"]} == {
        "mass",
        "collision_parameters.size",
        "mesh_parameters.size",
    }
    assert any(
        "node tree physical parameter mismatch detected" in item
        for item in result["errors"]
    )


def test_node_tree_fixed_lock_mismatch_gate_adds_error() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_smoke", SMOKE_TOOL)
    smoke_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(smoke_tool)

    result = smoke_tool._build_result(
        robot_config={
            "name": "fixed_lock_mismatch",
            "parts": [
                {"id": "base", "type": "base", "shape": "box"},
                {"id": "tool", "type": "tool", "shape": "box"},
            ],
            "connections": [
                {
                    "name": "weld",
                    "joint_type": "fixed",
                    "parent": "base",
                    "child": "tool",
                    "limits": {"lower": 0.0, "upper": 0.0},
                }
            ],
        },
        load_result={
            "status": "success",
            "parts_created": 2,
            "joints_created": 1,
            "part_nodes": [
                {
                    "part_id": "base",
                    "body_node": "/robot/base",
                    "body_class": "RigidBody3D",
                    "collision_shape": "BoxShape3D",
                    "mesh_type": "BoxMesh",
                },
                {
                    "part_id": "tool",
                    "body_node": "/robot/tool",
                    "body_class": "RigidBody3D",
                    "collision_shape": "BoxShape3D",
                    "mesh_type": "BoxMesh",
                },
            ],
            "joint_nodes": [
                {
                    "connection_name": "weld",
                    "joint_node": "/robot/weld",
                    "joint_class": "Generic6DOFJoint3D",
                    "applied_parameters": {
                        "runtime": {
                            "fixed_lock_applied": False,
                            "linear_limit_enabled": {"x": True, "y": True, "z": True},
                            "angular_limit_enabled": {"x": True, "y": True, "z": True},
                            "linear_lower": {"x": 0.0, "y": 0.0, "z": 0.0},
                            "linear_upper": {"x": 0.0, "y": 0.25, "z": 0.0},
                            "angular_lower": {"x": 0.0, "y": 0.0, "z": 0.0},
                            "angular_upper": {"x": 0.0, "y": 0.0, "z": 0.0},
                        }
                    },
                }
            ],
        },
        schema_result={"meta": {"dynamic_robot_generation": True}},
        step_result={"body_count": 2, "joint_count": 1, "body_states": {}, "joint_states": {}},
        step_results=[],
        max_endpoint_distance=None,
        max_relative_angle=None,
        min_body_displacement=None,
        max_linear_speed=None,
        min_joint_angle_delta=None,
        min_joint_angle_range=None,
        min_moving_joint_coverage=None,
        min_commanded_joint_response_coverage=None,
        joint_motion_epsilon=0.0,
        min_action_target_coverage=None,
        min_control_action_coverage=None,
        min_nonzero_action_targets=None,
        min_action_transitions=None,
        min_action_transition_delta=None,
        fail_on_joint_limit_violation=False,
        fail_on_incomplete_restoration=False,
        min_restoration_score=None,
        fail_on_parameter_mismatch=False,
        fail_on_control_mismatch=False,
        fail_on_action_target_mismatch=False,
        fail_on_action_sequence_target_mismatch=False,
        fail_on_unknown_action_target=False,
        fail_on_invalid_action_target=False,
        fail_on_incomplete_node_tree=False,
        fail_on_node_tree_class_mismatch=False,
        fail_on_node_tree_missing_parameters=False,
        fail_on_node_tree_transform_mismatch=False,
        fail_on_node_tree_physical_mismatch=False,
        fail_on_node_tree_fixed_lock_mismatch=True,
        node_tree_tolerance=1e-4,
        parameter_tolerance=1e-4,
        action=[0.0],
        actions=[],
        stdout="",
        stderr="",
    )

    manifest = result["node_tree_manifest"]
    assert result["status"] == "error"
    assert manifest["fixed_lock_checked_count"] == 1
    assert manifest["fixed_lock_mismatch_count"] == 2
    assert manifest["fixed_locks_complete"] is False
    assert {item["field"] for item in manifest["fixed_lock_mismatches"]} == {
        "fixed.lock_applied",
        "fixed.linear_upper.y",
    }
    assert any(
        "node tree fixed joint lock mismatch detected" in item
        for item in result["errors"]
    )


def test_dynamic_robot_generation_report_tool_builds_static_report(
    tmp_path: Path,
) -> None:
    output_path = tmp_path / "report.json"
    normalized_path = tmp_path / "normalized.json"
    gate_output_path = tmp_path / "gate.json"
    manifest_output_dir = tmp_path / "node_tree_manifests"

    result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(BIPED_TEMPLATE),
            "--output",
            str(output_path),
            "--normalized-output",
            str(normalized_path),
            "--gate-output",
            str(gate_output_path),
            "--static-node-tree-manifest-dir",
            str(manifest_output_dir),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    report = json.loads(output_path.read_text(encoding="utf-8"))
    normalized = json.loads(normalized_path.read_text(encoding="utf-8"))
    gate_report = json.loads(gate_output_path.read_text(encoding="utf-8"))
    manifest_output_path = (
        manifest_output_dir / "000_biped_basic.node_tree_manifest.json"
    )
    manifest_output = json.loads(manifest_output_path.read_text(encoding="utf-8"))
    assert report["status"] == "success"
    assert report["static_node_tree_manifest_output"] == str(manifest_output_path)
    assert report["static"]["parts_count"] == 5
    assert report["static"]["connections_count"] == 4
    assert report["static"]["topology_summary"] == {
        "schema_version": robot_schema.ROBOT_MECHANICAL_SCHEMA_VERSION,
        "parts_count": 5,
        "connections_count": 4,
        "root_parts": ["torso_1"],
        "root_part": "torso_1",
        "connected_parts_count": 5,
        "reachable_parts_count": 5,
        "disconnected_parts": [],
        "unreachable_parts": [],
        "duplicate_child_endpoints": [],
        "cycle": [],
        "complete_tree": True,
    }
    assert report["static"]["node_tree_manifest"]["manifest_version"] == (
        "godot_node_tree_manifest.v1"
    )
    assert report["static"]["node_tree_manifest_errors"] == []
    assert report["static"]["node_tree_manifest_path_map_mismatch_count"] == 0
    assert report["static"]["node_tree_manifest_path_map_mismatch_kind_counts"] == {}
    assert report["static"]["node_tree_manifest_path_map_mismatches"] == []
    assert report["static"]["node_tree_manifest"]["robot_node"] == "biped_basic"
    assert report["static"]["node_tree_manifest"]["controller_node"] == (
        "biped_basic/GeneratedRobotController"
    )
    assert report["static"]["node_tree_manifest"]["parts_count"] == 5
    assert report["static"]["node_tree_manifest"]["joints_count"] == 4
    assert report["static"]["node_tree_manifest"]["parameterized_joints"] == 4
    assert report["static"]["node_tree_manifest"]["parameters_complete"] is True
    assert report["static"]["node_tree_manifest"]["part_nodes"][0]["body_class"] == (
        "RigidBody3D"
    )
    assert report["static"]["node_tree_manifest"]["part_nodes"][0][
        "collision_node"
    ] == "biped_basic/torso_1/Collision"
    assert report["static"]["node_tree_manifest"]["joint_nodes"][0]["joint_class"] == (
        "HingeJoint3D"
    )
    assert report["static"]["node_tree_manifest"]["joint_nodes"][0][
        "applied_parameters"
    ]["motor_max_impulse"] == 100.0
    assert manifest_output == report["static"]["node_tree_manifest"]
    assert report["delivery_contract_preview"] == {
        "dynamic_robot_generation": True,
        "complete": True,
        "expected_parts": 5,
        "parts_created": 5,
        "parts_complete": True,
        "expected_joints": 4,
        "joints_created": 4,
        "failed_joints": None,
        "joints_complete": True,
        "parameterized_joints": None,
        "parameters_complete": None,
        "part_nodes_count": None,
        "joint_nodes_count": None,
        "failed_connections_count": None,
        "warnings_count": None,
        "source": "static_normalization",
    }
    assert report["delivery_acceptance_summary"] == {
        "delivery_acceptance_complete": False,
        "acceptance_profile": "custom",
        "delivery_acceptance_level": "static_only",
        "delivery_acceptance_reasons": [
            "1 robot(s) were not run through Godot smoke",
            "1 robot(s) only passed static normalization",
        ],
        "delivery_acceptance_reason_codes": [
            "missing_godot_smoke",
            "static_only",
        ],
        "delivery_acceptance_reason_details": [
            {
                "code": "missing_godot_smoke",
                "count": 1,
                "message": "1 robot(s) were not run through Godot smoke",
                "inputs": [str(BIPED_TEMPLATE)],
                "inputs_count": 1,
                "inputs_truncated": False,
            },
            {
                "code": "static_only",
                "count": 1,
                "message": "1 robot(s) only passed static normalization",
                "inputs": [str(BIPED_TEMPLATE)],
                "inputs_count": 1,
                "inputs_truncated": False,
            },
        ],
    }
    assert report["delivery_acceptance_gate"] == {
        "contract_version": "delivery_acceptance_gate.v1",
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": False,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": _acceptance_requirements(),
        "passed": True,
        "exit_code": 0,
        "level": "static_only",
        "complete": False,
        "reasons": [
            "1 robot(s) were not run through Godot smoke",
            "1 robot(s) only passed static normalization",
        ],
        "reason_codes": [
            "missing_godot_smoke",
            "static_only",
        ],
        "reason_details": [
            {
                "code": "missing_godot_smoke",
                "count": 1,
                "message": "1 robot(s) were not run through Godot smoke",
                "inputs": [str(BIPED_TEMPLATE)],
                "inputs_count": 1,
                "inputs_truncated": False,
            },
            {
                "code": "static_only",
                "count": 1,
                "message": "1 robot(s) only passed static normalization",
                "inputs": [str(BIPED_TEMPLATE)],
                "inputs_count": 1,
                "inputs_truncated": False,
            },
        ],
        "summary_counts": {
            "inputs_count": 1,
            "success_count": 1,
            "error_count": 0,
            "live_smoke_count": 0,
            "smoke_report_written_count": 0,
            "smoke_report_missing_count": 0,
            "smoke_report_read_error_count": 0,
            "delivery_godot_verified_count": 0,
            "delivery_static_only_count": 1,
            "delivery_unverified_count": 0,
            "delivery_dynamic_generation_count": 1,
            "delivery_complete_count": 1,
            "delivery_incomplete_count": 0,
            "delivery_parameters_incomplete_count": 0,
            "fixed_lock_checked_count": 0,
            "fixed_lock_mismatch_count": 0,
            "control_configured_count": 0,
            "control_readback_checked_count": 0,
            "control_readback_missing_count": 0,
            "node_tree_fixed_lock_checked_count": 0,
            "node_tree_fixed_lock_mismatch_count": 0,
            "node_tree_fixed_locks_complete_count": 0,
            "node_tree_fixed_locks_incomplete_count": 0,
            "node_tree_gate_enabled_count": 0,
            "node_tree_full_restoration_required_count": 0,
            "node_tree_full_restoration_not_required_count": 0,
            "node_tree_gate_check_counts": {
                "incomplete_node_tree": 0,
                "class_mismatch": 0,
                "missing_parameters": 0,
                "transform_mismatch": 0,
                "physical_mismatch": 0,
                "fixed_lock_mismatch": 0,
            },
            "static_topology_complete_count": 1,
            "static_topology_incomplete_count": 0,
            "static_topology_disconnected_parts_count": 0,
            "static_topology_unreachable_parts_count": 0,
            "static_topology_duplicate_child_endpoint_count": 0,
            "static_topology_cycle_count": 0,
            "static_node_tree_manifest_count": 1,
            "static_node_tree_manifest_valid_count": 1,
            "static_node_tree_manifest_invalid_count": 0,
            "static_node_tree_manifest_error_count": 0,
            "static_node_tree_manifest_output_count": 1,
            "static_node_tree_manifest_path_map_mismatch_count": 0,
            "static_node_tree_manifest_path_map_mismatch_kind_counts": {},
            "static_node_tree_parts_planned_count": 5,
            "static_node_tree_joints_planned_count": 4,
            "static_node_tree_parameterized_joints_count": 4,
            "static_node_tree_complete_count": 1,
            "static_node_tree_incomplete_count": 0,
            "static_node_tree_endpoint_paths_complete_count": 1,
            "static_node_tree_endpoint_paths_incomplete_count": 0,
            "static_node_tree_missing_endpoint_parts_count": 0,
            "static_node_tree_missing_endpoint_connections_count": 0,
            "static_node_tree_parameters_complete_count": 1,
            "static_node_tree_parameters_incomplete_count": 0,
            "mechanical_gate_enabled_count": 0,
            "full_mechanical_restoration_required_count": 0,
            "full_mechanical_restoration_not_required_count": 0,
            "mechanical_gate_check_counts": {
                "mechanical_restoration": 0,
                "joint_parameter_readback": 0,
                "control_parameter_readback": 0,
                "full_node_tree_restoration": 0,
            },
            "mechanical_behavior_evidence_count": 0,
            "mechanical_behavior_complete_count": 0,
            "mechanical_behavior_incomplete_count": 0,
            "mechanical_behavior_residual_risk_count": 0,
            "mechanical_behavior_threshold_failure_count": 0,
            "mechanical_behavior_center_of_mass_available_count": 0,
            "mechanical_behavior_contact_state_available_count": 0,
            "mechanical_behavior_step_trace_artifact_count": 0,
            "failure_reasons_count": 0,
        },
    }
    assert (
        workflow_contracts.validate_delivery_acceptance_gate(
            report["delivery_acceptance_gate"]
        )
        == []
    )
    assert gate_report == report["delivery_acceptance_gate"]
    assert "static" not in gate_report
    assert "delivery_acceptance_summary" not in gate_report
    assert "batch_summary" not in gate_report
    assert "reports" not in gate_report
    assert normalized["parts"][1]["id"] == "leg_left_1_upper"
    assert robot_schema.validate_godot_robot_config(normalized) == []


def test_dynamic_robot_generation_report_can_require_static_node_tree_manifest_output(
    tmp_path: Path,
) -> None:
    missing_output_path = tmp_path / "missing_manifest_output_report.json"
    missing_gate_path = tmp_path / "missing_manifest_output_gate.json"

    missing_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--require-static-node-tree-manifest-output",
            "--output",
            str(missing_output_path),
            "--gate-output",
            str(missing_gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert missing_result.returncode == 1
    missing_report = json.loads(missing_output_path.read_text(encoding="utf-8"))
    missing_gate = json.loads(missing_gate_path.read_text(encoding="utf-8"))
    assert missing_report["status"] == "success"
    assert missing_report["delivery_acceptance_gate"]["passed"] is False
    assert missing_report["delivery_acceptance_gate"]["exit_code"] == 1
    assert missing_report["delivery_acceptance_gate"]["acceptance_requirements"] == (
        _acceptance_requirements(static_node_tree_manifest_output=True)
    )
    assert "missing_static_node_tree_manifest_output" in missing_report[
        "delivery_acceptance_gate"
    ]["reason_codes"]
    assert missing_report["delivery_acceptance_gate"]["reason_details"][-1] == {
        "code": "missing_static_node_tree_manifest_output",
        "count": 1,
        "message": (
            "1 robot(s) did not write a static Godot node-tree manifest artifact"
        ),
        "inputs": [str(FIXED_PAIR_FIXTURE)],
        "inputs_count": 1,
        "inputs_truncated": False,
    }
    assert missing_report["delivery_acceptance_gate"]["summary_counts"][
        "static_node_tree_manifest_output_count"
    ] == 0
    assert missing_gate == missing_report["delivery_acceptance_gate"]
    assert "requirements=static_node_tree_manifest_output" in missing_result.stderr
    assert "missing_static_node_tree_manifest_output" in missing_result.stderr

    manifest_output_dir = tmp_path / "node_tree_manifests"
    present_output_path = tmp_path / "present_manifest_output_report.json"
    present_gate_path = tmp_path / "present_manifest_output_gate.json"
    present_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--require-static-node-tree-manifest-output",
            "--static-node-tree-manifest-dir",
            str(manifest_output_dir),
            "--output",
            str(present_output_path),
            "--gate-output",
            str(present_gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert present_result.returncode == 0
    present_report = json.loads(present_output_path.read_text(encoding="utf-8"))
    present_gate = json.loads(present_gate_path.read_text(encoding="utf-8"))
    assert present_report["delivery_acceptance_gate"]["passed"] is True
    assert present_report["delivery_acceptance_gate"]["exit_code"] == 0
    assert "missing_static_node_tree_manifest_output" not in present_report[
        "delivery_acceptance_gate"
    ]["reason_codes"]
    assert present_report["delivery_acceptance_gate"]["summary_counts"][
        "static_node_tree_manifest_output_count"
    ] == 1
    assert present_gate == present_report["delivery_acceptance_gate"]


def test_static_node_tree_manifest_ci_loop_validates_generated_sidecars(
    tmp_path: Path,
) -> None:
    report_path = tmp_path / "static_manifest_ci" / "report.json"
    gate_path = tmp_path / "static_manifest_ci" / "gate.json"
    summary_path = tmp_path / "static_manifest_ci" / "summary.json"
    manifest_output_dir = tmp_path / "static_manifest_ci" / "node_tree_manifests"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            str(FIXTURE),
            "--require-static-node-tree-manifest-output",
            "--static-node-tree-manifest-dir",
            str(manifest_output_dir),
            "--output",
            str(report_path),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert report_result.returncode == 0, report_result.stderr
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["batch_summary"]["static_node_tree_manifest_output_count"] == 2
    assert report["batch_summary"]["static_node_tree_manifest_error_count"] == 0
    assert (
        report["batch_summary"]["static_node_tree_manifest_path_map_mismatch_count"]
        == 0
    )
    assert (
        report["batch_summary"][
            "static_node_tree_manifest_path_map_mismatch_kind_counts"
        ]
        == {}
    )
    assert len(report["batch_summary"]["static_node_tree_manifest_outputs"]) == 2
    assert sorted(path.name for path in manifest_output_dir.glob("*.json")) == [
        "000_robot_dynamic_fixed_pair.node_tree_manifest.json",
        "001_robot_dynamic_biped.node_tree_manifest.json",
    ]

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(manifest_output_dir),
            "--fail-on-invalid-node-tree-manifest-sidecar",
            "--fail-on-node-tree-manifest-sidecar-validation-error",
            "--fail-on-node-tree-manifest-sidecar-path-incomplete",
            "--fail-on-node-tree-manifest-sidecar-path-map-mismatch",
            "--expect-node-tree-manifest-sidecar-count",
            "2",
            "--expect-node-tree-manifest-sidecar-complete-count",
            "2",
            "--expect-node-tree-manifest-sidecar-incomplete-count",
            "0",
            "--expect-node-tree-manifest-sidecar-valid-count",
            "2",
            "--expect-node-tree-manifest-sidecar-invalid-count",
            "0",
            "--expect-node-tree-manifest-sidecar-validation-error-count",
            "0",
            "--expect-node-tree-manifest-sidecar-path-incomplete-count",
            "0",
            "--expect-node-tree-manifest-sidecar-path-map-mismatch-count",
            "0",
            "--expect-node-tree-manifest-sidecar-path-map-mismatch-kind",
            "missing=0",
            "--expect-node-tree-manifest-sidecar-path-map-mismatch-kind",
            "unexpected=0",
            "--expect-node-tree-manifest-sidecar-path-map-mismatch-kind",
            "value_mismatch=0",
            "--expect-node-tree-manifest-sidecar-path-map-mismatch-kind",
            "duplicate=0",
            "--expect-node-tree-manifest-sidecar-path-map-mismatch-kind",
            "root_mismatch=0",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0, validate_result.stderr
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["node_tree_manifest_sidecar_count"] == 2
    assert payload["node_tree_manifest_sidecar_valid_count"] == 2
    assert payload["node_tree_manifest_sidecar_invalid_count"] == 0
    assert payload["node_tree_manifest_sidecar_path_map_mismatch_count"] == 0
    assert payload["node_tree_manifest_sidecar_path_map_mismatch_kind_counts"] == {}
    assert payload[
        "expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts"
    ] == {
        "duplicate": 0,
        "missing": 0,
        "root_mismatch": 0,
        "unexpected": 0,
        "value_mismatch": 0,
    }
    assert summary["node_tree_manifest_sidecar_count"] == 2
    assert summary["node_tree_manifest_sidecar_valid_count"] == 2
    assert summary["errors"] == []


def test_static_godot_node_tree_evidence_tool_writes_closeout(
    tmp_path: Path,
) -> None:
    output_root = tmp_path / "static_manifest_ci"
    manifest_dir = tmp_path / "static_node_tree_manifests"

    result = subprocess.run(
        [
            sys.executable,
            str(STATIC_EVIDENCE_TOOL),
            "--output-root",
            str(output_root),
            "--manifest-dir",
            str(manifest_dir),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    payload = json.loads(result.stdout)
    closeout_path = output_root / "static_godot_node_tree_evidence_closeout.json"
    closeout = json.loads(closeout_path.read_text(encoding="utf-8"))
    assert payload == closeout
    assert closeout["status"] == "success"
    assert closeout["acceptance_level"] == "static_only"
    assert closeout["live_godot_smoke_run"] is False
    assert closeout["inputs"] == [
        str(FIXED_PAIR_FIXTURE.resolve()),
        str(FIXTURE.resolve()),
        str(QUADRUPED_FIXTURE.resolve()),
    ]
    assert closeout["input_count"] == 3
    assert closeout["node_tree_manifest_sidecar_count"] == 3
    assert closeout["node_tree_manifest_sidecar_valid_count"] == 3
    assert closeout["node_tree_manifest_sidecar_invalid_count"] == 0
    assert closeout["node_tree_manifest_sidecar_validation_error_count"] == 0
    assert closeout["node_tree_manifest_sidecar_path_map_mismatch_count"] == 0
    assert closeout["node_tree_manifest_sidecar_path_map_mismatch_kind_counts"] == {}
    assert closeout["errors"] == []
    assert closeout["commands"]["report"]["returncode"] == 0
    assert closeout["commands"]["validation"]["returncode"] == 0

    for artifact_path in closeout["artifacts"].values():
        assert Path(artifact_path).exists()

    validation_summary = json.loads(
        Path(closeout["artifacts"]["validation_summary"]).read_text(
            encoding="utf-8"
        )
    )
    assert validation_summary["expected_node_tree_manifest_sidecar_count"] == 3
    assert validation_summary[
        "expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts"
    ] == {
        "duplicate": 0,
        "missing": 0,
        "root_mismatch": 0,
        "unexpected": 0,
        "value_mismatch": 0,
    }


def test_dynamic_godot_release_readiness_reports_static_only(
    tmp_path: Path,
) -> None:
    closeout_path = tmp_path / "static_closeout.json"
    output_path = tmp_path / "readiness.json"
    closeout_path.write_text(
        json.dumps(
            {
                "status": "success",
                "acceptance_level": "static_only",
                "residual_risks": ["Live Godot smoke was not run."],
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(RELEASE_READINESS_TOOL),
            str(closeout_path),
            "--output",
            str(output_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    payload = json.loads(result.stdout)
    assert payload == json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["summary_version"] == "dynamic_godot_release_readiness_summary.v1"
    assert payload["artifact_type"] == "dynamic_godot_release_readiness_summary"
    assert payload["status"] == "ready"
    assert payload["proven_level"] == "static_only"
    assert payload["levels_found"] == ["static_only"]
    assert payload["evidence"][0]["kind"] == "static_godot_node_tree_evidence_closeout"
    assert (
        "Full live Godot smoke-motion verification is not proven by the supplied evidence."
        in payload["residual_risks"]
    )


def test_dynamic_godot_release_readiness_prefers_web_load_over_static(
    tmp_path: Path,
) -> None:
    closeout_path = tmp_path / "static_closeout.json"
    web_delivery_path = tmp_path / "web_delivery.json"
    closeout_path.write_text(
        json.dumps({"status": "success", "acceptance_level": "static_only"}),
        encoding="utf-8",
    )
    web_delivery_path.write_text(
        json.dumps(
            {
                "status": "success",
                "delivery_acceptance_gate": {
                    "contract_version": "delivery_acceptance_gate.v1",
                    "source": "web_godot_delivery",
                    "verification_scope": "godot_load",
                    "acceptance_profile": "web_godot_load",
                    "level": "godot_load_verified",
                    "passed": True,
                    "complete": True,
                    "exit_code": 0,
                    "reason_codes": [],
                },
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(RELEASE_READINESS_TOOL),
            str(closeout_path),
            str(web_delivery_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    payload = json.loads(result.stdout)
    assert payload["status"] == "ready"
    assert payload["proven_level"] == "godot_load_verified"
    assert payload["levels_found"] == ["static_only", "godot_load_verified"]
    assert payload["evidence_count"] == 2
    assert (
        "Godot load through Web/session delivery is not proven by the supplied evidence."
        not in payload["residual_risks"]
    )
    assert (
        "Full live Godot smoke-motion verification is not proven by the supplied evidence."
        in payload["residual_risks"]
    )


def test_dynamic_godot_release_readiness_prefers_full_motion_gate(
    tmp_path: Path,
) -> None:
    web_gate_path = tmp_path / "web_gate.json"
    full_gate_path = tmp_path / "full_gate.json"
    web_gate_path.write_text(
        json.dumps(
            {
                "contract_version": "delivery_acceptance_gate.v1",
                "source": "web_godot_delivery",
                "verification_scope": "godot_load",
                "acceptance_profile": "web_godot_load",
                "level": "godot_load_verified",
                "passed": True,
                "complete": True,
                "exit_code": 0,
                "reason_codes": [],
            }
        ),
        encoding="utf-8",
    )
    full_gate_path.write_text(
        json.dumps(
            {
                "contract_version": "delivery_acceptance_gate.v1",
                "source": "dynamic_godot_report_cli",
                "verification_scope": "godot_smoke_motion",
                "acceptance_profile": "full_mechanical_restoration",
                "level": "godot_verified",
                "passed": True,
                "complete": True,
                "exit_code": 0,
                "reason_codes": [],
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(RELEASE_READINESS_TOOL),
            str(web_gate_path),
            str(full_gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    payload = json.loads(result.stdout)
    assert payload["status"] == "ready"
    assert payload["proven_level"] == "godot_verified"
    assert payload["proven_level_rank"] == 3
    assert payload["levels_found"] == ["godot_load_verified", "godot_verified"]
    assert payload["residual_risks"] == []


def test_dynamic_godot_release_readiness_blocks_unrecognized_input(
    tmp_path: Path,
) -> None:
    unknown_path = tmp_path / "unknown.json"
    unknown_path.write_text(json.dumps({"status": "success"}), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(RELEASE_READINESS_TOOL), str(unknown_path)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    payload = json.loads(result.stdout)
    assert payload["status"] == "blocked"
    assert payload["proven_level"] == "incomplete"
    assert payload["evidence_count"] == 0
    assert payload["input_errors"] == [
        {
            "path": str(unknown_path.resolve()),
            "error": "no recognized dynamic Godot evidence",
        }
    ]


def _write_static_release_bundle_inputs(tmp_path: Path) -> tuple[Path, Path, Path]:
    closeout_path = tmp_path / "static_closeout.json"
    gate_path = tmp_path / "delivery_gate.json"
    readiness_path = tmp_path / "readiness.json"
    closeout_path.write_text(
        json.dumps(
            {
                "status": "success",
                "acceptance_level": "static_only",
                "residual_risks": ["Live Godot smoke was not run."],
            }
        ),
        encoding="utf-8",
    )
    gate_path.write_text(
        json.dumps(
            {
                "contract_version": "delivery_acceptance_gate.v1",
                "source": "dynamic_godot_static_evidence",
                "verification_scope": "static_manifest",
                "acceptance_profile": "static_godot_node_tree_evidence",
                "level": "static_only",
                "passed": True,
                "complete": True,
                "exit_code": 0,
                "reason_codes": [],
            }
        ),
        encoding="utf-8",
    )
    readiness_path.write_text(
        json.dumps(
            {
                "summary_version": "dynamic_godot_release_readiness_summary.v1",
                "artifact_type": "dynamic_godot_release_readiness_summary",
                "status": "ready",
                "proven_level": "static_only",
                "proven_level_rank": 1,
                "levels_found": ["static_only"],
                "residual_risks": ["Full live Godot smoke-motion verification is not proven."],
            }
        ),
        encoding="utf-8",
    )
    return closeout_path, gate_path, readiness_path


def test_dynamic_godot_release_evidence_bundle_builds_and_validates_static_only(
    tmp_path: Path,
) -> None:
    closeout_path, gate_path, readiness_path = _write_static_release_bundle_inputs(
        tmp_path
    )
    output_root = tmp_path / "bundle"

    result = subprocess.run(
        [
            sys.executable,
            str(RELEASE_EVIDENCE_BUNDLE_TOOL),
            "--static-closeout",
            str(closeout_path),
            "--delivery-gate",
            str(gate_path),
            "--readiness-summary",
            str(readiness_path),
            "--output-root",
            str(output_root),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    index = json.loads(result.stdout)
    index_path = output_root / "bundle_index.json"
    validation_path = output_root / "bundle_validation.json"
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    assert index == json.loads(index_path.read_text(encoding="utf-8"))
    assert index["bundle_version"] == "dynamic_godot_release_evidence_bundle.v1"
    assert index["artifact_type"] == "dynamic_godot_release_evidence_bundle"
    assert index["evidence_level"] == "static_only"
    assert index["evidence_level_rank"] == 1
    assert index["validation_status"] == "ready"
    assert validation["status"] == "ready"
    assert {entry["key"] for entry in index["artifacts"]} == {
        "static_closeout",
        "delivery_gate",
        "readiness_summary",
    }
    assert {entry["role"] for entry in index["documentation"]} == {
        "static_workflow",
        "live_workflow",
        "web_workflow",
        "readiness_workflow",
    }
    for entry in [*index["artifacts"], *index["documentation"]]:
        assert (output_root / entry["bundle_path"]).exists()
        assert entry["size_bytes"] > 0
        assert len(entry["sha256"]) == 64

    validate_result = subprocess.run(
        [sys.executable, str(RELEASE_EVIDENCE_BUNDLE_VALIDATOR_TOOL), str(index_path)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert validate_result.returncode == 0
    assert json.loads(validate_result.stdout)["status"] == "ready"


def test_dynamic_godot_release_evidence_bundle_validator_rejects_missing_artifact(
    tmp_path: Path,
) -> None:
    closeout_path, gate_path, readiness_path = _write_static_release_bundle_inputs(
        tmp_path
    )
    output_root = tmp_path / "bundle"
    subprocess.run(
        [
            sys.executable,
            str(RELEASE_EVIDENCE_BUNDLE_TOOL),
            "--static-closeout",
            str(closeout_path),
            "--delivery-gate",
            str(gate_path),
            "--readiness-summary",
            str(readiness_path),
            "--output-root",
            str(output_root),
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    (output_root / "artifacts" / "delivery_gate.json").unlink()

    result = subprocess.run(
        [
            sys.executable,
            str(RELEASE_EVIDENCE_BUNDLE_VALIDATOR_TOOL),
            str(output_root / "bundle_index.json"),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    payload = json.loads(result.stdout)
    assert payload["status"] == "invalid"
    assert any("delivery_gate" in error for error in payload["errors"])


def test_dynamic_godot_release_evidence_bundle_validator_rejects_malformed_bundle(
    tmp_path: Path,
) -> None:
    closeout_path, gate_path, readiness_path = _write_static_release_bundle_inputs(
        tmp_path
    )
    output_root = tmp_path / "bundle"
    subprocess.run(
        [
            sys.executable,
            str(RELEASE_EVIDENCE_BUNDLE_TOOL),
            "--static-closeout",
            str(closeout_path),
            "--delivery-gate",
            str(gate_path),
            "--readiness-summary",
            str(readiness_path),
            "--output-root",
            str(output_root),
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    readiness_copy = output_root / "artifacts" / "readiness_summary.json"
    readiness_copy.write_text(
        json.dumps(
            {
                "summary_version": "dynamic_godot_release_readiness_summary.v1",
                "artifact_type": "dynamic_godot_release_readiness_summary",
                "status": "blocked",
                "proven_level": "godot_verified",
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(RELEASE_EVIDENCE_BUNDLE_VALIDATOR_TOOL),
            str(output_root / "bundle_index.json"),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    payload = json.loads(result.stdout)
    assert payload["status"] == "invalid"
    assert "artifacts.readiness_summary sha256 must equal" in "\n".join(
        payload["errors"]
    )
    assert "readiness_summary.status must be 'ready'" in payload["errors"]


def test_dynamic_godot_manual_live_smoke_checklist_documents_artifacts_and_fields() -> None:
    content = (ROOT / "docs" / "guides" / "DYNAMIC_GODOT_ROBOT_GENERATION.md").read_text(
        encoding="utf-8"
    )

    assert "--full-mechanical-restoration-acceptance" in content
    assert "--smoke-output test_env/dynamic_godot_biped_live_smoke.json" in content
    assert "--output test_env/dynamic_godot_biped_live_report.json" in content
    assert "--gate-output test_env/dynamic_godot_biped_live_gate.json" in content
    assert (
        "test_env/dynamic_godot_release_readiness_live.json"
        in content
    )
    assert "delivery_acceptance_gate.level=godot_verified" in content
    assert "delivery_acceptance_gate.summary_counts.live_smoke_count=1" in content
    assert "node_tree_manifest.static_manifest_comparison.complete=true" in content
    assert "node_tree_manifest.static_manifest_comparison.mismatch_count=0" in content
    assert "dynamic_godot_mechanical_behavior_evidence.v1" in content
    assert "--mechanical-trace-output path/to/trace.json" in content
    assert "dynamic_godot_mechanical_behavior_trace.v1" in content
    assert "step_trace_evidence.artifact_path" in content
    assert "center_of_mass_runtime_readback_missing" in content
    assert "contact_state_runtime_readback_missing" in content
    assert "proven_level=godot_verified" in content
    assert "proven_level_rank=3" in content


def test_dynamic_godot_acceptance_levels_document_proof_commands() -> None:
    content = (ROOT / "docs" / "guides" / "DYNAMIC_GODOT_ROBOT_GENERATION.md").read_text(
        encoding="utf-8"
    )

    assert "Acceptance levels and proof commands:" in content
    assert "| `static_only` |" in content
    assert "| `godot_load_verified` |" in content
    assert "| `godot_verified` |" in content
    assert "tools/build_static_godot_node_tree_evidence.py" in content
    assert "test_env/static_godot_node_tree_manifest_ci/static_godot_node_tree_evidence_closeout.json" in content
    assert "/api/workflows/runs/{run_id}/artifacts/{artifact_index}/godot-load" in content
    assert "/api/workflows/runs/{run_id}/godot-sync" in content
    assert "test_env/dynamic_godot_release_readiness_load.json" in content
    assert "tools/build_dynamic_robot_generation_report.py tests/fixtures/robot_dynamic_biped.json" in content
    assert "test_env/dynamic_godot_biped_live_gate.json" in content
    assert "proven_level=static_only" in content
    assert "proven_level_rank=1" in content
    assert "proven_level=godot_load_verified" in content
    assert "proven_level_rank=2" in content
    assert "proven_level=godot_verified" in content
    assert "proven_level_rank=3" in content
    assert "`incomplete` is not an acceptance level" in content
    assert "tools/build_dynamic_godot_release_evidence_bundle.py" in content
    assert "tools/validate_dynamic_godot_release_evidence_bundle.py" in content
    assert "dynamic_godot_release_evidence_bundle.v1" in content
    assert "dynamic_godot_release_evidence_bundle_validation.v1" in content


def test_ci_runs_static_node_tree_manifest_sidecar_gate() -> None:
    content = CI_WORKFLOW.read_text(encoding="utf-8")

    assert "static-godot-node-tree-manifest:" in content
    assert (
        "python tools/build_static_godot_node_tree_evidence.py "
        "tests/fixtures/robot_dynamic_fixed_pair.json "
        "tests/fixtures/robot_dynamic_biped.json "
        "tests/fixtures/robot_dynamic_quadruped.json"
    ) in content
    assert "--output-root test_env/static_godot_node_tree_manifest_ci" in content
    assert "--manifest-dir test_env/static_godot_node_tree_manifests" in content


def test_ci_keeps_browser_manual_validation_out_of_static_godot_gate() -> None:
    content = CI_WORKFLOW.read_text(encoding="utf-8")
    static_job = content.split("static-godot-node-tree-manifest:", 1)[1].split(
        "# Job 2b:",
        1,
    )[0]

    assert "tools/run_web_browser_playwright_smoke.py" not in static_job
    assert "tools/build_web_browser_manual_validation_report.py" not in static_job
    assert "tools/build_web_browser_validation_closeout.py" not in static_job
    assert "tools/build_web_browser_validation_evidence_pack.py" not in static_job
    assert "python tools/build_static_godot_node_tree_evidence.py" in static_job


def test_ci_exposes_dynamic_godot_live_verification_manual_scheduled_profile() -> None:
    content = CI_WORKFLOW.read_text(encoding="utf-8")
    live_job = content.split("dynamic-godot-live-verification:", 1)[1].split(
        "# -----------------------------------------------------------------------------",
        1,
    )[0]

    assert "if: github.event_name == 'workflow_dispatch' || github.event_name == 'schedule'" in live_job
    assert "runs-on: windows-latest" in live_job
    assert "AGI_WALKER_DYNAMIC_GODOT_LIVE_PROFILE" in live_job
    assert "scheduled_ci" in live_job
    assert "manual_ci" in live_job
    assert "tools/run_dynamic_godot_robot_smoke.py" in live_job
    assert "--dry-run-discovery" in live_job
    assert "--live-profile $profile" in live_job
    assert "tools/build_dynamic_robot_generation_report.py" in live_job
    assert "--full-mechanical-restoration-acceptance" in live_job
    assert "GODOT_EXECUTABLE" in live_job
    assert "tools/build_dynamic_godot_release_readiness.py" in live_job
    assert "dynamic-godot-live-verification-artifacts" in live_job
    assert "pull_request" not in live_job


def test_dynamic_robot_generation_report_can_fail_on_static_node_tree_incomplete(
    tmp_path: Path,
) -> None:
    complete_output_path = tmp_path / "complete_static_node_tree_report.json"
    complete_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--fail-on-static-node-tree-incomplete",
            "--output",
            str(complete_output_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert complete_result.returncode == 0
    complete_report = json.loads(complete_output_path.read_text(encoding="utf-8"))
    assert complete_report["delivery_acceptance_gate"]["acceptance_requirements"] == (
        _acceptance_requirements(static_node_tree_complete=True)
    )
    assert "static_node_tree_incomplete" not in complete_report[
        "delivery_acceptance_gate"
    ]["reason_codes"]

    invalid_config = tmp_path / "missing_endpoint_robot.json"
    invalid_output_path = tmp_path / "incomplete_static_node_tree_report.json"
    invalid_config.write_text(
        json.dumps(
            {
                "name": "missing_endpoint_robot",
                "parts": [
                    {
                        "id": "torso",
                        "type": "torso",
                        "shape": "box",
                        "params": {
                            "mass": 1.0,
                            "position": [0.0, 0.0, 0.0],
                            "rotation": [0.0, 0.0, 0.0],
                            "size": [0.2, 0.2, 0.2],
                        },
                    }
                ],
                "connections": [
                    {
                        "name": "torso_to_missing_leg",
                        "from": "torso",
                        "to": "missing_leg",
                        "joint_type": "hinge",
                        "origin": [0.0, -0.2, 0.0],
                        "axis": [1.0, 0.0, 0.0],
                        "limits": {"lower": -1.0, "upper": 1.0},
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    invalid_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(invalid_config),
            "--fail-on-static-node-tree-incomplete",
            "--output",
            str(invalid_output_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert invalid_result.returncode == 1
    invalid_report = json.loads(invalid_output_path.read_text(encoding="utf-8"))
    assert invalid_report["static"]["node_tree_manifest"]["complete"] is False
    assert invalid_report["delivery_acceptance_gate"]["acceptance_requirements"] == (
        _acceptance_requirements(static_node_tree_complete=True)
    )
    assert "static_node_tree_incomplete" in invalid_report[
        "delivery_acceptance_gate"
    ]["reason_codes"]
    assert invalid_report["delivery_acceptance_gate"]["reason_details"][-2] == {
        "code": "static_node_tree_incomplete",
        "count": 1,
        "message": "1 robot(s) have incomplete static Godot node-tree manifests",
        "inputs": [str(invalid_config)],
        "inputs_count": 1,
        "inputs_truncated": False,
    }
    assert "requirements=static_node_tree_complete" in invalid_result.stderr
    assert "static_node_tree_incomplete" in invalid_result.stderr


def test_delivery_acceptance_gate_validator_accepts_report_gate(
    tmp_path: Path,
) -> None:
    report_path = tmp_path / "report.json"
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--output",
            str(report_path),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [sys.executable, str(GATE_VALIDATOR_TOOL), str(gate_path)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    payload = json.loads(validate_result.stdout)
    assert payload == {
        "status": "success",
        "input": str(gate_path),
        "gate_source_path": ".",
        "contract_version": "delivery_acceptance_gate.v1",
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "acceptance_profile": "custom",
        "level": "static_only",
        "required": False,
        "complete": False,
        "requires_full_mechanical_restoration_gate": False,
        "passed": True,
        "exit_code": 0,
        "reason_codes": [
            "missing_godot_smoke",
            "static_only",
        ],
        "reasons_count": 2,
        "enabled_requirements": [],
        "affected_inputs": [str(FIXED_PAIR_FIXTURE)],
        "affected_inputs_count": 1,
        "affected_inputs_truncated": False,
        "summary_counts": {
            "inputs_count": 1,
            "success_count": 1,
            "error_count": 0,
            "live_smoke_count": 0,
            "smoke_report_written_count": 0,
            "smoke_report_missing_count": 0,
            "smoke_report_read_error_count": 0,
            "delivery_complete_count": 1,
            "delivery_dynamic_generation_count": 1,
            "delivery_godot_verified_count": 0,
            "delivery_incomplete_count": 0,
            "delivery_parameters_incomplete_count": 0,
            "delivery_static_only_count": 1,
            "delivery_unverified_count": 0,
            "fixed_lock_checked_count": 0,
            "fixed_lock_mismatch_count": 0,
            "control_configured_count": 0,
            "control_readback_checked_count": 0,
            "control_readback_missing_count": 0,
            "node_tree_fixed_lock_checked_count": 0,
            "node_tree_fixed_lock_mismatch_count": 0,
            "node_tree_fixed_locks_complete_count": 0,
            "node_tree_fixed_locks_incomplete_count": 0,
            "node_tree_gate_enabled_count": 0,
            "node_tree_full_restoration_required_count": 0,
            "node_tree_full_restoration_not_required_count": 0,
            "node_tree_gate_check_counts": {
                "class_mismatch": 0,
                "fixed_lock_mismatch": 0,
                "incomplete_node_tree": 0,
                "missing_parameters": 0,
                "physical_mismatch": 0,
                "transform_mismatch": 0,
            },
            "static_topology_complete_count": 1,
            "static_topology_incomplete_count": 0,
            "static_topology_disconnected_parts_count": 0,
            "static_topology_unreachable_parts_count": 0,
            "static_topology_duplicate_child_endpoint_count": 0,
            "static_topology_cycle_count": 0,
            "static_node_tree_manifest_count": 1,
            "static_node_tree_manifest_valid_count": 1,
            "static_node_tree_manifest_invalid_count": 0,
            "static_node_tree_manifest_error_count": 0,
            "static_node_tree_manifest_output_count": 0,
            "static_node_tree_manifest_path_map_mismatch_count": 0,
            "static_node_tree_parts_planned_count": 2,
            "static_node_tree_joints_planned_count": 1,
            "static_node_tree_parameterized_joints_count": 1,
            "static_node_tree_complete_count": 1,
            "static_node_tree_incomplete_count": 0,
            "static_node_tree_endpoint_paths_complete_count": 1,
            "static_node_tree_endpoint_paths_incomplete_count": 0,
            "static_node_tree_missing_endpoint_parts_count": 0,
            "static_node_tree_missing_endpoint_connections_count": 0,
            "static_node_tree_parameters_complete_count": 1,
            "static_node_tree_parameters_incomplete_count": 0,
            "mechanical_gate_enabled_count": 0,
            "full_mechanical_restoration_required_count": 0,
            "full_mechanical_restoration_not_required_count": 0,
            "mechanical_gate_check_counts": {
                "control_parameter_readback": 0,
                "full_node_tree_restoration": 0,
                "joint_parameter_readback": 0,
                "mechanical_restoration": 0,
            },
            "mechanical_behavior_evidence_count": 0,
            "mechanical_behavior_complete_count": 0,
            "mechanical_behavior_incomplete_count": 0,
            "mechanical_behavior_residual_risk_count": 0,
            "mechanical_behavior_threshold_failure_count": 0,
            "mechanical_behavior_center_of_mass_available_count": 0,
            "mechanical_behavior_contact_state_available_count": 0,
            "mechanical_behavior_step_trace_artifact_count": 0,
            "failure_reasons_count": 0,
        },
        "complete_required_summary_fields": [
            "control_readback_missing_count",
            "delivery_complete_count",
            "delivery_dynamic_generation_count",
            "delivery_godot_verified_count",
            "delivery_incomplete_count",
            "delivery_parameters_incomplete_count",
            "delivery_static_only_count",
            "delivery_unverified_count",
            "error_count",
            "failure_reasons_count",
            "fixed_lock_mismatch_count",
            "live_smoke_count",
            "node_tree_fixed_lock_mismatch_count",
            "node_tree_fixed_locks_incomplete_count",
            "smoke_report_missing_count",
            "smoke_report_read_error_count",
            "smoke_report_written_count",
            "static_node_tree_complete_count",
            "static_node_tree_endpoint_paths_complete_count",
            "static_node_tree_endpoint_paths_incomplete_count",
            "static_node_tree_incomplete_count",
            "static_node_tree_manifest_count",
            "static_node_tree_manifest_error_count",
            "static_node_tree_manifest_invalid_count",
            "static_node_tree_manifest_path_map_mismatch_count",
            "static_node_tree_manifest_valid_count",
            "static_node_tree_missing_endpoint_connections_count",
            "static_node_tree_missing_endpoint_parts_count",
            "static_node_tree_parameters_complete_count",
            "static_node_tree_parameters_incomplete_count",
            "static_topology_complete_count",
            "static_topology_cycle_count",
            "static_topology_disconnected_parts_count",
            "static_topology_duplicate_child_endpoint_count",
            "static_topology_incomplete_count",
            "static_topology_unreachable_parts_count",
            "success_count",
        ],
        "complete_required_summary_fields_count": 37,
        "require_passed": False,
        "require_required": False,
        "require_complete": False,
        "require_full_mechanical_restoration_gate": False,
        "errors": [],
    }


def test_delivery_acceptance_gate_validator_accepts_full_report(
    tmp_path: Path,
) -> None:
    report_path = tmp_path / "report.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--output",
            str(report_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [sys.executable, str(GATE_VALIDATOR_TOOL), str(report_path)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "success"
    assert payload["input"] == str(report_path)
    assert payload["gate_source_path"] == "delivery_acceptance_gate"
    assert payload["contract_version"] == "delivery_acceptance_gate.v1"
    assert payload["source"] == "dynamic_godot_report_cli"
    assert payload["passed"] is True
    assert payload["exit_code"] == 0
    assert payload["reason_codes"] == [
        "missing_godot_smoke",
        "static_only",
    ]
    assert payload["reasons_count"] == 2
    assert payload["enabled_requirements"] == []
    assert payload["affected_inputs"] == [str(FIXED_PAIR_FIXTURE)]
    assert payload["affected_inputs_count"] == 1
    assert payload["affected_inputs_truncated"] is False
    assert payload["summary_counts"]["inputs_count"] == 1
    assert payload["summary_counts"]["delivery_static_only_count"] == 1
    assert payload["require_passed"] is False
    assert payload["errors"] == []


def test_delivery_acceptance_gate_validator_can_require_passed_gate(
    tmp_path: Path,
) -> None:
    report_path = tmp_path / "report.json"
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--require-full-mechanical-restoration-gate",
            "--output",
            str(report_path),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 1

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--require-passed",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "error"
    assert payload["gate_source_path"] == "."
    assert payload["passed"] is False
    assert payload["exit_code"] == 1
    assert payload["reason_codes"] == [
        "missing_godot_smoke",
        "static_only",
        "missing_full_mechanical_restoration_gate",
    ]
    assert payload["reasons_count"] == 3
    assert payload["enabled_requirements"] == ["full_mechanical_restoration_gate"]
    assert payload["affected_inputs"] == [str(FIXED_PAIR_FIXTURE)]
    assert payload["affected_inputs_count"] == 1
    assert payload["affected_inputs_truncated"] is False
    assert payload["summary_counts"]["inputs_count"] == 1
    assert payload["summary_counts"]["full_mechanical_restoration_required_count"] == 0
    assert payload["require_passed"] is True
    assert payload["errors"] == ["delivery_acceptance_gate passed must be true"]


def test_delivery_acceptance_gate_validator_can_require_required_gate(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--require-required",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "error"
    assert payload["gate_source_path"] == "."
    assert payload["required"] is False
    assert payload["passed"] is True
    assert payload["require_passed"] is False
    assert payload["require_required"] is True
    assert payload["errors"] == ["delivery_acceptance_gate required must be true"]


def test_delivery_acceptance_gate_validator_can_require_complete_gate(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--require-complete",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "error"
    assert payload["gate_source_path"] == "."
    assert payload["complete"] is False
    assert payload["passed"] is True
    assert payload["require_passed"] is False
    assert payload["require_required"] is False
    assert payload["require_complete"] is True
    assert payload["errors"] == ["delivery_acceptance_gate complete must be true"]


def test_delivery_acceptance_gate_validator_can_require_full_mechanical_gate(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--require-full-mechanical-restoration-gate",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "error"
    assert payload["gate_source_path"] == "."
    assert payload["requires_full_mechanical_restoration_gate"] is False
    assert payload["passed"] is True
    assert payload["require_passed"] is False
    assert payload["require_required"] is False
    assert payload["require_complete"] is False
    assert payload["require_full_mechanical_restoration_gate"] is True
    assert payload["errors"] == [
        "delivery_acceptance_gate "
        "requires_full_mechanical_restoration_gate must be true"
    ]


def test_delivery_acceptance_gate_validator_can_emit_text_summary(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--require-full-mechanical-restoration-gate",
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 1

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--require-passed",
            "--format",
            "text",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    assert validate_result.stdout.strip() == (
        "delivery_acceptance_gate validation "
        "status=error "
        "source=dynamic_godot_report_cli "
        "scope=godot_smoke_motion "
        "profile=custom "
        "passed=false "
        "full_mechanical_gate=true "
        "exit_code=1 "
        "requirements=full_mechanical_restoration_gate "
        "complete_required_summary_fields=control_readback_missing_count+"
        "delivery_complete_count+delivery_dynamic_generation_count+"
        "delivery_godot_verified_count+delivery_incomplete_count+"
        "delivery_parameters_incomplete_count+delivery_static_only_count+"
        "delivery_unverified_count+error_count+failure_reasons_count+"
        "fixed_lock_mismatch_count+live_smoke_count+"
        "node_tree_fixed_lock_mismatch_count+"
        "node_tree_fixed_locks_incomplete_count+smoke_report_missing_count+"
        "smoke_report_read_error_count+smoke_report_written_count+"
        "static_node_tree_complete_count+"
        "static_node_tree_endpoint_paths_complete_count+"
        "static_node_tree_endpoint_paths_incomplete_count+"
        "static_node_tree_incomplete_count+"
        "static_node_tree_manifest_count+"
        "static_node_tree_manifest_error_count+"
        "static_node_tree_manifest_invalid_count+"
        "static_node_tree_manifest_path_map_mismatch_count+"
        "static_node_tree_manifest_valid_count+"
        "static_node_tree_missing_endpoint_connections_count+"
        "static_node_tree_missing_endpoint_parts_count+"
        "static_node_tree_parameters_complete_count+"
        "static_node_tree_parameters_incomplete_count+"
        "static_topology_complete_count+static_topology_cycle_count+"
        "static_topology_disconnected_parts_count+"
        "static_topology_duplicate_child_endpoint_count+"
        "static_topology_incomplete_count+"
        "static_topology_unreachable_parts_count+"
        "success_count "
        "complete_required_summary_fields_count=37 "
        "reason_codes=missing_godot_smoke,static_only,missing_full_mechanical_restoration_gate "
        "counts=inputs:1,errors:0,live:0,smokereports:0/0/0,"
        "control:0/0/0,verified:0,static:1,treegate:0/0,"
        "mechgate:0/0,failures:0 "
        "checks=none "
        f"affected_inputs={FIXED_PAIR_FIXTURE} "
        "errors=delivery_acceptance_gate passed must be true"
    )


def test_delivery_acceptance_gate_validator_writes_full_output_with_text_stdout(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    output_path = tmp_path / "artifacts" / "validation.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--output",
            str(output_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    assert validate_result.stdout.startswith("delivery_acceptance_gate validation ")
    output = json.loads(output_path.read_text(encoding="utf-8"))
    assert output["status"] == "success"
    assert output["input"] == str(gate_path)
    assert output["gate_source_path"] == "."
    assert output["errors"] == []


def test_delivery_acceptance_gate_validator_accepts_multiple_inputs(
    tmp_path: Path,
) -> None:
    report_path = tmp_path / "report.json"
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--output",
            str(report_path),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(report_path),
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "success"
    assert payload["inputs_count"] == 2
    assert payload["success_count"] == 2
    assert payload["error_count"] == 0
    assert payload["require_passed"] is False
    assert [item["gate_source_path"] for item in payload["results"]] == [
        "delivery_acceptance_gate",
        ".",
    ]
    assert [item["status"] for item in payload["results"]] == ["success", "success"]


def test_delivery_acceptance_gate_validator_accepts_directory_input(
    tmp_path: Path,
) -> None:
    artifacts_dir = tmp_path / "artifacts"
    artifacts_dir.mkdir()
    report_path = artifacts_dir / "report.json"
    gate_path = artifacts_dir / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--output",
            str(report_path),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [sys.executable, str(GATE_VALIDATOR_TOOL), str(artifacts_dir)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "success"
    assert payload["inputs_count"] == 2
    assert payload["success_count"] == 2
    assert [Path(item["input"]).name for item in payload["results"]] == [
        "gate.json",
        "report.json",
    ]


def test_delivery_acceptance_gate_validator_skips_node_tree_manifest_sidecar(
    tmp_path: Path,
) -> None:
    artifacts_dir = tmp_path / "artifacts"
    artifacts_dir.mkdir()
    report_path = artifacts_dir / "report.json"
    gate_path = artifacts_dir / "gate.json"
    manifest_path = (
        artifacts_dir / "000_robot_dynamic_fixed_pair.node_tree_manifest.json"
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--output",
            str(report_path),
            "--gate-output",
            str(gate_path),
            "--static-node-tree-manifest-dir",
            str(artifacts_dir),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr
    assert manifest_path.exists()
    manifest_payload = json.loads(manifest_path.read_text(encoding="utf-8"))

    validate_result = subprocess.run(
        [sys.executable, str(GATE_VALIDATOR_TOOL), str(artifacts_dir)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "success"
    assert payload["inputs_count"] == 3
    assert payload["success_count"] == 2
    assert payload["skipped_count"] == 1
    assert payload["node_tree_manifest_sidecar_count"] == 1
    assert payload["node_tree_manifest_sidecar_complete_count"] == 1
    assert payload["node_tree_manifest_sidecar_incomplete_count"] == 0
    assert payload["node_tree_manifest_sidecar_valid_count"] == 1
    assert payload["node_tree_manifest_sidecar_invalid_count"] == 0
    assert payload["node_tree_manifest_sidecar_validation_error_count"] == 0
    assert payload["node_tree_manifest_sidecar_path_incomplete_count"] == 0
    assert payload["node_tree_manifest_sidecar_path_map_mismatch_count"] == 0
    assert payload["node_tree_manifest_sidecar_path_map_mismatch_kind_counts"] == {}
    assert payload["node_tree_manifest_sidecar_parts_planned_count"] == 2
    assert payload["node_tree_manifest_sidecar_joints_planned_count"] == 1
    assert payload["node_tree_manifest_sidecar_part_path_count"] == 2
    assert payload["node_tree_manifest_sidecar_joint_path_count"] == 1
    assert payload["node_tree_manifest_sidecars"] == [
        {
            "input": str(manifest_path),
            "manifest_version": "godot_node_tree_manifest.v1",
            "schema_version": manifest_payload["schema_version"],
            "robot_name": manifest_payload["robot_name"],
            "parts_count": 2,
            "joints_count": 1,
            "part_node_path_count": 2,
            "joint_node_path_count": 1,
            "path_maps_complete": True,
            "parameterized_joints": 1,
            "complete": True,
            "endpoint_paths_complete": True,
            "parameters_complete": True,
            "missing_endpoint_parts_count": 0,
            "missing_endpoint_connections_count": 0,
            "node_tree_manifest_valid": True,
            "node_tree_manifest_validation_error_count": 0,
            "node_tree_manifest_validation_errors": [],
            "node_tree_manifest_path_map_mismatch_count": 0,
            "node_tree_manifest_path_map_mismatch_kind_counts": {},
            "node_tree_manifest_path_map_mismatches": [],
        }
    ]
    assert payload["node_tree_manifest_sidecars_truncated"] is False
    assert payload["error_count"] == 0
    assert [Path(item["input"]).name for item in payload["results"]] == [
        "000_robot_dynamic_fixed_pair.node_tree_manifest.json",
        "gate.json",
        "report.json",
    ]
    skipped = payload["results"][0]
    assert skipped["status"] == "skipped"
    assert skipped["skip_reason"] == "static Godot node-tree manifest sidecar"
    assert skipped["node_tree_manifest_summary"] == {
        "manifest_version": "godot_node_tree_manifest.v1",
        "schema_version": manifest_payload["schema_version"],
        "robot_name": manifest_payload["robot_name"],
        "parts_count": 2,
        "joints_count": 1,
        "part_node_path_count": 2,
        "joint_node_path_count": 1,
        "path_maps_complete": True,
        "parameterized_joints": 1,
        "complete": True,
        "endpoint_paths_complete": True,
        "parameters_complete": True,
        "missing_endpoint_parts_count": 0,
        "missing_endpoint_connections_count": 0,
    }
    assert skipped["node_tree_manifest_valid"] is True
    assert skipped["node_tree_manifest_validation_errors"] == []
    assert skipped["node_tree_manifest_path_map_mismatch_count"] == 0
    assert skipped["node_tree_manifest_path_map_mismatch_kind_counts"] == {}
    assert skipped["node_tree_manifest_path_map_mismatches"] == []

    constrained_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--expect-skipped-count",
            "1",
            "--expect-skipped-input",
            str(manifest_path),
            "--expect-skipped-reason",
            "static Godot node-tree manifest sidecar",
            "--expect-node-tree-manifest-sidecar-count",
            "1",
            "--expect-node-tree-manifest-sidecar-complete-count",
            "1",
            "--expect-node-tree-manifest-sidecar-incomplete-count",
            "0",
            "--expect-node-tree-manifest-sidecar-valid-count",
            "1",
            "--expect-node-tree-manifest-sidecar-invalid-count",
            "0",
            "--expect-node-tree-manifest-sidecar-validation-error-count",
            "0",
            "--expect-node-tree-manifest-sidecar-path-incomplete-count",
            "0",
            "--expect-node-tree-manifest-sidecar-path-map-mismatch-count",
            "0",
            "--expect-node-tree-manifest-sidecar-path-map-mismatch-kind",
            "root_mismatch=0",
            "--expect-node-tree-manifest-sidecar-parts-planned-count",
            "2",
            "--expect-node-tree-manifest-sidecar-joints-planned-count",
            "1",
            "--expect-node-tree-manifest-sidecar-part-path-count",
            "2",
            "--expect-node-tree-manifest-sidecar-joint-path-count",
            "1",
            "--show-skipped-reasons",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert constrained_result.returncode == 0
    constrained_payload = json.loads(constrained_result.stdout)
    assert constrained_payload["expected_skipped_count"] == 1
    assert constrained_payload["skipped_count"] == 1
    assert constrained_payload[
        "expected_node_tree_manifest_sidecar_path_map_mismatch_count"
    ] == 0
    assert constrained_payload[
        "expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts"
    ] == {"root_mismatch": 0}
    assert constrained_payload["expected_node_tree_manifest_sidecar_count"] == 1
    assert constrained_payload[
        "expected_node_tree_manifest_sidecar_complete_count"
    ] == 1
    assert constrained_payload[
        "expected_node_tree_manifest_sidecar_incomplete_count"
    ] == 0
    assert constrained_payload["expected_node_tree_manifest_sidecar_valid_count"] == 1
    assert constrained_payload["expected_node_tree_manifest_sidecar_invalid_count"] == 0
    assert (
        constrained_payload[
            "expected_node_tree_manifest_sidecar_validation_error_count"
        ]
        == 0
    )
    assert constrained_payload[
        "expected_node_tree_manifest_sidecar_path_incomplete_count"
    ] == 0
    assert constrained_payload[
        "expected_node_tree_manifest_sidecar_parts_planned_count"
    ] == 2
    assert constrained_payload[
        "expected_node_tree_manifest_sidecar_joints_planned_count"
    ] == 1
    assert constrained_payload["expected_node_tree_manifest_sidecar_part_path_count"] == 2
    assert constrained_payload["expected_node_tree_manifest_sidecar_joint_path_count"] == 1
    assert constrained_payload[
        "node_tree_manifest_sidecar_path_map_mismatch_count"
    ] == 0
    assert constrained_payload["expected_skipped_inputs"] == [str(manifest_path)]
    assert constrained_payload["missing_expected_skipped_inputs"] == []
    assert constrained_payload["expected_skipped_reasons"] == [
        "static Godot node-tree manifest sidecar"
    ]
    assert constrained_payload["missing_expected_skipped_reasons"] == []

    text_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--format",
            "text",
            "--show-diagnostics",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert text_result.returncode == 0
    assert (
        "node_tree_sidecars=count:1,complete:1,incomplete:0,valid:1,invalid:0,validation_errors:0,path_incomplete:0,path_mismatches:0,path_mismatch_kinds:none,parts:2,joints:1,part_paths:2,joint_paths:1"
        in text_result.stdout.splitlines()[0]
    )

    complete_gate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--fail-on-node-tree-manifest-sidecar-incomplete",
            "--format",
            "text",
            "--show-diagnostics",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert complete_gate_result.returncode == 0
    complete_gate_line = complete_gate_result.stdout.splitlines()[0]
    assert (
        "node_tree_sidecars=count:1,complete:1,incomplete:0,valid:1,invalid:0,validation_errors:0,path_incomplete:0,path_mismatches:0,path_mismatch_kinds:none,parts:2,joints:1,part_paths:2,joint_paths:1"
        in complete_gate_line
    )
    assert (
        "fail_on_node_tree_manifest_sidecar_incomplete=true"
        in complete_gate_line
    )

    path_incomplete_payload = json.loads(json.dumps(manifest_payload))
    path_incomplete_payload["part_node_paths"].pop("payload")
    path_incomplete_payload["path_maps_complete"] = False
    manifest_path.write_text(
        json.dumps(path_incomplete_payload, ensure_ascii=False), encoding="utf-8"
    )
    path_incomplete_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--fail-on-node-tree-manifest-sidecar-path-incomplete",
            "--format",
            "text",
            "--show-diagnostics",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert path_incomplete_result.returncode == 1
    path_incomplete_line = path_incomplete_result.stdout.splitlines()[0]
    assert (
        "node_tree_sidecars=count:1,complete:1,incomplete:0,valid:0,invalid:1,validation_errors:2,path_incomplete:1,path_mismatches:1,path_mismatch_kinds:missing:1,parts:2,joints:1,part_paths:1,joint_paths:1"
        in path_incomplete_line
    )
    assert (
        "aggregate_errors=node_tree_manifest_sidecar_path_incomplete_count present: 1"
        in path_incomplete_line
    )
    assert (
        "fail_on_node_tree_manifest_sidecar_path_incomplete=true"
        in path_incomplete_line
    )

    path_mismatch_payload = json.loads(json.dumps(manifest_payload))
    path_mismatch_payload["part_node_paths"]["payload"]["body_node"] = "wrong/path"
    path_mismatch_payload["path_maps_complete"] = True
    manifest_path.write_text(
        json.dumps(path_mismatch_payload, ensure_ascii=False), encoding="utf-8"
    )
    path_mismatch_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--fail-on-invalid-node-tree-manifest-sidecar",
            "--format",
            "text",
            "--show-diagnostics",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert path_mismatch_result.returncode == 1
    path_mismatch_line = path_mismatch_result.stdout.splitlines()[0]
    assert (
        "node_tree_sidecars=count:1,complete:1,incomplete:0,valid:0,invalid:1,validation_errors:3,path_incomplete:0,path_mismatches:1,path_mismatch_kinds:value_mismatch:1,parts:2,joints:1,part_paths:2,joint_paths:1"
        in path_mismatch_line
    )
    assert (
        "aggregate_errors=node_tree_manifest_sidecar_invalid_count present: 1"
        in path_mismatch_line
    )
    path_mismatch_json_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--fail-on-invalid-node-tree-manifest-sidecar",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert path_mismatch_json_result.returncode == 1
    path_mismatch_payload = json.loads(path_mismatch_json_result.stdout)
    path_mismatch_skipped = path_mismatch_payload["results"][0]
    assert path_mismatch_payload[
        "node_tree_manifest_sidecar_path_map_mismatch_count"
    ] == 1
    assert path_mismatch_payload[
        "node_tree_manifest_sidecar_path_map_mismatch_kind_counts"
    ] == {"value_mismatch": 1}
    assert path_mismatch_skipped["node_tree_manifest_path_map_mismatch_count"] == 1
    assert path_mismatch_skipped[
        "node_tree_manifest_path_map_mismatch_kind_counts"
    ] == {"value_mismatch": 1}
    assert path_mismatch_skipped["node_tree_manifest_path_map_mismatches"] == [
        {
            "map": "part_node_paths",
            "key": "payload",
            "field": "body_node",
            "kind": "value_mismatch",
            "expected": "dynamic_fixed_pair/payload",
            "actual": "wrong/path",
        }
    ]
    assert path_mismatch_payload["node_tree_manifest_sidecars"][0][
        "node_tree_manifest_path_map_mismatch_count"
    ] == 1
    assert path_mismatch_payload["node_tree_manifest_sidecars"][0][
        "node_tree_manifest_path_map_mismatch_kind_counts"
    ] == {"value_mismatch": 1}
    assert path_mismatch_payload["node_tree_manifest_sidecars"][0][
        "node_tree_manifest_path_map_mismatches"
    ] == path_mismatch_skipped["node_tree_manifest_path_map_mismatches"]

    path_map_mismatch_gate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--fail-on-node-tree-manifest-sidecar-path-map-mismatch",
            "--format",
            "text",
            "--show-diagnostics",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert path_map_mismatch_gate_result.returncode == 1
    path_map_mismatch_gate_line = path_map_mismatch_gate_result.stdout.splitlines()[0]
    assert (
        "aggregate_errors=node_tree_manifest_sidecar_path_map_mismatch_count present: 1"
        in path_map_mismatch_gate_line
    )
    assert (
        "fail_on_node_tree_manifest_sidecar_path_map_mismatch=true"
        in path_map_mismatch_gate_line
    )

    path_map_mismatch_expect_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--expect-node-tree-manifest-sidecar-valid-count",
            "1",
            "--expect-node-tree-manifest-sidecar-invalid-count",
            "0",
            "--expect-node-tree-manifest-sidecar-validation-error-count",
            "0",
            "--expect-node-tree-manifest-sidecar-path-map-mismatch-count",
            "0",
            "--expect-node-tree-manifest-sidecar-path-map-mismatch-kind",
            "value_mismatch=0",
            "--expect-node-tree-manifest-sidecar-parts-planned-count",
            "3",
            "--expect-node-tree-manifest-sidecar-part-path-count",
            "3",
            "--format",
            "text",
            "--show-diagnostics",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert path_map_mismatch_expect_result.returncode == 1
    path_map_mismatch_expect_line = (
        path_map_mismatch_expect_result.stdout.splitlines()[0]
    )
    assert (
        "expected_node_tree_manifest_sidecar_path_map_mismatch_count=0"
        in path_map_mismatch_expect_line
    )
    assert (
        "expected_path_mismatch_kinds=value_mismatch:0"
        in path_map_mismatch_expect_line
    )
    assert (
        "expected_node_tree_manifest_sidecar_valid_count=1"
        in path_map_mismatch_expect_line
    )
    assert (
        "expected_node_tree_manifest_sidecar_validation_error_count=0"
        in path_map_mismatch_expect_line
    )
    assert (
        "expected_node_tree_manifest_sidecar_parts_planned_count=3"
        in path_map_mismatch_expect_line
    )
    for expected_error in [
        "expected node_tree_manifest_sidecar_valid_count 1 but found 0",
        "expected node_tree_manifest_sidecar_invalid_count 0 but found 1",
        "expected node_tree_manifest_sidecar_validation_error_count 0 but found 3",
        "expected node_tree_manifest_sidecar_path_map_mismatch_count 0 but found 1",
        (
            "expected "
            "node_tree_manifest_sidecar_path_map_mismatch_kind_counts.value_mismatch "
            "0 but found 1"
        ),
        "expected node_tree_manifest_sidecar_parts_planned_count 3 but found 2",
        "expected node_tree_manifest_sidecar_part_path_count 3 but found 2",
    ]:
        assert expected_error in path_map_mismatch_expect_line

    root_mismatch_payload = json.loads(json.dumps(manifest_payload))
    root_mismatch_payload["part_nodes"][0]["body_node"] = "wrong_robot/base"
    root_mismatch_payload["part_nodes"][0][
        "collision_node"
    ] = "wrong_robot/base/Collision"
    root_mismatch_payload["part_nodes"][0]["mesh_node"] = "wrong_robot/base/Mesh"
    root_mismatch_payload["joint_nodes"][0][
        "joint_node"
    ] = "wrong_robot/base_payload_fixed"
    root_mismatch_payload["joint_nodes"][0]["node_a"] = "wrong_robot/base"
    root_mismatch_payload["joint_nodes"][0]["node_b"] = "wrong_robot/payload"
    root_mismatch_payload["part_node_paths"]["base"] = {
        "body_node": "wrong_robot/base",
        "collision_node": "wrong_robot/base/Collision",
        "mesh_node": "wrong_robot/base/Mesh",
    }
    root_mismatch_payload["joint_node_paths"]["base_payload_fixed"] = {
        "joint_node": "wrong_robot/base_payload_fixed",
        "node_a": "wrong_robot/base",
        "node_b": "wrong_robot/payload",
    }
    root_mismatch_payload["path_maps_complete"] = True
    manifest_path.write_text(
        json.dumps(root_mismatch_payload, ensure_ascii=False), encoding="utf-8"
    )
    root_mismatch_result = subprocess.run(
        [sys.executable, str(GATE_VALIDATOR_TOOL), str(artifacts_dir)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert root_mismatch_result.returncode == 0
    root_mismatch_gate = json.loads(root_mismatch_result.stdout)
    root_mismatch_skipped = root_mismatch_gate["results"][0]
    assert root_mismatch_gate["node_tree_manifest_sidecar_invalid_count"] == 1
    assert root_mismatch_gate["node_tree_manifest_sidecar_validation_error_count"] == 6
    assert root_mismatch_gate["node_tree_manifest_sidecar_path_incomplete_count"] == 0
    assert root_mismatch_gate["node_tree_manifest_sidecar_path_map_mismatch_count"] == 6
    assert root_mismatch_gate[
        "node_tree_manifest_sidecar_path_map_mismatch_kind_counts"
    ] == {"root_mismatch": 6}
    assert root_mismatch_skipped["node_tree_manifest_path_map_mismatch_count"] == 6
    assert root_mismatch_skipped[
        "node_tree_manifest_path_map_mismatch_kind_counts"
    ] == {"root_mismatch": 6}
    assert root_mismatch_skipped["node_tree_manifest_path_map_mismatches"][0] == {
        "map": "part_nodes",
        "key": "base",
        "field": "body_node",
        "kind": "root_mismatch",
        "expected": "dynamic_fixed_pair/base",
        "actual": "wrong_robot/base",
    }

    manifest_path.write_text(
        json.dumps(manifest_payload, ensure_ascii=False), encoding="utf-8"
    )
    manifest_payload["complete"] = False
    manifest_path.write_text(
        json.dumps(manifest_payload, ensure_ascii=False), encoding="utf-8"
    )
    incomplete_gate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--fail-on-node-tree-manifest-sidecar-incomplete",
            "--format",
            "text",
            "--show-diagnostics",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert incomplete_gate_result.returncode == 1
    incomplete_gate_line = incomplete_gate_result.stdout.splitlines()[0]
    assert (
        "node_tree_sidecars=count:1,complete:0,incomplete:1,valid:0,invalid:1,validation_errors:1,path_incomplete:0,path_mismatches:0,path_mismatch_kinds:none,parts:2,joints:1,part_paths:2,joint_paths:1"
        in incomplete_gate_line
    )
    assert (
        "aggregate_errors=node_tree_manifest_sidecar_incomplete_count present: 1"
        in incomplete_gate_line
    )
    assert (
        "fail_on_node_tree_manifest_sidecar_incomplete=true"
        in incomplete_gate_line
    )

    invalid_sidecar_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--fail-on-invalid-node-tree-manifest-sidecar",
            "--format",
            "text",
            "--show-diagnostics",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert invalid_sidecar_result.returncode == 1
    invalid_sidecar_line = invalid_sidecar_result.stdout.splitlines()[0]
    assert (
        "aggregate_errors=node_tree_manifest_sidecar_invalid_count present: 1"
        in invalid_sidecar_line
    )
    assert "fail_on_invalid_node_tree_manifest_sidecar=true" in invalid_sidecar_line

    validation_error_gate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--fail-on-node-tree-manifest-sidecar-validation-error",
            "--format",
            "text",
            "--show-diagnostics",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validation_error_gate_result.returncode == 1
    validation_error_gate_line = validation_error_gate_result.stdout.splitlines()[0]
    assert (
        "aggregate_errors=node_tree_manifest_sidecar_validation_error_count present: 1"
        in validation_error_gate_line
    )
    assert (
        "fail_on_node_tree_manifest_sidecar_validation_error=true"
        in validation_error_gate_line
    )


def test_delivery_acceptance_gate_validator_reports_empty_directory(
    tmp_path: Path,
) -> None:
    artifacts_dir = tmp_path / "artifacts"
    artifacts_dir.mkdir()

    validate_result = subprocess.run(
        [sys.executable, str(GATE_VALIDATOR_TOOL), str(artifacts_dir)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    assert "Traceback" not in validate_result.stderr
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "error"
    assert payload["inputs_count"] == 0
    assert payload["success_count"] == 0
    assert payload["skipped_count"] == 0
    assert payload["error_count"] == 0
    assert payload["errors"] == ["no JSON files found in input paths"]
    assert payload["results"] == []


def test_delivery_acceptance_gate_validator_writes_summary_for_empty_directory(
    tmp_path: Path,
) -> None:
    artifacts_dir = tmp_path / "artifacts"
    summary_path = tmp_path / "ci" / "gate_summary.json"
    artifacts_dir.mkdir()

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["summary_version"] == (
        workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
    )
    assert set(
        workflow_contracts.DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "validation_summary_required_fields"
        ]
    ).issubset(summary)
    assert workflow_contracts.validate_delivery_acceptance_validation_summary(summary) == []
    assert summary["status"] == "error"
    assert summary["inputs_count"] == 0
    assert summary["success_count"] == 0
    assert summary["skipped_count"] == 0
    assert summary["error_count"] == 0
    assert summary["errors"] == ["no JSON files found in input paths"]
    assert summary["failed_inputs"] == []
    assert summary["skipped_inputs"] == []


def test_delivery_acceptance_gate_validator_directory_can_ignore_non_gate_json(
    tmp_path: Path,
) -> None:
    artifacts_dir = tmp_path / "artifacts"
    artifacts_dir.mkdir()
    report_path = artifacts_dir / "report.json"
    gate_path = artifacts_dir / "gate.json"
    metadata_path = artifacts_dir / "metadata.json"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--output",
            str(report_path),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--ignore-non-gate",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "success"
    assert payload["inputs_count"] == 3
    assert payload["success_count"] == 2
    assert payload["skipped_count"] == 1
    assert payload["error_count"] == 0
    assert [Path(item["input"]).name for item in payload["results"]] == [
        "gate.json",
        "metadata.json",
        "report.json",
    ]
    skipped = payload["results"][1]
    assert skipped["status"] == "skipped"
    assert skipped["gate_source_path"] is None
    assert skipped["skip_reason"] == "not a delivery_acceptance_gate artifact or report"


def test_delivery_acceptance_gate_validator_writes_summary_output(
    tmp_path: Path,
) -> None:
    artifacts_dir = tmp_path / "artifacts"
    artifacts_dir.mkdir()
    report_path = artifacts_dir / "report.json"
    gate_path = artifacts_dir / "gate.json"
    metadata_path = artifacts_dir / "metadata.json"
    summary_path = tmp_path / "ci" / "gate_summary.json"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--output",
            str(report_path),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--ignore-non-gate",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert (
        workflow_contracts.validate_delivery_acceptance_validation_summary(summary)
        == []
    )
    assert summary["summary_version"] == (
        "delivery_acceptance_gate_validation_summary.v1"
    )
    assert summary["status"] == "success"
    assert summary["inputs_count"] == 3
    assert summary["success_count"] == 2
    assert summary["skipped_count"] == 1
    assert summary["error_count"] == 0
    assert summary["errors"] == []
    assert summary["failed_inputs"] == []
    assert summary["failed_inputs_count"] == 0
    assert summary["skipped_inputs"] == [str(metadata_path)]
    assert summary["skipped_inputs_count"] == 1
    assert summary["skipped_reasons"] == [
        "not a delivery_acceptance_gate artifact or report"
    ]
    assert summary["skipped_reasons_count"] == 1
    assert summary["affected_inputs"] == [str(FIXED_PAIR_FIXTURE)]
    assert summary["affected_inputs_count"] == 1


def test_delivery_acceptance_gate_validator_can_validate_summary_artifact(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "artifacts" / "gate.json"
    summary_path = tmp_path / "ci" / "gate_summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    summary_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert summary_result.returncode == 0, summary_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(summary_path),
            "--validate-validation-summary",
            "--expect-summary-version",
            workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION,
            "--expect-validation-summary-status",
            "success",
            "--expect-summary-versions-count",
            "1",
            "--expect-validation-summary-statuses-count",
            "1",
            "--allow-summary-version",
            workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION,
            "--allow-validation-summary-status",
            "success",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "success"
    assert payload["gate_source_path"] == "validation_summary"
    assert payload["summary_version"] == (
        workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
    )
    assert payload["validation_summary_status"] == "success"
    assert payload["validation_summary_errors_count"] == 0
    assert payload["errors"] == []

    summary_validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(summary_path),
            "--validate-validation-summary",
            "--summary-only",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert summary_validate_result.returncode == 0
    summary_payload = json.loads(summary_validate_result.stdout)
    assert summary_payload["summary_versions"] == [
        workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
    ]
    assert summary_payload["summary_versions_count"] == 1
    assert summary_payload["summary_versions_truncated"] is False
    assert summary_payload["validation_summary_statuses"] == ["success"]
    assert summary_payload["validation_summary_statuses_count"] == 1
    assert summary_payload["validation_summary_statuses_truncated"] is False


def test_delivery_acceptance_gate_validator_can_constrain_summary_artifact_metadata(
    tmp_path: Path,
) -> None:
    summary_path = tmp_path / "gate_summary.json"
    summary_path.write_text(
        json.dumps(
            {
                "summary_version": (
                    workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
                ),
                "status": "success",
                "expanded_inputs_count": 1,
                "inputs_count": 1,
                "success_count": 1,
                "skipped_count": 0,
                "error_count": 0,
                "errors": [],
            }
        ),
        encoding="utf-8",
    )

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(summary_path),
            "--validate-validation-summary",
            "--summary-only",
            "--expect-summary-version",
            "delivery_acceptance_gate_validation_summary.v2",
            "--expect-validation-summary-status",
            "error",
            "--expect-summary-versions-count",
            "2",
            "--expect-validation-summary-statuses-count",
            "2",
            "--allow-summary-version",
            "delivery_acceptance_gate_validation_summary.v2",
            "--allow-validation-summary-status",
            "error",
            "--forbid-summary-version",
            workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION,
            "--forbid-validation-summary-status",
            "success",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    assert payload["summary_versions"] == [
        workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
    ]
    assert payload["validation_summary_statuses"] == ["success"]
    assert payload["expected_summary_versions"] == [
        "delivery_acceptance_gate_validation_summary.v2"
    ]
    assert payload["missing_expected_summary_versions"] == [
        "delivery_acceptance_gate_validation_summary.v2"
    ]
    assert payload["expected_validation_summary_statuses"] == ["error"]
    assert payload["missing_expected_validation_summary_statuses"] == ["error"]
    assert payload["allowed_summary_versions"] == [
        "delivery_acceptance_gate_validation_summary.v2"
    ]
    assert payload["unexpected_summary_versions"] == [
        workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
    ]
    assert payload["allowed_validation_summary_statuses"] == ["error"]
    assert payload["unexpected_validation_summary_statuses"] == ["success"]
    assert payload["forbidden_summary_versions"] == [
        workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
    ]
    assert payload["present_forbidden_summary_versions"] == [
        workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
    ]
    assert payload["forbidden_validation_summary_statuses"] == ["success"]
    assert payload["present_forbidden_validation_summary_statuses"] == ["success"]
    assert (
        "expected summary_versions_count 2 but found 1" in payload["errors"]
    )
    assert (
        "expected validation_summary_statuses_count 2 but found 1"
        in payload["errors"]
    )
    assert any(
        "expected summary_version delivery_acceptance_gate_validation_summary.v2 "
        "was not found" in error
        for error in payload["errors"]
    )
    assert any(
        "expected validation_summary_status error was not found" in error
        for error in payload["errors"]
    )
    assert any(
        "unexpected summary_version "
        "delivery_acceptance_gate_validation_summary.v1 was found" in error
        for error in payload["errors"]
    )
    assert any(
        "unexpected validation_summary_status success was found" in error
        for error in payload["errors"]
    )
    assert any(
        "forbidden summary_version "
        "delivery_acceptance_gate_validation_summary.v1 was found" in error
        for error in payload["errors"]
    )
    assert any(
        "forbidden validation_summary_status success was found" in error
        for error in payload["errors"]
    )


def test_delivery_acceptance_gate_validator_rejects_invalid_summary_artifact(
    tmp_path: Path,
) -> None:
    summary_path = tmp_path / "gate_summary.json"
    summary_path.write_text(
        json.dumps(
            {
                "summary_version": "old",
                "status": "partial",
                "expanded_inputs_count": 0,
                "inputs_count": 1,
                "success_count": 0,
                "skipped_count": 0,
                "error_count": 0,
                "errors": ["bad", "bad", ""],
                "summary_versions": [
                    workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
                ],
                "summary_versions_count": 1,
                "summary_versions_truncated": False,
                "expected_summary_versions": ["future"],
                "missing_expected_summary_versions": [
                    workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
                ],
                "allowed_summary_versions": [
                    workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
                ],
                "unexpected_summary_versions": [
                    workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
                ],
                "forbidden_summary_versions": ["old"],
                "present_forbidden_summary_versions": ["future"],
                "expected_summary_versions_count": "1",
            }
        ),
        encoding="utf-8",
    )

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(summary_path),
            "--validate-validation-summary",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "error"
    result = payload["results"][0] if "results" in payload else payload
    assert result["summary_version"] == "old"
    assert any("summary_version must be" in error for error in result["errors"])
    assert any("status must be one of" in error for error in result["errors"])
    assert "expanded_inputs_count must be >= inputs_count" in result["errors"]
    assert (
        "success_count + skipped_count + error_count must equal inputs_count"
        in result["errors"]
    )
    assert any(
        "errors entries must be non-empty strings" in error
        for error in result["errors"]
    )
    assert any("errors entries must be unique" in error for error in result["errors"])
    assert any(
        "missing_expected_summary_versions must be a subset of "
        "expected_summary_versions" in error
        for error in result["errors"]
    )
    assert any(
        "unexpected_summary_versions must not include values present in "
        "allowed_summary_versions" in error
        for error in result["errors"]
    )
    assert any(
        "present_forbidden_summary_versions must be a subset of "
        "forbidden_summary_versions" in error
        for error in result["errors"]
    )
    assert any(
        "expected_summary_versions_count must be a non-negative integer or null"
        in error
        for error in result["errors"]
    )


def test_delivery_acceptance_gate_validator_can_fail_on_skipped_inputs(
    tmp_path: Path,
) -> None:
    artifacts_dir = tmp_path / "artifacts"
    artifacts_dir.mkdir()
    gate_path = artifacts_dir / "gate.json"
    metadata_path = artifacts_dir / "metadata.json"
    summary_path = tmp_path / "ci" / "summary.json"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--ignore-non-gate",
            "--fail-on-skipped",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["status"] == "error"
    assert payload["success_count"] == 1
    assert payload["skipped_count"] == 1
    assert payload["error_count"] == 0
    assert payload["errors"] == ["skipped inputs present: 1"]
    assert [item["status"] for item in payload["results"]] == ["success", "skipped"]
    assert summary["status"] == "error"
    assert summary["success_count"] == 1
    assert summary["skipped_count"] == 1
    assert summary["errors"] == ["skipped inputs present: 1"]
    assert summary["skipped_inputs"] == [str(metadata_path)]
    assert summary["skipped_reasons"] == [
        "not a delivery_acceptance_gate artifact or report"
    ]


def test_delivery_acceptance_gate_validator_can_expect_counts(
    tmp_path: Path,
) -> None:
    artifacts_dir = tmp_path / "artifacts"
    artifacts_dir.mkdir()
    report_path = artifacts_dir / "report.json"
    gate_path = artifacts_dir / "gate.json"
    metadata_path = artifacts_dir / "metadata.json"
    summary_path = tmp_path / "ci" / "summary.json"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--output",
            str(report_path),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--ignore-non-gate",
            "--expect-expanded-inputs-count",
            "4",
            "--expect-inputs-count",
            "4",
            "--expect-success-count",
            "3",
            "--expect-error-count",
            "1",
            "--expect-skipped-count",
            "0",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["status"] == "error"
    assert payload["inputs_count"] == 3
    assert payload["success_count"] == 2
    assert payload["skipped_count"] == 1
    assert payload["expected_expanded_inputs_count"] == 4
    assert payload["expected_inputs_count"] == 4
    assert payload["expected_success_count"] == 3
    assert payload["expected_error_count"] == 1
    assert payload["expected_skipped_count"] == 0
    assert payload["errors"] == [
        "expected expanded_inputs_count 4 but found 3",
        "expected inputs_count 4 but found 3",
        "expected success_count 3 but found 2",
        "expected error_count 1 but found 0",
        "expected skipped_count 0 but found 1",
    ]
    assert summary["status"] == "error"
    assert summary["inputs_count"] == 3
    assert summary["success_count"] == 2
    assert summary["expected_expanded_inputs_count"] == 4
    assert summary["expected_inputs_count"] == 4
    assert summary["expected_success_count"] == 3
    assert summary["expected_error_count"] == 1
    assert summary["expected_skipped_count"] == 0
    assert summary["errors"] == payload["errors"]


def test_delivery_acceptance_gate_validator_outputs_aggregate_for_matching_expectations(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-inputs-count",
            "1",
            "--expect-success-count",
            "1",
            "--expect-error-count",
            "0",
            "--expect-skipped-count",
            "0",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "success"
    assert payload["inputs_count"] == 1
    assert payload["success_count"] == 1
    assert payload["error_count"] == 0
    assert payload["skipped_count"] == 0
    assert payload["expected_inputs_count"] == 1
    assert payload["expected_success_count"] == 1
    assert payload["expected_error_count"] == 0
    assert payload["expected_skipped_count"] == 0
    assert payload["errors"] == []
    assert payload["results"][0]["status"] == "success"


def test_delivery_acceptance_gate_validator_text_outputs_summary_for_matching_expectations(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--expect-inputs-count",
            "1",
            "--expect-success-count",
            "1",
            "--expect-error-count",
            "0",
            "--expect-skipped-count",
            "0",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    lines = validate_result.stdout.splitlines()
    assert len(lines) == 2
    assert lines[0] == (
        "delivery_acceptance_gate validation summary "
        "status=success inputs=1 success=1 errors=0 "
        "expected_inputs_count=1 expected_success_count=1 "
        "expected_error_count=0 expected_skipped_count=0 "
        "require_passed=false"
    )
    assert "status=success" in lines[1]
    assert "full_mechanical_gate=false" in lines[1]
    assert f"affected_inputs={FIXED_PAIR_FIXTURE}" in lines[1]


def test_delivery_acceptance_gate_validator_can_expect_passed_state_counts(
    tmp_path: Path,
) -> None:
    passing_gate_path = tmp_path / "passing_gate.json"
    failing_gate_path = tmp_path / "failing_gate.json"
    summary_path = tmp_path / "summary.json"

    passing_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(passing_gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert passing_result.returncode == 0, passing_result.stderr

    failing_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--require-full-mechanical-restoration-gate",
            "--gate-output",
            str(failing_gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert failing_result.returncode == 1

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(passing_gate_path),
            str(failing_gate_path),
            "--expect-passed-true-count",
            "1",
            "--expect-passed-false-count",
            "1",
            "--expect-passed-unknown-count",
            "0",
            "--expect-full-mechanical-gate-true-count",
            "1",
            "--expect-full-mechanical-gate-false-count",
            "1",
            "--expect-full-mechanical-gate-unknown-count",
            "0",
            "--expect-full-mechanical-gate-true-input",
            str(failing_gate_path),
            "--expect-full-mechanical-gate-false-input",
            str(passing_gate_path),
            "--allow-full-mechanical-gate-true-input",
            str(failing_gate_path),
            "--allow-full-mechanical-gate-false-input",
            str(passing_gate_path),
            "--show-metadata",
            "--show-inputs",
            "--summary-output",
            str(summary_path),
            "--format",
            "text",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["sources"] == ["dynamic_godot_report_cli"]
    assert summary["sources_count"] == 1
    assert summary["sources_truncated"] is False
    assert summary["verification_scopes"] == ["godot_smoke_motion"]
    assert summary["verification_scopes_count"] == 1
    assert summary["verification_scopes_truncated"] is False
    assert summary["acceptance_profiles"] == ["custom"]
    assert summary["acceptance_profiles_count"] == 1
    assert summary["acceptance_profiles_truncated"] is False
    assert summary["passed_true_count"] == 1
    assert summary["passed_false_count"] == 1
    assert summary["passed_unknown_count"] == 0
    assert summary["passed_true_inputs"] == [str(passing_gate_path)]
    assert summary["passed_true_inputs_count"] == 1
    assert summary["passed_true_inputs_truncated"] is False
    assert summary["passed_false_inputs"] == [str(failing_gate_path)]
    assert summary["passed_false_inputs_count"] == 1
    assert summary["passed_false_inputs_truncated"] is False
    assert summary["passed_unknown_inputs"] == []
    assert summary["passed_unknown_inputs_count"] == 0
    assert summary["passed_unknown_inputs_truncated"] is False
    assert summary["requires_full_mechanical_restoration_gate_true_count"] == 1
    assert summary["requires_full_mechanical_restoration_gate_false_count"] == 1
    assert summary["requires_full_mechanical_restoration_gate_unknown_count"] == 0
    assert summary["full_mechanical_gate_true_inputs"] == [
        str(failing_gate_path)
    ]
    assert summary["full_mechanical_gate_true_inputs_count"] == 1
    assert summary["full_mechanical_gate_true_inputs_truncated"] is False
    assert summary["full_mechanical_gate_false_inputs"] == [
        str(passing_gate_path)
    ]
    assert summary["full_mechanical_gate_false_inputs_count"] == 1
    assert summary["full_mechanical_gate_false_inputs_truncated"] is False
    assert summary["full_mechanical_gate_unknown_inputs"] == []
    assert summary["full_mechanical_gate_unknown_inputs_count"] == 0
    assert summary["full_mechanical_gate_unknown_inputs_truncated"] is False
    assert summary["expected_passed_true_count"] == 1
    assert summary["expected_passed_false_count"] == 1
    assert summary["expected_passed_unknown_count"] == 0
    assert summary["expected_full_mechanical_gate_true_count"] == 1
    assert summary["expected_full_mechanical_gate_false_count"] == 1
    assert summary["expected_full_mechanical_gate_unknown_count"] == 0
    assert summary["expected_full_mechanical_gate_true_inputs"] == [
        str(failing_gate_path)
    ]
    assert summary["missing_expected_full_mechanical_gate_true_inputs"] == []
    assert summary["expected_full_mechanical_gate_false_inputs"] == [
        str(passing_gate_path)
    ]
    assert summary["missing_expected_full_mechanical_gate_false_inputs"] == []
    assert summary["allowed_full_mechanical_gate_true_inputs"] == [
        str(failing_gate_path)
    ]
    assert summary["unexpected_full_mechanical_gate_true_inputs"] == []
    assert summary["allowed_full_mechanical_gate_false_inputs"] == [
        str(passing_gate_path)
    ]
    assert summary["unexpected_full_mechanical_gate_false_inputs"] == []
    lines = validate_result.stdout.splitlines()
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "passed=true:1,false:1,unknown:0" in lines[0]
    assert "full_mechanical_gate=true:1,false:1,unknown:0" in lines[0]
    assert f"full_mechanical_gate_true_inputs={failing_gate_path}" in lines[0]
    assert f"full_mechanical_gate_false_inputs={passing_gate_path}" in lines[0]
    assert "full_mechanical_gate_unknown_inputs=none" in lines[0]
    assert "expected_passed_true_count=1" in lines[0]
    assert "expected_passed_false_count=1" in lines[0]
    assert "expected_passed_unknown_count=0" in lines[0]
    assert "expected_full_mechanical_gate_true_count=1" in lines[0]
    assert "expected_full_mechanical_gate_false_count=1" in lines[0]
    assert "expected_full_mechanical_gate_unknown_count=0" in lines[0]
    assert (
        f"expected_full_mechanical_gate_true_inputs={failing_gate_path}"
        in lines[0]
    )
    assert (
        f"expected_full_mechanical_gate_false_inputs={passing_gate_path}"
        in lines[0]
    )

    mismatch_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(passing_gate_path),
            str(failing_gate_path),
            "--expect-full-mechanical-gate-false-count",
            "0",
            "--forbid-full-mechanical-gate-false-input",
            str(passing_gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert mismatch_result.returncode == 1
    mismatch_payload = json.loads(mismatch_result.stdout)
    assert mismatch_payload["status"] == "error"
    assert mismatch_payload["expected_full_mechanical_gate_false_count"] == 0
    assert (
        "expected full_mechanical_gate_false_count 0 but found 1"
        in mismatch_payload["errors"]
    )
    assert mismatch_payload["forbidden_full_mechanical_gate_false_inputs"] == [
        str(passing_gate_path)
    ]
    assert mismatch_payload[
        "present_forbidden_full_mechanical_gate_false_inputs"
    ] == [str(passing_gate_path)]
    assert (
        f"forbidden full_mechanical_gate_false_input {passing_gate_path} was found"
        in mismatch_payload["errors"]
    )


def test_delivery_acceptance_gate_validator_can_fail_on_full_mechanical_gate_states(
    tmp_path: Path,
) -> None:
    passing_gate_path = tmp_path / "passing_gate.json"
    metadata_path = tmp_path / "metadata.json"
    summary_path = tmp_path / "summary.json"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    passing_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(passing_gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert passing_result.returncode == 0, passing_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(passing_gate_path),
            str(metadata_path),
            "--ignore-non-gate",
            "--fail-on-full-mechanical-gate-false",
            "--fail-on-full-mechanical-gate-unknown",
            "--summary-output",
            str(summary_path),
            "--format",
            "text",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert len(lines) == 3
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "status=error" in lines[0]
    assert "full_mechanical_gate=true:0,false:1,unknown:1" in lines[0]
    assert (
        "aggregate_errors=full_mechanical_gate=false inputs present: 1,"
        "full_mechanical_gate unknown inputs present: 1"
        in lines[0]
    )
    assert "full_mechanical_gate=false" in lines[1]
    assert "full_mechanical_gate=unknown" in lines[2]
    assert "fail_on_full_mechanical_gate_false=true" in lines[0]
    assert "fail_on_full_mechanical_gate_unknown=true" in lines[0]
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["status"] == "error"
    assert summary["fail_on_full_mechanical_gate_false"] is True
    assert summary["fail_on_full_mechanical_gate_unknown"] is True
    assert summary["errors"] == [
        "full_mechanical_gate=false inputs present: 1",
        "full_mechanical_gate unknown inputs present: 1",
    ]


def test_delivery_acceptance_gate_validator_records_full_mechanical_gate_fail_policy_when_clean(
    tmp_path: Path,
) -> None:
    strict_gate_path = tmp_path / "strict_gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--require-full-mechanical-restoration-gate",
            "--gate-output",
            str(strict_gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 1

    validate_text_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(strict_gate_path),
            "--fail-on-full-mechanical-gate-false",
            "--fail-on-full-mechanical-gate-unknown",
            "--format",
            "text",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_text_result.returncode == 0, validate_text_result.stderr
    lines = validate_text_result.stdout.splitlines()
    assert len(lines) == 2
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "status=success" in lines[0]
    assert "full_mechanical_gate=true:1,false:0,unknown:0" in lines[0]
    assert "fail_on_full_mechanical_gate_false=true" in lines[0]
    assert "fail_on_full_mechanical_gate_unknown=true" in lines[0]
    assert "require_passed=false" in lines[0]
    assert "full_mechanical_gate=true" in lines[1]

    validate_json_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(strict_gate_path),
            "--fail-on-full-mechanical-gate-false",
            "--fail-on-full-mechanical-gate-unknown",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_json_result.returncode == 0, validate_json_result.stderr
    payload = json.loads(validate_json_result.stdout)
    assert payload["status"] == "success"
    assert payload["fail_on_full_mechanical_gate_false"] is True
    assert payload["fail_on_full_mechanical_gate_unknown"] is True
    assert payload["requires_full_mechanical_restoration_gate_true_count"] == 1
    assert payload["requires_full_mechanical_restoration_gate_false_count"] == 0
    assert payload["requires_full_mechanical_restoration_gate_unknown_count"] == 0
    assert payload["results"][0]["requires_full_mechanical_restoration_gate"] is True


def test_delivery_acceptance_gate_validator_can_fail_on_passed_false(
    tmp_path: Path,
) -> None:
    passing_gate_path = tmp_path / "passing_gate.json"
    failing_gate_path = tmp_path / "failing_gate.json"
    summary_path = tmp_path / "summary.json"

    passing_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(passing_gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert passing_result.returncode == 0, passing_result.stderr

    failing_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--require-full-mechanical-restoration-gate",
            "--gate-output",
            str(failing_gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert failing_result.returncode == 1

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(passing_gate_path),
            str(failing_gate_path),
            "--fail-on-passed-false",
            "--summary-output",
            str(summary_path),
            "--format",
            "text",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["status"] == "error"
    assert summary["errors"] == ["passed=false gates present: 1"]
    assert summary["passed_false_inputs"] == [str(failing_gate_path)]
    assert summary["passed_true_inputs"] == [str(passing_gate_path)]
    lines = validate_result.stdout.splitlines()
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "passed=true:1,false:1,unknown:0" in lines[0]
    assert "aggregate_errors=passed=false gates present: 1" in lines[0]


def test_delivery_acceptance_gate_validator_can_expect_gate_metadata(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    summary_path = tmp_path / "summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-source",
            "dynamic_godot_report_cli",
            "--expect-verification-scope",
            "web_godot_load",
            "--expect-acceptance-profile",
            "custom",
            "--summary-output",
            str(summary_path),
            "--format",
            "text",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["sources"] == ["dynamic_godot_report_cli"]
    assert summary["verification_scopes"] == ["godot_smoke_motion"]
    assert summary["acceptance_profiles"] == ["custom"]
    assert summary["expected_sources"] == ["dynamic_godot_report_cli"]
    assert summary["missing_expected_sources"] == []
    assert summary["expected_verification_scopes"] == ["web_godot_load"]
    assert summary["missing_expected_verification_scopes"] == ["web_godot_load"]
    assert summary["expected_acceptance_profiles"] == ["custom"]
    assert summary["missing_expected_acceptance_profiles"] == []
    assert summary["errors"] == [
        "expected verification_scope web_godot_load was not found"
    ]
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "expected_sources=dynamic_godot_report_cli" in lines[0]
    assert "missing_expected_sources=none" in lines[0]
    assert "expected_verification_scopes=web_godot_load" in lines[0]
    assert "missing_expected_verification_scopes=web_godot_load" in lines[0]
    assert "expected_acceptance_profiles=custom" in lines[0]
    assert "missing_expected_acceptance_profiles=none" in lines[0]


def test_delivery_acceptance_gate_validator_can_allow_gate_metadata(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    summary_path = tmp_path / "summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--allow-source",
            "web_panel",
            "--allow-verification-scope",
            "web_godot_load",
            "--allow-acceptance-profile",
            "custom",
            "--summary-output",
            str(summary_path),
            "--format",
            "text",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["allowed_sources"] == ["web_panel"]
    assert summary["unexpected_sources"] == ["dynamic_godot_report_cli"]
    assert summary["allowed_verification_scopes"] == ["web_godot_load"]
    assert summary["unexpected_verification_scopes"] == ["godot_smoke_motion"]
    assert summary["allowed_acceptance_profiles"] == ["custom"]
    assert summary["unexpected_acceptance_profiles"] == []
    assert summary["errors"] == [
        "unexpected source dynamic_godot_report_cli was found",
        "unexpected verification_scope godot_smoke_motion was found",
    ]
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "allowed_sources=web_panel" in lines[0]
    assert "unexpected_sources=dynamic_godot_report_cli" in lines[0]
    assert "allowed_verification_scopes=web_godot_load" in lines[0]
    assert "unexpected_verification_scopes=godot_smoke_motion" in lines[0]
    assert "allowed_acceptance_profiles=custom" in lines[0]
    assert "unexpected_acceptance_profiles=none" in lines[0]


def test_delivery_acceptance_gate_validator_can_forbid_gate_metadata(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    summary_path = tmp_path / "summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--forbid-source",
            "dynamic_godot_report_cli",
            "--forbid-verification-scope",
            "godot_smoke_motion",
            "--forbid-acceptance-profile",
            "web_godot_load",
            "--summary-output",
            str(summary_path),
            "--format",
            "text",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["forbidden_sources"] == ["dynamic_godot_report_cli"]
    assert summary["present_forbidden_sources"] == ["dynamic_godot_report_cli"]
    assert summary["forbidden_verification_scopes"] == ["godot_smoke_motion"]
    assert summary["present_forbidden_verification_scopes"] == [
        "godot_smoke_motion"
    ]
    assert summary["forbidden_acceptance_profiles"] == ["web_godot_load"]
    assert summary["present_forbidden_acceptance_profiles"] == []
    assert summary["errors"] == [
        "forbidden source dynamic_godot_report_cli was found",
        "forbidden verification_scope godot_smoke_motion was found",
    ]
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "forbidden_sources=dynamic_godot_report_cli" in lines[0]
    assert "present_forbidden_sources=dynamic_godot_report_cli" in lines[0]
    assert "forbidden_verification_scopes=godot_smoke_motion" in lines[0]
    assert "present_forbidden_verification_scopes=godot_smoke_motion" in lines[0]
    assert "forbidden_acceptance_profiles=web_godot_load" in lines[0]
    assert "present_forbidden_acceptance_profiles=none" in lines[0]


def test_delivery_acceptance_gate_validator_can_constrain_enabled_requirements(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    summary_path = tmp_path / "summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--require-full-mechanical-restoration-gate",
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 1

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--show-metadata",
            "--expect-enabled-requirements-count",
            "2",
            "--expect-enabled-requirement",
            "full_mechanical_restoration_gate",
            "--expect-enabled-requirement",
            "missing_gate",
            "--allow-enabled-requirement",
            "missing_gate",
            "--forbid-enabled-requirement",
            "full_mechanical_restoration_gate",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["enabled_requirements"] == [
        "full_mechanical_restoration_gate"
    ]
    assert summary["enabled_requirements_count"] == 1
    assert summary["enabled_requirements_truncated"] is False
    assert summary["expected_enabled_requirements_count"] == 2
    assert summary["expected_enabled_requirements"] == [
        "full_mechanical_restoration_gate",
        "missing_gate",
    ]
    assert summary["missing_expected_enabled_requirements"] == ["missing_gate"]
    assert summary["allowed_enabled_requirements"] == ["missing_gate"]
    assert summary["unexpected_enabled_requirements"] == [
        "full_mechanical_restoration_gate"
    ]
    assert summary["forbidden_enabled_requirements"] == [
        "full_mechanical_restoration_gate"
    ]
    assert summary["present_forbidden_enabled_requirements"] == [
        "full_mechanical_restoration_gate"
    ]
    assert summary["errors"] == [
        "expected enabled_requirements_count 2 but found 1",
        "expected enabled_requirement missing_gate was not found",
        "unexpected enabled_requirement full_mechanical_restoration_gate was found",
        "forbidden enabled_requirement full_mechanical_restoration_gate was found",
    ]
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "enabled_requirements=full_mechanical_restoration_gate" in lines[0]
    assert "expected_enabled_requirements_count=2" in lines[0]
    assert (
        "expected_enabled_requirements=full_mechanical_restoration_gate,missing_gate"
        in lines[0]
    )
    assert "missing_expected_enabled_requirements=missing_gate" in lines[0]
    assert "allowed_enabled_requirements=missing_gate" in lines[0]
    assert (
        "unexpected_enabled_requirements=full_mechanical_restoration_gate"
        in lines[0]
    )
    assert (
        "forbidden_enabled_requirements=full_mechanical_restoration_gate"
        in lines[0]
    )
    assert (
        "present_forbidden_enabled_requirements=full_mechanical_restoration_gate"
        in lines[0]
    )


def test_delivery_acceptance_gate_validator_can_constrain_contract_versions(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    summary_path = tmp_path / "summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--show-metadata",
            "--expect-contract-versions-count",
            "2",
            "--expect-contract-version",
            "delivery_acceptance_gate.v1",
            "--expect-contract-version",
            "delivery_acceptance_gate.v2",
            "--allow-contract-version",
            "delivery_acceptance_gate.v2",
            "--forbid-contract-version",
            "delivery_acceptance_gate.v1",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["contract_versions"] == ["delivery_acceptance_gate.v1"]
    assert summary["contract_versions_count"] == 1
    assert summary["contract_versions_truncated"] is False
    assert summary["expected_contract_versions_count"] == 2
    assert summary["expected_contract_versions"] == [
        "delivery_acceptance_gate.v1",
        "delivery_acceptance_gate.v2",
    ]
    assert summary["missing_expected_contract_versions"] == [
        "delivery_acceptance_gate.v2"
    ]
    assert summary["allowed_contract_versions"] == [
        "delivery_acceptance_gate.v2"
    ]
    assert summary["unexpected_contract_versions"] == [
        "delivery_acceptance_gate.v1"
    ]
    assert summary["forbidden_contract_versions"] == [
        "delivery_acceptance_gate.v1"
    ]
    assert summary["present_forbidden_contract_versions"] == [
        "delivery_acceptance_gate.v1"
    ]
    assert summary["errors"] == [
        "expected contract_versions_count 2 but found 1",
        "expected contract_version delivery_acceptance_gate.v2 was not found",
        "unexpected contract_version delivery_acceptance_gate.v1 was found",
        "forbidden contract_version delivery_acceptance_gate.v1 was found",
    ]
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "contract_versions=delivery_acceptance_gate.v1" in lines[0]
    assert "expected_contract_versions_count=2" in lines[0]
    assert (
        "expected_contract_versions=delivery_acceptance_gate.v1,"
        "delivery_acceptance_gate.v2"
        in lines[0]
    )
    assert (
        "missing_expected_contract_versions=delivery_acceptance_gate.v2"
        in lines[0]
    )
    assert "allowed_contract_versions=delivery_acceptance_gate.v2" in lines[0]
    assert (
        "unexpected_contract_versions=delivery_acceptance_gate.v1"
        in lines[0]
    )
    assert "forbidden_contract_versions=delivery_acceptance_gate.v1" in lines[0]
    assert (
        "present_forbidden_contract_versions=delivery_acceptance_gate.v1"
        in lines[0]
    )


def test_delivery_acceptance_gate_validator_can_constrain_levels(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    summary_path = tmp_path / "summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--show-metadata",
            "--expect-levels-count",
            "2",
            "--expect-level",
            "static_only",
            "--expect-level",
            "godot_verified",
            "--allow-level",
            "godot_verified",
            "--forbid-level",
            "static_only",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["levels"] == ["static_only"]
    assert summary["levels_count"] == 1
    assert summary["levels_truncated"] is False
    assert summary["expected_levels_count"] == 2
    assert summary["expected_levels"] == ["static_only", "godot_verified"]
    assert summary["missing_expected_levels"] == ["godot_verified"]
    assert summary["allowed_levels"] == ["godot_verified"]
    assert summary["unexpected_levels"] == ["static_only"]
    assert summary["forbidden_levels"] == ["static_only"]
    assert summary["present_forbidden_levels"] == ["static_only"]
    assert summary["errors"] == [
        "expected levels_count 2 but found 1",
        "expected level godot_verified was not found",
        "unexpected level static_only was found",
        "forbidden level static_only was found",
    ]
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "levels=static_only" in lines[0]
    assert "expected_levels_count=2" in lines[0]
    assert "expected_levels=static_only,godot_verified" in lines[0]
    assert "missing_expected_levels=godot_verified" in lines[0]
    assert "allowed_levels=godot_verified" in lines[0]
    assert "unexpected_levels=static_only" in lines[0]
    assert "forbidden_levels=static_only" in lines[0]
    assert "present_forbidden_levels=static_only" in lines[0]


def test_delivery_acceptance_gate_validator_can_show_passed_counts_in_text(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--show-passed-counts",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    lines = validate_result.stdout.splitlines()
    assert len(lines) == 2
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "status=success" in lines[0]
    assert "passed=true:1,false:0,unknown:0" in lines[0]
    assert "require_passed=false" in lines[0]
    assert "status=success" in lines[1]


def test_delivery_acceptance_gate_validator_can_show_metadata_in_text(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--show-metadata",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    lines = validate_result.stdout.splitlines()
    assert len(lines) == 2
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "status=success" in lines[0]
    assert "sources=dynamic_godot_report_cli" in lines[0]
    assert "verification_scopes=godot_smoke_motion" in lines[0]
    assert "acceptance_profiles=custom" in lines[0]
    assert "require_passed=false" in lines[0]
    assert "status=success" in lines[1]


def test_delivery_acceptance_gate_validator_can_show_reason_codes_in_text(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--show-reason-codes",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    lines = validate_result.stdout.splitlines()
    assert len(lines) == 2
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "status=success" in lines[0]
    assert "reason_codes=missing_godot_smoke" in lines[0]
    assert "require_passed=false" in lines[0]
    assert "status=success" in lines[1]


def test_delivery_acceptance_gate_validator_can_show_input_previews_in_text(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    metadata_path = tmp_path / "metadata.json"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_path),
            "--ignore-non-gate",
            "--format",
            "text",
            "--show-inputs",
            "--show-skipped-reasons",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    lines = validate_result.stdout.splitlines()
    assert len(lines) == 3
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "status=success" in lines[0]
    assert "affected_inputs=" in lines[0]
    assert FIXED_PAIR_FIXTURE.name in lines[0]
    assert "failed_inputs=none" in lines[0]
    assert f"skipped_inputs={metadata_path}" in lines[0]
    assert f"full_mechanical_gate_false_inputs={gate_path}" in lines[0]
    assert f"full_mechanical_gate_unknown_inputs={metadata_path}" in lines[0]
    assert (
        "skipped_reasons=not a delivery_acceptance_gate artifact or report"
        in lines[0]
    )
    assert "status=success" in lines[1]
    assert "full_mechanical_gate=false" in lines[1]
    assert "status=skipped" in lines[2]
    assert "full_mechanical_gate=unknown" in lines[2]


def test_delivery_acceptance_gate_validator_can_show_diagnostics_in_text(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    metadata_path = tmp_path / "metadata.json"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_path),
            "--ignore-non-gate",
            "--format",
            "text",
            "--show-diagnostics",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    lines = validate_result.stdout.splitlines()
    assert len(lines) == 3
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "status=success" in lines[0]
    assert "passed=true:1,false:0,unknown:1" in lines[0]
    assert "sources=dynamic_godot_report_cli" in lines[0]
    assert "verification_scopes=godot_smoke_motion" in lines[0]
    assert "acceptance_profiles=custom" in lines[0]
    assert "reason_codes=missing_godot_smoke" in lines[0]
    assert "affected_inputs=" in lines[0]
    assert FIXED_PAIR_FIXTURE.name in lines[0]
    assert f"skipped_inputs={metadata_path}" in lines[0]
    assert (
        "skipped_reasons=not a delivery_acceptance_gate artifact or report"
        in lines[0]
    )
    assert "status=success" in lines[1]
    assert "status=skipped" in lines[2]


def test_delivery_acceptance_gate_validator_can_emit_summary_only_text(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    metadata_path = tmp_path / "metadata.json"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_path),
            "--ignore-non-gate",
            "--format",
            "text",
            "--show-diagnostics",
            "--summary-only",
            "--expect-complete-required-summary-fields-source-scope-count",
            "1",
            "--expect-complete-required-summary-fields-source-scope",
            "dynamic_godot_report_cli/godot_smoke_motion",
            "--expect-node-tree-gate-check-count",
            "fixed_lock_mismatch=0",
            "--expect-mechanical-gate-check-count",
            "mechanical_restoration=0",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    lines = validate_result.stdout.splitlines()
    assert len(lines) == 1
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "status=success" in lines[0]
    assert "inputs=2" in lines[0]
    assert "skipped=1" in lines[0]
    assert "passed=true:1,false:0,unknown:1" in lines[0]
    assert "reason_codes=missing_godot_smoke" in lines[0]
    assert "checks=none" in lines[0]
    assert (
        "expected_checks=tree[fixed_lock_mismatch:0]|"
        "mech[mechanical_restoration:0]" in lines[0]
    )
    assert "mismatched_expected_checks=none" in lines[0]
    assert f"skipped_inputs={metadata_path}" in lines[0]
    assert (
        "complete_required_summary_fields="
        "dynamic_godot_report_cli/godot_smoke_motion:" in lines[0]
    )
    assert "complete_required_summary_fields_source_scope_count=1" in lines[0]
    assert (
        "complete_required_summary_fields_source_scopes="
        "dynamic_godot_report_cli/godot_smoke_motion" in lines[0]
    )
    assert (
        "expected_complete_required_summary_fields_source_scope_count=1"
        in lines[0]
    )
    assert (
        "expected_complete_required_summary_fields_source_scopes="
        "dynamic_godot_report_cli/godot_smoke_motion" in lines[0]
    )
    assert (
        "missing_expected_complete_required_summary_fields_source_scopes=none"
        in lines[0]
    )
    assert "smoke_report_written_count" in lines[0]
    assert "status=skipped" not in lines[0]


def test_delivery_acceptance_gate_validator_can_emit_summary_only_json(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    metadata_path = tmp_path / "metadata.json"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_path),
            "--ignore-non-gate",
            "--summary-only",
            "--expect-complete-required-summary-fields-source-scope-count",
            "1",
            "--expect-complete-required-summary-fields-source-scope",
            "dynamic_godot_report_cli/godot_smoke_motion",
            "--expect-node-tree-gate-check-count",
            "fixed_lock_mismatch=0",
            "--expect-mechanical-gate-check-count",
            "mechanical_restoration=0",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    payload = json.loads(validate_result.stdout)
    assert payload["summary_version"] == (
        "delivery_acceptance_gate_validation_summary.v1"
    )
    assert payload["status"] == "success"
    assert payload["inputs_count"] == 2
    assert payload["success_count"] == 1
    assert payload["skipped_count"] == 1
    assert payload["passed_true_count"] == 1
    assert payload["passed_unknown_count"] == 1
    assert payload["reason_codes"] == ["missing_godot_smoke", "static_only"]
    assert payload["node_tree_gate_check_counts"] == {
        "class_mismatch": 0,
        "fixed_lock_mismatch": 0,
        "incomplete_node_tree": 0,
        "missing_parameters": 0,
        "physical_mismatch": 0,
        "transform_mismatch": 0,
    }
    assert payload["mechanical_gate_check_counts"] == {
        "control_parameter_readback": 0,
        "full_node_tree_restoration": 0,
        "joint_parameter_readback": 0,
        "mechanical_restoration": 0,
    }
    assert payload["expected_node_tree_gate_check_counts"] == {
        "fixed_lock_mismatch": 0,
    }
    assert payload["mismatched_expected_node_tree_gate_check_counts"] == {}
    assert payload["expected_mechanical_gate_check_counts"] == {
        "mechanical_restoration": 0,
    }
    assert payload["mismatched_expected_mechanical_gate_check_counts"] == {}
    assert payload["complete_required_summary_fields_by_source_scope"][
        "dynamic_godot_report_cli"
    ]["godot_smoke_motion"] == [
        "control_readback_missing_count",
        "delivery_complete_count",
        "delivery_dynamic_generation_count",
        "delivery_godot_verified_count",
        "delivery_incomplete_count",
        "delivery_parameters_incomplete_count",
        "delivery_static_only_count",
        "delivery_unverified_count",
        "error_count",
        "failure_reasons_count",
        "fixed_lock_mismatch_count",
        "live_smoke_count",
        "node_tree_fixed_lock_mismatch_count",
        "node_tree_fixed_locks_incomplete_count",
        "smoke_report_missing_count",
        "smoke_report_read_error_count",
        "smoke_report_written_count",
        "static_node_tree_complete_count",
        "static_node_tree_endpoint_paths_complete_count",
        "static_node_tree_endpoint_paths_incomplete_count",
        "static_node_tree_incomplete_count",
        "static_node_tree_manifest_count",
        "static_node_tree_manifest_error_count",
        "static_node_tree_manifest_invalid_count",
        "static_node_tree_manifest_path_map_mismatch_count",
        "static_node_tree_manifest_valid_count",
        "static_node_tree_missing_endpoint_connections_count",
        "static_node_tree_missing_endpoint_parts_count",
        "static_node_tree_parameters_complete_count",
        "static_node_tree_parameters_incomplete_count",
        "static_topology_complete_count",
        "static_topology_cycle_count",
        "static_topology_disconnected_parts_count",
        "static_topology_duplicate_child_endpoint_count",
        "static_topology_incomplete_count",
        "static_topology_unreachable_parts_count",
        "success_count",
    ]
    assert payload["complete_required_summary_fields_source_scope_count"] == 1
    assert payload["complete_required_summary_fields_source_scopes"] == [
        "dynamic_godot_report_cli/godot_smoke_motion"
    ]
    assert (
        payload["expected_complete_required_summary_fields_source_scope_count"]
        == 1
    )
    assert payload[
        "expected_complete_required_summary_fields_source_scopes"
    ] == ["dynamic_godot_report_cli/godot_smoke_motion"]
    assert payload[
        "missing_expected_complete_required_summary_fields_source_scopes"
    ] == []
    assert payload["skipped_inputs"] == [str(metadata_path)]
    assert payload["skipped_reasons"] == [
        "not a delivery_acceptance_gate artifact or report"
    ]
    assert "results" not in payload


def test_delivery_acceptance_gate_validator_can_expect_gate_check_counts(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--summary-only",
            "--expect-node-tree-gate-check-count",
            "fixed_lock_mismatch=1",
            "--expect-mechanical-gate-check-count",
            "mechanical_restoration=1",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    assert payload["expected_node_tree_gate_check_counts"] == {
        "fixed_lock_mismatch": 1,
    }
    assert payload["mismatched_expected_node_tree_gate_check_counts"] == {
        "fixed_lock_mismatch": {"actual": 0, "expected": 1},
    }
    assert payload["expected_mechanical_gate_check_counts"] == {
        "mechanical_restoration": 1,
    }
    assert payload["mismatched_expected_mechanical_gate_check_counts"] == {
        "mechanical_restoration": {"actual": 0, "expected": 1},
    }
    assert payload["errors"] == [
        "expected node_tree_gate_check_counts.fixed_lock_mismatch 1 but found 0",
        "expected mechanical_gate_check_counts.mechanical_restoration 1 but found 0",
    ]

    text_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--summary-only",
            "--expect-node-tree-gate-check-count",
            "fixed_lock_mismatch=1",
            "--expect-mechanical-gate-check-count",
            "mechanical_restoration=1",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert text_result.returncode == 1
    lines = text_result.stdout.splitlines()
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert (
        "expected_checks=tree[fixed_lock_mismatch:1]|"
        "mech[mechanical_restoration:1]" in lines[0]
    )
    assert (
        "mismatched_expected_checks=tree[fixed_lock_mismatch:1/0]|"
        "mech[mechanical_restoration:1/0]" in lines[0]
    )
    assert (
        "aggregate_errors=expected node_tree_gate_check_counts.fixed_lock_mismatch "
        "1 but found 0,expected mechanical_gate_check_counts.mechanical_restoration "
        "1 but found 0" in lines[0]
    )

    full_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-node-tree-gate-check-count",
            "fixed_lock_mismatch=0",
            "--expect-mechanical-gate-check-count",
            "mechanical_restoration=0",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert full_result.returncode == 0
    full_payload = json.loads(full_result.stdout)
    assert full_payload["expected_node_tree_gate_check_counts"] == {
        "fixed_lock_mismatch": 0,
    }
    assert full_payload["mismatched_expected_node_tree_gate_check_counts"] == {}
    assert full_payload["expected_mechanical_gate_check_counts"] == {
        "mechanical_restoration": 0,
    }
    assert full_payload["mismatched_expected_mechanical_gate_check_counts"] == {}
    assert full_payload["node_tree_gate_check_counts"]["fixed_lock_mismatch"] == 0
    assert full_payload["mechanical_gate_check_counts"]["mechanical_restoration"] == 0
    assert full_payload["results"][0]["summary_counts"][
        "node_tree_gate_check_counts"
    ]["fixed_lock_mismatch"] == 0


def test_delivery_acceptance_gate_validator_can_expect_summary_counts(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    manifest_output_dir = tmp_path / "node_tree_manifests"
    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
            "--static-node-tree-manifest-dir",
            str(manifest_output_dir),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--summary-only",
            "--expect-summary-count",
            "delivery_static_only_count=0",
            "--expect-summary-count",
            "failure_reasons_count=2",
            "--expect-summary-count",
            "static_node_tree_joints_planned_count=2",
            "--expect-summary-count",
            "static_node_tree_missing_endpoint_connections_count=1",
            "--expect-summary-count",
            "static_node_tree_manifest_output_count=0",
            "--expect-summary-count",
            "static_topology_complete_count=0",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert (
        "expected_summary_counts=delivery_static_only_count:0,"
        "failure_reasons_count:2,static_node_tree_joints_planned_count:2,"
        "static_node_tree_manifest_output_count:0,"
        "static_node_tree_missing_endpoint_connections_count:1,"
        "static_topology_complete_count:0" in lines[0]
    )
    assert (
        "mismatched_expected_summary_counts="
        "delivery_static_only_count:0/1,failure_reasons_count:2/0,"
        "static_node_tree_joints_planned_count:2/1,"
        "static_node_tree_manifest_output_count:0/1,"
        "static_node_tree_missing_endpoint_connections_count:1/0,"
        "static_topology_complete_count:0/1"
        in lines[0]
    )
    assert (
        "aggregate_errors=expected summary_counts.delivery_static_only_count "
        "0 but found 1,expected summary_counts.failure_reasons_count 2 but found 0,"
        "expected summary_counts.static_node_tree_joints_planned_count 2 but found 1,"
        "expected summary_counts.static_node_tree_manifest_output_count 0 but found 1,"
        "expected summary_counts.static_node_tree_missing_endpoint_connections_count "
        "1 but found 0,"
        "expected summary_counts.static_topology_complete_count 0 but found 1"
        in lines[0]
    )

    success_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-summary-count",
            "delivery_static_only_count=1",
            "--expect-summary-count",
            "failure_reasons_count=0",
            "--expect-summary-count",
            "static_node_tree_joints_planned_count=1",
            "--expect-summary-count",
            "static_node_tree_manifest_output_count=1",
            "--expect-summary-count",
            "static_node_tree_missing_endpoint_connections_count=0",
            "--expect-summary-count",
            "static_topology_complete_count=1",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert success_result.returncode == 0
    payload = json.loads(success_result.stdout)
    assert payload["expected_summary_counts"] == {
        "delivery_static_only_count": 1,
        "failure_reasons_count": 0,
        "static_node_tree_joints_planned_count": 1,
        "static_node_tree_manifest_output_count": 1,
        "static_node_tree_missing_endpoint_connections_count": 0,
        "static_topology_complete_count": 1,
    }
    assert payload["actual_expected_summary_counts"] == {
        "delivery_static_only_count": 1,
        "failure_reasons_count": 0,
        "static_node_tree_joints_planned_count": 1,
        "static_node_tree_manifest_output_count": 1,
        "static_node_tree_missing_endpoint_connections_count": 0,
        "static_topology_complete_count": 1,
    }
    assert payload["mismatched_expected_summary_counts"] == {}
    assert payload["results"][0]["summary_counts"]["delivery_complete_count"] == 1


def test_delivery_acceptance_gate_validator_can_expect_summary_values(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--summary-only",
            "--expect-summary-value",
            "delivery_static_only_count=0",
            "--expect-summary-value",
            "node_tree_gate_check_counts.fixed_lock_mismatch=1",
            "--expect-summary-value",
            "static_node_tree_missing_endpoint_connections_count=1",
            "--expect-summary-value",
            "static_node_tree_parts_planned_count=3",
            "--expect-summary-value-source",
            "dynamic_godot_report_cli",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert (
        "expected_summary_values=delivery_static_only_count:0,"
        "node_tree_gate_check_counts.fixed_lock_mismatch:1,"
        "static_node_tree_missing_endpoint_connections_count:1,"
        "static_node_tree_parts_planned_count:3" in lines[0]
    )
    assert "expected_summary_value_sources=dynamic_godot_report_cli" in lines[0]
    assert (
        "mismatched_expected_summary_values=delivery_static_only_count:0/1,"
        "node_tree_gate_check_counts.fixed_lock_mismatch:1/0,"
        "static_node_tree_missing_endpoint_connections_count:1/0,"
        "static_node_tree_parts_planned_count:3/2" in lines[0]
    )
    assert (
        "aggregate_errors=expected summary_counts.delivery_static_only_count "
        "0 but found 1,expected summary_counts.node_tree_gate_check_counts."
        "fixed_lock_mismatch 1 but found 0,"
        "expected summary_counts.static_node_tree_missing_endpoint_connections_count "
        "1 but found 0,"
        "expected summary_counts.static_node_tree_parts_planned_count 3 but found 2"
        in lines[0]
    )

    success_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-summary-value",
            "delivery_static_only_count=1",
            "--expect-summary-value",
            "node_tree_gate_check_counts.fixed_lock_mismatch=0",
            "--expect-summary-value",
            "static_node_tree_missing_endpoint_connections_count=0",
            "--expect-summary-value",
            "static_node_tree_parts_planned_count=2",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert success_result.returncode == 0
    payload = json.loads(success_result.stdout)
    assert payload["expected_summary_values"] == {
        "delivery_static_only_count": 1,
        "node_tree_gate_check_counts.fixed_lock_mismatch": 0,
        "static_node_tree_missing_endpoint_connections_count": 0,
        "static_node_tree_parts_planned_count": 2,
    }
    assert payload["actual_expected_summary_values"] == {
        "delivery_static_only_count": 1,
        "node_tree_gate_check_counts.fixed_lock_mismatch": 0,
        "static_node_tree_missing_endpoint_connections_count": 0,
        "static_node_tree_parts_planned_count": 2,
    }
    assert payload["mismatched_expected_summary_values"] == {}


def test_delivery_acceptance_gate_validator_rejects_unknown_gate_check_count(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    gate_path.write_text("{}", encoding="utf-8")

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-node-tree-gate-check-count",
            "unknown_check=0",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 2
    assert (
        "unsupported --expect-node-tree-gate-check-count check(s): unknown_check"
        in validate_result.stderr
    )
    assert "fixed_lock_mismatch" in validate_result.stderr


def test_delivery_acceptance_gate_validator_rejects_unknown_summary_count(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    gate_path.write_text("{}", encoding="utf-8")

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-summary-count",
            "unknown_count=0",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 2
    assert (
        "unsupported --expect-summary-count field(s): unknown_count"
        in validate_result.stderr
    )
    assert "delivery_static_only_count" in validate_result.stderr


def test_delivery_acceptance_gate_validator_rejects_unknown_summary_value(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    gate_path.write_text("{}", encoding="utf-8")

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-summary-value",
            "node_tree_gate_check_counts.unknown=0",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 2
    assert (
        "unsupported --expect-summary-value path(s): "
        "node_tree_gate_check_counts.unknown" in validate_result.stderr
    )
    assert "node_tree_gate_check_counts.fixed_lock_mismatch" in validate_result.stderr


def test_delivery_acceptance_gate_validator_rejects_source_specific_summary_value(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    gate_path.write_text("{}", encoding="utf-8")

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-summary-value",
            "control_readback_missing_count=0",
            "--expect-summary-value-source",
            "web_godot_delivery",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 2
    assert (
        "unsupported --expect-summary-value path(s) for source web_godot_delivery: "
        "control_readback_missing_count" in validate_result.stderr
    )
    assert "mechanical_gate_check_counts.mechanical_restoration" in (
        validate_result.stderr
    )


def test_delivery_acceptance_gate_validator_accepts_source_specific_summary_value(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    gate_path.write_text(
        json.dumps(
            {
                "contract_version": "delivery_acceptance_gate.v1",
                "source": "web_godot_delivery",
                "verification_scope": "godot_load",
                "required": False,
                "requires_full_mechanical_restoration_gate": False,
                "acceptance_profile": "web_godot_load",
                "acceptance_requirements": _acceptance_requirements(
                    godot_load=True
                ),
                "passed": False,
                "exit_code": 1,
                "level": "incomplete",
                "complete": False,
                "reasons": ["Godot delivery failed before load acceptance."],
                "reason_codes": ["godot_delivery_failed"],
                "reason_details": [
                    {
                        "code": "godot_delivery_failed",
                        "count": 1,
                        "message": "Godot delivery failed before load acceptance.",
                        "inputs": ["robot.json"],
                        "inputs_count": 1,
                        "inputs_truncated": False,
                    }
                ],
                "summary_counts": {
                    "inputs_count": 1,
                    "success_count": 0,
                    "error_count": 1,
                    "live_smoke_count": 0,
                    "delivery_godot_verified_count": 0,
                    "delivery_static_only_count": 0,
                    "delivery_unverified_count": 1,
                    "delivery_dynamic_generation_count": 0,
                    "delivery_complete_count": 0,
                    "delivery_incomplete_count": 1,
                    "delivery_parameters_incomplete_count": 1,
                    "fixed_lock_checked_count": 0,
                    "fixed_lock_mismatch_count": 0,
                    "node_tree_fixed_lock_checked_count": 0,
                    "node_tree_fixed_lock_mismatch_count": 0,
                    "node_tree_fixed_locks_complete_count": 0,
                    "node_tree_fixed_locks_incomplete_count": 0,
                    "node_tree_gate_enabled_count": 0,
                    "node_tree_full_restoration_required_count": 0,
                    "node_tree_full_restoration_not_required_count": 1,
                    "node_tree_gate_check_counts": {},
                    "mechanical_gate_enabled_count": 0,
                    "full_mechanical_restoration_required_count": 0,
                    "full_mechanical_restoration_not_required_count": 1,
                    "mechanical_gate_check_counts": {},
                    "failure_reasons_count": 1,
                },
            }
        ),
        encoding="utf-8",
    )

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-summary-value",
            "mechanical_gate_check_counts.mechanical_restoration=0",
            "--expect-summary-value-source",
            "web_godot_delivery",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    payload = json.loads(validate_result.stdout)
    assert payload["expected_summary_values"] == {
        "mechanical_gate_check_counts.mechanical_restoration": 0
    }
    assert payload["expected_summary_value_sources"] == ["web_godot_delivery"]
    assert payload["missing_expected_summary_value_sources"] == []
    assert payload["summary_value_source_matched_count"] == 1
    assert payload["summary_value_source_excluded_count"] == 0
    assert payload["summary_value_source_excluded_sources"] == []


def test_delivery_acceptance_gate_validator_rejects_missing_summary_value_source(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--summary-only",
            "--expect-summary-value",
            "mechanical_gate_check_counts.mechanical_restoration=0",
            "--expect-summary-value-source",
            "web_godot_delivery",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert "expected_summary_value_sources=web_godot_delivery" in lines[0]
    assert "missing_expected_summary_value_sources=web_godot_delivery" in lines[0]
    assert "summary_value_source_matched_count=0" in lines[0]
    assert (
        "expected summary_value_source web_godot_delivery was not found"
        in lines[0]
    )


def test_delivery_acceptance_gate_validator_scopes_summary_values_by_source(
    tmp_path: Path,
) -> None:
    def gate_payload(
        *,
        source: str,
        verification_scope: str,
        acceptance_profile: str,
        reason_code: str,
        reason: str,
        mechanical_restoration_count: int,
    ) -> dict[str, object]:
        return {
            "contract_version": "delivery_acceptance_gate.v1",
            "source": source,
            "verification_scope": verification_scope,
            "required": False,
            "requires_full_mechanical_restoration_gate": False,
            "acceptance_profile": acceptance_profile,
            "acceptance_requirements": _acceptance_requirements(
                godot_load=(source == "web_godot_delivery")
            ),
            "passed": False,
            "exit_code": 1,
            "level": "incomplete",
            "complete": False,
            "reasons": [reason],
            "reason_codes": [reason_code],
            "reason_details": [
                {
                    "code": reason_code,
                    "count": 1,
                    "message": reason,
                    "inputs": [f"{source}.json"],
                    "inputs_count": 1,
                    "inputs_truncated": False,
                }
            ],
            "summary_counts": {
                "inputs_count": 1,
                "success_count": 0,
                "error_count": 1,
                "live_smoke_count": 0,
                "delivery_godot_verified_count": 0,
                "delivery_static_only_count": 0,
                "delivery_unverified_count": 1,
                "delivery_dynamic_generation_count": 0,
                "delivery_complete_count": 0,
                "delivery_incomplete_count": 1,
                "delivery_parameters_incomplete_count": 1,
                "fixed_lock_checked_count": 0,
                "fixed_lock_mismatch_count": 0,
                "node_tree_fixed_lock_checked_count": 0,
                "node_tree_fixed_lock_mismatch_count": 0,
                "node_tree_fixed_locks_complete_count": 0,
                "node_tree_fixed_locks_incomplete_count": 0,
                "node_tree_gate_enabled_count": 0,
                "node_tree_full_restoration_required_count": 0,
                "node_tree_full_restoration_not_required_count": 1,
                "node_tree_gate_check_counts": {},
                "mechanical_gate_enabled_count": mechanical_restoration_count,
                "full_mechanical_restoration_required_count": 0,
                "full_mechanical_restoration_not_required_count": 1,
                "mechanical_gate_check_counts": {
                    "mechanical_restoration": mechanical_restoration_count
                },
                "failure_reasons_count": 1,
            },
        }

    (tmp_path / "cli_gate.json").write_text(
        json.dumps(
            gate_payload(
                source="dynamic_godot_report_cli",
                verification_scope="godot_smoke_motion",
                acceptance_profile="custom",
                reason_code="missing_godot_smoke",
                reason="1 robot(s) were not run through Godot smoke",
                mechanical_restoration_count=9,
            )
        ),
        encoding="utf-8",
    )
    (tmp_path / "web_gate.json").write_text(
        json.dumps(
            gate_payload(
                source="web_godot_delivery",
                verification_scope="godot_load",
                acceptance_profile="web_godot_load",
                reason_code="godot_delivery_failed",
                reason="Godot delivery failed before load acceptance.",
                mechanical_restoration_count=1,
            )
        ),
        encoding="utf-8",
    )

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(tmp_path),
            "--summary-only",
            "--expect-summary-value",
            "mechanical_gate_check_counts.mechanical_restoration=1",
            "--expect-summary-value-source",
            "web_godot_delivery",
            "--expect-summary-value-source-matched-count",
            "1",
            "--expect-summary-value-source-excluded-count",
            "1",
            "--allow-summary-value-source-matched-source",
            "web_godot_delivery",
            "--forbid-summary-value-source-matched-source",
            "dynamic_godot_report_cli",
            "--expect-summary-value-source-excluded-source",
            "dynamic_godot_report_cli",
            "--allow-summary-value-source-excluded-source",
            "dynamic_godot_report_cli",
            "--forbid-summary-value-source-excluded-source",
            "web_godot_delivery",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    payload = json.loads(validate_result.stdout)
    schema = workflow_contracts.DELIVERY_ACCEPTANCE_GATE_SCHEMA
    assert set(schema["summary_value_source_filter_fields"]).issubset(payload)
    assert sorted(schema["summary_value_source_filter_field_types"]) == schema[
        "summary_value_source_filter_fields"
    ]
    assert payload["actual_expected_summary_values"] == {
        "mechanical_gate_check_counts.mechanical_restoration": 1
    }
    assert payload["summary_value_source_matched_count"] == 1
    assert payload["summary_value_source_matched_sources"] == ["web_godot_delivery"]
    assert payload["allowed_summary_value_source_matched_sources"] == [
        "web_godot_delivery"
    ]
    assert payload["unexpected_summary_value_source_matched_sources"] == []
    assert payload["forbidden_summary_value_source_matched_sources"] == [
        "dynamic_godot_report_cli"
    ]
    assert payload["present_forbidden_summary_value_source_matched_sources"] == []
    assert payload["summary_value_source_excluded_count"] == 1
    assert payload["expected_summary_value_source_matched_count"] == 1
    assert payload["expected_summary_value_source_excluded_count"] == 1
    assert payload["expected_summary_value_source_excluded_sources"] == [
        "dynamic_godot_report_cli"
    ]
    assert payload["missing_expected_summary_value_source_excluded_sources"] == []
    assert payload["allowed_summary_value_source_excluded_sources"] == [
        "dynamic_godot_report_cli"
    ]
    assert payload["unexpected_summary_value_source_excluded_sources"] == []
    assert payload["forbidden_summary_value_source_excluded_sources"] == [
        "web_godot_delivery"
    ]
    assert payload["present_forbidden_summary_value_source_excluded_sources"] == []
    assert payload["summary_value_source_excluded_sources"] == [
        "dynamic_godot_report_cli"
    ]
    assert payload["sources"] == [
        "dynamic_godot_report_cli",
        "web_godot_delivery",
    ]


def test_delivery_acceptance_gate_validator_rejects_summary_value_source_count_mismatch(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--summary-only",
            "--expect-summary-value",
            "mechanical_gate_check_counts.mechanical_restoration=0",
            "--expect-summary-value-source",
            "dynamic_godot_report_cli",
            "--expect-summary-value-source-matched-count",
            "2",
            "--expect-summary-value-source-excluded-count",
            "1",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert "expected_summary_value_source_matched_count=2" in lines[0]
    assert "expected_summary_value_source_excluded_count=1" in lines[0]
    assert "summary_value_source_matched_count=1" in lines[0]
    assert "summary_value_source_excluded_count=0" in lines[0]
    assert (
        "expected summary_value_source_matched_count 2 but found 1"
        in lines[0]
    )
    assert (
        "expected summary_value_source_excluded_count 1 but found 0"
        in lines[0]
    )


def test_delivery_acceptance_gate_validator_rejects_missing_summary_value_excluded_source(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--summary-only",
            "--expect-summary-value",
            "mechanical_gate_check_counts.mechanical_restoration=0",
            "--expect-summary-value-source",
            "dynamic_godot_report_cli",
            "--expect-summary-value-source-excluded-source",
            "web_godot_delivery",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert (
        "expected_summary_value_source_excluded_sources=web_godot_delivery"
        in lines[0]
    )
    assert (
        "missing_expected_summary_value_source_excluded_sources=web_godot_delivery"
        in lines[0]
    )
    assert (
        "expected summary_value_source_excluded_source web_godot_delivery "
        "was not found" in lines[0]
    )


def test_delivery_acceptance_gate_validator_rejects_summary_value_excluded_source_allowlist(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--summary-only",
            "--expect-summary-value",
            "mechanical_gate_check_counts.mechanical_restoration=0",
            "--expect-summary-value-source",
            "web_godot_delivery",
            "--allow-summary-value-source-excluded-source",
            "web_godot_delivery",
            "--forbid-summary-value-source-excluded-source",
            "dynamic_godot_report_cli",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert (
        "allowed_summary_value_source_excluded_sources=web_godot_delivery"
        in lines[0]
    )
    assert (
        "unexpected_summary_value_source_excluded_sources=dynamic_godot_report_cli"
        in lines[0]
    )
    assert (
        "forbidden_summary_value_source_excluded_sources=dynamic_godot_report_cli"
        in lines[0]
    )
    assert (
        "present_forbidden_summary_value_source_excluded_sources="
        "dynamic_godot_report_cli" in lines[0]
    )
    assert (
        "unexpected summary_value_source_excluded_source "
        "dynamic_godot_report_cli was found" in lines[0]
    )
    assert (
        "forbidden summary_value_source_excluded_source "
        "dynamic_godot_report_cli was found" in lines[0]
    )


def test_delivery_acceptance_gate_validator_rejects_summary_value_matched_source_allowlist(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--summary-only",
            "--expect-summary-value",
            "mechanical_gate_check_counts.mechanical_restoration=0",
            "--expect-summary-value-source",
            "dynamic_godot_report_cli",
            "--allow-summary-value-source-matched-source",
            "web_godot_delivery",
            "--forbid-summary-value-source-matched-source",
            "dynamic_godot_report_cli",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert (
        "allowed_summary_value_source_matched_sources=web_godot_delivery"
        in lines[0]
    )
    assert (
        "unexpected_summary_value_source_matched_sources=dynamic_godot_report_cli"
        in lines[0]
    )
    assert (
        "present_forbidden_summary_value_source_matched_sources="
        "dynamic_godot_report_cli" in lines[0]
    )
    assert (
        "unexpected summary_value_source_matched_source "
        "dynamic_godot_report_cli was found" in lines[0]
    )
    assert (
        "forbidden summary_value_source_matched_source "
        "dynamic_godot_report_cli was found" in lines[0]
    )


def test_delivery_acceptance_gate_validator_requires_summary_value_source_for_source_filters(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    gate_path.write_text("{}", encoding="utf-8")

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-summary-value",
            "mechanical_gate_check_counts.mechanical_restoration=0",
            "--allow-summary-value-source-matched-source",
            "web_godot_delivery",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 2
    assert (
        "--allow-summary-value-source-matched-source and "
        "--forbid-summary-value-source-matched-source and "
        "--allow-summary-value-source-excluded-source and "
        "--forbid-summary-value-source-excluded-source require at least one "
        "--expect-summary-value-source" in validate_result.stderr
    )


def test_delivery_acceptance_gate_validator_fails_on_missing_required_summary_fields_source_scope(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--summary-only",
            "--expect-complete-required-summary-fields-source-scope",
            "web_godot_delivery/godot_load",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    assert payload[
        "expected_complete_required_summary_fields_source_scopes"
    ] == ["web_godot_delivery/godot_load"]
    assert payload[
        "missing_expected_complete_required_summary_fields_source_scopes"
    ] == ["web_godot_delivery/godot_load"]
    assert (
        "expected complete_required_summary_fields_source_scope "
        "web_godot_delivery/godot_load was not found"
    ) in payload["errors"]


def test_delivery_acceptance_gate_validator_can_allow_required_summary_fields_source_scope(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    summary_path = tmp_path / "summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--allow-complete-required-summary-fields-source-scope",
            "web_godot_delivery/godot_load",
            "--summary-output",
            str(summary_path),
            "--format",
            "text",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    payload = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload[
        "allowed_complete_required_summary_fields_source_scopes"
    ] == ["web_godot_delivery/godot_load"]
    assert payload[
        "unexpected_complete_required_summary_fields_source_scopes"
    ] == ["dynamic_godot_report_cli/godot_smoke_motion"]
    assert payload["errors"] == [
        "unexpected complete_required_summary_fields_source_scope "
        "dynamic_godot_report_cli/godot_smoke_motion was found"
    ]
    assert (
        "allowed_complete_required_summary_fields_source_scopes="
        "web_godot_delivery/godot_load" in lines[0]
    )
    assert (
        "unexpected_complete_required_summary_fields_source_scopes="
        "dynamic_godot_report_cli/godot_smoke_motion" in lines[0]
    )


def test_delivery_acceptance_gate_validator_can_forbid_required_summary_fields_source_scope(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    summary_path = tmp_path / "summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--forbid-complete-required-summary-fields-source-scope",
            "dynamic_godot_report_cli/godot_smoke_motion",
            "--summary-output",
            str(summary_path),
            "--format",
            "text",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    payload = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload[
        "forbidden_complete_required_summary_fields_source_scopes"
    ] == ["dynamic_godot_report_cli/godot_smoke_motion"]
    assert payload[
        "present_forbidden_complete_required_summary_fields_source_scopes"
    ] == ["dynamic_godot_report_cli/godot_smoke_motion"]
    assert payload["errors"] == [
        "forbidden complete_required_summary_fields_source_scope "
        "dynamic_godot_report_cli/godot_smoke_motion was found"
    ]
    assert (
        "forbidden_complete_required_summary_fields_source_scopes="
        "dynamic_godot_report_cli/godot_smoke_motion" in lines[0]
    )
    assert (
        "present_forbidden_complete_required_summary_fields_source_scopes="
        "dynamic_godot_report_cli/godot_smoke_motion" in lines[0]
    )


def test_delivery_acceptance_gate_validator_summary_only_output_writes_summary(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    output_path = tmp_path / "validation_summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--summary-only",
            "--output",
            str(output_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    stdout_payload = json.loads(validate_result.stdout)
    output_payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert output_payload == stdout_payload
    assert output_payload["summary_version"] == (
        "delivery_acceptance_gate_validation_summary.v1"
    )
    assert output_payload["status"] == "success"
    assert output_payload["inputs_count"] == 1
    assert output_payload["reason_codes"] == [
        "missing_godot_smoke",
        "static_only",
    ]
    assert "results" not in output_payload


def test_delivery_acceptance_gate_validator_can_force_full_output_shape(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    output_path = tmp_path / "validation_full.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--summary-only",
            "--output",
            str(output_path),
            "--output-shape",
            "full",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    stdout_payload = json.loads(validate_result.stdout)
    output_payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert stdout_payload["summary_version"] == (
        "delivery_acceptance_gate_validation_summary.v1"
    )
    assert stdout_payload["inputs_count"] == 1
    assert output_payload["status"] == "success"
    assert output_payload["input"] == str(gate_path)
    assert output_payload["gate_source_path"] == "."
    assert output_payload["errors"] == []
    assert "summary_version" not in output_payload


def test_delivery_acceptance_gate_validator_can_force_summary_output_shape(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    output_path = tmp_path / "validation_summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--output",
            str(output_path),
            "--output-shape",
            "summary",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    stdout_payload = json.loads(validate_result.stdout)
    output_payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert stdout_payload["status"] == "success"
    assert stdout_payload["input"] == str(gate_path)
    assert stdout_payload["gate_source_path"] == "."
    assert "summary_version" not in stdout_payload
    assert output_payload["summary_version"] == (
        "delivery_acceptance_gate_validation_summary.v1"
    )
    assert output_payload["status"] == "success"
    assert output_payload["inputs_count"] == 1
    assert output_payload["reason_codes"] == [
        "missing_godot_smoke",
        "static_only",
    ]
    assert "results" not in output_payload


def test_delivery_acceptance_gate_validator_rejects_output_shape_without_output(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--output-shape",
            "summary",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 2
    assert validate_result.stdout == ""
    assert "Traceback" not in validate_result.stderr
    assert "--output-shape requires --output" in validate_result.stderr

    explicit_auto_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--output-shape=auto",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert explicit_auto_result.returncode == 2
    assert explicit_auto_result.stdout == ""
    assert "Traceback" not in explicit_auto_result.stderr
    assert "--output-shape requires --output" in explicit_auto_result.stderr


def test_delivery_acceptance_gate_validator_can_fail_on_passed_unknown(
    tmp_path: Path,
) -> None:
    metadata_path = tmp_path / "metadata.json"
    summary_path = tmp_path / "summary.json"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(metadata_path),
            "--ignore-non-gate",
            "--allow-empty",
            "--fail-on-passed-unknown",
            "--summary-output",
            str(summary_path),
            "--format",
            "text",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["status"] == "error"
    assert summary["errors"] == ["passed unknown gates present: 1"]
    assert summary["passed_unknown_count"] == 1
    assert summary["passed_unknown_inputs"] == [str(metadata_path)]
    lines = validate_result.stdout.splitlines()
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "passed=true:0,false:0,unknown:1" in lines[0]
    assert "aggregate_errors=passed unknown gates present: 1" in lines[0]
    assert "status=skipped" in lines[1]


def test_delivery_acceptance_gate_validator_can_expect_reason_codes(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    summary_path = tmp_path / "ci" / "summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-reason-code",
            "missing_godot_smoke",
            "--expect-reason-code",
            "unexpected_live_motion_gap",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["status"] == "error"
    assert payload["expected_reason_codes"] == [
        "missing_godot_smoke",
        "unexpected_live_motion_gap",
    ]
    assert payload["missing_expected_reason_codes"] == [
        "unexpected_live_motion_gap"
    ]
    assert payload["errors"] == [
        "expected reason_code unexpected_live_motion_gap was not found"
    ]
    assert payload["results"][0]["status"] == "success"
    assert summary["expected_reason_codes"] == payload["expected_reason_codes"]
    assert summary["missing_expected_reason_codes"] == [
        "unexpected_live_motion_gap"
    ]
    assert summary["errors"] == payload["errors"]


def test_delivery_acceptance_gate_validator_can_expect_reason_codes_count(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    summary_path = tmp_path / "ci" / "summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-reason-codes-count",
            "1",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["status"] == "error"
    assert payload["expected_reason_codes_count"] == 1
    assert payload["reason_codes_count"] == 2
    assert payload["reason_codes"] == ["missing_godot_smoke", "static_only"]
    assert payload["errors"] == [
        "expected reason_codes_count 1 but found 2"
    ]
    assert summary["expected_reason_codes_count"] == 1
    assert summary["reason_codes_count"] == 2
    assert summary["errors"] == payload["errors"]


def test_delivery_acceptance_gate_validator_can_expect_metadata_counts(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    summary_path = tmp_path / "ci" / "summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-sources-count",
            "2",
            "--expect-verification-scopes-count",
            "2",
            "--expect-acceptance-profiles-count",
            "2",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["status"] == "error"
    assert payload["expected_sources_count"] == 2
    assert payload["expected_verification_scopes_count"] == 2
    assert payload["expected_acceptance_profiles_count"] == 2
    assert payload["sources_count"] == 1
    assert payload["verification_scopes_count"] == 1
    assert payload["acceptance_profiles_count"] == 1
    assert payload["errors"] == [
        "expected sources_count 2 but found 1",
        "expected verification_scopes_count 2 but found 1",
        "expected acceptance_profiles_count 2 but found 1",
    ]
    assert summary["expected_sources_count"] == 2
    assert summary["expected_verification_scopes_count"] == 2
    assert summary["expected_acceptance_profiles_count"] == 2
    assert summary["errors"] == payload["errors"]


def test_delivery_acceptance_gate_validator_can_expect_affected_inputs_count(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    summary_path = tmp_path / "ci" / "summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-affected-inputs-count",
            "2",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["status"] == "error"
    assert payload["expected_affected_inputs_count"] == 2
    assert payload["affected_inputs_count"] == 1
    assert FIXED_PAIR_FIXTURE.name in payload["affected_inputs"][0]
    assert payload["errors"] == [
        "expected affected_inputs_count 2 but found 1"
    ]
    assert summary["expected_affected_inputs_count"] == 2
    assert summary["affected_inputs_count"] == 1
    assert summary["errors"] == payload["errors"]


def test_delivery_acceptance_gate_validator_can_expect_input_preview_counts(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    metadata_path = tmp_path / "metadata.json"
    summary_path = tmp_path / "ci" / "summary.json"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_path),
            "--ignore-non-gate",
            "--expect-failed-inputs-count",
            "1",
            "--expect-skipped-inputs-count",
            "2",
            "--expect-skipped-reasons-count",
            "2",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["expected_failed_inputs_count"] == 1
    assert payload["expected_skipped_inputs_count"] == 2
    assert payload["expected_skipped_reasons_count"] == 2
    assert payload["failed_inputs_count"] == 0
    assert payload["skipped_inputs_count"] == 1
    assert payload["skipped_reasons_count"] == 1
    assert payload["skipped_inputs"] == [str(metadata_path)]
    assert payload["skipped_reasons"] == [
        "not a delivery_acceptance_gate artifact or report"
    ]
    assert payload["errors"] == [
        "expected failed_inputs_count 1 but found 0",
        "expected skipped_inputs_count 2 but found 1",
        "expected skipped_reasons_count 2 but found 1",
    ]
    assert summary["expected_failed_inputs_count"] == 1
    assert summary["expected_skipped_inputs_count"] == 2
    assert summary["expected_skipped_reasons_count"] == 2
    assert summary["failed_inputs_count"] == 0
    assert summary["skipped_inputs_count"] == 1
    assert summary["skipped_reasons_count"] == 1
    assert summary["errors"] == payload["errors"]


def test_delivery_acceptance_gate_validator_can_constrain_skipped_reasons(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    metadata_path = tmp_path / "metadata.json"
    summary_path = tmp_path / "ci" / "summary.json"
    expected_skip_reason = "not a delivery_acceptance_gate artifact or report"
    missing_skip_reason = "manual skip"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_path),
            "--ignore-non-gate",
            "--expect-skipped-reason",
            expected_skip_reason,
            "--expect-skipped-reason",
            missing_skip_reason,
            "--allow-skipped-reason",
            "approved skip",
            "--forbid-skipped-reason",
            expected_skip_reason,
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["expected_skipped_reasons"] == [
        expected_skip_reason,
        missing_skip_reason,
    ]
    assert payload["missing_expected_skipped_reasons"] == [missing_skip_reason]
    assert payload["allowed_skipped_reasons"] == ["approved skip"]
    assert payload["unexpected_skipped_reasons"] == [expected_skip_reason]
    assert payload["forbidden_skipped_reasons"] == [expected_skip_reason]
    assert payload["present_forbidden_skipped_reasons"] == [expected_skip_reason]
    assert payload["errors"] == [
        f"expected skipped_reason {missing_skip_reason} was not found",
        f"unexpected skipped_reason {expected_skip_reason} was found",
        f"forbidden skipped_reason {expected_skip_reason} was found",
    ]
    assert summary["expected_skipped_reasons"] == payload[
        "expected_skipped_reasons"
    ]
    assert summary["missing_expected_skipped_reasons"] == [
        missing_skip_reason
    ]
    assert summary["allowed_skipped_reasons"] == ["approved skip"]
    assert summary["unexpected_skipped_reasons"] == [expected_skip_reason]
    assert summary["forbidden_skipped_reasons"] == [expected_skip_reason]
    assert summary["present_forbidden_skipped_reasons"] == [
        expected_skip_reason
    ]
    assert summary["errors"] == payload["errors"]


def test_delivery_acceptance_gate_validator_can_limit_aggregate_previews(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    metadata_a_path = tmp_path / "metadata-a.json"
    metadata_b_path = tmp_path / "metadata-b.json"
    summary_path = tmp_path / "ci" / "summary.json"
    metadata_a_path.write_text(
        json.dumps({"build_id": "local-a", "notes": ["not a gate"]}),
        encoding="utf-8",
    )
    metadata_b_path.write_text(
        json.dumps({"build_id": "local-b", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_a_path),
            str(metadata_b_path),
            "--ignore-non-gate",
            "--preview-limit",
            "1",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0, validate_result.stderr
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["preview_limit"] == 1
    assert payload["skipped_inputs"] == [str(metadata_a_path)]
    assert payload["skipped_inputs_count"] == 2
    assert payload["skipped_inputs_truncated"] is True
    assert payload["previews_truncated"] is True
    assert "skipped_inputs" in payload["truncated_previews"]
    assert payload["affected_inputs"] == [str(FIXED_PAIR_FIXTURE)]
    assert payload["affected_inputs_count"] == 1
    assert payload["affected_inputs_truncated"] is False
    assert summary["preview_limit"] == 1
    assert summary["skipped_inputs"] == [str(metadata_a_path)]
    assert summary["skipped_inputs_count"] == 2
    assert summary["skipped_inputs_truncated"] is True
    assert summary["previews_truncated"] is True
    assert "skipped_inputs" in summary["truncated_previews"]


def test_delivery_acceptance_gate_validator_text_summary_reports_preview_limit(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    metadata_a_path = tmp_path / "metadata-a.json"
    metadata_b_path = tmp_path / "metadata-b.json"
    metadata_a_path.write_text(
        json.dumps({"build_id": "local-a", "notes": ["not a gate"]}),
        encoding="utf-8",
    )
    metadata_b_path.write_text(
        json.dumps({"build_id": "local-b", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_a_path),
            str(metadata_b_path),
            "--ignore-non-gate",
            "--format",
            "text",
            "--summary-only",
            "--show-inputs",
            "--preview-limit",
            "1",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0, validate_result.stderr
    lines = validate_result.stdout.splitlines()
    assert len(lines) == 1
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "preview_limit=1" in lines[0]
    assert "truncated_previews=" in lines[0]
    assert "skipped_inputs" in lines[0]
    assert f"skipped_inputs={metadata_a_path}" in lines[0]
    assert str(metadata_b_path) not in lines[0]


def test_delivery_acceptance_gate_validator_preview_limit_does_not_truncate_constraints(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    metadata_path = tmp_path / "metadata.json"
    summary_path = tmp_path / "ci" / "summary.json"
    skip_reason = "not a delivery_acceptance_gate artifact or report"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_path),
            "--ignore-non-gate",
            "--preview-limit",
            "0",
            "--expect-skipped-reason",
            skip_reason,
            "--allow-skipped-reason",
            skip_reason,
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0, validate_result.stderr
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["preview_limit"] == 0
    assert payload["skipped_reasons"] == []
    assert payload["skipped_reasons_count"] == 1
    assert payload["skipped_reasons_truncated"] is True
    assert payload["previews_truncated"] is True
    assert "skipped_reasons" in payload["truncated_previews"]
    assert payload["missing_expected_skipped_reasons"] == []
    assert payload["unexpected_skipped_reasons"] == []
    assert payload["errors"] == []
    assert summary["skipped_reasons"] == []
    assert summary["skipped_reasons_count"] == 1
    assert summary["missing_expected_skipped_reasons"] == []
    assert summary["unexpected_skipped_reasons"] == []


def test_delivery_acceptance_gate_validator_can_fail_on_truncated_previews(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    metadata_a_path = tmp_path / "metadata-a.json"
    metadata_b_path = tmp_path / "metadata-b.json"
    summary_path = tmp_path / "ci" / "summary.json"
    metadata_a_path.write_text(
        json.dumps({"build_id": "local-a", "notes": ["not a gate"]}),
        encoding="utf-8",
    )
    metadata_b_path.write_text(
        json.dumps({"build_id": "local-b", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_a_path),
            str(metadata_b_path),
            "--ignore-non-gate",
            "--preview-limit",
            "1",
            "--fail-on-truncated-previews",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["previews_truncated"] is True
    assert "skipped_inputs" in payload["truncated_previews"]
    assert payload["errors"] == [
        "truncated previews present: "
        + ",".join(payload["truncated_previews"])
    ]
    assert summary["truncated_previews"] == payload["truncated_previews"]
    assert summary["errors"] == payload["errors"]


def test_delivery_acceptance_gate_validator_text_summary_reports_truncated_preview_failure(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    metadata_a_path = tmp_path / "metadata-a.json"
    metadata_b_path = tmp_path / "metadata-b.json"
    metadata_a_path.write_text(
        json.dumps({"build_id": "local-a", "notes": ["not a gate"]}),
        encoding="utf-8",
    )
    metadata_b_path.write_text(
        json.dumps({"build_id": "local-b", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_a_path),
            str(metadata_b_path),
            "--ignore-non-gate",
            "--format",
            "text",
            "--summary-only",
            "--show-inputs",
            "--preview-limit",
            "1",
            "--fail-on-truncated-previews",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert len(lines) == 1
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "status=error" in lines[0]
    assert "preview_limit=1" in lines[0]
    assert "truncated_previews=" in lines[0]
    assert "skipped_inputs" in lines[0]
    assert "aggregate_errors=truncated previews present: " in lines[0]


def test_delivery_acceptance_gate_validator_can_constrain_truncated_preview_fields(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    metadata_a_path = tmp_path / "metadata-a.json"
    metadata_b_path = tmp_path / "metadata-b.json"
    metadata_a_path.write_text(
        json.dumps({"build_id": "local-a", "notes": ["not a gate"]}),
        encoding="utf-8",
    )
    metadata_b_path.write_text(
        json.dumps({"build_id": "local-b", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    allowed_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_a_path),
            str(metadata_b_path),
            "--ignore-non-gate",
            "--preview-limit",
            "1",
            "--expect-truncated-previews-count",
            "5",
            "--expect-truncated-preview",
            "skipped_inputs",
            "--allow-truncated-preview",
            "input_paths",
            "--allow-truncated-preview",
            "reason_codes",
            "--allow-truncated-preview",
            "passed_unknown_inputs",
            "--allow-truncated-preview",
            "full_mechanical_gate_unknown_inputs",
            "--allow-truncated-preview",
            "skipped_inputs",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert allowed_result.returncode == 0, allowed_result.stderr
    payload = json.loads(allowed_result.stdout)
    assert payload["truncated_previews_count"] == 5
    assert payload["expected_truncated_previews_count"] == 5
    assert payload["truncated_previews"] == [
        "input_paths",
        "reason_codes",
        "passed_unknown_inputs",
        "full_mechanical_gate_unknown_inputs",
        "skipped_inputs",
    ]
    assert payload["expected_truncated_previews"] == ["skipped_inputs"]
    assert payload["missing_expected_truncated_previews"] == []
    assert payload["allowed_truncated_previews"] == [
        "input_paths",
        "reason_codes",
        "passed_unknown_inputs",
        "full_mechanical_gate_unknown_inputs",
        "skipped_inputs",
    ]
    assert payload["unexpected_truncated_previews"] == []
    assert payload["errors"] == []

    forbidden_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_a_path),
            str(metadata_b_path),
            "--ignore-non-gate",
            "--format",
            "text",
            "--summary-only",
            "--preview-limit",
            "1",
            "--expect-truncated-previews-count",
            "2",
            "--forbid-truncated-preview",
            "reason_codes",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert forbidden_result.returncode == 1
    lines = forbidden_result.stdout.splitlines()
    assert len(lines) == 1
    assert "truncated_previews_count=5" in lines[0]
    assert "expected_truncated_previews_count=2" in lines[0]
    assert "forbidden_truncated_previews=reason_codes" in lines[0]
    assert "present_forbidden_truncated_previews=reason_codes" in lines[0]
    assert "expected truncated_previews_count 2 but found 5" in lines[0]
    assert "forbidden truncated_preview reason_codes was found" in lines[0]


def test_delivery_acceptance_gate_validator_text_summary_reports_skipped_reason_constraints(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    metadata_path = tmp_path / "metadata.json"
    expected_skip_reason = "not a delivery_acceptance_gate artifact or report"
    missing_skip_reason = "manual skip"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_path),
            "--ignore-non-gate",
            "--format",
            "text",
            "--show-skipped-reasons",
            "--expect-skipped-reason",
            expected_skip_reason,
            "--expect-skipped-reason",
            missing_skip_reason,
            "--allow-skipped-reason",
            "approved skip",
            "--forbid-skipped-reason",
            expected_skip_reason,
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert f"skipped_reasons={expected_skip_reason}" in lines[0]
    assert (
        f"expected_skipped_reasons={expected_skip_reason},{missing_skip_reason}"
        in lines[0]
    )
    assert f"missing_expected_skipped_reasons={missing_skip_reason}" in lines[0]
    assert "allowed_skipped_reasons=approved skip" in lines[0]
    assert f"unexpected_skipped_reasons={expected_skip_reason}" in lines[0]
    assert f"forbidden_skipped_reasons={expected_skip_reason}" in lines[0]
    assert f"present_forbidden_skipped_reasons={expected_skip_reason}" in lines[0]
    assert (
        f"aggregate_errors=expected skipped_reason {missing_skip_reason} was not found,"
        in lines[0]
    )
    assert f"unexpected skipped_reason {expected_skip_reason} was found" in lines[0]
    assert f"forbidden skipped_reason {expected_skip_reason} was found" in lines[0]


def test_delivery_acceptance_gate_validator_can_constrain_skipped_inputs(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    metadata_a_path = tmp_path / "metadata-a.json"
    metadata_b_path = tmp_path / "metadata-b.json"
    missing_path = tmp_path / "missing-metadata.json"
    metadata_a_path.write_text(
        json.dumps({"build_id": "local-a", "notes": ["not a gate"]}),
        encoding="utf-8",
    )
    metadata_b_path.write_text(
        json.dumps({"build_id": "local-b", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_a_path),
            str(metadata_b_path),
            "--ignore-non-gate",
            "--format",
            "text",
            "--summary-only",
            "--show-inputs",
            "--preview-limit",
            "1",
            "--expect-skipped-input",
            str(metadata_a_path),
            "--expect-skipped-input",
            str(missing_path),
            "--allow-skipped-input",
            str(metadata_a_path),
            "--forbid-skipped-input",
            str(metadata_a_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert len(lines) == 1
    assert f"expected_skipped_inputs={metadata_a_path},{missing_path}" in lines[0]
    assert f"missing_expected_skipped_inputs={missing_path}" in lines[0]
    assert f"allowed_skipped_inputs={metadata_a_path}" in lines[0]
    assert f"unexpected_skipped_inputs={metadata_b_path}" in lines[0]
    assert f"forbidden_skipped_inputs={metadata_a_path}" in lines[0]
    assert f"present_forbidden_skipped_inputs={metadata_a_path}" in lines[0]
    assert f"expected skipped_input {missing_path} was not found" in lines[0]
    assert f"unexpected skipped_input {metadata_b_path} was found" in lines[0]
    assert f"forbidden skipped_input {metadata_a_path} was found" in lines[0]


def test_delivery_acceptance_gate_validator_can_constrain_failed_inputs(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    invalid_a_path = tmp_path / "invalid-a.json"
    invalid_b_path = tmp_path / "invalid-b.json"
    missing_path = tmp_path / "missing-invalid.json"
    invalid_a_path.write_text("{", encoding="utf-8")
    invalid_b_path.write_text("{", encoding="utf-8")

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(invalid_a_path),
            str(invalid_b_path),
            "--format",
            "text",
            "--summary-only",
            "--show-inputs",
            "--preview-limit",
            "0",
            "--expect-failed-input",
            str(invalid_a_path),
            "--expect-failed-input",
            str(missing_path),
            "--allow-failed-input",
            str(invalid_a_path),
            "--forbid-failed-input",
            str(invalid_a_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert len(lines) == 1
    assert "failed_inputs=none" in lines[0]
    assert f"expected_failed_inputs={invalid_a_path},{missing_path}" in lines[0]
    assert f"missing_expected_failed_inputs={missing_path}" in lines[0]
    assert f"allowed_failed_inputs={invalid_a_path}" in lines[0]
    assert f"unexpected_failed_inputs={invalid_b_path}" in lines[0]
    assert f"forbidden_failed_inputs={invalid_a_path}" in lines[0]
    assert f"present_forbidden_failed_inputs={invalid_a_path}" in lines[0]
    assert f"expected failed_input {missing_path} was not found" in lines[0]
    assert f"unexpected failed_input {invalid_b_path} was found" in lines[0]
    assert f"forbidden failed_input {invalid_a_path} was found" in lines[0]


def test_delivery_acceptance_gate_validator_can_constrain_affected_inputs(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    missing_path = tmp_path / "missing-robot.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--summary-only",
            "--show-inputs",
            "--preview-limit",
            "0",
            "--expect-affected-input",
            str(FIXED_PAIR_FIXTURE),
            "--expect-affected-input",
            str(missing_path),
            "--allow-affected-input",
            str(missing_path),
            "--forbid-affected-input",
            str(FIXED_PAIR_FIXTURE),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert len(lines) == 1
    assert "affected_inputs=none" in lines[0]
    assert f"expected_affected_inputs={FIXED_PAIR_FIXTURE},{missing_path}" in lines[0]
    assert f"missing_expected_affected_inputs={missing_path}" in lines[0]
    assert f"allowed_affected_inputs={missing_path}" in lines[0]
    assert f"unexpected_affected_inputs={FIXED_PAIR_FIXTURE}" in lines[0]
    assert f"forbidden_affected_inputs={FIXED_PAIR_FIXTURE}" in lines[0]
    assert f"present_forbidden_affected_inputs={FIXED_PAIR_FIXTURE}" in lines[0]
    assert f"expected affected_input {missing_path} was not found" in lines[0]
    assert f"unexpected affected_input {FIXED_PAIR_FIXTURE} was found" in lines[0]
    assert f"forbidden affected_input {FIXED_PAIR_FIXTURE} was found" in lines[0]


def test_delivery_acceptance_gate_validator_can_constrain_passed_unknown_inputs(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    metadata_a_path = tmp_path / "metadata-a.json"
    metadata_b_path = tmp_path / "metadata-b.json"
    missing_path = tmp_path / "missing-unknown.json"
    metadata_a_path.write_text(
        json.dumps({"build_id": "local-a", "notes": ["not a gate"]}),
        encoding="utf-8",
    )
    metadata_b_path.write_text(
        json.dumps({"build_id": "local-b", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_a_path),
            str(metadata_b_path),
            "--ignore-non-gate",
            "--format",
            "text",
            "--summary-only",
            "--show-inputs",
            "--preview-limit",
            "1",
            "--expect-passed-unknown-input",
            str(metadata_a_path),
            "--expect-passed-unknown-input",
            str(missing_path),
            "--allow-passed-unknown-input",
            str(metadata_a_path),
            "--forbid-passed-unknown-input",
            str(metadata_a_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert len(lines) == 1
    assert f"passed_unknown_inputs={metadata_a_path}" in lines[0]
    assert (
        f"expected_passed_unknown_inputs={metadata_a_path},{missing_path}"
        in lines[0]
    )
    assert f"missing_expected_passed_unknown_inputs={missing_path}" in lines[0]
    assert f"allowed_passed_unknown_inputs={metadata_a_path}" in lines[0]
    assert f"unexpected_passed_unknown_inputs={metadata_b_path}" in lines[0]
    assert f"forbidden_passed_unknown_inputs={metadata_a_path}" in lines[0]
    assert (
        f"present_forbidden_passed_unknown_inputs={metadata_a_path}" in lines[0]
    )
    assert (
        f"expected passed_unknown_input {missing_path} was not found"
        in lines[0]
    )
    assert (
        f"unexpected passed_unknown_input {metadata_b_path} was found"
        in lines[0]
    )
    assert (
        f"forbidden passed_unknown_input {metadata_a_path} was found"
        in lines[0]
    )


def test_delivery_acceptance_gate_validator_can_constrain_passed_true_and_false_inputs(
    tmp_path: Path,
) -> None:
    true_gate_path = tmp_path / "true-gate.json"
    false_gate_path = tmp_path / "false-gate.json"
    missing_true_path = tmp_path / "missing-true.json"
    missing_false_path = tmp_path / "missing-false.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(true_gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr
    false_payload = json.loads(true_gate_path.read_text(encoding="utf-8"))
    false_payload["passed"] = False
    false_payload["reasons"] = [
        {
            "code": "manual_failure_fixture",
            "message": "Synthetic failed gate for input constraint coverage.",
        }
    ]
    false_gate_path.write_text(
        json.dumps(false_payload, indent=2),
        encoding="utf-8",
    )

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(true_gate_path),
            str(false_gate_path),
            "--format",
            "text",
            "--summary-only",
            "--show-inputs",
            "--preview-limit",
            "0",
            "--expect-passed-true-input",
            str(true_gate_path),
            "--expect-passed-true-input",
            str(missing_true_path),
            "--allow-passed-true-input",
            str(missing_true_path),
            "--forbid-passed-true-input",
            str(true_gate_path),
            "--expect-passed-false-input",
            str(false_gate_path),
            "--expect-passed-false-input",
            str(missing_false_path),
            "--allow-passed-false-input",
            str(missing_false_path),
            "--forbid-passed-false-input",
            str(false_gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert len(lines) == 1
    assert "passed_true_inputs=none" in lines[0]
    assert "passed_false_inputs=none" in lines[0]
    assert (
        f"expected_passed_true_inputs={true_gate_path},{missing_true_path}"
        in lines[0]
    )
    assert f"missing_expected_passed_true_inputs={missing_true_path}" in lines[0]
    assert f"allowed_passed_true_inputs={missing_true_path}" in lines[0]
    assert f"unexpected_passed_true_inputs={true_gate_path}" in lines[0]
    assert f"forbidden_passed_true_inputs={true_gate_path}" in lines[0]
    assert f"present_forbidden_passed_true_inputs={true_gate_path}" in lines[0]
    assert (
        f"expected_passed_false_inputs={false_gate_path},{missing_false_path}"
        in lines[0]
    )
    assert f"missing_expected_passed_false_inputs={missing_false_path}" in lines[0]
    assert f"allowed_passed_false_inputs={missing_false_path}" in lines[0]
    assert f"unexpected_passed_false_inputs={false_gate_path}" in lines[0]
    assert f"forbidden_passed_false_inputs={false_gate_path}" in lines[0]
    assert f"present_forbidden_passed_false_inputs={false_gate_path}" in lines[0]
    assert f"expected passed_true_input {missing_true_path} was not found" in lines[0]
    assert f"unexpected passed_true_input {true_gate_path} was found" in lines[0]
    assert f"forbidden passed_true_input {true_gate_path} was found" in lines[0]
    assert f"expected passed_false_input {missing_false_path} was not found" in lines[0]
    assert f"unexpected passed_false_input {false_gate_path} was found" in lines[0]
    assert f"forbidden passed_false_input {false_gate_path} was found" in lines[0]


def test_delivery_acceptance_gate_validator_can_constrain_validation_inputs(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    metadata_path = tmp_path / "metadata.json"
    missing_path = tmp_path / "missing-input.json"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            str(metadata_path),
            "--ignore-non-gate",
            "--format",
            "text",
            "--summary-only",
            "--show-inputs",
            "--preview-limit",
            "1",
            "--expect-input",
            str(gate_path),
            "--expect-input",
            str(missing_path),
            "--allow-input",
            str(gate_path),
            "--forbid-input",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert len(lines) == 1
    assert f"input_paths={gate_path}" in lines[0]
    assert f"expected_inputs={gate_path},{missing_path}" in lines[0]
    assert f"missing_expected_inputs={missing_path}" in lines[0]
    assert f"allowed_inputs={gate_path}" in lines[0]
    assert f"unexpected_inputs={metadata_path}" in lines[0]
    assert f"forbidden_inputs={gate_path}" in lines[0]
    assert f"present_forbidden_inputs={gate_path}" in lines[0]
    assert f"expected input {missing_path} was not found" in lines[0]
    assert f"unexpected input {metadata_path} was found" in lines[0]
    assert f"forbidden input {gate_path} was found" in lines[0]


def test_delivery_acceptance_gate_validator_text_summary_reports_reason_expectations(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--expect-reason-code",
            "missing_godot_smoke",
            "--expect-reason-code",
            "unexpected_live_motion_gap",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert (
        "expected_reason_codes=missing_godot_smoke,unexpected_live_motion_gap"
        in lines[0]
    )
    assert "missing_expected_reason_codes=unexpected_live_motion_gap" in lines[0]
    assert (
        "aggregate_errors=expected reason_code unexpected_live_motion_gap was not found"
        in lines[0]
    )
    assert "reason_codes=missing_godot_smoke,static_only" in lines[1]


def test_delivery_acceptance_gate_validator_can_allow_only_reason_codes(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    summary_path = tmp_path / "ci" / "summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--allow-reason-code",
            "missing_godot_smoke",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["status"] == "error"
    assert payload["allowed_reason_codes"] == ["missing_godot_smoke"]
    assert payload["unexpected_reason_codes"] == ["static_only"]
    assert payload["errors"] == ["unexpected reason_code static_only was found"]
    assert payload["results"][0]["status"] == "success"
    assert summary["allowed_reason_codes"] == ["missing_godot_smoke"]
    assert summary["unexpected_reason_codes"] == ["static_only"]
    assert summary["errors"] == payload["errors"]


def test_delivery_acceptance_gate_validator_text_summary_reports_reason_allowlist(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--allow-reason-code",
            "missing_godot_smoke",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "allowed_reason_codes=missing_godot_smoke" in lines[0]
    assert "unexpected_reason_codes=static_only" in lines[0]
    assert (
        "aggregate_errors=unexpected reason_code static_only was found"
        in lines[0]
    )
    assert "reason_codes=missing_godot_smoke,static_only" in lines[1]


def test_delivery_acceptance_gate_validator_can_forbid_reason_codes(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    summary_path = tmp_path / "ci" / "summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--forbid-reason-code",
            "static_only",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["status"] == "error"
    assert payload["forbidden_reason_codes"] == ["static_only"]
    assert payload["present_forbidden_reason_codes"] == ["static_only"]
    assert payload["errors"] == ["forbidden reason_code static_only was found"]
    assert payload["results"][0]["status"] == "success"
    assert summary["forbidden_reason_codes"] == ["static_only"]
    assert summary["present_forbidden_reason_codes"] == ["static_only"]
    assert summary["errors"] == payload["errors"]


def test_delivery_acceptance_gate_validator_text_summary_reports_forbidden_reasons(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--forbid-reason-code",
            "static_only",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "forbidden_reason_codes=static_only" in lines[0]
    assert "present_forbidden_reason_codes=static_only" in lines[0]
    assert (
        "aggregate_errors=forbidden reason_code static_only was found"
        in lines[0]
    )
    assert "reason_codes=missing_godot_smoke,static_only" in lines[1]


def test_delivery_acceptance_gate_validator_deduplicates_reason_constraints(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    summary_path = tmp_path / "ci" / "summary.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-reason-code",
            "missing_godot_smoke",
            "--expect-reason-code",
            "missing_godot_smoke",
            "--forbid-reason-code",
            "static_only",
            "--forbid-reason-code",
            "static_only",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["expected_reason_codes"] == ["missing_godot_smoke"]
    assert payload["missing_expected_reason_codes"] == []
    assert payload["forbidden_reason_codes"] == ["static_only"]
    assert payload["present_forbidden_reason_codes"] == ["static_only"]
    assert payload["errors"] == ["forbidden reason_code static_only was found"]
    assert summary["expected_reason_codes"] == ["missing_godot_smoke"]
    assert summary["forbidden_reason_codes"] == ["static_only"]
    assert summary["errors"] == payload["errors"]


def test_delivery_acceptance_gate_validator_text_summary_reports_count_expectations(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--format",
            "text",
            "--expect-inputs-count",
            "2",
            "--expect-success-count",
            "2",
            "--expect-error-count",
            "0",
            "--expect-skipped-count",
            "0",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.splitlines()
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "inputs=1" in lines[0]
    assert "success=1" in lines[0]
    assert "errors=0" in lines[0]
    assert "expected_inputs_count=2" in lines[0]
    assert "expected_success_count=2" in lines[0]
    assert "expected_error_count=0" in lines[0]
    assert "expected_skipped_count=0" in lines[0]
    assert "aggregate_errors=expected inputs_count 2 but found 1" in lines[0]


def test_delivery_acceptance_gate_validator_rejects_non_integer_expected_count(
    tmp_path: Path,
) -> None:
    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(tmp_path / "missing.json"),
            "--expect-inputs-count",
            "many",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 2
    assert validate_result.stdout == ""
    assert "Traceback" not in validate_result.stderr
    assert "value must be an integer >= 0" in validate_result.stderr


def test_delivery_acceptance_gate_validator_rejects_negative_expected_count(
    tmp_path: Path,
) -> None:
    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(tmp_path / "missing.json"),
            "--expect-skipped-count",
            "-1",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 2
    assert validate_result.stdout == ""
    assert "Traceback" not in validate_result.stderr
    assert "value must be an integer >= 0" in validate_result.stderr


def test_delivery_acceptance_gate_validator_rejects_empty_reason_code_constraint(
    tmp_path: Path,
) -> None:
    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(tmp_path / "missing.json"),
            "--forbid-reason-code",
            " ",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 2
    assert validate_result.stdout == ""
    assert "Traceback" not in validate_result.stderr
    assert "value must be non-empty" in validate_result.stderr


def test_delivery_acceptance_gate_validator_rejects_invalid_required_summary_fields_source_scope(
    tmp_path: Path,
) -> None:
    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(tmp_path / "missing.json"),
            "--expect-complete-required-summary-fields-source-scope",
            "godot_smoke_motion",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 2
    assert validate_result.stdout == ""
    assert "Traceback" not in validate_result.stderr
    assert (
        "value must be formatted as non-empty source/scope"
        in validate_result.stderr
    )


def test_delivery_acceptance_gate_validator_rejects_unknown_required_summary_fields_source_scope(
    tmp_path: Path,
) -> None:
    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(tmp_path / "missing.json"),
            "--expect-complete-required-summary-fields-source-scope",
            "unknown_source/unknown_scope",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 2
    assert validate_result.stdout == ""
    assert "Traceback" not in validate_result.stderr
    assert (
        "unsupported --expect-complete-required-summary-fields-source-scope "
        "value(s): unknown_source/unknown_scope"
    ) in validate_result.stderr
    assert "dynamic_godot_report_cli/godot_smoke_motion" in validate_result.stderr
    assert "web_godot_delivery/godot_load" in validate_result.stderr


def test_delivery_acceptance_gate_validator_rejects_unknown_allowed_required_summary_fields_source_scope(
    tmp_path: Path,
) -> None:
    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(tmp_path / "missing.json"),
            "--allow-complete-required-summary-fields-source-scope",
            "unknown_source/unknown_scope",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 2
    assert validate_result.stdout == ""
    assert "Traceback" not in validate_result.stderr
    assert (
        "unsupported --allow-complete-required-summary-fields-source-scope "
        "value(s): unknown_source/unknown_scope"
    ) in validate_result.stderr
    assert "dynamic_godot_report_cli/godot_smoke_motion" in validate_result.stderr
    assert "web_godot_delivery/godot_load" in validate_result.stderr


def test_delivery_acceptance_gate_validator_rejects_unknown_forbidden_required_summary_fields_source_scope(
    tmp_path: Path,
) -> None:
    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(tmp_path / "missing.json"),
            "--forbid-complete-required-summary-fields-source-scope",
            "unknown_source/unknown_scope",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 2
    assert validate_result.stdout == ""
    assert "Traceback" not in validate_result.stderr
    assert (
        "unsupported --forbid-complete-required-summary-fields-source-scope "
        "value(s): unknown_source/unknown_scope"
    ) in validate_result.stderr
    assert "dynamic_godot_report_cli/godot_smoke_motion" in validate_result.stderr
    assert "web_godot_delivery/godot_load" in validate_result.stderr


def test_delivery_acceptance_gate_validator_can_print_contract_schema() -> None:
    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            "--print-contract-schema",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    assert validate_result.stderr == ""
    payload = json.loads(validate_result.stdout)
    assert payload["contract_version"] == "delivery_acceptance_gate.v1"
    assert payload["validation_summary_version"] == (
        workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
    )
    assert payload["validation_summary_required_fields"] == sorted(
        workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_REQUIRED_FIELDS
    )
    assert payload["validation_summary_status_values"] == [
        "error",
        "success",
    ]
    assert payload["validation_summary_metadata_fields"] == sorted(
        workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_METADATA_FIELDS
    )
    assert sorted(payload["validation_summary_metadata_field_types"]) == payload[
        "validation_summary_metadata_fields"
    ]
    assert payload["validation_summary_metadata_field_types"][
        "summary_versions"
    ] == "list[str]"
    assert payload["validation_summary_metadata_field_types"][
        "summary_versions_count"
    ] == "integer"
    assert payload["validation_summary_metadata_field_types"][
        "node_tree_manifest_sidecar_path_map_mismatch_kind_counts"
    ] == "object"
    assert payload["validation_summary_metadata_field_types"][
        "validation_summary_statuses_truncated"
    ] == "boolean"
    assert payload["validation_summary_constraint_fields"] == sorted(
        workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_CONSTRAINT_FIELDS
    )
    assert sorted(payload["validation_summary_constraint_field_types"]) == payload[
        "validation_summary_constraint_fields"
    ]
    assert payload["validation_summary_constraint_field_types"][
        "expected_summary_versions_count"
    ] == "integer|null"
    assert payload["validation_summary_constraint_field_types"][
        "unexpected_validation_summary_statuses"
    ] == "list[str]"
    assert payload["source_values"] == [
        "dynamic_godot_report_cli",
        "web_godot_delivery",
    ]
    assert payload["source_scope_pairs"] == {
        "dynamic_godot_report_cli": "godot_smoke_motion",
        "web_godot_delivery": "godot_load",
    }
    assert payload["source_profile_values"]["web_godot_delivery"] == [
        "web_godot_load"
    ]
    assert payload["scope_level_values"]["godot_load"] == [
        "godot_load_verified",
        "incomplete",
    ]
    assert "run_godot_smoke" in payload["enabled_requirement_values_by_source"][
        "dynamic_godot_report_cli"
    ]
    assert payload["enabled_requirement_values_by_source"]["web_godot_delivery"] == [
        "godot_load",
        "joint_parameter_readback",
        "mechanical_restoration_complete",
        "node_tree_fixed_lock_match",
    ]
    assert "missing_godot_smoke" in payload["reason_code_values"]
    assert "godot_delivery_failed" in payload["reason_code_values"]
    assert "missing_godot_smoke" in payload["reason_code_values_by_source"][
        "dynamic_godot_report_cli"
    ]
    assert "godot_delivery_failed" in payload["reason_code_values_by_source"][
        "web_godot_delivery"
    ]
    assert "control_readback_missing_count" in payload["summary_count_fields"]
    assert "control_readback_missing_count" in payload["summary_fields_by_source"][
        "dynamic_godot_report_cli"
    ]
    assert "control_readback_missing_count" not in payload["summary_fields_by_source"][
        "web_godot_delivery"
    ]
    assert payload["summary_count_map_fields"] == [
        "mechanical_gate_check_counts",
        "node_tree_gate_check_counts",
        "static_node_tree_manifest_path_map_mismatch_kind_counts",
    ]
    assert "delivery_static_only_count" in payload["summary_value_paths"]
    assert (
        "node_tree_gate_check_counts.fixed_lock_mismatch"
        in payload["summary_value_paths"]
    )
    assert (
        "mechanical_gate_check_counts.mechanical_restoration"
        in payload["summary_value_paths"]
    )
    assert "control_readback_missing_count" in payload[
        "summary_value_paths_by_source"
    ]["dynamic_godot_report_cli"]
    assert "control_readback_missing_count" not in payload[
        "summary_value_paths_by_source"
    ]["web_godot_delivery"]
    assert "node_tree_gate_check_counts.fixed_lock_mismatch" in payload[
        "summary_value_paths_by_source"
    ]["web_godot_delivery"]
    assert "mechanical_gate_check_counts.mechanical_restoration" in payload[
        "summary_value_paths_by_source"
    ]["web_godot_delivery"]
    assert "expected_summary_value_sources" in payload[
        "summary_value_source_filter_fields"
    ]
    assert "summary_value_source_matched_sources" in payload[
        "summary_value_source_filter_fields"
    ]
    assert "summary_value_source_excluded_sources" in payload[
        "summary_value_source_filter_fields"
    ]
    assert "unexpected_summary_value_source_matched_sources" in payload[
        "summary_value_source_filter_fields"
    ]
    assert "present_forbidden_summary_value_source_excluded_sources" in payload[
        "summary_value_source_filter_fields"
    ]
    assert sorted(payload["summary_value_source_filter_field_types"]) == payload[
        "summary_value_source_filter_fields"
    ]
    assert payload["summary_value_source_filter_field_types"][
        "actual_expected_summary_values"
    ] == "object"
    assert payload["summary_value_source_filter_field_types"][
        "summary_value_source_matched_count"
    ] == "integer|null"
    assert payload["summary_value_source_filter_field_types"][
        "summary_value_source_matched_sources"
    ] == "list[str]"
    assert payload["complete_required_summary_fields_by_source_scope"][
        "dynamic_godot_report_cli"
    ]["godot_smoke_motion"] == sorted(
        set(
            payload["complete_summary_counts_by_source_scope"][
                "dynamic_godot_report_cli"
            ]["godot_smoke_motion"]["equal_inputs"]
        )
        | set(
            payload["complete_summary_counts_by_source_scope"][
                "dynamic_godot_report_cli"
            ]["godot_smoke_motion"]["zero"]
        )
    )
    assert "live_smoke_count" in payload[
        "complete_required_summary_fields_by_source_scope"
    ]["dynamic_godot_report_cli"]["godot_smoke_motion"]
    assert "smoke_report_written_count" in payload[
        "complete_required_summary_fields_by_source_scope"
    ]["dynamic_godot_report_cli"]["godot_smoke_motion"]
    assert "smoke_report_written_count" not in payload[
        "complete_required_summary_fields_by_source_scope"
    ]["web_godot_delivery"]["godot_load"]
    assert payload["complete_required_summary_fields_source_scopes"] == [
        "dynamic_godot_report_cli/godot_smoke_motion",
        "web_godot_delivery/godot_load",
    ]


def test_delivery_acceptance_gate_validator_still_requires_inputs_without_schema_flag() -> None:
    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 2
    assert validate_result.stdout == ""
    assert "Traceback" not in validate_result.stderr
    assert "the following arguments are required: inputs" in validate_result.stderr


def test_delivery_acceptance_gate_validator_reports_summary_write_failure(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    summary_path = tmp_path / "summary_is_directory"
    summary_path.mkdir()

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    assert "Traceback" not in validate_result.stderr
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "error"
    assert payload["inputs_count"] == 1
    assert payload["success_count"] == 1
    assert payload["error_count"] == 0
    assert payload["errors"]
    assert payload["errors"][0].startswith("unable to write summary output:")
    assert payload["results"][0]["status"] == "success"


def test_delivery_acceptance_gate_validator_reports_full_output_write_failure(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    output_path = tmp_path / "output_is_directory"
    output_path.mkdir()

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--output",
            str(output_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    assert "Traceback" not in validate_result.stderr
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "error"
    assert payload["inputs_count"] == 1
    assert payload["success_count"] == 1
    assert payload["error_count"] == 0
    assert payload["errors"]
    assert payload["errors"][0].startswith("unable to write output:")
    assert payload["results"][0]["status"] == "success"


def test_delivery_acceptance_gate_validator_summary_includes_output_write_failure(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    output_path = tmp_path / "output_is_directory"
    summary_path = tmp_path / "ci" / "summary.json"
    output_path.mkdir()

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--output",
            str(output_path),
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["errors"]
    assert payload["errors"][0].startswith("unable to write output:")
    assert summary["status"] == "error"
    assert summary["success_count"] == 1
    assert summary["error_count"] == 0
    assert summary["errors"]
    assert summary["errors"][0].startswith("unable to write output:")


def test_delivery_acceptance_gate_validator_output_includes_summary_write_failure(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    output_path = tmp_path / "validation.json"
    summary_path = tmp_path / "summary_is_directory"
    summary_path.mkdir()

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--output",
            str(output_path),
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    output = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["errors"]
    assert payload["errors"][0].startswith("unable to write summary output:")
    assert output["status"] == "error"
    assert output["success_count"] == 1
    assert output["error_count"] == 0
    assert output["errors"]
    assert output["errors"][0].startswith("unable to write summary output:")


def test_delivery_acceptance_gate_validator_rejects_same_output_paths(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"
    artifact_path = tmp_path / "validation.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--output",
            str(artifact_path),
            "--summary-output",
            str(artifact_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 2
    assert validate_result.stdout == ""
    assert "Traceback" not in validate_result.stderr
    assert "--output and --summary-output must be different paths" in (
        validate_result.stderr
    )
    assert not artifact_path.exists()


def test_delivery_acceptance_gate_validator_rejects_explicit_output_input(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr
    original_gate = gate_path.read_text(encoding="utf-8")

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 2
    assert validate_result.stdout == ""
    assert "Traceback" not in validate_result.stderr
    assert (
        "--output and --summary-output paths cannot also be explicit inputs"
        in validate_result.stderr
    )
    assert gate_path.read_text(encoding="utf-8") == original_gate


def test_delivery_acceptance_gate_validator_excludes_output_artifacts_from_directory_scan(
    tmp_path: Path,
) -> None:
    artifacts_dir = tmp_path / "artifacts"
    artifacts_dir.mkdir()
    gate_path = artifacts_dir / "gate.json"
    output_path = artifacts_dir / "validation.json"
    summary_path = artifacts_dir / "summary.json"
    output_path.write_text("{not json", encoding="utf-8")
    summary_path.write_text("{not json either", encoding="utf-8")

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--output",
            str(output_path),
            "--summary-output",
            str(summary_path),
            "--expect-expanded-inputs-count",
            "3",
            "--expect-excluded-output-artifacts-count",
            "2",
            "--expect-inputs-count",
            "1",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    payload = json.loads(validate_result.stdout)
    output = json.loads(output_path.read_text(encoding="utf-8"))
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["status"] == "success"
    assert payload["expanded_inputs_count"] == 3
    assert payload["inputs_count"] == 1
    assert payload["success_count"] == 1
    assert payload["expected_expanded_inputs_count"] == 3
    assert payload["expected_excluded_output_artifacts_count"] == 2
    assert payload["expected_inputs_count"] == 1
    assert payload["excluded_output_artifact_inputs"] == [
        str(output_path),
        str(summary_path),
    ]
    assert payload["excluded_output_artifact_inputs_count"] == 2
    assert payload["results"][0]["input"] == str(gate_path)
    assert output["status"] == "success"
    assert output["expanded_inputs_count"] == 3
    assert output["expected_expanded_inputs_count"] == 3
    assert output["expected_excluded_output_artifacts_count"] == 2
    assert output["excluded_output_artifact_inputs_count"] == 2
    assert summary["expanded_inputs_count"] == 3
    assert summary["expected_expanded_inputs_count"] == 3
    assert summary["expected_excluded_output_artifacts_count"] == 2
    assert summary["inputs_count"] == 1
    assert summary["expected_inputs_count"] == 1
    assert summary["success_count"] == 1
    assert summary["error_count"] == 0
    assert summary["excluded_output_artifact_inputs"] == [
        str(output_path),
        str(summary_path),
    ]
    assert summary["excluded_output_artifact_inputs_count"] == 2


def test_delivery_acceptance_gate_validator_text_summary_reports_excluded_outputs(
    tmp_path: Path,
) -> None:
    artifacts_dir = tmp_path / "artifacts"
    artifacts_dir.mkdir()
    gate_path = artifacts_dir / "gate.json"
    output_path = artifacts_dir / "validation.json"
    output_path.write_text("{not json", encoding="utf-8")

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--format",
            "text",
            "--output",
            str(output_path),
            "--expect-expanded-inputs-count",
            "2",
            "--expect-excluded-output-artifacts-count",
            "1",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    lines = validate_result.stdout.splitlines()
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "expanded_inputs=2" in lines[0]
    assert "inputs=1" in lines[0]
    assert "expected_expanded_inputs_count=2" in lines[0]
    assert "expected_excluded_output_artifacts_count=1" in lines[0]
    assert "excluded_output_artifacts=1" in lines[0]
    assert "input=" not in lines[0]
    assert "reason_codes=missing_godot_smoke,static_only" in lines[1]


def test_delivery_acceptance_gate_validator_can_expect_excluded_output_count(
    tmp_path: Path,
) -> None:
    artifacts_dir = tmp_path / "artifacts"
    artifacts_dir.mkdir()
    gate_path = artifacts_dir / "gate.json"
    output_path = artifacts_dir / "validation.json"
    output_path.write_text("{not json", encoding="utf-8")

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--output",
            str(output_path),
            "--expect-excluded-output-artifacts-count",
            "2",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "error"
    assert payload["excluded_output_artifact_inputs_count"] == 1
    assert payload["expected_excluded_output_artifacts_count"] == 2
    assert payload["errors"] == [
        "expected excluded_output_artifacts_count 2 but found 1"
    ]
    assert payload["results"][0]["status"] == "success"


def test_delivery_acceptance_gate_validator_ignore_non_gate_fails_when_empty(
    tmp_path: Path,
) -> None:
    artifacts_dir = tmp_path / "artifacts"
    artifacts_dir.mkdir()
    metadata_path = artifacts_dir / "metadata.json"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--ignore-non-gate",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "error"
    assert payload["inputs_count"] == 1
    assert payload["success_count"] == 0
    assert payload["skipped_count"] == 1
    assert payload["error_count"] == 0
    assert payload["errors"] == [
        "no delivery_acceptance_gate artifacts or reports found"
    ]
    assert payload["results"][0]["status"] == "skipped"


def test_delivery_acceptance_gate_validator_single_aggregate_error_has_text_summary(
    tmp_path: Path,
) -> None:
    metadata_path = tmp_path / "metadata.json"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(metadata_path),
            "--ignore-non-gate",
            "--format",
            "text",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.strip().splitlines()
    assert len(lines) == 2
    assert lines[0] == (
        "delivery_acceptance_gate validation summary "
        "status=error inputs=1 success=0 skipped=1 errors=0 "
        "aggregate_errors=no delivery_acceptance_gate artifacts or reports found "
        "require_passed=false"
    )
    assert "status=skipped" in lines[1]
    assert (
        "skip_reason=not a delivery_acceptance_gate artifact or report" in lines[1]
    )


def test_delivery_acceptance_gate_validator_ignore_non_gate_can_allow_empty(
    tmp_path: Path,
) -> None:
    artifacts_dir = tmp_path / "artifacts"
    artifacts_dir.mkdir()
    metadata_path = artifacts_dir / "metadata.json"
    metadata_path.write_text(
        json.dumps({"build_id": "local", "notes": ["not a gate"]}),
        encoding="utf-8",
    )

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(artifacts_dir),
            "--ignore-non-gate",
            "--allow-empty",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "skipped"
    assert payload["skip_reason"] == "not a delivery_acceptance_gate artifact or report"


def test_delivery_acceptance_gate_validator_multi_input_fails_on_any_error(
    tmp_path: Path,
) -> None:
    passing_gate_path = tmp_path / "passing_gate.json"
    failing_gate_path = tmp_path / "failing_gate.json"

    passing_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(passing_gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert passing_result.returncode == 0, passing_result.stderr

    failing_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--require-full-mechanical-restoration-gate",
            "--gate-output",
            str(failing_gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert failing_result.returncode == 1

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(passing_gate_path),
            str(failing_gate_path),
            "--require-passed",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "error"
    assert payload["inputs_count"] == 2
    assert payload["success_count"] == 1
    assert payload["error_count"] == 1
    assert payload["results"][0]["status"] == "success"
    assert payload["results"][1]["status"] == "error"
    assert payload["results"][1]["errors"] == [
        "delivery_acceptance_gate passed must be true"
    ]


def test_delivery_acceptance_gate_validator_reports_invalid_json(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "broken_gate.json"
    gate_path.write_text('{"delivery_acceptance_gate": ', encoding="utf-8")

    validate_result = subprocess.run(
        [sys.executable, str(GATE_VALIDATOR_TOOL), str(gate_path)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    assert "Traceback" not in validate_result.stderr
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "error"
    assert payload["input"] == str(gate_path)
    assert payload["gate_source_path"] is None
    assert payload["errors"][0].startswith("invalid JSON: line 1 column")


def test_delivery_acceptance_gate_validator_reports_unreadable_input(
    tmp_path: Path,
) -> None:
    missing_path = tmp_path / "missing_gate.json"

    validate_result = subprocess.run(
        [sys.executable, str(GATE_VALIDATOR_TOOL), str(missing_path)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    assert "Traceback" not in validate_result.stderr
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "error"
    assert payload["input"] == str(missing_path)
    assert payload["gate_source_path"] is None
    assert payload["errors"][0].startswith("unable to read input:")


def test_delivery_acceptance_gate_validator_multi_input_text_has_summary(
    tmp_path: Path,
) -> None:
    passing_gate_path = tmp_path / "passing_gate.json"
    failing_gate_path = tmp_path / "failing_gate.json"

    passing_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(passing_gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert passing_result.returncode == 0, passing_result.stderr

    failing_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--require-full-mechanical-restoration-gate",
            "--gate-output",
            str(failing_gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert failing_result.returncode == 1

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(passing_gate_path),
            str(failing_gate_path),
            "--require-passed",
            "--format",
            "text",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    lines = validate_result.stdout.strip().splitlines()
    assert len(lines) == 3
    assert lines[0] == (
        "delivery_acceptance_gate validation summary "
        "status=error inputs=2 success=1 errors=1 require_passed=true"
    )
    assert "status=success" in lines[1]
    assert f"affected_inputs={FIXED_PAIR_FIXTURE}" in lines[1]
    assert "status=error" in lines[2]
    assert "requirements=full_mechanical_restoration_gate" in lines[2]


def test_delivery_acceptance_gate_validator_rejects_malformed_gate(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "bad_gate.json"
    gate_path.write_text(
        json.dumps(
            {
                "contract_version": "old",
                "source": "",
                "acceptance_requirements": {"unknown_gate": True},
                "reason_details": [],
                "summary_counts": [],
            }
        ),
        encoding="utf-8",
    )

    validate_result = subprocess.run(
        [sys.executable, str(GATE_VALIDATOR_TOOL), str(gate_path)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    assert payload["status"] == "error"
    assert payload["gate_source_path"] == "."
    assert payload["contract_version"] == "old"
    assert payload["passed"] is None
    assert payload["exit_code"] is None
    assert payload["reason_codes"] == []
    assert payload["reasons_count"] == 0
    assert payload["enabled_requirements"] == ["unknown_gate"]
    assert payload["affected_inputs"] == []
    assert payload["affected_inputs_count"] == 0
    assert payload["affected_inputs_truncated"] is False
    assert payload["summary_counts"] == {}
    assert payload["require_passed"] is False
    assert any("missing required fields" in error for error in payload["errors"])
    assert any("contract_version must be" in error for error in payload["errors"])
    assert any("unknown fields" in error for error in payload["errors"])


def test_report_acceptance_requirements_expand_full_mechanical_gate() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_report", REPORT_TOOL)
    report_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(report_tool)

    requirements = report_tool._build_acceptance_requirements(
        _acceptance_args(
            run_godot_smoke=True,
            require_godot_verified_acceptance=True,
            require_full_mechanical_restoration_gate=True,
            fail_on_full_mechanical_restoration=True,
        )
    )

    assert requirements == _acceptance_requirements(
        run_godot_smoke=True,
        godot_verified_acceptance=True,
        full_mechanical_restoration_gate=True,
        full_mechanical_restoration_smoke_gate=True,
        mechanical_restoration_complete=True,
        joint_parameter_readback=True,
        control_parameter_readback=True,
        full_node_tree_restoration=True,
        node_tree_complete=True,
        node_tree_class_match=True,
        node_tree_parameters_applied=True,
        node_tree_transform_match=True,
        node_tree_physical_match=True,
        node_tree_fixed_lock_match=True,
    )


def test_dynamic_robot_generation_report_tool_builds_batch_report(
    tmp_path: Path,
) -> None:
    output_path = tmp_path / "batch_report.json"
    manifest_output_dir = tmp_path / "node_tree_manifests"

    result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(BIPED_TEMPLATE),
            str(QUADRUPED_TEMPLATE),
            "--output",
            str(output_path),
            "--static-node-tree-manifest-dir",
            str(manifest_output_dir),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    report = json.loads(output_path.read_text(encoding="utf-8"))
    biped_manifest_path = (
        manifest_output_dir / "000_biped_basic.node_tree_manifest.json"
    )
    quadruped_manifest_path = (
        manifest_output_dir / "001_quadruped_dog.node_tree_manifest.json"
    )
    biped_manifest = json.loads(biped_manifest_path.read_text(encoding="utf-8"))
    quadruped_manifest = json.loads(quadruped_manifest_path.read_text(encoding="utf-8"))
    assert report["status"] == "success"
    assert report["delivery_acceptance_gate"] == {
        "contract_version": "delivery_acceptance_gate.v1",
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": False,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": _acceptance_requirements(),
        "passed": True,
        "exit_code": 0,
        "level": "static_only",
        "complete": False,
        "reasons": [
            "2 robot(s) were not run through Godot smoke",
            "2 robot(s) only passed static normalization",
        ],
        "reason_codes": [
            "missing_godot_smoke",
            "static_only",
        ],
        "reason_details": [
            {
                "code": "missing_godot_smoke",
                "count": 2,
                "message": "2 robot(s) were not run through Godot smoke",
                "inputs": [str(BIPED_TEMPLATE), str(QUADRUPED_TEMPLATE)],
                "inputs_count": 2,
                "inputs_truncated": False,
            },
            {
                "code": "static_only",
                "count": 2,
                "message": "2 robot(s) only passed static normalization",
                "inputs": [str(BIPED_TEMPLATE), str(QUADRUPED_TEMPLATE)],
                "inputs_count": 2,
                "inputs_truncated": False,
            },
        ],
        "summary_counts": {
            "inputs_count": 2,
            "success_count": 2,
            "error_count": 0,
            "live_smoke_count": 0,
            "smoke_report_written_count": 0,
            "smoke_report_missing_count": 0,
            "smoke_report_read_error_count": 0,
            "delivery_godot_verified_count": 0,
            "delivery_static_only_count": 2,
            "delivery_unverified_count": 0,
            "delivery_dynamic_generation_count": 2,
            "delivery_complete_count": 2,
            "delivery_incomplete_count": 0,
            "delivery_parameters_incomplete_count": 0,
            "fixed_lock_checked_count": 0,
            "fixed_lock_mismatch_count": 0,
            "control_configured_count": 0,
            "control_readback_checked_count": 0,
            "control_readback_missing_count": 0,
            "node_tree_fixed_lock_checked_count": 0,
            "node_tree_fixed_lock_mismatch_count": 0,
            "node_tree_fixed_locks_complete_count": 0,
            "node_tree_fixed_locks_incomplete_count": 0,
            "node_tree_gate_enabled_count": 0,
            "node_tree_full_restoration_required_count": 0,
            "node_tree_full_restoration_not_required_count": 0,
            "node_tree_gate_check_counts": {
                "incomplete_node_tree": 0,
                "class_mismatch": 0,
                "missing_parameters": 0,
                "transform_mismatch": 0,
                "physical_mismatch": 0,
                "fixed_lock_mismatch": 0,
            },
            "static_topology_complete_count": 2,
            "static_topology_incomplete_count": 0,
            "static_topology_disconnected_parts_count": 0,
            "static_topology_unreachable_parts_count": 0,
            "static_topology_duplicate_child_endpoint_count": 0,
            "static_topology_cycle_count": 0,
            "static_node_tree_manifest_count": 2,
            "static_node_tree_manifest_valid_count": 2,
            "static_node_tree_manifest_invalid_count": 0,
            "static_node_tree_manifest_error_count": 0,
            "static_node_tree_manifest_output_count": 2,
            "static_node_tree_manifest_path_map_mismatch_count": 0,
            "static_node_tree_manifest_path_map_mismatch_kind_counts": {},
            "static_node_tree_parts_planned_count": 14,
            "static_node_tree_joints_planned_count": 12,
            "static_node_tree_parameterized_joints_count": 12,
            "static_node_tree_complete_count": 2,
            "static_node_tree_incomplete_count": 0,
            "static_node_tree_endpoint_paths_complete_count": 2,
            "static_node_tree_endpoint_paths_incomplete_count": 0,
            "static_node_tree_missing_endpoint_parts_count": 0,
            "static_node_tree_missing_endpoint_connections_count": 0,
            "static_node_tree_parameters_complete_count": 2,
            "static_node_tree_parameters_incomplete_count": 0,
            "mechanical_gate_enabled_count": 0,
            "full_mechanical_restoration_required_count": 0,
            "full_mechanical_restoration_not_required_count": 0,
            "mechanical_gate_check_counts": {
                "mechanical_restoration": 0,
                "joint_parameter_readback": 0,
                "control_parameter_readback": 0,
                "full_node_tree_restoration": 0,
            },
            "mechanical_behavior_evidence_count": 0,
            "mechanical_behavior_complete_count": 0,
            "mechanical_behavior_incomplete_count": 0,
            "mechanical_behavior_residual_risk_count": 0,
            "mechanical_behavior_threshold_failure_count": 0,
            "mechanical_behavior_center_of_mass_available_count": 0,
            "mechanical_behavior_contact_state_available_count": 0,
            "mechanical_behavior_step_trace_artifact_count": 0,
            "failure_reasons_count": 0,
        },
    }
    assert report["batch_summary"]["inputs_count"] == 2
    assert report["batch_summary"]["success_count"] == 2
    assert report["batch_summary"]["error_count"] == 0
    assert report["batch_summary"]["live_smoke_count"] == 0
    assert report["batch_summary"]["delivery_godot_verified_count"] == 0
    assert report["batch_summary"]["delivery_static_only_count"] == 2
    assert report["batch_summary"]["delivery_unverified_count"] == 0
    assert report["batch_summary"]["delivery_dynamic_generation_count"] == 2
    assert report["batch_summary"]["delivery_acceptance_complete"] is False
    assert report["batch_summary"]["delivery_acceptance_level"] == "static_only"
    assert report["batch_summary"]["delivery_acceptance_reasons"] == [
        "2 robot(s) were not run through Godot smoke",
        "2 robot(s) only passed static normalization",
    ]
    assert report["batch_summary"]["delivery_acceptance_reason_codes"] == [
        "missing_godot_smoke",
        "static_only",
    ]
    assert report["batch_summary"]["delivery_acceptance_reason_details"] == [
        {
            "code": "missing_godot_smoke",
            "count": 2,
            "message": "2 robot(s) were not run through Godot smoke",
            "inputs": [str(BIPED_TEMPLATE), str(QUADRUPED_TEMPLATE)],
            "inputs_count": 2,
            "inputs_truncated": False,
        },
        {
            "code": "static_only",
            "count": 2,
            "message": "2 robot(s) only passed static normalization",
            "inputs": [str(BIPED_TEMPLATE), str(QUADRUPED_TEMPLATE)],
            "inputs_count": 2,
            "inputs_truncated": False,
        },
    ]
    assert report["batch_summary"]["delivery_complete_count"] == 2
    assert report["batch_summary"]["delivery_incomplete_count"] == 0
    assert report["batch_summary"]["delivery_parameters_incomplete_count"] == 0
    assert report["batch_summary"]["parameter_mismatch_count"] == 0
    assert report["batch_summary"]["restoration_incomplete_count"] == 0
    assert report["batch_summary"]["node_tree_incomplete_count"] == 0
    assert report["batch_summary"]["node_tree_missing_parts_count"] == 0
    assert report["batch_summary"]["node_tree_missing_joints_count"] == 0
    assert report["batch_summary"]["static_topology_complete_count"] == 2
    assert report["batch_summary"]["static_topology_incomplete_count"] == 0
    assert report["batch_summary"]["static_topology_disconnected_parts_count"] == 0
    assert report["batch_summary"]["static_topology_unreachable_parts_count"] == 0
    assert report["batch_summary"]["static_topology_duplicate_child_endpoint_count"] == 0
    assert report["batch_summary"]["static_topology_cycle_count"] == 0
    assert report["batch_summary"]["static_topology_root_parts"] == ["torso_1"]
    assert report["batch_summary"]["static_node_tree_manifest_count"] == 2
    assert report["batch_summary"]["static_node_tree_manifest_error_count"] == 0
    assert (
        report["batch_summary"]["static_node_tree_manifest_path_map_mismatch_count"]
        == 0
    )
    assert (
        report["batch_summary"][
            "static_node_tree_manifest_path_map_mismatch_kind_counts"
        ]
        == {}
    )
    assert report["batch_summary"]["static_node_tree_manifest_path_map_mismatches"] == []
    assert report["batch_summary"]["static_node_tree_manifest_output_count"] == 2
    assert report["batch_summary"]["static_node_tree_manifest_outputs"] == [
        str(biped_manifest_path),
        str(quadruped_manifest_path),
    ]
    assert report["batch_summary"]["static_node_tree_parts_planned_count"] == 14
    assert report["batch_summary"]["static_node_tree_joints_planned_count"] == 12
    assert report["batch_summary"]["static_node_tree_parameterized_joints_count"] == 12
    assert report["batch_summary"]["static_node_tree_complete_count"] == 2
    assert report["batch_summary"]["static_node_tree_incomplete_count"] == 0
    assert report["batch_summary"]["static_node_tree_endpoint_paths_complete_count"] == 2
    assert report["batch_summary"]["static_node_tree_endpoint_paths_incomplete_count"] == 0
    assert report["batch_summary"]["static_node_tree_missing_endpoint_parts_count"] == 0
    assert report["batch_summary"]["static_node_tree_missing_endpoint_connections_count"] == 0
    assert report["batch_summary"]["static_node_tree_missing_endpoint_part_ids"] == []
    assert (
        report["batch_summary"]["static_node_tree_missing_endpoint_connection_names"]
        == []
    )
    assert report["batch_summary"]["static_node_tree_missing_endpoint_details"] == []
    assert report["batch_summary"]["static_node_tree_parameters_complete_count"] == 2
    assert report["batch_summary"]["static_node_tree_parameters_incomplete_count"] == 0
    assert report["batch_summary"]["failure_reasons_count"] == 0
    assert report["batch_summary"]["robots"][0]["godot_smoke_returncode"] is None
    assert report["batch_summary"]["robots"][0]["delivery_source"] == "static_normalization"
    assert report["batch_summary"]["robots"][0]["delivery_dynamic_robot_generation"] is True
    assert report["batch_summary"]["robots"][0]["delivery_complete"] is True
    assert report["batch_summary"]["robots"][0]["topology_root_part"] == "torso_1"
    assert report["batch_summary"]["robots"][0]["topology_complete_tree"] is True
    assert report["batch_summary"]["robots"][0]["topology_reachable_parts_count"] == 5
    assert report["batch_summary"]["robots"][0]["topology_disconnected_parts"] == []
    assert report["batch_summary"]["robots"][0]["topology_unreachable_parts"] == []
    assert report["batch_summary"]["robots"][0]["topology_duplicate_child_endpoints"] == []
    assert report["batch_summary"]["robots"][0]["topology_cycle"] == []
    assert report["batch_summary"]["robots"][0]["static_node_tree_manifest_output"] == (
        str(biped_manifest_path)
    )
    assert report["batch_summary"]["robots"][0]["static_node_tree_manifest_version"] == (
        "godot_node_tree_manifest.v1"
    )
    assert (
        report["batch_summary"]["robots"][0]["static_node_tree_manifest_error_count"]
        == 0
    )
    assert (
        report["batch_summary"]["robots"][0][
            "static_node_tree_manifest_path_map_mismatch_count"
        ]
        == 0
    )
    assert (
        report["batch_summary"]["robots"][0][
            "static_node_tree_manifest_path_map_mismatch_kind_counts"
        ]
        == {}
    )
    assert (
        report["batch_summary"]["robots"][0][
            "static_node_tree_manifest_path_map_mismatches"
        ]
        == []
    )
    assert report["batch_summary"]["robots"][0]["static_node_tree_parts_planned_count"] == 5
    assert report["batch_summary"]["robots"][0]["static_node_tree_joints_planned_count"] == 4
    assert (
        report["batch_summary"]["robots"][0][
            "static_node_tree_parameterized_joints_count"
        ]
        == 4
    )
    assert report["batch_summary"]["robots"][0]["static_node_tree_complete"] is True
    assert (
        report["batch_summary"]["robots"][0][
            "static_node_tree_endpoint_paths_complete"
        ]
        is True
    )
    assert report["batch_summary"]["robots"][0]["static_node_tree_missing_endpoint_part_ids"] == []
    assert (
        report["batch_summary"]["robots"][0][
            "static_node_tree_missing_endpoint_connection_names"
        ]
        == []
    )
    assert (
        report["batch_summary"]["robots"][0][
            "static_node_tree_missing_endpoint_details"
        ]
        == []
    )
    assert report["batch_summary"]["robots"][0]["static_node_tree_parameters_complete"] is True
    assert report["batch_summary"]["robots"][0]["delivery_expected_parts"] == 5
    assert report["batch_summary"]["robots"][0]["delivery_expected_joints"] == 4
    assert report["batch_summary"]["robots"][0]["delivery_joints_complete"] is True
    assert report["batch_summary"]["robots"][0]["delivery_part_nodes_count"] is None
    assert report["batch_summary"]["robots"][0]["delivery_joint_nodes_count"] is None
    assert report["batch_summary"]["robots"][0]["parameter_mismatch_count"] is None
    assert report["batch_summary"]["robots"][0]["node_tree_complete"] is None
    assert report["batch_summary"]["robots"][0]["failure_reasons"] == []
    assert report["batch_summary"]["robots"][1]["static_node_tree_manifest_output"] == (
        str(quadruped_manifest_path)
    )
    assert [item["static"]["parts_count"] for item in report["reports"]] == [5, 9]
    assert [item["static"]["connections_count"] for item in report["reports"]] == [
        4,
        8,
    ]
    assert biped_manifest == report["reports"][0]["static"]["node_tree_manifest"]
    assert quadruped_manifest == report["reports"][1]["static"]["node_tree_manifest"]


def test_dynamic_robot_generation_report_tool_can_require_godot_acceptance(
    tmp_path: Path,
) -> None:
    output_path = tmp_path / "batch_report.json"
    gate_output_path = tmp_path / "batch_gate.json"

    result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(BIPED_TEMPLATE),
            str(QUADRUPED_TEMPLATE),
            "--require-godot-verified-acceptance",
            "--output",
            str(output_path),
            "--gate-output",
            str(gate_output_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    assert (
        "delivery_acceptance_gate failed "
        "contract_version=delivery_acceptance_gate.v1 "
        "source=dynamic_godot_report_cli "
        "verification_scope=godot_smoke_motion "
        "acceptance_profile=custom "
        "requirements=godot_verified_acceptance "
        "exit_code=1 required=true "
        "level=static_only complete=false "
        "counts=inputs:2,errors:0,live:0,smokereports:0/0/0,verified:0,"
        "static:2,staticmanifest:2/0,fixed:0/0,control:0/0/0,treefixed:0/0,"
        "treefixedok:0/0,treegate:0/0,mechgate:0/0,mechbehavior:0/0,"
        "failures:0 "
        "checks=none "
        "topology=complete:2/2,incomplete:0,disconnected:0,unreachable:0,"
        "duplicates:0,cycles:0,roots:torso_1 "
        "reason_codes=missing_godot_smoke,static_only "
        f"affected_inputs={BIPED_TEMPLATE},{QUADRUPED_TEMPLATE}"
    ) in result.stderr
    report = json.loads(output_path.read_text(encoding="utf-8"))
    gate_report = json.loads(gate_output_path.read_text(encoding="utf-8"))
    assert report["status"] == "success"
    assert report["delivery_acceptance_gate"] == {
        "contract_version": "delivery_acceptance_gate.v1",
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": _acceptance_requirements(
            godot_verified_acceptance=True
        ),
        "passed": False,
        "exit_code": 1,
        "level": "static_only",
        "complete": False,
        "reasons": [
            "2 robot(s) were not run through Godot smoke",
            "2 robot(s) only passed static normalization",
        ],
        "reason_codes": [
            "missing_godot_smoke",
            "static_only",
        ],
        "reason_details": [
            {
                "code": "missing_godot_smoke",
                "count": 2,
                "message": "2 robot(s) were not run through Godot smoke",
                "inputs": [str(BIPED_TEMPLATE), str(QUADRUPED_TEMPLATE)],
                "inputs_count": 2,
                "inputs_truncated": False,
            },
            {
                "code": "static_only",
                "count": 2,
                "message": "2 robot(s) only passed static normalization",
                "inputs": [str(BIPED_TEMPLATE), str(QUADRUPED_TEMPLATE)],
                "inputs_count": 2,
                "inputs_truncated": False,
            },
        ],
        "summary_counts": {
            "inputs_count": 2,
            "success_count": 2,
            "error_count": 0,
            "live_smoke_count": 0,
            "smoke_report_written_count": 0,
            "smoke_report_missing_count": 0,
            "smoke_report_read_error_count": 0,
            "delivery_godot_verified_count": 0,
            "delivery_static_only_count": 2,
            "delivery_unverified_count": 0,
            "delivery_dynamic_generation_count": 2,
            "delivery_complete_count": 2,
            "delivery_incomplete_count": 0,
            "delivery_parameters_incomplete_count": 0,
            "fixed_lock_checked_count": 0,
            "fixed_lock_mismatch_count": 0,
            "control_configured_count": 0,
            "control_readback_checked_count": 0,
            "control_readback_missing_count": 0,
            "node_tree_fixed_lock_checked_count": 0,
            "node_tree_fixed_lock_mismatch_count": 0,
            "node_tree_fixed_locks_complete_count": 0,
            "node_tree_fixed_locks_incomplete_count": 0,
            "node_tree_gate_enabled_count": 0,
            "node_tree_full_restoration_required_count": 0,
            "node_tree_full_restoration_not_required_count": 0,
            "node_tree_gate_check_counts": {
                "incomplete_node_tree": 0,
                "class_mismatch": 0,
                "missing_parameters": 0,
                "transform_mismatch": 0,
                "physical_mismatch": 0,
                "fixed_lock_mismatch": 0,
            },
            "static_topology_complete_count": 2,
            "static_topology_incomplete_count": 0,
            "static_topology_disconnected_parts_count": 0,
            "static_topology_unreachable_parts_count": 0,
            "static_topology_duplicate_child_endpoint_count": 0,
            "static_topology_cycle_count": 0,
            "static_node_tree_manifest_count": 2,
            "static_node_tree_manifest_valid_count": 2,
            "static_node_tree_manifest_invalid_count": 0,
            "static_node_tree_manifest_error_count": 0,
            "static_node_tree_manifest_output_count": 0,
            "static_node_tree_manifest_path_map_mismatch_count": 0,
            "static_node_tree_manifest_path_map_mismatch_kind_counts": {},
            "static_node_tree_parts_planned_count": 14,
            "static_node_tree_joints_planned_count": 12,
            "static_node_tree_parameterized_joints_count": 12,
            "static_node_tree_complete_count": 2,
            "static_node_tree_incomplete_count": 0,
            "static_node_tree_endpoint_paths_complete_count": 2,
            "static_node_tree_endpoint_paths_incomplete_count": 0,
            "static_node_tree_missing_endpoint_parts_count": 0,
            "static_node_tree_missing_endpoint_connections_count": 0,
            "static_node_tree_parameters_complete_count": 2,
            "static_node_tree_parameters_incomplete_count": 0,
            "mechanical_gate_enabled_count": 0,
            "full_mechanical_restoration_required_count": 0,
            "full_mechanical_restoration_not_required_count": 0,
            "mechanical_gate_check_counts": {
                "mechanical_restoration": 0,
                "joint_parameter_readback": 0,
                "control_parameter_readback": 0,
                "full_node_tree_restoration": 0,
            },
            "mechanical_behavior_evidence_count": 0,
            "mechanical_behavior_complete_count": 0,
            "mechanical_behavior_incomplete_count": 0,
            "mechanical_behavior_residual_risk_count": 0,
            "mechanical_behavior_threshold_failure_count": 0,
            "mechanical_behavior_center_of_mass_available_count": 0,
            "mechanical_behavior_contact_state_available_count": 0,
            "mechanical_behavior_step_trace_artifact_count": 0,
            "failure_reasons_count": 0,
        },
    }
    assert gate_report == report["delivery_acceptance_gate"]
    assert "batch_summary" not in gate_report
    assert "reports" not in gate_report
    assert report["batch_summary"]["delivery_acceptance_complete"] is False
    assert report["batch_summary"]["delivery_acceptance_level"] == "static_only"
    assert report["batch_summary"]["delivery_acceptance_reasons"] == [
        "2 robot(s) were not run through Godot smoke",
        "2 robot(s) only passed static normalization",
    ]
    assert report["batch_summary"]["delivery_acceptance_reason_codes"] == [
        "missing_godot_smoke",
        "static_only",
    ]


def test_batch_summary_aggregates_live_gate_results() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_report", REPORT_TOOL)
    report_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(report_tool)

    reports = [
        {
            "status": "success",
            "static": {
                "input": "biped.json",
                "robot_name": "biped",
                "parts_count": 3,
                "connections_count": 2,
                "errors": [],
            },
            "delivery_contract_preview": {
                "source": "godot_smoke",
                "dynamic_robot_generation": True,
                "complete": True,
                "expected_parts": 3,
                "parts_complete": True,
                "expected_joints": 2,
                "failed_joints": 0,
                "joints_complete": True,
                "parameterized_joints": 2,
                "parameters_complete": True,
                "part_nodes_count": 3,
                "joint_nodes_count": 2,
            },
            "godot_smoke": {
                "returncode": 0,
                "report_summary": {
                    "joint_parameter_consistency_summary": {
                        "mismatch_count": 0,
                        "fixed_lock_checked_count": 1,
                        "fixed_lock_mismatch_count": 0,
                        "complete": True,
                    },
                    "joint_control_consistency_summary": {
                        "configured_count": 2,
                        "checked_count": 2,
                        "missing_count": 0,
                        "mismatch_count": 0,
                        "complete": True,
                    },
                    "joint_control_summary": {
                        "targeted_count": 2,
                        "nonzero_targets_under_min": False,
                    },
                    "action_target_consistency_summary": {
                        "mismatch_count": 0,
                        "unknown_target_count": 0,
                        "invalid_target_count": 0,
                        "complete": True,
                    },
                    "action_sequence_target_consistency_summary": {
                        "mismatch_count": 0,
                        "unknown_target_count": 0,
                        "invalid_target_count": 0,
                        "steps": 1,
                        "complete": True,
                    },
                    "action_target_coverage_summary": {
                        "coverage_ratio": 1.0,
                        "coverage_under_min": False,
                        "complete": True,
                    },
                    "control_action_coverage_summary": {
                        "coverage_ratio": 1.0,
                        "coverage_under_min": False,
                        "complete": True,
                    },
                    "joint_motion_summary": {
                        "max_abs_relative_angle_delta": 0.08,
                        "angle_delta_under_min": False,
                        "max_abs_relative_angle_range": 0.12,
                        "angle_range_under_min": False,
                        "moving_joint_coverage_ratio": 1.0,
                        "moving_joint_coverage_under_min": False,
                        "commanded_joint_response_ratio": 1.0,
                        "commanded_joint_response_under_min": False,
                        "commanded_static_joints": [],
                        "commanded_joint_response_details": [
                            {
                                "joint": "hip_left",
                                "max_abs_target": 0.35,
                                "angle_range": 0.12,
                                "responded": True,
                            }
                        ],
                    },
                    "action_sequence_summary": {
                        "transition_count": 1,
                        "transitions_under_min": False,
                        "max_numeric_transition_delta": 0.4,
                        "transition_delta_under_min": False,
                    },
                    "mechanical_restoration_summary": {
                        "complete": True,
                        "score": 1.0,
                    },
                    "mechanical_gate_summary": {
                        "checks": {
                            "mechanical_restoration": True,
                            "joint_parameter_readback": True,
                            "control_parameter_readback": True,
                            "full_node_tree_restoration": True,
                        },
                        "enabled_checks": [
                            "mechanical_restoration",
                            "joint_parameter_readback",
                            "control_parameter_readback",
                            "full_node_tree_restoration",
                        ],
                        "enabled_count": 4,
                        "full_mechanical_restoration_required": True,
                    },
                    "node_tree_gate_summary": {
                        "checks": {
                            "incomplete_node_tree": True,
                            "class_mismatch": True,
                            "missing_parameters": True,
                            "transform_mismatch": True,
                            "physical_mismatch": True,
                            "fixed_lock_mismatch": True,
                        },
                        "enabled_checks": [
                            "incomplete_node_tree",
                            "class_mismatch",
                            "missing_parameters",
                            "transform_mismatch",
                            "physical_mismatch",
                            "fixed_lock_mismatch",
                        ],
                        "enabled_count": 6,
                        "full_node_tree_restoration_required": True,
                    },
                    "node_tree_manifest": {
                        "complete": True,
                        "missing_part_ids": [],
                        "missing_connection_names": [],
                        "class_mismatch_count": 0,
                        "parameter_missing_count": 0,
                        "transform_mismatch_count": 0,
                        "physical_mismatch_count": 0,
                        "fixed_lock_checked_count": 0,
                        "fixed_lock_mismatch_count": 0,
                        "fixed_locks_complete": True,
                    },
                    "mechanical_behavior_evidence": {
                        "evidence_version": (
                            "dynamic_godot_mechanical_behavior_evidence.v1"
                        ),
                        "complete": True,
                        "residual_risks": [],
                        "threshold_failures": [],
                        "center_of_mass_evidence": {"available": True},
                        "contact_state_evidence": {"available": True},
                        "step_trace_evidence": {
                            "artifact_written": True,
                            "artifact_path": "trace-success.json",
                        },
                    },
                },
            },
        },
        {
            "status": "error",
            "static": {
                "input": "strict.json",
                "robot_name": "strict",
                "parts_count": 3,
                "connections_count": 2,
                "errors": [],
            },
            "delivery_contract_preview": {
                "source": "godot_smoke",
                "dynamic_robot_generation": True,
                "complete": False,
                "expected_parts": 3,
                "parts_complete": True,
                "expected_joints": 2,
                "failed_joints": 1,
                "joints_complete": False,
                "parameterized_joints": 1,
                "parameters_complete": False,
                "part_nodes_count": 3,
                "joint_nodes_count": 1,
            },
            "godot_smoke": {
                "returncode": 1,
                "report_summary": {
                    "joint_parameter_consistency_summary": {
                        "mismatch_count": 3,
                        "fixed_lock_checked_count": 1,
                        "fixed_lock_mismatch_count": 2,
                        "complete": False,
                    },
                    "joint_control_consistency_summary": {
                        "configured_count": 3,
                        "checked_count": 1,
                        "missing_count": 2,
                        "mismatch_count": 2,
                        "complete": False,
                    },
                    "joint_control_summary": {
                        "targeted_count": 0,
                        "nonzero_targets_under_min": True,
                    },
                    "action_target_consistency_summary": {
                        "mismatch_count": 1,
                        "unknown_target_count": 2,
                        "invalid_target_count": 4,
                        "complete": False,
                    },
                    "action_sequence_target_consistency_summary": {
                        "mismatch_count": 2,
                        "unknown_target_count": 3,
                        "invalid_target_count": 5,
                        "steps": 1,
                        "complete": False,
                    },
                    "action_target_coverage_summary": {
                        "coverage_ratio": 0.5,
                        "coverage_under_min": True,
                        "complete": False,
                    },
                    "control_action_coverage_summary": {
                        "coverage_ratio": 0.25,
                        "coverage_under_min": True,
                        "complete": False,
                    },
                    "joint_motion_summary": {
                        "max_abs_relative_angle_delta": 0.001,
                        "angle_delta_under_min": True,
                        "max_abs_relative_angle_range": 0.002,
                        "angle_range_under_min": True,
                        "moving_joint_coverage_ratio": 0.25,
                        "moving_joint_coverage_under_min": True,
                        "commanded_joint_response_ratio": 0.5,
                        "commanded_joint_response_under_min": True,
                        "commanded_static_joints": ["hip_left"],
                        "commanded_joint_response_details": [
                            {
                                "joint": "hip_left",
                                "max_abs_target": 0.35,
                                "angle_range": 0.002,
                                "responded": False,
                            }
                        ],
                    },
                    "action_sequence_summary": {
                        "transition_count": 0,
                        "transitions_under_min": True,
                        "max_numeric_transition_delta": 0.001,
                        "transition_delta_under_min": True,
                    },
                    "mechanical_restoration_summary": {
                        "complete": False,
                        "score": 0.83,
                    },
                    "mechanical_gate_summary": {
                        "checks": {
                            "mechanical_restoration": True,
                            "joint_parameter_readback": True,
                            "control_parameter_readback": False,
                            "full_node_tree_restoration": False,
                        },
                        "enabled_checks": [
                            "mechanical_restoration",
                            "joint_parameter_readback",
                        ],
                        "enabled_count": 2,
                        "full_mechanical_restoration_required": False,
                    },
                    "node_tree_gate_summary": {
                        "checks": {
                            "incomplete_node_tree": True,
                            "class_mismatch": False,
                            "missing_parameters": False,
                            "transform_mismatch": False,
                            "physical_mismatch": False,
                            "fixed_lock_mismatch": True,
                        },
                        "enabled_checks": [
                            "incomplete_node_tree",
                            "fixed_lock_mismatch",
                        ],
                        "enabled_count": 2,
                        "full_node_tree_restoration_required": False,
                    },
                    "node_tree_manifest": {
                        "complete": False,
                        "missing_part_ids": ["shin_left"],
                        "missing_connection_names": ["knee_left"],
                        "class_mismatch_count": 2,
                        "parameter_missing_count": 1,
                        "transform_mismatch_count": 4,
                        "physical_mismatch_count": 5,
                        "fixed_lock_checked_count": 2,
                        "fixed_lock_mismatch_count": 2,
                        "fixed_locks_complete": False,
                    },
                    "mechanical_behavior_evidence": {
                        "evidence_version": (
                            "dynamic_godot_mechanical_behavior_evidence.v1"
                        ),
                        "complete": False,
                        "residual_risks": [
                            "contact_state_runtime_readback_missing"
                        ],
                        "threshold_failures": [
                            "commanded_joint_response_under_min"
                        ],
                        "center_of_mass_evidence": {"available": True},
                        "contact_state_evidence": {"available": False},
                        "step_trace_evidence": {"artifact_written": False},
                    },
                    "errors": ["joint parameter mismatch detected: 3 mismatches"],
                },
            },
        },
    ]

    summary = report_tool._build_batch_summary(reports)
    accepted_summary = report_tool._build_batch_summary([reports[0]])

    assert accepted_summary["delivery_acceptance_complete"] is True
    assert accepted_summary["delivery_acceptance_level"] == "godot_verified"
    assert accepted_summary["delivery_acceptance_reasons"] == []
    assert accepted_summary["delivery_acceptance_reason_codes"] == []
    assert accepted_summary["delivery_acceptance_reason_details"] == []

    assert summary["live_smoke_count"] == 2
    assert summary["delivery_godot_verified_count"] == 2
    assert summary["delivery_static_only_count"] == 0
    assert summary["delivery_unverified_count"] == 0
    assert summary["delivery_dynamic_generation_count"] == 2
    assert summary["delivery_acceptance_complete"] is False
    assert summary["delivery_acceptance_level"] == "incomplete"
    assert summary["delivery_acceptance_reasons"] == [
        "1 robot(s) reported errors",
        "1 robot(s) have incomplete delivery",
        "1 robot(s) have incomplete joint parameters",
        "2 node-tree fixed joint lock mismatch(es) were reported",
        "2 control readback metadata item(s) were missing",
        "2 smoke failure reason(s) were reported",
    ]
    assert summary["delivery_acceptance_reason_codes"] == [
        "robot_errors",
        "incomplete_delivery",
        "incomplete_joint_parameters",
        "node_tree_fixed_lock_mismatch",
        "control_readback_missing",
        "smoke_failure_reasons",
    ]
    assert summary["delivery_acceptance_reason_details"] == [
        {
            "code": "robot_errors",
            "count": 1,
            "message": "1 robot(s) reported errors",
            "inputs": ["strict.json"],
            "inputs_count": 1,
            "inputs_truncated": False,
        },
        {
            "code": "incomplete_delivery",
            "count": 1,
            "message": "1 robot(s) have incomplete delivery",
            "inputs": ["strict.json"],
            "inputs_count": 1,
            "inputs_truncated": False,
        },
        {
            "code": "incomplete_joint_parameters",
            "count": 1,
            "message": "1 robot(s) have incomplete joint parameters",
            "inputs": ["strict.json"],
            "inputs_count": 1,
            "inputs_truncated": False,
        },
        {
            "code": "node_tree_fixed_lock_mismatch",
            "count": 2,
            "message": "2 node-tree fixed joint lock mismatch(es) were reported",
            "inputs": ["strict.json"],
            "inputs_count": 1,
            "inputs_truncated": False,
        },
        {
            "code": "control_readback_missing",
            "count": 2,
            "message": "2 control readback metadata item(s) were missing",
            "inputs": ["strict.json"],
            "inputs_count": 1,
            "inputs_truncated": False,
        },
        {
            "code": "smoke_failure_reasons",
            "count": 2,
            "message": "2 smoke failure reason(s) were reported",
            "inputs": ["strict.json"],
            "inputs_count": 1,
            "inputs_truncated": False,
        },
    ]
    assert summary["delivery_complete_count"] == 1
    assert summary["delivery_incomplete_count"] == 1
    assert summary["delivery_parameters_incomplete_count"] == 1
    assert summary["delivery_failure_robots"] == [
        {
            "input": "strict.json",
            "robot_name": "strict",
            "delivery_source": "godot_smoke",
            "delivery_dynamic_robot_generation": True,
            "delivery_complete": False,
            "delivery_expected_parts": 3,
            "delivery_parts_complete": True,
            "delivery_expected_joints": 2,
            "delivery_failed_joints": 1,
            "delivery_joints_complete": False,
            "delivery_parameterized_joints": 1,
            "delivery_parameters_complete": False,
            "delivery_part_nodes_count": 3,
            "delivery_joint_nodes_count": 1,
            "failure_reasons": [
                "godot smoke exited with returncode 1",
                "joint parameter mismatch detected: 3 mismatches",
            ],
        }
    ]
    assert summary["parameter_mismatch_count"] == 3
    assert summary["fixed_lock_checked_count"] == 2
    assert summary["fixed_lock_mismatch_count"] == 2
    assert summary["parameter_consistency_complete_count"] == 1
    assert summary["control_mismatch_count"] == 2
    assert summary["control_configured_count"] == 5
    assert summary["control_readback_checked_count"] == 3
    assert summary["control_readback_missing_count"] == 2
    assert summary["control_consistency_complete_count"] == 1
    assert summary["nonzero_action_targets_under_min_count"] == 1
    assert summary["action_target_mismatch_count"] == 1
    assert summary["unknown_action_target_count"] == 3
    assert summary["invalid_action_target_count"] == 5
    assert summary["action_target_consistency_complete_count"] == 1
    assert summary["action_sequence_target_mismatch_count"] == 2
    assert summary["action_sequence_target_consistency_complete_count"] == 1
    assert summary["action_target_coverage_under_min_count"] == 1
    assert summary["action_target_coverage_complete_count"] == 1
    assert summary["control_action_coverage_under_min_count"] == 1
    assert summary["control_action_coverage_complete_count"] == 1
    assert summary["joint_angle_delta_under_min_count"] == 1
    assert summary["joint_angle_range_under_min_count"] == 1
    assert summary["moving_joint_coverage_under_min_count"] == 1
    assert summary["commanded_joint_response_under_min_count"] == 1
    assert summary["commanded_static_joint_count"] == 1
    assert summary["commanded_response_failure_robots"] == [
        {
            "input": "strict.json",
            "robot_name": "strict",
            "commanded_joint_response_ratio": 0.5,
            "commanded_static_joints": ["hip_left"],
            "commanded_joint_response_details": [
                {
                    "joint": "hip_left",
                    "max_abs_target": 0.35,
                    "angle_range": 0.002,
                    "responded": False,
                }
            ],
            "failure_reasons": [
                "godot smoke exited with returncode 1",
                "joint parameter mismatch detected: 3 mismatches",
            ],
        }
    ]
    assert summary["action_transitions_under_min_count"] == 1
    assert summary["action_transition_delta_under_min_count"] == 1
    assert summary["restoration_complete_count"] == 1
    assert summary["restoration_incomplete_count"] == 1
    assert summary["mechanical_gate_enabled_count"] == 6
    assert summary["full_mechanical_restoration_required_count"] == 1
    assert summary["full_mechanical_restoration_not_required_count"] == 1
    assert summary["mechanical_gate_check_counts"] == {
        "mechanical_restoration": 2,
        "joint_parameter_readback": 2,
        "control_parameter_readback": 1,
        "full_node_tree_restoration": 1,
    }
    assert summary["mechanical_behavior_evidence_count"] == 2
    assert summary["mechanical_behavior_complete_count"] == 1
    assert summary["mechanical_behavior_incomplete_count"] == 1
    assert summary["mechanical_behavior_residual_risk_count"] == 1
    assert summary["mechanical_behavior_threshold_failure_count"] == 1
    assert summary["mechanical_behavior_center_of_mass_available_count"] == 2
    assert summary["mechanical_behavior_contact_state_available_count"] == 1
    assert summary["mechanical_behavior_step_trace_artifact_count"] == 1
    assert summary["node_tree_complete_count"] == 1
    assert summary["node_tree_incomplete_count"] == 1
    assert summary["node_tree_missing_parts_count"] == 1
    assert summary["node_tree_missing_joints_count"] == 1
    assert summary["node_tree_class_mismatch_count"] == 2
    assert summary["node_tree_parameter_missing_count"] == 1
    assert summary["node_tree_transform_mismatch_count"] == 4
    assert summary["node_tree_physical_mismatch_count"] == 5
    assert summary["node_tree_fixed_lock_checked_count"] == 2
    assert summary["node_tree_fixed_lock_mismatch_count"] == 2
    assert summary["node_tree_fixed_locks_complete_count"] == 1
    assert summary["node_tree_fixed_locks_incomplete_count"] == 1
    assert summary["node_tree_gate_enabled_count"] == 8
    assert summary["node_tree_full_restoration_required_count"] == 1
    assert summary["node_tree_full_restoration_not_required_count"] == 1
    assert summary["node_tree_gate_check_counts"] == {
        "incomplete_node_tree": 2,
        "class_mismatch": 1,
        "missing_parameters": 1,
        "transform_mismatch": 1,
        "physical_mismatch": 1,
        "fixed_lock_mismatch": 2,
    }
    assert summary["failure_reasons_count"] == 2
    assert summary["robots"][1]["godot_smoke_returncode"] == 1
    assert summary["robots"][1]["delivery_source"] == "godot_smoke"
    assert summary["robots"][1]["delivery_dynamic_robot_generation"] is True
    assert summary["robots"][1]["delivery_complete"] is False
    assert summary["robots"][1]["delivery_expected_parts"] == 3
    assert summary["robots"][1]["delivery_parts_complete"] is True
    assert summary["robots"][1]["delivery_expected_joints"] == 2
    assert summary["robots"][1]["delivery_failed_joints"] == 1
    assert summary["robots"][1]["delivery_joints_complete"] is False
    assert summary["robots"][1]["delivery_parameterized_joints"] == 1
    assert summary["robots"][1]["delivery_parameters_complete"] is False
    assert summary["robots"][1]["delivery_part_nodes_count"] == 3
    assert summary["robots"][1]["delivery_joint_nodes_count"] == 1
    assert summary["robots"][1]["parameter_mismatch_count"] == 3
    assert summary["robots"][1]["parameter_consistency_complete"] is False
    assert summary["robots"][1]["fixed_lock_checked_count"] == 1
    assert summary["robots"][1]["fixed_lock_mismatch_count"] == 2
    assert summary["robots"][1]["control_mismatch_count"] == 2
    assert summary["robots"][1]["control_configured_count"] == 3
    assert summary["robots"][1]["control_readback_checked_count"] == 1
    assert summary["robots"][1]["control_readback_missing_count"] == 2
    assert summary["robots"][1]["control_consistency_complete"] is False
    assert summary["robots"][1]["nonzero_action_target_count"] == 0
    assert summary["robots"][1]["nonzero_action_targets_under_min"] is True
    assert summary["robots"][1]["action_target_mismatch_count"] == 1
    assert summary["robots"][1]["unknown_action_target_count"] == 3
    assert summary["robots"][1]["invalid_action_target_count"] == 5
    assert summary["robots"][1]["action_target_consistency_complete"] is False
    assert summary["robots"][1]["action_sequence_target_mismatch_count"] == 2
    assert summary["robots"][1]["action_sequence_target_consistency_complete"] is False
    assert summary["robots"][1]["action_target_coverage_ratio"] == 0.5
    assert summary["robots"][1]["action_target_coverage_under_min"] is True
    assert summary["robots"][1]["action_target_coverage_complete"] is False
    assert summary["robots"][1]["control_action_coverage_ratio"] == 0.25
    assert summary["robots"][1]["control_action_coverage_under_min"] is True
    assert summary["robots"][1]["control_action_coverage_complete"] is False
    assert summary["robots"][1]["joint_angle_delta_max"] == 0.001
    assert summary["robots"][1]["joint_angle_delta_under_min"] is True
    assert summary["robots"][1]["joint_angle_range_max"] == 0.002
    assert summary["robots"][1]["joint_angle_range_under_min"] is True
    assert summary["robots"][1]["moving_joint_coverage_ratio"] == 0.25
    assert summary["robots"][1]["moving_joint_coverage_under_min"] is True
    assert summary["robots"][1]["commanded_joint_response_ratio"] == 0.5
    assert summary["robots"][1]["commanded_joint_response_under_min"] is True
    assert summary["robots"][1]["commanded_static_joints"] == ["hip_left"]
    assert summary["robots"][1]["commanded_joint_response_details"][0] == {
        "joint": "hip_left",
        "max_abs_target": 0.35,
        "angle_range": 0.002,
        "responded": False,
    }
    assert summary["robots"][1]["action_transition_count"] == 0
    assert summary["robots"][1]["action_transitions_under_min"] is True
    assert summary["robots"][1]["action_transition_delta_max"] == 0.001
    assert summary["robots"][1]["action_transition_delta_under_min"] is True
    assert summary["robots"][1]["restoration_complete"] is False
    assert summary["robots"][1]["restoration_score"] == 0.83
    assert summary["robots"][1]["mechanical_gate_enabled_count"] == 2
    assert summary["robots"][1]["full_mechanical_restoration_required"] is False
    assert summary["robots"][1]["mechanical_gate_enabled_checks"] == [
        "mechanical_restoration",
        "joint_parameter_readback",
    ]
    assert (
        summary["robots"][1]["mechanical_behavior_evidence_version"]
        == "dynamic_godot_mechanical_behavior_evidence.v1"
    )
    assert summary["robots"][1]["mechanical_behavior_complete"] is False
    assert summary["robots"][1]["mechanical_behavior_residual_risks"] == [
        "contact_state_runtime_readback_missing"
    ]
    assert summary["robots"][1]["mechanical_behavior_threshold_failures"] == [
        "commanded_joint_response_under_min"
    ]
    assert summary["robots"][1]["mechanical_behavior_center_of_mass_available"] is True
    assert summary["robots"][1]["mechanical_behavior_contact_state_available"] is False
    assert summary["robots"][1]["mechanical_behavior_step_trace_artifact_path"] is None
    assert summary["robots"][1]["node_tree_complete"] is False
    assert summary["robots"][1]["node_tree_gate_enabled_count"] == 2
    assert summary["robots"][1]["node_tree_full_restoration_required"] is False
    assert summary["robots"][1]["node_tree_gate_enabled_checks"] == [
        "incomplete_node_tree",
        "fixed_lock_mismatch",
    ]
    assert summary["robots"][1]["node_tree_missing_parts_count"] == 1
    assert summary["robots"][1]["node_tree_missing_joints_count"] == 1
    assert summary["robots"][1]["node_tree_class_mismatch_count"] == 2
    assert summary["robots"][1]["node_tree_parameter_missing_count"] == 1
    assert summary["robots"][1]["node_tree_transform_mismatch_count"] == 4
    assert summary["robots"][1]["node_tree_physical_mismatch_count"] == 5
    assert summary["robots"][1]["node_tree_fixed_lock_checked_count"] == 2
    assert summary["robots"][1]["node_tree_fixed_lock_mismatch_count"] == 2
    assert summary["robots"][1]["node_tree_fixed_locks_complete"] is False
    assert summary["robots"][1]["failure_reasons"] == [
        "godot smoke exited with returncode 1",
        "joint parameter mismatch detected: 3 mismatches",
    ]


def test_smoke_report_summary_includes_node_tree_mismatch_preview() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_report", REPORT_TOOL)
    report_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(report_tool)

    smoke_report = {
        "status": "error",
        "robot_name": "preview",
        "expected_parts": 1,
        "expected_joints": 1,
        "load_result": {"parts_created": 1, "joints_created": 1},
        "mapping_summary": {"part_nodes": 1, "joint_nodes": 1},
        "mechanical_gate_summary": {
            "enabled_count": 4,
            "full_mechanical_restoration_required": True,
        },
        "node_tree_gate_summary": {
            "enabled_count": 6,
            "full_node_tree_restoration_required": True,
        },
        "node_tree_manifest": {
            "class_mismatches": [
                {
                    "kind": "part",
                    "name": f"part_{index}",
                    "field": "mesh_type",
                    "expected": "CapsuleMesh",
                    "actual": "BoxMesh",
                }
                for index in range(7)
            ],
            "transform_mismatches": [
                {
                    "kind": "joint",
                    "name": f"joint_{index}",
                    "field": "axis",
                    "expected": [0.0, 0.0, 1.0],
                    "actual": [1.0, 0.0, 0.0],
                    "max_delta": 1.0,
                }
                for index in range(6)
            ],
            "physical_mismatches": [
                {
                    "kind": "part",
                    "name": f"part_{index}",
                    "field": "mass",
                    "expected": 5.0,
                    "actual": 4.0,
                    "max_delta": 1.0,
                }
                for index in range(8)
            ],
            "fixed_lock_mismatches": [
                {
                    "joint": f"fixed_{index}",
                    "field": "fixed.linear_upper.y",
                    "expected": 0.0,
                    "actual": 0.25,
                }
                for index in range(6)
            ],
            "missing_parameter_connection_names": [f"joint_{index}" for index in range(12)],
            },
            "step_summary": {},
            "mechanical_behavior_evidence": {
                "evidence_version": "dynamic_godot_mechanical_behavior_evidence.v1",
                "residual_risks": ["center_of_mass_runtime_readback_missing"],
            },
            "errors": [],
        }

    summary = report_tool._build_smoke_report_summary(smoke_report)
    assert summary["mechanical_gate_summary"] == {
        "enabled_count": 4,
        "full_mechanical_restoration_required": True,
    }
    assert summary["node_tree_gate_summary"] == {
        "enabled_count": 6,
        "full_node_tree_restoration_required": True,
    }
    assert summary["mechanical_behavior_evidence"] == {
        "evidence_version": "dynamic_godot_mechanical_behavior_evidence.v1",
        "residual_risks": ["center_of_mass_runtime_readback_missing"],
    }
    preview = summary["node_tree_mismatch_preview"]

    assert len(preview["class_mismatches"]) == 5
    assert len(preview["transform_mismatches"]) == 5
    assert len(preview["physical_mismatches"]) == 5
    assert len(preview["fixed_lock_mismatches"]) == 5
    assert len(preview["missing_parameter_connection_names"]) == 10
    assert preview["class_mismatches"][0]["name"] == "part_0"
    assert preview["transform_mismatches"][0]["max_delta"] == 1.0
    assert preview["fixed_lock_mismatches"][0]["joint"] == "fixed_0"


def test_batch_summary_collects_validation_skip_reasons() -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_report", REPORT_TOOL)
    report_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(report_tool)

    summary = report_tool._build_batch_summary(
        [
            {
                "status": "error",
                "static": {
                    "input": "invalid.json",
                    "robot_name": "invalid",
                    "parts_count": 1,
                    "connections_count": 1,
                    "errors": ["parts[0].shape must be one of box, capsule"],
                },
                "godot_smoke": None,
                "godot_smoke_skipped_reason": "validation_failed: smoke skipped",
            }
        ]
    )

    assert summary["failure_reasons_count"] == 2
    assert summary["robots"][0]["failure_reasons"] == [
        "parts[0].shape must be one of box, capsule",
        "validation_failed: smoke skipped",
    ]


def test_dynamic_robot_generation_report_tool_keeps_smoke_output_compact() -> None:
    content = REPORT_TOOL.read_text(encoding="utf-8")

    assert '"stdout_tail": stdout_lines[-20:]' in content
    assert '"stderr_tail": stderr_lines[-20:]' in content
    assert '"stdout_line_count": len(stdout_lines)' in content
    assert '"stderr_line_count": len(stderr_lines)' in content
    assert '"report_summary": _build_smoke_report_summary(smoke_report)' in content
    assert "def _build_smoke_report_summary" in content
    assert "def _build_node_tree_mismatch_preview" in content
    assert "def _slice_list" in content
    assert "def _report_input" in content
    assert "def _acceptance_reason_detail" in content
    assert "def _build_batch_summary" in content
    assert "def _build_batch_robot_summary" in content
    assert "def _build_delivery_failure_robots" in content
    assert "def _build_commanded_response_failure_robots" in content
    assert "def _build_batch_failure_reasons" in content
    assert "def _build_delivery_acceptance_gate" in content
    assert "def _format_delivery_acceptance_gate_log" in content
    assert "validate_delivery_acceptance_gate = _workflow_contracts.validate_delivery_acceptance_gate" in content
    assert "def _effective_unknown_action_target_count" in content
    assert "def _effective_invalid_action_target_count" in content
    assert '"delivery_complete_count": sum(' in content
    assert '"delivery_godot_verified_count": sum(' in content
    assert '"delivery_static_only_count": sum(' in content
    assert '"delivery_unverified_count": sum(' in content
    assert '"delivery_dynamic_generation_count": sum(' in content
    assert '"delivery_acceptance_complete": acceptance_complete' in content
    assert '"delivery_acceptance_level": level' in content
    assert '"delivery_acceptance_reasons": reasons' in content
    assert '"delivery_acceptance_reason_codes": [' in content
    assert '"delivery_acceptance_reason_details": reason_details' in content
    assert "node_tree_fixed_lock_mismatch_count = sum(" in content
    assert '"node_tree_fixed_lock_mismatch"' in content
    assert "control_readback_missing_count = sum(" in content
    assert 'code="control_readback_missing"' in content
    assert "node-tree fixed joint lock " in content
    assert "mismatch(es) were reported" in content
    assert '"inputs": _slice_list(inputs, 10)' in content
    assert '"inputs_count": len(inputs)' in content
    assert '"inputs_truncated": len(inputs) > 10' in content
    assert "DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION" in content
    assert "build_delivery_acceptance_requirements" in content
    assert 'DELIVERY_ACCEPTANCE_GATE_SOURCE = "dynamic_godot_report_cli"' in content
    assert 'DELIVERY_ACCEPTANCE_GATE_SCOPE = "godot_smoke_motion"' in content
    assert '"contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION' in content
    assert '"source": DELIVERY_ACCEPTANCE_GATE_SOURCE' in content
    assert '"verification_scope": DELIVERY_ACCEPTANCE_GATE_SCOPE' in content
    assert '"requires_full_mechanical_restoration_gate": requires_full_mechanical_gate' in content
    assert '"acceptance_profile": batch_summary.get("acceptance_profile", "custom")' in content
    assert '"acceptance_requirements": batch_summary.get("acceptance_requirements", {})' in content
    assert '"reason_codes": batch_summary["delivery_acceptance_reason_codes"]' in content
    assert '"reason_details": batch_summary["delivery_acceptance_reason_details"]' in content
    assert '"summary_counts": {' in content
    assert '"inputs_count": batch_summary.get("inputs_count", 0)' in content
    assert '"live_smoke_count": batch_summary.get("live_smoke_count", 0)' in content
    assert '"smoke_report_written_count": batch_summary.get(' in content
    assert '"smoke_report_missing_count": batch_summary.get(' in content
    assert '"smoke_report_read_error_count": batch_summary.get(' in content
    assert '"delivery_godot_verified_count": batch_summary.get(' in content
    assert '"delivery_parameters_incomplete_count": batch_summary.get(' in content
    assert '"fixed_lock_checked_count": batch_summary.get(' in content
    assert '"fixed_lock_mismatch_count": batch_summary.get(' in content
    assert '"control_configured_count": batch_summary.get(' in content
    assert '"control_readback_checked_count": batch_summary.get(' in content
    assert '"control_readback_missing_count": batch_summary.get(' in content
    assert '"node_tree_fixed_lock_checked_count": batch_summary.get(' in content
    assert '"node_tree_fixed_lock_mismatch_count": batch_summary.get(' in content
    assert '"node_tree_fixed_locks_complete_count": batch_summary.get(' in content
    assert '"node_tree_fixed_locks_incomplete_count": batch_summary.get(' in content
    assert '"node_tree_gate_enabled_count": batch_summary.get(' in content
    assert '"node_tree_full_restoration_required_count": batch_summary.get(' in content
    assert '"node_tree_full_restoration_not_required_count": batch_summary.get(' in content
    assert '"node_tree_gate_check_counts": batch_summary.get(' in content
    assert '"static_topology_complete_count": batch_summary.get(' in content
    assert '"static_topology_cycle_count": batch_summary.get(' in content
    assert '"static_node_tree_manifest_error_count": batch_summary.get(' in content
    assert (
        '"static_node_tree_manifest_path_map_mismatch_count": batch_summary.get('
        in content
    )
    assert (
        '"static_node_tree_manifest_path_map_mismatch_kind_counts": ('
        in content
    )
    assert '"static_node_tree_manifest_count": batch_summary.get(' in content
    assert '"static_node_tree_manifest_output_count": batch_summary.get(' in content
    assert '"static_node_tree_parts_planned_count": batch_summary.get(' in content
    assert '"static_node_tree_joints_planned_count": batch_summary.get(' in content
    assert '"static_node_tree_parameterized_joints_count": batch_summary.get(' in content
    assert '"static_node_tree_complete_count": batch_summary.get(' in content
    assert '"static_node_tree_incomplete_count": batch_summary.get(' in content
    assert '"static_node_tree_endpoint_paths_complete_count": batch_summary.get(' in content
    assert '"static_node_tree_endpoint_paths_incomplete_count": batch_summary.get(' in content
    assert '"static_node_tree_missing_endpoint_parts_count": batch_summary.get(' in content
    assert '"static_node_tree_missing_endpoint_connections_count": batch_summary.get(' in content
    assert '"static_node_tree_parameters_complete_count": batch_summary.get(' in content
    assert '"static_node_tree_parameters_incomplete_count": batch_summary.get(' in content
    assert '"mechanical_gate_enabled_count": batch_summary.get(' in content
    assert '"full_mechanical_restoration_required_count": batch_summary.get(' in content
    assert '"mechanical_gate_check_counts": batch_summary.get(' in content
    assert '"mechanical_behavior_evidence_count": batch_summary.get(' in content
    assert '"mechanical_behavior_complete_count": batch_summary.get(' in content
    assert '"mechanical_behavior_incomplete_count": batch_summary.get(' in content
    assert (
        '"mechanical_behavior_residual_risk_count": batch_summary.get('
        in content
    )
    assert (
        '"mechanical_behavior_threshold_failure_count": batch_summary.get('
        in content
    )
    assert (
        '"mechanical_behavior_center_of_mass_available_count": batch_summary.get('
        in content
    )
    assert (
        '"mechanical_behavior_contact_state_available_count": batch_summary.get('
        in content
    )
    assert (
        '"mechanical_behavior_step_trace_artifact_count": batch_summary.get('
        in content
    )
    assert '"failure_reasons_count": batch_summary.get("failure_reasons_count", 0)' in content
    assert "mechbehavior:{counts.get('mechanical_behavior_evidence_count', 0)}/" in content
    assert "inputs:{counts.get('inputs_count', 0)}" in content
    assert "counts={count_text}" in content
    assert "checks={check_text}" in content
    assert "topology={topology_text}" in content
    assert "def _format_static_topology_counts(batch_summary: dict[str, Any]) -> str" in content
    assert "def _format_gate_check_counts(counts: dict[str, Any]) -> str" in content
    assert '"node_tree_gate_check_counts"' in content
    assert '"mechanical_gate_check_counts"' in content
    assert 'parts.append(f"{label}[{\',\'.join(entries)}]")' in content
    assert "requirements = gate.get(\"acceptance_requirements\", {})" in content
    assert "requirements={requirement_text}" in content
    assert "verified:{counts.get('delivery_godot_verified_count', 0)}" in content
    assert "fixed:{counts.get('fixed_lock_checked_count', 0)}/" in content
    assert "control:{counts.get('control_readback_checked_count', 0)}/" in content
    assert "treefixed:{counts.get('node_tree_fixed_lock_checked_count', 0)}/" in content
    assert "treegate:{counts.get('node_tree_gate_enabled_count', 0)}/" in content
    assert "mechgate:{counts.get('mechanical_gate_enabled_count', 0)}/" in content
    assert "failures:{counts.get('failure_reasons_count', 0)}" in content
    assert "delivery_acceptance_gate failed" in content
    assert "delivery_acceptance_gate contract invalid" in content
    assert "gate_contract_errors = validate_delivery_acceptance_gate(acceptance_gate)" in content
    assert "--gate-output" in content
    assert "args.gate_output" in content
    assert "_write_json(args.gate_output, acceptance_gate)" in content
    assert "acceptance_profile={gate.get('acceptance_profile', 'custom')}" in content
    assert "affected_inputs={input_text}" in content
    assert "_format_delivery_acceptance_gate_log(acceptance_gate, batch_summary)" in content
    assert "--require-godot-verified-acceptance" in content
    assert "--require-full-mechanical-restoration-gate" in content
    assert "--full-mechanical-restoration-acceptance" in content
    assert "args.full_mechanical_restoration_acceptance" in content
    assert "args.run_godot_smoke = True" in content
    assert "args.fail_on_full_mechanical_restoration = True" in content
    assert "args.require_godot_verified_acceptance = True" in content
    assert "args.require_full_mechanical_restoration_gate = True" in content
    assert "args.require_full_mechanical_restoration_gate" in content
    assert "acceptance_profile = (" in content
    assert "def _build_acceptance_requirements" in content
    assert "acceptance_requirements = _build_acceptance_requirements(args)" in content
    assert "full_mechanical_restoration_smoke_gate=full_mechanical" in content
    assert "mechanical_restoration_complete=bool(args.fail_on_incomplete_restoration)" in content
    assert "joint_parameter_readback=bool(args.fail_on_parameter_mismatch)" in content
    assert "control_parameter_readback=bool(args.fail_on_control_mismatch)" in content
    assert "full_node_tree_restoration=full_node_tree" in content
    assert "node_tree_fixed_lock_match=bool(args.fail_on_node_tree_fixed_lock_mismatch)" in content
    assert '"full_mechanical_restoration"' in content
    assert '"acceptance_profile": batch_summary["acceptance_profile"]' in content
    assert "delivery_acceptance_summary" in content
    assert "delivery_acceptance_gate" in content
    assert "args.require_godot_verified_acceptance" in content
    assert 'return int(acceptance_gate["exit_code"])' in content
    assert '"delivery_incomplete_count": sum(' in content
    assert '"delivery_parameters_incomplete_count": sum(' in content
    assert '"fixed_lock_checked_count": sum(' in content
    assert '"fixed_lock_mismatch_count": sum(' in content
    assert '"fixed_lock_mismatches": _slice_list(' in content
    assert '"node_tree_fixed_lock_checked_count": sum(' in content
    assert '"node_tree_fixed_lock_mismatch_count": sum(' in content
    assert '"node_tree_fixed_locks_complete_count": sum(' in content
    assert '"node_tree_fixed_locks_incomplete_count": sum(' in content
    assert '"delivery_failure_robots": _build_delivery_failure_robots(robot_summaries)' in content
    assert '"delivery_complete": (' in content
    assert '"delivery_source": (' in content
    assert '"delivery_dynamic_robot_generation": (' in content
    assert '"delivery_expected_parts": (' in content
    assert '"delivery_parts_complete": (' in content
    assert '"delivery_expected_joints": (' in content
    assert '"delivery_failed_joints": (' in content
    assert '"delivery_joints_complete": (' in content
    assert '"delivery_parameterized_joints": (' in content
    assert '"delivery_parameters_complete": (' in content
    assert '"delivery_part_nodes_count": (' in content
    assert '"delivery_joint_nodes_count": (' in content
    assert '"fixed_lock_checked_count": (' in content
    assert '"fixed_lock_mismatch_count": (' in content
    assert '"node_tree_fixed_lock_checked_count": (' in content
    assert '"node_tree_fixed_lock_mismatch_count": (' in content
    assert '"node_tree_fixed_locks_complete": (' in content
    assert '"complete": load_result.get("complete")' in content
    assert '"expected_parts": load_result.get("expected_parts")' in content
    assert '"parts_complete": load_result.get("parts_complete")' in content
    assert '"expected_joints": load_result.get("expected_joints")' in content
    assert '"failed_joints": load_result.get("failed_joints")' in content
    assert '"joints_complete": load_result.get("joints_complete")' in content
    assert '"parameterized_joints": load_result.get("parameterized_joints")' in content
    assert '"parameters_complete": load_result.get("parameters_complete")' in content
    assert '"complete": static_report["status"] == "success"' in content
    assert '"parameter_mismatch_count": sum(' in content
    assert '"control_mismatch_count": sum(' in content
    assert '"control_configured_count": sum(' in content
    assert '"control_readback_checked_count": sum(' in content
    assert '"control_readback_missing_count": sum(' in content
    assert '"nonzero_action_targets_under_min_count": sum(' in content
    assert '"action_target_mismatch_count": sum(' in content
    assert '"unknown_action_target_count": sum(' in content
    assert '"invalid_action_target_count": sum(' in content
    assert '"action_sequence_target_mismatch_count": sum(' in content
    assert '"action_target_coverage_under_min_count": sum(' in content
    assert '"action_target_coverage_complete_count": sum(' in content
    assert '"control_action_coverage_under_min_count": sum(' in content
    assert '"control_action_coverage_complete_count": sum(' in content
    assert '"joint_angle_delta_under_min_count": sum(' in content
    assert '"joint_angle_range_under_min_count": sum(' in content
    assert '"moving_joint_coverage_under_min_count": sum(' in content
    assert '"commanded_joint_response_under_min_count": sum(' in content
    assert '"commanded_static_joint_count": sum(' in content
    assert '"commanded_response_failure_robots": _build_commanded_response_failure_robots(' in content
    assert '"action_transitions_under_min_count": sum(' in content
    assert '"action_transition_delta_under_min_count": sum(' in content
    assert '"restoration_incomplete_count": sum(' in content
    assert '"mechanical_gate_enabled_count": sum(' in content
    assert '"full_mechanical_restoration_required_count": sum(' in content
    assert '"mechanical_gate_check_counts": _mechanical_gate_check_counts(' in content
    assert '"node_tree_complete_count": sum(' in content
    assert '"node_tree_missing_parts_count": sum(' in content
    assert '"node_tree_missing_joints_count": sum(' in content
    assert '"node_tree_class_mismatch_count": sum(' in content
    assert '"node_tree_gate_enabled_count": sum(' in content
    assert '"node_tree_full_restoration_required_count": sum(' in content
    assert '"node_tree_gate_check_counts": _node_tree_gate_check_counts(' in content
    assert '"node_tree_parameter_missing_count": sum(' in content
    assert '"node_tree_transform_mismatch_count": sum(' in content
    assert '"node_tree_physical_mismatch_count": sum(' in content
    assert '"static_node_tree_manifest_count": sum(' in content
    assert '"static_node_tree_manifest_error_count": sum(' in content
    assert '"static_node_tree_manifest_output_count": len(' in content
    assert '"static_node_tree_manifest_path_map_mismatch_count": len(' in content
    assert (
        '"static_node_tree_manifest_path_map_mismatch_kind_counts": ('
        in content
    )
    assert '"static_node_tree_parts_planned_count": sum(' in content
    assert '"static_node_tree_complete_count": sum(' in content
    assert '"static_node_tree_incomplete_count": sum(' in content
    assert '"static_node_tree_endpoint_paths_complete_count": sum(' in content
    assert '"static_node_tree_endpoint_paths_incomplete_count": sum(' in content
    assert '"static_node_tree_missing_endpoint_parts_count": sum(' in content
    assert '"static_node_tree_missing_endpoint_connections_count": sum(' in content
    assert '"static_node_tree_missing_endpoint_part_ids": _unique_strings(' in content
    assert '"static_node_tree_missing_endpoint_connection_names": _unique_strings(' in content
    assert '"static_node_tree_missing_endpoint_details": _slice_list(' in content
    assert '"static_node_tree_parameters_complete_count": sum(' in content
    assert '"failure_reasons_count": sum(' in content
    assert '"failure_reasons": _build_batch_failure_reasons(report)' in content
    assert '"godot_smoke_returncode": (' in content
    assert '"node_tree_complete": (' in content
    assert '"joint_angle_delta_max": (' in content
    assert '"joint_angle_delta_under_min": (' in content
    assert '"joint_angle_range_max": (' in content
    assert '"joint_angle_range_under_min": (' in content
    assert '"moving_joint_coverage_ratio": (' in content
    assert '"moving_joint_coverage_under_min": (' in content
    assert '"commanded_joint_response_ratio": (' in content
    assert '"commanded_joint_response_under_min": (' in content
    assert '"commanded_static_joints": (' in content
    assert '"commanded_joint_response_details": (' in content
    assert '"action_transition_delta_max": (' in content
    assert '"action_transition_delta_under_min": (' in content
    assert '"mechanical_gate_enabled_count": (' in content
    assert '"full_mechanical_restoration_required": (' in content
    assert '"mechanical_gate_enabled_checks": (' in content
    assert '"node_tree_class_mismatch_count": (' in content
    assert '"node_tree_gate_enabled_count": (' in content
    assert '"node_tree_full_restoration_required": (' in content
    assert '"node_tree_gate_enabled_checks": (' in content
    assert '"node_tree_parameter_missing_count": (' in content
    assert '"node_tree_transform_mismatch_count": (' in content
    assert '"node_tree_physical_mismatch_count": (' in content
    assert "nargs=\"+\"" in content
    assert '"step_body_states_count": step.get("body_states")' in content
    assert '"step_joint_states_count": step.get("joint_states")' in content
    assert '"first_body_state": step.get("first_body_state", {})' in content
    assert '"first_joint_state": step.get("first_joint_state", {})' in content
    assert '"joint_endpoint_summary": step.get("joint_endpoint_summary", {})' in content
    assert '"joint_angle_summary": step.get("joint_angle_summary", {})' in content
    assert '"joint_limit_summary": step.get("joint_limit_summary", {})' in content
    assert '"joint_parameter_summary": step.get("joint_parameter_summary", {})' in content
    assert '"joint_parameter_consistency_summary": step.get(' in content
    assert '"joint_control_consistency_summary": step.get(' in content
    assert '"action_target_consistency_summary": step.get(' in content
    assert '"action_sequence_target_consistency_summary": step.get(' in content
    assert '"action_target_coverage_summary": step.get(' in content
    assert '"control_action_coverage_summary": step.get(' in content
    assert "node_tree_manifest = smoke_report.get(\"node_tree_manifest\", {})" in content
    assert '"mechanical_gate_summary": smoke_report.get("mechanical_gate_summary", {})' in content
    assert '"node_tree_gate_summary": smoke_report.get("node_tree_gate_summary", {})' in content
    assert '"mechanical_behavior_evidence": smoke_report.get(' in content
    assert "mechanical_trace_output=args.mechanical_trace_output" in content
    assert '"joint_motion_summary": step.get("joint_motion_summary", {})' in content
    assert '"node_tree_manifest": node_tree_manifest' in content
    assert '"node_tree_mismatch_preview": _build_node_tree_mismatch_preview(' in content
    assert "--parameter-tolerance" in content
    assert "parameter_tolerance=args.parameter_tolerance" in content
    assert '"joint_control_summary": step.get("joint_control_summary", {})' in content
    assert "fail_on_control_mismatch=args.fail_on_control_mismatch" in content
    assert "fail_on_full_mechanical_restoration=args.fail_on_full_mechanical_restoration" in content
    assert "fail_on_action_target_mismatch=args.fail_on_action_target_mismatch" in content
    assert "fail_on_action_sequence_target_mismatch=args.fail_on_action_sequence_target_mismatch" in content
    assert "fail_on_unknown_action_target=args.fail_on_unknown_action_target" in content
    assert "fail_on_invalid_action_target=args.fail_on_invalid_action_target" in content
    assert "--fail-on-control-mismatch" in content
    assert "--fail-on-full-mechanical-restoration" in content
    assert "--fail-on-action-target-mismatch" in content
    assert "--fail-on-action-sequence-target-mismatch" in content
    assert "--fail-on-unknown-action-target" in content
    assert "--fail-on-invalid-action-target" in content
    assert '"simulation_summary": step.get("simulation_summary", {})' in content
    assert '"action_sequence_summary": step.get("action_sequence_summary", {})' in content
    assert '"mechanical_restoration_summary": step.get(' in content
    assert '"action_sent": step.get("action_sent")' in content
    assert '"first_action_sent": step.get("first_action_sent")' in content
    assert '"last_action_sent": step.get("last_action_sent")' in content
    assert '"steps_run": step.get("steps_run")' in content
    assert "max_endpoint_distance=args.max_endpoint_distance" in content
    assert "max_relative_angle=args.max_relative_angle" in content
    assert "min_body_displacement=args.min_body_displacement" in content
    assert "max_linear_speed=args.max_linear_speed" in content
    assert "min_joint_angle_delta=args.min_joint_angle_delta" in content
    assert "min_joint_angle_range=args.min_joint_angle_range" in content
    assert "min_moving_joint_coverage=args.min_moving_joint_coverage" in content
    assert (
        "min_commanded_joint_response_coverage="
        "args.min_commanded_joint_response_coverage"
    ) in content
    assert "joint_motion_epsilon=args.joint_motion_epsilon" in content
    assert "min_action_target_coverage=args.min_action_target_coverage" in content
    assert "min_control_action_coverage=args.min_control_action_coverage" in content
    assert "min_nonzero_action_targets=args.min_nonzero_action_targets" in content
    assert "min_action_transitions=args.min_action_transitions" in content
    assert "min_action_transition_delta=args.min_action_transition_delta" in content
    assert "fail_on_joint_limit_violation=args.fail_on_joint_limit_violation" in content
    assert "fail_on_incomplete_restoration=args.fail_on_incomplete_restoration" in content
    assert "min_restoration_score=args.min_restoration_score" in content
    assert "fail_on_parameter_mismatch=args.fail_on_parameter_mismatch" in content
    assert "fail_on_incomplete_node_tree=args.fail_on_incomplete_node_tree" in content
    assert "fail_on_node_tree_class_mismatch=args.fail_on_node_tree_class_mismatch" in content
    assert "fail_on_node_tree_missing_parameters=args.fail_on_node_tree_missing_parameters" in content
    assert "fail_on_node_tree_transform_mismatch=args.fail_on_node_tree_transform_mismatch" in content
    assert "fail_on_node_tree_physical_mismatch=args.fail_on_node_tree_physical_mismatch" in content
    assert "fail_on_node_tree_fixed_lock_mismatch=args.fail_on_node_tree_fixed_lock_mismatch" in content
    assert "node_tree_tolerance=args.node_tree_tolerance" in content
    assert "action_json=args.action_json" in content
    assert "action_sequence_json=args.action_sequence_json" in content
    assert "steps=args.steps" in content
    assert "step_delay_seconds=args.step_delay_seconds" in content
    assert "--action-json" in content
    assert "--action-sequence-json" in content
    assert "--min-body-displacement" in content
    assert "--max-linear-speed" in content
    assert "--min-action-transition-delta" in content
    assert "--min-joint-angle-delta" in content
    assert "--min-joint-angle-range" in content
    assert "--min-moving-joint-coverage" in content
    assert "--min-commanded-joint-response-coverage" in content
    assert "--joint-motion-epsilon" in content
    assert "--steps" in content
    assert "--step-delay-seconds" in content
    assert "--mechanical-trace-output" in content
    assert "--fail-on-joint-limit-violation" in content
    assert "--fail-on-incomplete-restoration" in content
    assert "--min-restoration-score" in content
    assert "--fail-on-parameter-mismatch" in content
    assert "--fail-on-incomplete-node-tree" in content
    assert "--fail-on-full-node-tree-restoration" in content
    assert "--fail-on-node-tree-class-mismatch" in content
    assert "--fail-on-node-tree-missing-parameters" in content
    assert "--fail-on-node-tree-transform-mismatch" in content
    assert "--fail-on-node-tree-physical-mismatch" in content
    assert "--fail-on-node-tree-fixed-lock-mismatch" in content
    assert "--node-tree-tolerance" in content
    assert '"stdout": result.stdout' not in content
    assert '"stderr": result.stderr' not in content
    assert '"report": smoke_report' not in content


def test_report_smoke_wrapper_passes_node_tree_fixed_lock_gate(
    tmp_path: Path,
    monkeypatch,
) -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_report", REPORT_TOOL)
    report_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(report_tool)

    smoke_output = tmp_path / "smoke.json"
    trace_output = tmp_path / "trace.json"
    normalized_output = tmp_path / "normalized.json"
    normalized_output.write_text("{}", encoding="utf-8")

    def fake_run(command, **kwargs):
        smoke_output.write_text(
            json.dumps(
                {
                    "status": "success",
                    "load_result": {},
                    "mapping_summary": {},
                    "step_summary": {},
                    "node_tree_manifest": {},
                }
            ),
            encoding="utf-8",
        )
        return subprocess.CompletedProcess(command, 0, stdout="", stderr="")

    monkeypatch.setattr(report_tool.subprocess, "run", fake_run)

    result = report_tool._run_godot_smoke(
        repo_root=ROOT,
        normalized_output=normalized_output,
        smoke_output=smoke_output,
        godot_exe="Godot.exe",
        port=19170,
        timeout_seconds=1.0,
        max_endpoint_distance=None,
        max_relative_angle=None,
        min_body_displacement=None,
        max_linear_speed=None,
        min_joint_angle_delta=None,
        min_joint_angle_range=None,
        min_moving_joint_coverage=None,
        min_commanded_joint_response_coverage=None,
        joint_motion_epsilon=0.0,
        min_action_target_coverage=None,
        min_control_action_coverage=None,
        min_nonzero_action_targets=None,
        min_action_transitions=None,
        min_action_transition_delta=None,
        fail_on_joint_limit_violation=False,
        fail_on_incomplete_restoration=False,
        min_restoration_score=None,
        fail_on_parameter_mismatch=False,
        fail_on_control_mismatch=False,
        fail_on_full_mechanical_restoration=False,
        fail_on_action_target_mismatch=False,
        fail_on_action_sequence_target_mismatch=False,
        fail_on_unknown_action_target=False,
        fail_on_invalid_action_target=False,
        fail_on_incomplete_node_tree=False,
        fail_on_full_node_tree_restoration=False,
        fail_on_node_tree_class_mismatch=False,
        fail_on_node_tree_missing_parameters=False,
        fail_on_node_tree_transform_mismatch=False,
        fail_on_node_tree_physical_mismatch=False,
        fail_on_node_tree_fixed_lock_mismatch=True,
        node_tree_tolerance=1e-4,
        parameter_tolerance=1e-4,
        action_json="[0.0]",
        action_sequence_json=None,
        steps=1,
        step_delay_seconds=0.0,
        mechanical_trace_output=trace_output,
    )

    assert "--fail-on-node-tree-fixed-lock-mismatch" in result["command"]
    assert "--mechanical-trace-output" in result["command"]
    assert str(trace_output) in result["command"]
    assert result["returncode"] == 0


def test_report_smoke_wrapper_passes_full_node_tree_restoration_gate(
    tmp_path: Path,
    monkeypatch,
) -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_report", REPORT_TOOL)
    report_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(report_tool)

    smoke_output = tmp_path / "smoke.json"
    normalized_output = tmp_path / "normalized.json"
    normalized_output.write_text("{}", encoding="utf-8")

    def fake_run(command, **kwargs):
        smoke_output.write_text(
            json.dumps(
                {
                    "status": "success",
                    "load_result": {},
                    "mapping_summary": {},
                    "step_summary": {},
                    "node_tree_manifest": {},
                }
            ),
            encoding="utf-8",
        )
        return subprocess.CompletedProcess(command, 0, stdout="", stderr="")

    monkeypatch.setattr(report_tool.subprocess, "run", fake_run)

    result = report_tool._run_godot_smoke(
        repo_root=ROOT,
        normalized_output=normalized_output,
        smoke_output=smoke_output,
        godot_exe="Godot.exe",
        port=19171,
        timeout_seconds=1.0,
        max_endpoint_distance=None,
        max_relative_angle=None,
        min_body_displacement=None,
        max_linear_speed=None,
        min_joint_angle_delta=None,
        min_joint_angle_range=None,
        min_moving_joint_coverage=None,
        min_commanded_joint_response_coverage=None,
        joint_motion_epsilon=0.0,
        min_action_target_coverage=None,
        min_control_action_coverage=None,
        min_nonzero_action_targets=None,
        min_action_transitions=None,
        min_action_transition_delta=None,
        fail_on_joint_limit_violation=False,
        fail_on_incomplete_restoration=False,
        min_restoration_score=None,
        fail_on_parameter_mismatch=False,
        fail_on_control_mismatch=False,
        fail_on_full_mechanical_restoration=False,
        fail_on_action_target_mismatch=False,
        fail_on_action_sequence_target_mismatch=False,
        fail_on_unknown_action_target=False,
        fail_on_invalid_action_target=False,
        fail_on_incomplete_node_tree=False,
        fail_on_full_node_tree_restoration=True,
        fail_on_node_tree_class_mismatch=False,
        fail_on_node_tree_missing_parameters=False,
        fail_on_node_tree_transform_mismatch=False,
        fail_on_node_tree_physical_mismatch=False,
        fail_on_node_tree_fixed_lock_mismatch=False,
        node_tree_tolerance=1e-4,
        parameter_tolerance=1e-4,
        action_json="[0.0]",
        action_sequence_json=None,
        steps=1,
        step_delay_seconds=0.0,
    )

    assert "--fail-on-full-node-tree-restoration" in result["command"]
    assert "--fail-on-node-tree-fixed-lock-mismatch" not in result["command"]
    assert result["returncode"] == 0


def test_report_smoke_wrapper_passes_full_mechanical_restoration_gate(
    tmp_path: Path,
    monkeypatch,
) -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_report", REPORT_TOOL)
    report_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(report_tool)

    smoke_output = tmp_path / "smoke.json"
    normalized_output = tmp_path / "normalized.json"
    normalized_output.write_text("{}", encoding="utf-8")

    def fake_run(command, **kwargs):
        smoke_output.write_text(
            json.dumps(
                {
                    "status": "success",
                    "load_result": {},
                    "mapping_summary": {},
                    "step_summary": {},
                    "node_tree_manifest": {},
                }
            ),
            encoding="utf-8",
        )
        return subprocess.CompletedProcess(command, 0, stdout="", stderr="")

    monkeypatch.setattr(report_tool.subprocess, "run", fake_run)

    result = report_tool._run_godot_smoke(
        repo_root=ROOT,
        normalized_output=normalized_output,
        smoke_output=smoke_output,
        godot_exe="Godot.exe",
        port=19172,
        timeout_seconds=1.0,
        max_endpoint_distance=None,
        max_relative_angle=None,
        min_body_displacement=None,
        max_linear_speed=None,
        min_joint_angle_delta=None,
        min_joint_angle_range=None,
        min_moving_joint_coverage=None,
        min_commanded_joint_response_coverage=None,
        joint_motion_epsilon=0.0,
        min_action_target_coverage=None,
        min_control_action_coverage=None,
        min_nonzero_action_targets=None,
        min_action_transitions=None,
        min_action_transition_delta=None,
        fail_on_joint_limit_violation=False,
        fail_on_incomplete_restoration=False,
        min_restoration_score=None,
        fail_on_parameter_mismatch=False,
        fail_on_control_mismatch=False,
        fail_on_full_mechanical_restoration=True,
        fail_on_action_target_mismatch=False,
        fail_on_action_sequence_target_mismatch=False,
        fail_on_unknown_action_target=False,
        fail_on_invalid_action_target=False,
        fail_on_incomplete_node_tree=False,
        fail_on_full_node_tree_restoration=False,
        fail_on_node_tree_class_mismatch=False,
        fail_on_node_tree_missing_parameters=False,
        fail_on_node_tree_transform_mismatch=False,
        fail_on_node_tree_physical_mismatch=False,
        fail_on_node_tree_fixed_lock_mismatch=False,
        node_tree_tolerance=1e-4,
        parameter_tolerance=1e-4,
        action_json="[0.0]",
        action_sequence_json=None,
        steps=1,
        step_delay_seconds=0.0,
    )

    assert "--fail-on-full-mechanical-restoration" in result["command"]
    assert "--fail-on-full-node-tree-restoration" not in result["command"]
    assert "--fail-on-parameter-mismatch" not in result["command"]
    assert result["returncode"] == 0


def test_dynamic_robot_generation_report_tool_skips_smoke_for_invalid_config(
    tmp_path: Path,
) -> None:
    invalid_config = tmp_path / "invalid_robot.json"
    output_path = tmp_path / "invalid_report.json"
    normalized_path = tmp_path / "invalid_normalized.json"
    gate_output_path = tmp_path / "invalid_gate.json"
    invalid_config.write_text(
        json.dumps(
            {
                "name": "invalid_robot",
                "parts": [
                    {
                        "id": "torso",
                        "type": "torso",
                        "shape": "pyramid",
                        "params": {"mass": 1.0},
                    }
                ],
                "connections": [
                    {
                        "from": "torso",
                        "to": "missing_leg",
                        "joint_type": "hinge",
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(invalid_config),
            "--run-godot-smoke",
            "--output",
            str(output_path),
            "--normalized-output",
            str(normalized_path),
            "--gate-output",
            str(gate_output_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    report = json.loads(output_path.read_text(encoding="utf-8"))
    gate_report = json.loads(gate_output_path.read_text(encoding="utf-8"))
    assert report["status"] == "error"
    assert report["godot_smoke"] is None
    assert report["godot_smoke_skipped_reason"].startswith("validation_failed")
    assert any("shape must be one of" in error for error in report["static"]["errors"])
    assert any("unknown part" in error for error in report["static"]["errors"])
    assert report["delivery_acceptance_gate"]["passed"] is False
    assert report["delivery_acceptance_gate"]["exit_code"] == 1
    assert report["delivery_acceptance_gate"]["required"] is False
    assert report["delivery_acceptance_gate"]["level"] == "incomplete"
    assert report["delivery_acceptance_gate"]["reason_codes"] == [
        "robot_errors",
        "missing_godot_smoke",
        "static_only",
        "missing_dynamic_generation",
        "incomplete_delivery",
        "smoke_failure_reasons",
    ]
    assert report["static"]["node_tree_manifest"]["missing_endpoint_part_ids"] == [
        "missing_leg"
    ]
    assert report["static"]["node_tree_manifest"][
        "missing_endpoint_connection_names"
    ] == ["torso_to_missing_leg"]
    assert report["static"]["node_tree_manifest"]["missing_endpoint_details"] == [
        {
            "connection_name": "torso_to_missing_leg",
            "field": "to",
            "part_id": "missing_leg",
        }
    ]
    assert report["static"]["node_tree_manifest"]["endpoint_paths_complete"] is False
    assert report["delivery_acceptance_gate"]["summary_counts"] == {
        "inputs_count": 1,
        "success_count": 0,
        "error_count": 1,
        "live_smoke_count": 0,
        "smoke_report_written_count": 0,
        "smoke_report_missing_count": 0,
        "smoke_report_read_error_count": 0,
        "delivery_godot_verified_count": 0,
        "delivery_static_only_count": 1,
        "delivery_unverified_count": 0,
        "delivery_dynamic_generation_count": 0,
        "delivery_complete_count": 0,
        "delivery_incomplete_count": 1,
        "delivery_parameters_incomplete_count": 0,
        "fixed_lock_checked_count": 0,
        "fixed_lock_mismatch_count": 0,
        "control_configured_count": 0,
        "control_readback_checked_count": 0,
        "control_readback_missing_count": 0,
        "node_tree_fixed_lock_checked_count": 0,
        "node_tree_fixed_lock_mismatch_count": 0,
        "node_tree_fixed_locks_complete_count": 0,
        "node_tree_fixed_locks_incomplete_count": 0,
        "node_tree_gate_enabled_count": 0,
        "node_tree_full_restoration_required_count": 0,
        "node_tree_full_restoration_not_required_count": 0,
        "node_tree_gate_check_counts": {
            "incomplete_node_tree": 0,
            "class_mismatch": 0,
            "missing_parameters": 0,
            "transform_mismatch": 0,
            "physical_mismatch": 0,
            "fixed_lock_mismatch": 0,
        },
        "static_topology_complete_count": 1,
        "static_topology_incomplete_count": 0,
        "static_topology_disconnected_parts_count": 0,
        "static_topology_unreachable_parts_count": 0,
        "static_topology_duplicate_child_endpoint_count": 0,
        "static_topology_cycle_count": 0,
        "static_node_tree_manifest_count": 1,
        "static_node_tree_manifest_valid_count": 1,
        "static_node_tree_manifest_invalid_count": 0,
        "static_node_tree_manifest_error_count": 0,
        "static_node_tree_manifest_output_count": 0,
        "static_node_tree_manifest_path_map_mismatch_count": 0,
        "static_node_tree_manifest_path_map_mismatch_kind_counts": {},
        "static_node_tree_parts_planned_count": 1,
        "static_node_tree_joints_planned_count": 1,
        "static_node_tree_parameterized_joints_count": 1,
        "static_node_tree_complete_count": 0,
        "static_node_tree_incomplete_count": 1,
        "static_node_tree_endpoint_paths_complete_count": 0,
        "static_node_tree_endpoint_paths_incomplete_count": 1,
        "static_node_tree_missing_endpoint_parts_count": 1,
        "static_node_tree_missing_endpoint_connections_count": 1,
        "static_node_tree_parameters_complete_count": 1,
        "static_node_tree_parameters_incomplete_count": 0,
        "mechanical_gate_enabled_count": 0,
        "full_mechanical_restoration_required_count": 0,
        "full_mechanical_restoration_not_required_count": 0,
        "mechanical_gate_check_counts": {
            "mechanical_restoration": 0,
            "joint_parameter_readback": 0,
            "control_parameter_readback": 0,
            "full_node_tree_restoration": 0,
        },
        "mechanical_behavior_evidence_count": 0,
        "mechanical_behavior_complete_count": 0,
        "mechanical_behavior_incomplete_count": 0,
        "mechanical_behavior_residual_risk_count": 0,
        "mechanical_behavior_threshold_failure_count": 0,
        "mechanical_behavior_center_of_mass_available_count": 0,
        "mechanical_behavior_contact_state_available_count": 0,
        "mechanical_behavior_step_trace_artifact_count": 0,
        "failure_reasons_count": 3,
    }
    assert gate_report == report["delivery_acceptance_gate"]
    assert "static" not in gate_report
    assert "delivery_acceptance_summary" not in gate_report
    assert "delivery_acceptance_gate failed " in result.stderr
    assert (
        "counts=inputs:1,errors:1,live:0,smokereports:0/0/0,verified:0,"
        "static:1,staticmanifest:1/0,fixed:0/0,control:0/0/0,treefixed:0/0,"
        "treefixedok:0/0,treegate:0/0,mechgate:0/0,mechbehavior:0/0,"
        "failures:3"
    ) in result.stderr
    assert (
        "topology=complete:1/1,incomplete:0,disconnected:0,unreachable:0,"
        "duplicates:0,cycles:0,roots:torso"
    ) in result.stderr
    assert "reason_codes=robot_errors,missing_godot_smoke,static_only" in result.stderr
    assert f"affected_inputs={invalid_config}" in result.stderr


def test_dynamic_robot_generation_report_can_require_full_mechanical_gate(
    tmp_path: Path,
) -> None:
    output_path = tmp_path / "strict_report.json"
    gate_output_path = tmp_path / "strict_gate.json"

    result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--require-full-mechanical-restoration-gate",
            "--output",
            str(output_path),
            "--gate-output",
            str(gate_output_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    report = json.loads(output_path.read_text(encoding="utf-8"))
    gate_report = json.loads(gate_output_path.read_text(encoding="utf-8"))
    assert report["status"] == "success"
    assert report["delivery_acceptance_gate"]["required"] is False
    assert report["delivery_acceptance_gate"]["acceptance_profile"] == "custom"
    assert report["delivery_acceptance_gate"]["acceptance_requirements"] == (
        _acceptance_requirements(full_mechanical_restoration_gate=True)
    )
    assert (
        report["delivery_acceptance_gate"][
            "requires_full_mechanical_restoration_gate"
        ]
        is True
    )
    assert report["delivery_acceptance_gate"]["passed"] is False
    assert report["delivery_acceptance_gate"]["exit_code"] == 1
    assert "missing_full_mechanical_restoration_gate" in report[
        "delivery_acceptance_gate"
    ]["reason_codes"]
    assert gate_report == report["delivery_acceptance_gate"]
    assert "requirements=full_mechanical_restoration_gate" in result.stderr
    assert "reason_codes=missing_godot_smoke,static_only,missing_full_mechanical_restoration_gate" in result.stderr


def test_dynamic_godot_smoke_dry_run_records_live_profile_discovery(
    tmp_path: Path,
) -> None:
    output_path = tmp_path / "dry_run_smoke.json"
    artifact_root = tmp_path / "live_artifacts"

    result = subprocess.run(
        [
            sys.executable,
            str(SMOKE_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--dry-run-discovery",
            "--godot-exe",
            str(tmp_path / "missing-godot.exe"),
            "--live-profile",
            "manual_ci",
            "--live-artifact-root",
            str(artifact_root),
            "--live-retention-days",
            "5",
            "--flaky-retry-attempts",
            "2",
            "--output",
            str(output_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    live = payload["live_verification"]
    assert payload["status"] == "blocked"
    assert payload["errors"] == [
        "live verification discovery failed: missing_godot_executable"
    ]
    assert live["profile_version"] == "dynamic_godot_live_verification_profile.v1"
    assert live["profile_name"] == "manual_ci"
    assert live["environment_mode"] == "manual_ci"
    assert live["ci_mode"] is True
    assert live["scheduled"] is False
    assert live["artifact_retention"]["artifact_root"] == str(artifact_root)
    assert live["artifact_retention"]["retention_days"] == 5
    assert live["flaky_policy"]["retry_attempts"] == 2
    assert live["flaky_policy"]["classification"] == "not_retried"
    assert live["godot_executable"]["failure_category"] == (
        "missing_godot_executable"
    )
    assert live["failure_category"] == "missing_godot_executable"
    assert live["dry_run"] is True


def test_dynamic_godot_report_passes_live_profile_to_smoke_runner(
    tmp_path: Path,
) -> None:
    output_path = tmp_path / "report.json"
    gate_output_path = tmp_path / "gate.json"
    smoke_output_path = tmp_path / "smoke.json"
    artifact_root = tmp_path / "manual_ci_artifacts"

    result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--run-godot-smoke",
            "--godot-exe",
            str(tmp_path / "missing-godot.exe"),
            "--live-profile",
            "manual_ci",
            "--live-artifact-root",
            str(artifact_root),
            "--live-retention-days",
            "9",
            "--flaky-retry-attempts",
            "3",
            "--smoke-output",
            str(smoke_output_path),
            "--output",
            str(output_path),
            "--gate-output",
            str(gate_output_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    report = json.loads(output_path.read_text(encoding="utf-8"))
    smoke = json.loads(smoke_output_path.read_text(encoding="utf-8"))
    live = report["godot_smoke"]["live_verification"]
    assert smoke["live_verification"] == live
    assert live["profile_name"] == "manual_ci"
    assert live["artifact_retention"]["artifact_root"] == str(artifact_root)
    assert live["artifact_retention"]["retention_days"] == 9
    assert live["flaky_policy"]["retry_attempts"] == 3
    assert live["failure_category"] == "missing_godot_executable"
    assert report["godot_smoke"]["report_written"] is True
    assert report["delivery_acceptance_gate"]["summary_counts"][
        "smoke_report_written_count"
    ] == 1
    assert report["delivery_acceptance_gate"]["summary_counts"][
        "smoke_report_missing_count"
    ] == 0
    assert report["delivery_acceptance_gate"]["summary_counts"][
        "delivery_godot_verified_count"
    ] == 0


def test_dynamic_robot_generation_report_full_mechanical_acceptance_ignores_stale_smoke_output(
    tmp_path: Path,
) -> None:
    output_path = tmp_path / "full_acceptance_report.json"
    gate_output_path = tmp_path / "full_acceptance_gate.json"
    summary_path = tmp_path / "validation_summary.json"
    fail_on_summary_path = tmp_path / "fail_on_validation_summary.json"
    smoke_output_path = tmp_path / "stale_smoke.json"
    smoke_output_path.write_text(
        json.dumps(
            {
                "status": "success",
                "load_result": {
                    "complete": True,
                    "expected_parts": 99,
                    "parts_created": 99,
                    "parts_complete": True,
                    "expected_joints": 99,
                    "joints_created": 99,
                    "failed_joints": 0,
                    "joints_complete": True,
                    "parameterized_joints": 99,
                    "parameters_complete": True,
                },
                "mapping_summary": {"part_nodes": 99, "joint_nodes": 99},
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--full-mechanical-restoration-acceptance",
            "--godot-exe",
            str(tmp_path / "missing-godot.exe"),
            "--timeout-seconds",
            "0.1",
            "--smoke-output",
            str(smoke_output_path),
            "--output",
            str(output_path),
            "--gate-output",
            str(gate_output_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    report = json.loads(output_path.read_text(encoding="utf-8"))
    gate_report = json.loads(gate_output_path.read_text(encoding="utf-8"))
    smoke_report = json.loads(smoke_output_path.read_text(encoding="utf-8"))
    assert report["godot_smoke"]["returncode"] == 1
    assert report["godot_smoke"]["report_written"] is True
    assert report["godot_smoke"]["report_read_error"] is None
    assert smoke_report["live_verification"]["failure_category"] == (
        "missing_godot_executable"
    )
    assert report["godot_smoke"]["live_verification"] == smoke_report[
        "live_verification"
    ]
    assert report["delivery_contract_preview"]["parts_created"] is None
    assert report["delivery_acceptance_gate"]["required"] is True
    assert report["delivery_acceptance_gate"]["acceptance_profile"] == (
        "full_mechanical_restoration"
    )
    assert report["delivery_acceptance_gate"]["acceptance_requirements"] == (
        _acceptance_requirements(
            run_godot_smoke=True,
            godot_verified_acceptance=True,
            full_mechanical_restoration_gate=True,
            full_mechanical_restoration_smoke_gate=True,
            mechanical_restoration_complete=True,
            joint_parameter_readback=True,
            control_parameter_readback=True,
            full_node_tree_restoration=True,
            node_tree_complete=True,
            node_tree_class_match=True,
            node_tree_parameters_applied=True,
            node_tree_transform_match=True,
            node_tree_physical_match=True,
            node_tree_fixed_lock_match=True,
        )
    )
    assert report["delivery_acceptance_gate"]["passed"] is False
    assert report["delivery_acceptance_gate"]["summary_counts"]["live_smoke_count"] == 1
    assert (
        report["delivery_acceptance_gate"]["summary_counts"][
            "smoke_report_written_count"
        ]
        == 1
    )
    assert (
        report["delivery_acceptance_gate"]["summary_counts"][
            "smoke_report_missing_count"
        ]
        == 0
    )
    assert (
        report["delivery_acceptance_gate"]["summary_counts"][
            "smoke_report_read_error_count"
        ]
        == 0
    )
    assert (
        report["delivery_acceptance_gate"]["summary_counts"][
            "delivery_godot_verified_count"
        ]
        == 0
    )
    assert (
        report["delivery_acceptance_gate"]["summary_counts"][
            "delivery_dynamic_generation_count"
        ]
        == 0
    )
    assert (
        report["delivery_acceptance_gate"]["summary_counts"]["failure_reasons_count"]
        == 2
    )
    assert "missing_godot_smoke_report" not in report["delivery_acceptance_gate"][
        "reason_codes"
    ]
    assert "smoke_failure_reasons" in report["delivery_acceptance_gate"]["reason_codes"]
    assert gate_report == report["delivery_acceptance_gate"]
    assert "missing_godot_executable" in json.dumps(report)
    assert "requirements=run_godot_smoke,godot_verified_acceptance" in result.stderr

    validate_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_output_path),
            "--expect-smoke-report-written-count",
            "1",
            "--expect-smoke-report-missing-count",
            "0",
            "--expect-smoke-report-read-error-count",
            "0",
            "--summary-output",
            str(summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 0
    payload = json.loads(validate_result.stdout)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["smoke_report_written_count"] == 1
    assert payload["smoke_report_missing_count"] == 0
    assert payload["smoke_report_read_error_count"] == 0
    assert payload["expected_smoke_report_written_count"] == 1
    assert payload["expected_smoke_report_missing_count"] == 0
    assert payload["expected_smoke_report_read_error_count"] == 0
    assert payload["errors"] == []
    assert summary["errors"] == payload["errors"]

    fail_on_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_output_path),
            "--fail-on-smoke-report-missing",
            "--fail-on-smoke-report-read-error",
            "--summary-output",
            str(fail_on_summary_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert fail_on_result.returncode == 0
    fail_on_payload = json.loads(fail_on_result.stdout)
    fail_on_summary = json.loads(fail_on_summary_path.read_text(encoding="utf-8"))
    assert fail_on_payload["smoke_report_missing_count"] == 0
    assert fail_on_payload["smoke_report_read_error_count"] == 0
    assert fail_on_payload["fail_on_smoke_report_missing"] is True
    assert fail_on_payload["fail_on_smoke_report_read_error"] is True
    assert fail_on_payload["errors"] == []
    assert fail_on_summary["errors"] == fail_on_payload["errors"]


def test_dynamic_robot_generation_report_marks_invalid_smoke_report_json(
    tmp_path: Path,
    monkeypatch,
) -> None:
    spec = importlib.util.spec_from_file_location("dynamic_godot_report", REPORT_TOOL)
    report_tool = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(report_tool)
    normalized_output = tmp_path / "normalized.json"
    smoke_output = tmp_path / "bad_smoke.json"
    gate_output = tmp_path / "gate.json"
    normalized_output.write_text("{}", encoding="utf-8")
    real_subprocess_run = subprocess.run

    def fake_run(command, **kwargs):
        smoke_output.write_text("{bad json", encoding="utf-8")
        return SimpleNamespace(
            returncode=1,
            stdout="",
            stderr="smoke failed",
        )

    monkeypatch.setattr(report_tool.subprocess, "run", fake_run)

    smoke_result = report_tool._run_godot_smoke(
        repo_root=ROOT,
        normalized_output=normalized_output,
        smoke_output=smoke_output,
        godot_exe="godot",
        port=19170,
        timeout_seconds=0.1,
        max_endpoint_distance=None,
        max_relative_angle=None,
        min_body_displacement=None,
        max_linear_speed=None,
        min_joint_angle_delta=None,
        min_joint_angle_range=None,
        min_moving_joint_coverage=None,
        min_commanded_joint_response_coverage=None,
        joint_motion_epsilon=0.0,
        min_action_target_coverage=None,
        min_control_action_coverage=None,
        min_nonzero_action_targets=None,
        min_action_transitions=None,
        min_action_transition_delta=None,
        fail_on_joint_limit_violation=False,
        fail_on_incomplete_restoration=False,
        min_restoration_score=None,
        fail_on_parameter_mismatch=False,
        fail_on_control_mismatch=False,
        fail_on_full_mechanical_restoration=True,
        fail_on_action_target_mismatch=False,
        fail_on_action_sequence_target_mismatch=False,
        fail_on_unknown_action_target=False,
        fail_on_invalid_action_target=False,
        fail_on_incomplete_node_tree=False,
        fail_on_full_node_tree_restoration=False,
        fail_on_node_tree_class_mismatch=False,
        fail_on_node_tree_missing_parameters=False,
        fail_on_node_tree_transform_mismatch=False,
        fail_on_node_tree_physical_mismatch=False,
        fail_on_node_tree_fixed_lock_mismatch=False,
        node_tree_tolerance=0.0001,
        parameter_tolerance=0.0001,
        action_json="[0.0]",
        action_sequence_json=None,
        steps=1,
        step_delay_seconds=0.0,
    )

    assert smoke_result["report_written"] is True
    assert smoke_result["report_read_error"] == (
        "Expecting property name enclosed in double quotes at line 1 column 2"
    )
    assert smoke_result["execution_failure_reasons"] == [
        "godot smoke exited with returncode 1",
        (
            "godot smoke report could not be read: Expecting property name "
            "enclosed in double quotes at line 1 column 2"
        ),
    ]
    assert smoke_result["report_summary"]["status"] is None

    report = {
        "status": "error",
        "static": {
            "input": str(FIXED_PAIR_FIXTURE),
            "robot_name": "bad_smoke_report",
            "parts_count": 2,
            "connections_count": 1,
            "errors": [],
        },
        "delivery_contract_preview": report_tool._build_live_delivery_summary({}),
        "godot_smoke": smoke_result,
    }
    batch_summary = report_tool._build_batch_summary([report])
    gate = report_tool._build_delivery_acceptance_gate(
        required=True,
        report_status="error",
        batch_summary=batch_summary,
    )

    assert gate["summary_counts"]["smoke_report_written_count"] == 1
    assert gate["summary_counts"]["smoke_report_missing_count"] == 0
    assert gate["summary_counts"]["smoke_report_read_error_count"] == 1
    assert "invalid_godot_smoke_report" in gate["reason_codes"]
    assert "smoke_failure_reasons" in gate["reason_codes"]
    gate_output.write_text(json.dumps(gate), encoding="utf-8")

    validate_result = real_subprocess_run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_output),
            "--fail-on-smoke-report-read-error",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_result.returncode == 1
    payload = json.loads(validate_result.stdout)
    assert payload["smoke_report_written_count"] == 1
    assert payload["smoke_report_missing_count"] == 0
    assert payload["smoke_report_read_error_count"] == 1
    assert payload["fail_on_smoke_report_read_error"] is True
    assert payload["errors"] == ["smoke_report_read_error_count present: 1"]


def test_delivery_acceptance_gate_validator_records_smoke_report_fail_policy_when_clean(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr

    validate_text_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--fail-on-smoke-report-missing",
            "--fail-on-smoke-report-read-error",
            "--format",
            "text",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_text_result.returncode == 0, validate_text_result.stderr
    lines = validate_text_result.stdout.splitlines()
    assert len(lines) == 2
    assert lines[0].startswith("delivery_acceptance_gate validation summary")
    assert "status=success" in lines[0]
    assert "smoke_reports=written:0,missing:0,read_error:0" in lines[0]
    assert "fail_on_smoke_report_missing=true" in lines[0]
    assert "fail_on_smoke_report_read_error=true" in lines[0]
    assert "counts=inputs:1,errors:0,live:0,smokereports:0/0/0" in lines[1]

    validate_json_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--fail-on-smoke-report-missing",
            "--fail-on-smoke-report-read-error",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert validate_json_result.returncode == 0, validate_json_result.stderr
    payload = json.loads(validate_json_result.stdout)
    assert payload["status"] == "success"
    assert payload["smoke_report_written_count"] == 0
    assert payload["smoke_report_missing_count"] == 0
    assert payload["smoke_report_read_error_count"] == 0
    assert payload["fail_on_smoke_report_missing"] is True
    assert payload["fail_on_smoke_report_read_error"] is True
    assert payload["results"][0]["summary_counts"]["smoke_report_missing_count"] == 0


def test_delivery_acceptance_gate_validator_can_fail_on_control_readback_missing(
    tmp_path: Path,
) -> None:
    gate_path = tmp_path / "gate.json"

    report_result = subprocess.run(
        [
            sys.executable,
            str(REPORT_TOOL),
            str(FIXED_PAIR_FIXTURE),
            "--gate-output",
            str(gate_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert report_result.returncode == 0, report_result.stderr
    gate = json.loads(gate_path.read_text(encoding="utf-8"))
    gate["summary_counts"]["control_configured_count"] = 3
    gate["summary_counts"]["control_readback_checked_count"] = 1
    gate["summary_counts"]["control_readback_missing_count"] = 2
    gate_path.write_text(json.dumps(gate), encoding="utf-8")

    expected_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--expect-control-configured-count",
            "3",
            "--expect-control-readback-checked-count",
            "1",
            "--expect-control-readback-missing-count",
            "0",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert expected_result.returncode == 1
    expected_payload = json.loads(expected_result.stdout)
    assert expected_payload["control_configured_count"] == 3
    assert expected_payload["control_readback_checked_count"] == 1
    assert expected_payload["control_readback_missing_count"] == 2
    assert expected_payload["expected_control_configured_count"] == 3
    assert expected_payload["expected_control_readback_checked_count"] == 1
    assert expected_payload["expected_control_readback_missing_count"] == 0
    assert expected_payload["errors"] == [
        "expected control_readback_missing_count 0 but found 2"
    ]

    text_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--fail-on-control-readback-missing",
            "--format",
            "text",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert text_result.returncode == 1
    lines = text_result.stdout.splitlines()
    assert len(lines) == 2
    assert "control_readback=configured:3,checked:1,missing:2" in lines[0]
    assert "fail_on_control_readback_missing=true" in lines[0]
    assert "counts=inputs:1,errors:0,live:0,smokereports:0/0/0,control:1/2/3" in (
        lines[1]
    )

    fail_on_result = subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR_TOOL),
            str(gate_path),
            "--fail-on-control-readback-missing",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert fail_on_result.returncode == 1
    fail_on_payload = json.loads(fail_on_result.stdout)
    assert fail_on_payload["control_readback_missing_count"] == 2
    assert fail_on_payload["fail_on_control_readback_missing"] is True
    assert fail_on_payload["errors"] == [
        "control_readback_missing_count present: 2"
    ]
