"""Robot JSON normalization for dynamic Godot assembly."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from copy import deepcopy
from typing import Any

ROBOT_MECHANICAL_SCHEMA_VERSION = "1.5"
GODOT_NODE_TREE_MANIFEST_VERSION = "godot_node_tree_manifest.v1"
SUPPORTED_ROBOT_MECHANICAL_SCHEMA_VERSIONS = ("1.1", "1.2", "1.3", "1.4", "1.5")
SUPPORTED_GODOT_NODE_TREE_MANIFEST_VERSIONS = (GODOT_NODE_TREE_MANIFEST_VERSION,)

SUPPORTED_GODOT_SHAPES = {"box", "capsule", "cylinder", "sphere"}
SUPPORTED_GODOT_JOINTS = {"fixed", "hinge", "revolute", "prismatic", "slider"}

DEFAULT_BOX_SIZE = [0.2, 0.2, 0.2]
DEFAULT_AXIS = [0.0, 1.0, 0.0]
DEFAULT_LIMITS = {"lower": -1.57, "upper": 1.57}
DEFAULT_FIXED_LIMITS = {"lower": 0.0, "upper": 0.0}
DEFAULT_MOTOR = {"enabled": False, "target_velocity": 0.0, "max_impulse": 100.0}
DEFAULT_DYNAMICS = {"damping": 0.1, "friction": 0.01}


def godot_robot_generation_compatibility_matrix() -> dict[str, Any]:
    """Return supported schema and static manifest versions for dynamic Godot generation."""
    return {
        "robot_mechanical_schema_current": ROBOT_MECHANICAL_SCHEMA_VERSION,
        "robot_mechanical_schema_versions": list(
            SUPPORTED_ROBOT_MECHANICAL_SCHEMA_VERSIONS
        ),
        "godot_node_tree_manifest_current": GODOT_NODE_TREE_MANIFEST_VERSION,
        "godot_node_tree_manifest_versions": list(
            SUPPORTED_GODOT_NODE_TREE_MANIFEST_VERSIONS
        ),
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


def normalize_robot_config_for_godot(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Return a Godot-ready robot config while preserving source fields."""
    config = deepcopy(dict(payload))
    config["schema_version"] = str(
        config.get("schema_version") or ROBOT_MECHANICAL_SCHEMA_VERSION
    )
    config["metadata"] = dict(config.get("metadata") or {})
    parts, connections = _expand_compound_legs(
        config.get("parts", []),
        config.get("connections", []),
    )
    config["parts"] = [_normalize_part(part) for part in parts]
    config["connections"] = [
        _normalize_connection(conn) for conn in connections
    ]
    return config


def _expand_compound_legs(
    parts: Any,
    connections: Any,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Split legacy compound leg parts into upper/lower mechanical links."""
    if not isinstance(parts, list):
        return [], list(connections) if isinstance(connections, list) else []

    compound_leg_ids = {
        part.get("id")
        for part in parts
        if isinstance(part, Mapping) and _is_compound_leg(part)
    }
    if not compound_leg_ids:
        return list(parts), list(connections) if isinstance(connections, list) else []

    expanded_parts: list[dict[str, Any]] = []
    segment_map: dict[str, tuple[str, str, float, float]] = {}
    for part in parts:
        if isinstance(part, Mapping) and part.get("id") in compound_leg_ids:
            upper, lower = _compound_leg_segments(part)
            expanded_parts.extend([upper, lower])
            segment_map[str(part["id"])] = (
                upper["id"],
                lower["id"],
                float(upper["params"]["length"]),
                float(lower["params"]["length"]),
            )
        elif isinstance(part, Mapping):
            expanded_parts.append(deepcopy(dict(part)))

    _position_compound_leg_segments(expanded_parts, connections, segment_map)
    expanded_connections = _compound_leg_connections(connections, segment_map)
    return expanded_parts, expanded_connections


def _is_compound_leg(part: Mapping[str, Any]) -> bool:
    params = part.get("params")
    return (
        part.get("type") == "leg"
        and isinstance(params, Mapping)
        and any(key in params for key in ["thigh_length", "upper_length"])
        and any(key in params for key in ["shin_length", "lower_length"])
    )


def _compound_leg_segments(
    part: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    part_id = str(part["id"])
    side = str(part.get("side") or part_id)
    params = dict(part.get("params") or {})
    upper_length = float(params.get("thigh_length", params.get("upper_length", 0.3)))
    lower_length = float(params.get("shin_length", params.get("lower_length", 0.3)))
    upper_mass = float(params.get("thigh_mass", params.get("upper_mass", 0.5)))
    lower_mass = float(params.get("shin_mass", params.get("lower_mass", 0.5)))
    radius = float(params.get("radius", 0.04))
    return (
        _leg_segment(part_id, side, "upper", upper_length, upper_mass, radius),
        _leg_segment(part_id, side, "lower", lower_length, lower_mass, radius),
    )


def _leg_segment(
    part_id: str,
    side: str,
    segment: str,
    length: float,
    mass: float,
    radius: float,
) -> dict[str, Any]:
    return {
        "id": f"{part_id}_{segment}",
        "type": "thigh" if segment == "upper" else "shin",
        "side": side,
        "shape": "capsule",
        "params": {
            "mass": mass,
            "length": length,
            "radius": radius,
            "position": [0.0, 0.0, 0.0],
            "rotation": [0.0, 0.0, 0.0],
            "source_compound_part": part_id,
        },
    }


def _position_compound_leg_segments(
    expanded_parts: list[dict[str, Any]],
    connections: Any,
    segment_map: Mapping[str, tuple[str, str, float, float]],
) -> None:
    if not isinstance(connections, list):
        return
    part_index = {
        str(part.get("id")): part
        for part in expanded_parts
        if isinstance(part, Mapping) and part.get("id") is not None
    }
    for connection in connections:
        if not isinstance(connection, Mapping) or connection.get("to") not in segment_map:
            continue
        upper_id, lower_id, upper_length, lower_length = segment_map[
            str(connection["to"])
        ]
        hip_origin = _vector3_list(connection.get("origin", connection.get("offset")))
        _set_part_position(
            part_index.get(upper_id),
            [hip_origin[0], hip_origin[1] - upper_length / 2.0, hip_origin[2]],
        )
        _set_part_position(
            part_index.get(lower_id),
            [
                hip_origin[0],
                hip_origin[1] - upper_length - lower_length / 2.0,
                hip_origin[2],
            ],
        )


def _set_part_position(part: Any, position: list[float]) -> None:
    if not isinstance(part, dict):
        return
    params = part.setdefault("params", {})
    if isinstance(params, dict):
        params["position"] = [round(value, 10) for value in position]


def _compound_leg_connections(
    connections: Any,
    segment_map: Mapping[str, tuple[str, str, float, float]],
) -> list[dict[str, Any]]:
    if not isinstance(connections, list):
        return []
    expanded: list[dict[str, Any]] = []
    for connection in connections:
        if not isinstance(connection, Mapping):
            continue
        child_id = connection.get("to")
        if child_id in segment_map:
            expanded.extend(_split_compound_leg_connection(connection, segment_map))
        else:
            expanded.append(_retarget_compound_leg_connection(connection, segment_map))
    return expanded


def _split_compound_leg_connection(
    connection: Mapping[str, Any],
    segment_map: Mapping[str, tuple[str, str, float, float]],
) -> list[dict[str, Any]]:
    upper_id, lower_id, upper_length, _lower_length = segment_map[str(connection["to"])]
    hip_origin = _vector3_list(connection.get("origin", connection.get("offset")))
    hip = deepcopy(dict(connection))
    hip.update(
        {
            "to": upper_id,
            "name": str(connection.get("name") or f"hip_{connection['to']}"),
            "origin": hip_origin,
        }
    )
    knee = {
        "from": upper_id,
        "to": lower_id,
        "joint_type": connection.get("joint_type", "hinge"),
        "name": f"knee_{connection['to']}",
        "origin": [hip_origin[0], hip_origin[1] - upper_length, hip_origin[2]],
        "axis": connection.get("axis", DEFAULT_AXIS.copy()),
        "limits": _default_limits_for_joint_type(connection.get("joint_type", "hinge")),
        "motor": DEFAULT_MOTOR.copy(),
        "dynamics": DEFAULT_DYNAMICS.copy(),
    }
    return [hip, knee]


def _retarget_compound_leg_connection(
    connection: Mapping[str, Any],
    segment_map: Mapping[str, tuple[str, str, float, float]],
) -> dict[str, Any]:
    item = deepcopy(dict(connection))
    if item.get("from") in segment_map:
        item["from"] = segment_map[str(item["from"])][1]
    if item.get("to") in segment_map:
        item["to"] = segment_map[str(item["to"])][0]
    return item


def validate_godot_robot_config(payload: Any) -> list[str]:
    """Validate the fields required to assemble a Godot mechanical node tree."""
    if not isinstance(payload, Mapping):
        return ["robot config must be an object"]

    errors: list[str] = []
    _validate_schema_version(payload.get("schema_version"), errors)
    part_ids = _validate_parts(payload.get("parts"), errors)
    connection_names = _validate_connections(payload.get("connections"), part_ids, errors)
    _validate_control(payload.get("control"), connection_names, errors)
    return errors


def build_mechanical_topology_summary(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Return a machine-readable topology summary for Godot assembly checks."""
    parts = payload.get("parts")
    connections = payload.get("connections")
    part_ids = (
        [
            str(part["id"])
            for part in parts
            if isinstance(part, Mapping) and _is_non_empty_string(part.get("id"))
        ]
        if isinstance(parts, list)
        else []
    )
    part_id_set = set(part_ids)
    edges = _valid_connection_edges(connections, part_id_set)
    connected_parts = (
        set(part_id_set)
        if len(part_id_set) <= 1
        else {part for edge in edges for part in edge[:2]}
    )
    child_parts = {child for _parent, child, _prefix in edges}
    root_parts = sorted(part_id_set - child_parts)
    adjacency: dict[str, list[str]] = {part_id: [] for part_id in part_id_set}
    for parent, child, _prefix in edges:
        adjacency.setdefault(parent, []).append(child)
    reachable = (
        _reachable_parts(root_parts[0], adjacency)
        if len(root_parts) == 1
        else set()
    )
    cycle = _first_connection_cycle(edges)
    duplicate_child_endpoints = _duplicate_child_endpoints(edges)
    disconnected_parts = sorted(part_id_set - connected_parts)
    unreachable_parts = sorted(part_id_set - reachable) if len(root_parts) == 1 else []
    return {
        "schema_version": str(
            payload.get("schema_version") or ROBOT_MECHANICAL_SCHEMA_VERSION
        ),
        "parts_count": len(part_id_set),
        "connections_count": len(edges),
        "root_parts": root_parts,
        "root_part": root_parts[0] if len(root_parts) == 1 else None,
        "connected_parts_count": len(connected_parts),
        "reachable_parts_count": len(reachable),
        "disconnected_parts": disconnected_parts,
        "unreachable_parts": unreachable_parts,
        "duplicate_child_endpoints": duplicate_child_endpoints,
        "cycle": cycle,
        "complete_tree": (
            len(part_id_set) <= 1
            or (
                len(root_parts) == 1
                and not disconnected_parts
                and not unreachable_parts
                and not duplicate_child_endpoints
                and not cycle
            )
        ),
    }


def build_godot_node_tree_manifest(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Return the static Godot node tree that the dynamic assembler should build."""
    robot_name = str(payload.get("name") or "GeneratedRobot")
    parts = payload.get("parts")
    connections = payload.get("connections")
    part_ids = {
        str(part["id"])
        for part in parts
        if isinstance(parts, list)
        and isinstance(part, Mapping)
        and _is_non_empty_string(part.get("id"))
    }
    part_nodes = [
        _planned_part_node(part, robot_name)
        for part in parts
        if isinstance(part, Mapping) and _is_non_empty_string(part.get("id"))
    ] if isinstance(parts, list) else []
    joint_nodes = [
        _planned_joint_node(connection, robot_name, part_ids)
        for connection in connections
        if isinstance(connection, Mapping)
    ] if isinstance(connections, list) else []
    parameterized_joints = sum(
        1 for joint in joint_nodes if joint.get("applied_parameters")
    )
    missing_endpoint_part_ids = sorted(
        {
            part_id
            for joint in joint_nodes
            for part_id in joint.get("missing_endpoint_part_ids", [])
        }
    )
    missing_endpoint_connection_names = [
        str(joint["connection_name"])
        for joint in joint_nodes
        if joint.get("missing_endpoint_part_ids")
    ]
    missing_endpoint_details = [
        {
            "connection_name": str(joint["connection_name"]),
            "field": str(detail["field"]),
            "part_id": str(detail["part_id"]),
        }
        for joint in joint_nodes
        for detail in joint.get("missing_endpoint_details", [])
        if isinstance(detail, Mapping)
    ]
    endpoint_paths_complete = not missing_endpoint_part_ids
    parameters_complete = parameterized_joints == len(joint_nodes)
    part_node_paths = {
        str(part["part_id"]): {
            "body_node": part["body_node"],
            "collision_node": part["collision_node"],
            "mesh_node": part["mesh_node"],
        }
        for part in part_nodes
    }
    joint_node_paths = {
        str(joint["connection_name"]): {
            "joint_node": joint["joint_node"],
            "node_a": joint["node_a"],
            "node_b": joint["node_b"],
        }
        for joint in joint_nodes
    }
    path_maps_complete = (
        len(part_node_paths) == len(part_nodes)
        and len(joint_node_paths) == len(joint_nodes)
    )
    return {
        "manifest_version": GODOT_NODE_TREE_MANIFEST_VERSION,
        "schema_version": str(
            payload.get("schema_version") or ROBOT_MECHANICAL_SCHEMA_VERSION
        ),
        "robot_name": robot_name,
        "robot_node": robot_name,
        "controller_node": f"{robot_name}/GeneratedRobotController",
        "parts_count": len(part_nodes),
        "joints_count": len(joint_nodes),
        "parameterized_joints": parameterized_joints,
        "parameters_complete": parameters_complete,
        "endpoint_paths_complete": endpoint_paths_complete,
        "path_maps_complete": path_maps_complete,
        "missing_endpoint_part_ids": missing_endpoint_part_ids,
        "missing_endpoint_connection_names": missing_endpoint_connection_names,
        "missing_endpoint_details": missing_endpoint_details,
        "complete": parameters_complete and endpoint_paths_complete,
        "part_node_paths": part_node_paths,
        "joint_node_paths": joint_node_paths,
        "part_nodes": part_nodes,
        "joint_nodes": joint_nodes,
    }


def validate_godot_node_tree_manifest(payload: Any) -> list[str]:
    """Validate the static Godot node-tree manifest emitted for a robot JSON."""
    errors: list[str] = []
    if not isinstance(payload, Mapping):
        return ["node_tree_manifest must be an object"]

    if payload.get("manifest_version") != GODOT_NODE_TREE_MANIFEST_VERSION:
        errors.append(
            "node_tree_manifest.manifest_version must be "
            f"{GODOT_NODE_TREE_MANIFEST_VERSION!r}"
        )
    for field in ["robot_name", "robot_node", "controller_node"]:
        if not _is_non_empty_string(payload.get(field)):
            errors.append(f"node_tree_manifest.{field} must be a non-empty string")
    _validate_manifest_root_nodes(payload, errors)

    part_nodes = payload.get("part_nodes")
    joint_nodes = payload.get("joint_nodes")
    if not isinstance(part_nodes, list):
        errors.append("node_tree_manifest.part_nodes must be a list")
        part_nodes = []
    if not isinstance(joint_nodes, list):
        errors.append("node_tree_manifest.joint_nodes must be a list")
        joint_nodes = []
    _validate_manifest_unique_values(
        part_nodes,
        field="part_id",
        prefix="node_tree_manifest.part_nodes",
        errors=errors,
    )
    _validate_manifest_unique_values(
        joint_nodes,
        field="connection_name",
        prefix="node_tree_manifest.joint_nodes",
        errors=errors,
    )
    _validate_manifest_part_node_paths(payload.get("part_node_paths"), part_nodes, errors)
    _validate_manifest_joint_node_paths(
        payload.get("joint_node_paths"), joint_nodes, errors
    )

    _validate_manifest_count(
        payload, "parts_count", len(part_nodes), "node_tree_manifest", errors
    )
    _validate_manifest_count(
        payload, "joints_count", len(joint_nodes), "node_tree_manifest", errors
    )
    parameterized_joints = sum(
        1
        for joint in joint_nodes
        if isinstance(joint, Mapping)
        and isinstance(joint.get("applied_parameters"), Mapping)
        and bool(joint.get("applied_parameters"))
    )
    _validate_manifest_count(
        payload,
        "parameterized_joints",
        parameterized_joints,
        "node_tree_manifest",
        errors,
    )

    for field in [
        "parameters_complete",
        "endpoint_paths_complete",
        "path_maps_complete",
        "complete",
    ]:
        if not isinstance(payload.get(field), bool):
            errors.append(f"node_tree_manifest.{field} must be a boolean")
    parameters_complete = payload.get("parameters_complete")
    endpoint_paths_complete = payload.get("endpoint_paths_complete")
    path_maps_complete = payload.get("path_maps_complete")
    complete = payload.get("complete")
    if isinstance(parameters_complete, bool) and isinstance(joint_nodes, list):
        expected_parameters_complete = parameterized_joints == len(joint_nodes)
        if parameters_complete is not expected_parameters_complete:
            errors.append(
                "node_tree_manifest.parameters_complete must equal "
                "parameterized_joints == joints_count"
            )

    missing_endpoint_part_ids = _manifest_missing_endpoint_part_ids(joint_nodes)
    missing_endpoint_connection_names = (
        _manifest_missing_endpoint_connection_names(joint_nodes)
    )
    missing_endpoint_details = _manifest_missing_endpoint_details(joint_nodes)
    _validate_manifest_string_list(
        payload,
        "missing_endpoint_part_ids",
        missing_endpoint_part_ids,
        "node_tree_manifest",
        errors,
    )
    _validate_manifest_string_list(
        payload,
        "missing_endpoint_connection_names",
        missing_endpoint_connection_names,
        "node_tree_manifest",
        errors,
    )
    _validate_manifest_missing_endpoint_details(
        payload.get("missing_endpoint_details"),
        missing_endpoint_details,
        errors,
    )
    if isinstance(endpoint_paths_complete, bool):
        expected_endpoint_paths_complete = not missing_endpoint_part_ids
        if endpoint_paths_complete is not expected_endpoint_paths_complete:
            errors.append(
                "node_tree_manifest.endpoint_paths_complete must match missing "
                "endpoint part ids"
            )
    if isinstance(path_maps_complete, bool):
        expected_path_maps_complete = _manifest_path_maps_complete(
            payload.get("part_node_paths"),
            payload.get("joint_node_paths"),
            part_nodes,
            joint_nodes,
        )
        if path_maps_complete is not expected_path_maps_complete:
            errors.append(
                "node_tree_manifest.path_maps_complete must match path map counts"
            )
    if (
        isinstance(complete, bool)
        and isinstance(parameters_complete, bool)
        and isinstance(endpoint_paths_complete, bool)
        and complete is not (parameters_complete and endpoint_paths_complete)
    ):
        errors.append(
            "node_tree_manifest.complete must equal parameters_complete and "
            "endpoint_paths_complete"
        )

    robot_node = (
        str(payload.get("robot_node"))
        if _is_non_empty_string(payload.get("robot_node"))
        else ""
    )
    for index, part in enumerate(part_nodes, start=1):
        _validate_manifest_part_node(part, index, errors, robot_node=robot_node)
    for index, joint in enumerate(joint_nodes, start=1):
        _validate_manifest_joint_node(joint, index, errors, robot_node=robot_node)

    return errors


def build_godot_node_tree_manifest_path_map_mismatches(
    payload: Any,
) -> list[dict[str, Any]]:
    """Return structured path-map mismatches for a static Godot node-tree manifest."""
    if not isinstance(payload, Mapping):
        return []
    part_nodes = payload.get("part_nodes")
    joint_nodes = payload.get("joint_nodes")
    if not isinstance(part_nodes, list):
        part_nodes = []
    if not isinstance(joint_nodes, list):
        joint_nodes = []
    mismatches: list[dict[str, Any]] = []
    mismatches.extend(
        _manifest_duplicate_key_mismatches(
            map_name="part_node_paths",
            key_field="part_id",
            items=part_nodes,
        )
    )
    mismatches.extend(
        _manifest_duplicate_key_mismatches(
            map_name="joint_node_paths",
            key_field="connection_name",
            items=joint_nodes,
        )
    )
    mismatches.extend(
        _manifest_root_path_mismatches(
            robot_node=str(payload.get("robot_node", "")),
            part_nodes=part_nodes,
            joint_nodes=joint_nodes,
        )
    )
    if isinstance(payload.get("part_node_paths"), Mapping):
        mismatches.extend(
            _manifest_path_map_mismatches(
                map_name="part_node_paths",
                expected=_expected_manifest_part_node_paths(part_nodes),
                actual=_normalized_manifest_part_node_paths(
                    payload["part_node_paths"]
                ),
            )
        )
    if isinstance(payload.get("joint_node_paths"), Mapping):
        mismatches.extend(
            _manifest_path_map_mismatches(
                map_name="joint_node_paths",
                expected=_expected_manifest_joint_node_paths(joint_nodes),
                actual=_normalized_manifest_joint_node_paths(
                    payload["joint_node_paths"]
                ),
            )
        )
    return mismatches


def compare_godot_node_tree_manifest_to_runtime(
    static_manifest: Any,
    runtime_part_nodes: Any,
    runtime_joint_nodes: Any,
    *,
    tolerance: float = 1e-4,
) -> dict[str, Any]:
    """Compare a static node-tree manifest with Godot runtime node mappings."""
    part_nodes = _manifest_list(static_manifest, "part_nodes")
    joint_nodes = _manifest_list(static_manifest, "joint_nodes")
    part_mismatches = _runtime_node_mismatches(
        expected_items=part_nodes,
        actual_items=runtime_part_nodes,
        key_field="part_id",
        fields=[
            "body_node",
            "body_class",
            "collision_node",
            "collision_shape",
            "collision_parameters",
            "mesh_node",
            "mesh_type",
            "mesh_parameters",
            "mass",
            "position",
            "rotation",
        ],
        map_name="part_nodes",
        tolerance=tolerance,
    )
    joint_mismatches = _runtime_node_mismatches(
        expected_items=joint_nodes,
        actual_items=runtime_joint_nodes,
        key_field="connection_name",
        fields=["joint_node", "joint_class", "from", "to", "node_a", "node_b", "origin", "axis"],
        map_name="joint_nodes",
        tolerance=tolerance,
    )
    parameter_mismatches = _runtime_joint_parameter_mismatches(
        joint_nodes,
        runtime_joint_nodes,
        tolerance=tolerance,
    )
    mismatches = [*part_mismatches, *joint_mismatches, *parameter_mismatches]
    return {
        "static_manifest_version": (
            static_manifest.get("manifest_version")
            if isinstance(static_manifest, Mapping)
            else None
        ),
        "tolerance": tolerance,
        "part_count": len(part_nodes),
        "joint_count": len(joint_nodes),
        "restored_part_count": len(_runtime_node_map(runtime_part_nodes, "part_id")),
        "restored_joint_count": len(
            _runtime_node_map(runtime_joint_nodes, "connection_name")
        ),
        "mismatch_count": len(mismatches),
        "mismatch_kind_counts": _mismatch_kind_counts(mismatches),
        "mismatches": mismatches[:20],
        "mismatches_truncated": len(mismatches) > 20,
        "complete": not mismatches,
    }


def _validate_manifest_count(
    payload: Mapping[str, Any],
    field: str,
    expected: int,
    prefix: str,
    errors: list[str],
) -> None:
    value = payload.get(field)
    if not _is_non_negative_int(value):
        errors.append(f"{prefix}.{field} must be a non-negative integer")
    elif int(value) != expected:
        errors.append(f"{prefix}.{field} must equal {expected}")


def _validate_manifest_root_nodes(
    payload: Mapping[str, Any],
    errors: list[str],
) -> None:
    robot_name = payload.get("robot_name")
    robot_node = payload.get("robot_node")
    controller_node = payload.get("controller_node")
    if _is_non_empty_string(robot_name) and _is_non_empty_string(robot_node):
        if str(robot_node) != str(robot_name):
            errors.append("node_tree_manifest.robot_node must equal robot_name")
    if _is_non_empty_string(robot_node) and _is_non_empty_string(controller_node):
        expected_controller_node = f"{robot_node}/GeneratedRobotController"
        if str(controller_node) != expected_controller_node:
            errors.append(
                "node_tree_manifest.controller_node must equal "
                f"{expected_controller_node!r}"
            )


def _validate_manifest_unique_values(
    items: list[Any],
    *,
    field: str,
    prefix: str,
    errors: list[str],
) -> None:
    for value in _manifest_duplicate_values(items, field):
        errors.append(f"{prefix}.{field} duplicates {value!r}")


def _validate_manifest_string_list(
    payload: Mapping[str, Any],
    field: str,
    expected: list[str],
    prefix: str,
    errors: list[str],
) -> None:
    value = payload.get(field)
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        errors.append(f"{prefix}.{field} must be a list of strings")
    elif value != expected:
        errors.append(f"{prefix}.{field} must match joint missing endpoints")


def _manifest_missing_endpoint_part_ids(joint_nodes: list[Any]) -> list[str]:
    return sorted(
        {
            str(part_id)
            for joint in joint_nodes
            if isinstance(joint, Mapping)
            for part_id in joint.get("missing_endpoint_part_ids", [])
        }
    )


def _manifest_missing_endpoint_connection_names(joint_nodes: list[Any]) -> list[str]:
    return [
        str(joint.get("connection_name"))
        for joint in joint_nodes
        if isinstance(joint, Mapping)
        and joint.get("missing_endpoint_part_ids")
        and _is_non_empty_string(joint.get("connection_name"))
    ]


def _manifest_missing_endpoint_details(joint_nodes: list[Any]) -> list[dict[str, str]]:
    return [
        {
            "connection_name": str(joint.get("connection_name")),
            "field": str(detail.get("field")),
            "part_id": str(detail.get("part_id")),
        }
        for joint in joint_nodes
        if isinstance(joint, Mapping)
        for detail in joint.get("missing_endpoint_details", [])
        if isinstance(detail, Mapping)
        and _is_non_empty_string(joint.get("connection_name"))
        and _is_non_empty_string(detail.get("field"))
        and _is_non_empty_string(detail.get("part_id"))
    ]


def _validate_manifest_missing_endpoint_details(
    value: Any,
    expected: list[dict[str, str]],
    errors: list[str],
) -> None:
    if not isinstance(value, list):
        errors.append("node_tree_manifest.missing_endpoint_details must be a list")
        return
    normalized: list[dict[str, str]] = []
    for index, detail in enumerate(value, start=1):
        prefix = f"node_tree_manifest.missing_endpoint_details[{index}]"
        if not isinstance(detail, Mapping):
            errors.append(f"{prefix} must be an object")
            continue
        for field in ["connection_name", "field", "part_id"]:
            if not _is_non_empty_string(detail.get(field)):
                errors.append(f"{prefix}.{field} must be a non-empty string")
        if all(_is_non_empty_string(detail.get(field)) for field in ["connection_name", "field", "part_id"]):
            normalized.append(
                {
                    "connection_name": str(detail["connection_name"]),
                    "field": str(detail["field"]),
                    "part_id": str(detail["part_id"]),
                }
            )
    if normalized != expected:
        errors.append(
            "node_tree_manifest.missing_endpoint_details must match joint "
            "missing endpoints"
        )


def _validate_manifest_part_node_paths(
    value: Any,
    part_nodes: list[Any],
    errors: list[str],
) -> None:
    expected = _expected_manifest_part_node_paths(part_nodes)
    if not isinstance(value, Mapping):
        errors.append("node_tree_manifest.part_node_paths must be an object")
        return
    normalized = _normalized_manifest_part_node_paths(value, errors)
    if normalized != expected:
        errors.append("node_tree_manifest.part_node_paths must match part_nodes")
        _append_manifest_path_map_mismatch_errors(
            errors,
            prefix="node_tree_manifest.part_node_paths",
            expected=expected,
            actual=normalized,
        )


def _expected_manifest_part_node_paths(part_nodes: list[Any]) -> dict[str, dict[str, str]]:
    return {
        str(part["part_id"]): {
            "body_node": str(part["body_node"]),
            "collision_node": str(part["collision_node"]),
            "mesh_node": str(part["mesh_node"]),
        }
        for part in part_nodes
        if isinstance(part, Mapping)
        and _is_non_empty_string(part.get("part_id"))
        and _is_non_empty_string(part.get("body_node"))
        and _is_non_empty_string(part.get("collision_node"))
        and _is_non_empty_string(part.get("mesh_node"))
    }


def _normalized_manifest_part_node_paths(
    value: Mapping[Any, Any],
    errors: list[str] | None = None,
) -> dict[str, dict[str, str]]:
    normalized: dict[str, dict[str, str]] = {}
    for part_id, item in value.items():
        prefix = f"node_tree_manifest.part_node_paths.{part_id}"
        if not isinstance(item, Mapping):
            if errors is not None:
                errors.append(f"{prefix} must be an object")
            continue
        for field in ["body_node", "collision_node", "mesh_node"]:
            if errors is not None and not _is_non_empty_string(item.get(field)):
                errors.append(f"{prefix}.{field} must be a non-empty string")
        if all(
            _is_non_empty_string(item.get(field))
            for field in ["body_node", "collision_node", "mesh_node"]
        ):
            normalized[str(part_id)] = {
                "body_node": str(item["body_node"]),
                "collision_node": str(item["collision_node"]),
                "mesh_node": str(item["mesh_node"]),
            }
    return normalized


def _manifest_path_maps_complete(
    part_node_paths: Any,
    joint_node_paths: Any,
    part_nodes: list[Any],
    joint_nodes: list[Any],
) -> bool:
    if not isinstance(part_node_paths, Mapping) or not isinstance(
        joint_node_paths, Mapping
    ):
        return False
    if _manifest_duplicate_values(part_nodes, "part_id") or _manifest_duplicate_values(
        joint_nodes, "connection_name"
    ):
        return False
    return (
        _normalized_manifest_part_node_paths(part_node_paths)
        == _expected_manifest_part_node_paths(part_nodes)
        and _normalized_manifest_joint_node_paths(joint_node_paths)
        == _expected_manifest_joint_node_paths(joint_nodes)
    )


def _manifest_duplicate_values(items: list[Any], field: str) -> list[str]:
    seen: set[str] = set()
    duplicates: list[str] = []
    for item in items:
        if not isinstance(item, Mapping) or not _is_non_empty_string(item.get(field)):
            continue
        value = str(item[field])
        if value in seen and value not in duplicates:
            duplicates.append(value)
        seen.add(value)
    return duplicates


def _validate_manifest_joint_node_paths(
    value: Any,
    joint_nodes: list[Any],
    errors: list[str],
) -> None:
    expected = _expected_manifest_joint_node_paths(joint_nodes)
    if not isinstance(value, Mapping):
        errors.append("node_tree_manifest.joint_node_paths must be an object")
        return
    normalized = _normalized_manifest_joint_node_paths(value, errors)
    if normalized != expected:
        errors.append("node_tree_manifest.joint_node_paths must match joint_nodes")
        _append_manifest_path_map_mismatch_errors(
            errors,
            prefix="node_tree_manifest.joint_node_paths",
            expected=expected,
            actual=normalized,
        )


def _append_manifest_path_map_mismatch_errors(
    errors: list[str],
    *,
    prefix: str,
    expected: dict[str, dict[str, str]],
    actual: dict[str, dict[str, str]],
) -> None:
    for key in sorted(set(expected) - set(actual)):
        errors.append(f"{prefix}.{key} is missing")
    for key in sorted(set(actual) - set(expected)):
        errors.append(f"{prefix}.{key} is unexpected")
    for key in sorted(set(expected) & set(actual)):
        for field, expected_value in sorted(expected[key].items()):
            actual_value = actual[key].get(field)
            if actual_value != expected_value:
                errors.append(
                    f"{prefix}.{key}.{field} must equal {expected_value!r}"
                )


def _manifest_path_map_mismatches(
    *,
    map_name: str,
    expected: dict[str, dict[str, str]],
    actual: dict[str, dict[str, str]],
) -> list[dict[str, Any]]:
    mismatches: list[dict[str, Any]] = []
    for key in sorted(set(expected) - set(actual)):
        mismatches.append({"map": map_name, "key": key, "kind": "missing"})
    for key in sorted(set(actual) - set(expected)):
        mismatches.append({"map": map_name, "key": key, "kind": "unexpected"})
    for key in sorted(set(expected) & set(actual)):
        for field, expected_value in sorted(expected[key].items()):
            actual_value = actual[key].get(field)
            if actual_value != expected_value:
                mismatches.append(
                    {
                        "map": map_name,
                        "key": key,
                        "field": field,
                        "kind": "value_mismatch",
                        "expected": expected_value,
                        "actual": actual_value,
                    }
                )
    return mismatches


def _manifest_list(payload: Any, field: str) -> list[Any]:
    if isinstance(payload, Mapping) and isinstance(payload.get(field), list):
        return payload[field]
    return []


def _runtime_node_map(items: Any, key_field: str) -> dict[str, Mapping[str, Any]]:
    if not isinstance(items, list):
        return {}
    return {
        str(item[key_field]): item
        for item in items
        if isinstance(item, Mapping) and _is_non_empty_string(item.get(key_field))
    }


def _runtime_node_mismatches(
    *,
    expected_items: list[Any],
    actual_items: Any,
    key_field: str,
    fields: list[str],
    map_name: str,
    tolerance: float,
) -> list[dict[str, Any]]:
    actual_by_key = _runtime_node_map(actual_items, key_field)
    expected_by_key = _runtime_node_map(expected_items, key_field)
    mismatches: list[dict[str, Any]] = []
    for key in sorted(set(expected_by_key) - set(actual_by_key)):
        mismatches.append({"map": map_name, "key": key, "kind": "missing"})
    for key in sorted(set(actual_by_key) - set(expected_by_key)):
        mismatches.append({"map": map_name, "key": key, "kind": "unexpected"})
    for key in sorted(set(expected_by_key) & set(actual_by_key)):
        _append_runtime_field_mismatches(
            mismatches,
            map_name=map_name,
            key=key,
            expected=expected_by_key[key],
            actual=actual_by_key[key],
            fields=fields,
            tolerance=tolerance,
        )
    return mismatches


def _append_runtime_field_mismatches(
    mismatches: list[dict[str, Any]],
    *,
    map_name: str,
    key: str,
    expected: Mapping[str, Any],
    actual: Mapping[str, Any],
    fields: list[str],
    tolerance: float,
) -> None:
    for field in fields:
        if isinstance(expected.get(field), Mapping):
            _append_runtime_nested_mismatches(
                mismatches,
                map_name=map_name,
                key=key,
                field=field,
                expected=expected.get(field),
                actual=actual.get(field),
                tolerance=tolerance,
            )
            continue
        _append_runtime_value_mismatch(
            mismatches,
            map_name=map_name,
            key=key,
            field=field,
            expected=expected.get(field),
            actual=actual.get(field),
            tolerance=tolerance,
        )


def _runtime_joint_parameter_mismatches(
    expected_joints: list[Any],
    actual_joints: Any,
    *,
    tolerance: float,
) -> list[dict[str, Any]]:
    actual_by_key = _runtime_node_map(actual_joints, "connection_name")
    mismatches: list[dict[str, Any]] = []
    for joint in expected_joints:
        if not isinstance(joint, Mapping) or not _is_non_empty_string(
            joint.get("connection_name")
        ):
            continue
        key = str(joint["connection_name"])
        actual = actual_by_key.get(key)
        if not actual:
            continue
        expected_parameters = joint.get("applied_parameters", {})
        actual_parameters = actual.get("applied_parameters", {})
        if isinstance(actual_parameters, Mapping) and isinstance(
            actual_parameters.get("runtime"), Mapping
        ):
            actual_parameters = actual_parameters["runtime"]
        _append_runtime_nested_mismatches(
            mismatches,
            map_name="joint_nodes",
            key=key,
            field="applied_parameters",
            expected=expected_parameters,
            actual=actual_parameters,
            tolerance=tolerance,
        )
    return mismatches


def _append_runtime_nested_mismatches(
    mismatches: list[dict[str, Any]],
    *,
    map_name: str,
    key: str,
    field: str,
    expected: Any,
    actual: Any,
    tolerance: float,
) -> None:
    if isinstance(expected, Mapping):
        actual_mapping = actual if isinstance(actual, Mapping) else {}
        for child_key, child_expected in expected.items():
            _append_runtime_nested_mismatches(
                mismatches,
                map_name=map_name,
                key=key,
                field=f"{field}.{child_key}",
                expected=child_expected,
                actual=actual_mapping.get(child_key),
                tolerance=tolerance,
            )
        return
    _append_runtime_value_mismatch(
        mismatches,
        map_name=map_name,
        key=key,
        field=field,
        expected=expected,
        actual=actual,
        tolerance=tolerance,
    )


def _append_runtime_value_mismatch(
    mismatches: list[dict[str, Any]],
    *,
    map_name: str,
    key: str,
    field: str,
    expected: Any,
    actual: Any,
    tolerance: float,
) -> None:
    if _runtime_node_paths_match(field, expected, actual):
        return
    max_delta = _runtime_value_delta(expected, actual)
    values_match = (
        max_delta <= tolerance
        if max_delta is not None
        else expected == actual
    )
    if values_match:
        return
    mismatches.append(
        {
            "map": map_name,
            "key": key,
            "field": field,
            "kind": "value_mismatch",
            "expected": expected,
            "actual": actual,
            "max_delta": max_delta,
        }
    )


def _runtime_node_paths_match(field: str, expected: Any, actual: Any) -> bool:
    path_fields = {
        "body_node",
        "collision_node",
        "mesh_node",
        "joint_node",
        "node_a",
        "node_b",
    }
    if field.rsplit(".", 1)[-1] not in path_fields:
        return False
    if not _is_non_empty_string(expected) or not _is_non_empty_string(actual):
        return False
    expected_parts = _node_path_parts(str(expected))
    actual_parts = _node_path_parts(str(actual))
    return (
        len(actual_parts) >= len(expected_parts)
        and actual_parts[-len(expected_parts) :] == expected_parts
    )


def _node_path_parts(value: str) -> list[str]:
    return [part for part in value.replace("\\", "/").split("/") if part]


def _runtime_value_delta(expected: Any, actual: Any) -> float | None:
    if _is_number(expected) and _is_number(actual):
        return abs(float(expected) - float(actual))
    if not isinstance(expected, Sequence) or isinstance(expected, (str, bytes)):
        return None
    if not isinstance(actual, Sequence) or isinstance(actual, (str, bytes)):
        return None
    if len(expected) != len(actual):
        return None
    deltas = [
        abs(float(left) - float(right))
        for left, right in zip(expected, actual)
        if _is_number(left) and _is_number(right)
    ]
    return max(deltas, default=0.0) if len(deltas) == len(expected) else None


def _mismatch_kind_counts(mismatches: list[dict[str, Any]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for mismatch in mismatches:
        kind = str(mismatch.get("kind", "unknown"))
        counts[kind] = counts.get(kind, 0) + 1
    return counts


def _manifest_duplicate_key_mismatches(
    *,
    map_name: str,
    key_field: str,
    items: list[Any],
) -> list[dict[str, str]]:
    return [
        {
            "map": map_name,
            "key": value,
            "field": key_field,
            "kind": "duplicate",
        }
        for value in _manifest_duplicate_values(items, key_field)
    ]


def _manifest_root_path_mismatches(
    *,
    robot_node: str,
    part_nodes: list[Any],
    joint_nodes: list[Any],
) -> list[dict[str, str]]:
    if not _is_non_empty_string(robot_node):
        return []
    mismatches: list[dict[str, str]] = []
    for part in part_nodes:
        if not isinstance(part, Mapping) or not _is_non_empty_string(
            part.get("part_id")
        ):
            continue
        part_id = str(part["part_id"])
        body_node = f"{robot_node}/{part_id}"
        for field, expected in [
            ("body_node", body_node),
            ("collision_node", f"{body_node}/Collision"),
            ("mesh_node", f"{body_node}/Mesh"),
        ]:
            _append_manifest_root_path_mismatch(
                mismatches,
                map_name="part_nodes",
                key=part_id,
                field=field,
                expected=expected,
                actual=part.get(field),
            )
    for joint in joint_nodes:
        if not isinstance(joint, Mapping) or not _is_non_empty_string(
            joint.get("connection_name")
        ):
            continue
        connection_name = str(joint["connection_name"])
        _append_manifest_root_path_mismatch(
            mismatches,
            map_name="joint_nodes",
            key=connection_name,
            field="joint_node",
            expected=f"{robot_node}/{connection_name}",
            actual=joint.get("joint_node"),
        )
        for endpoint_field, node_field in [("from", "node_a"), ("to", "node_b")]:
            if not _is_non_empty_string(joint.get(endpoint_field)):
                continue
            _append_manifest_root_path_mismatch(
                mismatches,
                map_name="joint_nodes",
                key=connection_name,
                field=node_field,
                expected=f"{robot_node}/{joint[endpoint_field]}",
                actual=joint.get(node_field),
            )
    return mismatches


def _append_manifest_root_path_mismatch(
    mismatches: list[dict[str, str]],
    *,
    map_name: str,
    key: str,
    field: str,
    expected: str,
    actual: Any,
) -> None:
    if _is_non_empty_string(actual) and str(actual) != expected:
        mismatches.append(
            {
                "map": map_name,
                "key": key,
                "field": field,
                "kind": "root_mismatch",
                "expected": expected,
                "actual": str(actual),
            }
        )


def _expected_manifest_joint_node_paths(joint_nodes: list[Any]) -> dict[str, dict[str, str]]:
    return {
        str(joint["connection_name"]): {
            "joint_node": str(joint["joint_node"]),
            "node_a": str(joint["node_a"]),
            "node_b": str(joint["node_b"]),
        }
        for joint in joint_nodes
        if isinstance(joint, Mapping)
        and _is_non_empty_string(joint.get("connection_name"))
        and _is_non_empty_string(joint.get("joint_node"))
        and _is_non_empty_string(joint.get("node_a"))
        and _is_non_empty_string(joint.get("node_b"))
    }


def _normalized_manifest_joint_node_paths(
    value: Mapping[Any, Any],
    errors: list[str] | None = None,
) -> dict[str, dict[str, str]]:
    normalized: dict[str, dict[str, str]] = {}
    for connection_name, item in value.items():
        prefix = f"node_tree_manifest.joint_node_paths.{connection_name}"
        if not isinstance(item, Mapping):
            if errors is not None:
                errors.append(f"{prefix} must be an object")
            continue
        for field in ["joint_node", "node_a", "node_b"]:
            if errors is not None and not _is_non_empty_string(item.get(field)):
                errors.append(f"{prefix}.{field} must be a non-empty string")
        if all(
            _is_non_empty_string(item.get(field))
            for field in ["joint_node", "node_a", "node_b"]
        ):
            normalized[str(connection_name)] = {
                "joint_node": str(item["joint_node"]),
                "node_a": str(item["node_a"]),
                "node_b": str(item["node_b"]),
            }
    return normalized


def _validate_manifest_part_node(
    part: Any,
    index: int,
    errors: list[str],
    *,
    robot_node: str,
) -> None:
    prefix = f"node_tree_manifest.part_nodes[{index}]"
    if not isinstance(part, Mapping):
        errors.append(f"{prefix} must be an object")
        return
    for field in [
        "part_id",
        "shape",
        "body_node",
        "body_class",
        "collision_node",
        "collision_shape",
        "mesh_node",
        "mesh_type",
    ]:
        if not _is_non_empty_string(part.get(field)):
            errors.append(f"{prefix}.{field} must be a non-empty string")
    if part.get("body_class") != "RigidBody3D":
        errors.append(f"{prefix}.body_class must be 'RigidBody3D'")
    part_id = part.get("part_id")
    if _is_non_empty_string(part_id) and robot_node:
        expected_body_node = f"{robot_node}/{part_id}"
        if (
            _is_non_empty_string(part.get("body_node"))
            and part["body_node"] != expected_body_node
        ):
            errors.append(f"{prefix}.body_node must equal {expected_body_node!r}")
        expected_collision_node = f"{expected_body_node}/Collision"
        if (
            _is_non_empty_string(part.get("collision_node"))
            and part["collision_node"] != expected_collision_node
        ):
            errors.append(
                f"{prefix}.collision_node must equal {expected_collision_node!r}"
            )
        expected_mesh_node = f"{expected_body_node}/Mesh"
        if (
            _is_non_empty_string(part.get("mesh_node"))
            and part["mesh_node"] != expected_mesh_node
        ):
            errors.append(f"{prefix}.mesh_node must equal {expected_mesh_node!r}")
    if not isinstance(part.get("collision_parameters"), Mapping):
        errors.append(f"{prefix}.collision_parameters must be an object")
    if not isinstance(part.get("mesh_parameters"), Mapping):
        errors.append(f"{prefix}.mesh_parameters must be an object")
    if not _is_non_negative_number(part.get("mass")):
        errors.append(f"{prefix}.mass must be a non-negative number")
    for field in ["position", "rotation"]:
        if not _is_vector3(part.get(field)):
            errors.append(f"{prefix}.{field} must be a 3-number sequence")


def _validate_manifest_joint_node(
    joint: Any,
    index: int,
    errors: list[str],
    *,
    robot_node: str,
) -> None:
    prefix = f"node_tree_manifest.joint_nodes[{index}]"
    if not isinstance(joint, Mapping):
        errors.append(f"{prefix} must be an object")
        return
    for field in [
        "connection_name",
        "joint_node",
        "joint_class",
        "joint_type",
        "from",
        "to",
        "node_a",
        "node_b",
    ]:
        if not _is_non_empty_string(joint.get(field)):
            errors.append(f"{prefix}.{field} must be a non-empty string")
    if not isinstance(joint.get("endpoint_parts_exist"), bool):
        errors.append(f"{prefix}.endpoint_parts_exist must be a boolean")
    connection_name = joint.get("connection_name")
    if _is_non_empty_string(connection_name) and robot_node:
        expected_joint_node = f"{robot_node}/{connection_name}"
        if (
            _is_non_empty_string(joint.get("joint_node"))
            and joint["joint_node"] != expected_joint_node
        ):
            errors.append(f"{prefix}.joint_node must equal {expected_joint_node!r}")
    if robot_node:
        for field, node_field in [("from", "node_a"), ("to", "node_b")]:
            part_id = joint.get(field)
            if not _is_non_empty_string(part_id):
                continue
            expected_node = f"{robot_node}/{part_id}"
            if (
                _is_non_empty_string(joint.get(node_field))
                and joint[node_field] != expected_node
            ):
                errors.append(f"{prefix}.{node_field} must equal {expected_node!r}")
    for field in ["missing_endpoint_part_ids", "missing_endpoint_fields"]:
        value = joint.get(field)
        if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
            errors.append(f"{prefix}.{field} must be a list of strings")
    details = joint.get("missing_endpoint_details")
    if not isinstance(details, list):
        errors.append(f"{prefix}.missing_endpoint_details must be a list")
    else:
        for detail_index, detail in enumerate(details, start=1):
            detail_prefix = f"{prefix}.missing_endpoint_details[{detail_index}]"
            if not isinstance(detail, Mapping):
                errors.append(f"{detail_prefix} must be an object")
                continue
            for field in ["field", "part_id"]:
                if not _is_non_empty_string(detail.get(field)):
                    errors.append(f"{detail_prefix}.{field} must be a non-empty string")
    for field in ["origin", "axis"]:
        if not _is_vector3(joint.get(field)):
            errors.append(f"{prefix}.{field} must be a 3-number sequence")
    if not isinstance(joint.get("applied_parameters"), Mapping):
        errors.append(f"{prefix}.applied_parameters must be an object")



def _normalize_part(part: Any) -> dict[str, Any]:
    item = deepcopy(dict(part)) if isinstance(part, Mapping) else {}
    params = dict(item.get("params") or {})
    item["shape"] = str(item.get("shape") or _default_shape(item.get("type")))
    params.setdefault("mass", 1.0)
    params.setdefault("position", [0.0, 0.0, 0.0])
    params.setdefault("rotation", [0.0, 0.0, 0.0])
    if item["shape"] == "box":
        params.setdefault("size", _part_box_size(item, params))
    elif item["shape"] in {"capsule", "cylinder"}:
        params.setdefault("length", _part_length(params))
        params.setdefault("radius", 0.04)
    elif item["shape"] == "sphere":
        params.setdefault("radius", 0.05)
    item["params"] = params
    return item


def _planned_part_node(part: Mapping[str, Any], robot_name: str) -> dict[str, Any]:
    part_id = str(part["id"])
    shape = str(part.get("shape") or _default_shape(part.get("type")))
    params = part.get("params") if isinstance(part.get("params"), Mapping) else {}
    body_node = f"{robot_name}/{part_id}"
    node = {
        "part_id": part_id,
        "part_type": str(part.get("type", "")),
        "shape": shape,
        "body_node": body_node,
        "body_class": "RigidBody3D",
        "collision_node": f"{body_node}/Collision",
        "collision_shape": _planned_collision_shape(shape),
        "collision_parameters": _planned_collision_parameters(shape, params),
        "mesh_node": f"{body_node}/Mesh",
        "mesh_type": _planned_mesh_type(shape),
        "mesh_parameters": _planned_mesh_parameters(shape, params),
        "mass": _float_or_default(params.get("mass"), 1.0),
        "position": _vector3_list(params.get("position", [0.0, 0.0, 0.0])),
        "rotation": _vector3_list(params.get("rotation", [0.0, 0.0, 0.0])),
    }
    for field in ["material", "physics"]:
        if isinstance(part.get(field), Mapping):
            node[field] = deepcopy(dict(part[field]))
    return node


def _planned_joint_node(
    connection: Mapping[str, Any],
    robot_name: str,
    part_ids: set[str],
) -> dict[str, Any]:
    joint_name = str(
        connection.get("name")
        or f"{connection.get('from', 'parent')}_to_{connection.get('to', 'child')}"
    )
    joint_type = str(connection.get("joint_type") or "fixed")
    parent = str(connection.get("from", ""))
    child = str(connection.get("to", ""))
    endpoint_fields = [("from", parent), ("to", child)]
    missing_endpoint_details = [
        {"field": field, "part_id": part_id}
        for field, part_id in endpoint_fields
        if part_id and part_id not in part_ids
    ]
    missing_endpoint_part_ids = [
        str(detail["part_id"]) for detail in missing_endpoint_details
    ]
    return {
        "connection_name": joint_name,
        "joint_node": f"{robot_name}/{joint_name}",
        "joint_class": _planned_joint_class(joint_type),
        "joint_type": joint_type,
        "from": parent,
        "to": child,
        "endpoint_parts_exist": not missing_endpoint_part_ids,
        "missing_endpoint_part_ids": missing_endpoint_part_ids,
        "missing_endpoint_fields": [
            str(detail["field"]) for detail in missing_endpoint_details
        ],
        "missing_endpoint_details": missing_endpoint_details,
        "node_a": f"{robot_name}/{parent}" if parent else "",
        "node_b": f"{robot_name}/{child}" if child else "",
        "origin": _vector3_list(connection.get("origin", [0.0, 0.0, 0.0])),
        "axis": _vector3_list(connection.get("axis", DEFAULT_AXIS)),
        "applied_parameters": _planned_joint_parameters(connection, joint_type),
    }


def _planned_collision_shape(shape: str) -> str:
    return {
        "box": "BoxShape3D",
        "capsule": "CapsuleShape3D",
        "cylinder": "CylinderShape3D",
        "sphere": "SphereShape3D",
    }.get(shape, "BoxShape3D")


def _planned_mesh_type(shape: str) -> str:
    return {
        "box": "BoxMesh",
        "capsule": "CapsuleMesh",
        "cylinder": "CylinderMesh",
        "sphere": "SphereMesh",
    }.get(shape, "BoxMesh")


def _planned_collision_parameters(
    shape: str,
    params: Mapping[str, Any],
) -> dict[str, Any]:
    if shape == "box":
        return {"size": _vector3_list(params.get("size", DEFAULT_BOX_SIZE))}
    if shape in {"capsule", "cylinder"}:
        return {
            "radius": _float_or_default(params.get("radius"), 0.04),
            "height": _float_or_default(params.get("length"), 0.3),
        }
    if shape == "sphere":
        return {"radius": _float_or_default(params.get("radius"), 0.05)}
    return {}


def _planned_mesh_parameters(
    shape: str,
    params: Mapping[str, Any],
) -> dict[str, Any]:
    if shape == "box":
        return {"size": _vector3_list(params.get("size", DEFAULT_BOX_SIZE))}
    if shape == "capsule":
        return {
            "radius": _float_or_default(params.get("radius"), 0.04),
            "height": _float_or_default(params.get("length"), 0.3),
        }
    if shape == "cylinder":
        radius = _float_or_default(params.get("radius"), 0.04)
        return {
            "top_radius": radius,
            "bottom_radius": radius,
            "height": _float_or_default(params.get("length"), 0.3),
        }
    if shape == "sphere":
        radius = _float_or_default(params.get("radius"), 0.05)
        return {"radius": radius, "height": radius * 2.0}
    return {}


def _planned_joint_class(joint_type: str) -> str:
    if joint_type in {"hinge", "revolute"}:
        return "HingeJoint3D"
    if joint_type in {"slider", "prismatic"}:
        return "SliderJoint3D"
    return "Generic6DOFJoint3D"


def _planned_joint_parameters(
    connection: Mapping[str, Any],
    joint_type: str,
) -> dict[str, Any]:
    limits = connection.get("limits") if isinstance(connection.get("limits"), Mapping) else {}
    motor = connection.get("motor") if isinstance(connection.get("motor"), Mapping) else {}
    extra = _planned_schema_1_5_joint_metadata(connection, limits)
    if joint_type in {"hinge", "revolute"}:
        return {
            "limit_enabled": True,
            "limit_lower": _float_or_default(limits.get("lower"), -1.57),
            "limit_upper": _float_or_default(limits.get("upper"), 1.57),
            "motor_enabled": bool(motor.get("enabled", False)),
            "motor_target_velocity": _float_or_default(
                motor.get("target_velocity"), 0.0
            ),
            "motor_max_impulse": _float_or_default(motor.get("max_impulse"), 100.0),
            **extra,
        }
    if joint_type in {"slider", "prismatic"}:
        return {
            "limit_lower": _float_or_default(limits.get("lower"), -0.5),
            "limit_upper": _float_or_default(limits.get("upper"), 0.5),
            **extra,
        }
    zero_axes = {"x": 0.0, "y": 0.0, "z": 0.0}
    locked_axes = {"x": True, "y": True, "z": True}
    return {
        "fixed_lock_applied": True,
        "linear_limit_enabled": locked_axes.copy(),
        "linear_lower": zero_axes.copy(),
        "linear_upper": zero_axes.copy(),
        "angular_limit_enabled": locked_axes.copy(),
        "angular_lower": zero_axes.copy(),
        "angular_upper": zero_axes.copy(),
        **extra,
    }


def _planned_schema_1_5_joint_metadata(
    connection: Mapping[str, Any],
    limits: Mapping[str, Any],
) -> dict[str, Any]:
    metadata: dict[str, Any] = {}
    for field in ["effort", "velocity"]:
        if _is_number(limits.get(field)):
            metadata[f"limit_{field}"] = float(limits[field])
    for field in ["actuator", "sensor", "controller"]:
        if isinstance(connection.get(field), Mapping):
            metadata[field] = deepcopy(dict(connection[field]))
    if isinstance(connection.get("sensors"), list):
        metadata["sensors"] = deepcopy(connection["sensors"])
    return metadata


def _normalize_connection(connection: Any) -> dict[str, Any]:
    item = deepcopy(dict(connection)) if isinstance(connection, Mapping) else {}
    joint_type = str(item.get("joint_type") or "fixed")
    if joint_type == "revolute":
        item["joint_type"] = "hinge"
    elif joint_type == "prismatic":
        item["joint_type"] = "slider"
    else:
        item["joint_type"] = joint_type
    item.setdefault(
        "name",
        f"{item.get('from', 'parent')}_to_{item.get('to', 'child')}",
    )
    item.setdefault("origin", item.get("offset", [0.0, 0.0, 0.0]))
    item.setdefault("axis", DEFAULT_AXIS.copy())
    if item["joint_type"] == "fixed":
        limits = dict(item.get("limits") or {})
        limits.update(DEFAULT_FIXED_LIMITS)
        item["limits"] = limits
    else:
        item.setdefault("limits", _default_limits_for_joint_type(item["joint_type"]))
    item.setdefault("motor", DEFAULT_MOTOR.copy())
    item.setdefault("dynamics", DEFAULT_DYNAMICS.copy())
    return item


def _validate_schema_version(schema_version: Any, errors: list[str]) -> None:
    if schema_version is None:
        return
    if str(schema_version) not in SUPPORTED_ROBOT_MECHANICAL_SCHEMA_VERSIONS:
        errors.append(
            "schema_version must be one of "
            f"{list(SUPPORTED_ROBOT_MECHANICAL_SCHEMA_VERSIONS)}"
        )


def _validate_parts(parts: Any, errors: list[str]) -> set[str]:
    if not isinstance(parts, list) or not parts:
        errors.append("parts must be a non-empty list")
        return set()

    part_ids: set[str] = set()
    for index, part in enumerate(parts, start=1):
        prefix = f"parts[{index}]"
        if not isinstance(part, Mapping):
            errors.append(f"{prefix} must be an object")
            continue
        part_id = part.get("id")
        if not _is_non_empty_string(part_id):
            errors.append(f"{prefix}.id must be a non-empty string")
        elif part_id in part_ids:
            errors.append(f"{prefix}.id duplicates {part_id!r}")
        else:
            part_ids.add(part_id)
        if part.get("shape") not in SUPPORTED_GODOT_SHAPES:
            errors.append(
                f"{prefix}.shape must be one of {sorted(SUPPORTED_GODOT_SHAPES)}"
            )
        _validate_part_params(part.get("params"), part.get("shape"), prefix, errors)
        _validate_material(part.get("material"), prefix, errors)
        _validate_physics(part.get("physics"), prefix, errors)
    return part_ids


def _validate_part_params(
    params: Any, shape: Any, prefix: str, errors: list[str]
) -> None:
    if not isinstance(params, Mapping):
        errors.append(f"{prefix}.params must be an object")
        return
    if not _is_non_negative_number(params.get("mass")):
        errors.append(f"{prefix}.params.mass must be a non-negative number")
    for field in ["position", "rotation"]:
        if not _is_vector3(params.get(field)):
            errors.append(f"{prefix}.params.{field} must be a 3-number sequence")
    if shape == "box" and not _is_positive_vector3(params.get("size")):
        errors.append(f"{prefix}.params.size must be a positive 3-number sequence")
    if shape in {"capsule", "cylinder"}:
        for field in ["length", "radius"]:
            if not _is_positive_number(params.get(field)):
                errors.append(f"{prefix}.params.{field} must be a positive number")
    if shape == "sphere" and not _is_positive_number(params.get("radius")):
        errors.append(f"{prefix}.params.radius must be a positive number")


def _validate_connections(
    connections: Any, part_ids: set[str], errors: list[str]
) -> set[str]:
    connection_names: set[str] = set()
    edges: list[tuple[str, str, str]] = []
    child_connections: dict[str, str] = {}
    if not isinstance(connections, list):
        errors.append("connections must be a list")
        return connection_names
    for index, connection in enumerate(connections, start=1):
        prefix = f"connections[{index}]"
        if not isinstance(connection, Mapping):
            errors.append(f"{prefix} must be an object")
            continue
        name = connection.get("name")
        if _is_non_empty_string(name):
            if name in connection_names:
                errors.append(f"{prefix}.name duplicates {name!r}")
            else:
                connection_names.add(str(name))
        else:
            errors.append(f"{prefix}.name must be a non-empty string")
        _validate_connection_endpoint(connection, "from", part_ids, prefix, errors)
        _validate_connection_endpoint(connection, "to", part_ids, prefix, errors)
        if (
            _is_non_empty_string(connection.get("from"))
            and connection.get("from") == connection.get("to")
        ):
            errors.append(f"{prefix}.from and .to must reference different parts")
        parent = connection.get("from")
        child = connection.get("to")
        if (
            _is_non_empty_string(parent)
            and _is_non_empty_string(child)
            and parent in part_ids
            and child in part_ids
            and parent != child
        ):
            if str(child) in child_connections:
                errors.append(
                    f"{prefix}.to duplicates child endpoint {child!r}; "
                    f"already connected by {child_connections[str(child)]}"
                )
            else:
                child_connections[str(child)] = str(name) if _is_non_empty_string(name) else prefix
            edges.append((str(parent), str(child), prefix))
        if connection.get("joint_type") not in SUPPORTED_GODOT_JOINTS:
            errors.append(
                f"{prefix}.joint_type must be one of {sorted(SUPPORTED_GODOT_JOINTS)}"
            )
        for field in ["origin", "axis"]:
            if not _is_vector3(connection.get(field)):
                errors.append(f"{prefix}.{field} must be a 3-number sequence")
        if connection.get("joint_type") in {"hinge", "revolute", "slider", "prismatic"}:
            if _is_vector3(connection.get("axis")) and not _is_nonzero_vector3(
                connection.get("axis")
            ):
                errors.append(
                    f"{prefix}.axis must be non-zero for movable joints"
                )
        _validate_limits(connection.get("limits"), prefix, errors)
        if connection.get("joint_type") == "fixed":
            _validate_fixed_limits(connection.get("limits"), prefix, errors)
        _validate_motor(connection.get("motor"), prefix, errors)
        _validate_dynamics(connection.get("dynamics"), prefix, errors)
        _validate_actuator(connection.get("actuator"), prefix, errors)
        _validate_sensor(connection.get("sensor"), f"{prefix}.sensor", errors)
        _validate_sensors(connection.get("sensors"), prefix, errors)
        _validate_controller_tuning(connection.get("controller"), prefix, errors)
    _validate_connection_tree(part_ids, edges, errors)
    return connection_names


def _validate_connection_tree(
    part_ids: set[str],
    edges: list[tuple[str, str, str]],
    errors: list[str],
) -> None:
    if len(part_ids) <= 1:
        return
    if not edges:
        errors.append("connections must connect all parts into one mechanical tree")
        return
    connected_parts = {part for edge in edges for part in edge[:2]}
    missing_parts = sorted(part_ids - connected_parts)
    if missing_parts:
        errors.append(
            "connections must connect every part; disconnected parts: "
            f"{', '.join(missing_parts)}"
        )
    child_parts = {child for _parent, child, _prefix in edges}
    root_parts = sorted(part_ids - child_parts)
    if len(root_parts) != 1:
        errors.append(
            "connections must define exactly one root part; roots: "
            f"{', '.join(root_parts) if root_parts else 'none'}"
        )
    else:
        adjacency: dict[str, list[str]] = {part_id: [] for part_id in part_ids}
        for parent, child, _prefix in edges:
            adjacency.setdefault(parent, []).append(child)
        reachable = _reachable_parts(root_parts[0], adjacency)
        unreachable = sorted(part_ids - reachable)
        if unreachable:
            errors.append(
                "connections must be reachable from root "
                f"{root_parts[0]!r}; unreachable parts: {', '.join(unreachable)}"
            )

    cycle = _first_connection_cycle(edges)
    if cycle:
        errors.append(
            "connections must not contain directed cycles: "
            f"{' -> '.join(cycle)}"
        )


def _valid_connection_edges(
    connections: Any,
    part_ids: set[str],
) -> list[tuple[str, str, str]]:
    if not isinstance(connections, list):
        return []
    edges: list[tuple[str, str, str]] = []
    for index, connection in enumerate(connections, start=1):
        if not isinstance(connection, Mapping):
            continue
        parent = connection.get("from")
        child = connection.get("to")
        if (
            _is_non_empty_string(parent)
            and _is_non_empty_string(child)
            and parent in part_ids
            and child in part_ids
            and parent != child
        ):
            edges.append((str(parent), str(child), f"connections[{index}]"))
    return edges


def _duplicate_child_endpoints(edges: list[tuple[str, str, str]]) -> list[str]:
    seen: set[str] = set()
    duplicates: list[str] = []
    for _parent, child, _prefix in edges:
        if child in seen and child not in duplicates:
            duplicates.append(child)
        seen.add(child)
    return duplicates


def _reachable_parts(
    root: str,
    adjacency: Mapping[str, list[str]],
) -> set[str]:
    reachable: set[str] = set()
    stack = [root]
    while stack:
        part_id = stack.pop()
        if part_id in reachable:
            continue
        reachable.add(part_id)
        stack.extend(adjacency.get(part_id, []))
    return reachable


def _first_connection_cycle(edges: list[tuple[str, str, str]]) -> list[str]:
    adjacency: dict[str, list[str]] = {}
    for parent, child, _prefix in edges:
        adjacency.setdefault(parent, []).append(child)

    visiting: set[str] = set()
    visited: set[str] = set()
    path: list[str] = []

    def visit(part_id: str) -> list[str]:
        if part_id in visiting:
            start = path.index(part_id)
            return path[start:] + [part_id]
        if part_id in visited:
            return []
        visiting.add(part_id)
        path.append(part_id)
        for child in adjacency.get(part_id, []):
            cycle = visit(child)
            if cycle:
                return cycle
        path.pop()
        visiting.remove(part_id)
        visited.add(part_id)
        return []

    for parent, _child, _prefix in edges:
        cycle = visit(parent)
        if cycle:
            return cycle
    return []


def _validate_control(
    control: Any, connection_names: set[str], errors: list[str]
) -> None:
    if control is None:
        return
    if not isinstance(control, Mapping):
        errors.append("control must be an object when present")
        return
    mode = control.get("mode")
    if mode is not None and not _is_non_empty_string(mode):
        errors.append("control.mode must be a non-empty string when present")
    joints = control.get("joints")
    if joints is None:
        return
    if not isinstance(joints, Mapping):
        errors.append("control.joints must be an object when present")
        return
    for joint_name, params in joints.items():
        if not _is_non_empty_string(joint_name):
            errors.append("control.joints keys must be non-empty strings")
            continue
        if connection_names and joint_name not in connection_names:
            errors.append(
                f"control.joints.{joint_name} references unknown connection "
                f"{joint_name!r}"
            )
        if not isinstance(params, Mapping):
            errors.append(f"control.joints.{joint_name} must be an object")
            continue
        for field in ["kp", "kd"]:
            if field in params and not _is_number(params.get(field)):
                errors.append(f"control.joints.{joint_name}.{field} must be a number")


def _validate_connection_endpoint(
    connection: Mapping[str, Any],
    field: str,
    part_ids: set[str],
    prefix: str,
    errors: list[str],
) -> None:
    value = connection.get(field)
    if not _is_non_empty_string(value):
        errors.append(f"{prefix}.{field} must be a non-empty string")
    elif part_ids and value not in part_ids:
        errors.append(f"{prefix}.{field} references unknown part {value!r}")


def _validate_limits(limits: Any, prefix: str, errors: list[str]) -> None:
    if not isinstance(limits, Mapping):
        errors.append(f"{prefix}.limits must be an object")
        return
    for field in ["lower", "upper"]:
        if not _is_number(limits.get(field)):
            errors.append(f"{prefix}.limits.{field} must be a number")
    for field in ["effort", "velocity"]:
        if field in limits and not _is_non_negative_number(limits.get(field)):
            errors.append(
                f"{prefix}.limits.{field} must be a non-negative number"
            )
    if _is_number(limits.get("lower")) and _is_number(limits.get("upper")):
        if float(limits["lower"]) > float(limits["upper"]):
            errors.append(f"{prefix}.limits.lower must be <= limits.upper")


def _validate_fixed_limits(limits: Any, prefix: str, errors: list[str]) -> None:
    if not isinstance(limits, Mapping):
        return
    for field in ["lower", "upper"]:
        value = limits.get(field)
        if _is_number(value) and abs(float(value)) > 1e-9:
            errors.append(f"{prefix}.limits.{field} must be 0.0 for fixed joints")


def _default_limits_for_joint_type(joint_type: Any) -> dict[str, float]:
    return (
        DEFAULT_FIXED_LIMITS.copy()
        if str(joint_type) == "fixed"
        else DEFAULT_LIMITS.copy()
    )


def _validate_motor(motor: Any, prefix: str, errors: list[str]) -> None:
    if not isinstance(motor, Mapping):
        errors.append(f"{prefix}.motor must be an object")
        return
    if not isinstance(motor.get("enabled"), bool):
        errors.append(f"{prefix}.motor.enabled must be a boolean")
    if not _is_number(motor.get("target_velocity")):
        errors.append(f"{prefix}.motor.target_velocity must be a number")
    if not _is_non_negative_number(motor.get("max_impulse")):
        errors.append(f"{prefix}.motor.max_impulse must be a non-negative number")


def _validate_dynamics(dynamics: Any, prefix: str, errors: list[str]) -> None:
    if not isinstance(dynamics, Mapping):
        errors.append(f"{prefix}.dynamics must be an object")
        return
    for field in ["damping", "friction"]:
        if field in dynamics and not _is_non_negative_number(dynamics.get(field)):
            errors.append(
                f"{prefix}.dynamics.{field} must be a non-negative number"
            )


def _validate_material(material: Any, prefix: str, errors: list[str]) -> None:
    if material is None:
        return
    if not isinstance(material, Mapping):
        errors.append(f"{prefix}.material must be an object when present")
        return
    _validate_optional_preset(material, f"{prefix}.material", errors)
    for field in ["density", "friction", "restitution"]:
        _validate_optional_non_negative(material, field, f"{prefix}.material", errors)


def _validate_physics(physics: Any, prefix: str, errors: list[str]) -> None:
    if physics is None:
        return
    if not isinstance(physics, Mapping):
        errors.append(f"{prefix}.physics must be an object when present")
        return
    _validate_optional_preset(physics, f"{prefix}.physics", errors)
    for field in ["linear_damping", "angular_damping"]:
        _validate_optional_non_negative(physics, field, f"{prefix}.physics", errors)


def _validate_actuator(actuator: Any, prefix: str, errors: list[str]) -> None:
    if actuator is None:
        return
    if not isinstance(actuator, Mapping):
        errors.append(f"{prefix}.actuator must be an object when present")
        return
    if not _is_non_empty_string(actuator.get("type")):
        errors.append(f"{prefix}.actuator.type must be a non-empty string")
    for field in ["max_torque", "max_velocity", "gear_ratio", "response_time"]:
        _validate_optional_non_negative(actuator, field, f"{prefix}.actuator", errors)


def _validate_sensors(sensors: Any, prefix: str, errors: list[str]) -> None:
    if sensors is None:
        return
    if not isinstance(sensors, list):
        errors.append(f"{prefix}.sensors must be a list when present")
        return
    for index, sensor in enumerate(sensors, start=1):
        _validate_sensor(sensor, f"{prefix}.sensors[{index}]", errors)


def _validate_sensor(sensor: Any, prefix: str, errors: list[str]) -> None:
    if sensor is None:
        return
    if not isinstance(sensor, Mapping):
        errors.append(f"{prefix} must be an object when present")
        return
    if not _is_non_empty_string(sensor.get("type")):
        errors.append(f"{prefix}.type must be a non-empty string")
    for field in ["rate_hz", "noise_stddev"]:
        _validate_optional_non_negative(sensor, field, prefix, errors)


def _validate_controller_tuning(
    controller: Any,
    prefix: str,
    errors: list[str],
) -> None:
    if controller is None:
        return
    if not isinstance(controller, Mapping):
        errors.append(f"{prefix}.controller must be an object when present")
        return
    for field in ["kp", "ki", "kd", "max_output"]:
        _validate_optional_non_negative(controller, field, f"{prefix}.controller", errors)
    for field in ["target_position", "target_velocity"]:
        if field in controller and not _is_number(controller.get(field)):
            errors.append(f"{prefix}.controller.{field} must be a number")


def _validate_optional_preset(
    value: Mapping[str, Any],
    prefix: str,
    errors: list[str],
) -> None:
    if "preset" in value and not _is_non_empty_string(value.get("preset")):
        errors.append(f"{prefix}.preset must be a non-empty string")


def _validate_optional_non_negative(
    value: Mapping[str, Any],
    field: str,
    prefix: str,
    errors: list[str],
) -> None:
    if field in value and not _is_non_negative_number(value.get(field)):
        errors.append(f"{prefix}.{field} must be a non-negative number")


def _default_shape(part_type: Any) -> str:
    if part_type == "torso":
        return "box"
    if part_type in {"leg", "thigh", "shin", "arm"}:
        return "capsule"
    return "box"


def _part_box_size(part: Mapping[str, Any], params: Mapping[str, Any]) -> list[float]:
    if _is_positive_vector3(params.get("size")):
        return [float(item) for item in params["size"]]
    return [
        float(params.get("width", params.get("length", DEFAULT_BOX_SIZE[0]))),
        float(params.get("depth", DEFAULT_BOX_SIZE[1])),
        float(params.get("height", DEFAULT_BOX_SIZE[2])),
    ]


def _part_length(params: Mapping[str, Any]) -> float:
    for key in ["length", "height", "thigh_length", "shin_length", "upper_length"]:
        if _is_positive_number(params.get(key)):
            return float(params[key])
    return 0.3


def _vector3_list(value: Any) -> list[float]:
    if _is_vector3(value):
        return [float(value[0]), float(value[1]), float(value[2])]
    return [0.0, 0.0, 0.0]


def _float_or_default(value: Any, default: float) -> float:
    return float(value) if _is_number(value) else default


def _is_vector3(value: Any) -> bool:
    return _is_number_sequence(value, length=3)


def _is_positive_vector3(value: Any) -> bool:
    return _is_number_sequence(value, length=3) and all(
        float(item) > 0 for item in value
    )


def _is_nonzero_vector3(value: Any) -> bool:
    return _is_number_sequence(value, length=3) and any(
        abs(float(item)) > 1e-9 for item in value
    )


def _is_number_sequence(value: Any, *, length: int) -> bool:
    return (
        isinstance(value, Sequence)
        and not isinstance(value, (str, bytes))
        and len(value) == length
        and all(_is_number(item) for item in value)
    )


def _is_non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _is_non_negative_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _is_positive_number(value: Any) -> bool:
    return _is_number(value) and value > 0


def _is_non_negative_number(value: Any) -> bool:
    return _is_number(value) and value >= 0
