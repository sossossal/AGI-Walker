"""Stable contracts for phase-one workflow artifacts and core payloads."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict, is_dataclass
from typing import Any

WORKFLOW_CONTRACT_VERSION = "1.0"

WORKFLOW_STEP_STATUSES = {"pending", "running", "completed", "failed", "skipped"}
WORKFLOW_EXECUTOR_MODES = {"mock", "real"}

WORKFLOW_STEP_ARTIFACT_REQUIRED_FIELDS = {
    "schema_version",
    "artifact_type",
    "workflow",
    "step",
    "executor",
    "action",
    "status",
    "mode",
    "inputs",
    "output",
    "artifact_index",
    "attempts",
    "duration_seconds",
    "created_at",
}

WORKFLOW_STEP_ARTIFACT_SCHEMA = {
    "schema_version": WORKFLOW_CONTRACT_VERSION,
    "artifact_type": "workflow_step",
    "required": sorted(WORKFLOW_STEP_ARTIFACT_REQUIRED_FIELDS),
    "status_values": sorted(WORKFLOW_STEP_STATUSES),
    "mode_values": sorted(WORKFLOW_EXECUTOR_MODES),
}

ROBOT_CONFIG_SCHEMA = {
    "schema_version": WORKFLOW_CONTRACT_VERSION,
    "required": ["name", "parts", "connections"],
    "part_required": ["id", "type", "params"],
    "connection_required": ["from", "to", "joint_type"],
}

PART_SPEC_SCHEMA = {
    "schema_version": WORKFLOW_CONTRACT_VERSION,
    "required": ["id", "category", "name", "weight_kg", "cost_usd", "specs"],
}

OPTIMIZATION_RESULT_SCHEMA = {
    "schema_version": WORKFLOW_CONTRACT_VERSION,
    "required": ["success", "iterations", "mass_distribution", "com_position"],
    "optional": ["com_error", "message", "parameters", "final_value"],
}

EXPORT_RESULT_SCHEMA = {
    "schema_version": WORKFLOW_CONTRACT_VERSION,
    "required": ["status", "action", "output_file", "format"],
    "format_values": ["urdf", "sdf", "mjcf"],
}


def to_jsonable(value: Any) -> Any:
    """Convert common executor return values into deterministic JSON payloads."""
    if is_dataclass(value):
        return to_jsonable(asdict(value))
    if isinstance(value, Mapping):
        return {str(key): to_jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [to_jsonable(item) for item in value]
    if hasattr(value, "tolist") and callable(value.tolist):
        return to_jsonable(value.tolist())
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def build_workflow_step_artifact(
    *,
    workflow: str,
    step: str,
    executor: str,
    action: str,
    status: str,
    mode: str,
    inputs: Mapping[str, Any],
    output: Mapping[str, Any],
    artifact_index: int,
    attempts: int,
    duration_seconds: float,
    created_at: str,
    error: str | None = None,
    error_type: str | None = None,
) -> dict[str, Any]:
    """Create the canonical JSON shape for a persisted workflow step."""
    return {
        "schema_version": WORKFLOW_CONTRACT_VERSION,
        "artifact_type": "workflow_step",
        "workflow": workflow,
        "step": step,
        "executor": executor,
        "action": action,
        "status": status,
        "mode": mode,
        "inputs": to_jsonable(dict(inputs)),
        "output": to_jsonable(dict(output)),
        "artifact_index": artifact_index,
        "attempts": attempts,
        "duration_seconds": round(float(duration_seconds), 6),
        "created_at": created_at,
        "error": error,
        "error_type": error_type,
    }


def validate_workflow_step_artifact(payload: Any) -> list[str]:
    """Return validation errors for a workflow-step artifact payload."""
    if not isinstance(payload, Mapping):
        return ["artifact must be an object"]

    errors: list[str] = []
    missing = sorted(WORKFLOW_STEP_ARTIFACT_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(f"missing required fields: {', '.join(missing)}")

    if payload.get("schema_version") != WORKFLOW_CONTRACT_VERSION:
        errors.append(
            f"schema_version must be {WORKFLOW_CONTRACT_VERSION!r}, got {payload.get('schema_version')!r}"
        )
    if payload.get("artifact_type") != "workflow_step":
        errors.append("artifact_type must be 'workflow_step'")

    for key in [
        "workflow",
        "step",
        "executor",
        "action",
        "status",
        "mode",
        "created_at",
    ]:
        if key in payload and not _is_non_empty_string(payload.get(key)):
            errors.append(f"{key} must be a non-empty string")

    if payload.get("status") not in WORKFLOW_STEP_STATUSES:
        errors.append(f"status must be one of {sorted(WORKFLOW_STEP_STATUSES)}")
    if payload.get("mode") not in WORKFLOW_EXECUTOR_MODES:
        errors.append(f"mode must be one of {sorted(WORKFLOW_EXECUTOR_MODES)}")

    if "inputs" in payload and not isinstance(payload.get("inputs"), Mapping):
        errors.append("inputs must be an object")
    if "output" in payload and not isinstance(payload.get("output"), Mapping):
        errors.append("output must be an object")
    if "artifact_index" in payload and not _is_non_negative_int(
        payload.get("artifact_index")
    ):
        errors.append("artifact_index must be a non-negative integer")
    if "attempts" in payload and not _is_non_negative_int(payload.get("attempts")):
        errors.append("attempts must be a non-negative integer")
    if "duration_seconds" in payload and not _is_non_negative_number(
        payload.get("duration_seconds")
    ):
        errors.append("duration_seconds must be a non-negative number")

    return errors


def validate_workflow_definition(
    workflow_name: str,
    workflow: Any,
    executor_actions: Mapping[str, set[str]],
) -> list[str]:
    """Validate the built-in/custom workflow definition shape."""
    if not isinstance(workflow, Mapping):
        return [f"workflow {workflow_name!r} must be an object"]

    errors: list[str] = []
    if workflow.get("name") != workflow_name:
        errors.append(
            f"workflow name mismatch: expected {workflow_name!r}, got {workflow.get('name')!r}"
        )

    steps = workflow.get("steps")
    if not isinstance(steps, list) or not steps:
        return errors + ["workflow steps must be a non-empty list"]

    seen_steps: set[str] = set()
    previous_steps: set[str] = set()
    for index, step in enumerate(steps, start=1):
        prefix = f"steps[{index}]"
        if not isinstance(step, Mapping):
            errors.append(f"{prefix} must be an object")
            continue

        step_name = step.get("name")
        executor_name = step.get("skill_executor")
        action = step.get("action")
        inputs = step.get("inputs", {})

        if not _is_non_empty_string(step_name):
            errors.append(f"{prefix}.name must be a non-empty string")
        elif step_name in seen_steps:
            errors.append(f"{prefix}.name duplicates step {step_name!r}")
        else:
            seen_steps.add(step_name)

        if not _is_non_empty_string(executor_name):
            errors.append(f"{prefix}.skill_executor must be a non-empty string")
        elif executor_name not in executor_actions:
            errors.append(
                f"{prefix}.skill_executor {executor_name!r} is not registered"
            )

        if not _is_non_empty_string(action):
            errors.append(f"{prefix}.action must be a non-empty string")
        elif (
            _is_non_empty_string(executor_name)
            and executor_name in executor_actions
            and action not in executor_actions[executor_name]
        ):
            supported = ", ".join(sorted(executor_actions[executor_name]))
            errors.append(
                f"{prefix}.action {action!r} is not supported by {executor_name!r}; supported: {supported}"
            )

        if not isinstance(inputs, Mapping):
            errors.append(f"{prefix}.inputs must be an object")
        else:
            for ref in _iter_exact_references(inputs):
                if "." not in ref:
                    errors.append(
                        f"{prefix}.inputs reference {{{ref}}} must be step.key"
                    )
                    continue
                ref_step, ref_key = ref.split(".", 1)
                if not ref_step or not ref_key:
                    errors.append(
                        f"{prefix}.inputs reference {{{ref}}} must be step.key"
                    )
                elif ref_step not in previous_steps:
                    errors.append(
                        f"{prefix}.inputs reference {{{ref}}} points to a missing or future step"
                    )

        if _is_non_empty_string(step_name):
            previous_steps.add(step_name)

    return errors


def validate_robot_config(payload: Any) -> list[str]:
    """Validate the phase-one RobotConfig JSON contract."""
    if not isinstance(payload, Mapping):
        return ["robot config must be an object"]

    errors: list[str] = []
    if not _is_non_empty_string(payload.get("name")):
        errors.append("name must be a non-empty string")

    parts = payload.get("parts")
    if not isinstance(parts, list):
        errors.append("parts must be a list")
    else:
        for index, part in enumerate(parts, start=1):
            prefix = f"parts[{index}]"
            if not isinstance(part, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            if not _is_non_empty_string(part.get("id")):
                errors.append(f"{prefix}.id must be a non-empty string")
            if not _is_non_empty_string(part.get("type")):
                errors.append(f"{prefix}.type must be a non-empty string")
            if not isinstance(part.get("params"), Mapping):
                errors.append(f"{prefix}.params must be an object")

    connections = payload.get("connections")
    if not isinstance(connections, list):
        errors.append("connections must be a list")
    else:
        for index, connection in enumerate(connections, start=1):
            prefix = f"connections[{index}]"
            if not isinstance(connection, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            for field in ["from", "to", "joint_type"]:
                if not _is_non_empty_string(connection.get(field)):
                    errors.append(f"{prefix}.{field} must be a non-empty string")

    if "metadata" in payload and not isinstance(payload.get("metadata"), Mapping):
        errors.append("metadata must be an object when present")

    return errors


def validate_part_spec(payload: Any) -> list[str]:
    """Validate a normalized PartSpec payload."""
    if not isinstance(payload, Mapping):
        return ["part spec must be an object"]

    errors: list[str] = []
    for field in ["id", "category", "name"]:
        if not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    for field in ["weight_kg", "cost_usd"]:
        if not _is_non_negative_number(payload.get(field)):
            errors.append(f"{field} must be a non-negative number")
    if not isinstance(payload.get("specs"), Mapping):
        errors.append("specs must be an object")
    return errors


def validate_optimization_result(payload: Any) -> list[str]:
    """Validate mass-optimization result payloads persisted by real workflows."""
    if not isinstance(payload, Mapping):
        return ["optimization result must be an object"]

    errors: list[str] = []
    if not isinstance(payload.get("success"), bool):
        errors.append("success must be a boolean")
    if not _is_non_negative_int(payload.get("iterations")):
        errors.append("iterations must be a non-negative integer")
    if not isinstance(payload.get("mass_distribution"), Mapping):
        errors.append("mass_distribution must be an object")
    com_position = payload.get("com_position")
    if not (
        isinstance(com_position, Sequence)
        and not isinstance(com_position, (str, bytes))
        and len(com_position) == 3
        and all(_is_number(item) for item in com_position)
    ):
        errors.append("com_position must be a 3-number sequence")
    if "com_error" in payload and not _is_non_negative_number(payload.get("com_error")):
        errors.append("com_error must be a non-negative number when present")
    return errors


def validate_export_result(payload: Any) -> list[str]:
    """Validate URDF/SDF/MJCF export executor result payloads."""
    if not isinstance(payload, Mapping):
        return ["export result must be an object"]

    errors: list[str] = []
    for field in ["status", "action", "output_file", "format"]:
        if not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    if payload.get("format") not in set(EXPORT_RESULT_SCHEMA["format_values"]):
        errors.append(f"format must be one of {EXPORT_RESULT_SCHEMA['format_values']}")
    if "file_size" in payload and not _is_non_negative_int(payload.get("file_size")):
        errors.append("file_size must be a non-negative integer when present")
    if "output_generated" in payload and not isinstance(
        payload.get("output_generated"), bool
    ):
        errors.append("output_generated must be a boolean when present")
    return errors


def _iter_exact_references(value: Any):
    if isinstance(value, Mapping):
        for item in value.values():
            yield from _iter_exact_references(item)
    elif isinstance(value, list):
        for item in value:
            yield from _iter_exact_references(item)
    elif isinstance(value, str) and value.startswith("{") and value.endswith("}"):
        yield value[1:-1]


def _is_non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _is_non_negative_number(value: Any) -> bool:
    return _is_number(value) and value >= 0


def _is_non_negative_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0
