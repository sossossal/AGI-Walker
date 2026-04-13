"""Stable contracts for training run metadata and artifacts."""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from agi_walker.core.api.workflow_contracts import to_jsonable

TRAINING_CONTRACT_VERSION = "1.0"

TRAINING_RUN_TYPES = {
    "mock_training",
    "offline_dataset_training",
    "sim_training",
    "hardware_in_the_loop",
}
TRAINING_RUN_STATUSES = {"running", "completed", "failed", "skipped"}

TRAINING_RUN_ARTIFACT_REQUIRED_FIELDS = {
    "schema_version",
    "artifact_type",
    "run_id",
    "run_type",
    "stage",
    "status",
    "algorithm",
    "environment",
    "inputs",
    "metrics",
    "artifacts",
    "hardware_required",
    "hardware_enabled",
    "started_at",
    "finished_at",
    "duration_seconds",
}

TRAINING_RUN_ARTIFACT_SCHEMA = {
    "schema_version": TRAINING_CONTRACT_VERSION,
    "artifact_type": "training_run",
    "required": sorted(TRAINING_RUN_ARTIFACT_REQUIRED_FIELDS),
    "run_type_values": sorted(TRAINING_RUN_TYPES),
    "status_values": sorted(TRAINING_RUN_STATUSES),
    "artifact_required": ["name", "path", "artifact_type"],
}


def build_training_run_artifact(
    *,
    run_id: str,
    run_type: str,
    stage: str,
    status: str,
    algorithm: str,
    environment: Mapping[str, Any],
    inputs: Mapping[str, Any],
    metrics: Mapping[str, Any],
    artifacts: Sequence[Mapping[str, Any]],
    hardware_required: bool,
    hardware_enabled: bool,
    started_at: str,
    finished_at: str | None,
    duration_seconds: float,
    error: str | None = None,
    error_type: str | None = None,
) -> dict[str, Any]:
    """Create the canonical JSON shape for a training run artifact."""
    return {
        "schema_version": TRAINING_CONTRACT_VERSION,
        "artifact_type": "training_run",
        "run_id": run_id,
        "run_type": run_type,
        "stage": stage,
        "status": status,
        "algorithm": algorithm,
        "environment": to_jsonable(dict(environment)),
        "inputs": to_jsonable(dict(inputs)),
        "metrics": to_jsonable(dict(metrics)),
        "artifacts": to_jsonable([dict(item) for item in artifacts]),
        "hardware_required": hardware_required,
        "hardware_enabled": hardware_enabled,
        "started_at": started_at,
        "finished_at": finished_at,
        "duration_seconds": round(float(duration_seconds), 6),
        "error": error,
        "error_type": error_type,
    }


def validate_training_run_artifact(payload: Any) -> list[str]:
    """Return validation errors for a training-run artifact payload."""
    if not isinstance(payload, Mapping):
        return ["training artifact must be an object"]

    errors: list[str] = []
    missing = sorted(TRAINING_RUN_ARTIFACT_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(f"missing required fields: {', '.join(missing)}")

    if payload.get("schema_version") != TRAINING_CONTRACT_VERSION:
        errors.append(
            f"schema_version must be {TRAINING_CONTRACT_VERSION!r}, got {payload.get('schema_version')!r}"
        )
    if payload.get("artifact_type") != "training_run":
        errors.append("artifact_type must be 'training_run'")

    for key in ["run_id", "run_type", "stage", "status", "algorithm", "started_at"]:
        if key in payload and not _is_non_empty_string(payload.get(key)):
            errors.append(f"{key} must be a non-empty string")

    if payload.get("run_type") not in TRAINING_RUN_TYPES:
        errors.append(f"run_type must be one of {sorted(TRAINING_RUN_TYPES)}")
    if payload.get("status") not in TRAINING_RUN_STATUSES:
        errors.append(f"status must be one of {sorted(TRAINING_RUN_STATUSES)}")

    for key in ["environment", "inputs", "metrics"]:
        if key in payload and not isinstance(payload.get(key), Mapping):
            errors.append(f"{key} must be an object")

    artifacts = payload.get("artifacts")
    if not isinstance(artifacts, list):
        errors.append("artifacts must be a list")
    else:
        for index, artifact in enumerate(artifacts, start=1):
            prefix = f"artifacts[{index}]"
            if not isinstance(artifact, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            for field in TRAINING_RUN_ARTIFACT_SCHEMA["artifact_required"]:
                if not _is_non_empty_string(artifact.get(field)):
                    errors.append(f"{prefix}.{field} must be a non-empty string")

    for key in ["hardware_required", "hardware_enabled"]:
        if key in payload and not isinstance(payload.get(key), bool):
            errors.append(f"{key} must be a boolean")

    if (
        payload.get("run_type") == "hardware_in_the_loop"
        and payload.get("hardware_required") is not True
    ):
        errors.append("hardware_in_the_loop runs must set hardware_required=true")
    if (
        payload.get("hardware_enabled") is True
        and payload.get("hardware_required") is not True
    ):
        errors.append("hardware_enabled=true requires hardware_required=true")

    if "finished_at" in payload and payload.get("finished_at") is not None:
        if not _is_non_empty_string(payload.get("finished_at")):
            errors.append("finished_at must be null or a non-empty string")
    if "duration_seconds" in payload and not _is_non_negative_number(
        payload.get("duration_seconds")
    ):
        errors.append("duration_seconds must be a non-negative number")

    return errors


def write_training_run_artifact(payload: Mapping[str, Any], path: str | Path) -> Path:
    """Validate and write a training-run artifact to disk."""
    errors = validate_training_run_artifact(payload)
    if errors:
        raise ValueError("; ".join(errors))

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def _is_non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _is_non_negative_number(value: Any) -> bool:
    return _is_number(value) and value >= 0
