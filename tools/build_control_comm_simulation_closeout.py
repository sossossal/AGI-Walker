"""Build a closeout report for control/communication simulation evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from agi_walker.core.simulation.control_comm_simulation import (  # noqa: E402
    validate_control_comm_simulation_report,
    validate_ethercat_model_trace,
    validate_godot_control_comm_simulation_log,
    validate_live_hardware_migration_gate,
    validate_motor_joint_response_trace,
    validate_simulator_adapter_boundary,
    validate_zenoh_openneuro_topic_mapping,
    validate_zenoh_simulated_trace,
)

CLOSEOUT_VERSION = "control_comm_simulation_closeout.v1"


Validator = Callable[[Any], list[str]]


def build_closeout(report_path: Path) -> dict[str, Any]:
    report = _read_json_object(report_path)
    report_errors = validate_control_comm_simulation_report(report)
    artifact_paths = report.get("artifact_paths", {})
    if not isinstance(artifact_paths, dict):
        artifact_paths = {}

    artifact_specs: list[tuple[str, Validator]] = [
        ("godot_log_contract_preview", validate_godot_control_comm_simulation_log),
        ("zenoh_openneuro_topic_mapping", validate_zenoh_openneuro_topic_mapping),
        ("zenoh_simulated_trace", validate_zenoh_simulated_trace),
        ("ethercat_model_trace", validate_ethercat_model_trace),
        ("motor_joint_response_trace", validate_motor_joint_response_trace),
        ("simulator_adapter_boundary", validate_simulator_adapter_boundary),
        ("live_hardware_migration_gate", validate_live_hardware_migration_gate),
    ]
    artifact_results = [
        _validate_artifact(report_path.parent, artifact_paths, key, validator)
        for key, validator in artifact_specs
    ]
    artifact_errors = [
        f"{artifact['artifact_key']}: {error}"
        for artifact in artifact_results
        for error in artifact["validation_errors"]
    ]
    live_gate = report.get("live_hardware_migration_gate", {})
    live_gate_status = (
        live_gate.get("release_gate_status")
        if isinstance(live_gate, dict)
        else None
    )
    external_blockers = _external_blockers(report)
    errors = [*report_errors, *artifact_errors]
    local_status = "accepted_with_documented_external_blockers"
    status = local_status if not errors and live_gate_status == "blocked" else "blocked"

    return {
        "closeout_version": CLOSEOUT_VERSION,
        "status": status,
        "evidence_level": "non_live_simulation",
        "scenario_id": report.get("scenario_id"),
        "source_report": str(report_path),
        "report_status": report.get("status"),
        "clock_mode": report.get("clock_mode"),
        "transport_mode": report.get("transport_mode"),
        "simulated_transport_modes": report.get("simulated_transport_modes", []),
        "cycle_count": report.get("cycle_count"),
        "timing_metrics": report.get("timing_metrics", {}),
        "message_integrity": report.get("message_integrity", {}),
        "artifact_results": artifact_results,
        "artifact_error_count": len(artifact_errors),
        "live_hardware_release_gate_status": live_gate_status,
        "external_blockers": external_blockers,
        "residual_risks": report.get("residual_risks", []),
        "closeout_validation_errors": [],
        "errors": errors,
    }


def validate_control_comm_simulation_closeout(payload: Any) -> list[str]:
    if not isinstance(payload, dict):
        return ["control communication simulation closeout must be an object"]

    errors: list[str] = []
    if payload.get("closeout_version") != CLOSEOUT_VERSION:
        errors.append(f"closeout_version must be {CLOSEOUT_VERSION!r}")
    if payload.get("status") not in {
        "accepted_with_documented_external_blockers",
        "blocked",
    }:
        errors.append(
            "status must be 'accepted_with_documented_external_blockers' or 'blocked'"
        )
    if payload.get("evidence_level") != "non_live_simulation":
        errors.append("evidence_level must be 'non_live_simulation'")
    for field in ["scenario_id", "source_report", "report_status", "clock_mode", "transport_mode"]:
        if not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    for field in ["simulated_transport_modes", "external_blockers", "residual_risks", "errors"]:
        if not isinstance(payload.get(field), list):
            errors.append(f"{field} must be a list")
    for field in ["timing_metrics", "message_integrity"]:
        if not isinstance(payload.get(field), dict):
            errors.append(f"{field} must be an object")
    if not _is_non_negative_int(payload.get("cycle_count")):
        errors.append("cycle_count must be a non-negative integer")
    if not _is_non_negative_int(payload.get("artifact_error_count")):
        errors.append("artifact_error_count must be a non-negative integer")
    if payload.get("live_hardware_release_gate_status") != "blocked":
        errors.append("live_hardware_release_gate_status must be 'blocked'")
    artifact_results = payload.get("artifact_results")
    if not isinstance(artifact_results, list):
        errors.append("artifact_results must be a list")
        return errors
    counted_artifact_errors = 0
    for index, artifact in enumerate(artifact_results):
        if not isinstance(artifact, dict):
            errors.append(f"artifact_results[{index}] must be an object")
            continue
        if not _is_non_empty_string(artifact.get("artifact_key")):
            errors.append(
                f"artifact_results[{index}].artifact_key must be a non-empty string"
            )
        artifact_path = artifact.get("artifact_path")
        if artifact_path is not None and not _is_non_empty_string(artifact_path):
            errors.append(
                f"artifact_results[{index}].artifact_path must be null or a non-empty string"
            )
        if not isinstance(artifact.get("present"), bool):
            errors.append(f"artifact_results[{index}].present must be a boolean")
        if artifact.get("present") is True:
            if not _is_non_negative_int(artifact.get("size_bytes")):
                errors.append(
                    f"artifact_results[{index}].size_bytes must be a non-negative integer"
                )
            if not _is_sha256_hex(artifact.get("sha256")):
                errors.append(
                    f"artifact_results[{index}].sha256 must be a SHA-256 hex digest"
                )
        else:
            if artifact.get("size_bytes") is not None:
                errors.append(f"artifact_results[{index}].size_bytes must be null")
            if artifact.get("sha256") is not None:
                errors.append(f"artifact_results[{index}].sha256 must be null")
        validation_errors = artifact.get("validation_errors")
        if not isinstance(validation_errors, list):
            errors.append(
                f"artifact_results[{index}].validation_errors must be a list"
            )
        else:
            counted_artifact_errors += len(validation_errors)
    if _is_non_negative_int(payload.get("artifact_error_count")) and int(
        payload["artifact_error_count"]
    ) != counted_artifact_errors:
        errors.append("artifact_error_count must equal total artifact validation errors")
    if payload.get("status") == "accepted_with_documented_external_blockers":
        if counted_artifact_errors:
            errors.append(
                "accepted closeout must not contain artifact validation errors"
            )
        if payload.get("errors"):
            errors.append("accepted closeout must not contain errors")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build a closeout JSON for control/communication simulation evidence."
    )
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument(
        "--output",
        type=Path,
        help="Output path. Defaults to control_comm_simulation_closeout.json beside the report.",
    )
    args = parser.parse_args()

    report_path = args.report.resolve()
    closeout = build_closeout(report_path)
    output_path = args.output or report_path.with_name(
        "control_comm_simulation_closeout.json"
    )
    closeout_validation_errors = validate_control_comm_simulation_closeout(closeout)
    closeout["closeout_validation_errors"] = closeout_validation_errors
    if closeout_validation_errors:
        closeout["status"] = "blocked"
        closeout["errors"] = [*closeout["errors"], *closeout_validation_errors]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(closeout, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(f"control_comm_simulation_closeout={output_path}")
    print(f"control_comm_simulation_closeout_status={closeout['status']}")
    if closeout["errors"]:
        print("control_comm_simulation_closeout_errors=" + "; ".join(closeout["errors"]))
    return 0 if not closeout["errors"] else 1


def _validate_artifact(
    report_dir: Path,
    artifact_paths: dict[str, Any],
    artifact_key: str,
    validator: Validator,
) -> dict[str, Any]:
    raw_path = artifact_paths.get(artifact_key)
    if not isinstance(raw_path, str) or not raw_path.strip():
        return {
            "artifact_key": artifact_key,
            "artifact_path": raw_path,
            "present": False,
            "size_bytes": None,
            "sha256": None,
            "validation_errors": ["artifact path is missing"],
        }
    artifact_path = Path(raw_path)
    if not artifact_path.is_absolute():
        artifact_path = artifact_path if artifact_path.exists() else report_dir / artifact_path
    if not artifact_path.exists():
        return {
            "artifact_key": artifact_key,
            "artifact_path": str(artifact_path),
            "present": False,
            "size_bytes": None,
            "sha256": None,
            "validation_errors": ["artifact file is missing"],
        }
    try:
        payload = _read_json_object(artifact_path)
        validation_errors = validator(payload)
    except ValueError as exc:
        validation_errors = [str(exc)]
    return {
        "artifact_key": artifact_key,
        "artifact_path": str(artifact_path),
        "present": True,
        "size_bytes": artifact_path.stat().st_size,
        "sha256": _sha256_file(artifact_path),
        "validation_errors": validation_errors,
    }


def _external_blockers(report: dict[str, Any]) -> list[str]:
    blockers: list[str] = []
    for section_name in [
        "godot_log_evidence",
        "zenoh_openneuro_simulation",
        "ethercat_cycle_model",
        "motor_joint_model",
        "simulator_adapter_boundary",
        "live_hardware_migration_gate",
    ]:
        section = report.get(section_name)
        if isinstance(section, dict):
            risks = section.get("residual_risks", [])
            if isinstance(risks, list):
                blockers.extend(str(risk) for risk in risks)
    blockers.extend(str(risk) for risk in report.get("residual_risks", []))
    return sorted(set(blockers))


def _read_json_object(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ValueError(f"{path}: failed to read JSON: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"{path}: invalid JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"{path}: JSON payload must be an object")
    return payload


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _is_non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _is_non_negative_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _is_sha256_hex(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


if __name__ == "__main__":
    raise SystemExit(main())
