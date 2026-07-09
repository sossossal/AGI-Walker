#!/usr/bin/env python
"""Build an operator checklist from next-stage readiness blockers."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for candidate in current.parents:
        if (candidate / "pyproject.toml").exists() and (candidate / "agi_walker").exists():
            return candidate
    return current.parent


PROJECT_ROOT = _find_repo_root()
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.build_next_stage_readiness_report import (
    DEFAULT_OUTPUT as DEFAULT_READINESS_REPORT,
    build_next_stage_readiness_report,
    validate_next_stage_readiness_report,
)


SCHEMA_VERSION = "next_stage_external_evidence_checklist.v1"
DEFAULT_OUTPUT = "test_env/next_stage/next_stage_external_evidence_checklist.json"
EXECUTION_PREREQUISITES = {
    "python": {
        "required": ">=3.10",
        "recommended": "3.12",
        "command_policy": (
            "Run evidence commands with the project-supported Python interpreter. "
            "On Windows, prefer py -3.12 when registered; otherwise use an "
            "activated 3.12 virtual environment or the full Python 3.12 path."
        ),
    },
    "evidence_policy": (
        "Do not replace blocked external evidence with placeholders. Route "
        "artifacts become acceptable only after the real external input reaches "
        "the target status recorded by the readiness report."
    ),
}
HANDOFF_BY_ARTIFACT: dict[str, dict[str, list[str]]] = {
    "hardware_live_closeout": {
        "evidence_commands": [
            "python tools/build_hardware_live_closeout_report.py --output test_env/hardware_live/hardware_live_closeout_report.json",
        ],
        "input_templates": [
            "deployment/hardware/imc22_live_transport.template.json",
            "deployment/customer_site_live_smoke.template.json",
        ],
        "guide_paths": [
            "docs/hardware/HARDWARE_INTEGRATION_GUIDE.md",
            "docs/guides/CUSTOMER_SITE_REAL_DEVICE_SMOKE_20260427.md",
        ],
    },
    "ros2_typed_idl_cutover": {
        "evidence_commands": [
            "python tools/build_ros2_typed_inventory.py --output test_env/ros2_typed_idl_cutover/typed_inventory.json",
            "python tools/build_ros2_typed_idl_cutover_report.py --output test_env/ros2_typed_idl_cutover/ros2_typed_idl_cutover_report.json",
        ],
        "input_templates": ["deployment/ros2_typed_idl_cutover.template.json"],
        "guide_paths": [
            "docs/ros2/ROS2_QUICK_START.md",
            "docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md",
        ],
    },
    "operator_delivery_checklist": {
        "evidence_commands": [
            "python tools/build_operator_delivery_checklist.py --output test_env/operator_delivery/operator_delivery_checklist.json",
        ],
        "input_templates": ["deployment/operator_delivery_checklist.template.json"],
        "guide_paths": [
            "docs/guides/OPERATOR_DELIVERY_CHECKLIST_AUTOMATION_20260427.md",
            "docs/guides/OPERATOR_HARDWARE_RECOVERY_RUNBOOK_20260427.md",
        ],
    },
    "industrial_live_evidence_archive": {
        "evidence_commands": [
            "python tools/build_customer_site_live_smoke_report.py --output test_env/customer_site_live_smoke/customer_site_live_smoke_report.json",
            (
                "python tools/build_industrial_live_evidence_archive_report.py "
                "--customer-site-smoke test_env/customer_site_live_smoke/customer_site_live_smoke_report.json "
                "--require-customer-site-smoke "
                "--output test_env/industrial_live_evidence/industrial_live_evidence_archive_report.json"
            ),
        ],
        "input_templates": [
            "deployment/external_mainline.inputs.json",
            "deployment/customer_site_live_smoke.template.json",
        ],
        "guide_paths": [
            "docs/guides/INDUSTRIAL_LIVE_EVIDENCE_ARCHIVE_20260427.md",
            "docs/guides/CUSTOMER_SITE_REAL_DEVICE_SMOKE_20260427.md",
        ],
    },
    "vendor_fault_sample_closeout": {
        "evidence_commands": [
            "python tools/build_vendor_fault_sample_closeout.py --output test_env/hardware_live/vendor_fault_sample_closeout.json",
        ],
        "input_templates": [
            "deployment/hardware/imc22_vendor_fault_samples.template.json",
            "deployment/hardware/imc22_reflex_fault_table.json",
            "deployment/hardware/imc22_reflex_recovery_policy.json",
        ],
        "guide_paths": ["docs/hardware/HARDWARE_INTEGRATION_GUIDE.md"],
    },
    "vendor_fault_data_review": {
        "evidence_commands": [
            (
                "python tools/build_vendor_fault_data_review.py "
                "--telemetry-report test_env/hardware_live/hardware_fault_telemetry_report.json "
                "--output test_env/hardware_live/vendor_fault_data_review.json"
            ),
        ],
        "input_templates": [
            "deployment/hardware/imc22_fault_telemetry_fields.json",
            "deployment/hardware/imc22_vendor_fault_samples.template.json",
            "deployment/hardware/imc22_reflex_fault_table.json",
            "deployment/hardware/imc22_reflex_recovery_policy.json",
        ],
        "guide_paths": ["docs/hardware/HARDWARE_INTEGRATION_GUIDE.md"],
    },
    "vendor_data_promotion": {
        "evidence_commands": [
            (
                "python tools/build_vendor_data_promotion_checklist.py "
                "--sample-archive-file deployment/hardware/imc22_vendor_fault_samples.template.json "
                "--vendor-review-file test_env/hardware_live/vendor_fault_data_review.json "
                "--output test_env/hardware_live/vendor_data_promotion_checklist.json"
            ),
        ],
        "input_templates": [
            "deployment/hardware/imc22_vendor_fault_samples.template.json",
            "deployment/hardware/imc22_reflex_fault_table.json",
            "deployment/hardware/imc22_reflex_recovery_policy.json",
        ],
        "guide_paths": ["docs/hardware/HARDWARE_INTEGRATION_GUIDE.md"],
    },
    "web_browser_evidence_pack": {
        "evidence_commands": [
            "python tools/build_web_browser_manual_validation_report.py --output test_env/web_browser_manual_validation/web_browser_manual_validation_report.json",
            "python tools/build_web_browser_validation_closeout.py --output test_env/web_browser_manual_validation/web_browser_validation_closeout.json",
            "python tools/build_web_browser_validation_evidence_pack.py --output test_env/web_browser_manual_validation/web_browser_validation_evidence_pack.json",
        ],
        "input_templates": ["deployment/web_browser_manual_validation.template.json"],
        "guide_paths": [
            "docs/guides/WEB_BROWSER_MANUAL_VALIDATION_CHECKLIST_20260426.md",
            "docs/guides/WEB_PANEL_GUIDE.md",
        ],
    },
}


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build an external evidence checklist from next-stage readiness blockers."
    )
    parser.add_argument(
        "--readiness-report",
        default=DEFAULT_READINESS_REPORT,
        help=(
            "Existing next_stage_readiness_report.json. If it does not exist, "
            "the report is generated from current artifacts."
        ),
    )
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _load_readiness_report(path: str | Path) -> dict[str, Any]:
    report_path = Path(path)
    if report_path.is_file():
        return json.loads(report_path.read_text(encoding="utf-8"))
    return build_next_stage_readiness_report()


def _as_list(value: object) -> list[Any]:
    return value if isinstance(value, list) else []


def _checklist_items(readiness_report: dict[str, Any]) -> list[dict[str, Any]]:
    action_by_id = {
        str(item.get("artifact_id")): item
        for item in _as_list(readiness_report.get("action_plan"))
        if isinstance(item, dict) and item.get("artifact_id") is not None
    }
    items: list[dict[str, Any]] = []
    for detail in _as_list(readiness_report.get("blocker_details")):
        if not isinstance(detail, dict):
            continue
        artifact_id = str(detail.get("id") or "")
        action = action_by_id.get(artifact_id, {})
        issues = [
            str(item)
            for item in [
                *_as_list(detail.get("blockers")),
                *_as_list(detail.get("blocked_steps")),
            ]
        ]
        handoff = _handoff_for_artifact(artifact_id)
        items.append(
            {
                "artifact_id": artifact_id,
                "artifact_path": str(detail.get("path") or ""),
                "actual_status": detail.get("actual_status"),
                "target_status": _target_status(readiness_report, artifact_id),
                "execution_scope": action.get("execution_scope", "unknown"),
                "requires_real_input": bool(action.get("requires_real_input")),
                "issue_count": len(issues),
                "issues": issues,
                "warnings": [str(item) for item in _as_list(detail.get("warnings"))],
                "primary_next_action": str(action.get("primary_next_action") or ""),
                "next_actions": [str(item) for item in _as_list(detail.get("next_actions"))],
                "acceptance_evidence": (
                    f"{detail.get('path')} reaches target status "
                    f"{_target_status(readiness_report, artifact_id)}"
                ),
                **handoff,
            }
        )
    return items


def _handoff_for_artifact(artifact_id: str) -> dict[str, list[str]]:
    handoff = HANDOFF_BY_ARTIFACT.get(artifact_id, {})
    return {
        "evidence_commands": list(handoff.get("evidence_commands", [])),
        "input_templates": list(handoff.get("input_templates", [])),
        "guide_paths": list(handoff.get("guide_paths", [])),
    }


def _target_status(readiness_report: dict[str, Any], artifact_id: str) -> str:
    for artifact in _as_list(readiness_report.get("artifacts")):
        if not isinstance(artifact, dict) or artifact.get("id") != artifact_id:
            continue
        expected = _as_list(artifact.get("expected_statuses"))
        return str(expected[0]) if expected else "ready"
    return "ready"


def build_next_stage_external_evidence_checklist(
    readiness_report: dict[str, Any],
) -> dict[str, Any]:
    readiness_errors = validate_next_stage_readiness_report(readiness_report)
    items = _checklist_items(readiness_report)
    handoff_errors = validate_next_stage_external_evidence_checklist_handoff(items)
    prerequisite_errors = validate_execution_prerequisites(EXECUTION_PREREQUISITES)
    unresolved = [item["artifact_id"] for item in items if item["issue_count"] > 0]
    non_external = [
        item["artifact_id"]
        for item in items
        if item["execution_scope"] != "external_input"
    ]
    status = (
        "ready"
        if not readiness_errors and not handoff_errors and not prerequisite_errors and not unresolved
        else "blocked"
    )
    checklist = {
        "schema_version": SCHEMA_VERSION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "status": status,
        "readiness_status": readiness_report.get("status"),
        "readiness_git": readiness_report.get("git", {}),
        "execution_prerequisites": EXECUTION_PREREQUISITES,
        "summary": {
            "item_count": len(items),
            "unresolved_item_count": len(unresolved),
            "external_input_item_count": sum(
                1 for item in items if item["execution_scope"] == "external_input"
            ),
            "code_or_config_item_count": sum(
                1 for item in items if item["execution_scope"] == "code_or_config"
            ),
            "non_external_item_count": len(non_external),
            "readiness_validation_error_count": len(readiness_errors),
            "handoff_validation_error_count": len(handoff_errors),
            "execution_prerequisite_validation_error_count": len(prerequisite_errors),
        },
        "unresolved_items": unresolved,
        "non_external_items": non_external,
        "items": items,
        "readiness_validation_errors": readiness_errors,
        "handoff_validation_errors": handoff_errors,
        "execution_prerequisite_validation_errors": prerequisite_errors,
        "next_actions": _next_actions(
            status=status,
            unresolved=unresolved,
            non_external=non_external,
            readiness_errors=readiness_errors,
            handoff_errors=handoff_errors,
            prerequisite_errors=prerequisite_errors,
        ),
    }
    checklist["validation_errors"] = validate_next_stage_external_evidence_checklist(checklist)
    return checklist


def validate_next_stage_external_evidence_checklist(payload: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if payload.get("schema_version") != SCHEMA_VERSION:
        errors.append(f"schema_version must be {SCHEMA_VERSION}")
    try:
        generated_at = datetime.fromisoformat(str(payload.get("generated_at")))
    except ValueError:
        generated_at = None
    if generated_at is None or generated_at.tzinfo is None:
        errors.append("generated_at must be a timezone-aware ISO timestamp")
    summary = payload.get("summary")
    if not isinstance(summary, dict):
        return [*errors, "summary must be an object"]
    collections = {
        "items": payload.get("items"),
        "unresolved_items": payload.get("unresolved_items"),
        "non_external_items": payload.get("non_external_items"),
        "readiness_validation_errors": payload.get("readiness_validation_errors"),
        "handoff_validation_errors": payload.get("handoff_validation_errors"),
        "execution_prerequisite_validation_errors": payload.get(
            "execution_prerequisite_validation_errors"
        ),
    }
    if not all(isinstance(value, list) for value in collections.values()):
        return [*errors, "checklist collections must be lists"]
    errors.extend(_validate_checklist_counts(summary=summary, **collections))
    errors.extend(_validate_checklist_items(collections["items"]))
    errors.extend(_validate_checklist_status(payload))
    return errors


def _validate_checklist_counts(
    *,
    summary: dict[str, Any],
    items: list[Any],
    unresolved_items: list[Any],
    non_external_items: list[Any],
    readiness_validation_errors: list[Any],
    handoff_validation_errors: list[Any],
    execution_prerequisite_validation_errors: list[Any],
) -> list[str]:
    expected = {
        "item_count": len(items),
        "unresolved_item_count": len(unresolved_items),
        "external_input_item_count": _count_items(items, "execution_scope", "external_input"),
        "code_or_config_item_count": _count_items(items, "execution_scope", "code_or_config"),
        "non_external_item_count": len(non_external_items),
        "readiness_validation_error_count": len(readiness_validation_errors),
        "handoff_validation_error_count": len(handoff_validation_errors),
        "execution_prerequisite_validation_error_count": len(
            execution_prerequisite_validation_errors
        ),
    }
    errors: list[str] = []
    for field, expected_value in expected.items():
        if not _is_non_negative_int(summary.get(field)):
            errors.append(f"summary.{field} must be a non-negative integer")
        elif summary[field] != expected_value:
            errors.append(f"summary.{field} must equal {expected_value}")
    item_ids = [item.get("artifact_id") for item in items if isinstance(item, dict)]
    expected_unresolved = [
        item.get("artifact_id")
        for item in items
        if isinstance(item, dict) and _issue_count_gt_zero(item)
    ]
    expected_non_external = [
        item.get("artifact_id")
        for item in items
        if isinstance(item, dict) and item.get("execution_scope") != "external_input"
    ]
    if unresolved_items != expected_unresolved:
        errors.append("unresolved_items must match items with issue_count > 0")
    if non_external_items != expected_non_external:
        errors.append("non_external_items must match non-external items")
    if len(item_ids) != len(items):
        errors.append("items must contain artifact_id values")
    return errors


def _validate_checklist_items(items: list[Any]) -> list[str]:
    errors: list[str] = []
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            errors.append(f"items[{index}] must be an object")
            continue
        artifact_id = item.get("artifact_id")
        label = str(artifact_id) if artifact_id else f"items[{index}]"
        for field in (
            "artifact_id",
            "artifact_path",
            "target_status",
            "execution_scope",
            "primary_next_action",
            "acceptance_evidence",
        ):
            if not isinstance(item.get(field), str) or not item.get(field, "").strip():
                errors.append(f"{label}.{field} must be a non-empty string")
        artifact_path = item.get("artifact_path")
        if isinstance(artifact_path, str) and artifact_path.strip():
            path = Path(artifact_path)
            if path.is_absolute() or ".." in path.parts:
                errors.append(f"{label}.artifact_path must be a repository-relative path")
        if item.get("execution_scope") not in {
            "external_input",
            "code_or_config",
            "unknown",
        }:
            errors.append(f"{label}.execution_scope must be a known scope")
        if not isinstance(item.get("requires_real_input"), bool):
            errors.append(f"{label}.requires_real_input must be a boolean")
        if not _is_non_negative_int(item.get("issue_count")):
            errors.append(f"{label}.issue_count must be a non-negative integer")
        actual_status = item.get("actual_status")
        if actual_status is not None and not isinstance(actual_status, str):
            errors.append(f"{label}.actual_status must be a string or null")
        for field in (
            "issues",
            "warnings",
            "next_actions",
            "evidence_commands",
            "input_templates",
            "guide_paths",
        ):
            value = item.get(field)
            if not isinstance(value, list):
                errors.append(f"{label}.{field} must be a list")
            elif any(not isinstance(entry, str) for entry in value):
                errors.append(f"{label}.{field} entries must be strings")
    return errors


def _validate_checklist_status(payload: dict[str, Any]) -> list[str]:
    items = payload.get("items") if isinstance(payload.get("items"), list) else []
    unresolved_from_items = [
        item for item in items if isinstance(item, dict) and _issue_count_gt_zero(item)
    ]
    has_blockers = any(
        payload.get(field)
        for field in (
            "readiness_validation_errors",
            "handoff_validation_errors",
            "execution_prerequisite_validation_errors",
        )
    ) or bool(unresolved_from_items)
    expected_status = "blocked" if has_blockers else "ready"
    if payload.get("status") != expected_status:
        return [f"status must be {expected_status}"]
    return []


def _count_items(items: list[Any], field: str, value: str) -> int:
    return sum(1 for item in items if isinstance(item, dict) and item.get(field) == value)


def _issue_count_gt_zero(item: dict[str, Any]) -> bool:
    issue_count = item.get("issue_count")
    return isinstance(issue_count, int) and issue_count > 0


def _is_non_negative_int(value: Any) -> bool:
    return isinstance(value, int) and value >= 0


def validate_execution_prerequisites(prerequisites: object) -> list[str]:
    if not isinstance(prerequisites, dict):
        return ["execution_prerequisites must be an object"]
    errors: list[str] = []
    python = prerequisites.get("python")
    if not isinstance(python, dict):
        errors.append("execution_prerequisites.python must be an object")
    else:
        for field in ("required", "recommended", "command_policy"):
            if not isinstance(python.get(field), str) or not python.get(field, "").strip():
                errors.append(f"execution_prerequisites.python.{field} must be a non-empty string")
    if (
        not isinstance(prerequisites.get("evidence_policy"), str)
        or not prerequisites.get("evidence_policy", "").strip()
    ):
        errors.append("execution_prerequisites.evidence_policy must be a non-empty string")
    return errors


def validate_next_stage_external_evidence_checklist_handoff(
    items: Sequence[dict[str, Any]],
) -> list[str]:
    errors: list[str] = []
    for index, item in enumerate(items):
        artifact_id = str(item.get("artifact_id") or f"item[{index}]")
        errors.extend(_validate_nonempty_string_list(item, artifact_id, "evidence_commands"))
        for field in ("input_templates", "guide_paths"):
            errors.extend(_validate_existing_repo_paths(item, artifact_id, field))
    return errors


def _validate_nonempty_string_list(
    item: dict[str, Any], artifact_id: str, field: str,
) -> list[str]:
    values = item.get(field)
    if not isinstance(values, list):
        return [f"{artifact_id}.{field} must be a list"]
    if not values:
        return [f"{artifact_id}.{field} must not be empty"]
    return [
        f"{artifact_id}.{field}[{index}] must be a non-empty string"
        for index, value in enumerate(values)
        if not isinstance(value, str) or not value.strip()
    ]


def _validate_existing_repo_paths(
    item: dict[str, Any], artifact_id: str, field: str,
) -> list[str]:
    errors = _validate_nonempty_string_list(item, artifact_id, field)
    values = item.get(field)
    if errors or not isinstance(values, list):
        return errors
    for index, value in enumerate(values):
        path = Path(value)
        if path.is_absolute() or ".." in path.parts:
            errors.append(f"{artifact_id}.{field}[{index}] must be a repository-relative path")
            continue
        if not (PROJECT_ROOT / path).is_file():
            errors.append(f"{artifact_id}.{field}[{index}] does not exist: {value}")
    return errors


def _next_actions(
    *,
    status: str,
    unresolved: list[str],
    non_external: list[str],
    readiness_errors: list[str],
    handoff_errors: list[str],
    prerequisite_errors: list[str],
) -> list[str]:
    if readiness_errors:
        return ["Fix next-stage readiness report validation errors before acting."]
    if handoff_errors:
        return ["Fix next-stage external evidence checklist handoff validation errors before acting."]
    if prerequisite_errors:
        return ["Fix next-stage external evidence checklist execution prerequisites before acting."]
    if non_external:
        return [
            "Resolve code/config scoped next-stage items before requesting external evidence.",
            f"Code/config items: {', '.join(non_external)}.",
        ]
    if status == "blocked":
        return [
            f"Collect real external evidence for unresolved items: {', '.join(unresolved)}.",
            "Rebuild the route-specific artifacts, then rebuild next_stage_readiness_report.json.",
        ]
    return ["All next-stage external evidence items are ready."]


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    readiness_report = _load_readiness_report(args.readiness_report)
    checklist = build_next_stage_external_evidence_checklist(readiness_report)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(checklist, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"next_stage_external_evidence_checklist_written={output.as_posix()}")
    print(f"next_stage_external_evidence_checklist_status={checklist['status']}")
    print(
        "next_stage_external_evidence_checklist_items="
        f"unresolved:{checklist['summary']['unresolved_item_count']},"
        f"external_input:{checklist['summary']['external_input_item_count']},"
        f"code_or_config:{checklist['summary']['code_or_config_item_count']},"
        f"handoff_errors:{checklist['summary']['handoff_validation_error_count']},"
        "prerequisite_errors:"
        f"{checklist['summary']['execution_prerequisite_validation_error_count']},"
        f"validation_errors:{len(checklist['validation_errors'])}"
    )
    return 0 if checklist["status"] == "ready" and not checklist["validation_errors"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
