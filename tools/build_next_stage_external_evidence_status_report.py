#!/usr/bin/env python
"""Build a status report for next-stage external evidence checklist items."""

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

from tools.build_next_stage_external_evidence_checklist import (
    DEFAULT_OUTPUT as DEFAULT_CHECKLIST,
    validate_next_stage_external_evidence_checklist,
)


SCHEMA_VERSION = "next_stage_external_evidence_status_report.v1"
DEFAULT_OUTPUT = "test_env/next_stage/next_stage_external_evidence_status_report.json"


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a status report for next-stage external evidence checklist items."
    )
    parser.add_argument("--checklist", default=DEFAULT_CHECKLIST)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--expected-status",
        choices=("blocked", "ready"),
        default=None,
        help=(
            "Return success when the generated report has this status and "
            "self-validation passes. Omit to require ready."
        ),
    )
    return parser.parse_args(argv)


def _load_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _resolve_repo_relative_path(path: str | Path) -> Path | None:
    candidate = Path(path)
    if candidate.is_absolute() or ".." in candidate.parts or not str(path).strip():
        return None
    return PROJECT_ROOT / candidate


def _load_json_if_exists(path: str | Path) -> dict[str, Any] | None:
    json_path = _resolve_repo_relative_path(path)
    if json_path is None:
        return None
    if not json_path.is_file():
        return None
    return json.loads(json_path.read_text(encoding="utf-8"))


def _artifact_status(payload: dict[str, Any] | None) -> Any:
    if payload is None:
        return None
    summary = payload.get("summary")
    return payload.get("status") or (
        summary.get("status") if isinstance(summary, dict) else None
    )


def _is_non_negative_int(value: Any) -> bool:
    return isinstance(value, int) and value >= 0


def build_next_stage_external_evidence_status_report(
    checklist: dict[str, Any],
) -> dict[str, Any]:
    checklist_errors = validate_next_stage_external_evidence_checklist(checklist)
    items = checklist.get("items") if isinstance(checklist.get("items"), list) else []
    evidence_items = [_evidence_item(item) for item in items if isinstance(item, dict)]
    blocked_items = [
        item["artifact_id"] for item in evidence_items if not item["ready"]
    ]
    status = "ready" if not checklist_errors and not blocked_items else "blocked"
    report = {
        "schema_version": SCHEMA_VERSION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "status": status,
        "checklist_status": checklist.get("status"),
        "checklist_validation_errors": checklist_errors,
        "summary": {
            "item_count": len(evidence_items),
            "ready_item_count": sum(1 for item in evidence_items if item["ready"]),
            "blocked_item_count": len(blocked_items),
            "missing_item_count": sum(1 for item in evidence_items if not item["exists"]),
            "checklist_validation_error_count": len(checklist_errors),
        },
        "blocked_items": blocked_items,
        "items": evidence_items,
        "next_actions": _next_actions(
            checklist_errors=checklist_errors,
            blocked_items=blocked_items,
        ),
    }
    report["validation_errors"] = validate_next_stage_external_evidence_status_report(
        report
    )
    return report


def _evidence_item(item: dict[str, Any]) -> dict[str, Any]:
    artifact_id = str(item.get("artifact_id") or "")
    artifact_path = str(item.get("artifact_path") or "")
    target_status = str(item.get("target_status") or "ready")
    artifact_path_valid = _resolve_repo_relative_path(artifact_path) is not None
    payload = _load_json_if_exists(artifact_path) if artifact_path_valid else None
    actual_status = _artifact_status(payload)
    ready = actual_status == target_status
    return {
        "artifact_id": artifact_id,
        "artifact_path": artifact_path,
        "artifact_path_valid": artifact_path_valid,
        "exists": payload is not None,
        "target_status": target_status,
        "actual_status": actual_status,
        "ready": ready,
        "requires_real_input": bool(item.get("requires_real_input")),
        "execution_scope": item.get("execution_scope"),
        "issue_count": 0 if ready else int(item.get("issue_count") or 0),
        "remaining_issues": [] if ready else list(item.get("issues") or []),
        "acceptance_evidence": item.get("acceptance_evidence"),
    }


def validate_next_stage_external_evidence_status_report(
    report: dict[str, Any],
) -> list[str]:
    errors: list[str] = []
    if report.get("schema_version") != SCHEMA_VERSION:
        errors.append(f"schema_version must be {SCHEMA_VERSION}")
    try:
        generated_at = datetime.fromisoformat(str(report.get("generated_at")))
    except ValueError:
        generated_at = None
    if generated_at is None or generated_at.tzinfo is None:
        errors.append("generated_at must be a timezone-aware ISO timestamp")
    summary = report.get("summary")
    items = report.get("items")
    blocked_items = report.get("blocked_items")
    checklist_errors = report.get("checklist_validation_errors")
    if not isinstance(summary, dict) or not isinstance(items, list) or not isinstance(
        blocked_items, list
    ) or not isinstance(checklist_errors, list):
        return [*errors, "status report collections and summary must use canonical shapes"]
    expected = {
        "item_count": len(items),
        "ready_item_count": sum(
            1 for item in items if isinstance(item, dict) and item.get("ready") is True
        ),
        "blocked_item_count": len(blocked_items),
        "missing_item_count": sum(
            1 for item in items if isinstance(item, dict) and item.get("exists") is False
        ),
        "checklist_validation_error_count": len(checklist_errors),
    }
    for field, expected_value in expected.items():
        if not _is_non_negative_int(summary.get(field)):
            errors.append(f"summary.{field} must be a non-negative integer")
        elif summary[field] != expected_value:
            errors.append(f"summary.{field} must equal {expected_value}")
    item_errors = _validate_status_report_items(items)
    errors.extend(item_errors)
    expected_blocked = [
        item.get("artifact_id")
        for item in items
        if isinstance(item, dict) and item.get("ready") is not True
    ]
    invalid_paths = [
        item.get("artifact_id")
        for item in items
        if isinstance(item, dict) and item.get("artifact_path_valid") is not True
    ]
    if blocked_items != expected_blocked:
        errors.append("blocked_items must match non-ready items")
    if invalid_paths:
        errors.append(
            "items must use repository-relative artifact_path values: "
            + ", ".join(str(item) for item in invalid_paths)
        )
    expected_status = (
        "blocked" if checklist_errors or expected_blocked or item_errors else "ready"
    )
    if report.get("status") != expected_status:
        errors.append(f"status must be {expected_status}")
    return errors


def _validate_status_report_items(items: list[Any]) -> list[str]:
    errors: list[str] = []
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            errors.append(f"items[{index}] must be an object")
            continue
        artifact_id = item.get("artifact_id")
        label = str(artifact_id) if artifact_id else f"items[{index}]"
        for field in ("artifact_id", "artifact_path", "target_status"):
            if not isinstance(item.get(field), str) or not item.get(field, "").strip():
                errors.append(f"{label}.{field} must be a non-empty string")
        for field in (
            "artifact_path_valid",
            "exists",
            "ready",
            "requires_real_input",
        ):
            if not isinstance(item.get(field), bool):
                errors.append(f"{label}.{field} must be a boolean")
        if item.get("execution_scope") not in {
            "external_input",
            "code_or_config",
            "unknown",
        }:
            errors.append(f"{label}.execution_scope must be a known scope")
        if not _is_non_negative_int(item.get("issue_count")):
            errors.append(f"{label}.issue_count must be a non-negative integer")
        if not isinstance(item.get("remaining_issues"), list):
            errors.append(f"{label}.remaining_issues must be a list")
        actual_status = item.get("actual_status")
        if actual_status is not None and not isinstance(actual_status, str):
            errors.append(f"{label}.actual_status must be a string or null")
    return errors


def _next_actions(
    *, checklist_errors: list[str], blocked_items: list[str],
) -> list[str]:
    if checklist_errors:
        return ["Fix next-stage external evidence checklist validation errors first."]
    if blocked_items:
        return [
            f"Collect or rebuild real external evidence for: {', '.join(blocked_items)}.",
            "Rerun this status report, then rerun next-stage readiness.",
        ]
    return ["All checklist evidence artifacts match their target statuses."]


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    checklist = _load_json(args.checklist)
    report = build_next_stage_external_evidence_status_report(checklist)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"next_stage_external_evidence_status_report_written={output.as_posix()}")
    print(f"next_stage_external_evidence_status_report_status={report['status']}")
    print(
        "next_stage_external_evidence_status_report_items="
        f"ready:{report['summary']['ready_item_count']},"
        f"blocked:{report['summary']['blocked_item_count']},"
        f"missing:{report['summary']['missing_item_count']},"
        f"checklist_errors:{report['summary']['checklist_validation_error_count']},"
        f"validation_errors:{len(report['validation_errors'])}"
    )
    if report["validation_errors"]:
        return 1
    required_status = args.expected_status or "ready"
    return 0 if report["status"] == required_status else 1


if __name__ == "__main__":
    raise SystemExit(main())
