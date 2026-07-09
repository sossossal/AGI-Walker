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
            }
        )
    return items


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
    unresolved = [item["artifact_id"] for item in items if item["issue_count"] > 0]
    non_external = [
        item["artifact_id"]
        for item in items
        if item["execution_scope"] != "external_input"
    ]
    status = "ready" if not readiness_errors and not unresolved else "blocked"
    return {
        "schema_version": SCHEMA_VERSION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "status": status,
        "readiness_status": readiness_report.get("status"),
        "readiness_git": readiness_report.get("git", {}),
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
        },
        "unresolved_items": unresolved,
        "non_external_items": non_external,
        "items": items,
        "readiness_validation_errors": readiness_errors,
        "next_actions": _next_actions(
            status=status,
            unresolved=unresolved,
            non_external=non_external,
            readiness_errors=readiness_errors,
        ),
    }


def _next_actions(
    *,
    status: str,
    unresolved: list[str],
    non_external: list[str],
    readiness_errors: list[str],
) -> list[str]:
    if readiness_errors:
        return ["Fix next-stage readiness report validation errors before acting."]
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
        f"code_or_config:{checklist['summary']['code_or_config_item_count']}"
    )
    return 0 if checklist["status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
