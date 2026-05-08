from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence


DEFAULT_OUTPUT = "test_env/next_stage/next_stage_readiness_report.json"
SCHEMA_VERSION = "1.0"
EXTERNAL_INPUT_ISSUES = {
    "change_request",
    "customer_site_smoke",
    "external_mainline_industrial_live_evidence_waiting",
    "fault_telemetry",
    "hardware_transport_diagnostics",
    "json_writers_disabled",
    "operator",
    "operator_delivery_checklist",
    "rollback_owner",
    "target_environment",
    "telemetry_entries_missing",
    "telemetry_report_missing",
    "vendor_data_promotion",
    "vendor_review",
}
DEFAULT_ARTIFACTS = {
    "hardware_live_closeout": {
        "path": "test_env/hardware_live/hardware_live_closeout_report.json",
        "ready_statuses": {"ready"},
        "required": True,
    },
    "ros2_typed_idl_cutover": {
        "path": "test_env/ros2_typed_idl_cutover/ros2_typed_idl_cutover_report.json",
        "ready_statuses": {"ready"},
        "required": True,
    },
    "operator_delivery_checklist": {
        "path": "test_env/operator_delivery/operator_delivery_checklist.json",
        "ready_statuses": {"ready"},
        "required": True,
    },
    "industrial_live_evidence_archive": {
        "path": "test_env/industrial_live_evidence/industrial_live_evidence_archive_report.json",
        "ready_statuses": {"ready"},
        "required": True,
    },
    "vendor_fault_sample_closeout": {
        "path": "test_env/hardware_live/vendor_fault_sample_closeout.json",
        "ready_statuses": {"ready"},
        "required": True,
    },
    "vendor_fault_data_review": {
        "path": "test_env/hardware_live/vendor_fault_data_review.json",
        "ready_statuses": {"passed"},
        "required": True,
    },
    "vendor_data_promotion": {
        "path": "test_env/hardware_live/vendor_data_promotion_checklist.json",
        "ready_statuses": {"ready"},
        "required": True,
    },
    "web_browser_evidence_pack": {
        "path": "test_env/web_browser_manual_validation/web_browser_validation_evidence_pack.json",
        "ready_statuses": {"ready"},
        "required": True,
    },
}


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build an aggregate readiness report for the next-stage execution plan."
    )
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _load_json_if_exists(path: str | Path) -> dict[str, Any] | None:
    json_path = Path(path)
    if not json_path.exists():
        return None
    return json.loads(json_path.read_text(encoding="utf-8"))


def _status(payload: dict[str, Any] | None) -> Any:
    if payload is None:
        return None
    return payload.get("status") or payload.get("summary", {}).get("status")


def _list_field(payload: dict[str, Any] | None, field: str) -> list[Any]:
    if payload is None:
        return []
    value = payload.get(field)
    return value if isinstance(value, list) else []


def _blocked_step_ids(payload: dict[str, Any] | None) -> list[str]:
    if payload is None:
        return []
    steps = payload.get("steps")
    if not isinstance(steps, list):
        return []
    return [
        str(step["id"])
        for step in steps
        if isinstance(step, dict)
        and step.get("status") == "blocked"
        and step.get("id") is not None
    ]


def _blocker_detail(
    *,
    artifact_id: str,
    path: str,
    actual_status: Any,
    payload: dict[str, Any] | None,
) -> dict[str, Any]:
    if payload is None:
        return {
            "id": artifact_id,
            "path": path,
            "actual_status": actual_status,
            "blockers": ["evidence_missing"],
            "blocked_steps": [],
            "warnings": [],
            "next_actions": [f"Create missing next-stage evidence: {path}."],
        }
    blockers = _list_field(payload, "blockers")
    blocked_steps = _blocked_step_ids(payload)
    return {
        "id": artifact_id,
        "path": path,
        "actual_status": actual_status,
        "blockers": blockers,
        "blocked_steps": blocked_steps,
        "warnings": _list_field(payload, "warnings"),
        "next_actions": _list_field(payload, "next_actions"),
    }


def _action_plan(blocker_details: list[dict[str, Any]]) -> list[dict[str, Any]]:
    actions: list[dict[str, Any]] = []
    for detail in blocker_details:
        issues = [str(item) for item in detail["blockers"] + detail["blocked_steps"]]
        if not issues:
            issues = ["status_not_ready"]
        external_input_required = all(issue in EXTERNAL_INPUT_ISSUES for issue in issues)
        actions.append(
            {
                "artifact_id": detail["id"],
                "path": detail["path"],
                "execution_scope": (
                    "external_input" if external_input_required else "code_or_config"
                ),
                "requires_real_input": external_input_required,
                "issue_count": len(issues),
                "issues": issues,
                "primary_next_action": (
                    detail["next_actions"][0]
                    if detail["next_actions"]
                    else f"Resolve {detail['id']} before rerunning next-stage readiness."
                ),
            }
        )
    return actions


def build_next_stage_readiness_report() -> dict[str, Any]:
    artifacts: list[dict[str, Any]] = []
    blockers: list[str] = []
    warnings: list[str] = []
    blocker_details: list[dict[str, Any]] = []
    for artifact_id, spec in DEFAULT_ARTIFACTS.items():
        path = str(spec["path"])
        payload = _load_json_if_exists(path)
        actual_status = _status(payload)
        ready = actual_status in spec["ready_statuses"]
        if spec["required"] and not ready:
            blockers.append(artifact_id)
            blocker_details.append(
                _blocker_detail(
                    artifact_id=artifact_id,
                    path=path,
                    actual_status=actual_status,
                    payload=payload,
                )
            )
        elif not ready:
            warnings.append(artifact_id)
        artifacts.append(
            {
                "id": artifact_id,
                "path": path,
                "required": spec["required"],
                "status": "ready" if ready else "blocked",
                "actual_status": actual_status,
                "expected_statuses": sorted(spec["ready_statuses"]),
                "exists": payload is not None,
                "blockers": [] if payload is None else payload.get("blockers", []),
                "warnings": [] if payload is None else payload.get("warnings", []),
            }
        )
    status = "ready" if not blockers else "blocked"
    action_plan = _action_plan(blocker_details)
    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "summary": {
            "artifact_count": len(artifacts),
            "ready_artifact_count": sum(1 for item in artifacts if item["status"] == "ready"),
            "blocked_artifact_count": len(blockers),
            "warning_artifact_count": len(warnings),
            "blocker_detail_count": len(blocker_details),
            "external_input_action_count": sum(
                1 for item in action_plan if item["execution_scope"] == "external_input"
            ),
            "code_or_config_action_count": sum(
                1 for item in action_plan if item["execution_scope"] == "code_or_config"
            ),
        },
        "blockers": blockers,
        "blocker_details": blocker_details,
        "action_plan": action_plan,
        "warnings": warnings,
        "artifacts": artifacts,
        "next_actions": _next_actions(
            status=status,
            blockers=blockers,
            action_plan=action_plan,
        ),
    }


def _next_actions(
    *,
    status: str,
    blockers: list[str],
    action_plan: list[dict[str, Any]],
) -> list[str]:
    if status == "blocked":
        primary_actions = [
            f"{item['artifact_id']}: {item['primary_next_action']}"
            for item in action_plan[:3]
        ]
        return [
            f"Resolve next-stage blockers: {', '.join(blockers)}.",
            *primary_actions,
        ]
    return ["Next-stage execution plan evidence is ready for final archive."]


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    report = build_next_stage_readiness_report()
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0 if report["status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
