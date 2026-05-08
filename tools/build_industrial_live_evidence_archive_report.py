from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence


DEFAULT_INPUTS = "deployment/external_mainline.inputs.json"
DEFAULT_OPERATOR_CHECKLIST = "test_env/operator_delivery/operator_delivery_checklist.json"
DEFAULT_EXTERNAL_MAINLINE_PLAN = (
    "test_env/release_evidence/operations/external_mainline_execution_plan.json"
)
DEFAULT_OUTPUT = "test_env/industrial_live_evidence/industrial_live_evidence_archive_report.json"
SCHEMA_VERSION = "1.0"
REQUIRED_FIELDS = (
    "target_environment",
    "access_method",
    "install_entrypoint",
    "upgrade_entrypoint",
    "rollback_entrypoint",
    "backup_restore_entrypoint",
    "closure_archive_root",
    "evidence_output_root",
)


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build an archive report for industrial live evidence inputs and evidence files."
    )
    parser.add_argument("--inputs-file", default=DEFAULT_INPUTS)
    parser.add_argument("--operator-checklist", default=DEFAULT_OPERATOR_CHECKLIST)
    parser.add_argument("--external-mainline-plan", default=DEFAULT_EXTERNAL_MAINLINE_PLAN)
    parser.add_argument("--hardware-diagnostics")
    parser.add_argument("--vendor-promotion")
    parser.add_argument("--browser-closeout")
    parser.add_argument("--customer-site-smoke")
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _load_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _load_json_if_exists(path: str | Path | None) -> dict[str, Any] | None:
    if not path:
        return None
    json_path = Path(path)
    if not json_path.exists():
        return None
    return _load_json(json_path)


def _text(value: Any) -> str:
    return value.strip() if isinstance(value, str) else ""


def _is_placeholder(value: Any) -> bool:
    text = _text(value)
    return not text or "<" in text or ">" in text


def _status(payload: dict[str, Any] | None) -> Any:
    if payload is None:
        return None
    return payload.get("status") or payload.get("summary", {}).get("status")


def _external_mainline_industrial_step(
    payload: dict[str, Any] | None,
) -> dict[str, Any] | None:
    if payload is None:
        return None
    steps = payload.get("steps")
    if isinstance(steps, dict):
        step = steps.get("industrial_delivery_live_evidence")
        return step if isinstance(step, dict) else None
    if isinstance(steps, list):
        for step in steps:
            if (
                isinstance(step, dict)
                and step.get("id") == "industrial_delivery_live_evidence"
            ):
                return step
    return None


def _evidence_result(
    evidence_id: str,
    path: str | None,
    payload: dict[str, Any] | None,
    *,
    expected_statuses: set[str],
    required: bool,
) -> dict[str, Any]:
    actual_status = _status(payload)
    if payload is None:
        status = "blocked" if required else "warning"
        reason = "evidence_missing"
    elif actual_status not in expected_statuses:
        status = "blocked" if required else "warning"
        reason = "unexpected_status"
    else:
        status = "ready"
        reason = "ok"
    return {
        "id": evidence_id,
        "required": required,
        "status": status,
        "reason": reason,
        "path": path,
        "actual_status": actual_status,
        "expected_statuses": sorted(expected_statuses),
    }


def build_industrial_live_evidence_archive_report(
    *,
    inputs_payload: dict[str, Any],
    sources: dict[str, str | None],
    external_mainline_plan: dict[str, Any] | None,
    operator_checklist: dict[str, Any] | None,
    hardware_diagnostics: dict[str, Any] | None,
    vendor_promotion: dict[str, Any] | None,
    browser_closeout: dict[str, Any] | None,
    customer_site_smoke: dict[str, Any] | None,
) -> dict[str, Any]:
    live_inputs = inputs_payload.get("industrial_live_evidence")
    if not isinstance(live_inputs, dict):
        live_inputs = {}

    missing_fields = [
        field for field in REQUIRED_FIELDS if _is_placeholder(live_inputs.get(field))
    ]
    industrial_step = _external_mainline_industrial_step(external_mainline_plan)
    industrial_step_status = None if industrial_step is None else industrial_step.get("status")
    managed_inputs_ready = (
        None if industrial_step is None else industrial_step.get("managed_inputs_ready")
    )

    evidence = [
        _evidence_result(
            "operator_delivery_checklist",
            sources.get("operator_checklist"),
            operator_checklist,
            expected_statuses={"ready"},
            required=True,
        ),
        _evidence_result(
            "external_mainline_plan",
            sources.get("external_mainline_plan"),
            external_mainline_plan,
            expected_statuses={"ready", "ready_to_run", "passed", "completed"},
            required=True,
        ),
        _evidence_result(
            "hardware_diagnostics",
            sources.get("hardware_diagnostics"),
            hardware_diagnostics,
            expected_statuses={"ready", "ready_to_run", "passed"},
            required=False,
        ),
        _evidence_result(
            "vendor_promotion",
            sources.get("vendor_promotion"),
            vendor_promotion,
            expected_statuses={"ready"},
            required=False,
        ),
        _evidence_result(
            "browser_closeout",
            sources.get("browser_closeout"),
            browser_closeout,
            expected_statuses={"passed"},
            required=False,
        ),
        _evidence_result(
            "customer_site_smoke",
            sources.get("customer_site_smoke"),
            customer_site_smoke,
            expected_statuses={"passed"},
            required=False,
        ),
    ]

    blockers = []
    if missing_fields:
        blockers.append("industrial_live_evidence_fields")
    if industrial_step_status == "waiting_external_input":
        blockers.append("external_mainline_industrial_live_evidence_waiting")
    blockers.extend(
        item["id"]
        for item in evidence
        if item["required"] and item["status"] == "blocked"
    )
    warnings = [
        item["id"]
        for item in evidence
        if not item["required"] and item["status"] == "warning"
    ]
    status = "ready" if not blockers else "blocked"

    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "sources": sources,
        "summary": {
            "target_environment": live_inputs.get("target_environment"),
            "closure_archive_root": live_inputs.get("closure_archive_root"),
            "evidence_output_root": live_inputs.get("evidence_output_root"),
            "missing_field_count": len(missing_fields),
            "required_evidence_blocker_count": sum(
                1
                for item in evidence
                if item["required"] and item["status"] == "blocked"
            ),
            "optional_evidence_warning_count": len(warnings),
            "external_mainline_industrial_step_status": industrial_step_status,
            "external_mainline_managed_inputs_ready": managed_inputs_ready,
        },
        "missing_fields": missing_fields,
        "blockers": blockers,
        "warnings": warnings,
        "industrial_live_evidence": {
            field: live_inputs.get(field) for field in REQUIRED_FIELDS
        },
        "evidence": evidence,
        "next_actions": _next_actions(status=status, blockers=blockers, warnings=warnings),
    }


def _next_actions(
    *, status: str, blockers: list[str], warnings: list[str]
) -> list[str]:
    if status == "blocked":
        return [f"Resolve industrial live evidence blockers: {', '.join(blockers)}."]
    if warnings:
        return [
            "Industrial live evidence archive report is ready; review optional evidence warnings before customer signoff."
        ]
    return ["Industrial live evidence archive report is ready for customer archive."]


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    sources = {
        "inputs_file": args.inputs_file,
        "operator_checklist": args.operator_checklist,
        "external_mainline_plan": args.external_mainline_plan,
        "hardware_diagnostics": args.hardware_diagnostics,
        "vendor_promotion": args.vendor_promotion,
        "browser_closeout": args.browser_closeout,
        "customer_site_smoke": args.customer_site_smoke,
    }
    report = build_industrial_live_evidence_archive_report(
        inputs_payload=_load_json(args.inputs_file),
        sources=sources,
        external_mainline_plan=_load_json_if_exists(args.external_mainline_plan),
        operator_checklist=_load_json_if_exists(args.operator_checklist),
        hardware_diagnostics=_load_json_if_exists(args.hardware_diagnostics),
        vendor_promotion=_load_json_if_exists(args.vendor_promotion),
        browser_closeout=_load_json_if_exists(args.browser_closeout),
        customer_site_smoke=_load_json_if_exists(args.customer_site_smoke),
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0 if report["status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
