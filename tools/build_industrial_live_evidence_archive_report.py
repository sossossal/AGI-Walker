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
PROJECT_ROOT = Path(__file__).resolve().parents[1]
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
SOURCE_EVIDENCE_FIELDS = (
    "operator_checklist",
    "external_mainline_plan",
    "hardware_diagnostics",
    "vendor_promotion",
    "browser_closeout",
    "customer_site_smoke",
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
    parser.add_argument(
        "--require-customer-site-smoke",
        action="store_true",
        help=(
            "Require --customer-site-smoke to point to a passed customer-site "
            "real device smoke report."
        ),
    )
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


def _resolve_source_path(
    value: str | None,
    *,
    inputs_file: str,
) -> tuple[bool, Path | None, str | None]:
    text = _text(value)
    if not text:
        return True, None, None
    path = Path(text)
    if path.is_absolute():
        return False, None, "absolute"
    if ".." in path.parts:
        return False, None, "parent_directory"

    input_relative = Path(inputs_file).resolve().parent / path
    if input_relative.exists():
        return True, input_relative, None
    return True, PROJECT_ROOT / path, None


def _source_path_statuses(
    sources: dict[str, str | None],
    *,
    inputs_file: str,
) -> dict[str, dict[str, Any]]:
    statuses: dict[str, dict[str, Any]] = {}
    for field in SOURCE_EVIDENCE_FIELDS:
        source = sources.get(field)
        valid, resolved_path, error = _resolve_source_path(source, inputs_file=inputs_file)
        statuses[field] = {
            "path": _text(source),
            "path_valid": valid,
            "path_error": error,
            "resolved_path": str(resolved_path) if resolved_path is not None else None,
            "exists": bool(resolved_path and resolved_path.exists()),
        }
    return statuses


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
    path_valid: bool = True,
) -> dict[str, Any]:
    actual_status = _status(payload)
    if not path_valid:
        status = "blocked" if required else "warning"
        reason = "evidence_path_invalid"
    elif payload is None:
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
    source_path_statuses: dict[str, dict[str, Any]] | None = None,
    require_customer_site_smoke: bool = False,
) -> dict[str, Any]:
    if source_path_statuses is None:
        source_path_statuses = {
            field: {"path_valid": True} for field in SOURCE_EVIDENCE_FIELDS
        }
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
            path_valid=source_path_statuses["operator_checklist"]["path_valid"],
        ),
        _evidence_result(
            "external_mainline_plan",
            sources.get("external_mainline_plan"),
            external_mainline_plan,
            expected_statuses={"ready", "ready_to_run", "passed", "completed"},
            required=True,
            path_valid=source_path_statuses["external_mainline_plan"]["path_valid"],
        ),
        _evidence_result(
            "hardware_diagnostics",
            sources.get("hardware_diagnostics"),
            hardware_diagnostics,
            expected_statuses={"ready", "ready_to_run", "passed"},
            required=False,
            path_valid=source_path_statuses["hardware_diagnostics"]["path_valid"],
        ),
        _evidence_result(
            "vendor_promotion",
            sources.get("vendor_promotion"),
            vendor_promotion,
            expected_statuses={"ready"},
            required=False,
            path_valid=source_path_statuses["vendor_promotion"]["path_valid"],
        ),
        _evidence_result(
            "browser_closeout",
            sources.get("browser_closeout"),
            browser_closeout,
            expected_statuses={"passed"},
            required=False,
            path_valid=source_path_statuses["browser_closeout"]["path_valid"],
        ),
        _evidence_result(
            "customer_site_smoke",
            sources.get("customer_site_smoke"),
            customer_site_smoke,
            expected_statuses={"passed"},
            required=require_customer_site_smoke,
            path_valid=source_path_statuses["customer_site_smoke"]["path_valid"],
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
            "require_customer_site_smoke": require_customer_site_smoke,
            "source_path_validation_error_count": sum(
                1 for item in source_path_statuses.values() if not item["path_valid"]
            ),
        },
        "missing_fields": missing_fields,
        "blockers": blockers,
        "warnings": warnings,
        "industrial_live_evidence": {
            field: live_inputs.get(field) for field in REQUIRED_FIELDS
        },
        "evidence": evidence,
        "next_actions": _next_actions(status=status, blockers=blockers, warnings=warnings),
        "source_path_statuses": source_path_statuses,
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
    source_path_statuses = _source_path_statuses(sources, inputs_file=args.inputs_file)
    report = build_industrial_live_evidence_archive_report(
        inputs_payload=_load_json(args.inputs_file),
        sources=sources,
        source_path_statuses=source_path_statuses,
        external_mainline_plan=_load_json_if_exists(
            source_path_statuses["external_mainline_plan"]["resolved_path"]
        ),
        operator_checklist=_load_json_if_exists(
            source_path_statuses["operator_checklist"]["resolved_path"]
        ),
        hardware_diagnostics=_load_json_if_exists(
            source_path_statuses["hardware_diagnostics"]["resolved_path"]
        ),
        vendor_promotion=_load_json_if_exists(
            source_path_statuses["vendor_promotion"]["resolved_path"]
        ),
        browser_closeout=_load_json_if_exists(
            source_path_statuses["browser_closeout"]["resolved_path"]
        ),
        customer_site_smoke=_load_json_if_exists(
            source_path_statuses["customer_site_smoke"]["resolved_path"]
        ),
        require_customer_site_smoke=args.require_customer_site_smoke,
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
