from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence


DEFAULT_CHECKLIST = "test_env/hardware_live/live_diagnostics_checklist.json"
DEFAULT_DIAGNOSTICS = "test_env/hardware_live/hardware_transport_diagnostics_report.json"
DEFAULT_TELEMETRY = "test_env/hardware_live/hardware_fault_telemetry_report.json"
DEFAULT_CUSTOMER_SITE_SMOKE = (
    "test_env/customer_site_live_smoke/customer_site_live_smoke_report.json"
)
DEFAULT_OUTPUT = "test_env/hardware_live/hardware_live_closeout_report.json"
SCHEMA_VERSION = "1.0"
PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_EVIDENCE_FIELDS = (
    "checklist",
    "diagnostics",
    "telemetry",
    "customer_site_smoke",
    "vendor_review",
    "vendor_promotion",
)


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a fail-closed closeout report for real hardware live validation."
    )
    parser.add_argument("--checklist", default=DEFAULT_CHECKLIST)
    parser.add_argument("--diagnostics", default=DEFAULT_DIAGNOSTICS)
    parser.add_argument("--telemetry", default=DEFAULT_TELEMETRY)
    parser.add_argument("--customer-site-smoke", default=DEFAULT_CUSTOMER_SITE_SMOKE)
    parser.add_argument("--vendor-review")
    parser.add_argument("--vendor-promotion")
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _load_json_if_exists(path: str | Path | None) -> dict[str, Any] | None:
    if not path:
        return None
    json_path = Path(path)
    if not json_path.exists():
        return None
    return json.loads(json_path.read_text(encoding="utf-8"))


def _text(value: Any) -> str:
    return value.strip() if isinstance(value, str) else ""


def _resolve_source_path(
    value: str | None,
    *,
    output_path: str,
) -> tuple[bool, Path | None, str | None]:
    text = _text(value)
    if not text:
        return True, None, None
    path = Path(text)
    if path.is_absolute():
        return False, None, "absolute"
    if ".." in path.parts:
        return False, None, "parent_directory"

    output_relative = Path(output_path).resolve().parent / path
    if output_relative.exists():
        return True, output_relative, None
    return True, PROJECT_ROOT / path, None


def _source_path_statuses(
    sources: dict[str, str | None],
    *,
    output_path: str,
) -> dict[str, dict[str, Any]]:
    statuses: dict[str, dict[str, Any]] = {}
    for field in SOURCE_EVIDENCE_FIELDS:
        source = sources.get(field)
        valid, resolved_path, error = _resolve_source_path(source, output_path=output_path)
        statuses[field] = {
            "path": _text(source),
            "path_valid": valid,
            "path_error": error,
            "resolved_path": str(resolved_path) if resolved_path is not None else None,
            "exists": bool(resolved_path and resolved_path.exists()),
        }
    return statuses


def _status(payload: dict[str, Any] | None) -> Any:
    if payload is None:
        return None
    return payload.get("status") or payload.get("summary", {}).get("status")


def _telemetry_entries(payload: dict[str, Any] | None) -> list[Any]:
    if payload is None:
        return []
    entries = payload.get("entries")
    return entries if isinstance(entries, list) else []


def _evidence_result(
    evidence_id: str,
    path: str | None,
    payload: dict[str, Any] | None,
    *,
    expected_statuses: set[str],
    required: bool,
    path_valid: bool = True,
    extra_ready: bool = True,
    extra_blocker_reason: str = "extra_condition_failed",
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
    elif not extra_ready:
        status = "blocked" if required else "warning"
        reason = extra_blocker_reason
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


def build_hardware_live_closeout_report(
    *,
    sources: dict[str, str | None],
    checklist: dict[str, Any] | None,
    diagnostics: dict[str, Any] | None,
    telemetry: dict[str, Any] | None,
    customer_site_smoke: dict[str, Any] | None,
    vendor_review: dict[str, Any] | None,
    vendor_promotion: dict[str, Any] | None,
    source_path_statuses: dict[str, dict[str, Any]] | None = None,
) -> dict[str, Any]:
    if source_path_statuses is None:
        source_path_statuses = {
            field: {"path_valid": True} for field in SOURCE_EVIDENCE_FIELDS
        }
    telemetry_entries = _telemetry_entries(telemetry)
    evidence = [
        _evidence_result(
            "live_diagnostics_checklist",
            sources.get("checklist"),
            checklist,
            expected_statuses={"ready_to_run", "ready", "passed"},
            required=True,
            path_valid=source_path_statuses["checklist"]["path_valid"],
        ),
        _evidence_result(
            "hardware_transport_diagnostics",
            sources.get("diagnostics"),
            diagnostics,
            expected_statuses={"ready"},
            required=True,
            path_valid=source_path_statuses["diagnostics"]["path_valid"],
        ),
        _evidence_result(
            "fault_telemetry",
            sources.get("telemetry"),
            telemetry,
            expected_statuses={"ready", "passed"},
            required=True,
            path_valid=source_path_statuses["telemetry"]["path_valid"],
            extra_ready=bool(telemetry_entries),
            extra_blocker_reason="telemetry_entries_missing",
        ),
        _evidence_result(
            "customer_site_smoke",
            sources.get("customer_site_smoke"),
            customer_site_smoke,
            expected_statuses={"passed"},
            required=True,
            path_valid=source_path_statuses["customer_site_smoke"]["path_valid"],
        ),
        _evidence_result(
            "vendor_review",
            sources.get("vendor_review"),
            vendor_review,
            expected_statuses={"passed"},
            required=False,
            path_valid=source_path_statuses["vendor_review"]["path_valid"],
        ),
        _evidence_result(
            "vendor_promotion",
            sources.get("vendor_promotion"),
            vendor_promotion,
            expected_statuses={"ready"},
            required=False,
            path_valid=source_path_statuses["vendor_promotion"]["path_valid"],
        ),
    ]
    blockers = [
        item["id"]
        for item in evidence
        if item["required"] and item["status"] == "blocked"
    ]
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
            "transport": None if checklist is None else checklist.get("transport"),
            "telemetry_entry_count": len(telemetry_entries),
            "required_blocker_count": len(blockers),
            "optional_warning_count": len(warnings),
            "diagnostics_status": _status(diagnostics),
            "customer_site_smoke_status": _status(customer_site_smoke),
            "source_path_validation_error_count": sum(
                1 for item in source_path_statuses.values() if not item["path_valid"]
            ),
        },
        "blockers": blockers,
        "warnings": warnings,
        "evidence": evidence,
        "next_actions": _next_actions(status=status, blockers=blockers, warnings=warnings),
        "source_path_statuses": source_path_statuses,
    }


def _next_actions(
    *, status: str, blockers: list[str], warnings: list[str]
) -> list[str]:
    if status == "blocked":
        return [f"Resolve hardware live closeout blockers: {', '.join(blockers)}."]
    if warnings:
        return [
            "Hardware live closeout is ready; review optional vendor warnings before customer signoff."
        ]
    return ["Hardware live closeout is ready for live evidence archive."]


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    sources = {
        "checklist": args.checklist,
        "diagnostics": args.diagnostics,
        "telemetry": args.telemetry,
        "customer_site_smoke": args.customer_site_smoke,
        "vendor_review": args.vendor_review,
        "vendor_promotion": args.vendor_promotion,
    }
    source_path_statuses = _source_path_statuses(sources, output_path=args.output)
    report = build_hardware_live_closeout_report(
        sources=sources,
        checklist=_load_json_if_exists(source_path_statuses["checklist"]["resolved_path"]),
        diagnostics=_load_json_if_exists(
            source_path_statuses["diagnostics"]["resolved_path"]
        ),
        telemetry=_load_json_if_exists(source_path_statuses["telemetry"]["resolved_path"]),
        customer_site_smoke=_load_json_if_exists(
            source_path_statuses["customer_site_smoke"]["resolved_path"]
        ),
        vendor_review=_load_json_if_exists(
            source_path_statuses["vendor_review"]["resolved_path"]
        ),
        vendor_promotion=_load_json_if_exists(
            source_path_statuses["vendor_promotion"]["resolved_path"]
        ),
        source_path_statuses=source_path_statuses,
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
