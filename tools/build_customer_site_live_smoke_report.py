from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence


DEFAULT_INPUT = "deployment/customer_site_live_smoke.template.json"
DEFAULT_OUTPUT = "test_env/customer_site_live_smoke/customer_site_live_smoke_report.json"
SCHEMA_VERSION = "1.0"
REQUIRED_SITE_FIELDS = (
    "engagement_id",
    "site_id",
    "target_environment",
    "operator",
    "started_at",
    "finished_at",
)
REQUIRED_DEVICE_FIELDS = (
    "vendor",
    "device_id",
    "transport",
    "endpoint",
    "firmware_version",
)
REQUIRED_SAFETY_FLAGS = (
    "emergency_stop_verified",
    "power_verified",
    "mechanical_clearance_verified",
    "operator_has_hardware_recovery_role",
)
PASS_STATUSES = {"passed", "ready"}


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a fail-closed report for customer-site real device smoke evidence."
    )
    parser.add_argument("--input", default=DEFAULT_INPUT)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--require-evidence-files",
        action="store_true",
        help="Require every check evidence_path to exist on disk.",
    )
    return parser.parse_args(argv)


def _load_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _text(value: Any) -> str:
    return value.strip() if isinstance(value, str) else ""


def _is_placeholder(value: Any) -> bool:
    text = _text(value)
    return not text or "<" in text or ">" in text


def _missing_fields(payload: dict[str, Any], prefix: str, fields: Sequence[str]) -> list[str]:
    section = payload.get(prefix)
    if not isinstance(section, dict):
        return [f"{prefix}.{field}" for field in fields]
    return [
        f"{prefix}.{field}"
        for field in fields
        if _is_placeholder(section.get(field))
    ]


def _safety_blockers(payload: dict[str, Any]) -> list[str]:
    safety = payload.get("safety_precheck")
    if not isinstance(safety, dict):
        return [f"safety_precheck.{flag}" for flag in REQUIRED_SAFETY_FLAGS]
    return [
        f"safety_precheck.{flag}"
        for flag in REQUIRED_SAFETY_FLAGS
        if safety.get(flag) is not True
    ]


def _check_results(
    checks: Any, *, require_evidence_files: bool
) -> tuple[list[dict[str, Any]], list[str]]:
    if not isinstance(checks, list) or not checks:
        return [], ["checks"]
    results: list[dict[str, Any]] = []
    blockers: list[str] = []
    for index, check in enumerate(checks):
        if not isinstance(check, dict):
            blockers.append(f"checks[{index}]")
            continue
        check_id = _text(check.get("id")) or f"checks[{index}]"
        evidence_path = _text(check.get("evidence_path"))
        evidence_exists = bool(evidence_path) and Path(evidence_path).exists()
        status = _text(check.get("status"))
        reason = "ok"
        result_status = "ready"
        if status not in PASS_STATUSES:
            reason = "check_not_passed"
            result_status = "blocked"
            blockers.append(check_id)
        elif require_evidence_files and not evidence_exists:
            reason = "evidence_file_missing"
            result_status = "blocked"
            blockers.append(check_id)
        results.append(
            {
                "id": check_id,
                "status": result_status,
                "reason": reason,
                "actual_status": status,
                "evidence_path": evidence_path,
                "evidence_exists": evidence_exists,
                "description": check.get("description", ""),
            }
        )
    return results, blockers


def build_customer_site_live_smoke_report(
    *,
    payload: dict[str, Any],
    input_path: str,
    require_evidence_files: bool = False,
) -> dict[str, Any]:
    missing_fields = [
        *_missing_fields(payload, "site", REQUIRED_SITE_FIELDS),
        *_missing_fields(payload, "device", REQUIRED_DEVICE_FIELDS),
    ]
    safety_blockers = _safety_blockers(payload)
    check_results, check_blockers = _check_results(
        payload.get("checks"), require_evidence_files=require_evidence_files
    )
    archive = payload.get("archive") if isinstance(payload.get("archive"), dict) else {}
    archive_blockers = []
    if _is_placeholder(archive.get("closure_archive_root")):
        archive_blockers.append("archive.closure_archive_root")

    blockers = [
        *missing_fields,
        *safety_blockers,
        *archive_blockers,
        *check_blockers,
    ]
    status = "passed" if not blockers else "blocked"
    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "input": input_path,
        "summary": {
            "site_id": payload.get("site", {}).get("site_id")
            if isinstance(payload.get("site"), dict)
            else None,
            "target_environment": payload.get("site", {}).get("target_environment")
            if isinstance(payload.get("site"), dict)
            else None,
            "device_id": payload.get("device", {}).get("device_id")
            if isinstance(payload.get("device"), dict)
            else None,
            "transport": payload.get("device", {}).get("transport")
            if isinstance(payload.get("device"), dict)
            else None,
            "passed_check_count": sum(
                1 for result in check_results if result["status"] == "ready"
            ),
            "blocked_check_count": sum(
                1 for result in check_results if result["status"] == "blocked"
            ),
            "missing_field_count": len(missing_fields),
            "safety_blocker_count": len(safety_blockers),
            "require_evidence_files": require_evidence_files,
        },
        "blockers": blockers,
        "missing_fields": missing_fields,
        "safety_blockers": safety_blockers,
        "archive_blockers": archive_blockers,
        "checks": check_results,
        "archive": {
            "closure_archive_root": archive.get("closure_archive_root"),
            "report_output": archive.get("report_output"),
        },
        "next_actions": _next_actions(status=status, blockers=blockers),
    }


def _next_actions(*, status: str, blockers: list[str]) -> list[str]:
    if status == "blocked":
        return [f"Resolve customer-site live smoke blockers: {', '.join(blockers)}."]
    return ["Customer-site real device smoke is passed and ready for archive."]


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    report = build_customer_site_live_smoke_report(
        payload=_load_json(args.input),
        input_path=args.input,
        require_evidence_files=args.require_evidence_files,
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
