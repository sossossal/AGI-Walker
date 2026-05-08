from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence

DEFAULT_FAULT_TABLE = "deployment/hardware/imc22_reflex_fault_table.json"
DEFAULT_RECOVERY_POLICY = "deployment/hardware/imc22_reflex_recovery_policy.json"
DEFAULT_TELEMETRY_FIELDS = "deployment/hardware/imc22_fault_telemetry_fields.json"
DEFAULT_OUTPUT = "test_env/hardware_live/vendor_data_promotion_checklist.json"
SCHEMA_VERSION = "1.0"


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a checklist for promoting reviewed vendor hardware data."
    )
    parser.add_argument("--fault-table-file", default=DEFAULT_FAULT_TABLE)
    parser.add_argument("--recovery-policy-file", default=DEFAULT_RECOVERY_POLICY)
    parser.add_argument("--telemetry-fields-file", default=DEFAULT_TELEMETRY_FIELDS)
    parser.add_argument("--sample-archive-file", required=True)
    parser.add_argument("--vendor-review-file", required=True)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _load_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _text(value: Any) -> str:
    return value.strip() if isinstance(value, str) else ""


def _is_placeholder(value: Any) -> bool:
    text = _text(value)
    return not text or "<" in text or ">" in text or "YYYY" in text or "Replace with" in text


def _change_log_versions(payload: dict[str, Any]) -> set[str]:
    change_log = payload.get("change_log")
    if not isinstance(change_log, list):
        return set()
    return {
        entry["version"]
        for entry in change_log
        if isinstance(entry, dict) and _text(entry.get("version"))
    }


def _build_step(step_id: str, ready: bool, detail: str) -> dict[str, Any]:
    return {
        "id": step_id,
        "status": "ready" if ready else "blocked",
        "detail": detail,
    }


def build_vendor_data_promotion_checklist(
    *,
    fault_table: dict[str, Any],
    recovery_policy: dict[str, Any],
    telemetry_fields: dict[str, Any],
    sample_archive: dict[str, Any],
    vendor_review: dict[str, Any],
    sources: dict[str, str],
) -> dict[str, Any]:
    fault_table_version = _text(fault_table.get("data_version"))
    recovery_policy_version = _text(recovery_policy.get("data_version"))
    telemetry_fields_version = _text(telemetry_fields.get("data_version"))
    sample_archive_version = _text(sample_archive.get("data_version"))
    change_request = _text(sample_archive.get("change_request"))
    sample_count = len(sample_archive.get("samples") or [])

    steps = [
        _build_step(
            "schema_versions",
            all(
                payload.get("schema_version") == SCHEMA_VERSION
                for payload in (
                    fault_table,
                    recovery_policy,
                    telemetry_fields,
                    sample_archive,
                    vendor_review,
                )
            ),
            "All vendor promotion inputs must use schema_version 1.0.",
        ),
        _build_step(
            "data_versions",
            all(
                (
                    fault_table_version,
                    recovery_policy_version,
                    telemetry_fields_version,
                    sample_archive_version,
                )
            ),
            "Fault table, recovery policy, telemetry fields and sample archive must carry data_version.",
        ),
        _build_step(
            "change_logs",
            fault_table_version in _change_log_versions(fault_table)
            and recovery_policy_version in _change_log_versions(recovery_policy)
            and telemetry_fields_version in _change_log_versions(telemetry_fields)
            and sample_archive_version in _change_log_versions(sample_archive),
            "Each data_version must have a matching change_log entry.",
        ),
        _build_step(
            "change_request",
            not _is_placeholder(change_request),
            "Sample archive must bind the promotion to a real field change_request.",
        ),
        _build_step(
            "sample_archive",
            sample_count > 0,
            "Sample archive must contain at least one reviewed live raw error sample.",
        ),
        _build_step(
            "vendor_review",
            vendor_review.get("status") == "passed"
            and vendor_review.get("summary", {}).get("sample_archive_present") is True,
            "Vendor fault data review must pass with a bound sample_archive_file.",
        ),
    ]
    status = "ready" if all(step["status"] == "ready" for step in steps) else "blocked"
    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "sources": sources,
        "summary": {
            "change_request": change_request,
            "sample_count": sample_count,
            "fault_table_data_version": fault_table_version,
            "recovery_policy_data_version": recovery_policy_version,
            "telemetry_fields_data_version": telemetry_fields_version,
            "sample_archive_data_version": sample_archive_version,
            "ready_step_count": sum(1 for step in steps if step["status"] == "ready"),
            "blocked_step_count": sum(
                1 for step in steps if step["status"] == "blocked"
            ),
        },
        "steps": steps,
        "next_actions": _next_actions(status=status, steps=steps),
    }


def _next_actions(*, status: str, steps: list[dict[str, Any]]) -> list[str]:
    if status == "ready":
        return [
            "Promote reviewed vendor data and archive this checklist with live evidence."
        ]
    blocked_ids = [step["id"] for step in steps if step["status"] == "blocked"]
    return [f"Resolve blocked vendor promotion steps: {', '.join(blocked_ids)}."]


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    sources = {
        "fault_table_file": args.fault_table_file,
        "recovery_policy_file": args.recovery_policy_file,
        "telemetry_fields_file": args.telemetry_fields_file,
        "sample_archive_file": args.sample_archive_file,
        "vendor_review_file": args.vendor_review_file,
    }
    checklist = build_vendor_data_promotion_checklist(
        fault_table=_load_json(args.fault_table_file),
        recovery_policy=_load_json(args.recovery_policy_file),
        telemetry_fields=_load_json(args.telemetry_fields_file),
        sample_archive=_load_json(args.sample_archive_file),
        vendor_review=_load_json(args.vendor_review_file),
        sources=sources,
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(checklist, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0 if checklist["status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
