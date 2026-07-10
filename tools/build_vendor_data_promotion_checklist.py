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
PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_FILE_FIELDS = (
    "fault_table_file",
    "recovery_policy_file",
    "telemetry_fields_file",
    "sample_archive_file",
    "vendor_review_file",
)


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


def _load_json_if_exists(path: str | Path | None) -> dict[str, Any]:
    if path is None:
        return {}
    json_path = Path(path)
    if not json_path.exists():
        return {}
    return _load_json(json_path)


def _text(value: Any) -> str:
    return value.strip() if isinstance(value, str) else ""


def _is_placeholder(value: Any) -> bool:
    text = _text(value)
    return not text or "<" in text or ">" in text or "YYYY" in text or "Replace with" in text


def _resolve_relative_path(
    value: str | None,
    *,
    base_dir: Path,
) -> tuple[bool, Path | None, str | None]:
    text = _text(value)
    if not text:
        return False, None, "empty"
    path = Path(text)
    if path.is_absolute():
        return False, None, "absolute"
    if ".." in path.parts:
        return False, None, "parent_directory"
    base_relative = base_dir / path
    if base_relative.exists():
        return True, base_relative, None
    return True, PROJECT_ROOT / path, None


def _source_file_statuses(
    sources: dict[str, str],
    *,
    output_path: str,
) -> dict[str, dict[str, Any]]:
    output_dir = Path(output_path).resolve().parent
    statuses: dict[str, dict[str, Any]] = {}
    for field in SOURCE_FILE_FIELDS:
        valid, resolved_path, error = _resolve_relative_path(
            sources.get(field),
            base_dir=output_dir,
        )
        statuses[field] = {
            "path": _text(sources.get(field)),
            "path_valid": valid,
            "path_error": error,
            "resolved_path": str(resolved_path) if resolved_path is not None else None,
            "exists": bool(resolved_path and resolved_path.exists()),
        }
    return statuses


def _sample_source_evidence_statuses(
    sample_archive: dict[str, Any],
    *,
    sample_archive_path: Path | None,
) -> list[dict[str, Any]]:
    base_dir = sample_archive_path.resolve().parent if sample_archive_path else PROJECT_ROOT
    statuses: list[dict[str, Any]] = []
    for index, sample in enumerate(sample_archive.get("samples") or []):
        value = sample.get("source_evidence") if isinstance(sample, dict) else None
        valid, resolved_path, error = _resolve_relative_path(
            _text(value),
            base_dir=base_dir,
        )
        statuses.append(
            {
                "index": index,
                "path": _text(value),
                "path_valid": valid,
                "path_error": error,
                "resolved_path": str(resolved_path) if resolved_path is not None else None,
                "exists": bool(resolved_path and resolved_path.exists()),
            }
        )
    return statuses


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
    source_file_statuses: dict[str, dict[str, Any]] | None = None,
    sample_source_evidence_statuses: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    fault_table_version = _text(fault_table.get("data_version"))
    recovery_policy_version = _text(recovery_policy.get("data_version"))
    telemetry_fields_version = _text(telemetry_fields.get("data_version"))
    sample_archive_version = _text(sample_archive.get("data_version"))
    change_request = _text(sample_archive.get("change_request"))
    sample_count = len(sample_archive.get("samples") or [])
    path_statuses = source_file_statuses or {}
    source_path_error_count = sum(
        1 for status in path_statuses.values() if not status.get("path_valid")
    )
    sample_path_statuses = sample_source_evidence_statuses or []
    sample_source_path_error_count = sum(
        1 for status in sample_path_statuses if not status.get("path_valid")
    )

    steps = [
        _build_step(
            "source_paths",
            source_path_error_count == 0,
            "Vendor promotion input paths must be relative and cannot use parent-directory traversal.",
        ),
        _build_step(
            "sample_source_evidence_paths",
            sample_source_path_error_count == 0,
            "Sample archive source_evidence paths must be relative and cannot use parent-directory traversal.",
        ),
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
            "source_file_path_validation_error_count": source_path_error_count,
            "sample_source_evidence_path_validation_error_count": sample_source_path_error_count,
        },
        "source_file_statuses": path_statuses,
        "sample_source_evidence_statuses": sample_path_statuses,
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
    source_file_statuses = _source_file_statuses(sources, output_path=args.output)
    sample_archive = _load_json_if_exists(
        source_file_statuses["sample_archive_file"]["resolved_path"]
    )
    sample_source_evidence_statuses = _sample_source_evidence_statuses(
        sample_archive,
        sample_archive_path=(
            Path(source_file_statuses["sample_archive_file"]["resolved_path"])
            if source_file_statuses["sample_archive_file"]["resolved_path"]
            else None
        ),
    )
    checklist = build_vendor_data_promotion_checklist(
        fault_table=_load_json_if_exists(
            source_file_statuses["fault_table_file"]["resolved_path"]
        ),
        recovery_policy=_load_json_if_exists(
            source_file_statuses["recovery_policy_file"]["resolved_path"]
        ),
        telemetry_fields=_load_json_if_exists(
            source_file_statuses["telemetry_fields_file"]["resolved_path"]
        ),
        sample_archive=sample_archive,
        vendor_review=_load_json_if_exists(
            source_file_statuses["vendor_review_file"]["resolved_path"]
        ),
        sources=sources,
        source_file_statuses=source_file_statuses,
        sample_source_evidence_statuses=sample_source_evidence_statuses,
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
