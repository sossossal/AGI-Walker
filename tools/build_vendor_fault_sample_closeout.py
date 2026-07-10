from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence


DEFAULT_SAMPLE_ARCHIVE = "deployment/hardware/imc22_vendor_fault_samples.template.json"
DEFAULT_FAULT_TABLE = "deployment/hardware/imc22_reflex_fault_table.json"
DEFAULT_RECOVERY_POLICY = "deployment/hardware/imc22_reflex_recovery_policy.json"
DEFAULT_OUTPUT = "test_env/hardware_live/vendor_fault_sample_closeout.json"
SCHEMA_VERSION = "1.0"
PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_FILE_FIELDS = (
    "sample_archive_file",
    "fault_table_file",
    "recovery_policy_file",
)


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a closeout report for live vendor raw fault samples."
    )
    parser.add_argument("--sample-archive-file", default=DEFAULT_SAMPLE_ARCHIVE)
    parser.add_argument("--fault-table-file", default=DEFAULT_FAULT_TABLE)
    parser.add_argument("--recovery-policy-file", default=DEFAULT_RECOVERY_POLICY)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _load_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _load_json_if_valid(path: Path | None) -> dict[str, Any]:
    if path is None or not path.exists():
        return {}
    return _load_json(path)


def _text(value: Any) -> str:
    return value.strip() if isinstance(value, str) else ""


def _is_placeholder(value: Any) -> bool:
    text = _text(value)
    return not text or "<" in text or "YYYY" in text or "Replace with" in text


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
    statuses: dict[str, dict[str, Any]] = {}
    output_dir = Path(output_path).resolve().parent
    for field in SOURCE_FILE_FIELDS:
        value = sources.get(field)
        valid, resolved_path, error = _resolve_relative_path(
            value,
            base_dir=output_dir,
        )
        statuses[field] = {
            "path": _text(value),
            "path_valid": valid,
            "path_error": error,
            "resolved_path": str(resolved_path) if resolved_path is not None else None,
            "exists": bool(resolved_path and resolved_path.exists()),
        }
    return statuses


def _sample_source_evidence_status(
    value: Any,
    *,
    sample_archive_path: Path | None,
) -> dict[str, Any]:
    base_dir = (
        sample_archive_path.resolve().parent
        if sample_archive_path is not None
        else PROJECT_ROOT
    )
    valid, resolved_path, error = _resolve_relative_path(
        _text(value),
        base_dir=base_dir,
    )
    return {
        "path": _text(value),
        "path_valid": valid,
        "path_error": error,
        "resolved_path": str(resolved_path) if resolved_path is not None else None,
        "exists": bool(resolved_path and resolved_path.exists()),
    }


def _change_log_versions(payload: dict[str, Any]) -> set[str]:
    change_log = payload.get("change_log")
    if not isinstance(change_log, list):
        return set()
    return {
        entry["version"]
        for entry in change_log
        if isinstance(entry, dict) and _text(entry.get("version"))
    }


def _fault_table_classes(fault_table: dict[str, Any]) -> set[str]:
    classes = set()
    exact_codes = fault_table.get("exact_codes")
    if isinstance(exact_codes, dict):
        classes.update(str(value) for value in exact_codes.values())
    for entry in fault_table.get("ranges") or []:
        if isinstance(entry, dict) and _text(entry.get("fault_class")):
            classes.add(str(entry["fault_class"]))
    return classes


def _recovery_policy_classes(recovery_policy: dict[str, Any]) -> set[str]:
    fault_actions = recovery_policy.get("fault_actions")
    return set(fault_actions) if isinstance(fault_actions, dict) else set()


def build_vendor_fault_sample_closeout(
    *,
    sample_archive: dict[str, Any],
    fault_table: dict[str, Any],
    recovery_policy: dict[str, Any],
    sources: dict[str, str],
    source_file_statuses: dict[str, dict[str, Any]] | None = None,
) -> dict[str, Any]:
    if source_file_statuses is None:
        source_file_statuses = {
            field: {"path_valid": True, "resolved_path": None}
            for field in SOURCE_FILE_FIELDS
        }
    samples = sample_archive.get("samples") if isinstance(sample_archive.get("samples"), list) else []
    sample_fault_classes = {
        str(sample.get("fault_class"))
        for sample in samples
        if isinstance(sample, dict) and _text(sample.get("fault_class"))
    }
    table_classes = _fault_table_classes(fault_table)
    policy_classes = _recovery_policy_classes(recovery_policy)
    sample_version = _text(sample_archive.get("data_version"))
    fault_table_version = _text(fault_table.get("data_version"))
    recovery_policy_version = _text(recovery_policy.get("data_version"))

    blockers: list[str] = []
    invalid_source_files = [
        field
        for field, status_item in source_file_statuses.items()
        if not status_item["path_valid"]
    ]
    blockers.extend(invalid_source_files)
    if sample_archive.get("schema_version") != SCHEMA_VERSION:
        blockers.append("sample_archive_schema")
    if fault_table.get("schema_version") != SCHEMA_VERSION:
        blockers.append("fault_table_schema")
    if recovery_policy.get("schema_version") != SCHEMA_VERSION:
        blockers.append("recovery_policy_schema")
    if _is_placeholder(sample_archive.get("change_request")):
        blockers.append("change_request")
    if _is_placeholder(sample_archive.get("review_owner")):
        blockers.append("review_owner")
    if not sample_version or sample_version not in _change_log_versions(sample_archive):
        blockers.append("sample_archive_change_log")
    if not fault_table_version or fault_table_version not in _change_log_versions(fault_table):
        blockers.append("fault_table_change_log")
    if not recovery_policy_version or recovery_policy_version not in _change_log_versions(recovery_policy):
        blockers.append("recovery_policy_change_log")
    if not samples:
        blockers.append("samples")

    invalid_samples: list[dict[str, Any]] = []
    sample_source_statuses: list[dict[str, Any]] = []
    sample_archive_path_text = source_file_statuses["sample_archive_file"].get(
        "resolved_path"
    )
    sample_archive_path = (
        Path(sample_archive_path_text) if isinstance(sample_archive_path_text, str) else None
    )
    for index, sample in enumerate(samples):
        sample_blockers: list[str] = []
        if not isinstance(sample, dict):
            invalid_samples.append({"index": index, "blockers": ["sample_type"]})
            continue
        source_status = _sample_source_evidence_status(
            sample.get("source_evidence"),
            sample_archive_path=sample_archive_path,
        )
        sample_source_statuses.append({"index": index, **source_status})
        for field in ("node_id", "raw_error_value"):
            if sample.get(field) is None:
                sample_blockers.append(field)
        for field in ("fault_class", "source_evidence", "captured_at", "captured_by"):
            if _is_placeholder(sample.get(field)):
                sample_blockers.append(field)
        if (
            not _is_placeholder(sample.get("source_evidence"))
            and not source_status["path_valid"]
        ):
            sample_blockers.append("source_evidence_path")
        fault_class = _text(sample.get("fault_class"))
        if fault_class and fault_class not in table_classes:
            sample_blockers.append("fault_class_not_in_fault_table")
        if fault_class and fault_class not in policy_classes:
            sample_blockers.append("fault_class_not_in_recovery_policy")
        try:
            float(sample.get("raw_error_value"))
        except (TypeError, ValueError):
            sample_blockers.append("raw_error_value_numeric")
        if sample_blockers:
            invalid_samples.append({"index": index, "blockers": sample_blockers})
    if invalid_samples:
        blockers.append("sample_records")

    status = "ready" if not blockers else "blocked"
    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "sources": sources,
        "summary": {
            "sample_count": len(samples),
            "sample_fault_classes": sorted(sample_fault_classes),
            "fault_table_data_version": fault_table_version,
            "recovery_policy_data_version": recovery_policy_version,
            "sample_archive_data_version": sample_version,
            "invalid_sample_count": len(invalid_samples),
            "source_file_path_validation_error_count": len(invalid_source_files),
            "sample_source_evidence_path_validation_error_count": sum(
                1 for item in sample_source_statuses if not item["path_valid"]
            ),
        },
        "blockers": blockers,
        "invalid_samples": invalid_samples,
        "source_file_statuses": source_file_statuses,
        "sample_source_evidence_statuses": sample_source_statuses,
        "next_actions": _next_actions(status=status, blockers=blockers),
    }


def _next_actions(*, status: str, blockers: list[str]) -> list[str]:
    if status == "blocked":
        return [f"Resolve vendor sample closeout blockers: {', '.join(blockers)}."]
    return ["Vendor raw fault sample dataset is ready for review and promotion."]


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    sources = {
        "sample_archive_file": args.sample_archive_file,
        "fault_table_file": args.fault_table_file,
        "recovery_policy_file": args.recovery_policy_file,
    }
    source_file_statuses = _source_file_statuses(sources, output_path=args.output)
    report = build_vendor_fault_sample_closeout(
        sample_archive=_load_json_if_valid(
            Path(source_file_statuses["sample_archive_file"]["resolved_path"])
            if source_file_statuses["sample_archive_file"]["resolved_path"]
            else None
        ),
        fault_table=_load_json_if_valid(
            Path(source_file_statuses["fault_table_file"]["resolved_path"])
            if source_file_statuses["fault_table_file"]["resolved_path"]
            else None
        ),
        recovery_policy=_load_json_if_valid(
            Path(source_file_statuses["recovery_policy_file"]["resolved_path"])
            if source_file_statuses["recovery_policy_file"]["resolved_path"]
            else None
        ),
        sources=sources,
        source_file_statuses=source_file_statuses,
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
