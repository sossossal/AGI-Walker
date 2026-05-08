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


def _text(value: Any) -> str:
    return value.strip() if isinstance(value, str) else ""


def _is_placeholder(value: Any) -> bool:
    text = _text(value)
    return not text or "<" in text or "YYYY" in text or "Replace with" in text


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
) -> dict[str, Any]:
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
    for index, sample in enumerate(samples):
        sample_blockers: list[str] = []
        if not isinstance(sample, dict):
            invalid_samples.append({"index": index, "blockers": ["sample_type"]})
            continue
        for field in ("node_id", "raw_error_value"):
            if sample.get(field) is None:
                sample_blockers.append(field)
        for field in ("fault_class", "source_evidence", "captured_at", "captured_by"):
            if _is_placeholder(sample.get(field)):
                sample_blockers.append(field)
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
        },
        "blockers": blockers,
        "invalid_samples": invalid_samples,
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
    report = build_vendor_fault_sample_closeout(
        sample_archive=_load_json(args.sample_archive_file),
        fault_table=_load_json(args.fault_table_file),
        recovery_policy=_load_json(args.recovery_policy_file),
        sources=sources,
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
