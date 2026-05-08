from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Any, Sequence

DEFAULT_FAULT_TABLE = "deployment/hardware/imc22_reflex_fault_table.json"
DEFAULT_RECOVERY_POLICY = "deployment/hardware/imc22_reflex_recovery_policy.json"
DEFAULT_TELEMETRY_FIELDS = "deployment/hardware/imc22_fault_telemetry_fields.json"
DEFAULT_OUTPUT = "test_env/hardware_live/vendor_fault_data_review.json"
SCHEMA_VERSION = "1.0"
FAULT_CLASSES = {
    "ok",
    "overload",
    "overcurrent",
    "sensor_fault",
    "communication_fault",
    "unknown_fault",
}
RECOVERY_ACTIONS = {
    "recover_hold_position",
    "recover_relaxed_hold",
    "clear_only",
    "rediscover_node",
}


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a vendor fault data review from live fault telemetry."
    )
    parser.add_argument("--telemetry-report", required=True)
    parser.add_argument("--fault-table-file", default=DEFAULT_FAULT_TABLE)
    parser.add_argument("--recovery-policy-file", default=DEFAULT_RECOVERY_POLICY)
    parser.add_argument("--telemetry-fields-file", default=DEFAULT_TELEMETRY_FIELDS)
    parser.add_argument("--sample-archive-file", default=None)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _load_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _load_json_if_exists(path: str | Path) -> dict[str, Any] | None:
    json_path = Path(path)
    if not json_path.exists():
        return None
    return _load_json(json_path)


def validate_fault_table(fault_table: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if fault_table.get("schema_version") != SCHEMA_VERSION:
        errors.append(f"fault_table.schema_version must be {SCHEMA_VERSION!r}")
    if not isinstance(fault_table.get("vendor"), str) or not fault_table["vendor"]:
        errors.append("fault_table.vendor must be a non-empty string")
    exact_codes = fault_table.get("exact_codes")
    if not isinstance(exact_codes, dict):
        errors.append("fault_table.exact_codes must be a dict")
    else:
        for code, fault_class in exact_codes.items():
            try:
                int(code)
            except ValueError:
                errors.append("fault_table.exact_codes keys must be ints")
            if fault_class not in FAULT_CLASSES:
                errors.append("fault_table.exact_codes values must be valid fault classes")
    ranges = fault_table.get("ranges")
    if not isinstance(ranges, list):
        errors.append("fault_table.ranges must be a list")
    else:
        for index, entry in enumerate(ranges):
            if not isinstance(entry, dict):
                errors.append(f"fault_table.ranges[{index}] must be a dict")
                continue
            if entry.get("fault_class") not in FAULT_CLASSES:
                errors.append(
                    f"fault_table.ranges[{index}].fault_class must be a valid fault class"
                )
            lower = entry.get("min")
            upper = entry.get("max")
            if lower is not None and not isinstance(lower, (int, float)):
                errors.append(f"fault_table.ranges[{index}].min must be numeric or null")
            if upper is not None and not isinstance(upper, (int, float)):
                errors.append(f"fault_table.ranges[{index}].max must be numeric or null")
            if isinstance(lower, (int, float)) and isinstance(upper, (int, float)):
                if float(lower) > float(upper):
                    errors.append(f"fault_table.ranges[{index}].min must be <= max")
    if fault_table.get("fallback_fault_class") not in FAULT_CLASSES:
        errors.append("fault_table.fallback_fault_class must be a valid fault class")
    return errors


def validate_recovery_policy(recovery_policy: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if recovery_policy.get("schema_version") != SCHEMA_VERSION:
        errors.append(f"recovery_policy.schema_version must be {SCHEMA_VERSION!r}")
    if not isinstance(recovery_policy.get("vendor"), str) or not recovery_policy["vendor"]:
        errors.append("recovery_policy.vendor must be a non-empty string")
    if not isinstance(recovery_policy.get("watchdog_action"), dict):
        errors.append("recovery_policy.watchdog_action must be a dict")
    fault_actions = recovery_policy.get("fault_actions")
    if not isinstance(fault_actions, dict):
        errors.append("recovery_policy.fault_actions must be a dict")
    else:
        for fault_class, action_spec in fault_actions.items():
            if fault_class not in FAULT_CLASSES:
                errors.append(
                    f"recovery_policy.fault_actions key {fault_class!r} must be a valid fault class"
                )
            if not isinstance(action_spec, dict):
                errors.append(
                    f"recovery_policy.fault_actions[{fault_class!r}] must be a dict"
                )
                continue
            if action_spec.get("action") not in RECOVERY_ACTIONS:
                errors.append(
                    f"recovery_policy.fault_actions[{fault_class!r}].action is invalid"
                )
    return errors


def validate_telemetry_fields(telemetry_fields: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if telemetry_fields.get("schema_version") != SCHEMA_VERSION:
        errors.append(f"telemetry_fields.schema_version must be {SCHEMA_VERSION!r}")
    if not isinstance(telemetry_fields.get("vendor"), str) or not telemetry_fields["vendor"]:
        errors.append("telemetry_fields.vendor must be a non-empty string")
    fields = telemetry_fields.get("fields")
    if not isinstance(fields, dict):
        errors.append("telemetry_fields.fields must be a dict")
        return errors
    for field_name, field_spec in fields.items():
        if not isinstance(field_spec, dict):
            errors.append(f"telemetry_fields.fields[{field_name!r}] must be a dict")
            continue
        if field_spec.get("required") is not None and not isinstance(
            field_spec["required"], bool
        ):
            errors.append(
                f"telemetry_fields.fields[{field_name!r}].required must be a bool"
            )
        if not isinstance(field_spec.get("source"), str) or not field_spec["source"]:
            errors.append(
                f"telemetry_fields.fields[{field_name!r}].source must be a non-empty string"
            )
    return errors


def _required_telemetry_fields(telemetry_fields: dict[str, Any]) -> list[str]:
    fields = telemetry_fields.get("fields")
    if not isinstance(fields, dict):
        return []
    return sorted(
        field_name
        for field_name, field_spec in fields.items()
        if isinstance(field_spec, dict) and field_spec.get("required") is True
    )


def validate_sample_archive(sample_archive: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if sample_archive.get("schema_version") != SCHEMA_VERSION:
        errors.append(f"sample_archive.schema_version must be {SCHEMA_VERSION!r}")
    if not isinstance(sample_archive.get("vendor"), str) or not sample_archive["vendor"]:
        errors.append("sample_archive.vendor must be a non-empty string")
    if not isinstance(sample_archive.get("change_request"), str) or not sample_archive[
        "change_request"
    ]:
        errors.append("sample_archive.change_request must be a non-empty string")
    samples = sample_archive.get("samples")
    if not isinstance(samples, list):
        errors.append("sample_archive.samples must be a list")
        return errors
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            errors.append(f"sample_archive.samples[{index}] must be a dict")
            continue
        for field_name in (
            "node_id",
            "raw_error_value",
            "fault_class",
            "source_evidence",
            "captured_at",
            "captured_by",
        ):
            if sample.get(field_name) in (None, ""):
                errors.append(
                    f"sample_archive.samples[{index}].{field_name} is required"
                )
        if sample.get("fault_class") not in FAULT_CLASSES:
            errors.append(
                f"sample_archive.samples[{index}].fault_class must be a valid fault class"
            )
        try:
            float(sample.get("raw_error_value"))
        except (TypeError, ValueError):
            errors.append(
                f"sample_archive.samples[{index}].raw_error_value must be numeric"
            )
    return errors


def classify_fault(raw_error_value: float, *, fault_table: dict[str, Any]) -> str:
    exact_codes = fault_table.get("exact_codes") or {}
    exact_key = str(int(raw_error_value))
    if exact_key in exact_codes:
        return str(exact_codes[exact_key])
    for entry in fault_table.get("ranges") or []:
        lower = entry.get("min")
        upper = entry.get("max")
        if lower is not None and raw_error_value < float(lower):
            continue
        if upper is not None and raw_error_value > float(upper):
            continue
        return str(entry["fault_class"])
    return str(fault_table.get("fallback_fault_class", "unknown_fault"))


def _entry_raw_error(entry: dict[str, Any]) -> float:
    raw_value = entry.get("raw_error_value", entry.get("raw_error"))
    return float(raw_value)


def build_vendor_fault_data_review(
    *,
    telemetry_report: dict[str, Any],
    fault_table: dict[str, Any],
    recovery_policy: dict[str, Any],
    telemetry_fields: dict[str, Any],
    sample_archive: dict[str, Any] | None,
    telemetry_report_path: str,
    fault_table_path: str,
    recovery_policy_path: str,
    telemetry_fields_path: str,
    sample_archive_path: str | None,
    telemetry_report_present: bool = True,
) -> dict[str, Any]:
    table_errors = validate_fault_table(fault_table)
    policy_errors = validate_recovery_policy(recovery_policy)
    telemetry_field_errors = validate_telemetry_fields(telemetry_fields)
    sample_archive_errors = (
        validate_sample_archive(sample_archive) if sample_archive is not None else []
    )
    entries = telemetry_report.get("entries") or []
    required_fields = _required_telemetry_fields(telemetry_fields)
    entry_reviews: list[dict[str, Any]] = []
    mismatch_count = 0
    missing_recovery_classes: set[str] = set()
    missing_required_field_count = 0
    fault_class_counts: Counter[str] = Counter()
    raw_error_counts: Counter[str] = Counter()

    for index, entry in enumerate(entries):
        raw_error = _entry_raw_error(entry)
        missing_required_fields = [
            field_name for field_name in required_fields if field_name not in entry
        ]
        observed_fault_class = str(entry.get("fault_class", "unknown_fault"))
        expected_fault_class = classify_fault(raw_error, fault_table=fault_table)
        has_recovery_action = expected_fault_class in recovery_policy.get(
            "fault_actions", {}
        )
        if observed_fault_class != expected_fault_class:
            mismatch_count += 1
        if not has_recovery_action:
            missing_recovery_classes.add(expected_fault_class)
        if missing_required_fields:
            missing_required_field_count += 1
        fault_class_counts[expected_fault_class] += 1
        raw_error_counts[str(raw_error)] += 1
        entry_reviews.append(
            {
                "index": index,
                "node_id": entry.get("node_id"),
                "raw_error_value": raw_error,
                "observed_fault_class": observed_fault_class,
                "expected_fault_class": expected_fault_class,
                "matches_fault_table": observed_fault_class == expected_fault_class,
                "has_recovery_action": has_recovery_action,
                "missing_required_fields": missing_required_fields,
            }
        )

    sample_reviews: list[dict[str, Any]] = []
    sample_mismatch_count = 0
    if sample_archive is not None and not sample_archive_errors:
        for index, sample in enumerate(sample_archive.get("samples") or []):
            raw_error = _entry_raw_error(sample)
            observed_fault_class = str(sample.get("fault_class", "unknown_fault"))
            expected_fault_class = classify_fault(raw_error, fault_table=fault_table)
            if observed_fault_class != expected_fault_class:
                sample_mismatch_count += 1
            sample_reviews.append(
                {
                    "index": index,
                    "node_id": sample.get("node_id"),
                    "raw_error_value": raw_error,
                    "observed_fault_class": observed_fault_class,
                    "expected_fault_class": expected_fault_class,
                    "matches_fault_table": observed_fault_class == expected_fault_class,
                    "source_evidence": sample.get("source_evidence"),
                    "change_request": sample_archive.get("change_request"),
                }
            )

    status = "passed"
    blockers: list[str] = []
    if table_errors:
        blockers.append("fault_table_invalid")
    if policy_errors:
        blockers.append("recovery_policy_invalid")
    if telemetry_field_errors:
        blockers.append("telemetry_fields_invalid")
    if sample_archive_errors:
        blockers.append("sample_archive_invalid")
    if not telemetry_report_present:
        blockers.append("telemetry_report_missing")
    if not entries:
        blockers.append("telemetry_entries_missing")
    if mismatch_count:
        blockers.append("telemetry_fault_class_mismatch")
    if missing_required_field_count:
        blockers.append("telemetry_required_fields_missing")
    if sample_mismatch_count:
        blockers.append("sample_archive_fault_class_mismatch")
    if missing_recovery_classes:
        blockers.append("recovery_policy_missing_fault_class")
    if blockers:
        status = "blocked"

    return {
        "schema_version": "1.0",
        "status": status,
        "blockers": blockers,
        "sources": {
            "telemetry_report": telemetry_report_path,
            "fault_table_file": fault_table_path,
            "recovery_policy_file": recovery_policy_path,
            "telemetry_fields_file": telemetry_fields_path,
            "sample_archive_file": sample_archive_path,
        },
        "summary": {
            "entry_count": len(entries),
            "telemetry_report_present": telemetry_report_present,
            "mismatch_count": mismatch_count,
            "missing_required_field_count": missing_required_field_count,
            "fault_class_counts": dict(sorted(fault_class_counts.items())),
            "raw_error_counts": dict(sorted(raw_error_counts.items())),
            "missing_recovery_classes": sorted(missing_recovery_classes),
            "fault_table_error_count": len(table_errors),
            "recovery_policy_error_count": len(policy_errors),
            "telemetry_field_error_count": len(telemetry_field_errors),
            "required_telemetry_fields": required_fields,
            "sample_archive_present": sample_archive is not None,
            "sample_archive_error_count": len(sample_archive_errors),
            "sample_archive_mismatch_count": sample_mismatch_count,
        },
        "validation_errors": {
            "fault_table": table_errors,
            "recovery_policy": policy_errors,
            "telemetry_fields": telemetry_field_errors,
            "sample_archive": sample_archive_errors,
        },
        "entries": entry_reviews,
        "sample_archive_entries": sample_reviews,
        "next_actions": _next_actions(
            has_entries=bool(entries),
            mismatch_count=mismatch_count,
            missing_required_field_count=missing_required_field_count,
            missing_recovery_classes=missing_recovery_classes,
            table_errors=table_errors,
            policy_errors=policy_errors,
            telemetry_field_errors=telemetry_field_errors,
            sample_archive_errors=sample_archive_errors,
            sample_mismatch_count=sample_mismatch_count,
            has_sample_archive=sample_archive is not None,
        ),
    }


def _next_actions(
    *,
    has_entries: bool,
    mismatch_count: int,
    missing_required_field_count: int,
    missing_recovery_classes: set[str],
    table_errors: list[str],
    policy_errors: list[str],
    telemetry_field_errors: list[str],
    sample_archive_errors: list[str],
    sample_mismatch_count: int,
    has_sample_archive: bool,
) -> list[str]:
    actions: list[str] = []
    if not has_entries:
        actions.append("Run live hardware diagnostics and export fault telemetry.")
    if table_errors:
        actions.append("Fix fault table schema errors before accepting vendor data.")
    if policy_errors:
        actions.append("Fix recovery policy schema errors before accepting vendor data.")
    if telemetry_field_errors:
        actions.append("Fix telemetry field mapping schema errors before accepting vendor data.")
    if sample_archive_errors:
        actions.append("Fix sample archive schema errors before accepting vendor data.")
    if mismatch_count:
        actions.append("Update fault table or telemetry classification for mismatched raw error samples.")
    if missing_required_field_count:
        actions.append("Export all required telemetry fields before archiving vendor data.")
    if sample_mismatch_count:
        actions.append("Update sample archive fault classes or fault table mappings.")
    if missing_recovery_classes:
        actions.append("Add recovery policy actions for missing fault classes.")
    if not has_sample_archive:
        actions.append("Attach a reviewed sample archive before promoting new vendor data.")
    if not actions:
        actions.append("Archive this review with live evidence and use it as vendor data baseline.")
    return actions


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    fault_table = _load_json(args.fault_table_file)
    recovery_policy = _load_json(args.recovery_policy_file)
    telemetry_fields = _load_json(args.telemetry_fields_file)
    telemetry_report = _load_json_if_exists(args.telemetry_report)
    sample_archive = (
        _load_json(args.sample_archive_file) if args.sample_archive_file else None
    )
    review = build_vendor_fault_data_review(
        telemetry_report=telemetry_report or {"schema_version": SCHEMA_VERSION, "entries": []},
        fault_table=fault_table,
        recovery_policy=recovery_policy,
        telemetry_fields=telemetry_fields,
        sample_archive=sample_archive,
        telemetry_report_path=args.telemetry_report,
        telemetry_report_present=telemetry_report is not None,
        fault_table_path=args.fault_table_file,
        recovery_policy_path=args.recovery_policy_file,
        telemetry_fields_path=args.telemetry_fields_file,
        sample_archive_path=args.sample_archive_file,
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(review, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0 if review["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
