"""Validate a delivery_acceptance_gate JSON artifact."""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any, Mapping


def _load_workflow_contracts_module() -> Any:
    repo_root = Path(__file__).resolve().parents[1]
    module_path = repo_root / "agi_walker" / "core" / "api" / "workflow_contracts.py"
    spec = importlib.util.spec_from_file_location("workflow_contracts", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load workflow contracts from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_robot_schema_module() -> Any:
    repo_root = Path(__file__).resolve().parents[1]
    module_path = repo_root / "agi_walker" / "core" / "api" / "robot_schema.py"
    spec = importlib.util.spec_from_file_location("robot_schema", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load robot schema from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _extract_gate_payload(payload: Any) -> tuple[Any, str]:
    if isinstance(payload, dict) and isinstance(
        payload.get("delivery_acceptance_gate"), dict
    ):
        return payload["delivery_acceptance_gate"], "delivery_acceptance_gate"
    return payload, "."


def _looks_like_gate_or_report_payload(
    payload: Any,
    *,
    workflow_contracts: Any,
) -> bool:
    if not isinstance(payload, dict):
        return False
    if isinstance(payload.get("delivery_acceptance_gate"), dict):
        return True
    return (
        payload.get("contract_version")
        == workflow_contracts.DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION
        or (
            "acceptance_requirements" in payload
            and "summary_counts" in payload
            and "reason_details" in payload
        )
    )


def _looks_like_validation_summary_payload(
    payload: Any,
    *,
    workflow_contracts: Any,
) -> bool:
    return (
        isinstance(payload, dict)
        and payload.get("summary_version")
        == workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
    )


def _looks_like_godot_node_tree_manifest_payload(payload: Any) -> bool:
    return (
        isinstance(payload, dict)
        and payload.get("manifest_version") == "godot_node_tree_manifest.v1"
    )


def _godot_node_tree_manifest_summary(payload: dict[str, Any]) -> dict[str, Any]:
    missing_endpoint_part_ids = payload.get("missing_endpoint_part_ids", [])
    missing_endpoint_connection_names = payload.get(
        "missing_endpoint_connection_names", []
    )
    part_node_paths = payload.get("part_node_paths", {})
    joint_node_paths = payload.get("joint_node_paths", {})
    part_node_path_count = (
        len(part_node_paths) if isinstance(part_node_paths, dict) else None
    )
    joint_node_path_count = (
        len(joint_node_paths) if isinstance(joint_node_paths, dict) else None
    )
    parts_count = payload.get("parts_count")
    joints_count = payload.get("joints_count")
    path_maps_complete = payload.get("path_maps_complete")
    if not isinstance(path_maps_complete, bool):
        path_maps_complete = (
            isinstance(parts_count, int)
            and isinstance(joints_count, int)
            and isinstance(part_node_path_count, int)
            and isinstance(joint_node_path_count, int)
            and part_node_path_count >= parts_count
            and joint_node_path_count >= joints_count
        )
    return {
        "manifest_version": payload.get("manifest_version"),
        "schema_version": payload.get("schema_version"),
        "robot_name": payload.get("robot_name"),
        "parts_count": parts_count,
        "joints_count": joints_count,
        "part_node_path_count": part_node_path_count,
        "joint_node_path_count": joint_node_path_count,
        "path_maps_complete": path_maps_complete,
        "parameterized_joints": payload.get("parameterized_joints"),
        "complete": payload.get("complete"),
        "endpoint_paths_complete": payload.get("endpoint_paths_complete"),
        "parameters_complete": payload.get("parameters_complete"),
        "missing_endpoint_parts_count": len(missing_endpoint_part_ids)
        if isinstance(missing_endpoint_part_ids, list)
        else None,
        "missing_endpoint_connections_count": len(missing_endpoint_connection_names)
        if isinstance(missing_endpoint_connection_names, list)
        else None,
    }


def _node_tree_manifest_summary_has_incomplete_paths(
    summary: dict[str, Any],
) -> bool:
    if isinstance(summary.get("path_maps_complete"), bool):
        return summary["path_maps_complete"] is False
    parts_count = summary.get("parts_count")
    part_path_count = summary.get("part_node_path_count")
    joints_count = summary.get("joints_count")
    joint_path_count = summary.get("joint_node_path_count")
    return (
        isinstance(parts_count, int)
        and isinstance(part_path_count, int)
        and part_path_count < parts_count
    ) or (
        isinstance(joints_count, int)
        and isinstance(joint_path_count, int)
        and joint_path_count < joints_count
    )


def _node_tree_manifest_path_map_mismatch_kind_counts(
    mismatches: Any,
) -> dict[str, int]:
    counts: dict[str, int] = {}
    if not isinstance(mismatches, list):
        return counts
    for mismatch in mismatches:
        if not isinstance(mismatch, Mapping):
            continue
        kind = mismatch.get("kind")
        if not isinstance(kind, str) or not kind:
            continue
        counts[kind] = counts.get(kind, 0) + 1
    return dict(sorted(counts.items()))


def _node_tree_manifest_sidecar_path_map_mismatch_kind_counts(
    sidecars: list[dict[str, Any]],
) -> dict[str, int]:
    counts: dict[str, int] = {}
    for sidecar in sidecars:
        kind_counts = sidecar.get("node_tree_manifest_path_map_mismatch_kind_counts")
        if not isinstance(kind_counts, Mapping):
            continue
        for kind, count in kind_counts.items():
            if isinstance(kind, str) and isinstance(count, int) and count >= 0:
                counts[kind] = counts.get(kind, 0) + count
    return dict(sorted(counts.items()))


def _compact_summary_counts(gate_payload: Any) -> dict[str, Any]:
    if not isinstance(gate_payload, dict) or not isinstance(
        gate_payload.get("summary_counts"), dict
    ):
        return {}
    counts = gate_payload["summary_counts"]
    compact = {
        str(key): value
        for key, value in sorted(counts.items())
        if isinstance(value, int) and not isinstance(value, bool)
    }
    for map_key in [
        "node_tree_gate_check_counts",
        "mechanical_gate_check_counts",
    ]:
        value = counts.get(map_key)
        if isinstance(value, dict):
            compact[map_key] = {
                str(key): count
                for key, count in sorted(value.items())
                if isinstance(count, int) and not isinstance(count, bool)
            }
    return compact


def _complete_required_summary_fields(
    gate_payload: Any,
    *,
    workflow_contracts: Any,
) -> list[str]:
    if not isinstance(gate_payload, dict):
        return []
    source = gate_payload.get("source")
    scope = gate_payload.get("verification_scope")
    if not isinstance(source, str) or not isinstance(scope, str):
        return []
    by_source_scope = workflow_contracts.DELIVERY_ACCEPTANCE_GATE_SCHEMA.get(
        "complete_required_summary_fields_by_source_scope",
        {},
    )
    if not isinstance(by_source_scope, dict):
        return []
    by_scope = by_source_scope.get(source, {})
    if not isinstance(by_scope, dict):
        return []
    fields = by_scope.get(scope, [])
    if not isinstance(fields, list):
        return []
    return [str(field) for field in fields]


def _schema_complete_required_summary_fields_source_scopes(
    *,
    workflow_contracts: Any,
) -> list[str]:
    values = workflow_contracts.DELIVERY_ACCEPTANCE_GATE_SCHEMA.get(
        "complete_required_summary_fields_source_scopes",
        [],
    )
    if isinstance(values, list):
        return [str(value) for value in values]
    by_source_scope = workflow_contracts.DELIVERY_ACCEPTANCE_GATE_SCHEMA.get(
        "complete_required_summary_fields_by_source_scope",
        {},
    )
    return _source_scope_values(by_source_scope if isinstance(by_source_scope, dict) else {})


def _summary_counts_total(results: list[dict[str, Any]], key: str) -> int:
    total = 0
    for result in results:
        counts = result.get("summary_counts")
        if not isinstance(counts, dict):
            continue
        value = counts.get(key)
        if isinstance(value, int) and not isinstance(value, bool):
            total += value
    return total


def _summary_counts_map_totals(
    results: list[dict[str, Any]],
    key: str,
) -> dict[str, int]:
    totals: dict[str, int] = {}
    for result in results:
        counts = result.get("summary_counts")
        if not isinstance(counts, dict):
            continue
        values = counts.get(key)
        if not isinstance(values, dict):
            continue
        for count_key, count_value in values.items():
            if isinstance(count_value, int) and not isinstance(count_value, bool):
                totals[str(count_key)] = totals.get(str(count_key), 0) + count_value
    return dict(sorted(totals.items()))


def _complete_required_summary_fields_by_source_scope(
    results: list[dict[str, Any]],
) -> dict[str, dict[str, list[str]]]:
    fields_by_source_scope: dict[str, dict[str, set[str]]] = {}
    for result in results:
        source = result.get("source")
        scope = result.get("verification_scope")
        fields = result.get("complete_required_summary_fields")
        if not isinstance(source, str) or not isinstance(scope, str):
            continue
        if not isinstance(fields, list):
            continue
        scope_map = fields_by_source_scope.setdefault(source, {})
        scope_fields = scope_map.setdefault(scope, set())
        scope_fields.update(str(field) for field in fields)
    return {
        source: {
            scope: sorted(fields)
            for scope, fields in sorted(scope_map.items())
        }
        for source, scope_map in sorted(fields_by_source_scope.items())
    }


def _enabled_requirements(gate_payload: Any) -> list[str]:
    if not isinstance(gate_payload, dict) or not isinstance(
        gate_payload.get("acceptance_requirements"), dict
    ):
        return []
    return [
        str(key)
        for key, value in gate_payload["acceptance_requirements"].items()
        if value is True
    ]


def _affected_inputs(gate_payload: Any, limit: int = 10) -> tuple[list[str], int]:
    if not isinstance(gate_payload, dict) or not isinstance(
        gate_payload.get("reason_details"), list
    ):
        return [], 0
    inputs: list[str] = []
    for detail in gate_payload["reason_details"]:
        if not isinstance(detail, dict) or not isinstance(detail.get("inputs"), list):
            continue
        for input_path in detail["inputs"]:
            input_text = str(input_path)
            if input_text not in inputs:
                inputs.append(input_text)
    return inputs[:limit], len(inputs)


def _join_or_none(values: Any) -> str:
    if not isinstance(values, list) or not values:
        return "none"
    return ",".join(str(value) for value in values)


def _summary_text_field(name: str, value: Any, *, enabled: bool = True) -> str:
    if not enabled or value is None:
        return ""
    if isinstance(value, list):
        value = _join_or_none(value)
    return f"{name}={value} "


def _format_count_map(value: Mapping[str, int] | None) -> str:
    if not isinstance(value, Mapping) or not value:
        return "none"
    return "+".join(
        f"{key}:{value[key]}"
        for key in sorted(value)
        if isinstance(key, str)
        and isinstance(value[key], int)
        and not isinstance(value[key], bool)
    ) or "none"


def _source_scope_summary_fields_text(
    fields_by_source_scope: dict[str, dict[str, list[str]]] | None,
) -> list[str]:
    if not isinstance(fields_by_source_scope, dict):
        return []
    values: list[str] = []
    for source, scope_map in sorted(fields_by_source_scope.items()):
        if not isinstance(scope_map, dict):
            continue
        for scope, fields in sorted(scope_map.items()):
            if not isinstance(fields, list):
                continue
            fields_text = "+".join(str(field) for field in fields)
            values.append(f"{source}/{scope}:{fields_text}")
    return values


def _source_scope_values(
    fields_by_source_scope: dict[str, dict[str, list[str]]] | None,
) -> list[str]:
    if not isinstance(fields_by_source_scope, dict):
        return []
    values: list[str] = []
    for source, scope_map in sorted(fields_by_source_scope.items()):
        if not isinstance(scope_map, dict):
            continue
        for scope in sorted(scope_map):
            values.append(f"{source}/{scope}")
    return values


def _nonnegative_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("value must be an integer >= 0") from exc
    if parsed < 0:
        raise argparse.ArgumentTypeError("value must be an integer >= 0")
    return parsed


def _nonempty_text(value: str) -> str:
    if not value.strip():
        raise argparse.ArgumentTypeError("value must be non-empty")
    return value


def _source_scope_text(value: str) -> str:
    value = _nonempty_text(value).strip()
    parts = value.split("/")
    if len(parts) != 2 or not parts[0].strip() or not parts[1].strip():
        raise argparse.ArgumentTypeError(
            "value must be formatted as non-empty source/scope"
        )
    return f"{parts[0].strip()}/{parts[1].strip()}"


def _gate_check_count_text(value: str) -> tuple[str, int]:
    value = _nonempty_text(value).strip()
    parts = value.split("=", 1)
    if len(parts) != 2 or not parts[0].strip() or not parts[1].strip():
        raise argparse.ArgumentTypeError("value must be formatted as check=count")
    try:
        count = _nonnegative_int(parts[1].strip())
    except argparse.ArgumentTypeError as exc:
        raise argparse.ArgumentTypeError(
            "value must be formatted as check=count with count >= 0"
        ) from exc
    return parts[0].strip(), count


def _gate_check_expectations(values: list[tuple[str, int]]) -> dict[str, int]:
    return {str(key): int(count) for key, count in values}


def _summary_count_text(value: str) -> tuple[str, int]:
    value = _nonempty_text(value).strip()
    parts = value.split("=", 1)
    if len(parts) != 2 or not parts[0].strip() or not parts[1].strip():
        raise argparse.ArgumentTypeError("value must be formatted as field=count")
    try:
        count = _nonnegative_int(parts[1].strip())
    except argparse.ArgumentTypeError as exc:
        raise argparse.ArgumentTypeError(
            "value must be formatted as field=count with count >= 0"
        ) from exc
    return parts[0].strip(), count


def _summary_count_expectations(values: list[tuple[str, int]]) -> dict[str, int]:
    return {str(key): int(count) for key, count in values}


def _mismatch_kind_count_text(value: str) -> tuple[str, int]:
    value = _nonempty_text(value).strip()
    parts = value.split("=", 1)
    if len(parts) != 2 or not parts[0].strip() or not parts[1].strip():
        raise argparse.ArgumentTypeError("value must be formatted as kind=count")
    try:
        count = _nonnegative_int(parts[1].strip())
    except argparse.ArgumentTypeError as exc:
        raise argparse.ArgumentTypeError(
            "value must be formatted as kind=count with count >= 0"
        ) from exc
    return parts[0].strip(), count


def _mismatch_kind_count_expectations(
    values: list[tuple[str, int]],
) -> dict[str, int]:
    return {str(key): int(count) for key, count in values}


def _summary_value_text(value: str) -> tuple[str, int]:
    value = _nonempty_text(value).strip()
    parts = value.split("=", 1)
    if len(parts) != 2 or not parts[0].strip() or not parts[1].strip():
        raise argparse.ArgumentTypeError("value must be formatted as path=count")
    try:
        count = _nonnegative_int(parts[1].strip())
    except argparse.ArgumentTypeError as exc:
        raise argparse.ArgumentTypeError(
            "value must be formatted as path=count with count >= 0"
        ) from exc
    return parts[0].strip(), count


def _summary_value_expectations(values: list[tuple[str, int]]) -> dict[str, int]:
    return {str(key): int(count) for key, count in values}


def _same_path(left: Path, right: Path) -> bool:
    return left.resolve(strict=False) == right.resolve(strict=False)


def _path_in_list(path: Path, paths: list[Path]) -> bool:
    return any(_same_path(path, item) for item in paths)


def _format_text_result(result: dict[str, Any]) -> str:
    counts = result.get("summary_counts", {})
    if not isinstance(counts, dict):
        counts = {}
    full_mechanical_gate = result.get(
        "requires_full_mechanical_restoration_gate"
    )
    if full_mechanical_gate is True:
        full_mechanical_gate_text = "true"
    elif full_mechanical_gate is False:
        full_mechanical_gate_text = "false"
    else:
        full_mechanical_gate_text = "unknown"
    count_text = (
        f"inputs:{counts.get('inputs_count', 0)},"
        f"errors:{counts.get('error_count', 0)},"
        f"live:{counts.get('live_smoke_count', 0)},"
        f"smokereports:{counts.get('smoke_report_written_count', 0)}/"
        f"{counts.get('smoke_report_missing_count', 0)}/"
        f"{counts.get('smoke_report_read_error_count', 0)},"
        f"control:{counts.get('control_readback_checked_count', 0)}/"
        f"{counts.get('control_readback_missing_count', 0)}/"
        f"{counts.get('control_configured_count', 0)},"
        f"verified:{counts.get('delivery_godot_verified_count', 0)},"
        f"static:{counts.get('delivery_static_only_count', 0)},"
        f"treegate:{counts.get('node_tree_gate_enabled_count', 0)}/"
        f"{counts.get('node_tree_full_restoration_required_count', 0)},"
        f"mechgate:{counts.get('mechanical_gate_enabled_count', 0)}/"
        f"{counts.get('full_mechanical_restoration_required_count', 0)},"
        f"failures:{counts.get('failure_reasons_count', 0)}"
    )
    check_text = _format_gate_check_counts(counts)
    skip_reason_text = (
        f" skip_reason={result.get('skip_reason')}"
        if result.get("skip_reason")
        else ""
    )
    complete_required_summary_fields = result.get("complete_required_summary_fields")
    complete_required_summary_fields_text = (
        "complete_required_summary_fields="
        f"{'+'.join(str(field) for field in complete_required_summary_fields)} "
        if isinstance(complete_required_summary_fields, list)
        and complete_required_summary_fields
        else ""
    )
    complete_required_summary_fields_count_text = (
        "complete_required_summary_fields_count="
        f"{result.get('complete_required_summary_fields_count')} "
        if isinstance(complete_required_summary_fields, list)
        and complete_required_summary_fields
        else ""
    )
    return (
        "delivery_acceptance_gate validation "
        f"status={result.get('status')} "
        f"source={result.get('source')} "
        f"scope={result.get('verification_scope')} "
        f"profile={result.get('acceptance_profile')} "
        f"passed={str(result.get('passed')).lower()} "
        f"full_mechanical_gate={full_mechanical_gate_text} "
        f"exit_code={result.get('exit_code')} "
        f"requirements={_join_or_none(result.get('enabled_requirements'))} "
        f"{complete_required_summary_fields_text}"
        f"{complete_required_summary_fields_count_text}"
        f"reason_codes={_join_or_none(result.get('reason_codes'))} "
        f"counts={count_text} "
        f"checks={check_text} "
        f"affected_inputs={_join_or_none(result.get('affected_inputs'))} "
        f"errors={_join_or_none(result.get('errors'))}"
        f"{skip_reason_text}"
    )


def _format_gate_check_counts(counts: dict[str, Any]) -> str:
    groups = [
        ("tree", counts.get("node_tree_gate_check_counts")),
        ("mech", counts.get("mechanical_gate_check_counts")),
    ]
    parts: list[str] = []
    for label, raw_counts in groups:
        if not isinstance(raw_counts, dict):
            continue
        entries = [
            f"{key}:{value}"
            for key, value in sorted(raw_counts.items())
            if _is_positive_count(value)
        ]
        if entries:
            parts.append(f"{label}[{','.join(entries)}]")
    return "|".join(parts) if parts else "none"


def _format_gate_check_count_expectations(
    *,
    node_tree_gate_check_counts: dict[str, int] | None = None,
    mechanical_gate_check_counts: dict[str, int] | None = None,
) -> str:
    groups = [
        ("tree", node_tree_gate_check_counts),
        ("mech", mechanical_gate_check_counts),
    ]
    parts: list[str] = []
    for label, raw_counts in groups:
        if not isinstance(raw_counts, dict):
            continue
        entries = [
            f"{key}:{value}"
            for key, value in sorted(raw_counts.items())
            if isinstance(value, int) and not isinstance(value, bool)
        ]
        if entries:
            parts.append(f"{label}[{','.join(entries)}]")
    return "|".join(parts) if parts else "none"


def _format_gate_check_count_mismatches(
    *,
    node_tree_gate_check_counts: dict[str, dict[str, int]] | None = None,
    mechanical_gate_check_counts: dict[str, dict[str, int]] | None = None,
) -> str:
    groups = [
        ("tree", node_tree_gate_check_counts),
        ("mech", mechanical_gate_check_counts),
    ]
    parts: list[str] = []
    for label, raw_counts in groups:
        if not isinstance(raw_counts, dict):
            continue
        entries: list[str] = []
        for key, mismatch in sorted(raw_counts.items()):
            if not isinstance(mismatch, dict):
                continue
            expected = mismatch.get("expected")
            actual = mismatch.get("actual")
            if (
                isinstance(expected, int)
                and not isinstance(expected, bool)
                and isinstance(actual, int)
                and not isinstance(actual, bool)
            ):
                entries.append(f"{key}:{expected}/{actual}")
        if entries:
            parts.append(f"{label}[{','.join(entries)}]")
    return "|".join(parts) if parts else "none"


def _format_expected_summary_counts(counts: dict[str, int] | None) -> str:
    if not isinstance(counts, dict) or not counts:
        return "none"
    entries = [
        f"{key}:{value}"
        for key, value in sorted(counts.items())
        if isinstance(value, int) and not isinstance(value, bool)
    ]
    return ",".join(entries) if entries else "none"


def _format_summary_count_mismatches(
    counts: dict[str, dict[str, int]] | None,
) -> str:
    if not isinstance(counts, dict) or not counts:
        return "none"
    entries: list[str] = []
    for key, mismatch in sorted(counts.items()):
        if not isinstance(mismatch, dict):
            continue
        expected = mismatch.get("expected")
        actual = mismatch.get("actual")
        if (
            isinstance(expected, int)
            and not isinstance(expected, bool)
            and isinstance(actual, int)
            and not isinstance(actual, bool)
        ):
            entries.append(f"{key}:{expected}/{actual}")
    return ",".join(entries) if entries else "none"


def _format_expected_summary_values(counts: dict[str, int] | None) -> str:
    return _format_expected_summary_counts(counts)


def _format_summary_value_mismatches(
    counts: dict[str, dict[str, int]] | None,
) -> str:
    return _format_summary_count_mismatches(counts)


def _is_positive_count(value: Any) -> bool:
    try:
        return int(value) > 0
    except (TypeError, ValueError):
        return False


def _format_text_summary(
    *,
    preview_limit: int | None = None,
    inputs_count: int,
    expanded_inputs_count: int | None = None,
    expected_expanded_inputs_count: int | None = None,
    expected_excluded_output_artifacts_count: int | None = None,
    success_count: int,
    error_count: int,
    passed_true_count: int = 0,
    passed_false_count: int = 0,
    passed_unknown_count: int = 0,
    requires_full_mechanical_restoration_gate_true_count: int = 0,
    requires_full_mechanical_restoration_gate_false_count: int = 0,
    requires_full_mechanical_restoration_gate_unknown_count: int = 0,
    smoke_report_written_count: int = 0,
    smoke_report_missing_count: int = 0,
    smoke_report_read_error_count: int = 0,
    control_configured_count: int = 0,
    control_readback_checked_count: int = 0,
    control_readback_missing_count: int = 0,
    node_tree_gate_check_counts: dict[str, int] | None = None,
    mechanical_gate_check_counts: dict[str, int] | None = None,
    skipped_count: int = 0,
    node_tree_manifest_sidecar_count: int = 0,
    node_tree_manifest_sidecar_complete_count: int = 0,
    node_tree_manifest_sidecar_incomplete_count: int = 0,
    node_tree_manifest_sidecar_valid_count: int = 0,
    node_tree_manifest_sidecar_invalid_count: int = 0,
    node_tree_manifest_sidecar_validation_error_count: int = 0,
    node_tree_manifest_sidecar_path_incomplete_count: int = 0,
    node_tree_manifest_sidecar_path_map_mismatch_count: int = 0,
    node_tree_manifest_sidecar_path_map_mismatch_kind_counts: (
        dict[str, int] | None
    ) = None,
    node_tree_manifest_sidecar_parts_planned_count: int = 0,
    node_tree_manifest_sidecar_joints_planned_count: int = 0,
    node_tree_manifest_sidecar_part_path_count: int = 0,
    node_tree_manifest_sidecar_joint_path_count: int = 0,
    aggregate_errors: list[str] | None = None,
    input_paths: list[str] | None = None,
    contract_versions: list[str] | None = None,
    levels: list[str] | None = None,
    sources: list[str] | None = None,
    verification_scopes: list[str] | None = None,
    acceptance_profiles: list[str] | None = None,
    enabled_requirements: list[str] | None = None,
    complete_required_summary_fields_by_source_scope: (
        dict[str, dict[str, list[str]]] | None
    ) = None,
    complete_required_summary_fields_source_scope_count: int = 0,
    complete_required_summary_fields_source_scopes: list[str] | None = None,
    reason_codes: list[str] | None = None,
    affected_inputs: list[str] | None = None,
    failed_inputs: list[str] | None = None,
    passed_true_inputs: list[str] | None = None,
    passed_false_inputs: list[str] | None = None,
    passed_unknown_inputs: list[str] | None = None,
    full_mechanical_gate_true_inputs: list[str] | None = None,
    full_mechanical_gate_false_inputs: list[str] | None = None,
    full_mechanical_gate_unknown_inputs: list[str] | None = None,
    skipped_inputs: list[str] | None = None,
    skipped_reasons: list[str] | None = None,
    truncated_previews: list[str] | None = None,
    expected_affected_inputs: list[str] | None = None,
    missing_expected_affected_inputs: list[str] | None = None,
    expected_failed_inputs: list[str] | None = None,
    missing_expected_failed_inputs: list[str] | None = None,
    expected_passed_true_inputs: list[str] | None = None,
    missing_expected_passed_true_inputs: list[str] | None = None,
    expected_passed_false_inputs: list[str] | None = None,
    missing_expected_passed_false_inputs: list[str] | None = None,
    expected_passed_unknown_inputs: list[str] | None = None,
    missing_expected_passed_unknown_inputs: list[str] | None = None,
    expected_full_mechanical_gate_true_inputs: list[str] | None = None,
    missing_expected_full_mechanical_gate_true_inputs: list[str] | None = None,
    expected_full_mechanical_gate_false_inputs: list[str] | None = None,
    missing_expected_full_mechanical_gate_false_inputs: list[str] | None = None,
    expected_full_mechanical_gate_unknown_inputs: list[str] | None = None,
    missing_expected_full_mechanical_gate_unknown_inputs: list[str] | None = None,
    expected_truncated_previews: list[str] | None = None,
    missing_expected_truncated_previews: list[str] | None = None,
    expected_skipped_inputs: list[str] | None = None,
    missing_expected_skipped_inputs: list[str] | None = None,
    allowed_affected_inputs: list[str] | None = None,
    unexpected_affected_inputs: list[str] | None = None,
    allowed_failed_inputs: list[str] | None = None,
    unexpected_failed_inputs: list[str] | None = None,
    allowed_passed_true_inputs: list[str] | None = None,
    unexpected_passed_true_inputs: list[str] | None = None,
    allowed_passed_false_inputs: list[str] | None = None,
    unexpected_passed_false_inputs: list[str] | None = None,
    allowed_passed_unknown_inputs: list[str] | None = None,
    unexpected_passed_unknown_inputs: list[str] | None = None,
    allowed_full_mechanical_gate_true_inputs: list[str] | None = None,
    unexpected_full_mechanical_gate_true_inputs: list[str] | None = None,
    allowed_full_mechanical_gate_false_inputs: list[str] | None = None,
    unexpected_full_mechanical_gate_false_inputs: list[str] | None = None,
    allowed_full_mechanical_gate_unknown_inputs: list[str] | None = None,
    unexpected_full_mechanical_gate_unknown_inputs: list[str] | None = None,
    allowed_truncated_previews: list[str] | None = None,
    unexpected_truncated_previews: list[str] | None = None,
    allowed_skipped_inputs: list[str] | None = None,
    unexpected_skipped_inputs: list[str] | None = None,
    forbidden_affected_inputs: list[str] | None = None,
    present_forbidden_affected_inputs: list[str] | None = None,
    forbidden_failed_inputs: list[str] | None = None,
    present_forbidden_failed_inputs: list[str] | None = None,
    forbidden_passed_true_inputs: list[str] | None = None,
    present_forbidden_passed_true_inputs: list[str] | None = None,
    forbidden_passed_false_inputs: list[str] | None = None,
    present_forbidden_passed_false_inputs: list[str] | None = None,
    forbidden_passed_unknown_inputs: list[str] | None = None,
    present_forbidden_passed_unknown_inputs: list[str] | None = None,
    forbidden_full_mechanical_gate_true_inputs: list[str] | None = None,
    present_forbidden_full_mechanical_gate_true_inputs: list[str] | None = None,
    forbidden_full_mechanical_gate_false_inputs: list[str] | None = None,
    present_forbidden_full_mechanical_gate_false_inputs: list[str] | None = None,
    forbidden_full_mechanical_gate_unknown_inputs: list[str] | None = None,
    present_forbidden_full_mechanical_gate_unknown_inputs: list[str] | None = None,
    expected_inputs: list[str] | None = None,
    missing_expected_inputs: list[str] | None = None,
    allowed_inputs: list[str] | None = None,
    unexpected_inputs: list[str] | None = None,
    forbidden_inputs: list[str] | None = None,
    present_forbidden_inputs: list[str] | None = None,
    forbidden_truncated_previews: list[str] | None = None,
    present_forbidden_truncated_previews: list[str] | None = None,
    forbidden_skipped_inputs: list[str] | None = None,
    present_forbidden_skipped_inputs: list[str] | None = None,
    expected_inputs_count: int | None = None,
    expected_success_count: int | None = None,
    expected_error_count: int | None = None,
    expected_passed_true_count: int | None = None,
    expected_passed_false_count: int | None = None,
    expected_passed_unknown_count: int | None = None,
    expected_full_mechanical_gate_true_count: int | None = None,
    expected_full_mechanical_gate_false_count: int | None = None,
    expected_full_mechanical_gate_unknown_count: int | None = None,
    expected_smoke_report_written_count: int | None = None,
    expected_smoke_report_missing_count: int | None = None,
    expected_smoke_report_read_error_count: int | None = None,
    expected_control_configured_count: int | None = None,
    expected_control_readback_checked_count: int | None = None,
    expected_control_readback_missing_count: int | None = None,
    expected_node_tree_manifest_sidecar_count: int | None = None,
    expected_node_tree_manifest_sidecar_complete_count: int | None = None,
    expected_node_tree_manifest_sidecar_incomplete_count: int | None = None,
    expected_node_tree_manifest_sidecar_valid_count: int | None = None,
    expected_node_tree_manifest_sidecar_invalid_count: int | None = None,
    expected_node_tree_manifest_sidecar_validation_error_count: int | None = None,
    expected_node_tree_manifest_sidecar_path_incomplete_count: int | None = None,
    expected_node_tree_manifest_sidecar_path_map_mismatch_count: int | None = None,
    expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts: (
        dict[str, int] | None
    ) = None,
    expected_node_tree_manifest_sidecar_parts_planned_count: int | None = None,
    expected_node_tree_manifest_sidecar_joints_planned_count: int | None = None,
    expected_node_tree_manifest_sidecar_part_path_count: int | None = None,
    expected_node_tree_manifest_sidecar_joint_path_count: int | None = None,
    expected_node_tree_gate_check_counts: dict[str, int] | None = None,
    expected_mechanical_gate_check_counts: dict[str, int] | None = None,
    expected_summary_counts: dict[str, int] | None = None,
    mismatched_expected_summary_counts: dict[str, dict[str, int]] | None = None,
    expected_summary_values: dict[str, int] | None = None,
    expected_summary_value_sources: list[str] | None = None,
    missing_expected_summary_value_sources: list[str] | None = None,
    expected_summary_value_source_matched_count: int | None = None,
    expected_summary_value_source_excluded_count: int | None = None,
    expected_summary_value_source_excluded_sources: list[str] | None = None,
    missing_expected_summary_value_source_excluded_sources: list[str] | None = None,
    allowed_summary_value_source_excluded_sources: list[str] | None = None,
    unexpected_summary_value_source_excluded_sources: list[str] | None = None,
    forbidden_summary_value_source_excluded_sources: list[str] | None = None,
    present_forbidden_summary_value_source_excluded_sources: list[str] | None = None,
    summary_value_source_matched_count: int | None = None,
    summary_value_source_matched_sources: list[str] | None = None,
    allowed_summary_value_source_matched_sources: list[str] | None = None,
    unexpected_summary_value_source_matched_sources: list[str] | None = None,
    forbidden_summary_value_source_matched_sources: list[str] | None = None,
    present_forbidden_summary_value_source_matched_sources: list[str] | None = None,
    summary_value_source_excluded_count: int | None = None,
    summary_value_source_excluded_sources: list[str] | None = None,
    mismatched_expected_summary_values: dict[str, dict[str, int]] | None = None,
    mismatched_expected_node_tree_gate_check_counts: (
        dict[str, dict[str, int]] | None
    ) = None,
    mismatched_expected_mechanical_gate_check_counts: (
        dict[str, dict[str, int]] | None
    ) = None,
    expected_skipped_count: int | None = None,
    expected_reason_codes_count: int | None = None,
    expected_contract_versions_count: int | None = None,
    expected_summary_versions_count: int | None = None,
    expected_validation_summary_statuses_count: int | None = None,
    expected_levels_count: int | None = None,
    expected_sources_count: int | None = None,
    expected_verification_scopes_count: int | None = None,
    expected_acceptance_profiles_count: int | None = None,
    expected_enabled_requirements_count: int | None = None,
    expected_complete_required_summary_fields_source_scope_count: int | None = None,
    expected_affected_inputs_count: int | None = None,
    expected_failed_inputs_count: int | None = None,
    expected_skipped_inputs_count: int | None = None,
    expected_skipped_reasons_count: int | None = None,
    expected_truncated_previews_count: int | None = None,
    expected_contract_versions: list[str] | None = None,
    missing_expected_contract_versions: list[str] | None = None,
    expected_levels: list[str] | None = None,
    missing_expected_levels: list[str] | None = None,
    expected_sources: list[str] | None = None,
    missing_expected_sources: list[str] | None = None,
    expected_verification_scopes: list[str] | None = None,
    missing_expected_verification_scopes: list[str] | None = None,
    expected_acceptance_profiles: list[str] | None = None,
    missing_expected_acceptance_profiles: list[str] | None = None,
    expected_enabled_requirements: list[str] | None = None,
    missing_expected_enabled_requirements: list[str] | None = None,
    expected_complete_required_summary_fields_source_scopes: (
        list[str] | None
    ) = None,
    missing_expected_complete_required_summary_fields_source_scopes: (
        list[str] | None
    ) = None,
    allowed_complete_required_summary_fields_source_scopes: (
        list[str] | None
    ) = None,
    unexpected_complete_required_summary_fields_source_scopes: (
        list[str] | None
    ) = None,
    forbidden_complete_required_summary_fields_source_scopes: (
        list[str] | None
    ) = None,
    present_forbidden_complete_required_summary_fields_source_scopes: (
        list[str] | None
    ) = None,
    allowed_contract_versions: list[str] | None = None,
    unexpected_contract_versions: list[str] | None = None,
    allowed_levels: list[str] | None = None,
    unexpected_levels: list[str] | None = None,
    allowed_sources: list[str] | None = None,
    unexpected_sources: list[str] | None = None,
    allowed_verification_scopes: list[str] | None = None,
    unexpected_verification_scopes: list[str] | None = None,
    allowed_acceptance_profiles: list[str] | None = None,
    unexpected_acceptance_profiles: list[str] | None = None,
    allowed_enabled_requirements: list[str] | None = None,
    unexpected_enabled_requirements: list[str] | None = None,
    forbidden_sources: list[str] | None = None,
    present_forbidden_sources: list[str] | None = None,
    forbidden_verification_scopes: list[str] | None = None,
    present_forbidden_verification_scopes: list[str] | None = None,
    forbidden_acceptance_profiles: list[str] | None = None,
    present_forbidden_acceptance_profiles: list[str] | None = None,
    forbidden_enabled_requirements: list[str] | None = None,
    present_forbidden_enabled_requirements: list[str] | None = None,
    forbidden_contract_versions: list[str] | None = None,
    present_forbidden_contract_versions: list[str] | None = None,
    forbidden_levels: list[str] | None = None,
    present_forbidden_levels: list[str] | None = None,
    expected_reason_codes: list[str] | None = None,
    missing_expected_reason_codes: list[str] | None = None,
    allowed_reason_codes: list[str] | None = None,
    unexpected_reason_codes: list[str] | None = None,
    forbidden_reason_codes: list[str] | None = None,
    present_forbidden_reason_codes: list[str] | None = None,
    expected_skipped_reasons: list[str] | None = None,
    missing_expected_skipped_reasons: list[str] | None = None,
    allowed_skipped_reasons: list[str] | None = None,
    unexpected_skipped_reasons: list[str] | None = None,
    forbidden_skipped_reasons: list[str] | None = None,
    present_forbidden_skipped_reasons: list[str] | None = None,
    excluded_output_artifact_inputs: list[str] | None = None,
    show_passed_counts: bool = False,
    show_metadata: bool = False,
    show_reason_codes: bool = False,
    show_inputs: bool = False,
    show_skipped_reasons: bool = False,
    show_full_mechanical_gate_counts: bool = False,
    show_smoke_report_counts: bool = False,
    show_control_readback_counts: bool = False,
    require_passed: bool,
    require_full_mechanical_restoration_gate: bool = False,
    fail_on_full_mechanical_gate_false: bool = False,
    fail_on_full_mechanical_gate_unknown: bool = False,
    fail_on_smoke_report_missing: bool = False,
    fail_on_smoke_report_read_error: bool = False,
    fail_on_control_readback_missing: bool = False,
    fail_on_node_tree_manifest_sidecar_incomplete: bool = False,
    fail_on_invalid_node_tree_manifest_sidecar: bool = False,
    fail_on_node_tree_manifest_sidecar_validation_error: bool = False,
    fail_on_node_tree_manifest_sidecar_path_incomplete: bool = False,
    fail_on_node_tree_manifest_sidecar_path_map_mismatch: bool = False,
) -> str:
    aggregate_errors = aggregate_errors or []
    input_paths = input_paths or []
    contract_versions = contract_versions or []
    levels = levels or []
    sources = sources or []
    verification_scopes = verification_scopes or []
    acceptance_profiles = acceptance_profiles or []
    enabled_requirements = enabled_requirements or []
    complete_required_summary_fields = _source_scope_summary_fields_text(
        complete_required_summary_fields_by_source_scope
    )
    complete_required_summary_fields_source_scopes = (
        complete_required_summary_fields_source_scopes
        or _source_scope_values(complete_required_summary_fields_by_source_scope)
    )
    reason_codes = reason_codes or []
    affected_inputs = affected_inputs or []
    failed_inputs = failed_inputs or []
    passed_true_inputs = passed_true_inputs or []
    passed_false_inputs = passed_false_inputs or []
    passed_unknown_inputs = passed_unknown_inputs or []
    full_mechanical_gate_true_inputs = full_mechanical_gate_true_inputs or []
    full_mechanical_gate_false_inputs = full_mechanical_gate_false_inputs or []
    full_mechanical_gate_unknown_inputs = full_mechanical_gate_unknown_inputs or []
    skipped_inputs = skipped_inputs or []
    skipped_reasons = skipped_reasons or []
    truncated_previews = truncated_previews or []
    expected_affected_inputs = expected_affected_inputs or []
    missing_expected_affected_inputs = missing_expected_affected_inputs or []
    expected_failed_inputs = expected_failed_inputs or []
    missing_expected_failed_inputs = missing_expected_failed_inputs or []
    expected_passed_true_inputs = expected_passed_true_inputs or []
    missing_expected_passed_true_inputs = missing_expected_passed_true_inputs or []
    expected_passed_false_inputs = expected_passed_false_inputs or []
    missing_expected_passed_false_inputs = (
        missing_expected_passed_false_inputs or []
    )
    expected_passed_unknown_inputs = expected_passed_unknown_inputs or []
    missing_expected_passed_unknown_inputs = (
        missing_expected_passed_unknown_inputs or []
    )
    expected_full_mechanical_gate_true_inputs = (
        expected_full_mechanical_gate_true_inputs or []
    )
    missing_expected_full_mechanical_gate_true_inputs = (
        missing_expected_full_mechanical_gate_true_inputs or []
    )
    expected_full_mechanical_gate_false_inputs = (
        expected_full_mechanical_gate_false_inputs or []
    )
    missing_expected_full_mechanical_gate_false_inputs = (
        missing_expected_full_mechanical_gate_false_inputs or []
    )
    expected_full_mechanical_gate_unknown_inputs = (
        expected_full_mechanical_gate_unknown_inputs or []
    )
    missing_expected_full_mechanical_gate_unknown_inputs = (
        missing_expected_full_mechanical_gate_unknown_inputs or []
    )
    expected_truncated_previews = expected_truncated_previews or []
    missing_expected_truncated_previews = missing_expected_truncated_previews or []
    expected_skipped_inputs = expected_skipped_inputs or []
    missing_expected_skipped_inputs = missing_expected_skipped_inputs or []
    allowed_affected_inputs = allowed_affected_inputs or []
    unexpected_affected_inputs = unexpected_affected_inputs or []
    allowed_failed_inputs = allowed_failed_inputs or []
    unexpected_failed_inputs = unexpected_failed_inputs or []
    allowed_passed_true_inputs = allowed_passed_true_inputs or []
    unexpected_passed_true_inputs = unexpected_passed_true_inputs or []
    allowed_passed_false_inputs = allowed_passed_false_inputs or []
    unexpected_passed_false_inputs = unexpected_passed_false_inputs or []
    allowed_passed_unknown_inputs = allowed_passed_unknown_inputs or []
    unexpected_passed_unknown_inputs = unexpected_passed_unknown_inputs or []
    allowed_full_mechanical_gate_true_inputs = (
        allowed_full_mechanical_gate_true_inputs or []
    )
    unexpected_full_mechanical_gate_true_inputs = (
        unexpected_full_mechanical_gate_true_inputs or []
    )
    allowed_full_mechanical_gate_false_inputs = (
        allowed_full_mechanical_gate_false_inputs or []
    )
    unexpected_full_mechanical_gate_false_inputs = (
        unexpected_full_mechanical_gate_false_inputs or []
    )
    allowed_full_mechanical_gate_unknown_inputs = (
        allowed_full_mechanical_gate_unknown_inputs or []
    )
    unexpected_full_mechanical_gate_unknown_inputs = (
        unexpected_full_mechanical_gate_unknown_inputs or []
    )
    allowed_truncated_previews = allowed_truncated_previews or []
    unexpected_truncated_previews = unexpected_truncated_previews or []
    allowed_skipped_inputs = allowed_skipped_inputs or []
    unexpected_skipped_inputs = unexpected_skipped_inputs or []
    forbidden_affected_inputs = forbidden_affected_inputs or []
    present_forbidden_affected_inputs = present_forbidden_affected_inputs or []
    forbidden_failed_inputs = forbidden_failed_inputs or []
    present_forbidden_failed_inputs = present_forbidden_failed_inputs or []
    forbidden_passed_true_inputs = forbidden_passed_true_inputs or []
    present_forbidden_passed_true_inputs = (
        present_forbidden_passed_true_inputs or []
    )
    forbidden_passed_false_inputs = forbidden_passed_false_inputs or []
    present_forbidden_passed_false_inputs = (
        present_forbidden_passed_false_inputs or []
    )
    forbidden_passed_unknown_inputs = forbidden_passed_unknown_inputs or []
    present_forbidden_passed_unknown_inputs = (
        present_forbidden_passed_unknown_inputs or []
    )
    forbidden_full_mechanical_gate_true_inputs = (
        forbidden_full_mechanical_gate_true_inputs or []
    )
    present_forbidden_full_mechanical_gate_true_inputs = (
        present_forbidden_full_mechanical_gate_true_inputs or []
    )
    forbidden_full_mechanical_gate_false_inputs = (
        forbidden_full_mechanical_gate_false_inputs or []
    )
    present_forbidden_full_mechanical_gate_false_inputs = (
        present_forbidden_full_mechanical_gate_false_inputs or []
    )
    forbidden_full_mechanical_gate_unknown_inputs = (
        forbidden_full_mechanical_gate_unknown_inputs or []
    )
    present_forbidden_full_mechanical_gate_unknown_inputs = (
        present_forbidden_full_mechanical_gate_unknown_inputs or []
    )
    expected_inputs = expected_inputs or []
    missing_expected_inputs = missing_expected_inputs or []
    allowed_inputs = allowed_inputs or []
    unexpected_inputs = unexpected_inputs or []
    forbidden_inputs = forbidden_inputs or []
    present_forbidden_inputs = present_forbidden_inputs or []
    forbidden_truncated_previews = forbidden_truncated_previews or []
    present_forbidden_truncated_previews = (
        present_forbidden_truncated_previews or []
    )
    forbidden_skipped_inputs = forbidden_skipped_inputs or []
    present_forbidden_skipped_inputs = present_forbidden_skipped_inputs or []
    expected_contract_versions = expected_contract_versions or []
    missing_expected_contract_versions = missing_expected_contract_versions or []
    expected_levels = expected_levels or []
    missing_expected_levels = missing_expected_levels or []
    expected_sources = expected_sources or []
    missing_expected_sources = missing_expected_sources or []
    expected_verification_scopes = expected_verification_scopes or []
    missing_expected_verification_scopes = (
        missing_expected_verification_scopes or []
    )
    expected_acceptance_profiles = expected_acceptance_profiles or []
    missing_expected_acceptance_profiles = missing_expected_acceptance_profiles or []
    expected_enabled_requirements = expected_enabled_requirements or []
    missing_expected_enabled_requirements = (
        missing_expected_enabled_requirements or []
    )
    expected_complete_required_summary_fields_source_scopes = (
        expected_complete_required_summary_fields_source_scopes or []
    )
    missing_expected_complete_required_summary_fields_source_scopes = (
        missing_expected_complete_required_summary_fields_source_scopes or []
    )
    allowed_complete_required_summary_fields_source_scopes = (
        allowed_complete_required_summary_fields_source_scopes or []
    )
    unexpected_complete_required_summary_fields_source_scopes = (
        unexpected_complete_required_summary_fields_source_scopes or []
    )
    forbidden_complete_required_summary_fields_source_scopes = (
        forbidden_complete_required_summary_fields_source_scopes or []
    )
    present_forbidden_complete_required_summary_fields_source_scopes = (
        present_forbidden_complete_required_summary_fields_source_scopes or []
    )
    allowed_contract_versions = allowed_contract_versions or []
    unexpected_contract_versions = unexpected_contract_versions or []
    allowed_levels = allowed_levels or []
    unexpected_levels = unexpected_levels or []
    allowed_sources = allowed_sources or []
    unexpected_sources = unexpected_sources or []
    allowed_verification_scopes = allowed_verification_scopes or []
    unexpected_verification_scopes = unexpected_verification_scopes or []
    allowed_acceptance_profiles = allowed_acceptance_profiles or []
    unexpected_acceptance_profiles = unexpected_acceptance_profiles or []
    allowed_enabled_requirements = allowed_enabled_requirements or []
    unexpected_enabled_requirements = unexpected_enabled_requirements or []
    forbidden_sources = forbidden_sources or []
    present_forbidden_sources = present_forbidden_sources or []
    forbidden_verification_scopes = forbidden_verification_scopes or []
    present_forbidden_verification_scopes = (
        present_forbidden_verification_scopes or []
    )
    forbidden_acceptance_profiles = forbidden_acceptance_profiles or []
    present_forbidden_acceptance_profiles = (
        present_forbidden_acceptance_profiles or []
    )
    forbidden_enabled_requirements = forbidden_enabled_requirements or []
    present_forbidden_enabled_requirements = (
        present_forbidden_enabled_requirements or []
    )
    forbidden_contract_versions = forbidden_contract_versions or []
    present_forbidden_contract_versions = (
        present_forbidden_contract_versions or []
    )
    forbidden_levels = forbidden_levels or []
    present_forbidden_levels = present_forbidden_levels or []
    expected_reason_codes = expected_reason_codes or []
    missing_expected_reason_codes = missing_expected_reason_codes or []
    allowed_reason_codes = allowed_reason_codes or []
    unexpected_reason_codes = unexpected_reason_codes or []
    forbidden_reason_codes = forbidden_reason_codes or []
    present_forbidden_reason_codes = present_forbidden_reason_codes or []
    expected_skipped_reasons = expected_skipped_reasons or []
    missing_expected_skipped_reasons = missing_expected_skipped_reasons or []
    allowed_skipped_reasons = allowed_skipped_reasons or []
    unexpected_skipped_reasons = unexpected_skipped_reasons or []
    forbidden_skipped_reasons = forbidden_skipped_reasons or []
    present_forbidden_skipped_reasons = present_forbidden_skipped_reasons or []
    excluded_output_artifact_inputs = excluded_output_artifact_inputs or []
    expected_node_tree_gate_check_counts = expected_node_tree_gate_check_counts or {}
    expected_mechanical_gate_check_counts = (
        expected_mechanical_gate_check_counts or {}
    )
    expected_summary_counts = expected_summary_counts or {}
    mismatched_expected_summary_counts = mismatched_expected_summary_counts or {}
    expected_summary_values = expected_summary_values or {}
    expected_summary_value_sources = expected_summary_value_sources or []
    missing_expected_summary_value_sources = (
        missing_expected_summary_value_sources or []
    )
    summary_value_source_excluded_sources = (
        summary_value_source_excluded_sources or []
    )
    expected_summary_value_source_excluded_sources = (
        expected_summary_value_source_excluded_sources or []
    )
    missing_expected_summary_value_source_excluded_sources = (
        missing_expected_summary_value_source_excluded_sources or []
    )
    allowed_summary_value_source_excluded_sources = (
        allowed_summary_value_source_excluded_sources or []
    )
    unexpected_summary_value_source_excluded_sources = (
        unexpected_summary_value_source_excluded_sources or []
    )
    forbidden_summary_value_source_excluded_sources = (
        forbidden_summary_value_source_excluded_sources or []
    )
    present_forbidden_summary_value_source_excluded_sources = (
        present_forbidden_summary_value_source_excluded_sources or []
    )
    summary_value_source_matched_sources = summary_value_source_matched_sources or []
    allowed_summary_value_source_matched_sources = (
        allowed_summary_value_source_matched_sources or []
    )
    unexpected_summary_value_source_matched_sources = (
        unexpected_summary_value_source_matched_sources or []
    )
    forbidden_summary_value_source_matched_sources = (
        forbidden_summary_value_source_matched_sources or []
    )
    present_forbidden_summary_value_source_matched_sources = (
        present_forbidden_summary_value_source_matched_sources or []
    )
    mismatched_expected_summary_values = mismatched_expected_summary_values or {}
    mismatched_expected_node_tree_gate_check_counts = (
        mismatched_expected_node_tree_gate_check_counts or {}
    )
    mismatched_expected_mechanical_gate_check_counts = (
        mismatched_expected_mechanical_gate_check_counts or {}
    )
    node_tree_manifest_sidecar_path_map_mismatch_kind_counts = (
        node_tree_manifest_sidecar_path_map_mismatch_kind_counts or {}
    )
    expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts = (
        expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts or {}
    )
    status = "error" if error_count or aggregate_errors else "success"
    skipped_text = _summary_text_field(
        "skipped", skipped_count, enabled=skipped_count > 0
    )
    node_tree_manifest_sidecar_text = (
        "node_tree_sidecars="
        f"count:{node_tree_manifest_sidecar_count},"
        f"complete:{node_tree_manifest_sidecar_complete_count},"
        f"incomplete:{node_tree_manifest_sidecar_incomplete_count},"
        f"valid:{node_tree_manifest_sidecar_valid_count},"
        f"invalid:{node_tree_manifest_sidecar_invalid_count},"
        f"validation_errors:{node_tree_manifest_sidecar_validation_error_count},"
        f"path_incomplete:{node_tree_manifest_sidecar_path_incomplete_count},"
        f"path_mismatches:{node_tree_manifest_sidecar_path_map_mismatch_count},"
        f"path_mismatch_kinds:{_format_count_map(node_tree_manifest_sidecar_path_map_mismatch_kind_counts)},"
        f"parts:{node_tree_manifest_sidecar_parts_planned_count},"
        f"joints:{node_tree_manifest_sidecar_joints_planned_count},"
        f"part_paths:{node_tree_manifest_sidecar_part_path_count},"
        f"joint_paths:{node_tree_manifest_sidecar_joint_path_count} "
        if show_metadata or node_tree_manifest_sidecar_count > 0
        else ""
    )
    aggregate_errors_text = _summary_text_field(
        "aggregate_errors", aggregate_errors, enabled=bool(aggregate_errors)
    )
    has_expected_passed_counts = any(
        value is not None
        for value in (
            expected_passed_true_count,
            expected_passed_false_count,
            expected_passed_unknown_count,
        )
    )
    passed_counts_text = (
        f"passed=true:{passed_true_count},false:{passed_false_count},"
        f"unknown:{passed_unknown_count} "
        if show_passed_counts or has_expected_passed_counts
        else ""
    )
    full_mechanical_gate_counts_text = (
        "full_mechanical_gate="
        f"true:{requires_full_mechanical_restoration_gate_true_count},"
        f"false:{requires_full_mechanical_restoration_gate_false_count},"
        f"unknown:{requires_full_mechanical_restoration_gate_unknown_count} "
        if (
            show_metadata
            or require_full_mechanical_restoration_gate
            or show_full_mechanical_gate_counts
        )
        else ""
    )
    has_expected_smoke_report_counts = any(
        value is not None
        for value in (
            expected_smoke_report_written_count,
            expected_smoke_report_missing_count,
            expected_smoke_report_read_error_count,
        )
    )
    smoke_report_counts_text = (
        "smoke_reports="
        f"written:{smoke_report_written_count},"
        f"missing:{smoke_report_missing_count},"
        f"read_error:{smoke_report_read_error_count} "
        if show_metadata
        or show_smoke_report_counts
        or has_expected_smoke_report_counts
        else ""
    )
    has_expected_control_readback_counts = any(
        value is not None
        for value in (
            expected_control_configured_count,
            expected_control_readback_checked_count,
            expected_control_readback_missing_count,
        )
    )
    control_readback_counts_text = (
        "control_readback="
        f"configured:{control_configured_count},"
        f"checked:{control_readback_checked_count},"
        f"missing:{control_readback_missing_count} "
        if show_metadata
        or show_control_readback_counts
        or has_expected_control_readback_counts
        else ""
    )
    gate_check_counts_text = _summary_text_field(
        "checks",
        _format_gate_check_counts(
            {
                "node_tree_gate_check_counts": node_tree_gate_check_counts or {},
                "mechanical_gate_check_counts": mechanical_gate_check_counts or {},
            }
        ),
        enabled=show_metadata,
    )
    has_expected_gate_check_counts = bool(
        expected_node_tree_gate_check_counts
        or expected_mechanical_gate_check_counts
    )
    expected_gate_check_counts_text = _summary_text_field(
        "expected_checks",
        _format_gate_check_count_expectations(
            node_tree_gate_check_counts=expected_node_tree_gate_check_counts,
            mechanical_gate_check_counts=expected_mechanical_gate_check_counts,
        ),
        enabled=has_expected_gate_check_counts,
    )
    mismatched_expected_gate_check_counts_text = _summary_text_field(
        "mismatched_expected_checks",
        _format_gate_check_count_mismatches(
            node_tree_gate_check_counts=(
                mismatched_expected_node_tree_gate_check_counts
            ),
            mechanical_gate_check_counts=(
                mismatched_expected_mechanical_gate_check_counts
            ),
        ),
        enabled=has_expected_gate_check_counts,
    )
    expected_summary_counts_text = _summary_text_field(
        "expected_summary_counts",
        _format_expected_summary_counts(expected_summary_counts),
        enabled=bool(expected_summary_counts),
    )
    expected_path_mismatch_kind_counts_text = _summary_text_field(
        "expected_path_mismatch_kinds",
        _format_count_map(
            expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts
        ),
        enabled=bool(
            expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts
        ),
    )
    mismatched_expected_summary_counts_text = _summary_text_field(
        "mismatched_expected_summary_counts",
        _format_summary_count_mismatches(mismatched_expected_summary_counts),
        enabled=bool(expected_summary_counts),
    )
    expected_summary_values_text = _summary_text_field(
        "expected_summary_values",
        _format_expected_summary_values(expected_summary_values),
        enabled=bool(expected_summary_values),
    )
    expected_summary_value_sources_text = _summary_text_field(
        "expected_summary_value_sources",
        expected_summary_value_sources,
        enabled=bool(expected_summary_value_sources),
    )
    missing_expected_summary_value_sources_text = _summary_text_field(
        "missing_expected_summary_value_sources",
        missing_expected_summary_value_sources,
        enabled=bool(expected_summary_value_sources),
    )
    summary_value_source_matched_count_text = _summary_text_field(
        "summary_value_source_matched_count",
        summary_value_source_matched_count,
        enabled=bool(expected_summary_value_sources),
    )
    summary_value_source_matched_sources_text = _summary_text_field(
        "summary_value_source_matched_sources",
        summary_value_source_matched_sources,
        enabled=bool(expected_summary_value_sources)
        and bool(summary_value_source_matched_sources),
    )
    allowed_summary_value_source_matched_sources_text = _summary_text_field(
        "allowed_summary_value_source_matched_sources",
        allowed_summary_value_source_matched_sources,
        enabled=bool(allowed_summary_value_source_matched_sources),
    )
    unexpected_summary_value_source_matched_sources_text = _summary_text_field(
        "unexpected_summary_value_source_matched_sources",
        unexpected_summary_value_source_matched_sources,
        enabled=bool(allowed_summary_value_source_matched_sources),
    )
    forbidden_summary_value_source_matched_sources_text = _summary_text_field(
        "forbidden_summary_value_source_matched_sources",
        forbidden_summary_value_source_matched_sources,
        enabled=bool(forbidden_summary_value_source_matched_sources),
    )
    present_forbidden_summary_value_source_matched_sources_text = (
        _summary_text_field(
            "present_forbidden_summary_value_source_matched_sources",
            present_forbidden_summary_value_source_matched_sources,
            enabled=bool(forbidden_summary_value_source_matched_sources),
        )
    )
    summary_value_source_excluded_count_text = _summary_text_field(
        "summary_value_source_excluded_count",
        summary_value_source_excluded_count,
        enabled=bool(expected_summary_value_sources),
    )
    summary_value_source_excluded_sources_text = _summary_text_field(
        "summary_value_source_excluded_sources",
        summary_value_source_excluded_sources,
        enabled=bool(expected_summary_value_sources)
        and bool(summary_value_source_excluded_sources),
    )
    expected_summary_value_source_excluded_sources_text = _summary_text_field(
        "expected_summary_value_source_excluded_sources",
        expected_summary_value_source_excluded_sources,
        enabled=bool(expected_summary_value_source_excluded_sources),
    )
    missing_expected_summary_value_source_excluded_sources_text = (
        _summary_text_field(
            "missing_expected_summary_value_source_excluded_sources",
            missing_expected_summary_value_source_excluded_sources,
            enabled=bool(expected_summary_value_source_excluded_sources),
        )
    )
    allowed_summary_value_source_excluded_sources_text = _summary_text_field(
        "allowed_summary_value_source_excluded_sources",
        allowed_summary_value_source_excluded_sources,
        enabled=bool(allowed_summary_value_source_excluded_sources),
    )
    unexpected_summary_value_source_excluded_sources_text = _summary_text_field(
        "unexpected_summary_value_source_excluded_sources",
        unexpected_summary_value_source_excluded_sources,
        enabled=bool(allowed_summary_value_source_excluded_sources),
    )
    forbidden_summary_value_source_excluded_sources_text = _summary_text_field(
        "forbidden_summary_value_source_excluded_sources",
        forbidden_summary_value_source_excluded_sources,
        enabled=bool(forbidden_summary_value_source_excluded_sources),
    )
    present_forbidden_summary_value_source_excluded_sources_text = (
        _summary_text_field(
            "present_forbidden_summary_value_source_excluded_sources",
            present_forbidden_summary_value_source_excluded_sources,
            enabled=bool(forbidden_summary_value_source_excluded_sources),
        )
    )
    mismatched_expected_summary_values_text = _summary_text_field(
        "mismatched_expected_summary_values",
        _format_summary_value_mismatches(mismatched_expected_summary_values),
        enabled=bool(expected_summary_values),
    )
    input_paths_text = _summary_text_field(
        "input_paths", input_paths, enabled=show_inputs
    )
    expected_inputs_text = _summary_text_field(
        "expected_inputs", expected_inputs, enabled=bool(expected_inputs)
    )
    missing_expected_inputs_text = _summary_text_field(
        "missing_expected_inputs",
        missing_expected_inputs,
        enabled=bool(expected_inputs),
    )
    allowed_inputs_text = _summary_text_field(
        "allowed_inputs", allowed_inputs, enabled=bool(allowed_inputs)
    )
    unexpected_inputs_text = _summary_text_field(
        "unexpected_inputs", unexpected_inputs, enabled=bool(allowed_inputs)
    )
    forbidden_inputs_text = _summary_text_field(
        "forbidden_inputs", forbidden_inputs, enabled=bool(forbidden_inputs)
    )
    present_forbidden_inputs_text = _summary_text_field(
        "present_forbidden_inputs",
        present_forbidden_inputs,
        enabled=bool(forbidden_inputs),
    )
    contract_versions_text = _summary_text_field(
        "contract_versions",
        contract_versions,
        enabled=show_metadata,
    )
    levels_text = _summary_text_field(
        "levels",
        levels,
        enabled=show_metadata,
    )
    sources_text = _summary_text_field("sources", sources, enabled=show_metadata)
    scopes_text = _summary_text_field(
        "verification_scopes",
        verification_scopes,
        enabled=show_metadata,
    )
    profiles_text = _summary_text_field(
        "acceptance_profiles",
        acceptance_profiles,
        enabled=show_metadata,
    )
    requirements_text = _summary_text_field(
        "enabled_requirements",
        enabled_requirements,
        enabled=show_metadata,
    )
    complete_required_summary_fields_text = _summary_text_field(
        "complete_required_summary_fields",
        complete_required_summary_fields,
        enabled=show_metadata and bool(complete_required_summary_fields),
    )
    complete_required_summary_fields_source_scope_count_text = _summary_text_field(
        "complete_required_summary_fields_source_scope_count",
        complete_required_summary_fields_source_scope_count,
        enabled=show_metadata and bool(complete_required_summary_fields),
    )
    complete_required_summary_fields_source_scopes_text = _summary_text_field(
        "complete_required_summary_fields_source_scopes",
        complete_required_summary_fields_source_scopes,
        enabled=show_metadata and bool(complete_required_summary_fields_source_scopes),
    )
    reason_codes_text = _summary_text_field(
        "reason_codes",
        reason_codes,
        enabled=show_reason_codes,
    )
    affected_inputs_text = _summary_text_field(
        "affected_inputs",
        affected_inputs,
        enabled=show_inputs,
    )
    expected_affected_inputs_text = _summary_text_field(
        "expected_affected_inputs",
        expected_affected_inputs,
        enabled=bool(expected_affected_inputs),
    )
    missing_expected_affected_inputs_text = _summary_text_field(
        "missing_expected_affected_inputs",
        missing_expected_affected_inputs,
        enabled=bool(expected_affected_inputs),
    )
    allowed_affected_inputs_text = _summary_text_field(
        "allowed_affected_inputs",
        allowed_affected_inputs,
        enabled=bool(allowed_affected_inputs),
    )
    unexpected_affected_inputs_text = _summary_text_field(
        "unexpected_affected_inputs",
        unexpected_affected_inputs,
        enabled=bool(allowed_affected_inputs),
    )
    forbidden_affected_inputs_text = _summary_text_field(
        "forbidden_affected_inputs",
        forbidden_affected_inputs,
        enabled=bool(forbidden_affected_inputs),
    )
    present_forbidden_affected_inputs_text = _summary_text_field(
        "present_forbidden_affected_inputs",
        present_forbidden_affected_inputs,
        enabled=bool(forbidden_affected_inputs),
    )
    failed_inputs_text = _summary_text_field(
        "failed_inputs",
        failed_inputs,
        enabled=show_inputs,
    )
    expected_failed_inputs_text = _summary_text_field(
        "expected_failed_inputs",
        expected_failed_inputs,
        enabled=bool(expected_failed_inputs),
    )
    missing_expected_failed_inputs_text = _summary_text_field(
        "missing_expected_failed_inputs",
        missing_expected_failed_inputs,
        enabled=bool(expected_failed_inputs),
    )
    allowed_failed_inputs_text = _summary_text_field(
        "allowed_failed_inputs",
        allowed_failed_inputs,
        enabled=bool(allowed_failed_inputs),
    )
    unexpected_failed_inputs_text = _summary_text_field(
        "unexpected_failed_inputs",
        unexpected_failed_inputs,
        enabled=bool(allowed_failed_inputs),
    )
    forbidden_failed_inputs_text = _summary_text_field(
        "forbidden_failed_inputs",
        forbidden_failed_inputs,
        enabled=bool(forbidden_failed_inputs),
    )
    present_forbidden_failed_inputs_text = _summary_text_field(
        "present_forbidden_failed_inputs",
        present_forbidden_failed_inputs,
        enabled=bool(forbidden_failed_inputs),
    )
    passed_true_inputs_text = _summary_text_field(
        "passed_true_inputs",
        passed_true_inputs,
        enabled=show_inputs,
    )
    expected_passed_true_inputs_text = _summary_text_field(
        "expected_passed_true_inputs",
        expected_passed_true_inputs,
        enabled=bool(expected_passed_true_inputs),
    )
    missing_expected_passed_true_inputs_text = _summary_text_field(
        "missing_expected_passed_true_inputs",
        missing_expected_passed_true_inputs,
        enabled=bool(expected_passed_true_inputs),
    )
    allowed_passed_true_inputs_text = _summary_text_field(
        "allowed_passed_true_inputs",
        allowed_passed_true_inputs,
        enabled=bool(allowed_passed_true_inputs),
    )
    unexpected_passed_true_inputs_text = _summary_text_field(
        "unexpected_passed_true_inputs",
        unexpected_passed_true_inputs,
        enabled=bool(allowed_passed_true_inputs),
    )
    forbidden_passed_true_inputs_text = _summary_text_field(
        "forbidden_passed_true_inputs",
        forbidden_passed_true_inputs,
        enabled=bool(forbidden_passed_true_inputs),
    )
    present_forbidden_passed_true_inputs_text = _summary_text_field(
        "present_forbidden_passed_true_inputs",
        present_forbidden_passed_true_inputs,
        enabled=bool(forbidden_passed_true_inputs),
    )
    passed_false_inputs_text = _summary_text_field(
        "passed_false_inputs",
        passed_false_inputs,
        enabled=show_inputs,
    )
    expected_passed_false_inputs_text = _summary_text_field(
        "expected_passed_false_inputs",
        expected_passed_false_inputs,
        enabled=bool(expected_passed_false_inputs),
    )
    missing_expected_passed_false_inputs_text = _summary_text_field(
        "missing_expected_passed_false_inputs",
        missing_expected_passed_false_inputs,
        enabled=bool(expected_passed_false_inputs),
    )
    allowed_passed_false_inputs_text = _summary_text_field(
        "allowed_passed_false_inputs",
        allowed_passed_false_inputs,
        enabled=bool(allowed_passed_false_inputs),
    )
    unexpected_passed_false_inputs_text = _summary_text_field(
        "unexpected_passed_false_inputs",
        unexpected_passed_false_inputs,
        enabled=bool(allowed_passed_false_inputs),
    )
    forbidden_passed_false_inputs_text = _summary_text_field(
        "forbidden_passed_false_inputs",
        forbidden_passed_false_inputs,
        enabled=bool(forbidden_passed_false_inputs),
    )
    present_forbidden_passed_false_inputs_text = _summary_text_field(
        "present_forbidden_passed_false_inputs",
        present_forbidden_passed_false_inputs,
        enabled=bool(forbidden_passed_false_inputs),
    )
    passed_unknown_inputs_text = _summary_text_field(
        "passed_unknown_inputs",
        passed_unknown_inputs,
        enabled=show_inputs,
    )
    expected_passed_unknown_inputs_text = _summary_text_field(
        "expected_passed_unknown_inputs",
        expected_passed_unknown_inputs,
        enabled=bool(expected_passed_unknown_inputs),
    )
    missing_expected_passed_unknown_inputs_text = _summary_text_field(
        "missing_expected_passed_unknown_inputs",
        missing_expected_passed_unknown_inputs,
        enabled=bool(expected_passed_unknown_inputs),
    )
    allowed_passed_unknown_inputs_text = _summary_text_field(
        "allowed_passed_unknown_inputs",
        allowed_passed_unknown_inputs,
        enabled=bool(allowed_passed_unknown_inputs),
    )
    unexpected_passed_unknown_inputs_text = _summary_text_field(
        "unexpected_passed_unknown_inputs",
        unexpected_passed_unknown_inputs,
        enabled=bool(allowed_passed_unknown_inputs),
    )
    forbidden_passed_unknown_inputs_text = _summary_text_field(
        "forbidden_passed_unknown_inputs",
        forbidden_passed_unknown_inputs,
        enabled=bool(forbidden_passed_unknown_inputs),
    )
    present_forbidden_passed_unknown_inputs_text = _summary_text_field(
        "present_forbidden_passed_unknown_inputs",
        present_forbidden_passed_unknown_inputs,
        enabled=bool(forbidden_passed_unknown_inputs),
    )
    full_mechanical_gate_true_inputs_text = _summary_text_field(
        "full_mechanical_gate_true_inputs",
        full_mechanical_gate_true_inputs,
        enabled=show_inputs,
    )
    expected_full_mechanical_gate_true_inputs_text = _summary_text_field(
        "expected_full_mechanical_gate_true_inputs",
        expected_full_mechanical_gate_true_inputs,
        enabled=bool(expected_full_mechanical_gate_true_inputs),
    )
    missing_expected_full_mechanical_gate_true_inputs_text = (
        _summary_text_field(
            "missing_expected_full_mechanical_gate_true_inputs",
            missing_expected_full_mechanical_gate_true_inputs,
            enabled=bool(expected_full_mechanical_gate_true_inputs),
        )
    )
    allowed_full_mechanical_gate_true_inputs_text = _summary_text_field(
        "allowed_full_mechanical_gate_true_inputs",
        allowed_full_mechanical_gate_true_inputs,
        enabled=bool(allowed_full_mechanical_gate_true_inputs),
    )
    unexpected_full_mechanical_gate_true_inputs_text = _summary_text_field(
        "unexpected_full_mechanical_gate_true_inputs",
        unexpected_full_mechanical_gate_true_inputs,
        enabled=bool(allowed_full_mechanical_gate_true_inputs),
    )
    forbidden_full_mechanical_gate_true_inputs_text = _summary_text_field(
        "forbidden_full_mechanical_gate_true_inputs",
        forbidden_full_mechanical_gate_true_inputs,
        enabled=bool(forbidden_full_mechanical_gate_true_inputs),
    )
    present_forbidden_full_mechanical_gate_true_inputs_text = (
        _summary_text_field(
            "present_forbidden_full_mechanical_gate_true_inputs",
            present_forbidden_full_mechanical_gate_true_inputs,
            enabled=bool(forbidden_full_mechanical_gate_true_inputs),
        )
    )
    full_mechanical_gate_false_inputs_text = _summary_text_field(
        "full_mechanical_gate_false_inputs",
        full_mechanical_gate_false_inputs,
        enabled=show_inputs,
    )
    expected_full_mechanical_gate_false_inputs_text = _summary_text_field(
        "expected_full_mechanical_gate_false_inputs",
        expected_full_mechanical_gate_false_inputs,
        enabled=bool(expected_full_mechanical_gate_false_inputs),
    )
    missing_expected_full_mechanical_gate_false_inputs_text = (
        _summary_text_field(
            "missing_expected_full_mechanical_gate_false_inputs",
            missing_expected_full_mechanical_gate_false_inputs,
            enabled=bool(expected_full_mechanical_gate_false_inputs),
        )
    )
    allowed_full_mechanical_gate_false_inputs_text = _summary_text_field(
        "allowed_full_mechanical_gate_false_inputs",
        allowed_full_mechanical_gate_false_inputs,
        enabled=bool(allowed_full_mechanical_gate_false_inputs),
    )
    unexpected_full_mechanical_gate_false_inputs_text = _summary_text_field(
        "unexpected_full_mechanical_gate_false_inputs",
        unexpected_full_mechanical_gate_false_inputs,
        enabled=bool(allowed_full_mechanical_gate_false_inputs),
    )
    forbidden_full_mechanical_gate_false_inputs_text = _summary_text_field(
        "forbidden_full_mechanical_gate_false_inputs",
        forbidden_full_mechanical_gate_false_inputs,
        enabled=bool(forbidden_full_mechanical_gate_false_inputs),
    )
    present_forbidden_full_mechanical_gate_false_inputs_text = (
        _summary_text_field(
            "present_forbidden_full_mechanical_gate_false_inputs",
            present_forbidden_full_mechanical_gate_false_inputs,
            enabled=bool(forbidden_full_mechanical_gate_false_inputs),
        )
    )
    full_mechanical_gate_unknown_inputs_text = _summary_text_field(
        "full_mechanical_gate_unknown_inputs",
        full_mechanical_gate_unknown_inputs,
        enabled=show_inputs,
    )
    expected_full_mechanical_gate_unknown_inputs_text = _summary_text_field(
        "expected_full_mechanical_gate_unknown_inputs",
        expected_full_mechanical_gate_unknown_inputs,
        enabled=bool(expected_full_mechanical_gate_unknown_inputs),
    )
    missing_expected_full_mechanical_gate_unknown_inputs_text = (
        _summary_text_field(
            "missing_expected_full_mechanical_gate_unknown_inputs",
            missing_expected_full_mechanical_gate_unknown_inputs,
            enabled=bool(expected_full_mechanical_gate_unknown_inputs),
        )
    )
    allowed_full_mechanical_gate_unknown_inputs_text = _summary_text_field(
        "allowed_full_mechanical_gate_unknown_inputs",
        allowed_full_mechanical_gate_unknown_inputs,
        enabled=bool(allowed_full_mechanical_gate_unknown_inputs),
    )
    unexpected_full_mechanical_gate_unknown_inputs_text = _summary_text_field(
        "unexpected_full_mechanical_gate_unknown_inputs",
        unexpected_full_mechanical_gate_unknown_inputs,
        enabled=bool(allowed_full_mechanical_gate_unknown_inputs),
    )
    forbidden_full_mechanical_gate_unknown_inputs_text = _summary_text_field(
        "forbidden_full_mechanical_gate_unknown_inputs",
        forbidden_full_mechanical_gate_unknown_inputs,
        enabled=bool(forbidden_full_mechanical_gate_unknown_inputs),
    )
    present_forbidden_full_mechanical_gate_unknown_inputs_text = (
        _summary_text_field(
            "present_forbidden_full_mechanical_gate_unknown_inputs",
            present_forbidden_full_mechanical_gate_unknown_inputs,
            enabled=bool(forbidden_full_mechanical_gate_unknown_inputs),
        )
    )
    skipped_inputs_text = _summary_text_field(
        "skipped_inputs",
        skipped_inputs,
        enabled=show_inputs,
    )
    expected_skipped_inputs_text = _summary_text_field(
        "expected_skipped_inputs",
        expected_skipped_inputs,
        enabled=bool(expected_skipped_inputs),
    )
    missing_expected_skipped_inputs_text = _summary_text_field(
        "missing_expected_skipped_inputs",
        missing_expected_skipped_inputs,
        enabled=bool(expected_skipped_inputs),
    )
    allowed_skipped_inputs_text = _summary_text_field(
        "allowed_skipped_inputs",
        allowed_skipped_inputs,
        enabled=bool(allowed_skipped_inputs),
    )
    unexpected_skipped_inputs_text = _summary_text_field(
        "unexpected_skipped_inputs",
        unexpected_skipped_inputs,
        enabled=bool(allowed_skipped_inputs),
    )
    forbidden_skipped_inputs_text = _summary_text_field(
        "forbidden_skipped_inputs",
        forbidden_skipped_inputs,
        enabled=bool(forbidden_skipped_inputs),
    )
    present_forbidden_skipped_inputs_text = _summary_text_field(
        "present_forbidden_skipped_inputs",
        present_forbidden_skipped_inputs,
        enabled=bool(forbidden_skipped_inputs),
    )
    skipped_reasons_text = _summary_text_field(
        "skipped_reasons",
        skipped_reasons,
        enabled=show_skipped_reasons,
    )
    truncated_previews_text = _summary_text_field(
        "truncated_previews",
        truncated_previews,
        enabled=bool(truncated_previews),
    )
    truncated_previews_count_text = _summary_text_field(
        "truncated_previews_count",
        len(truncated_previews),
        enabled=bool(truncated_previews)
        or expected_truncated_previews_count is not None,
    )
    expected_truncated_previews_text = _summary_text_field(
        "expected_truncated_previews",
        expected_truncated_previews,
        enabled=bool(expected_truncated_previews),
    )
    missing_expected_truncated_previews_text = _summary_text_field(
        "missing_expected_truncated_previews",
        missing_expected_truncated_previews,
        enabled=bool(expected_truncated_previews),
    )
    allowed_truncated_previews_text = _summary_text_field(
        "allowed_truncated_previews",
        allowed_truncated_previews,
        enabled=bool(allowed_truncated_previews),
    )
    unexpected_truncated_previews_text = _summary_text_field(
        "unexpected_truncated_previews",
        unexpected_truncated_previews,
        enabled=bool(allowed_truncated_previews),
    )
    forbidden_truncated_previews_text = _summary_text_field(
        "forbidden_truncated_previews",
        forbidden_truncated_previews,
        enabled=bool(forbidden_truncated_previews),
    )
    present_forbidden_truncated_previews_text = _summary_text_field(
        "present_forbidden_truncated_previews",
        present_forbidden_truncated_previews,
        enabled=bool(forbidden_truncated_previews),
    )
    expected_counts_text = "".join(
        _summary_text_field(field_name, field_value)
        for field_name, field_value in (
            ("expected_inputs_count", expected_inputs_count),
            ("expected_expanded_inputs_count", expected_expanded_inputs_count),
            (
                "expected_excluded_output_artifacts_count",
                expected_excluded_output_artifacts_count,
            ),
            ("expected_success_count", expected_success_count),
            ("expected_error_count", expected_error_count),
            ("expected_passed_true_count", expected_passed_true_count),
            ("expected_passed_false_count", expected_passed_false_count),
            ("expected_passed_unknown_count", expected_passed_unknown_count),
            (
                "expected_full_mechanical_gate_true_count",
                expected_full_mechanical_gate_true_count,
            ),
            (
                "expected_full_mechanical_gate_false_count",
                expected_full_mechanical_gate_false_count,
            ),
            (
                "expected_full_mechanical_gate_unknown_count",
                expected_full_mechanical_gate_unknown_count,
            ),
            (
                "expected_smoke_report_written_count",
                expected_smoke_report_written_count,
            ),
            (
                "expected_smoke_report_missing_count",
                expected_smoke_report_missing_count,
            ),
            (
                "expected_smoke_report_read_error_count",
                expected_smoke_report_read_error_count,
            ),
            (
                "expected_control_configured_count",
                expected_control_configured_count,
            ),
            (
                "expected_control_readback_checked_count",
                expected_control_readback_checked_count,
            ),
            (
                "expected_control_readback_missing_count",
                expected_control_readback_missing_count,
            ),
            (
                "expected_node_tree_manifest_sidecar_count",
                expected_node_tree_manifest_sidecar_count,
            ),
            (
                "expected_node_tree_manifest_sidecar_complete_count",
                expected_node_tree_manifest_sidecar_complete_count,
            ),
            (
                "expected_node_tree_manifest_sidecar_incomplete_count",
                expected_node_tree_manifest_sidecar_incomplete_count,
            ),
            (
                "expected_node_tree_manifest_sidecar_valid_count",
                expected_node_tree_manifest_sidecar_valid_count,
            ),
            (
                "expected_node_tree_manifest_sidecar_invalid_count",
                expected_node_tree_manifest_sidecar_invalid_count,
            ),
            (
                "expected_node_tree_manifest_sidecar_validation_error_count",
                expected_node_tree_manifest_sidecar_validation_error_count,
            ),
            (
                "expected_node_tree_manifest_sidecar_path_incomplete_count",
                expected_node_tree_manifest_sidecar_path_incomplete_count,
            ),
            (
                "expected_node_tree_manifest_sidecar_path_map_mismatch_count",
                expected_node_tree_manifest_sidecar_path_map_mismatch_count,
            ),
            (
                "expected_node_tree_manifest_sidecar_parts_planned_count",
                expected_node_tree_manifest_sidecar_parts_planned_count,
            ),
            (
                "expected_node_tree_manifest_sidecar_joints_planned_count",
                expected_node_tree_manifest_sidecar_joints_planned_count,
            ),
            (
                "expected_node_tree_manifest_sidecar_part_path_count",
                expected_node_tree_manifest_sidecar_part_path_count,
            ),
            (
                "expected_node_tree_manifest_sidecar_joint_path_count",
                expected_node_tree_manifest_sidecar_joint_path_count,
            ),
            (
                "expected_summary_value_source_matched_count",
                expected_summary_value_source_matched_count,
            ),
            (
                "expected_summary_value_source_excluded_count",
                expected_summary_value_source_excluded_count,
            ),
            ("expected_skipped_count", expected_skipped_count),
            ("expected_reason_codes_count", expected_reason_codes_count),
            (
                "expected_contract_versions_count",
                expected_contract_versions_count,
            ),
            ("expected_levels_count", expected_levels_count),
            ("expected_sources_count", expected_sources_count),
            (
                "expected_verification_scopes_count",
                expected_verification_scopes_count,
            ),
            (
                "expected_acceptance_profiles_count",
                expected_acceptance_profiles_count,
            ),
            (
                "expected_enabled_requirements_count",
                expected_enabled_requirements_count,
            ),
            (
                "expected_complete_required_summary_fields_source_scope_count",
                expected_complete_required_summary_fields_source_scope_count,
            ),
            ("expected_affected_inputs_count", expected_affected_inputs_count),
            ("expected_failed_inputs_count", expected_failed_inputs_count),
            ("expected_skipped_inputs_count", expected_skipped_inputs_count),
            ("expected_skipped_reasons_count", expected_skipped_reasons_count),
            (
                "expected_truncated_previews_count",
                expected_truncated_previews_count,
            ),
        )
    )
    expected_reason_codes_text = _summary_text_field(
        "expected_reason_codes",
        expected_reason_codes,
        enabled=bool(expected_reason_codes),
    )
    missing_expected_reason_codes_text = _summary_text_field(
        "missing_expected_reason_codes",
        missing_expected_reason_codes,
        enabled=bool(expected_reason_codes),
    )
    expected_contract_versions_text = _summary_text_field(
        "expected_contract_versions",
        expected_contract_versions,
        enabled=bool(expected_contract_versions),
    )
    missing_expected_contract_versions_text = _summary_text_field(
        "missing_expected_contract_versions",
        missing_expected_contract_versions,
        enabled=bool(expected_contract_versions),
    )
    expected_levels_text = _summary_text_field(
        "expected_levels",
        expected_levels,
        enabled=bool(expected_levels),
    )
    missing_expected_levels_text = _summary_text_field(
        "missing_expected_levels",
        missing_expected_levels,
        enabled=bool(expected_levels),
    )
    expected_sources_text = _summary_text_field(
        "expected_sources", expected_sources, enabled=bool(expected_sources)
    )
    missing_expected_sources_text = _summary_text_field(
        "missing_expected_sources",
        missing_expected_sources,
        enabled=bool(expected_sources),
    )
    expected_scopes_text = _summary_text_field(
        "expected_verification_scopes",
        expected_verification_scopes,
        enabled=bool(expected_verification_scopes),
    )
    missing_expected_scopes_text = _summary_text_field(
        "missing_expected_verification_scopes",
        missing_expected_verification_scopes,
        enabled=bool(expected_verification_scopes),
    )
    expected_profiles_text = _summary_text_field(
        "expected_acceptance_profiles",
        expected_acceptance_profiles,
        enabled=bool(expected_acceptance_profiles),
    )
    missing_expected_profiles_text = _summary_text_field(
        "missing_expected_acceptance_profiles",
        missing_expected_acceptance_profiles,
        enabled=bool(expected_acceptance_profiles),
    )
    expected_requirements_text = _summary_text_field(
        "expected_enabled_requirements",
        expected_enabled_requirements,
        enabled=bool(expected_enabled_requirements),
    )
    missing_expected_requirements_text = _summary_text_field(
        "missing_expected_enabled_requirements",
        missing_expected_enabled_requirements,
        enabled=bool(expected_enabled_requirements),
    )
    expected_complete_required_summary_fields_source_scopes_text = (
        _summary_text_field(
            "expected_complete_required_summary_fields_source_scopes",
            expected_complete_required_summary_fields_source_scopes,
            enabled=bool(expected_complete_required_summary_fields_source_scopes),
        )
    )
    missing_expected_complete_required_summary_fields_source_scopes_text = (
        _summary_text_field(
            "missing_expected_complete_required_summary_fields_source_scopes",
            missing_expected_complete_required_summary_fields_source_scopes,
            enabled=bool(expected_complete_required_summary_fields_source_scopes),
        )
    )
    allowed_complete_required_summary_fields_source_scopes_text = (
        _summary_text_field(
            "allowed_complete_required_summary_fields_source_scopes",
            allowed_complete_required_summary_fields_source_scopes,
            enabled=bool(allowed_complete_required_summary_fields_source_scopes),
        )
    )
    unexpected_complete_required_summary_fields_source_scopes_text = (
        _summary_text_field(
            "unexpected_complete_required_summary_fields_source_scopes",
            unexpected_complete_required_summary_fields_source_scopes,
            enabled=bool(allowed_complete_required_summary_fields_source_scopes),
        )
    )
    forbidden_complete_required_summary_fields_source_scopes_text = (
        _summary_text_field(
            "forbidden_complete_required_summary_fields_source_scopes",
            forbidden_complete_required_summary_fields_source_scopes,
            enabled=bool(forbidden_complete_required_summary_fields_source_scopes),
        )
    )
    present_forbidden_complete_required_summary_fields_source_scopes_text = (
        _summary_text_field(
            "present_forbidden_complete_required_summary_fields_source_scopes",
            present_forbidden_complete_required_summary_fields_source_scopes,
            enabled=bool(forbidden_complete_required_summary_fields_source_scopes),
        )
    )
    allowed_sources_text = _summary_text_field(
        "allowed_sources", allowed_sources, enabled=bool(allowed_sources)
    )
    unexpected_sources_text = _summary_text_field(
        "unexpected_sources",
        unexpected_sources,
        enabled=bool(allowed_sources),
    )
    allowed_scopes_text = _summary_text_field(
        "allowed_verification_scopes",
        allowed_verification_scopes,
        enabled=bool(allowed_verification_scopes),
    )
    unexpected_scopes_text = _summary_text_field(
        "unexpected_verification_scopes",
        unexpected_verification_scopes,
        enabled=bool(allowed_verification_scopes),
    )
    allowed_profiles_text = _summary_text_field(
        "allowed_acceptance_profiles",
        allowed_acceptance_profiles,
        enabled=bool(allowed_acceptance_profiles),
    )
    unexpected_profiles_text = _summary_text_field(
        "unexpected_acceptance_profiles",
        unexpected_acceptance_profiles,
        enabled=bool(allowed_acceptance_profiles),
    )
    allowed_requirements_text = _summary_text_field(
        "allowed_enabled_requirements",
        allowed_enabled_requirements,
        enabled=bool(allowed_enabled_requirements),
    )
    unexpected_requirements_text = _summary_text_field(
        "unexpected_enabled_requirements",
        unexpected_enabled_requirements,
        enabled=bool(allowed_enabled_requirements),
    )
    allowed_contract_versions_text = _summary_text_field(
        "allowed_contract_versions",
        allowed_contract_versions,
        enabled=bool(allowed_contract_versions),
    )
    unexpected_contract_versions_text = _summary_text_field(
        "unexpected_contract_versions",
        unexpected_contract_versions,
        enabled=bool(allowed_contract_versions),
    )
    allowed_levels_text = _summary_text_field(
        "allowed_levels",
        allowed_levels,
        enabled=bool(allowed_levels),
    )
    unexpected_levels_text = _summary_text_field(
        "unexpected_levels",
        unexpected_levels,
        enabled=bool(allowed_levels),
    )
    forbidden_sources_text = _summary_text_field(
        "forbidden_sources", forbidden_sources, enabled=bool(forbidden_sources)
    )
    present_forbidden_sources_text = _summary_text_field(
        "present_forbidden_sources",
        present_forbidden_sources,
        enabled=bool(forbidden_sources),
    )
    forbidden_scopes_text = _summary_text_field(
        "forbidden_verification_scopes",
        forbidden_verification_scopes,
        enabled=bool(forbidden_verification_scopes),
    )
    present_forbidden_scopes_text = _summary_text_field(
        "present_forbidden_verification_scopes",
        present_forbidden_verification_scopes,
        enabled=bool(forbidden_verification_scopes),
    )
    forbidden_profiles_text = _summary_text_field(
        "forbidden_acceptance_profiles",
        forbidden_acceptance_profiles,
        enabled=bool(forbidden_acceptance_profiles),
    )
    present_forbidden_profiles_text = _summary_text_field(
        "present_forbidden_acceptance_profiles",
        present_forbidden_acceptance_profiles,
        enabled=bool(forbidden_acceptance_profiles),
    )
    forbidden_requirements_text = _summary_text_field(
        "forbidden_enabled_requirements",
        forbidden_enabled_requirements,
        enabled=bool(forbidden_enabled_requirements),
    )
    present_forbidden_requirements_text = _summary_text_field(
        "present_forbidden_enabled_requirements",
        present_forbidden_enabled_requirements,
        enabled=bool(forbidden_enabled_requirements),
    )
    forbidden_contract_versions_text = _summary_text_field(
        "forbidden_contract_versions",
        forbidden_contract_versions,
        enabled=bool(forbidden_contract_versions),
    )
    present_forbidden_contract_versions_text = _summary_text_field(
        "present_forbidden_contract_versions",
        present_forbidden_contract_versions,
        enabled=bool(forbidden_contract_versions),
    )
    forbidden_levels_text = _summary_text_field(
        "forbidden_levels",
        forbidden_levels,
        enabled=bool(forbidden_levels),
    )
    present_forbidden_levels_text = _summary_text_field(
        "present_forbidden_levels",
        present_forbidden_levels,
        enabled=bool(forbidden_levels),
    )
    allowed_reason_codes_text = _summary_text_field(
        "allowed_reason_codes",
        allowed_reason_codes,
        enabled=bool(allowed_reason_codes),
    )
    unexpected_reason_codes_text = _summary_text_field(
        "unexpected_reason_codes",
        unexpected_reason_codes,
        enabled=bool(allowed_reason_codes),
    )
    forbidden_reason_codes_text = _summary_text_field(
        "forbidden_reason_codes",
        forbidden_reason_codes,
        enabled=bool(forbidden_reason_codes),
    )
    present_forbidden_reason_codes_text = _summary_text_field(
        "present_forbidden_reason_codes",
        present_forbidden_reason_codes,
        enabled=bool(forbidden_reason_codes),
    )
    expected_skipped_reasons_text = _summary_text_field(
        "expected_skipped_reasons",
        expected_skipped_reasons,
        enabled=bool(expected_skipped_reasons),
    )
    missing_expected_skipped_reasons_text = _summary_text_field(
        "missing_expected_skipped_reasons",
        missing_expected_skipped_reasons,
        enabled=bool(expected_skipped_reasons),
    )
    allowed_skipped_reasons_text = _summary_text_field(
        "allowed_skipped_reasons",
        allowed_skipped_reasons,
        enabled=bool(allowed_skipped_reasons),
    )
    unexpected_skipped_reasons_text = _summary_text_field(
        "unexpected_skipped_reasons",
        unexpected_skipped_reasons,
        enabled=bool(allowed_skipped_reasons),
    )
    forbidden_skipped_reasons_text = _summary_text_field(
        "forbidden_skipped_reasons",
        forbidden_skipped_reasons,
        enabled=bool(forbidden_skipped_reasons),
    )
    present_forbidden_skipped_reasons_text = _summary_text_field(
        "present_forbidden_skipped_reasons",
        present_forbidden_skipped_reasons,
        enabled=bool(forbidden_skipped_reasons),
    )
    excluded_output_artifacts_text = _summary_text_field(
        "excluded_output_artifacts",
        len(excluded_output_artifact_inputs),
        enabled=bool(excluded_output_artifact_inputs),
    )
    expanded_inputs_text = _summary_text_field(
        "expanded_inputs",
        expanded_inputs_count,
        enabled=bool(excluded_output_artifact_inputs),
    )
    preview_limit_text = _summary_text_field(
        "preview_limit",
        preview_limit,
        enabled=preview_limit is not None,
    )
    fail_on_full_mechanical_gate_false_text = _summary_text_field(
        "fail_on_full_mechanical_gate_false",
        str(fail_on_full_mechanical_gate_false).lower(),
        enabled=fail_on_full_mechanical_gate_false,
    )
    fail_on_full_mechanical_gate_unknown_text = _summary_text_field(
        "fail_on_full_mechanical_gate_unknown",
        str(fail_on_full_mechanical_gate_unknown).lower(),
        enabled=fail_on_full_mechanical_gate_unknown,
    )
    fail_on_smoke_report_missing_text = _summary_text_field(
        "fail_on_smoke_report_missing",
        str(fail_on_smoke_report_missing).lower(),
        enabled=fail_on_smoke_report_missing,
    )
    fail_on_smoke_report_read_error_text = _summary_text_field(
        "fail_on_smoke_report_read_error",
        str(fail_on_smoke_report_read_error).lower(),
        enabled=fail_on_smoke_report_read_error,
    )
    fail_on_control_readback_missing_text = _summary_text_field(
        "fail_on_control_readback_missing",
        str(fail_on_control_readback_missing).lower(),
        enabled=fail_on_control_readback_missing,
    )
    fail_on_node_tree_manifest_sidecar_incomplete_text = _summary_text_field(
        "fail_on_node_tree_manifest_sidecar_incomplete",
        str(fail_on_node_tree_manifest_sidecar_incomplete).lower(),
        enabled=fail_on_node_tree_manifest_sidecar_incomplete,
    )
    fail_on_invalid_node_tree_manifest_sidecar_text = _summary_text_field(
        "fail_on_invalid_node_tree_manifest_sidecar",
        str(fail_on_invalid_node_tree_manifest_sidecar).lower(),
        enabled=fail_on_invalid_node_tree_manifest_sidecar,
    )
    fail_on_node_tree_manifest_sidecar_validation_error_text = _summary_text_field(
        "fail_on_node_tree_manifest_sidecar_validation_error",
        str(fail_on_node_tree_manifest_sidecar_validation_error).lower(),
        enabled=fail_on_node_tree_manifest_sidecar_validation_error,
    )
    fail_on_node_tree_manifest_sidecar_path_incomplete_text = _summary_text_field(
        "fail_on_node_tree_manifest_sidecar_path_incomplete",
        str(fail_on_node_tree_manifest_sidecar_path_incomplete).lower(),
        enabled=fail_on_node_tree_manifest_sidecar_path_incomplete,
    )
    fail_on_node_tree_manifest_sidecar_path_map_mismatch_text = _summary_text_field(
        "fail_on_node_tree_manifest_sidecar_path_map_mismatch",
        str(fail_on_node_tree_manifest_sidecar_path_map_mismatch).lower(),
        enabled=fail_on_node_tree_manifest_sidecar_path_map_mismatch,
    )
    return (
        "delivery_acceptance_gate validation summary "
        f"status={status} "
        f"{preview_limit_text}"
        f"{expanded_inputs_text}"
        f"inputs={inputs_count} "
        f"success={success_count} "
        f"{skipped_text}"
        f"{node_tree_manifest_sidecar_text}"
        f"errors={error_count} "
        f"{passed_counts_text}"
        f"{full_mechanical_gate_counts_text}"
        f"{smoke_report_counts_text}"
        f"{control_readback_counts_text}"
        f"{gate_check_counts_text}"
        f"{expected_gate_check_counts_text}"
        f"{mismatched_expected_gate_check_counts_text}"
        f"{expected_summary_counts_text}"
        f"{expected_path_mismatch_kind_counts_text}"
        f"{mismatched_expected_summary_counts_text}"
        f"{expected_summary_values_text}"
        f"{expected_summary_value_sources_text}"
        f"{missing_expected_summary_value_sources_text}"
        f"{summary_value_source_matched_count_text}"
        f"{summary_value_source_matched_sources_text}"
        f"{allowed_summary_value_source_matched_sources_text}"
        f"{unexpected_summary_value_source_matched_sources_text}"
        f"{forbidden_summary_value_source_matched_sources_text}"
        f"{present_forbidden_summary_value_source_matched_sources_text}"
        f"{summary_value_source_excluded_count_text}"
        f"{summary_value_source_excluded_sources_text}"
        f"{expected_summary_value_source_excluded_sources_text}"
        f"{missing_expected_summary_value_source_excluded_sources_text}"
        f"{allowed_summary_value_source_excluded_sources_text}"
        f"{unexpected_summary_value_source_excluded_sources_text}"
        f"{forbidden_summary_value_source_excluded_sources_text}"
        f"{present_forbidden_summary_value_source_excluded_sources_text}"
        f"{mismatched_expected_summary_values_text}"
        f"{input_paths_text}"
        f"{expected_inputs_text}"
        f"{missing_expected_inputs_text}"
        f"{allowed_inputs_text}"
        f"{unexpected_inputs_text}"
        f"{forbidden_inputs_text}"
        f"{present_forbidden_inputs_text}"
        f"{contract_versions_text}"
        f"{levels_text}"
        f"{sources_text}"
        f"{scopes_text}"
        f"{profiles_text}"
        f"{requirements_text}"
        f"{complete_required_summary_fields_text}"
        f"{complete_required_summary_fields_source_scope_count_text}"
        f"{complete_required_summary_fields_source_scopes_text}"
        f"{reason_codes_text}"
        f"{affected_inputs_text}"
        f"{expected_affected_inputs_text}"
        f"{missing_expected_affected_inputs_text}"
        f"{allowed_affected_inputs_text}"
        f"{unexpected_affected_inputs_text}"
        f"{forbidden_affected_inputs_text}"
        f"{present_forbidden_affected_inputs_text}"
        f"{failed_inputs_text}"
        f"{expected_failed_inputs_text}"
        f"{missing_expected_failed_inputs_text}"
        f"{allowed_failed_inputs_text}"
        f"{unexpected_failed_inputs_text}"
        f"{forbidden_failed_inputs_text}"
        f"{present_forbidden_failed_inputs_text}"
        f"{passed_true_inputs_text}"
        f"{expected_passed_true_inputs_text}"
        f"{missing_expected_passed_true_inputs_text}"
        f"{allowed_passed_true_inputs_text}"
        f"{unexpected_passed_true_inputs_text}"
        f"{forbidden_passed_true_inputs_text}"
        f"{present_forbidden_passed_true_inputs_text}"
        f"{passed_false_inputs_text}"
        f"{expected_passed_false_inputs_text}"
        f"{missing_expected_passed_false_inputs_text}"
        f"{allowed_passed_false_inputs_text}"
        f"{unexpected_passed_false_inputs_text}"
        f"{forbidden_passed_false_inputs_text}"
        f"{present_forbidden_passed_false_inputs_text}"
        f"{passed_unknown_inputs_text}"
        f"{expected_passed_unknown_inputs_text}"
        f"{missing_expected_passed_unknown_inputs_text}"
        f"{allowed_passed_unknown_inputs_text}"
        f"{unexpected_passed_unknown_inputs_text}"
        f"{forbidden_passed_unknown_inputs_text}"
        f"{present_forbidden_passed_unknown_inputs_text}"
        f"{full_mechanical_gate_true_inputs_text}"
        f"{expected_full_mechanical_gate_true_inputs_text}"
        f"{missing_expected_full_mechanical_gate_true_inputs_text}"
        f"{allowed_full_mechanical_gate_true_inputs_text}"
        f"{unexpected_full_mechanical_gate_true_inputs_text}"
        f"{forbidden_full_mechanical_gate_true_inputs_text}"
        f"{present_forbidden_full_mechanical_gate_true_inputs_text}"
        f"{full_mechanical_gate_false_inputs_text}"
        f"{expected_full_mechanical_gate_false_inputs_text}"
        f"{missing_expected_full_mechanical_gate_false_inputs_text}"
        f"{allowed_full_mechanical_gate_false_inputs_text}"
        f"{unexpected_full_mechanical_gate_false_inputs_text}"
        f"{forbidden_full_mechanical_gate_false_inputs_text}"
        f"{present_forbidden_full_mechanical_gate_false_inputs_text}"
        f"{full_mechanical_gate_unknown_inputs_text}"
        f"{expected_full_mechanical_gate_unknown_inputs_text}"
        f"{missing_expected_full_mechanical_gate_unknown_inputs_text}"
        f"{allowed_full_mechanical_gate_unknown_inputs_text}"
        f"{unexpected_full_mechanical_gate_unknown_inputs_text}"
        f"{forbidden_full_mechanical_gate_unknown_inputs_text}"
        f"{present_forbidden_full_mechanical_gate_unknown_inputs_text}"
        f"{skipped_inputs_text}"
        f"{expected_skipped_inputs_text}"
        f"{missing_expected_skipped_inputs_text}"
        f"{allowed_skipped_inputs_text}"
        f"{unexpected_skipped_inputs_text}"
        f"{forbidden_skipped_inputs_text}"
        f"{present_forbidden_skipped_inputs_text}"
        f"{skipped_reasons_text}"
        f"{truncated_previews_count_text}"
        f"{truncated_previews_text}"
        f"{expected_truncated_previews_text}"
        f"{missing_expected_truncated_previews_text}"
        f"{allowed_truncated_previews_text}"
        f"{unexpected_truncated_previews_text}"
        f"{forbidden_truncated_previews_text}"
        f"{present_forbidden_truncated_previews_text}"
        f"{aggregate_errors_text}"
        f"{expected_counts_text}"
        f"{expected_reason_codes_text}"
        f"{missing_expected_reason_codes_text}"
        f"{expected_contract_versions_text}"
        f"{missing_expected_contract_versions_text}"
        f"{expected_levels_text}"
        f"{missing_expected_levels_text}"
        f"{expected_sources_text}"
        f"{missing_expected_sources_text}"
        f"{expected_scopes_text}"
        f"{missing_expected_scopes_text}"
        f"{expected_profiles_text}"
        f"{missing_expected_profiles_text}"
        f"{expected_requirements_text}"
        f"{missing_expected_requirements_text}"
        f"{expected_complete_required_summary_fields_source_scopes_text}"
        f"{missing_expected_complete_required_summary_fields_source_scopes_text}"
        f"{allowed_complete_required_summary_fields_source_scopes_text}"
        f"{unexpected_complete_required_summary_fields_source_scopes_text}"
        f"{forbidden_complete_required_summary_fields_source_scopes_text}"
        f"{present_forbidden_complete_required_summary_fields_source_scopes_text}"
        f"{allowed_sources_text}"
        f"{unexpected_sources_text}"
        f"{allowed_scopes_text}"
        f"{unexpected_scopes_text}"
        f"{allowed_profiles_text}"
        f"{unexpected_profiles_text}"
        f"{allowed_requirements_text}"
        f"{unexpected_requirements_text}"
        f"{allowed_contract_versions_text}"
        f"{unexpected_contract_versions_text}"
        f"{allowed_levels_text}"
        f"{unexpected_levels_text}"
        f"{forbidden_sources_text}"
        f"{present_forbidden_sources_text}"
        f"{forbidden_scopes_text}"
        f"{present_forbidden_scopes_text}"
        f"{forbidden_profiles_text}"
        f"{present_forbidden_profiles_text}"
        f"{forbidden_requirements_text}"
        f"{present_forbidden_requirements_text}"
        f"{forbidden_contract_versions_text}"
        f"{present_forbidden_contract_versions_text}"
        f"{forbidden_levels_text}"
        f"{present_forbidden_levels_text}"
        f"{allowed_reason_codes_text}"
        f"{unexpected_reason_codes_text}"
        f"{forbidden_reason_codes_text}"
        f"{present_forbidden_reason_codes_text}"
        f"{expected_skipped_reasons_text}"
        f"{missing_expected_skipped_reasons_text}"
        f"{allowed_skipped_reasons_text}"
        f"{unexpected_skipped_reasons_text}"
        f"{forbidden_skipped_reasons_text}"
        f"{present_forbidden_skipped_reasons_text}"
        f"{excluded_output_artifacts_text}"
        f"{fail_on_full_mechanical_gate_false_text}"
        f"{fail_on_full_mechanical_gate_unknown_text}"
        f"{fail_on_smoke_report_missing_text}"
        f"{fail_on_smoke_report_read_error_text}"
        f"{fail_on_control_readback_missing_text}"
        f"{fail_on_node_tree_manifest_sidecar_incomplete_text}"
        f"{fail_on_invalid_node_tree_manifest_sidecar_text}"
        f"{fail_on_node_tree_manifest_sidecar_validation_error_text}"
        f"{fail_on_node_tree_manifest_sidecar_path_incomplete_text}"
        f"{fail_on_node_tree_manifest_sidecar_path_map_mismatch_text}"
        f"require_passed={str(require_passed).lower()}"
    )


def _expand_input_paths(input_items: list[str], *, recursive: bool) -> list[Path]:
    paths: list[Path] = []
    for item in input_items:
        path = Path(item)
        if path.is_dir():
            iterator = path.rglob("*.json") if recursive else path.glob("*.json")
            paths.extend(sorted(child for child in iterator if child.is_file()))
        else:
            paths.append(path)
    return paths


def _reject_explicit_output_inputs(
    parser: argparse.ArgumentParser,
    input_items: list[str],
    output_paths: list[Path],
) -> None:
    for item in input_items:
        path = Path(item)
        if path.is_dir():
            continue
        if _path_in_list(path, output_paths):
            parser.error(
                "--output and --summary-output paths cannot also be explicit inputs"
            )


def _exclude_output_artifact_inputs(
    input_paths: list[Path],
    output_paths: list[Path],
) -> list[Path]:
    if not output_paths:
        return input_paths
    return [path for path in input_paths if not _path_in_list(path, output_paths)]


def _output_artifact_inputs(
    input_paths: list[Path],
    output_paths: list[Path],
) -> list[str]:
    if not output_paths:
        return []
    return _unique_texts(
        [str(path) for path in output_paths if _path_in_list(path, input_paths)]
    )


def _input_error_result(
    *,
    input_path: Path,
    require_passed: bool,
    require_required: bool,
    require_complete: bool,
    require_full_mechanical_restoration_gate: bool,
    error: str,
) -> dict[str, Any]:
    return {
        "status": "error",
        "input": str(input_path),
        "gate_source_path": None,
        "contract_version": None,
        "source": None,
        "verification_scope": None,
        "acceptance_profile": None,
        "level": None,
        "required": None,
        "complete": None,
        "requires_full_mechanical_restoration_gate": None,
        "passed": None,
        "exit_code": None,
        "reason_codes": [],
        "reasons_count": 0,
        "enabled_requirements": [],
        "affected_inputs": [],
        "affected_inputs_count": 0,
        "affected_inputs_truncated": False,
        "summary_counts": {},
        "complete_required_summary_fields": [],
        "complete_required_summary_fields_count": 0,
        "require_passed": bool(require_passed),
        "require_required": bool(require_required),
        "require_complete": bool(require_complete),
        "require_full_mechanical_restoration_gate": bool(
            require_full_mechanical_restoration_gate
        ),
        "errors": [error],
    }


def _unique_limited(values: list[Any], *, limit: int = 20) -> tuple[list[str], int]:
    unique_values: list[str] = []
    for value in values:
        text = str(value)
        if text not in unique_values:
            unique_values.append(text)
    return unique_values[:limit], len(unique_values)


def _unique_texts(values: list[Any]) -> list[str]:
    return _unique_limited(values, limit=len(values))[0]


def _expected_count_errors(
    expectations: list[tuple[str, int | None, int]],
) -> list[str]:
    return [
        f"expected {field_name} {expected_count} but found {actual_count}"
        for field_name, expected_count, actual_count in expectations
        if expected_count is not None and actual_count != expected_count
    ]


def _expected_map_count_errors(
    field_name: str,
    expected_counts: dict[str, int],
    actual_counts: dict[str, int],
) -> list[str]:
    return [
        f"expected {field_name}.{key} {expected_count} but found {actual_counts.get(key, 0)}"
        for key, expected_count in sorted(expected_counts.items())
        if actual_counts.get(key, 0) != expected_count
    ]


def _mismatched_expected_map_counts(
    expected_counts: dict[str, int],
    actual_counts: dict[str, int],
) -> dict[str, dict[str, int]]:
    return {
        key: {"expected": expected_count, "actual": actual_counts.get(key, 0)}
        for key, expected_count in sorted(expected_counts.items())
        if actual_counts.get(key, 0) != expected_count
    }


def _selected_summary_count_totals(
    results: list[dict[str, Any]],
    expected_counts: dict[str, int],
) -> dict[str, int]:
    return {
        key: _summary_counts_total(results, key)
        for key in sorted(expected_counts)
    }


def _selected_summary_value_totals(
    results: list[dict[str, Any]],
    expected_values: dict[str, int],
    source_filter: list[str] | None = None,
) -> dict[str, int]:
    if source_filter:
        source_values = set(source_filter)
        results = [
            result
            for result in results
            if result.get("source") in source_values
        ]
    totals: dict[str, int] = {}
    map_totals: dict[str, dict[str, int]] = {}
    for path in sorted(expected_values):
        parts = path.split(".", 1)
        if len(parts) == 1:
            totals[path] = _summary_counts_total(results, path)
            continue
        map_key, count_key = parts
        if map_key not in map_totals:
            map_totals[map_key] = _summary_counts_map_totals(results, map_key)
        totals[path] = map_totals[map_key].get(count_key, 0)
    return totals


def _missing_expected_values(expected: list[str], actual: list[str]) -> list[str]:
    return [value for value in expected if value not in actual]


def _unexpected_values(actual: list[str], allowed: list[str] | None) -> list[str]:
    if allowed is None:
        return []
    return [value for value in actual if value not in allowed]


def _present_forbidden_values(
    actual: list[str],
    forbidden: list[str] | None,
) -> list[str]:
    return [value for value in actual if value in (forbidden or [])]


def _expected_value_errors(
    label: str,
    expected: list[str],
    actual: list[str],
) -> list[str]:
    return [
        f"expected {label} {value} was not found"
        for value in _missing_expected_values(expected, actual)
    ]


def _unexpected_value_errors(
    label: str,
    actual: list[str],
    allowed: list[str] | None,
) -> list[str]:
    return [
        f"unexpected {label} {value} was found"
        for value in _unexpected_values(actual, allowed)
    ]


def _forbidden_value_errors(
    label: str,
    actual: list[str],
    forbidden: list[str] | None,
    *,
    order_by_forbidden: bool = False,
) -> list[str]:
    values = (
        [value for value in (forbidden or []) if value in actual]
        if order_by_forbidden
        else _present_forbidden_values(actual, forbidden)
    )
    return [
        f"forbidden {label} {value} was found"
        for value in values
    ]


def _partition_passed_results(
    results: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    return (
        [result for result in results if result.get("passed") is True],
        [result for result in results if result.get("passed") is False],
        [
            result
            for result in results
            if not isinstance(result.get("passed"), bool)
        ],
    )


def _build_summary_payload(
    *,
    results: list[dict[str, Any]],
    aggregate_errors: list[str],
    require_passed: bool,
    summary_version: str = "delivery_acceptance_gate_validation_summary.v1",
    require_required: bool = False,
    require_complete: bool = False,
    require_full_mechanical_restoration_gate: bool = False,
    fail_on_full_mechanical_gate_false: bool = False,
    fail_on_full_mechanical_gate_unknown: bool = False,
    fail_on_smoke_report_missing: bool = False,
    fail_on_smoke_report_read_error: bool = False,
    fail_on_control_readback_missing: bool = False,
    fail_on_node_tree_manifest_sidecar_incomplete: bool = False,
    fail_on_invalid_node_tree_manifest_sidecar: bool = False,
    fail_on_node_tree_manifest_sidecar_validation_error: bool = False,
    fail_on_node_tree_manifest_sidecar_path_incomplete: bool = False,
    fail_on_node_tree_manifest_sidecar_path_map_mismatch: bool = False,
    preview_limit: int = 20,
    expanded_inputs_count: int | None = None,
    expected_expanded_inputs_count: int | None = None,
    expected_excluded_output_artifacts_count: int | None = None,
    expected_inputs_count: int | None = None,
    expected_success_count: int | None = None,
    expected_error_count: int | None = None,
    expected_passed_true_count: int | None = None,
    expected_passed_false_count: int | None = None,
    expected_passed_unknown_count: int | None = None,
    expected_full_mechanical_gate_true_count: int | None = None,
    expected_full_mechanical_gate_false_count: int | None = None,
    expected_full_mechanical_gate_unknown_count: int | None = None,
    expected_smoke_report_written_count: int | None = None,
    expected_smoke_report_missing_count: int | None = None,
    expected_smoke_report_read_error_count: int | None = None,
    expected_control_configured_count: int | None = None,
    expected_control_readback_checked_count: int | None = None,
    expected_control_readback_missing_count: int | None = None,
    expected_node_tree_manifest_sidecar_count: int | None = None,
    expected_node_tree_manifest_sidecar_complete_count: int | None = None,
    expected_node_tree_manifest_sidecar_incomplete_count: int | None = None,
    expected_node_tree_manifest_sidecar_valid_count: int | None = None,
    expected_node_tree_manifest_sidecar_invalid_count: int | None = None,
    expected_node_tree_manifest_sidecar_validation_error_count: int | None = None,
    expected_node_tree_manifest_sidecar_path_incomplete_count: int | None = None,
    expected_node_tree_manifest_sidecar_path_map_mismatch_count: int | None = None,
    expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts: (
        dict[str, int] | None
    ) = None,
    expected_node_tree_manifest_sidecar_parts_planned_count: int | None = None,
    expected_node_tree_manifest_sidecar_joints_planned_count: int | None = None,
    expected_node_tree_manifest_sidecar_part_path_count: int | None = None,
    expected_node_tree_manifest_sidecar_joint_path_count: int | None = None,
    expected_node_tree_gate_check_counts: dict[str, int] | None = None,
    expected_mechanical_gate_check_counts: dict[str, int] | None = None,
    expected_summary_counts: dict[str, int] | None = None,
    expected_summary_values: dict[str, int] | None = None,
    expected_summary_value_sources: list[str] | None = None,
    expected_summary_value_source_matched_count: int | None = None,
    expected_summary_value_source_excluded_count: int | None = None,
    expected_summary_value_source_excluded_sources: list[str] | None = None,
    allowed_summary_value_source_matched_sources: list[str] | None = None,
    forbidden_summary_value_source_matched_sources: list[str] | None = None,
    allowed_summary_value_source_excluded_sources: list[str] | None = None,
    forbidden_summary_value_source_excluded_sources: list[str] | None = None,
    expected_skipped_count: int | None = None,
    expected_reason_codes_count: int | None = None,
    expected_contract_versions_count: int | None = None,
    expected_summary_versions_count: int | None = None,
    expected_validation_summary_statuses_count: int | None = None,
    expected_levels_count: int | None = None,
    expected_sources_count: int | None = None,
    expected_verification_scopes_count: int | None = None,
    expected_acceptance_profiles_count: int | None = None,
    expected_enabled_requirements_count: int | None = None,
    expected_complete_required_summary_fields_source_scope_count: int | None = None,
    expected_affected_inputs_count: int | None = None,
    expected_failed_inputs_count: int | None = None,
    expected_skipped_inputs_count: int | None = None,
    expected_skipped_reasons_count: int | None = None,
    expected_truncated_previews_count: int | None = None,
    expected_contract_versions: list[str] | None = None,
    expected_levels: list[str] | None = None,
    expected_sources: list[str] | None = None,
    expected_verification_scopes: list[str] | None = None,
    expected_acceptance_profiles: list[str] | None = None,
    expected_enabled_requirements: list[str] | None = None,
    expected_complete_required_summary_fields_source_scopes: list[str] | None = None,
    allowed_complete_required_summary_fields_source_scopes: list[str] | None = None,
    forbidden_complete_required_summary_fields_source_scopes: list[str] | None = None,
    expected_reason_codes: list[str] | None = None,
    expected_inputs: list[str] | None = None,
    expected_affected_inputs: list[str] | None = None,
    expected_failed_inputs: list[str] | None = None,
    expected_passed_true_inputs: list[str] | None = None,
    expected_passed_false_inputs: list[str] | None = None,
    expected_passed_unknown_inputs: list[str] | None = None,
    expected_full_mechanical_gate_true_inputs: list[str] | None = None,
    expected_full_mechanical_gate_false_inputs: list[str] | None = None,
    expected_full_mechanical_gate_unknown_inputs: list[str] | None = None,
    expected_truncated_previews: list[str] | None = None,
    expected_skipped_inputs: list[str] | None = None,
    allowed_contract_versions: list[str] | None = None,
    expected_summary_versions: list[str] | None = None,
    expected_validation_summary_statuses: list[str] | None = None,
    allowed_summary_versions: list[str] | None = None,
    allowed_validation_summary_statuses: list[str] | None = None,
    allowed_levels: list[str] | None = None,
    allowed_sources: list[str] | None = None,
    allowed_verification_scopes: list[str] | None = None,
    allowed_acceptance_profiles: list[str] | None = None,
    allowed_enabled_requirements: list[str] | None = None,
    allowed_reason_codes: list[str] | None = None,
    allowed_inputs: list[str] | None = None,
    allowed_affected_inputs: list[str] | None = None,
    allowed_failed_inputs: list[str] | None = None,
    allowed_passed_true_inputs: list[str] | None = None,
    allowed_passed_false_inputs: list[str] | None = None,
    allowed_passed_unknown_inputs: list[str] | None = None,
    allowed_full_mechanical_gate_true_inputs: list[str] | None = None,
    allowed_full_mechanical_gate_false_inputs: list[str] | None = None,
    allowed_full_mechanical_gate_unknown_inputs: list[str] | None = None,
    allowed_truncated_previews: list[str] | None = None,
    allowed_skipped_inputs: list[str] | None = None,
    expected_skipped_reasons: list[str] | None = None,
    allowed_skipped_reasons: list[str] | None = None,
    forbidden_contract_versions: list[str] | None = None,
    forbidden_levels: list[str] | None = None,
    forbidden_sources: list[str] | None = None,
    forbidden_verification_scopes: list[str] | None = None,
    forbidden_acceptance_profiles: list[str] | None = None,
    forbidden_enabled_requirements: list[str] | None = None,
    forbidden_reason_codes: list[str] | None = None,
    forbidden_inputs: list[str] | None = None,
    forbidden_affected_inputs: list[str] | None = None,
    forbidden_failed_inputs: list[str] | None = None,
    forbidden_passed_true_inputs: list[str] | None = None,
    forbidden_passed_false_inputs: list[str] | None = None,
    forbidden_passed_unknown_inputs: list[str] | None = None,
    forbidden_full_mechanical_gate_true_inputs: list[str] | None = None,
    forbidden_full_mechanical_gate_false_inputs: list[str] | None = None,
    forbidden_full_mechanical_gate_unknown_inputs: list[str] | None = None,
    forbidden_truncated_previews: list[str] | None = None,
    forbidden_skipped_inputs: list[str] | None = None,
    forbidden_skipped_reasons: list[str] | None = None,
    forbidden_summary_versions: list[str] | None = None,
    forbidden_validation_summary_statuses: list[str] | None = None,
    excluded_output_artifact_inputs: list[str] | None = None,
) -> dict[str, Any]:
    failed = [result for result in results if result["status"] == "error"]
    skipped = [result for result in results if result["status"] == "skipped"]
    successes = [result for result in results if result["status"] == "success"]
    passed_true, passed_false, passed_unknown = _partition_passed_results(results)
    full_mechanical_gate_true = [
        result
        for result in results
        if result.get("requires_full_mechanical_restoration_gate") is True
    ]
    full_mechanical_gate_false = [
        result
        for result in results
        if result.get("requires_full_mechanical_restoration_gate") is False
    ]
    full_mechanical_gate_unknown = [
        result
        for result in results
        if result.get("requires_full_mechanical_restoration_gate") is not True
        and result.get("requires_full_mechanical_restoration_gate") is not False
    ]
    all_full_mechanical_gate_true_inputs = _unique_texts(
        [result.get("input") for result in full_mechanical_gate_true]
    )
    full_mechanical_gate_true_inputs = (
        all_full_mechanical_gate_true_inputs[:preview_limit]
    )
    full_mechanical_gate_true_inputs_count = len(
        all_full_mechanical_gate_true_inputs
    )
    all_full_mechanical_gate_false_inputs = _unique_texts(
        [result.get("input") for result in full_mechanical_gate_false]
    )
    full_mechanical_gate_false_inputs = (
        all_full_mechanical_gate_false_inputs[:preview_limit]
    )
    full_mechanical_gate_false_inputs_count = len(
        all_full_mechanical_gate_false_inputs
    )
    all_full_mechanical_gate_unknown_inputs = _unique_texts(
        [result.get("input") for result in full_mechanical_gate_unknown]
    )
    full_mechanical_gate_unknown_inputs = (
        all_full_mechanical_gate_unknown_inputs[:preview_limit]
    )
    full_mechanical_gate_unknown_inputs_count = len(
        all_full_mechanical_gate_unknown_inputs
    )
    smoke_report_written_count = _summary_counts_total(
        results, "smoke_report_written_count"
    )
    smoke_report_missing_count = _summary_counts_total(
        results, "smoke_report_missing_count"
    )
    smoke_report_read_error_count = _summary_counts_total(
        results, "smoke_report_read_error_count"
    )
    control_configured_count = _summary_counts_total(
        results, "control_configured_count"
    )
    control_readback_checked_count = _summary_counts_total(
        results, "control_readback_checked_count"
    )
    control_readback_missing_count = _summary_counts_total(
        results, "control_readback_missing_count"
    )
    node_tree_gate_check_counts = _summary_counts_map_totals(
        results,
        "node_tree_gate_check_counts",
    )
    mechanical_gate_check_counts = _summary_counts_map_totals(
        results,
        "mechanical_gate_check_counts",
    )
    expected_summary_counts = expected_summary_counts or {}
    actual_expected_summary_counts = _selected_summary_count_totals(
        results,
        expected_summary_counts,
    )
    mismatched_expected_summary_counts = _mismatched_expected_map_counts(
        expected_summary_counts,
        actual_expected_summary_counts,
    )
    expected_summary_values = expected_summary_values or {}
    expected_summary_value_sources = expected_summary_value_sources or []
    actual_expected_summary_values = _selected_summary_value_totals(
        results,
        expected_summary_values,
        expected_summary_value_sources,
    )
    mismatched_expected_summary_values = _mismatched_expected_map_counts(
        expected_summary_values,
        actual_expected_summary_values,
    )
    complete_required_summary_fields_by_source_scope = (
        _complete_required_summary_fields_by_source_scope(results)
    )
    complete_required_summary_fields_source_scopes = _source_scope_values(
        complete_required_summary_fields_by_source_scope
    )
    complete_required_summary_fields_source_scope_count = sum(
        len(scopes)
        for scopes in complete_required_summary_fields_by_source_scope.values()
    )
    all_inputs = _unique_texts([result.get("input") for result in results])
    input_paths = all_inputs[:preview_limit]
    input_paths_count = len(all_inputs)
    all_contract_versions = _unique_texts(
        [
            result.get("contract_version")
            for result in results
            if result.get("contract_version")
        ]
    )
    contract_versions = all_contract_versions[:preview_limit]
    contract_versions_count = len(all_contract_versions)
    all_summary_versions = _unique_texts(
        [
            result.get("summary_version")
            for result in results
            if result.get("summary_version")
        ]
    )
    summary_versions = all_summary_versions[:preview_limit]
    summary_versions_count = len(all_summary_versions)
    all_validation_summary_statuses = _unique_texts(
        [
            result.get("validation_summary_status")
            for result in results
            if result.get("validation_summary_status")
        ]
    )
    validation_summary_statuses = all_validation_summary_statuses[:preview_limit]
    validation_summary_statuses_count = len(all_validation_summary_statuses)
    all_levels = _unique_texts(
        [result.get("level") for result in results if result.get("level")]
    )
    levels = all_levels[:preview_limit]
    levels_count = len(all_levels)
    all_reason_codes = _unique_texts(
        [
            reason_code
            for result in results
            for reason_code in result.get("reason_codes", [])
        ]
    )
    reason_codes = all_reason_codes[:preview_limit]
    reason_codes_count = len(all_reason_codes)
    all_sources = _unique_texts(
        [result.get("source") for result in results if result.get("source")]
    )
    summary_value_source_matched_results = [
        result
        for result in results
        if expected_summary_value_sources
        and result.get("source") in set(expected_summary_value_sources)
    ]
    summary_value_source_excluded_results = [
        result
        for result in results
        if expected_summary_value_sources
        and result.get("source") not in set(expected_summary_value_sources)
    ]
    summary_value_source_matched_count = (
        len(summary_value_source_matched_results)
        if expected_summary_value_sources
        else None
    )
    summary_value_source_excluded_count = (
        len(summary_value_source_excluded_results)
        if expected_summary_value_sources
        else None
    )
    summary_value_source_excluded_sources = _unique_texts(
        [
            result.get("source")
            for result in summary_value_source_excluded_results
            if result.get("source")
        ]
    )
    summary_value_source_matched_sources = _unique_texts(
        [
            result.get("source")
            for result in summary_value_source_matched_results
            if result.get("source")
        ]
    )
    unexpected_summary_value_source_matched_sources = _unexpected_values(
        summary_value_source_matched_sources,
        allowed_summary_value_source_matched_sources,
    )
    present_forbidden_summary_value_source_matched_sources = (
        _present_forbidden_values(
            summary_value_source_matched_sources,
            forbidden_summary_value_source_matched_sources,
        )
    )
    missing_expected_summary_value_source_excluded_sources = (
        _missing_expected_values(
            expected_summary_value_source_excluded_sources or [],
            summary_value_source_excluded_sources,
        )
    )
    unexpected_summary_value_source_excluded_sources = _unexpected_values(
        summary_value_source_excluded_sources,
        allowed_summary_value_source_excluded_sources,
    )
    present_forbidden_summary_value_source_excluded_sources = (
        _present_forbidden_values(
            summary_value_source_excluded_sources,
            forbidden_summary_value_source_excluded_sources,
        )
    )
    missing_expected_summary_value_sources = _missing_expected_values(
        expected_summary_value_sources,
        all_sources,
    )
    sources = all_sources[:preview_limit]
    sources_count = len(all_sources)
    all_verification_scopes = _unique_texts(
        [
            result.get("verification_scope")
            for result in results
            if result.get("verification_scope")
        ]
    )
    verification_scopes = all_verification_scopes[:preview_limit]
    verification_scopes_count = len(all_verification_scopes)
    all_acceptance_profiles = _unique_texts(
        [
            result.get("acceptance_profile")
            for result in results
            if result.get("acceptance_profile")
        ]
    )
    acceptance_profiles = all_acceptance_profiles[:preview_limit]
    acceptance_profiles_count = len(all_acceptance_profiles)
    all_enabled_requirements = _unique_texts(
        [
            requirement
            for result in results
            for requirement in result.get("enabled_requirements", [])
        ]
    )
    enabled_requirements = all_enabled_requirements[:preview_limit]
    enabled_requirements_count = len(all_enabled_requirements)
    missing_expected_contract_versions = _missing_expected_values(
        expected_contract_versions or [], all_contract_versions
    )
    missing_expected_summary_versions = _missing_expected_values(
        expected_summary_versions or [], all_summary_versions
    )
    missing_expected_validation_summary_statuses = _missing_expected_values(
        expected_validation_summary_statuses or [], all_validation_summary_statuses
    )
    missing_expected_levels = _missing_expected_values(
        expected_levels or [], all_levels
    )
    missing_expected_sources = _missing_expected_values(
        expected_sources or [], all_sources
    )
    missing_expected_verification_scopes = _missing_expected_values(
        expected_verification_scopes or [], all_verification_scopes
    )
    missing_expected_acceptance_profiles = _missing_expected_values(
        expected_acceptance_profiles or [], all_acceptance_profiles
    )
    missing_expected_enabled_requirements = _missing_expected_values(
        expected_enabled_requirements or [], all_enabled_requirements
    )
    missing_expected_complete_required_summary_fields_source_scopes = (
        _missing_expected_values(
            expected_complete_required_summary_fields_source_scopes or [],
            complete_required_summary_fields_source_scopes,
        )
    )
    unexpected_contract_versions = _unexpected_values(
        all_contract_versions, allowed_contract_versions
    )
    unexpected_summary_versions = _unexpected_values(
        all_summary_versions, allowed_summary_versions
    )
    unexpected_validation_summary_statuses = _unexpected_values(
        all_validation_summary_statuses, allowed_validation_summary_statuses
    )
    unexpected_levels = _unexpected_values(all_levels, allowed_levels)
    unexpected_sources = _unexpected_values(all_sources, allowed_sources)
    unexpected_verification_scopes = _unexpected_values(
        all_verification_scopes, allowed_verification_scopes
    )
    unexpected_acceptance_profiles = _unexpected_values(
        all_acceptance_profiles, allowed_acceptance_profiles
    )
    unexpected_enabled_requirements = _unexpected_values(
        all_enabled_requirements, allowed_enabled_requirements
    )
    unexpected_complete_required_summary_fields_source_scopes = (
        _unexpected_values(
            complete_required_summary_fields_source_scopes,
            allowed_complete_required_summary_fields_source_scopes,
        )
    )
    present_forbidden_complete_required_summary_fields_source_scopes = (
        _present_forbidden_values(
            complete_required_summary_fields_source_scopes,
            forbidden_complete_required_summary_fields_source_scopes,
        )
    )
    present_forbidden_contract_versions = _present_forbidden_values(
        all_contract_versions, forbidden_contract_versions
    )
    present_forbidden_summary_versions = _present_forbidden_values(
        all_summary_versions, forbidden_summary_versions
    )
    present_forbidden_validation_summary_statuses = _present_forbidden_values(
        all_validation_summary_statuses, forbidden_validation_summary_statuses
    )
    present_forbidden_levels = _present_forbidden_values(
        all_levels, forbidden_levels
    )
    present_forbidden_sources = _present_forbidden_values(
        all_sources, forbidden_sources
    )
    present_forbidden_verification_scopes = _present_forbidden_values(
        all_verification_scopes, forbidden_verification_scopes
    )
    present_forbidden_acceptance_profiles = _present_forbidden_values(
        all_acceptance_profiles, forbidden_acceptance_profiles
    )
    present_forbidden_enabled_requirements = _present_forbidden_values(
        all_enabled_requirements, forbidden_enabled_requirements
    )
    missing_expected_inputs = _missing_expected_values(
        expected_inputs or [], all_inputs
    )
    unexpected_inputs = _unexpected_values(all_inputs, allowed_inputs)
    present_forbidden_inputs = _present_forbidden_values(
        all_inputs, forbidden_inputs
    )
    all_affected_inputs = _unique_texts(
        [
            input_path
            for result in results
            for input_path in result.get("affected_inputs", [])
        ]
    )
    affected_inputs = all_affected_inputs[:preview_limit]
    affected_inputs_count = len(all_affected_inputs)
    all_failed_inputs = _unique_texts([result.get("input") for result in failed])
    failed_inputs = all_failed_inputs[:preview_limit]
    failed_inputs_count = len(all_failed_inputs)
    all_passed_true_inputs = _unique_texts(
        [result.get("input") for result in passed_true]
    )
    passed_true_inputs = all_passed_true_inputs[:preview_limit]
    passed_true_inputs_count = len(all_passed_true_inputs)
    all_passed_false_inputs = _unique_texts(
        [result.get("input") for result in passed_false]
    )
    passed_false_inputs = all_passed_false_inputs[:preview_limit]
    passed_false_inputs_count = len(all_passed_false_inputs)
    all_passed_unknown_inputs = _unique_texts(
        [result.get("input") for result in passed_unknown]
    )
    passed_unknown_inputs = all_passed_unknown_inputs[:preview_limit]
    passed_unknown_inputs_count = len(all_passed_unknown_inputs)
    all_skipped_inputs = _unique_texts([result.get("input") for result in skipped])
    skipped_inputs = all_skipped_inputs[:preview_limit]
    skipped_inputs_count = len(all_skipped_inputs)
    all_skipped_reasons = _unique_texts(
        [result.get("skip_reason") for result in skipped if result.get("skip_reason")]
    )
    skipped_reasons = all_skipped_reasons[:preview_limit]
    skipped_reasons_count = len(all_skipped_reasons)
    all_node_tree_manifest_sidecars = [
        {
            **result["node_tree_manifest_summary"],
            "input": result.get("input"),
            "node_tree_manifest_valid": result.get("node_tree_manifest_valid"),
            "node_tree_manifest_validation_error_count": len(
                result.get("node_tree_manifest_validation_errors", [])
            )
            if isinstance(result.get("node_tree_manifest_validation_errors"), list)
            else 0,
            "node_tree_manifest_validation_errors": result.get(
                "node_tree_manifest_validation_errors", []
            ),
            "node_tree_manifest_path_map_mismatch_count": result.get(
                "node_tree_manifest_path_map_mismatch_count", 0
            ),
            "node_tree_manifest_path_map_mismatches": result.get(
                "node_tree_manifest_path_map_mismatches", []
            )[:preview_limit],
            "node_tree_manifest_path_map_mismatch_kind_counts": (
                _node_tree_manifest_path_map_mismatch_kind_counts(
                    result.get("node_tree_manifest_path_map_mismatches", [])
                )
            ),
        }
        for result in skipped
        if isinstance(result.get("node_tree_manifest_summary"), dict)
    ]
    node_tree_manifest_sidecars = all_node_tree_manifest_sidecars[:preview_limit]
    node_tree_manifest_sidecar_count = len(all_node_tree_manifest_sidecars)
    node_tree_manifest_sidecar_complete_count = sum(
        1
        for summary in all_node_tree_manifest_sidecars
        if summary.get("complete") is True
    )
    node_tree_manifest_sidecar_incomplete_count = sum(
        1
        for summary in all_node_tree_manifest_sidecars
        if summary.get("complete") is False
    )
    node_tree_manifest_sidecar_valid_count = sum(
        1
        for result in skipped
        if isinstance(result.get("node_tree_manifest_summary"), dict)
        and result.get("node_tree_manifest_valid") is True
    )
    node_tree_manifest_sidecar_invalid_count = sum(
        1
        for result in skipped
        if isinstance(result.get("node_tree_manifest_summary"), dict)
        and result.get("node_tree_manifest_valid") is False
    )
    node_tree_manifest_sidecar_validation_error_count = sum(
        int(summary.get("node_tree_manifest_validation_error_count") or 0)
        for summary in all_node_tree_manifest_sidecars
    )
    node_tree_manifest_sidecar_path_incomplete_count = sum(
        1
        for summary in all_node_tree_manifest_sidecars
        if _node_tree_manifest_summary_has_incomplete_paths(summary)
    )
    node_tree_manifest_sidecar_path_map_mismatch_count = sum(
        int(summary.get("node_tree_manifest_path_map_mismatch_count") or 0)
        for summary in all_node_tree_manifest_sidecars
    )
    node_tree_manifest_sidecar_path_map_mismatch_kind_counts = (
        _node_tree_manifest_sidecar_path_map_mismatch_kind_counts(
            all_node_tree_manifest_sidecars
        )
    )
    expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts = (
        expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts or {}
    )
    node_tree_manifest_sidecar_parts_planned_count = sum(
        int(summary.get("parts_count") or 0)
        for summary in all_node_tree_manifest_sidecars
    )
    node_tree_manifest_sidecar_joints_planned_count = sum(
        int(summary.get("joints_count") or 0)
        for summary in all_node_tree_manifest_sidecars
    )
    node_tree_manifest_sidecar_part_path_count = sum(
        int(summary.get("part_node_path_count") or 0)
        for summary in all_node_tree_manifest_sidecars
    )
    node_tree_manifest_sidecar_joint_path_count = sum(
        int(summary.get("joint_node_path_count") or 0)
        for summary in all_node_tree_manifest_sidecars
    )
    missing_expected_reason_codes = [
        reason_code
        for reason_code in (expected_reason_codes or [])
        if reason_code not in all_reason_codes
    ]
    unexpected_reason_codes = _unexpected_values(all_reason_codes, allowed_reason_codes)
    present_forbidden_reason_codes = _present_forbidden_values(
        all_reason_codes, forbidden_reason_codes
    )
    missing_expected_affected_inputs = _missing_expected_values(
        expected_affected_inputs or [], all_affected_inputs
    )
    unexpected_affected_inputs = _unexpected_values(
        all_affected_inputs, allowed_affected_inputs
    )
    present_forbidden_affected_inputs = _present_forbidden_values(
        all_affected_inputs, forbidden_affected_inputs
    )
    missing_expected_failed_inputs = _missing_expected_values(
        expected_failed_inputs or [], all_failed_inputs
    )
    unexpected_failed_inputs = _unexpected_values(
        all_failed_inputs, allowed_failed_inputs
    )
    present_forbidden_failed_inputs = _present_forbidden_values(
        all_failed_inputs, forbidden_failed_inputs
    )
    missing_expected_passed_true_inputs = _missing_expected_values(
        expected_passed_true_inputs or [], all_passed_true_inputs
    )
    unexpected_passed_true_inputs = _unexpected_values(
        all_passed_true_inputs, allowed_passed_true_inputs
    )
    present_forbidden_passed_true_inputs = _present_forbidden_values(
        all_passed_true_inputs, forbidden_passed_true_inputs
    )
    missing_expected_passed_false_inputs = _missing_expected_values(
        expected_passed_false_inputs or [], all_passed_false_inputs
    )
    unexpected_passed_false_inputs = _unexpected_values(
        all_passed_false_inputs, allowed_passed_false_inputs
    )
    present_forbidden_passed_false_inputs = _present_forbidden_values(
        all_passed_false_inputs, forbidden_passed_false_inputs
    )
    missing_expected_passed_unknown_inputs = _missing_expected_values(
        expected_passed_unknown_inputs or [], all_passed_unknown_inputs
    )
    unexpected_passed_unknown_inputs = _unexpected_values(
        all_passed_unknown_inputs, allowed_passed_unknown_inputs
    )
    present_forbidden_passed_unknown_inputs = _present_forbidden_values(
        all_passed_unknown_inputs, forbidden_passed_unknown_inputs
    )
    missing_expected_full_mechanical_gate_true_inputs = _missing_expected_values(
        expected_full_mechanical_gate_true_inputs or [],
        all_full_mechanical_gate_true_inputs,
    )
    unexpected_full_mechanical_gate_true_inputs = _unexpected_values(
        all_full_mechanical_gate_true_inputs,
        allowed_full_mechanical_gate_true_inputs,
    )
    present_forbidden_full_mechanical_gate_true_inputs = (
        _present_forbidden_values(
            all_full_mechanical_gate_true_inputs,
            forbidden_full_mechanical_gate_true_inputs,
        )
    )
    missing_expected_full_mechanical_gate_false_inputs = _missing_expected_values(
        expected_full_mechanical_gate_false_inputs or [],
        all_full_mechanical_gate_false_inputs,
    )
    unexpected_full_mechanical_gate_false_inputs = _unexpected_values(
        all_full_mechanical_gate_false_inputs,
        allowed_full_mechanical_gate_false_inputs,
    )
    present_forbidden_full_mechanical_gate_false_inputs = (
        _present_forbidden_values(
            all_full_mechanical_gate_false_inputs,
            forbidden_full_mechanical_gate_false_inputs,
        )
    )
    missing_expected_full_mechanical_gate_unknown_inputs = (
        _missing_expected_values(
            expected_full_mechanical_gate_unknown_inputs or [],
            all_full_mechanical_gate_unknown_inputs,
        )
    )
    unexpected_full_mechanical_gate_unknown_inputs = _unexpected_values(
        all_full_mechanical_gate_unknown_inputs,
        allowed_full_mechanical_gate_unknown_inputs,
    )
    present_forbidden_full_mechanical_gate_unknown_inputs = (
        _present_forbidden_values(
            all_full_mechanical_gate_unknown_inputs,
            forbidden_full_mechanical_gate_unknown_inputs,
        )
    )
    missing_expected_skipped_reasons = _missing_expected_values(
        expected_skipped_reasons or [], all_skipped_reasons
    )
    unexpected_skipped_reasons = _unexpected_values(
        all_skipped_reasons, allowed_skipped_reasons
    )
    present_forbidden_skipped_reasons = _present_forbidden_values(
        all_skipped_reasons, forbidden_skipped_reasons
    )
    expected_node_tree_gate_check_counts = expected_node_tree_gate_check_counts or {}
    expected_mechanical_gate_check_counts = expected_mechanical_gate_check_counts or {}
    mismatched_expected_node_tree_gate_check_counts = _mismatched_expected_map_counts(
        expected_node_tree_gate_check_counts,
        node_tree_gate_check_counts,
    )
    mismatched_expected_mechanical_gate_check_counts = _mismatched_expected_map_counts(
        expected_mechanical_gate_check_counts,
        mechanical_gate_check_counts,
    )
    excluded_output_artifact_inputs = excluded_output_artifact_inputs or []
    preview_counts = {
        "input_paths": (input_paths_count, len(input_paths)),
        "contract_versions": (
            contract_versions_count,
            len(contract_versions),
        ),
        "levels": (levels_count, len(levels)),
        "sources": (sources_count, len(sources)),
        "verification_scopes": (
            verification_scopes_count,
            len(verification_scopes),
        ),
        "acceptance_profiles": (
            acceptance_profiles_count,
            len(acceptance_profiles),
        ),
        "enabled_requirements": (
            enabled_requirements_count,
            len(enabled_requirements),
        ),
        "reason_codes": (reason_codes_count, len(reason_codes)),
        "affected_inputs": (affected_inputs_count, len(affected_inputs)),
        "failed_inputs": (failed_inputs_count, len(failed_inputs)),
        "passed_true_inputs": (
            passed_true_inputs_count,
            len(passed_true_inputs),
        ),
        "passed_false_inputs": (
            passed_false_inputs_count,
            len(passed_false_inputs),
        ),
        "passed_unknown_inputs": (
            passed_unknown_inputs_count,
            len(passed_unknown_inputs),
        ),
        "full_mechanical_gate_true_inputs": (
            full_mechanical_gate_true_inputs_count,
            len(full_mechanical_gate_true_inputs),
        ),
        "full_mechanical_gate_false_inputs": (
            full_mechanical_gate_false_inputs_count,
            len(full_mechanical_gate_false_inputs),
        ),
        "full_mechanical_gate_unknown_inputs": (
            full_mechanical_gate_unknown_inputs_count,
            len(full_mechanical_gate_unknown_inputs),
        ),
        "skipped_inputs": (skipped_inputs_count, len(skipped_inputs)),
        "skipped_reasons": (
            skipped_reasons_count,
            len(skipped_reasons),
        ),
        "node_tree_manifest_sidecars": (
            node_tree_manifest_sidecar_count,
            len(node_tree_manifest_sidecars),
        ),
    }
    truncated_previews = [
        field_name
        for field_name, (full_count, preview_count) in preview_counts.items()
        if full_count > preview_count
    ]
    missing_expected_truncated_previews = _missing_expected_values(
        expected_truncated_previews or [], truncated_previews
    )
    unexpected_truncated_previews = _unexpected_values(
        truncated_previews, allowed_truncated_previews
    )
    present_forbidden_truncated_previews = _present_forbidden_values(
        truncated_previews, forbidden_truncated_previews
    )
    missing_expected_skipped_inputs = _missing_expected_values(
        expected_skipped_inputs or [], all_skipped_inputs
    )
    unexpected_skipped_inputs = _unexpected_values(
        all_skipped_inputs, allowed_skipped_inputs
    )
    present_forbidden_skipped_inputs = _present_forbidden_values(
        all_skipped_inputs, forbidden_skipped_inputs
    )
    return {
        "summary_version": summary_version,
        "status": "error" if failed or aggregate_errors else "success",
        "complete_required_summary_fields_by_source_scope": (
            complete_required_summary_fields_by_source_scope
        ),
        "complete_required_summary_fields_source_scope_count": (
            complete_required_summary_fields_source_scope_count
        ),
        "complete_required_summary_fields_source_scopes": (
            complete_required_summary_fields_source_scopes
        ),
        "preview_limit": preview_limit,
        "truncated_previews": truncated_previews,
        "truncated_previews_count": len(truncated_previews),
        "previews_truncated": bool(truncated_previews),
        "expected_truncated_previews": expected_truncated_previews or [],
        "missing_expected_truncated_previews": (
            missing_expected_truncated_previews
        ),
        "expected_skipped_inputs": expected_skipped_inputs or [],
        "missing_expected_skipped_inputs": missing_expected_skipped_inputs,
        "allowed_truncated_previews": allowed_truncated_previews or [],
        "unexpected_truncated_previews": unexpected_truncated_previews,
        "expected_affected_inputs": expected_affected_inputs or [],
        "missing_expected_affected_inputs": missing_expected_affected_inputs,
        "allowed_affected_inputs": allowed_affected_inputs or [],
        "unexpected_affected_inputs": unexpected_affected_inputs,
        "forbidden_affected_inputs": forbidden_affected_inputs or [],
        "present_forbidden_affected_inputs": (
            present_forbidden_affected_inputs
        ),
        "expected_failed_inputs": expected_failed_inputs or [],
        "missing_expected_failed_inputs": missing_expected_failed_inputs,
        "allowed_failed_inputs": allowed_failed_inputs or [],
        "unexpected_failed_inputs": unexpected_failed_inputs,
        "forbidden_failed_inputs": forbidden_failed_inputs or [],
        "present_forbidden_failed_inputs": present_forbidden_failed_inputs,
        "expected_passed_true_inputs": expected_passed_true_inputs or [],
        "missing_expected_passed_true_inputs": missing_expected_passed_true_inputs,
        "allowed_passed_true_inputs": allowed_passed_true_inputs or [],
        "unexpected_passed_true_inputs": unexpected_passed_true_inputs,
        "forbidden_passed_true_inputs": forbidden_passed_true_inputs or [],
        "present_forbidden_passed_true_inputs": (
            present_forbidden_passed_true_inputs
        ),
        "expected_passed_false_inputs": expected_passed_false_inputs or [],
        "missing_expected_passed_false_inputs": (
            missing_expected_passed_false_inputs
        ),
        "allowed_passed_false_inputs": allowed_passed_false_inputs or [],
        "unexpected_passed_false_inputs": unexpected_passed_false_inputs,
        "forbidden_passed_false_inputs": forbidden_passed_false_inputs or [],
        "present_forbidden_passed_false_inputs": (
            present_forbidden_passed_false_inputs
        ),
        "expected_passed_unknown_inputs": expected_passed_unknown_inputs or [],
        "missing_expected_passed_unknown_inputs": (
            missing_expected_passed_unknown_inputs
        ),
        "allowed_passed_unknown_inputs": allowed_passed_unknown_inputs or [],
        "unexpected_passed_unknown_inputs": unexpected_passed_unknown_inputs,
        "forbidden_passed_unknown_inputs": forbidden_passed_unknown_inputs or [],
        "present_forbidden_passed_unknown_inputs": (
            present_forbidden_passed_unknown_inputs
        ),
        "expected_full_mechanical_gate_true_inputs": (
            expected_full_mechanical_gate_true_inputs or []
        ),
        "missing_expected_full_mechanical_gate_true_inputs": (
            missing_expected_full_mechanical_gate_true_inputs
        ),
        "allowed_full_mechanical_gate_true_inputs": (
            allowed_full_mechanical_gate_true_inputs or []
        ),
        "unexpected_full_mechanical_gate_true_inputs": (
            unexpected_full_mechanical_gate_true_inputs
        ),
        "forbidden_full_mechanical_gate_true_inputs": (
            forbidden_full_mechanical_gate_true_inputs or []
        ),
        "present_forbidden_full_mechanical_gate_true_inputs": (
            present_forbidden_full_mechanical_gate_true_inputs
        ),
        "expected_full_mechanical_gate_false_inputs": (
            expected_full_mechanical_gate_false_inputs or []
        ),
        "missing_expected_full_mechanical_gate_false_inputs": (
            missing_expected_full_mechanical_gate_false_inputs
        ),
        "allowed_full_mechanical_gate_false_inputs": (
            allowed_full_mechanical_gate_false_inputs or []
        ),
        "unexpected_full_mechanical_gate_false_inputs": (
            unexpected_full_mechanical_gate_false_inputs
        ),
        "forbidden_full_mechanical_gate_false_inputs": (
            forbidden_full_mechanical_gate_false_inputs or []
        ),
        "present_forbidden_full_mechanical_gate_false_inputs": (
            present_forbidden_full_mechanical_gate_false_inputs
        ),
        "expected_full_mechanical_gate_unknown_inputs": (
            expected_full_mechanical_gate_unknown_inputs or []
        ),
        "missing_expected_full_mechanical_gate_unknown_inputs": (
            missing_expected_full_mechanical_gate_unknown_inputs
        ),
        "allowed_full_mechanical_gate_unknown_inputs": (
            allowed_full_mechanical_gate_unknown_inputs or []
        ),
        "unexpected_full_mechanical_gate_unknown_inputs": (
            unexpected_full_mechanical_gate_unknown_inputs
        ),
        "forbidden_full_mechanical_gate_unknown_inputs": (
            forbidden_full_mechanical_gate_unknown_inputs or []
        ),
        "present_forbidden_full_mechanical_gate_unknown_inputs": (
            present_forbidden_full_mechanical_gate_unknown_inputs
        ),
        "allowed_skipped_inputs": allowed_skipped_inputs or [],
        "unexpected_skipped_inputs": unexpected_skipped_inputs,
        "forbidden_truncated_previews": forbidden_truncated_previews or [],
        "present_forbidden_truncated_previews": (
            present_forbidden_truncated_previews
        ),
        "forbidden_skipped_inputs": forbidden_skipped_inputs or [],
        "present_forbidden_skipped_inputs": present_forbidden_skipped_inputs,
        "expanded_inputs_count": (
            len(results) if expanded_inputs_count is None else expanded_inputs_count
        ),
        "inputs_count": len(results),
        "input_paths": input_paths,
        "input_paths_count": input_paths_count,
        "input_paths_truncated": input_paths_count > len(input_paths),
        "success_count": len(successes),
        "skipped_count": len(skipped),
        "node_tree_manifest_sidecar_count": node_tree_manifest_sidecar_count,
        "node_tree_manifest_sidecar_complete_count": (
            node_tree_manifest_sidecar_complete_count
        ),
        "node_tree_manifest_sidecar_incomplete_count": (
            node_tree_manifest_sidecar_incomplete_count
        ),
        "node_tree_manifest_sidecar_valid_count": (
            node_tree_manifest_sidecar_valid_count
        ),
        "node_tree_manifest_sidecar_invalid_count": (
            node_tree_manifest_sidecar_invalid_count
        ),
        "node_tree_manifest_sidecar_validation_error_count": (
            node_tree_manifest_sidecar_validation_error_count
        ),
        "node_tree_manifest_sidecar_path_incomplete_count": (
            node_tree_manifest_sidecar_path_incomplete_count
        ),
        "node_tree_manifest_sidecar_path_map_mismatch_count": (
            node_tree_manifest_sidecar_path_map_mismatch_count
        ),
        "node_tree_manifest_sidecar_path_map_mismatch_kind_counts": (
            node_tree_manifest_sidecar_path_map_mismatch_kind_counts
        ),
        "node_tree_manifest_sidecar_parts_planned_count": (
            node_tree_manifest_sidecar_parts_planned_count
        ),
        "node_tree_manifest_sidecar_joints_planned_count": (
            node_tree_manifest_sidecar_joints_planned_count
        ),
        "node_tree_manifest_sidecar_part_path_count": (
            node_tree_manifest_sidecar_part_path_count
        ),
        "node_tree_manifest_sidecar_joint_path_count": (
            node_tree_manifest_sidecar_joint_path_count
        ),
        "node_tree_manifest_sidecars": node_tree_manifest_sidecars,
        "node_tree_manifest_sidecars_truncated": (
            node_tree_manifest_sidecar_count > len(node_tree_manifest_sidecars)
        ),
        "error_count": len(failed),
        "passed_true_count": len(passed_true),
        "passed_false_count": len(passed_false),
        "passed_unknown_count": len(passed_unknown),
        "requires_full_mechanical_restoration_gate_true_count": len(
            full_mechanical_gate_true
        ),
        "requires_full_mechanical_restoration_gate_false_count": len(
            full_mechanical_gate_false
        ),
        "requires_full_mechanical_restoration_gate_unknown_count": len(
            full_mechanical_gate_unknown
        ),
        "smoke_report_written_count": smoke_report_written_count,
        "smoke_report_missing_count": smoke_report_missing_count,
        "smoke_report_read_error_count": smoke_report_read_error_count,
        "control_configured_count": control_configured_count,
        "control_readback_checked_count": control_readback_checked_count,
        "control_readback_missing_count": control_readback_missing_count,
        "node_tree_gate_check_counts": node_tree_gate_check_counts,
        "mechanical_gate_check_counts": mechanical_gate_check_counts,
        "full_mechanical_gate_true_inputs": full_mechanical_gate_true_inputs,
        "full_mechanical_gate_true_inputs_count": (
            full_mechanical_gate_true_inputs_count
        ),
        "full_mechanical_gate_true_inputs_truncated": (
            full_mechanical_gate_true_inputs_count
            > len(full_mechanical_gate_true_inputs)
        ),
        "full_mechanical_gate_false_inputs": full_mechanical_gate_false_inputs,
        "full_mechanical_gate_false_inputs_count": (
            full_mechanical_gate_false_inputs_count
        ),
        "full_mechanical_gate_false_inputs_truncated": (
            full_mechanical_gate_false_inputs_count
            > len(full_mechanical_gate_false_inputs)
        ),
        "full_mechanical_gate_unknown_inputs": full_mechanical_gate_unknown_inputs,
        "full_mechanical_gate_unknown_inputs_count": (
            full_mechanical_gate_unknown_inputs_count
        ),
        "full_mechanical_gate_unknown_inputs_truncated": (
            full_mechanical_gate_unknown_inputs_count
            > len(full_mechanical_gate_unknown_inputs)
        ),
        "require_passed": bool(require_passed),
        "require_required": bool(require_required),
        "require_complete": bool(require_complete),
        "require_full_mechanical_restoration_gate": bool(
            require_full_mechanical_restoration_gate
        ),
        "fail_on_full_mechanical_gate_false": bool(
            fail_on_full_mechanical_gate_false
        ),
        "fail_on_full_mechanical_gate_unknown": bool(
            fail_on_full_mechanical_gate_unknown
        ),
        "fail_on_smoke_report_missing": bool(fail_on_smoke_report_missing),
        "fail_on_smoke_report_read_error": bool(
            fail_on_smoke_report_read_error
        ),
        "fail_on_control_readback_missing": bool(
            fail_on_control_readback_missing
        ),
        "fail_on_node_tree_manifest_sidecar_incomplete": bool(
            fail_on_node_tree_manifest_sidecar_incomplete
        ),
        "fail_on_invalid_node_tree_manifest_sidecar": bool(
            fail_on_invalid_node_tree_manifest_sidecar
        ),
        "fail_on_node_tree_manifest_sidecar_validation_error": bool(
            fail_on_node_tree_manifest_sidecar_validation_error
        ),
        "fail_on_node_tree_manifest_sidecar_path_incomplete": bool(
            fail_on_node_tree_manifest_sidecar_path_incomplete
        ),
        "fail_on_node_tree_manifest_sidecar_path_map_mismatch": bool(
            fail_on_node_tree_manifest_sidecar_path_map_mismatch
        ),
        "expected_inputs_count": expected_inputs_count,
        "expected_expanded_inputs_count": expected_expanded_inputs_count,
        "expected_excluded_output_artifacts_count": (
            expected_excluded_output_artifacts_count
        ),
        "expected_success_count": expected_success_count,
        "expected_error_count": expected_error_count,
        "expected_passed_true_count": expected_passed_true_count,
        "expected_passed_false_count": expected_passed_false_count,
        "expected_passed_unknown_count": expected_passed_unknown_count,
        "expected_full_mechanical_gate_true_count": (
            expected_full_mechanical_gate_true_count
        ),
        "expected_full_mechanical_gate_false_count": (
            expected_full_mechanical_gate_false_count
        ),
        "expected_full_mechanical_gate_unknown_count": (
            expected_full_mechanical_gate_unknown_count
        ),
        "expected_smoke_report_written_count": expected_smoke_report_written_count,
        "expected_smoke_report_missing_count": expected_smoke_report_missing_count,
        "expected_smoke_report_read_error_count": (
            expected_smoke_report_read_error_count
        ),
        "expected_control_configured_count": expected_control_configured_count,
        "expected_control_readback_checked_count": (
            expected_control_readback_checked_count
        ),
        "expected_control_readback_missing_count": (
            expected_control_readback_missing_count
        ),
        "expected_node_tree_manifest_sidecar_count": (
            expected_node_tree_manifest_sidecar_count
        ),
        "expected_node_tree_manifest_sidecar_complete_count": (
            expected_node_tree_manifest_sidecar_complete_count
        ),
        "expected_node_tree_manifest_sidecar_incomplete_count": (
            expected_node_tree_manifest_sidecar_incomplete_count
        ),
        "expected_node_tree_manifest_sidecar_valid_count": (
            expected_node_tree_manifest_sidecar_valid_count
        ),
        "expected_node_tree_manifest_sidecar_invalid_count": (
            expected_node_tree_manifest_sidecar_invalid_count
        ),
        "expected_node_tree_manifest_sidecar_validation_error_count": (
            expected_node_tree_manifest_sidecar_validation_error_count
        ),
        "expected_node_tree_manifest_sidecar_path_incomplete_count": (
            expected_node_tree_manifest_sidecar_path_incomplete_count
        ),
        "expected_node_tree_manifest_sidecar_path_map_mismatch_count": (
            expected_node_tree_manifest_sidecar_path_map_mismatch_count
        ),
        "expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts": (
            expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts
        ),
        "expected_node_tree_manifest_sidecar_parts_planned_count": (
            expected_node_tree_manifest_sidecar_parts_planned_count
        ),
        "expected_node_tree_manifest_sidecar_joints_planned_count": (
            expected_node_tree_manifest_sidecar_joints_planned_count
        ),
        "expected_node_tree_manifest_sidecar_part_path_count": (
            expected_node_tree_manifest_sidecar_part_path_count
        ),
        "expected_node_tree_manifest_sidecar_joint_path_count": (
            expected_node_tree_manifest_sidecar_joint_path_count
        ),
        "expected_summary_counts": expected_summary_counts,
        "actual_expected_summary_counts": actual_expected_summary_counts,
        "mismatched_expected_summary_counts": mismatched_expected_summary_counts,
        "expected_summary_values": expected_summary_values,
        "expected_summary_value_sources": expected_summary_value_sources,
        "missing_expected_summary_value_sources": (
            missing_expected_summary_value_sources
        ),
        "expected_summary_value_source_matched_count": (
            expected_summary_value_source_matched_count
        ),
        "expected_summary_value_source_excluded_count": (
            expected_summary_value_source_excluded_count
        ),
        "expected_summary_value_source_excluded_sources": (
            expected_summary_value_source_excluded_sources or []
        ),
        "missing_expected_summary_value_source_excluded_sources": (
            missing_expected_summary_value_source_excluded_sources
        ),
        "allowed_summary_value_source_excluded_sources": (
            allowed_summary_value_source_excluded_sources or []
        ),
        "unexpected_summary_value_source_excluded_sources": (
            unexpected_summary_value_source_excluded_sources
        ),
        "forbidden_summary_value_source_excluded_sources": (
            forbidden_summary_value_source_excluded_sources or []
        ),
        "present_forbidden_summary_value_source_excluded_sources": (
            present_forbidden_summary_value_source_excluded_sources
        ),
        "summary_value_source_matched_count": summary_value_source_matched_count,
        "summary_value_source_matched_sources": summary_value_source_matched_sources,
        "allowed_summary_value_source_matched_sources": (
            allowed_summary_value_source_matched_sources or []
        ),
        "unexpected_summary_value_source_matched_sources": (
            unexpected_summary_value_source_matched_sources
        ),
        "forbidden_summary_value_source_matched_sources": (
            forbidden_summary_value_source_matched_sources or []
        ),
        "present_forbidden_summary_value_source_matched_sources": (
            present_forbidden_summary_value_source_matched_sources
        ),
        "summary_value_source_excluded_count": summary_value_source_excluded_count,
        "summary_value_source_excluded_sources": (
            summary_value_source_excluded_sources
        ),
        "actual_expected_summary_values": actual_expected_summary_values,
        "mismatched_expected_summary_values": mismatched_expected_summary_values,
        "expected_node_tree_gate_check_counts": expected_node_tree_gate_check_counts,
        "mismatched_expected_node_tree_gate_check_counts": (
            mismatched_expected_node_tree_gate_check_counts
        ),
        "expected_mechanical_gate_check_counts": expected_mechanical_gate_check_counts,
        "mismatched_expected_mechanical_gate_check_counts": (
            mismatched_expected_mechanical_gate_check_counts
        ),
        "expected_skipped_count": expected_skipped_count,
        "expected_reason_codes_count": expected_reason_codes_count,
        "expected_contract_versions_count": expected_contract_versions_count,
        "expected_summary_versions_count": expected_summary_versions_count,
        "expected_validation_summary_statuses_count": (
            expected_validation_summary_statuses_count
        ),
        "expected_levels_count": expected_levels_count,
        "expected_sources_count": expected_sources_count,
        "expected_verification_scopes_count": expected_verification_scopes_count,
        "expected_acceptance_profiles_count": expected_acceptance_profiles_count,
        "expected_enabled_requirements_count": expected_enabled_requirements_count,
        "expected_complete_required_summary_fields_source_scope_count": (
            expected_complete_required_summary_fields_source_scope_count
        ),
        "expected_affected_inputs_count": expected_affected_inputs_count,
        "expected_failed_inputs_count": expected_failed_inputs_count,
        "expected_skipped_inputs_count": expected_skipped_inputs_count,
        "expected_skipped_reasons_count": expected_skipped_reasons_count,
        "expected_truncated_previews_count": expected_truncated_previews_count,
        "expected_inputs": expected_inputs or [],
        "missing_expected_inputs": missing_expected_inputs,
        "allowed_inputs": allowed_inputs or [],
        "unexpected_inputs": unexpected_inputs,
        "forbidden_inputs": forbidden_inputs or [],
        "present_forbidden_inputs": present_forbidden_inputs,
        "expected_contract_versions": expected_contract_versions or [],
        "missing_expected_contract_versions": missing_expected_contract_versions,
        "expected_summary_versions": expected_summary_versions or [],
        "missing_expected_summary_versions": missing_expected_summary_versions,
        "expected_validation_summary_statuses": (
            expected_validation_summary_statuses or []
        ),
        "missing_expected_validation_summary_statuses": (
            missing_expected_validation_summary_statuses
        ),
        "expected_levels": expected_levels or [],
        "missing_expected_levels": missing_expected_levels,
        "expected_sources": expected_sources or [],
        "missing_expected_sources": missing_expected_sources,
        "expected_verification_scopes": expected_verification_scopes or [],
        "missing_expected_verification_scopes": (
            missing_expected_verification_scopes
        ),
        "expected_acceptance_profiles": expected_acceptance_profiles or [],
        "missing_expected_acceptance_profiles": missing_expected_acceptance_profiles,
        "expected_enabled_requirements": expected_enabled_requirements or [],
        "missing_expected_enabled_requirements": (
            missing_expected_enabled_requirements
        ),
        "expected_complete_required_summary_fields_source_scopes": (
            expected_complete_required_summary_fields_source_scopes or []
        ),
        "missing_expected_complete_required_summary_fields_source_scopes": (
            missing_expected_complete_required_summary_fields_source_scopes
        ),
        "allowed_complete_required_summary_fields_source_scopes": (
            allowed_complete_required_summary_fields_source_scopes or []
        ),
        "unexpected_complete_required_summary_fields_source_scopes": (
            unexpected_complete_required_summary_fields_source_scopes
        ),
        "forbidden_complete_required_summary_fields_source_scopes": (
            forbidden_complete_required_summary_fields_source_scopes or []
        ),
        "present_forbidden_complete_required_summary_fields_source_scopes": (
            present_forbidden_complete_required_summary_fields_source_scopes
        ),
        "allowed_contract_versions": allowed_contract_versions or [],
        "unexpected_contract_versions": unexpected_contract_versions,
        "allowed_summary_versions": allowed_summary_versions or [],
        "unexpected_summary_versions": unexpected_summary_versions,
        "allowed_validation_summary_statuses": (
            allowed_validation_summary_statuses or []
        ),
        "unexpected_validation_summary_statuses": (
            unexpected_validation_summary_statuses
        ),
        "allowed_levels": allowed_levels or [],
        "unexpected_levels": unexpected_levels,
        "allowed_sources": allowed_sources or [],
        "unexpected_sources": unexpected_sources,
        "allowed_verification_scopes": allowed_verification_scopes or [],
        "unexpected_verification_scopes": unexpected_verification_scopes,
        "allowed_acceptance_profiles": allowed_acceptance_profiles or [],
        "unexpected_acceptance_profiles": unexpected_acceptance_profiles,
        "allowed_enabled_requirements": allowed_enabled_requirements or [],
        "unexpected_enabled_requirements": unexpected_enabled_requirements,
        "forbidden_contract_versions": forbidden_contract_versions or [],
        "present_forbidden_contract_versions": present_forbidden_contract_versions,
        "forbidden_summary_versions": forbidden_summary_versions or [],
        "present_forbidden_summary_versions": present_forbidden_summary_versions,
        "forbidden_validation_summary_statuses": (
            forbidden_validation_summary_statuses or []
        ),
        "present_forbidden_validation_summary_statuses": (
            present_forbidden_validation_summary_statuses
        ),
        "forbidden_levels": forbidden_levels or [],
        "present_forbidden_levels": present_forbidden_levels,
        "forbidden_sources": forbidden_sources or [],
        "present_forbidden_sources": present_forbidden_sources,
        "forbidden_verification_scopes": forbidden_verification_scopes or [],
        "present_forbidden_verification_scopes": (
            present_forbidden_verification_scopes
        ),
        "forbidden_acceptance_profiles": forbidden_acceptance_profiles or [],
        "present_forbidden_acceptance_profiles": (
            present_forbidden_acceptance_profiles
        ),
        "forbidden_enabled_requirements": forbidden_enabled_requirements or [],
        "present_forbidden_enabled_requirements": (
            present_forbidden_enabled_requirements
        ),
        "expected_reason_codes": expected_reason_codes or [],
        "missing_expected_reason_codes": missing_expected_reason_codes,
        "allowed_reason_codes": allowed_reason_codes or [],
        "unexpected_reason_codes": unexpected_reason_codes,
        "forbidden_reason_codes": forbidden_reason_codes or [],
        "present_forbidden_reason_codes": present_forbidden_reason_codes,
        "expected_skipped_reasons": expected_skipped_reasons or [],
        "missing_expected_skipped_reasons": missing_expected_skipped_reasons,
        "allowed_skipped_reasons": allowed_skipped_reasons or [],
        "unexpected_skipped_reasons": unexpected_skipped_reasons,
        "forbidden_skipped_reasons": forbidden_skipped_reasons or [],
        "present_forbidden_skipped_reasons": present_forbidden_skipped_reasons,
        "excluded_output_artifact_inputs": excluded_output_artifact_inputs,
        "excluded_output_artifact_inputs_count": len(
            excluded_output_artifact_inputs
        ),
        "errors": aggregate_errors,
        "contract_versions": contract_versions,
        "contract_versions_count": contract_versions_count,
        "contract_versions_truncated": (
            contract_versions_count > len(contract_versions)
        ),
        "summary_versions": summary_versions,
        "summary_versions_count": summary_versions_count,
        "summary_versions_truncated": (
            summary_versions_count > len(summary_versions)
        ),
        "validation_summary_statuses": validation_summary_statuses,
        "validation_summary_statuses_count": validation_summary_statuses_count,
        "validation_summary_statuses_truncated": (
            validation_summary_statuses_count > len(validation_summary_statuses)
        ),
        "levels": levels,
        "levels_count": levels_count,
        "levels_truncated": levels_count > len(levels),
        "sources": sources,
        "sources_count": sources_count,
        "sources_truncated": sources_count > len(sources),
        "verification_scopes": verification_scopes,
        "verification_scopes_count": verification_scopes_count,
        "verification_scopes_truncated": (
            verification_scopes_count > len(verification_scopes)
        ),
        "acceptance_profiles": acceptance_profiles,
        "acceptance_profiles_count": acceptance_profiles_count,
        "acceptance_profiles_truncated": (
            acceptance_profiles_count > len(acceptance_profiles)
        ),
        "enabled_requirements": enabled_requirements,
        "enabled_requirements_count": enabled_requirements_count,
        "enabled_requirements_truncated": (
            enabled_requirements_count > len(enabled_requirements)
        ),
        "reason_codes": reason_codes,
        "reason_codes_count": reason_codes_count,
        "reason_codes_truncated": reason_codes_count > len(reason_codes),
        "affected_inputs": affected_inputs,
        "affected_inputs_count": affected_inputs_count,
        "affected_inputs_truncated": affected_inputs_count > len(affected_inputs),
        "failed_inputs": failed_inputs,
        "failed_inputs_count": failed_inputs_count,
        "failed_inputs_truncated": failed_inputs_count > len(failed_inputs),
        "passed_true_inputs": passed_true_inputs,
        "passed_true_inputs_count": passed_true_inputs_count,
        "passed_true_inputs_truncated": (
            passed_true_inputs_count > len(passed_true_inputs)
        ),
        "passed_false_inputs": passed_false_inputs,
        "passed_false_inputs_count": passed_false_inputs_count,
        "passed_false_inputs_truncated": (
            passed_false_inputs_count > len(passed_false_inputs)
        ),
        "passed_unknown_inputs": passed_unknown_inputs,
        "passed_unknown_inputs_count": passed_unknown_inputs_count,
        "passed_unknown_inputs_truncated": (
            passed_unknown_inputs_count > len(passed_unknown_inputs)
        ),
        "skipped_inputs": skipped_inputs,
        "skipped_inputs_count": skipped_inputs_count,
        "skipped_inputs_truncated": skipped_inputs_count > len(skipped_inputs),
        "skipped_reasons": skipped_reasons,
        "skipped_reasons_count": skipped_reasons_count,
        "skipped_reasons_truncated": skipped_reasons_count > len(skipped_reasons),
    }


def _write_json(path: Path, payload: Any, *, label: str) -> str | None:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
    except OSError as exc:
        return f"unable to write {label}: {exc}"
    return None


def _option_supplied(argv: list[str], option: str) -> bool:
    return any(arg == option or arg.startswith(f"{option}=") for arg in argv)


def _output_payload_for_shape(
    *,
    output_shape: str,
    summary_only: bool,
    summary_payload: dict[str, Any],
    result: dict[str, Any],
) -> dict[str, Any]:
    if output_shape == "summary" or (
        output_shape == "auto" and summary_only
    ):
        return summary_payload
    return result


def _build_result_payload(
    *,
    results: list[dict[str, Any]],
    summary_payload: dict[str, Any],
    aggregate_errors: list[str],
) -> dict[str, Any]:
    if not _has_validation_summary_context(
        results=results,
        summary_payload=summary_payload,
        aggregate_errors=aggregate_errors,
    ):
        return results[0]
    return {
        "status": summary_payload["status"],
        "complete_required_summary_fields_by_source_scope": summary_payload[
            "complete_required_summary_fields_by_source_scope"
        ],
        "complete_required_summary_fields_source_scope_count": summary_payload[
            "complete_required_summary_fields_source_scope_count"
        ],
        "complete_required_summary_fields_source_scopes": summary_payload[
            "complete_required_summary_fields_source_scopes"
        ],
        "preview_limit": summary_payload["preview_limit"],
        "truncated_previews": summary_payload["truncated_previews"],
        "truncated_previews_count": summary_payload["truncated_previews_count"],
        "previews_truncated": summary_payload["previews_truncated"],
        "expected_truncated_previews": summary_payload[
            "expected_truncated_previews"
        ],
        "missing_expected_truncated_previews": summary_payload[
            "missing_expected_truncated_previews"
        ],
        "expected_affected_inputs": summary_payload["expected_affected_inputs"],
        "missing_expected_affected_inputs": summary_payload[
            "missing_expected_affected_inputs"
        ],
        "expected_failed_inputs": summary_payload["expected_failed_inputs"],
        "missing_expected_failed_inputs": summary_payload[
            "missing_expected_failed_inputs"
        ],
        "expected_passed_true_inputs": summary_payload[
            "expected_passed_true_inputs"
        ],
        "missing_expected_passed_true_inputs": summary_payload[
            "missing_expected_passed_true_inputs"
        ],
        "expected_passed_false_inputs": summary_payload[
            "expected_passed_false_inputs"
        ],
        "missing_expected_passed_false_inputs": summary_payload[
            "missing_expected_passed_false_inputs"
        ],
        "expected_passed_unknown_inputs": summary_payload[
            "expected_passed_unknown_inputs"
        ],
        "missing_expected_passed_unknown_inputs": summary_payload[
            "missing_expected_passed_unknown_inputs"
        ],
        "expected_full_mechanical_gate_true_inputs": summary_payload[
            "expected_full_mechanical_gate_true_inputs"
        ],
        "missing_expected_full_mechanical_gate_true_inputs": summary_payload[
            "missing_expected_full_mechanical_gate_true_inputs"
        ],
        "expected_full_mechanical_gate_false_inputs": summary_payload[
            "expected_full_mechanical_gate_false_inputs"
        ],
        "missing_expected_full_mechanical_gate_false_inputs": summary_payload[
            "missing_expected_full_mechanical_gate_false_inputs"
        ],
        "expected_full_mechanical_gate_unknown_inputs": summary_payload[
            "expected_full_mechanical_gate_unknown_inputs"
        ],
        "missing_expected_full_mechanical_gate_unknown_inputs": summary_payload[
            "missing_expected_full_mechanical_gate_unknown_inputs"
        ],
        "expected_skipped_inputs": summary_payload["expected_skipped_inputs"],
        "missing_expected_skipped_inputs": summary_payload[
            "missing_expected_skipped_inputs"
        ],
        "allowed_truncated_previews": summary_payload[
            "allowed_truncated_previews"
        ],
        "unexpected_truncated_previews": summary_payload[
            "unexpected_truncated_previews"
        ],
        "allowed_affected_inputs": summary_payload["allowed_affected_inputs"],
        "unexpected_affected_inputs": summary_payload[
            "unexpected_affected_inputs"
        ],
        "allowed_failed_inputs": summary_payload["allowed_failed_inputs"],
        "unexpected_failed_inputs": summary_payload[
            "unexpected_failed_inputs"
        ],
        "allowed_passed_true_inputs": summary_payload[
            "allowed_passed_true_inputs"
        ],
        "unexpected_passed_true_inputs": summary_payload[
            "unexpected_passed_true_inputs"
        ],
        "allowed_passed_false_inputs": summary_payload[
            "allowed_passed_false_inputs"
        ],
        "unexpected_passed_false_inputs": summary_payload[
            "unexpected_passed_false_inputs"
        ],
        "allowed_passed_unknown_inputs": summary_payload[
            "allowed_passed_unknown_inputs"
        ],
        "unexpected_passed_unknown_inputs": summary_payload[
            "unexpected_passed_unknown_inputs"
        ],
        "allowed_full_mechanical_gate_true_inputs": summary_payload[
            "allowed_full_mechanical_gate_true_inputs"
        ],
        "unexpected_full_mechanical_gate_true_inputs": summary_payload[
            "unexpected_full_mechanical_gate_true_inputs"
        ],
        "allowed_full_mechanical_gate_false_inputs": summary_payload[
            "allowed_full_mechanical_gate_false_inputs"
        ],
        "unexpected_full_mechanical_gate_false_inputs": summary_payload[
            "unexpected_full_mechanical_gate_false_inputs"
        ],
        "allowed_full_mechanical_gate_unknown_inputs": summary_payload[
            "allowed_full_mechanical_gate_unknown_inputs"
        ],
        "unexpected_full_mechanical_gate_unknown_inputs": summary_payload[
            "unexpected_full_mechanical_gate_unknown_inputs"
        ],
        "allowed_skipped_inputs": summary_payload["allowed_skipped_inputs"],
        "unexpected_skipped_inputs": summary_payload[
            "unexpected_skipped_inputs"
        ],
        "forbidden_truncated_previews": summary_payload[
            "forbidden_truncated_previews"
        ],
        "present_forbidden_truncated_previews": summary_payload[
            "present_forbidden_truncated_previews"
        ],
        "forbidden_affected_inputs": summary_payload[
            "forbidden_affected_inputs"
        ],
        "present_forbidden_affected_inputs": summary_payload[
            "present_forbidden_affected_inputs"
        ],
        "forbidden_failed_inputs": summary_payload["forbidden_failed_inputs"],
        "present_forbidden_failed_inputs": summary_payload[
            "present_forbidden_failed_inputs"
        ],
        "forbidden_passed_true_inputs": summary_payload[
            "forbidden_passed_true_inputs"
        ],
        "present_forbidden_passed_true_inputs": summary_payload[
            "present_forbidden_passed_true_inputs"
        ],
        "forbidden_passed_false_inputs": summary_payload[
            "forbidden_passed_false_inputs"
        ],
        "present_forbidden_passed_false_inputs": summary_payload[
            "present_forbidden_passed_false_inputs"
        ],
        "forbidden_passed_unknown_inputs": summary_payload[
            "forbidden_passed_unknown_inputs"
        ],
        "present_forbidden_passed_unknown_inputs": summary_payload[
            "present_forbidden_passed_unknown_inputs"
        ],
        "forbidden_full_mechanical_gate_true_inputs": summary_payload[
            "forbidden_full_mechanical_gate_true_inputs"
        ],
        "present_forbidden_full_mechanical_gate_true_inputs": summary_payload[
            "present_forbidden_full_mechanical_gate_true_inputs"
        ],
        "forbidden_full_mechanical_gate_false_inputs": summary_payload[
            "forbidden_full_mechanical_gate_false_inputs"
        ],
        "present_forbidden_full_mechanical_gate_false_inputs": summary_payload[
            "present_forbidden_full_mechanical_gate_false_inputs"
        ],
        "forbidden_full_mechanical_gate_unknown_inputs": summary_payload[
            "forbidden_full_mechanical_gate_unknown_inputs"
        ],
        "present_forbidden_full_mechanical_gate_unknown_inputs": summary_payload[
            "present_forbidden_full_mechanical_gate_unknown_inputs"
        ],
        "forbidden_skipped_inputs": summary_payload["forbidden_skipped_inputs"],
        "present_forbidden_skipped_inputs": summary_payload[
            "present_forbidden_skipped_inputs"
        ],
        "expanded_inputs_count": summary_payload["expanded_inputs_count"],
        "inputs_count": summary_payload["inputs_count"],
        "input_paths": summary_payload["input_paths"],
        "input_paths_count": summary_payload["input_paths_count"],
        "input_paths_truncated": summary_payload["input_paths_truncated"],
        "expected_inputs": summary_payload["expected_inputs"],
        "missing_expected_inputs": summary_payload["missing_expected_inputs"],
        "allowed_inputs": summary_payload["allowed_inputs"],
        "unexpected_inputs": summary_payload["unexpected_inputs"],
        "forbidden_inputs": summary_payload["forbidden_inputs"],
        "present_forbidden_inputs": summary_payload["present_forbidden_inputs"],
        "success_count": summary_payload["success_count"],
        "skipped_count": summary_payload["skipped_count"],
        "node_tree_manifest_sidecar_count": summary_payload[
            "node_tree_manifest_sidecar_count"
        ],
        "node_tree_manifest_sidecar_complete_count": summary_payload[
            "node_tree_manifest_sidecar_complete_count"
        ],
        "node_tree_manifest_sidecar_incomplete_count": summary_payload[
            "node_tree_manifest_sidecar_incomplete_count"
        ],
        "node_tree_manifest_sidecar_valid_count": summary_payload[
            "node_tree_manifest_sidecar_valid_count"
        ],
        "node_tree_manifest_sidecar_invalid_count": summary_payload[
            "node_tree_manifest_sidecar_invalid_count"
        ],
        "node_tree_manifest_sidecar_validation_error_count": summary_payload[
            "node_tree_manifest_sidecar_validation_error_count"
        ],
        "node_tree_manifest_sidecar_path_incomplete_count": summary_payload[
            "node_tree_manifest_sidecar_path_incomplete_count"
        ],
        "node_tree_manifest_sidecar_path_map_mismatch_count": summary_payload[
            "node_tree_manifest_sidecar_path_map_mismatch_count"
        ],
        "node_tree_manifest_sidecar_path_map_mismatch_kind_counts": summary_payload[
            "node_tree_manifest_sidecar_path_map_mismatch_kind_counts"
        ],
        "node_tree_manifest_sidecar_parts_planned_count": summary_payload[
            "node_tree_manifest_sidecar_parts_planned_count"
        ],
        "node_tree_manifest_sidecar_joints_planned_count": summary_payload[
            "node_tree_manifest_sidecar_joints_planned_count"
        ],
        "node_tree_manifest_sidecar_part_path_count": summary_payload[
            "node_tree_manifest_sidecar_part_path_count"
        ],
        "node_tree_manifest_sidecar_joint_path_count": summary_payload[
            "node_tree_manifest_sidecar_joint_path_count"
        ],
        "node_tree_manifest_sidecars": summary_payload[
            "node_tree_manifest_sidecars"
        ],
        "node_tree_manifest_sidecars_truncated": summary_payload[
            "node_tree_manifest_sidecars_truncated"
        ],
        "error_count": summary_payload["error_count"],
        "passed_true_count": summary_payload["passed_true_count"],
        "passed_false_count": summary_payload["passed_false_count"],
        "passed_unknown_count": summary_payload["passed_unknown_count"],
        "passed_true_inputs": summary_payload["passed_true_inputs"],
        "passed_true_inputs_count": summary_payload["passed_true_inputs_count"],
        "passed_true_inputs_truncated": summary_payload[
            "passed_true_inputs_truncated"
        ],
        "passed_false_inputs": summary_payload["passed_false_inputs"],
        "passed_false_inputs_count": summary_payload["passed_false_inputs_count"],
        "passed_false_inputs_truncated": summary_payload[
            "passed_false_inputs_truncated"
        ],
        "passed_unknown_inputs": summary_payload["passed_unknown_inputs"],
        "passed_unknown_inputs_count": summary_payload[
            "passed_unknown_inputs_count"
        ],
        "passed_unknown_inputs_truncated": summary_payload[
            "passed_unknown_inputs_truncated"
        ],
        "requires_full_mechanical_restoration_gate_true_count": summary_payload[
            "requires_full_mechanical_restoration_gate_true_count"
        ],
        "requires_full_mechanical_restoration_gate_false_count": summary_payload[
            "requires_full_mechanical_restoration_gate_false_count"
        ],
        "requires_full_mechanical_restoration_gate_unknown_count": summary_payload[
            "requires_full_mechanical_restoration_gate_unknown_count"
        ],
        "smoke_report_written_count": summary_payload[
            "smoke_report_written_count"
        ],
        "smoke_report_missing_count": summary_payload[
            "smoke_report_missing_count"
        ],
        "smoke_report_read_error_count": summary_payload[
            "smoke_report_read_error_count"
        ],
        "control_configured_count": summary_payload["control_configured_count"],
        "control_readback_checked_count": summary_payload[
            "control_readback_checked_count"
        ],
        "control_readback_missing_count": summary_payload[
            "control_readback_missing_count"
        ],
        "node_tree_gate_check_counts": summary_payload[
            "node_tree_gate_check_counts"
        ],
        "mechanical_gate_check_counts": summary_payload[
            "mechanical_gate_check_counts"
        ],
        "full_mechanical_gate_true_inputs": summary_payload[
            "full_mechanical_gate_true_inputs"
        ],
        "full_mechanical_gate_true_inputs_count": summary_payload[
            "full_mechanical_gate_true_inputs_count"
        ],
        "full_mechanical_gate_true_inputs_truncated": summary_payload[
            "full_mechanical_gate_true_inputs_truncated"
        ],
        "full_mechanical_gate_false_inputs": summary_payload[
            "full_mechanical_gate_false_inputs"
        ],
        "full_mechanical_gate_false_inputs_count": summary_payload[
            "full_mechanical_gate_false_inputs_count"
        ],
        "full_mechanical_gate_false_inputs_truncated": summary_payload[
            "full_mechanical_gate_false_inputs_truncated"
        ],
        "full_mechanical_gate_unknown_inputs": summary_payload[
            "full_mechanical_gate_unknown_inputs"
        ],
        "full_mechanical_gate_unknown_inputs_count": summary_payload[
            "full_mechanical_gate_unknown_inputs_count"
        ],
        "full_mechanical_gate_unknown_inputs_truncated": summary_payload[
            "full_mechanical_gate_unknown_inputs_truncated"
        ],
        "require_passed": summary_payload["require_passed"],
        "require_required": summary_payload["require_required"],
        "require_complete": summary_payload["require_complete"],
        "require_full_mechanical_restoration_gate": summary_payload[
            "require_full_mechanical_restoration_gate"
        ],
        "fail_on_full_mechanical_gate_false": summary_payload[
            "fail_on_full_mechanical_gate_false"
        ],
        "fail_on_full_mechanical_gate_unknown": summary_payload[
            "fail_on_full_mechanical_gate_unknown"
        ],
        "fail_on_smoke_report_missing": summary_payload[
            "fail_on_smoke_report_missing"
        ],
        "fail_on_smoke_report_read_error": summary_payload[
            "fail_on_smoke_report_read_error"
        ],
        "fail_on_control_readback_missing": summary_payload[
            "fail_on_control_readback_missing"
        ],
        "fail_on_node_tree_manifest_sidecar_incomplete": summary_payload[
            "fail_on_node_tree_manifest_sidecar_incomplete"
        ],
        "fail_on_invalid_node_tree_manifest_sidecar": summary_payload[
            "fail_on_invalid_node_tree_manifest_sidecar"
        ],
        "fail_on_node_tree_manifest_sidecar_validation_error": summary_payload[
            "fail_on_node_tree_manifest_sidecar_validation_error"
        ],
        "fail_on_node_tree_manifest_sidecar_path_incomplete": summary_payload[
            "fail_on_node_tree_manifest_sidecar_path_incomplete"
        ],
        "fail_on_node_tree_manifest_sidecar_path_map_mismatch": summary_payload[
            "fail_on_node_tree_manifest_sidecar_path_map_mismatch"
        ],
        "expected_inputs_count": summary_payload["expected_inputs_count"],
        "expected_expanded_inputs_count": summary_payload[
            "expected_expanded_inputs_count"
        ],
        "expected_excluded_output_artifacts_count": summary_payload[
            "expected_excluded_output_artifacts_count"
        ],
        "expected_success_count": summary_payload["expected_success_count"],
        "expected_error_count": summary_payload["expected_error_count"],
        "expected_passed_true_count": summary_payload[
            "expected_passed_true_count"
        ],
        "expected_passed_false_count": summary_payload[
            "expected_passed_false_count"
        ],
        "expected_passed_unknown_count": summary_payload[
            "expected_passed_unknown_count"
        ],
        "expected_full_mechanical_gate_true_count": summary_payload[
            "expected_full_mechanical_gate_true_count"
        ],
        "expected_full_mechanical_gate_false_count": summary_payload[
            "expected_full_mechanical_gate_false_count"
        ],
        "expected_full_mechanical_gate_unknown_count": summary_payload[
            "expected_full_mechanical_gate_unknown_count"
        ],
        "expected_smoke_report_written_count": summary_payload[
            "expected_smoke_report_written_count"
        ],
        "expected_smoke_report_missing_count": summary_payload[
            "expected_smoke_report_missing_count"
        ],
        "expected_smoke_report_read_error_count": summary_payload[
            "expected_smoke_report_read_error_count"
        ],
        "expected_control_configured_count": summary_payload[
            "expected_control_configured_count"
        ],
        "expected_control_readback_checked_count": summary_payload[
            "expected_control_readback_checked_count"
        ],
        "expected_control_readback_missing_count": summary_payload[
            "expected_control_readback_missing_count"
        ],
        "expected_node_tree_manifest_sidecar_count": summary_payload[
            "expected_node_tree_manifest_sidecar_count"
        ],
        "expected_node_tree_manifest_sidecar_complete_count": summary_payload[
            "expected_node_tree_manifest_sidecar_complete_count"
        ],
        "expected_node_tree_manifest_sidecar_incomplete_count": summary_payload[
            "expected_node_tree_manifest_sidecar_incomplete_count"
        ],
        "expected_node_tree_manifest_sidecar_valid_count": summary_payload[
            "expected_node_tree_manifest_sidecar_valid_count"
        ],
        "expected_node_tree_manifest_sidecar_invalid_count": summary_payload[
            "expected_node_tree_manifest_sidecar_invalid_count"
        ],
        "expected_node_tree_manifest_sidecar_validation_error_count": (
            summary_payload[
                "expected_node_tree_manifest_sidecar_validation_error_count"
            ]
        ),
        "expected_node_tree_manifest_sidecar_path_incomplete_count": summary_payload[
            "expected_node_tree_manifest_sidecar_path_incomplete_count"
        ],
        "expected_node_tree_manifest_sidecar_path_map_mismatch_count": (
            summary_payload[
                "expected_node_tree_manifest_sidecar_path_map_mismatch_count"
            ]
        ),
        "expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts": (
            summary_payload[
                "expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts"
            ]
        ),
        "expected_node_tree_manifest_sidecar_parts_planned_count": summary_payload[
            "expected_node_tree_manifest_sidecar_parts_planned_count"
        ],
        "expected_node_tree_manifest_sidecar_joints_planned_count": summary_payload[
            "expected_node_tree_manifest_sidecar_joints_planned_count"
        ],
        "expected_node_tree_manifest_sidecar_part_path_count": summary_payload[
            "expected_node_tree_manifest_sidecar_part_path_count"
        ],
        "expected_node_tree_manifest_sidecar_joint_path_count": summary_payload[
            "expected_node_tree_manifest_sidecar_joint_path_count"
        ],
        "expected_summary_counts": summary_payload["expected_summary_counts"],
        "actual_expected_summary_counts": summary_payload[
            "actual_expected_summary_counts"
        ],
        "mismatched_expected_summary_counts": summary_payload[
            "mismatched_expected_summary_counts"
        ],
        "expected_summary_values": summary_payload["expected_summary_values"],
        "expected_summary_value_sources": summary_payload[
            "expected_summary_value_sources"
        ],
        "missing_expected_summary_value_sources": summary_payload[
            "missing_expected_summary_value_sources"
        ],
        "expected_summary_value_source_matched_count": summary_payload[
            "expected_summary_value_source_matched_count"
        ],
        "expected_summary_value_source_excluded_count": summary_payload[
            "expected_summary_value_source_excluded_count"
        ],
        "expected_summary_value_source_excluded_sources": summary_payload[
            "expected_summary_value_source_excluded_sources"
        ],
        "missing_expected_summary_value_source_excluded_sources": summary_payload[
            "missing_expected_summary_value_source_excluded_sources"
        ],
        "allowed_summary_value_source_excluded_sources": summary_payload[
            "allowed_summary_value_source_excluded_sources"
        ],
        "unexpected_summary_value_source_excluded_sources": summary_payload[
            "unexpected_summary_value_source_excluded_sources"
        ],
        "forbidden_summary_value_source_excluded_sources": summary_payload[
            "forbidden_summary_value_source_excluded_sources"
        ],
        "present_forbidden_summary_value_source_excluded_sources": summary_payload[
            "present_forbidden_summary_value_source_excluded_sources"
        ],
        "summary_value_source_matched_count": summary_payload[
            "summary_value_source_matched_count"
        ],
        "summary_value_source_matched_sources": summary_payload[
            "summary_value_source_matched_sources"
        ],
        "allowed_summary_value_source_matched_sources": summary_payload[
            "allowed_summary_value_source_matched_sources"
        ],
        "unexpected_summary_value_source_matched_sources": summary_payload[
            "unexpected_summary_value_source_matched_sources"
        ],
        "forbidden_summary_value_source_matched_sources": summary_payload[
            "forbidden_summary_value_source_matched_sources"
        ],
        "present_forbidden_summary_value_source_matched_sources": summary_payload[
            "present_forbidden_summary_value_source_matched_sources"
        ],
        "summary_value_source_excluded_count": summary_payload[
            "summary_value_source_excluded_count"
        ],
        "summary_value_source_excluded_sources": summary_payload[
            "summary_value_source_excluded_sources"
        ],
        "actual_expected_summary_values": summary_payload[
            "actual_expected_summary_values"
        ],
        "mismatched_expected_summary_values": summary_payload[
            "mismatched_expected_summary_values"
        ],
        "expected_node_tree_gate_check_counts": summary_payload[
            "expected_node_tree_gate_check_counts"
        ],
        "mismatched_expected_node_tree_gate_check_counts": summary_payload[
            "mismatched_expected_node_tree_gate_check_counts"
        ],
        "expected_mechanical_gate_check_counts": summary_payload[
            "expected_mechanical_gate_check_counts"
        ],
        "mismatched_expected_mechanical_gate_check_counts": summary_payload[
            "mismatched_expected_mechanical_gate_check_counts"
        ],
        "expected_skipped_count": summary_payload["expected_skipped_count"],
        "expected_reason_codes_count": summary_payload[
            "expected_reason_codes_count"
        ],
        "expected_contract_versions_count": summary_payload[
            "expected_contract_versions_count"
        ],
        "expected_summary_versions_count": summary_payload[
            "expected_summary_versions_count"
        ],
        "expected_validation_summary_statuses_count": summary_payload[
            "expected_validation_summary_statuses_count"
        ],
        "expected_levels_count": summary_payload["expected_levels_count"],
        "expected_sources_count": summary_payload["expected_sources_count"],
        "expected_verification_scopes_count": summary_payload[
            "expected_verification_scopes_count"
        ],
        "expected_acceptance_profiles_count": summary_payload[
            "expected_acceptance_profiles_count"
        ],
        "expected_enabled_requirements_count": summary_payload[
            "expected_enabled_requirements_count"
        ],
        "expected_complete_required_summary_fields_source_scope_count": (
            summary_payload[
                "expected_complete_required_summary_fields_source_scope_count"
            ]
        ),
        "expected_affected_inputs_count": summary_payload[
            "expected_affected_inputs_count"
        ],
        "expected_failed_inputs_count": summary_payload[
            "expected_failed_inputs_count"
        ],
        "expected_skipped_inputs_count": summary_payload[
            "expected_skipped_inputs_count"
        ],
        "expected_skipped_reasons_count": summary_payload[
            "expected_skipped_reasons_count"
        ],
        "expected_truncated_previews_count": summary_payload[
            "expected_truncated_previews_count"
        ],
        "expected_contract_versions": summary_payload[
            "expected_contract_versions"
        ],
        "missing_expected_contract_versions": summary_payload[
            "missing_expected_contract_versions"
        ],
        "expected_summary_versions": summary_payload[
            "expected_summary_versions"
        ],
        "missing_expected_summary_versions": summary_payload[
            "missing_expected_summary_versions"
        ],
        "expected_validation_summary_statuses": summary_payload[
            "expected_validation_summary_statuses"
        ],
        "missing_expected_validation_summary_statuses": summary_payload[
            "missing_expected_validation_summary_statuses"
        ],
        "expected_levels": summary_payload["expected_levels"],
        "missing_expected_levels": summary_payload["missing_expected_levels"],
        "expected_sources": summary_payload["expected_sources"],
        "missing_expected_sources": summary_payload["missing_expected_sources"],
        "expected_verification_scopes": summary_payload[
            "expected_verification_scopes"
        ],
        "missing_expected_verification_scopes": summary_payload[
            "missing_expected_verification_scopes"
        ],
        "expected_acceptance_profiles": summary_payload[
            "expected_acceptance_profiles"
        ],
        "missing_expected_acceptance_profiles": summary_payload[
            "missing_expected_acceptance_profiles"
        ],
        "expected_enabled_requirements": summary_payload[
            "expected_enabled_requirements"
        ],
        "missing_expected_enabled_requirements": summary_payload[
            "missing_expected_enabled_requirements"
        ],
        "expected_complete_required_summary_fields_source_scopes": (
            summary_payload[
                "expected_complete_required_summary_fields_source_scopes"
            ]
        ),
        "missing_expected_complete_required_summary_fields_source_scopes": (
            summary_payload[
                "missing_expected_complete_required_summary_fields_source_scopes"
            ]
        ),
        "allowed_complete_required_summary_fields_source_scopes": (
            summary_payload[
                "allowed_complete_required_summary_fields_source_scopes"
            ]
        ),
        "unexpected_complete_required_summary_fields_source_scopes": (
            summary_payload[
                "unexpected_complete_required_summary_fields_source_scopes"
            ]
        ),
        "forbidden_complete_required_summary_fields_source_scopes": (
            summary_payload[
                "forbidden_complete_required_summary_fields_source_scopes"
            ]
        ),
        "present_forbidden_complete_required_summary_fields_source_scopes": (
            summary_payload[
                "present_forbidden_complete_required_summary_fields_source_scopes"
            ]
        ),
        "allowed_contract_versions": summary_payload["allowed_contract_versions"],
        "unexpected_contract_versions": summary_payload[
            "unexpected_contract_versions"
        ],
        "allowed_summary_versions": summary_payload["allowed_summary_versions"],
        "unexpected_summary_versions": summary_payload[
            "unexpected_summary_versions"
        ],
        "allowed_validation_summary_statuses": summary_payload[
            "allowed_validation_summary_statuses"
        ],
        "unexpected_validation_summary_statuses": summary_payload[
            "unexpected_validation_summary_statuses"
        ],
        "allowed_levels": summary_payload["allowed_levels"],
        "unexpected_levels": summary_payload["unexpected_levels"],
        "allowed_sources": summary_payload["allowed_sources"],
        "unexpected_sources": summary_payload["unexpected_sources"],
        "allowed_verification_scopes": summary_payload[
            "allowed_verification_scopes"
        ],
        "unexpected_verification_scopes": summary_payload[
            "unexpected_verification_scopes"
        ],
        "allowed_acceptance_profiles": summary_payload[
            "allowed_acceptance_profiles"
        ],
        "unexpected_acceptance_profiles": summary_payload[
            "unexpected_acceptance_profiles"
        ],
        "allowed_enabled_requirements": summary_payload[
            "allowed_enabled_requirements"
        ],
        "unexpected_enabled_requirements": summary_payload[
            "unexpected_enabled_requirements"
        ],
        "forbidden_sources": summary_payload["forbidden_sources"],
        "present_forbidden_sources": summary_payload[
            "present_forbidden_sources"
        ],
        "forbidden_verification_scopes": summary_payload[
            "forbidden_verification_scopes"
        ],
        "present_forbidden_verification_scopes": summary_payload[
            "present_forbidden_verification_scopes"
        ],
        "forbidden_acceptance_profiles": summary_payload[
            "forbidden_acceptance_profiles"
        ],
        "present_forbidden_acceptance_profiles": summary_payload[
            "present_forbidden_acceptance_profiles"
        ],
        "forbidden_enabled_requirements": summary_payload[
            "forbidden_enabled_requirements"
        ],
        "present_forbidden_enabled_requirements": summary_payload[
            "present_forbidden_enabled_requirements"
        ],
        "forbidden_contract_versions": summary_payload[
            "forbidden_contract_versions"
        ],
        "present_forbidden_contract_versions": summary_payload[
            "present_forbidden_contract_versions"
        ],
        "forbidden_summary_versions": summary_payload[
            "forbidden_summary_versions"
        ],
        "present_forbidden_summary_versions": summary_payload[
            "present_forbidden_summary_versions"
        ],
        "forbidden_validation_summary_statuses": summary_payload[
            "forbidden_validation_summary_statuses"
        ],
        "present_forbidden_validation_summary_statuses": summary_payload[
            "present_forbidden_validation_summary_statuses"
        ],
        "forbidden_levels": summary_payload["forbidden_levels"],
        "present_forbidden_levels": summary_payload["present_forbidden_levels"],
        "expected_reason_codes": summary_payload["expected_reason_codes"],
        "missing_expected_reason_codes": summary_payload[
            "missing_expected_reason_codes"
        ],
        "allowed_reason_codes": summary_payload["allowed_reason_codes"],
        "unexpected_reason_codes": summary_payload["unexpected_reason_codes"],
        "forbidden_reason_codes": summary_payload["forbidden_reason_codes"],
        "present_forbidden_reason_codes": summary_payload[
            "present_forbidden_reason_codes"
        ],
        "expected_skipped_reasons": summary_payload["expected_skipped_reasons"],
        "missing_expected_skipped_reasons": summary_payload[
            "missing_expected_skipped_reasons"
        ],
        "allowed_skipped_reasons": summary_payload["allowed_skipped_reasons"],
        "unexpected_skipped_reasons": summary_payload[
            "unexpected_skipped_reasons"
        ],
        "forbidden_skipped_reasons": summary_payload["forbidden_skipped_reasons"],
        "present_forbidden_skipped_reasons": summary_payload[
            "present_forbidden_skipped_reasons"
        ],
        "excluded_output_artifact_inputs": summary_payload[
            "excluded_output_artifact_inputs"
        ],
        "excluded_output_artifact_inputs_count": summary_payload[
            "excluded_output_artifact_inputs_count"
        ],
        "errors": summary_payload["errors"],
        "contract_versions": summary_payload["contract_versions"],
        "contract_versions_count": summary_payload["contract_versions_count"],
        "contract_versions_truncated": summary_payload[
            "contract_versions_truncated"
        ],
        "summary_versions": summary_payload["summary_versions"],
        "summary_versions_count": summary_payload["summary_versions_count"],
        "summary_versions_truncated": summary_payload[
            "summary_versions_truncated"
        ],
        "validation_summary_statuses": summary_payload[
            "validation_summary_statuses"
        ],
        "validation_summary_statuses_count": summary_payload[
            "validation_summary_statuses_count"
        ],
        "validation_summary_statuses_truncated": summary_payload[
            "validation_summary_statuses_truncated"
        ],
        "levels": summary_payload["levels"],
        "levels_count": summary_payload["levels_count"],
        "levels_truncated": summary_payload["levels_truncated"],
        "sources": summary_payload["sources"],
        "sources_count": summary_payload["sources_count"],
        "sources_truncated": summary_payload["sources_truncated"],
        "verification_scopes": summary_payload["verification_scopes"],
        "verification_scopes_count": summary_payload["verification_scopes_count"],
        "verification_scopes_truncated": summary_payload[
            "verification_scopes_truncated"
        ],
        "acceptance_profiles": summary_payload["acceptance_profiles"],
        "acceptance_profiles_count": summary_payload["acceptance_profiles_count"],
        "acceptance_profiles_truncated": summary_payload[
            "acceptance_profiles_truncated"
        ],
        "enabled_requirements": summary_payload["enabled_requirements"],
        "enabled_requirements_count": summary_payload[
            "enabled_requirements_count"
        ],
        "enabled_requirements_truncated": summary_payload[
            "enabled_requirements_truncated"
        ],
        "reason_codes": summary_payload["reason_codes"],
        "reason_codes_count": summary_payload["reason_codes_count"],
        "reason_codes_truncated": summary_payload["reason_codes_truncated"],
        "affected_inputs": summary_payload["affected_inputs"],
        "affected_inputs_count": summary_payload["affected_inputs_count"],
        "affected_inputs_truncated": summary_payload[
            "affected_inputs_truncated"
        ],
        "failed_inputs": summary_payload["failed_inputs"],
        "failed_inputs_count": summary_payload["failed_inputs_count"],
        "failed_inputs_truncated": summary_payload["failed_inputs_truncated"],
        "skipped_inputs": summary_payload["skipped_inputs"],
        "skipped_inputs_count": summary_payload["skipped_inputs_count"],
        "skipped_inputs_truncated": summary_payload["skipped_inputs_truncated"],
        "skipped_reasons": summary_payload["skipped_reasons"],
        "skipped_reasons_count": summary_payload["skipped_reasons_count"],
        "skipped_reasons_truncated": summary_payload["skipped_reasons_truncated"],
        "results": results,
    }


def _has_validation_summary_context(
    *,
    results: list[dict[str, Any]],
    summary_payload: dict[str, Any],
    aggregate_errors: list[str],
) -> bool:
    has_expected_counts = any(
        summary_payload.get(key) is not None
        for key in (
            "expected_inputs_count",
            "expected_expanded_inputs_count",
            "expected_excluded_output_artifacts_count",
            "expected_success_count",
            "expected_error_count",
            "expected_passed_true_count",
            "expected_passed_false_count",
            "expected_passed_unknown_count",
            "expected_full_mechanical_gate_true_count",
            "expected_full_mechanical_gate_false_count",
            "expected_full_mechanical_gate_unknown_count",
            "expected_smoke_report_written_count",
            "expected_smoke_report_missing_count",
            "expected_smoke_report_read_error_count",
            "expected_control_configured_count",
            "expected_control_readback_checked_count",
            "expected_control_readback_missing_count",
            "expected_node_tree_manifest_sidecar_count",
            "expected_node_tree_manifest_sidecar_complete_count",
            "expected_node_tree_manifest_sidecar_incomplete_count",
            "expected_node_tree_manifest_sidecar_valid_count",
            "expected_node_tree_manifest_sidecar_invalid_count",
            "expected_node_tree_manifest_sidecar_validation_error_count",
            "expected_node_tree_manifest_sidecar_path_incomplete_count",
            "expected_node_tree_manifest_sidecar_path_map_mismatch_count",
            "expected_node_tree_manifest_sidecar_parts_planned_count",
            "expected_node_tree_manifest_sidecar_joints_planned_count",
            "expected_node_tree_manifest_sidecar_part_path_count",
            "expected_node_tree_manifest_sidecar_joint_path_count",
            "expected_summary_value_source_matched_count",
            "expected_summary_value_source_excluded_count",
            "expected_skipped_count",
            "expected_reason_codes_count",
            "expected_contract_versions_count",
            "expected_levels_count",
            "expected_sources_count",
            "expected_verification_scopes_count",
            "expected_acceptance_profiles_count",
            "expected_enabled_requirements_count",
            "expected_complete_required_summary_fields_source_scope_count",
            "expected_affected_inputs_count",
            "expected_failed_inputs_count",
            "expected_skipped_inputs_count",
            "expected_skipped_reasons_count",
            "expected_truncated_previews_count",
        )
    )
    has_expected_reason_codes = bool(summary_payload.get("expected_reason_codes"))
    has_expected_inputs = bool(summary_payload.get("expected_inputs"))
    has_expected_truncated_previews = bool(
        summary_payload.get("expected_truncated_previews")
    )
    has_expected_affected_inputs = bool(
        summary_payload.get("expected_affected_inputs")
    )
    has_expected_failed_inputs = bool(summary_payload.get("expected_failed_inputs"))
    has_expected_passed_true_inputs = bool(
        summary_payload.get("expected_passed_true_inputs")
    )
    has_expected_passed_false_inputs = bool(
        summary_payload.get("expected_passed_false_inputs")
    )
    has_expected_passed_unknown_inputs = bool(
        summary_payload.get("expected_passed_unknown_inputs")
    )
    has_expected_full_mechanical_gate_true_inputs = bool(
        summary_payload.get("expected_full_mechanical_gate_true_inputs")
    )
    has_expected_full_mechanical_gate_false_inputs = bool(
        summary_payload.get("expected_full_mechanical_gate_false_inputs")
    )
    has_expected_full_mechanical_gate_unknown_inputs = bool(
        summary_payload.get("expected_full_mechanical_gate_unknown_inputs")
    )
    has_expected_skipped_inputs = bool(summary_payload.get("expected_skipped_inputs"))
    has_expected_gate_check_counts = bool(
        summary_payload.get("expected_node_tree_gate_check_counts")
        or summary_payload.get("expected_mechanical_gate_check_counts")
    )
    has_expected_summary_counts = bool(
        summary_payload.get("expected_summary_counts")
    )
    has_expected_summary_values = bool(
        summary_payload.get("expected_summary_values")
    )
    has_expected_path_mismatch_kind_counts = bool(
        summary_payload.get(
            "expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts"
        )
    )
    has_expected_metadata = (
        bool(summary_payload.get("expected_contract_versions"))
        or bool(summary_payload.get("expected_levels"))
        or bool(summary_payload.get("expected_sources"))
        or bool(summary_payload.get("expected_verification_scopes"))
        or bool(summary_payload.get("expected_acceptance_profiles"))
        or bool(summary_payload.get("expected_enabled_requirements"))
        or bool(
            summary_payload.get(
                "expected_complete_required_summary_fields_source_scopes"
            )
        )
    )
    has_allowed_metadata = (
        bool(summary_payload.get("allowed_contract_versions"))
        or bool(summary_payload.get("allowed_levels"))
        or bool(summary_payload.get("allowed_sources"))
        or bool(summary_payload.get("allowed_verification_scopes"))
        or bool(summary_payload.get("allowed_acceptance_profiles"))
        or bool(summary_payload.get("allowed_enabled_requirements"))
        or bool(
            summary_payload.get(
                "allowed_complete_required_summary_fields_source_scopes"
            )
        )
    )
    has_forbidden_metadata = (
        bool(summary_payload.get("forbidden_contract_versions"))
        or bool(summary_payload.get("forbidden_levels"))
        or bool(summary_payload.get("forbidden_sources"))
        or bool(summary_payload.get("forbidden_verification_scopes"))
        or bool(summary_payload.get("forbidden_acceptance_profiles"))
        or bool(summary_payload.get("forbidden_enabled_requirements"))
        or bool(
            summary_payload.get(
                "forbidden_complete_required_summary_fields_source_scopes"
            )
        )
    )
    has_allowed_reason_codes = bool(summary_payload.get("allowed_reason_codes"))
    has_forbidden_reason_codes = bool(summary_payload.get("forbidden_reason_codes"))
    has_allowed_inputs = bool(summary_payload.get("allowed_inputs"))
    has_forbidden_inputs = bool(summary_payload.get("forbidden_inputs"))
    has_allowed_truncated_previews = bool(
        summary_payload.get("allowed_truncated_previews")
    )
    has_allowed_affected_inputs = bool(
        summary_payload.get("allowed_affected_inputs")
    )
    has_allowed_failed_inputs = bool(summary_payload.get("allowed_failed_inputs"))
    has_allowed_passed_true_inputs = bool(
        summary_payload.get("allowed_passed_true_inputs")
    )
    has_allowed_passed_false_inputs = bool(
        summary_payload.get("allowed_passed_false_inputs")
    )
    has_allowed_passed_unknown_inputs = bool(
        summary_payload.get("allowed_passed_unknown_inputs")
    )
    has_allowed_full_mechanical_gate_true_inputs = bool(
        summary_payload.get("allowed_full_mechanical_gate_true_inputs")
    )
    has_allowed_full_mechanical_gate_false_inputs = bool(
        summary_payload.get("allowed_full_mechanical_gate_false_inputs")
    )
    has_allowed_full_mechanical_gate_unknown_inputs = bool(
        summary_payload.get("allowed_full_mechanical_gate_unknown_inputs")
    )
    has_allowed_skipped_inputs = bool(summary_payload.get("allowed_skipped_inputs"))
    has_forbidden_truncated_previews = bool(
        summary_payload.get("forbidden_truncated_previews")
    )
    has_forbidden_affected_inputs = bool(
        summary_payload.get("forbidden_affected_inputs")
    )
    has_forbidden_failed_inputs = bool(
        summary_payload.get("forbidden_failed_inputs")
    )
    has_forbidden_passed_true_inputs = bool(
        summary_payload.get("forbidden_passed_true_inputs")
    )
    has_forbidden_passed_false_inputs = bool(
        summary_payload.get("forbidden_passed_false_inputs")
    )
    has_forbidden_passed_unknown_inputs = bool(
        summary_payload.get("forbidden_passed_unknown_inputs")
    )
    has_forbidden_full_mechanical_gate_true_inputs = bool(
        summary_payload.get("forbidden_full_mechanical_gate_true_inputs")
    )
    has_forbidden_full_mechanical_gate_false_inputs = bool(
        summary_payload.get("forbidden_full_mechanical_gate_false_inputs")
    )
    has_forbidden_full_mechanical_gate_unknown_inputs = bool(
        summary_payload.get("forbidden_full_mechanical_gate_unknown_inputs")
    )
    has_forbidden_skipped_inputs = bool(
        summary_payload.get("forbidden_skipped_inputs")
    )
    has_expected_skipped_reasons = bool(
        summary_payload.get("expected_skipped_reasons")
    )
    has_allowed_skipped_reasons = bool(summary_payload.get("allowed_skipped_reasons"))
    has_forbidden_skipped_reasons = bool(
        summary_payload.get("forbidden_skipped_reasons")
    )
    has_excluded_output_artifacts = bool(
        summary_payload.get("excluded_output_artifact_inputs")
    )
    has_fail_on_full_mechanical_gate = bool(
        summary_payload.get("fail_on_full_mechanical_gate_false")
        or summary_payload.get("fail_on_full_mechanical_gate_unknown")
    )
    has_fail_on_smoke_report_policy = bool(
        summary_payload.get("fail_on_smoke_report_missing")
        or summary_payload.get("fail_on_smoke_report_read_error")
    )
    has_fail_on_control_readback_policy = bool(
        summary_payload.get("fail_on_control_readback_missing")
    )
    return (
        len(results) > 1
        or bool(aggregate_errors)
        or has_expected_counts
        or has_expected_reason_codes
        or has_expected_inputs
        or has_expected_truncated_previews
        or has_expected_affected_inputs
        or has_expected_failed_inputs
        or has_expected_passed_true_inputs
        or has_expected_passed_false_inputs
        or has_expected_passed_unknown_inputs
        or has_expected_full_mechanical_gate_true_inputs
        or has_expected_full_mechanical_gate_false_inputs
        or has_expected_full_mechanical_gate_unknown_inputs
        or has_expected_skipped_inputs
        or has_expected_gate_check_counts
        or has_expected_summary_counts
        or has_expected_summary_values
        or has_expected_path_mismatch_kind_counts
        or has_expected_metadata
        or has_allowed_metadata
        or has_forbidden_metadata
        or has_allowed_reason_codes
        or has_forbidden_reason_codes
        or has_allowed_inputs
        or has_forbidden_inputs
        or has_allowed_truncated_previews
        or has_forbidden_truncated_previews
        or has_allowed_affected_inputs
        or has_forbidden_affected_inputs
        or has_allowed_failed_inputs
        or has_forbidden_failed_inputs
        or has_allowed_passed_true_inputs
        or has_forbidden_passed_true_inputs
        or has_allowed_passed_false_inputs
        or has_forbidden_passed_false_inputs
        or has_allowed_passed_unknown_inputs
        or has_forbidden_passed_unknown_inputs
        or has_allowed_full_mechanical_gate_true_inputs
        or has_forbidden_full_mechanical_gate_true_inputs
        or has_allowed_full_mechanical_gate_false_inputs
        or has_forbidden_full_mechanical_gate_false_inputs
        or has_allowed_full_mechanical_gate_unknown_inputs
        or has_forbidden_full_mechanical_gate_unknown_inputs
        or has_allowed_skipped_inputs
        or has_forbidden_skipped_inputs
        or has_expected_skipped_reasons
        or has_allowed_skipped_reasons
        or has_forbidden_skipped_reasons
        or has_excluded_output_artifacts
        or has_fail_on_full_mechanical_gate
        or has_fail_on_smoke_report_policy
        or has_fail_on_control_readback_policy
    )


def _show_passed_counts(args: argparse.Namespace) -> bool:
    return (
        bool(args.show_diagnostics)
        or bool(args.show_passed_counts)
        or bool(args.fail_on_passed_false)
        or bool(args.fail_on_passed_unknown)
    )


def _should_emit_text_summary(
    *,
    args: argparse.Namespace,
    results: list[dict[str, Any]],
    summary_payload: dict[str, Any],
    aggregate_errors: list[str],
) -> bool:
    return (
        _show_passed_counts(args)
        or bool(args.show_metadata)
        or bool(args.show_reason_codes)
        or bool(args.show_inputs)
        or bool(args.show_skipped_reasons)
        or bool(args.show_diagnostics)
        or bool(args.summary_only)
        or bool(args.fail_on_full_mechanical_gate_false)
        or bool(args.fail_on_full_mechanical_gate_unknown)
        or bool(args.fail_on_smoke_report_missing)
        or bool(args.fail_on_smoke_report_read_error)
        or bool(args.fail_on_control_readback_missing)
        or _has_validation_summary_context(
            results=results,
            summary_payload=summary_payload,
            aggregate_errors=aggregate_errors,
        )
    )


def _validate_input(
    *,
    input_path: Path,
    workflow_contracts: Any,
    robot_schema: Any,
    require_passed: bool,
    require_required: bool,
    require_complete: bool,
    require_full_mechanical_restoration_gate: bool,
    ignore_non_gate: bool,
    validate_validation_summary: bool = False,
) -> dict[str, Any]:
    try:
        raw_payload = input_path.read_text(encoding="utf-8")
    except OSError as exc:
        return _input_error_result(
            input_path=input_path,
            require_passed=require_passed,
            require_required=require_required,
            require_complete=require_complete,
            require_full_mechanical_restoration_gate=(
                require_full_mechanical_restoration_gate
            ),
            error=f"unable to read input: {exc}",
        )
    try:
        payload = json.loads(raw_payload)
    except json.JSONDecodeError as exc:
        return _input_error_result(
            input_path=input_path,
            require_passed=require_passed,
            require_required=require_required,
            require_complete=require_complete,
            require_full_mechanical_restoration_gate=(
                require_full_mechanical_restoration_gate
            ),
            error=f"invalid JSON: line {exc.lineno} column {exc.colno}: {exc.msg}",
        )
    if validate_validation_summary:
        if ignore_non_gate and not _looks_like_validation_summary_payload(
            payload,
            workflow_contracts=workflow_contracts,
        ):
            return {
                "status": "skipped",
                "input": str(input_path),
                "gate_source_path": None,
                "contract_version": None,
                "source": None,
                "verification_scope": None,
                "acceptance_profile": None,
                "level": None,
                "required": None,
                "complete": None,
                "requires_full_mechanical_restoration_gate": None,
                "passed": None,
                "exit_code": None,
                "reason_codes": [],
                "reasons_count": 0,
                "enabled_requirements": [],
                "affected_inputs": [],
                "affected_inputs_count": 0,
                "affected_inputs_truncated": False,
                "summary_counts": {},
                "complete_required_summary_fields": [],
                "complete_required_summary_fields_count": 0,
                "require_passed": bool(require_passed),
                "require_required": bool(require_required),
                "require_complete": bool(require_complete),
                "require_full_mechanical_restoration_gate": bool(
                    require_full_mechanical_restoration_gate
                ),
                "errors": [],
                "skip_reason": (
                    "not a delivery_acceptance_gate validation summary artifact"
                ),
            }
        errors = workflow_contracts.validate_delivery_acceptance_validation_summary(
            payload
        )
        summary_errors = payload.get("errors") if isinstance(payload, dict) else []
        return {
            "status": "error" if errors else "success",
            "input": str(input_path),
            "gate_source_path": "validation_summary",
            "contract_version": None,
            "summary_version": payload.get("summary_version")
            if isinstance(payload, dict)
            else None,
            "source": None,
            "verification_scope": None,
            "acceptance_profile": None,
            "level": None,
            "required": None,
            "complete": None,
            "requires_full_mechanical_restoration_gate": None,
            "passed": None,
            "exit_code": None,
            "reason_codes": [],
            "reasons_count": 0,
            "enabled_requirements": [],
            "affected_inputs": [],
            "affected_inputs_count": 0,
            "affected_inputs_truncated": False,
            "summary_counts": {},
            "complete_required_summary_fields": [],
            "complete_required_summary_fields_count": 0,
            "validation_summary_status": payload.get("status")
            if isinstance(payload, dict)
            else None,
            "validation_summary_errors_count": len(summary_errors)
            if isinstance(summary_errors, list)
            else None,
            "require_passed": bool(require_passed),
            "require_required": bool(require_required),
            "require_complete": bool(require_complete),
            "require_full_mechanical_restoration_gate": bool(
                require_full_mechanical_restoration_gate
            ),
            "errors": errors,
        }
    if _looks_like_godot_node_tree_manifest_payload(payload):
        node_tree_manifest_validation_errors = (
            robot_schema.validate_godot_node_tree_manifest(payload)
        )
        node_tree_manifest_path_map_mismatches = (
            robot_schema.build_godot_node_tree_manifest_path_map_mismatches(payload)
        )
        return {
            "status": "skipped",
            "input": str(input_path),
            "gate_source_path": None,
            "contract_version": None,
            "source": None,
            "verification_scope": None,
            "acceptance_profile": None,
            "level": None,
            "required": None,
            "complete": None,
            "requires_full_mechanical_restoration_gate": None,
            "passed": None,
            "exit_code": None,
            "reason_codes": [],
            "reasons_count": 0,
            "enabled_requirements": [],
            "affected_inputs": [],
            "affected_inputs_count": 0,
            "affected_inputs_truncated": False,
            "summary_counts": {},
            "complete_required_summary_fields": [],
            "complete_required_summary_fields_count": 0,
            "require_passed": bool(require_passed),
            "require_required": bool(require_required),
            "require_complete": bool(require_complete),
            "require_full_mechanical_restoration_gate": bool(
                require_full_mechanical_restoration_gate
            ),
            "errors": [],
            "skip_reason": "static Godot node-tree manifest sidecar",
            "node_tree_manifest_summary": _godot_node_tree_manifest_summary(payload),
            "node_tree_manifest_valid": not node_tree_manifest_validation_errors,
            "node_tree_manifest_validation_errors": (
                node_tree_manifest_validation_errors
            ),
            "node_tree_manifest_path_map_mismatches": (
                node_tree_manifest_path_map_mismatches
            ),
            "node_tree_manifest_path_map_mismatch_count": len(
                node_tree_manifest_path_map_mismatches
            ),
            "node_tree_manifest_path_map_mismatch_kind_counts": (
                _node_tree_manifest_path_map_mismatch_kind_counts(
                    node_tree_manifest_path_map_mismatches
                )
            ),
        }
    if ignore_non_gate and not _looks_like_gate_or_report_payload(
        payload,
        workflow_contracts=workflow_contracts,
    ):
        return {
            "status": "skipped",
            "input": str(input_path),
            "gate_source_path": None,
            "contract_version": None,
            "source": None,
            "verification_scope": None,
            "acceptance_profile": None,
            "level": None,
            "required": None,
            "complete": None,
            "requires_full_mechanical_restoration_gate": None,
            "passed": None,
            "exit_code": None,
            "reason_codes": [],
            "reasons_count": 0,
            "enabled_requirements": [],
            "affected_inputs": [],
            "affected_inputs_count": 0,
            "affected_inputs_truncated": False,
            "summary_counts": {},
            "complete_required_summary_fields": [],
            "complete_required_summary_fields_count": 0,
            "require_passed": bool(require_passed),
            "require_required": bool(require_required),
            "require_complete": bool(require_complete),
            "require_full_mechanical_restoration_gate": bool(
                require_full_mechanical_restoration_gate
            ),
            "errors": [],
            "skip_reason": "not a delivery_acceptance_gate artifact or report",
        }
    gate_payload, gate_source_path = _extract_gate_payload(payload)
    errors = workflow_contracts.validate_delivery_acceptance_gate(gate_payload)
    required = (
        gate_payload.get("required") if isinstance(gate_payload, dict) else None
    )
    complete = (
        gate_payload.get("complete") if isinstance(gate_payload, dict) else None
    )
    requires_full_mechanical_gate = (
        gate_payload.get("requires_full_mechanical_restoration_gate")
        if isinstance(gate_payload, dict)
        else None
    )
    passed = gate_payload.get("passed") if isinstance(gate_payload, dict) else None
    exit_code = gate_payload.get("exit_code") if isinstance(gate_payload, dict) else None
    reason_codes = (
        gate_payload.get("reason_codes", []) if isinstance(gate_payload, dict) else []
    )
    reasons = gate_payload.get("reasons", []) if isinstance(gate_payload, dict) else []
    affected_inputs, affected_inputs_count = _affected_inputs(gate_payload)
    complete_required_summary_fields = _complete_required_summary_fields(
        gate_payload,
        workflow_contracts=workflow_contracts,
    )
    if require_passed and not errors and passed is not True:
        errors.append("delivery_acceptance_gate passed must be true")
    if require_required and not errors and required is not True:
        errors.append("delivery_acceptance_gate required must be true")
    if require_complete and not errors and complete is not True:
        errors.append("delivery_acceptance_gate complete must be true")
    if (
        require_full_mechanical_restoration_gate
        and not errors
        and requires_full_mechanical_gate is not True
    ):
        errors.append(
            "delivery_acceptance_gate "
            "requires_full_mechanical_restoration_gate must be true"
        )

    return {
        "status": "error" if errors else "success",
        "input": str(input_path),
        "gate_source_path": gate_source_path,
        "contract_version": gate_payload.get("contract_version")
        if isinstance(gate_payload, dict)
        else None,
        "source": gate_payload.get("source") if isinstance(gate_payload, dict) else None,
        "verification_scope": gate_payload.get("verification_scope")
        if isinstance(gate_payload, dict)
        else None,
        "acceptance_profile": gate_payload.get("acceptance_profile")
        if isinstance(gate_payload, dict)
        else None,
        "level": gate_payload.get("level") if isinstance(gate_payload, dict) else None,
        "required": required,
        "complete": complete,
        "requires_full_mechanical_restoration_gate": (
            requires_full_mechanical_gate
        ),
        "passed": passed,
        "exit_code": exit_code,
        "reason_codes": reason_codes if isinstance(reason_codes, list) else [],
        "reasons_count": len(reasons) if isinstance(reasons, list) else None,
        "enabled_requirements": _enabled_requirements(gate_payload),
        "affected_inputs": affected_inputs,
        "affected_inputs_count": affected_inputs_count,
        "affected_inputs_truncated": affected_inputs_count > len(affected_inputs),
        "summary_counts": _compact_summary_counts(gate_payload),
        "complete_required_summary_fields": complete_required_summary_fields,
        "complete_required_summary_fields_count": len(
            complete_required_summary_fields
        ),
        "require_passed": bool(require_passed),
        "require_required": bool(require_required),
        "require_complete": bool(require_complete),
        "require_full_mechanical_restoration_gate": bool(
            require_full_mechanical_restoration_gate
        ),
        "errors": errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate a delivery_acceptance_gate JSON artifact."
    )
    parser.add_argument(
        "inputs",
        nargs="*",
        help="One or more delivery_acceptance_gate JSON files or full reports.",
    )
    parser.add_argument(
        "--print-contract-schema",
        action="store_true",
        help=(
            "Print the machine-readable delivery_acceptance_gate contract schema "
            "as JSON and exit without validating inputs."
        ),
    )
    parser.add_argument(
        "--require-passed",
        action="store_true",
        help="Also fail when the gate is valid but passed is not true.",
    )
    parser.add_argument(
        "--require-required",
        action="store_true",
        help="Also fail when the gate is valid but required is not true.",
    )
    parser.add_argument(
        "--require-complete",
        action="store_true",
        help="Also fail when the gate is valid but complete is not true.",
    )
    parser.add_argument(
        "--require-full-mechanical-restoration-gate",
        action="store_true",
        help=(
            "Also fail when the gate is valid but "
            "requires_full_mechanical_restoration_gate is not true."
        ),
    )
    parser.add_argument(
        "--format",
        choices=["json", "text"],
        default="json",
        help="Output JSON details or stable text summary line(s).",
    )
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="Print only the aggregate summary: one summary line in text mode or compact summary JSON in JSON mode.",
    )
    parser.add_argument(
        "--preview-limit",
        type=_nonnegative_int,
        default=20,
        help="Maximum number of unique aggregate preview values to include in JSON and text summaries.",
    )
    parser.add_argument(
        "--fail-on-truncated-previews",
        action="store_true",
        help="Fail when any aggregate preview list is truncated by --preview-limit.",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Recursively expand directory inputs as *.json. Directories are non-recursive by default.",
    )
    parser.add_argument(
        "--ignore-non-gate",
        action="store_true",
        help="Skip JSON inputs that are not delivery_acceptance_gate artifacts or full reports.",
    )
    parser.add_argument(
        "--validate-validation-summary",
        action="store_true",
        help="Validate compact delivery_acceptance_gate validation-summary artifacts instead of gate/report artifacts.",
    )
    parser.add_argument(
        "--allow-empty",
        action="store_true",
        help="When --ignore-non-gate skips every input, exit successfully instead of failing closed.",
    )
    parser.add_argument(
        "--fail-on-skipped",
        action="store_true",
        help="Fail when any input is skipped, while preserving skipped result status and counts.",
    )
    parser.add_argument(
        "--fail-on-passed-false",
        action="store_true",
        help="Fail when any valid gate/report has passed=false, while preserving per-input result status.",
    )
    parser.add_argument(
        "--fail-on-passed-unknown",
        action="store_true",
        help="Fail when any input has a missing or non-boolean passed field.",
    )
    parser.add_argument(
        "--fail-on-full-mechanical-gate-false",
        action="store_true",
        help="Fail when any input has requires_full_mechanical_restoration_gate=false.",
    )
    parser.add_argument(
        "--fail-on-full-mechanical-gate-unknown",
        action="store_true",
        help="Fail when any input has a missing or non-boolean requires_full_mechanical_restoration_gate field.",
    )
    parser.add_argument(
        "--fail-on-smoke-report-missing",
        action="store_true",
        help="Fail when gate summary_counts report any missing Godot smoke report output.",
    )
    parser.add_argument(
        "--fail-on-smoke-report-read-error",
        action="store_true",
        help="Fail when gate summary_counts report any unreadable Godot smoke report output.",
    )
    parser.add_argument(
        "--fail-on-control-readback-missing",
        action="store_true",
        help="Fail when gate summary_counts report missing generated control readback metadata.",
    )
    parser.add_argument(
        "--fail-on-node-tree-manifest-sidecar-incomplete",
        action="store_true",
        help="Fail when scanned static Godot node-tree manifest sidecars report incomplete static node trees.",
    )
    parser.add_argument(
        "--fail-on-invalid-node-tree-manifest-sidecar",
        action="store_true",
        help="Fail when scanned static Godot node-tree manifest sidecars fail structural validation.",
    )
    parser.add_argument(
        "--fail-on-node-tree-manifest-sidecar-validation-error",
        action="store_true",
        help="Fail when scanned static Godot node-tree manifest sidecars report any structural validation errors.",
    )
    parser.add_argument(
        "--fail-on-node-tree-manifest-sidecar-path-incomplete",
        action="store_true",
        help="Fail when scanned static Godot node-tree manifest sidecars do not provide part/joint path maps for every planned part/joint.",
    )
    parser.add_argument(
        "--fail-on-node-tree-manifest-sidecar-path-map-mismatch",
        action="store_true",
        help="Fail when scanned static Godot node-tree manifest sidecars contain path maps that differ from planned node paths.",
    )
    parser.add_argument(
        "--show-passed-counts",
        action="store_true",
        help="In text mode, print aggregate passed=true/false/unknown counts without adding a failure condition.",
    )
    parser.add_argument(
        "--show-diagnostics",
        action="store_true",
        help="In text mode, print all aggregate diagnostic previews without adding a failure condition.",
    )
    parser.add_argument(
        "--show-metadata",
        action="store_true",
        help="In text mode, print aggregate source, verification_scope, and acceptance_profile values without adding a failure condition.",
    )
    parser.add_argument(
        "--show-reason-codes",
        action="store_true",
        help="In text mode, print aggregate reason_codes without adding a failure condition.",
    )
    parser.add_argument(
        "--show-inputs",
        action="store_true",
        help="In text mode, print aggregate affected, failed, and skipped input previews without adding a failure condition.",
    )
    parser.add_argument(
        "--show-skipped-reasons",
        action="store_true",
        help="In text mode, print aggregate skipped input reasons without adding a failure condition.",
    )
    parser.add_argument(
        "--expect-inputs-count",
        type=_nonnegative_int,
        help="Fail unless the post-exclusion validation input count equals this value.",
    )
    parser.add_argument(
        "--expect-expanded-inputs-count",
        type=_nonnegative_int,
        help="Fail unless the directory-expanded input count before output artifact exclusions equals this value.",
    )
    parser.add_argument(
        "--expect-excluded-output-artifacts-count",
        type=_nonnegative_int,
        help="Fail unless the output artifact exclusion count equals this value.",
    )
    parser.add_argument(
        "--expect-success-count",
        type=_nonnegative_int,
        help="Fail unless the successful gate/report validation count equals this value.",
    )
    parser.add_argument(
        "--expect-error-count",
        type=_nonnegative_int,
        help="Fail unless the validation error count equals this value.",
    )
    parser.add_argument(
        "--expect-passed-true-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate passed=true gate count equals this value.",
    )
    parser.add_argument(
        "--expect-passed-false-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate passed=false gate count equals this value.",
    )
    parser.add_argument(
        "--expect-passed-unknown-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate missing/non-boolean passed count equals this value.",
    )
    parser.add_argument(
        "--expect-full-mechanical-gate-true-count",
        type=_nonnegative_int,
        help="Fail unless requires_full_mechanical_restoration_gate=true count equals this value.",
    )
    parser.add_argument(
        "--expect-full-mechanical-gate-false-count",
        type=_nonnegative_int,
        help="Fail unless requires_full_mechanical_restoration_gate=false count equals this value.",
    )
    parser.add_argument(
        "--expect-full-mechanical-gate-unknown-count",
        type=_nonnegative_int,
        help="Fail unless missing/non-boolean requires_full_mechanical_restoration_gate count equals this value.",
    )
    parser.add_argument(
        "--expect-smoke-report-written-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate smoke_report_written_count in gate summary_counts equals this value.",
    )
    parser.add_argument(
        "--expect-smoke-report-missing-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate smoke_report_missing_count in gate summary_counts equals this value.",
    )
    parser.add_argument(
        "--expect-smoke-report-read-error-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate smoke_report_read_error_count in gate summary_counts equals this value.",
    )
    parser.add_argument(
        "--expect-control-configured-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate control_configured_count in gate summary_counts equals this value.",
    )
    parser.add_argument(
        "--expect-control-readback-checked-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate control_readback_checked_count in gate summary_counts equals this value.",
    )
    parser.add_argument(
        "--expect-control-readback-missing-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate control_readback_missing_count in gate summary_counts equals this value.",
    )
    parser.add_argument(
        "--expect-node-tree-manifest-sidecar-count",
        type=_nonnegative_int,
        help="Fail unless the scanned static Godot node-tree manifest sidecar count equals this value.",
    )
    parser.add_argument(
        "--expect-node-tree-manifest-sidecar-complete-count",
        type=_nonnegative_int,
        help="Fail unless the scanned static Godot node-tree manifest sidecar complete count equals this value.",
    )
    parser.add_argument(
        "--expect-node-tree-manifest-sidecar-incomplete-count",
        type=_nonnegative_int,
        help="Fail unless the scanned static Godot node-tree manifest sidecar incomplete count equals this value.",
    )
    parser.add_argument(
        "--expect-node-tree-manifest-sidecar-valid-count",
        type=_nonnegative_int,
        help="Fail unless the scanned static Godot node-tree manifest sidecar structural-valid count equals this value.",
    )
    parser.add_argument(
        "--expect-node-tree-manifest-sidecar-invalid-count",
        type=_nonnegative_int,
        help="Fail unless the scanned static Godot node-tree manifest sidecar structural-invalid count equals this value.",
    )
    parser.add_argument(
        "--expect-node-tree-manifest-sidecar-validation-error-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate static Godot node-tree manifest sidecar structural validation error count equals this value.",
    )
    parser.add_argument(
        "--expect-node-tree-manifest-sidecar-path-incomplete-count",
        type=_nonnegative_int,
        help="Fail unless the scanned static Godot node-tree manifest sidecar path-incomplete count equals this value.",
    )
    parser.add_argument(
        "--expect-node-tree-manifest-sidecar-path-map-mismatch-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate static Godot node-tree manifest sidecar path-map mismatch count equals this value.",
    )
    parser.add_argument(
        "--expect-node-tree-manifest-sidecar-path-map-mismatch-kind",
        action="append",
        default=[],
        type=_mismatch_kind_count_text,
        metavar="KIND=COUNT",
        help="Fail unless the aggregate static Godot node-tree manifest sidecar path-map mismatch count for KIND equals COUNT. May be repeated.",
    )
    parser.add_argument(
        "--expect-node-tree-manifest-sidecar-parts-planned-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate planned part count across static Godot node-tree manifest sidecars equals this value.",
    )
    parser.add_argument(
        "--expect-node-tree-manifest-sidecar-joints-planned-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate planned joint count across static Godot node-tree manifest sidecars equals this value.",
    )
    parser.add_argument(
        "--expect-node-tree-manifest-sidecar-part-path-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate part path count across static Godot node-tree manifest sidecars equals this value.",
    )
    parser.add_argument(
        "--expect-node-tree-manifest-sidecar-joint-path-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate joint path count across static Godot node-tree manifest sidecars equals this value.",
    )
    parser.add_argument(
        "--expect-summary-count",
        action="append",
        default=[],
        type=_summary_count_text,
        metavar="FIELD=COUNT",
        help="Fail unless aggregate gate summary_counts[FIELD] equals COUNT. FIELD must be a known scalar summary count.",
    )
    parser.add_argument(
        "--expect-summary-value",
        action="append",
        default=[],
        type=_summary_value_text,
        metavar="PATH=COUNT",
        help="Fail unless aggregate gate summary_counts PATH equals COUNT. PATH may be a scalar field or a map path such as node_tree_gate_check_counts.fixed_lock_mismatch.",
    )
    parser.add_argument(
        "--expect-summary-value-source",
        action="append",
        default=[],
        type=_nonempty_text,
        metavar="SOURCE",
        help="Validate --expect-summary-value paths against the source-specific schema for SOURCE. May be repeated.",
    )
    parser.add_argument(
        "--expect-summary-value-source-matched-count",
        type=_nonnegative_int,
        help="Fail unless the number of gates included by --expect-summary-value-source filters equals this value.",
    )
    parser.add_argument(
        "--expect-summary-value-source-excluded-count",
        type=_nonnegative_int,
        help="Fail unless the number of gates excluded by --expect-summary-value-source filters equals this value.",
    )
    parser.add_argument(
        "--expect-summary-value-source-excluded-source",
        action="append",
        default=[],
        type=_nonempty_text,
        metavar="SOURCE",
        help="Fail unless SOURCE appears in summary_value_source_excluded_sources. May be repeated.",
    )
    parser.add_argument(
        "--allow-summary-value-source-matched-source",
        action="append",
        default=[],
        type=_nonempty_text,
        metavar="SOURCE",
        help="Fail when summary_value_source_matched_sources contains a source not in this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--forbid-summary-value-source-matched-source",
        action="append",
        default=[],
        type=_nonempty_text,
        metavar="SOURCE",
        help="Fail when SOURCE appears in summary_value_source_matched_sources. May be repeated.",
    )
    parser.add_argument(
        "--allow-summary-value-source-excluded-source",
        action="append",
        default=[],
        type=_nonempty_text,
        metavar="SOURCE",
        help="Fail when summary_value_source_excluded_sources contains a source not in this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--forbid-summary-value-source-excluded-source",
        action="append",
        default=[],
        type=_nonempty_text,
        metavar="SOURCE",
        help="Fail when SOURCE appears in summary_value_source_excluded_sources. May be repeated.",
    )
    parser.add_argument(
        "--expect-node-tree-gate-check-count",
        action="append",
        default=[],
        type=_gate_check_count_text,
        metavar="CHECK=COUNT",
        help="Fail unless aggregate node_tree_gate_check_counts[CHECK] equals COUNT.",
    )
    parser.add_argument(
        "--expect-mechanical-gate-check-count",
        action="append",
        default=[],
        type=_gate_check_count_text,
        metavar="CHECK=COUNT",
        help="Fail unless aggregate mechanical_gate_check_counts[CHECK] equals COUNT.",
    )
    parser.add_argument(
        "--expect-skipped-count",
        type=_nonnegative_int,
        help="Fail unless the skipped input count equals this value.",
    )
    parser.add_argument(
        "--expect-reason-codes-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate unique reason_codes count equals this value.",
    )
    parser.add_argument(
        "--expect-contract-versions-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate unique contract_version count equals this value.",
    )
    parser.add_argument(
        "--expect-summary-versions-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate unique validation summary_version count equals this value.",
    )
    parser.add_argument(
        "--expect-validation-summary-statuses-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate unique validation summary status count equals this value.",
    )
    parser.add_argument(
        "--expect-levels-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate unique delivery acceptance level count equals this value.",
    )
    parser.add_argument(
        "--expect-sources-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate unique source count equals this value.",
    )
    parser.add_argument(
        "--expect-verification-scopes-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate unique verification_scope count equals this value.",
    )
    parser.add_argument(
        "--expect-acceptance-profiles-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate unique acceptance_profile count equals this value.",
    )
    parser.add_argument(
        "--expect-enabled-requirements-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate unique enabled acceptance requirement count equals this value.",
    )
    parser.add_argument(
        "--expect-complete-required-summary-fields-source-scope-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate source/scope entries with complete required summary fields equal this value.",
    )
    parser.add_argument(
        "--expect-affected-inputs-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate unique affected_inputs count equals this value.",
    )
    parser.add_argument(
        "--expect-failed-inputs-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate unique failed input count equals this value.",
    )
    parser.add_argument(
        "--expect-skipped-inputs-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate unique skipped input count equals this value.",
    )
    parser.add_argument(
        "--expect-skipped-reasons-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate unique skipped reason count equals this value.",
    )
    parser.add_argument(
        "--expect-truncated-previews-count",
        type=_nonnegative_int,
        help="Fail unless the aggregate truncated preview field count equals this value.",
    )
    parser.add_argument(
        "--expect-reason-code",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate gate/report reason_codes include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the post-exclusion validation input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-truncated-preview",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate truncated preview field list includes this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-skipped-reason",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate skipped input reasons include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-skipped-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate skipped input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-failed-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate failed input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-affected-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate affected input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-passed-unknown-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate passed=unknown input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-passed-true-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate passed=true input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-passed-false-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate passed=false input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-full-mechanical-gate-true-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate full_mechanical_gate=true input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-full-mechanical-gate-false-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate full_mechanical_gate=false input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-full-mechanical-gate-unknown-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate full_mechanical_gate=unknown input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-source",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate gate/report sources include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-contract-version",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate gate/report contract versions include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-summary-version",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate validation summary_version values include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-validation-summary-status",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate validation summary statuses include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-level",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate gate/report delivery acceptance levels include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-verification-scope",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate gate/report verification scopes include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-acceptance-profile",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate gate/report acceptance profiles include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-enabled-requirement",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail unless the aggregate enabled acceptance requirements include this value. May be repeated.",
    )
    parser.add_argument(
        "--expect-complete-required-summary-fields-source-scope",
        action="append",
        default=[],
        type=_source_scope_text,
        help="Fail unless complete required summary fields include this source/scope entry, formatted as source/scope. May be repeated.",
    )
    parser.add_argument(
        "--allow-complete-required-summary-fields-source-scope",
        action="append",
        default=None,
        type=_source_scope_text,
        help="Fail when complete required summary fields include a source/scope entry outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--forbid-complete-required-summary-fields-source-scope",
        action="append",
        default=[],
        type=_source_scope_text,
        help="Fail when complete required summary fields include this source/scope entry. May be repeated.",
    )
    parser.add_argument(
        "--allow-reason-code",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate gate/report reason_codes include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-input",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when post-exclusion validation input paths include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-truncated-preview",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate truncated preview fields include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-skipped-reason",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate skipped input reasons include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-skipped-input",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate skipped input paths include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-failed-input",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate failed input paths include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-affected-input",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate affected input paths include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-passed-unknown-input",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate passed=unknown input paths include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-passed-true-input",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate passed=true input paths include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-passed-false-input",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate passed=false input paths include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-full-mechanical-gate-true-input",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate full_mechanical_gate=true input paths include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-full-mechanical-gate-false-input",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate full_mechanical_gate=false input paths include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-full-mechanical-gate-unknown-input",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate full_mechanical_gate=unknown input paths include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-source",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate gate/report sources include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-contract-version",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate gate/report contract versions include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-summary-version",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate validation summary_version values include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-validation-summary-status",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate validation summary statuses include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-level",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate gate/report delivery acceptance levels include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-verification-scope",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate gate/report verification scopes include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-acceptance-profile",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate gate/report acceptance profiles include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--allow-enabled-requirement",
        action="append",
        default=None,
        type=_nonempty_text,
        help="Fail when aggregate enabled acceptance requirements include a value outside this allowlist. May be repeated.",
    )
    parser.add_argument(
        "--forbid-reason-code",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate gate/report reason_codes include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when post-exclusion validation input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-truncated-preview",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate truncated preview fields include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-skipped-reason",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate skipped input reasons include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-skipped-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate skipped input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-failed-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate failed input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-affected-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate affected input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-passed-unknown-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate passed=unknown input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-passed-true-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate passed=true input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-passed-false-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate passed=false input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-full-mechanical-gate-true-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate full_mechanical_gate=true input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-full-mechanical-gate-false-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate full_mechanical_gate=false input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-full-mechanical-gate-unknown-input",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate full_mechanical_gate=unknown input paths include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-source",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate gate/report sources include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-contract-version",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate gate/report contract versions include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-summary-version",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate validation summary_version values include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-validation-summary-status",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate validation summary statuses include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-level",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate gate/report delivery acceptance levels include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-verification-scope",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate gate/report verification scopes include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-acceptance-profile",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate gate/report acceptance profiles include this value. May be repeated.",
    )
    parser.add_argument(
        "--forbid-enabled-requirement",
        action="append",
        default=[],
        type=_nonempty_text,
        help="Fail when aggregate enabled acceptance requirements include this value. May be repeated.",
    )
    parser.add_argument(
        "--summary-output",
        type=Path,
        help="Write a compact machine-readable validation summary JSON file.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Write a validation result JSON file in addition to stdout.",
    )
    parser.add_argument(
        "--output-shape",
        choices=["auto", "full", "summary"],
        default="auto",
        help="Control --output payload shape. auto writes summary with --summary-only, otherwise full.",
    )
    args = parser.parse_args()
    workflow_contracts = _load_workflow_contracts_module()
    robot_schema = _load_robot_schema_module()
    if args.print_contract_schema:
        print(
            json.dumps(
                workflow_contracts.DELIVERY_ACCEPTANCE_GATE_SCHEMA,
                indent=2,
                ensure_ascii=False,
            )
        )
        return 0
    if not args.inputs:
        parser.error("the following arguments are required: inputs")
    if args.output is None and _option_supplied(sys.argv[1:], "--output-shape"):
        parser.error("--output-shape requires --output")
    if (
        args.output is not None
        and args.summary_output is not None
        and _same_path(args.output, args.summary_output)
    ):
        parser.error("--output and --summary-output must be different paths")
    output_artifact_paths = [
        output_path
        for output_path in (args.output, args.summary_output)
        if output_path is not None
    ]
    _reject_explicit_output_inputs(parser, args.inputs, output_artifact_paths)
    expected_inputs = _unique_texts(args.expect_input)
    expected_reason_codes = _unique_texts(args.expect_reason_code)
    expected_truncated_previews = _unique_texts(args.expect_truncated_preview)
    expected_skipped_reasons = _unique_texts(args.expect_skipped_reason)
    expected_skipped_inputs = _unique_texts(args.expect_skipped_input)
    expected_failed_inputs = _unique_texts(args.expect_failed_input)
    expected_affected_inputs = _unique_texts(args.expect_affected_input)
    expected_passed_unknown_inputs = _unique_texts(
        args.expect_passed_unknown_input
    )
    expected_passed_true_inputs = _unique_texts(args.expect_passed_true_input)
    expected_passed_false_inputs = _unique_texts(args.expect_passed_false_input)
    expected_full_mechanical_gate_true_inputs = _unique_texts(
        args.expect_full_mechanical_gate_true_input
    )
    expected_full_mechanical_gate_false_inputs = _unique_texts(
        args.expect_full_mechanical_gate_false_input
    )
    expected_full_mechanical_gate_unknown_inputs = _unique_texts(
        args.expect_full_mechanical_gate_unknown_input
    )
    expected_contract_versions = _unique_texts(args.expect_contract_version)
    expected_summary_versions = _unique_texts(args.expect_summary_version)
    expected_validation_summary_statuses = _unique_texts(
        args.expect_validation_summary_status
    )
    expected_levels = _unique_texts(args.expect_level)
    expected_sources = _unique_texts(args.expect_source)
    expected_verification_scopes = _unique_texts(args.expect_verification_scope)
    expected_acceptance_profiles = _unique_texts(args.expect_acceptance_profile)
    expected_enabled_requirements = _unique_texts(args.expect_enabled_requirement)
    expected_summary_counts = _summary_count_expectations(
        args.expect_summary_count
    )
    expected_summary_values = _summary_value_expectations(
        args.expect_summary_value
    )
    expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts = (
        _mismatch_kind_count_expectations(
            args.expect_node_tree_manifest_sidecar_path_map_mismatch_kind
        )
    )
    expected_summary_value_sources = _unique_texts(
        args.expect_summary_value_source
    )
    expected_summary_value_source_excluded_sources = _unique_texts(
        args.expect_summary_value_source_excluded_source
    )
    allowed_summary_value_source_matched_sources = (
        _unique_texts(args.allow_summary_value_source_matched_source)
        if args.allow_summary_value_source_matched_source
        else None
    )
    forbidden_summary_value_source_matched_sources = _unique_texts(
        args.forbid_summary_value_source_matched_source
    )
    allowed_summary_value_source_excluded_sources = (
        _unique_texts(args.allow_summary_value_source_excluded_source)
        if args.allow_summary_value_source_excluded_source
        else None
    )
    forbidden_summary_value_source_excluded_sources = _unique_texts(
        args.forbid_summary_value_source_excluded_source
    )
    known_summary_count_fields = sorted(
        workflow_contracts.DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_FIELDS
    )
    unknown_summary_count_fields = sorted(
        set(expected_summary_counts) - set(known_summary_count_fields)
    )
    if unknown_summary_count_fields:
        parser.error(
            "unsupported --expect-summary-count field(s): "
            f"{', '.join(unknown_summary_count_fields)}; supported values: "
            f"{', '.join(known_summary_count_fields) or 'none'}"
        )
    summary_count_map_key_values = (
        workflow_contracts.DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_MAP_KEY_VALUES
    )
    known_summary_value_paths = set(
        workflow_contracts.DELIVERY_ACCEPTANCE_GATE_SCHEMA.get(
            "summary_value_paths",
            [],
        )
    )
    if not known_summary_value_paths:
        known_summary_value_paths = set(known_summary_count_fields)
        for map_key, count_keys in summary_count_map_key_values.items():
            known_summary_value_paths.update(
                f"{map_key}.{count_key}" for count_key in count_keys
            )
    unknown_summary_value_paths = sorted(
        set(expected_summary_values) - known_summary_value_paths
    )
    if unknown_summary_value_paths:
        parser.error(
            "unsupported --expect-summary-value path(s): "
            f"{', '.join(unknown_summary_value_paths)}; supported values: "
            f"{', '.join(sorted(known_summary_value_paths)) or 'none'}"
        )
    known_sources = set(workflow_contracts.DELIVERY_ACCEPTANCE_GATE_SOURCES)
    unknown_summary_value_sources = sorted(
        set(expected_summary_value_sources) - known_sources
    )
    if unknown_summary_value_sources:
        parser.error(
            "unsupported --expect-summary-value-source value(s): "
            f"{', '.join(unknown_summary_value_sources)}; supported values: "
            f"{', '.join(sorted(known_sources)) or 'none'}"
        )
    unknown_summary_value_excluded_sources = sorted(
        set(expected_summary_value_source_excluded_sources) - known_sources
    )
    if unknown_summary_value_excluded_sources:
        parser.error(
            "unsupported --expect-summary-value-source-excluded-source "
            "value(s): "
            f"{', '.join(unknown_summary_value_excluded_sources)}; "
            "supported values: "
            f"{', '.join(sorted(known_sources)) or 'none'}"
        )
    unknown_allowed_summary_value_matched_sources = sorted(
        set(allowed_summary_value_source_matched_sources or []) - known_sources
    )
    if unknown_allowed_summary_value_matched_sources:
        parser.error(
            "unsupported --allow-summary-value-source-matched-source value(s): "
            f"{', '.join(unknown_allowed_summary_value_matched_sources)}; "
            "supported values: "
            f"{', '.join(sorted(known_sources)) or 'none'}"
        )
    unknown_forbidden_summary_value_matched_sources = sorted(
        set(forbidden_summary_value_source_matched_sources) - known_sources
    )
    if unknown_forbidden_summary_value_matched_sources:
        parser.error(
            "unsupported --forbid-summary-value-source-matched-source value(s): "
            f"{', '.join(unknown_forbidden_summary_value_matched_sources)}; "
            "supported values: "
            f"{', '.join(sorted(known_sources)) or 'none'}"
        )
    unknown_allowed_summary_value_excluded_sources = sorted(
        set(allowed_summary_value_source_excluded_sources or []) - known_sources
    )
    if unknown_allowed_summary_value_excluded_sources:
        parser.error(
            "unsupported --allow-summary-value-source-excluded-source "
            "value(s): "
            f"{', '.join(unknown_allowed_summary_value_excluded_sources)}; "
            "supported values: "
            f"{', '.join(sorted(known_sources)) or 'none'}"
        )
    unknown_forbidden_summary_value_excluded_sources = sorted(
        set(forbidden_summary_value_source_excluded_sources) - known_sources
    )
    if unknown_forbidden_summary_value_excluded_sources:
        parser.error(
            "unsupported --forbid-summary-value-source-excluded-source "
            "value(s): "
            f"{', '.join(unknown_forbidden_summary_value_excluded_sources)}; "
            "supported values: "
            f"{', '.join(sorted(known_sources)) or 'none'}"
        )
    if expected_summary_value_sources and not expected_summary_values:
        parser.error(
            "--expect-summary-value-source requires at least one "
            "--expect-summary-value"
        )
    if (
        args.expect_summary_value_source_matched_count is not None
        or args.expect_summary_value_source_excluded_count is not None
        or expected_summary_value_source_excluded_sources
        or allowed_summary_value_source_matched_sources is not None
        or forbidden_summary_value_source_matched_sources
        or allowed_summary_value_source_excluded_sources is not None
        or forbidden_summary_value_source_excluded_sources
    ) and not expected_summary_value_sources:
        parser.error(
            "--expect-summary-value-source-matched-count and "
            "--expect-summary-value-source-excluded-count and "
            "--expect-summary-value-source-excluded-source and "
            "--allow-summary-value-source-matched-source and "
            "--forbid-summary-value-source-matched-source and "
            "--allow-summary-value-source-excluded-source and "
            "--forbid-summary-value-source-excluded-source require at least "
            "one --expect-summary-value-source"
        )
    summary_value_paths_by_source = (
        workflow_contracts.DELIVERY_ACCEPTANCE_GATE_SCHEMA.get(
            "summary_value_paths_by_source",
            {},
        )
    )
    for source in expected_summary_value_sources:
        source_paths = set(summary_value_paths_by_source.get(source, []))
        unsupported_source_paths = sorted(
            set(expected_summary_values) - source_paths
        )
        if unsupported_source_paths:
            parser.error(
                "unsupported --expect-summary-value path(s) for source "
                f"{source}: {', '.join(unsupported_source_paths)}; "
                "supported source values: "
                f"{', '.join(sorted(source_paths)) or 'none'}"
            )
    expected_node_tree_gate_check_counts = _gate_check_expectations(
        args.expect_node_tree_gate_check_count
    )
    expected_mechanical_gate_check_counts = _gate_check_expectations(
        args.expect_mechanical_gate_check_count
    )
    known_node_tree_gate_check_keys = sorted(
        summary_count_map_key_values["node_tree_gate_check_counts"]
    )
    unknown_node_tree_gate_check_keys = sorted(
        set(expected_node_tree_gate_check_counts) - set(known_node_tree_gate_check_keys)
    )
    if unknown_node_tree_gate_check_keys:
        parser.error(
            "unsupported --expect-node-tree-gate-check-count check(s): "
            f"{', '.join(unknown_node_tree_gate_check_keys)}; supported values: "
            f"{', '.join(known_node_tree_gate_check_keys) or 'none'}"
        )
    known_mechanical_gate_check_keys = sorted(
        summary_count_map_key_values["mechanical_gate_check_counts"]
    )
    unknown_mechanical_gate_check_keys = sorted(
        set(expected_mechanical_gate_check_counts)
        - set(known_mechanical_gate_check_keys)
    )
    if unknown_mechanical_gate_check_keys:
        parser.error(
            "unsupported --expect-mechanical-gate-check-count check(s): "
            f"{', '.join(unknown_mechanical_gate_check_keys)}; supported values: "
            f"{', '.join(known_mechanical_gate_check_keys) or 'none'}"
        )
    expected_complete_required_summary_fields_source_scopes = _unique_texts(
        args.expect_complete_required_summary_fields_source_scope
    )
    known_complete_required_summary_fields_source_scopes = (
        _schema_complete_required_summary_fields_source_scopes(
            workflow_contracts=workflow_contracts
        )
    )
    unknown_complete_required_summary_fields_source_scopes = [
        value
        for value in expected_complete_required_summary_fields_source_scopes
        if value not in known_complete_required_summary_fields_source_scopes
    ]
    if unknown_complete_required_summary_fields_source_scopes:
        parser.error(
            "unsupported --expect-complete-required-summary-fields-source-scope "
            f"value(s): {', '.join(unknown_complete_required_summary_fields_source_scopes)}; "
            "supported values: "
            f"{', '.join(known_complete_required_summary_fields_source_scopes) or 'none'}"
        )
    allowed_complete_required_summary_fields_source_scopes = (
        None
        if args.allow_complete_required_summary_fields_source_scope is None
        else _unique_texts(args.allow_complete_required_summary_fields_source_scope)
    )
    unknown_allowed_complete_required_summary_fields_source_scopes = [
        value
        for value in (allowed_complete_required_summary_fields_source_scopes or [])
        if value not in known_complete_required_summary_fields_source_scopes
    ]
    if unknown_allowed_complete_required_summary_fields_source_scopes:
        parser.error(
            "unsupported --allow-complete-required-summary-fields-source-scope "
            f"value(s): {', '.join(unknown_allowed_complete_required_summary_fields_source_scopes)}; "
            "supported values: "
            f"{', '.join(known_complete_required_summary_fields_source_scopes) or 'none'}"
        )
    forbidden_complete_required_summary_fields_source_scopes = _unique_texts(
        args.forbid_complete_required_summary_fields_source_scope
    )
    unknown_forbidden_complete_required_summary_fields_source_scopes = [
        value
        for value in forbidden_complete_required_summary_fields_source_scopes
        if value not in known_complete_required_summary_fields_source_scopes
    ]
    if unknown_forbidden_complete_required_summary_fields_source_scopes:
        parser.error(
            "unsupported --forbid-complete-required-summary-fields-source-scope "
            f"value(s): {', '.join(unknown_forbidden_complete_required_summary_fields_source_scopes)}; "
            "supported values: "
            f"{', '.join(known_complete_required_summary_fields_source_scopes) or 'none'}"
        )
    allowed_reason_codes = (
        None
        if args.allow_reason_code is None
        else _unique_texts(args.allow_reason_code)
    )
    allowed_inputs = (
        None if args.allow_input is None else _unique_texts(args.allow_input)
    )
    allowed_truncated_previews = (
        None
        if args.allow_truncated_preview is None
        else _unique_texts(args.allow_truncated_preview)
    )
    allowed_skipped_reasons = (
        None
        if args.allow_skipped_reason is None
        else _unique_texts(args.allow_skipped_reason)
    )
    allowed_skipped_inputs = (
        None
        if args.allow_skipped_input is None
        else _unique_texts(args.allow_skipped_input)
    )
    allowed_failed_inputs = (
        None
        if args.allow_failed_input is None
        else _unique_texts(args.allow_failed_input)
    )
    allowed_affected_inputs = (
        None
        if args.allow_affected_input is None
        else _unique_texts(args.allow_affected_input)
    )
    allowed_passed_unknown_inputs = (
        None
        if args.allow_passed_unknown_input is None
        else _unique_texts(args.allow_passed_unknown_input)
    )
    allowed_passed_true_inputs = (
        None
        if args.allow_passed_true_input is None
        else _unique_texts(args.allow_passed_true_input)
    )
    allowed_passed_false_inputs = (
        None
        if args.allow_passed_false_input is None
        else _unique_texts(args.allow_passed_false_input)
    )
    allowed_full_mechanical_gate_true_inputs = (
        None
        if args.allow_full_mechanical_gate_true_input is None
        else _unique_texts(args.allow_full_mechanical_gate_true_input)
    )
    allowed_full_mechanical_gate_false_inputs = (
        None
        if args.allow_full_mechanical_gate_false_input is None
        else _unique_texts(args.allow_full_mechanical_gate_false_input)
    )
    allowed_full_mechanical_gate_unknown_inputs = (
        None
        if args.allow_full_mechanical_gate_unknown_input is None
        else _unique_texts(args.allow_full_mechanical_gate_unknown_input)
    )
    allowed_contract_versions = (
        None
        if args.allow_contract_version is None
        else _unique_texts(args.allow_contract_version)
    )
    allowed_summary_versions = (
        None
        if args.allow_summary_version is None
        else _unique_texts(args.allow_summary_version)
    )
    allowed_validation_summary_statuses = (
        None
        if args.allow_validation_summary_status is None
        else _unique_texts(args.allow_validation_summary_status)
    )
    allowed_levels = (
        None if args.allow_level is None else _unique_texts(args.allow_level)
    )
    allowed_sources = (
        None if args.allow_source is None else _unique_texts(args.allow_source)
    )
    allowed_verification_scopes = (
        None
        if args.allow_verification_scope is None
        else _unique_texts(args.allow_verification_scope)
    )
    allowed_acceptance_profiles = (
        None
        if args.allow_acceptance_profile is None
        else _unique_texts(args.allow_acceptance_profile)
    )
    allowed_enabled_requirements = (
        None
        if args.allow_enabled_requirement is None
        else _unique_texts(args.allow_enabled_requirement)
    )
    forbidden_reason_codes = _unique_texts(args.forbid_reason_code)
    forbidden_inputs = _unique_texts(args.forbid_input)
    forbidden_truncated_previews = _unique_texts(args.forbid_truncated_preview)
    forbidden_skipped_reasons = _unique_texts(args.forbid_skipped_reason)
    forbidden_skipped_inputs = _unique_texts(args.forbid_skipped_input)
    forbidden_failed_inputs = _unique_texts(args.forbid_failed_input)
    forbidden_affected_inputs = _unique_texts(args.forbid_affected_input)
    forbidden_passed_unknown_inputs = _unique_texts(
        args.forbid_passed_unknown_input
    )
    forbidden_passed_true_inputs = _unique_texts(args.forbid_passed_true_input)
    forbidden_passed_false_inputs = _unique_texts(args.forbid_passed_false_input)
    forbidden_full_mechanical_gate_true_inputs = _unique_texts(
        args.forbid_full_mechanical_gate_true_input
    )
    forbidden_full_mechanical_gate_false_inputs = _unique_texts(
        args.forbid_full_mechanical_gate_false_input
    )
    forbidden_full_mechanical_gate_unknown_inputs = _unique_texts(
        args.forbid_full_mechanical_gate_unknown_input
    )
    forbidden_contract_versions = _unique_texts(args.forbid_contract_version)
    forbidden_summary_versions = _unique_texts(args.forbid_summary_version)
    forbidden_validation_summary_statuses = _unique_texts(
        args.forbid_validation_summary_status
    )
    forbidden_levels = _unique_texts(args.forbid_level)
    forbidden_sources = _unique_texts(args.forbid_source)
    forbidden_verification_scopes = _unique_texts(args.forbid_verification_scope)
    forbidden_acceptance_profiles = _unique_texts(args.forbid_acceptance_profile)
    forbidden_enabled_requirements = _unique_texts(
        args.forbid_enabled_requirement
    )
    expanded_input_paths = _expand_input_paths(args.inputs, recursive=args.recursive)
    excluded_output_artifact_inputs = _output_artifact_inputs(
        expanded_input_paths,
        output_artifact_paths,
    )
    input_paths = _exclude_output_artifact_inputs(
        expanded_input_paths,
        output_artifact_paths,
    )
    validation_inputs = _unique_texts([str(path) for path in input_paths])
    results = [
        _validate_input(
            input_path=input_path,
            workflow_contracts=workflow_contracts,
            robot_schema=robot_schema,
            require_passed=args.require_passed,
            require_required=args.require_required,
            require_complete=args.require_complete,
            require_full_mechanical_restoration_gate=(
                args.require_full_mechanical_restoration_gate
            ),
            ignore_non_gate=args.ignore_non_gate,
            validate_validation_summary=args.validate_validation_summary,
        )
        for input_path in input_paths
    ]
    failed = [result for result in results if result["status"] == "error"]
    skipped = [result for result in results if result["status"] == "skipped"]
    successes = [result for result in results if result["status"] == "success"]
    passed_true, passed_false, passed_unknown = _partition_passed_results(results)
    contract_versions = _unique_texts(
        [
            result.get("contract_version")
            for result in results
            if result.get("contract_version")
        ]
    )
    summary_versions = _unique_texts(
        [
            result.get("summary_version")
            for result in results
            if result.get("summary_version")
        ]
    )
    validation_summary_statuses = _unique_texts(
        [
            result.get("validation_summary_status")
            for result in results
            if result.get("validation_summary_status")
        ]
    )
    levels = _unique_texts(
        [result.get("level") for result in results if result.get("level")]
    )
    sources = _unique_texts(
        [result.get("source") for result in results if result.get("source")]
    )
    verification_scopes = _unique_texts(
        [
            result.get("verification_scope")
            for result in results
            if result.get("verification_scope")
        ]
    )
    acceptance_profiles = _unique_texts(
        [
            result.get("acceptance_profile")
            for result in results
            if result.get("acceptance_profile")
        ]
    )
    enabled_requirements = _unique_texts(
        [
            requirement
            for result in results
            for requirement in result.get("enabled_requirements", [])
        ]
    )
    affected_inputs = _unique_texts(
        [
            input_path
            for result in results
            for input_path in result.get("affected_inputs", [])
        ]
    )
    failed_inputs = _unique_texts([result.get("input") for result in failed])
    passed_true_inputs = _unique_texts([result.get("input") for result in passed_true])
    passed_false_inputs = _unique_texts(
        [result.get("input") for result in passed_false]
    )
    passed_unknown_inputs = _unique_texts(
        [result.get("input") for result in passed_unknown]
    )
    full_mechanical_gate_true = [
        result
        for result in results
        if result.get("requires_full_mechanical_restoration_gate") is True
    ]
    full_mechanical_gate_false = [
        result
        for result in results
        if result.get("requires_full_mechanical_restoration_gate") is False
    ]
    full_mechanical_gate_unknown = [
        result
        for result in results
        if result.get("requires_full_mechanical_restoration_gate") is not True
        and result.get("requires_full_mechanical_restoration_gate") is not False
    ]
    full_mechanical_gate_true_inputs = _unique_texts(
        [result.get("input") for result in full_mechanical_gate_true]
    )
    full_mechanical_gate_false_inputs = _unique_texts(
        [result.get("input") for result in full_mechanical_gate_false]
    )
    full_mechanical_gate_unknown_inputs = _unique_texts(
        [result.get("input") for result in full_mechanical_gate_unknown]
    )
    smoke_report_written_count = _summary_counts_total(
        results, "smoke_report_written_count"
    )
    smoke_report_missing_count = _summary_counts_total(
        results, "smoke_report_missing_count"
    )
    smoke_report_read_error_count = _summary_counts_total(
        results, "smoke_report_read_error_count"
    )
    control_configured_count = _summary_counts_total(
        results, "control_configured_count"
    )
    control_readback_checked_count = _summary_counts_total(
        results, "control_readback_checked_count"
    )
    control_readback_missing_count = _summary_counts_total(
        results, "control_readback_missing_count"
    )
    node_tree_gate_check_counts = _summary_counts_map_totals(
        results,
        "node_tree_gate_check_counts",
    )
    mechanical_gate_check_counts = _summary_counts_map_totals(
        results,
        "mechanical_gate_check_counts",
    )
    actual_expected_summary_counts = _selected_summary_count_totals(
        results,
        expected_summary_counts,
    )
    actual_expected_summary_values = _selected_summary_value_totals(
        results,
        expected_summary_values,
        expected_summary_value_sources,
    )
    complete_required_summary_fields_by_source_scope = (
        _complete_required_summary_fields_by_source_scope(results)
    )
    complete_required_summary_fields_source_scopes = _source_scope_values(
        complete_required_summary_fields_by_source_scope
    )
    complete_required_summary_fields_source_scope_count = sum(
        len(scopes)
        for scopes in complete_required_summary_fields_by_source_scope.values()
    )
    skipped_inputs = _unique_texts([result.get("input") for result in skipped])
    skipped_reasons = _unique_texts(
        [result.get("skip_reason") for result in skipped if result.get("skip_reason")]
    )
    expected_summary_value_source_set = set(expected_summary_value_sources)
    summary_value_source_matched_count = (
        len(
            [
                result
                for result in results
                if result.get("source") in expected_summary_value_source_set
            ]
        )
        if expected_summary_value_sources
        else None
    )
    summary_value_source_excluded_count = (
        len(
            [
                result
                for result in results
                if result.get("source") not in expected_summary_value_source_set
            ]
        )
        if expected_summary_value_sources
        else None
    )
    summary_value_source_excluded_sources = _unique_texts(
        [
            result.get("source")
            for result in results
            if expected_summary_value_sources
            and result.get("source") not in expected_summary_value_source_set
            and result.get("source")
        ]
    )
    summary_value_source_matched_sources = _unique_texts(
        [
            result.get("source")
            for result in results
            if expected_summary_value_sources
            and result.get("source") in expected_summary_value_source_set
            and result.get("source")
        ]
    )
    unexpected_summary_value_source_matched_sources = _unexpected_values(
        summary_value_source_matched_sources,
        allowed_summary_value_source_matched_sources,
    )
    present_forbidden_summary_value_source_matched_sources = (
        _present_forbidden_values(
            summary_value_source_matched_sources,
            forbidden_summary_value_source_matched_sources,
        )
    )
    unexpected_summary_value_source_excluded_sources = _unexpected_values(
        summary_value_source_excluded_sources,
        allowed_summary_value_source_excluded_sources,
    )
    present_forbidden_summary_value_source_excluded_sources = (
        _present_forbidden_values(
            summary_value_source_excluded_sources,
            forbidden_summary_value_source_excluded_sources,
        )
    )
    node_tree_manifest_sidecar_count = sum(
        1
        for result in skipped
        if isinstance(result.get("node_tree_manifest_summary"), dict)
    )
    node_tree_manifest_sidecar_complete_count = sum(
        1
        for result in skipped
        if isinstance(result.get("node_tree_manifest_summary"), dict)
        and result["node_tree_manifest_summary"].get("complete") is True
    )
    node_tree_manifest_sidecar_incomplete_count = sum(
        1
        for result in skipped
        if isinstance(result.get("node_tree_manifest_summary"), dict)
        and result["node_tree_manifest_summary"].get("complete") is False
    )
    node_tree_manifest_sidecar_valid_count = sum(
        1
        for result in skipped
        if isinstance(result.get("node_tree_manifest_summary"), dict)
        and result.get("node_tree_manifest_valid") is True
    )
    node_tree_manifest_sidecar_invalid_count = sum(
        1
        for result in skipped
        if isinstance(result.get("node_tree_manifest_summary"), dict)
        and result.get("node_tree_manifest_valid") is False
    )
    node_tree_manifest_sidecar_validation_error_count = sum(
        len(result.get("node_tree_manifest_validation_errors", []))
        for result in skipped
        if isinstance(result.get("node_tree_manifest_summary"), dict)
        and isinstance(result.get("node_tree_manifest_validation_errors"), list)
    )
    node_tree_manifest_sidecar_path_incomplete_count = sum(
        1
        for result in skipped
        if isinstance(result.get("node_tree_manifest_summary"), dict)
        and _node_tree_manifest_summary_has_incomplete_paths(
            result["node_tree_manifest_summary"]
        )
    )
    node_tree_manifest_sidecar_path_map_mismatch_count = sum(
        int(result.get("node_tree_manifest_path_map_mismatch_count") or 0)
        for result in skipped
        if isinstance(result.get("node_tree_manifest_summary"), dict)
    )
    node_tree_manifest_sidecar_path_map_mismatch_kind_counts = (
        _node_tree_manifest_sidecar_path_map_mismatch_kind_counts(
            [
                {
                    "node_tree_manifest_path_map_mismatch_kind_counts": result.get(
                        "node_tree_manifest_path_map_mismatch_kind_counts", {}
                    )
                }
                for result in skipped
                if isinstance(result.get("node_tree_manifest_summary"), dict)
            ]
        )
    )
    node_tree_manifest_sidecar_parts_planned_count = sum(
        int(result["node_tree_manifest_summary"].get("parts_count") or 0)
        for result in skipped
        if isinstance(result.get("node_tree_manifest_summary"), dict)
    )
    node_tree_manifest_sidecar_joints_planned_count = sum(
        int(result["node_tree_manifest_summary"].get("joints_count") or 0)
        for result in skipped
        if isinstance(result.get("node_tree_manifest_summary"), dict)
    )
    node_tree_manifest_sidecar_part_path_count = sum(
        int(result["node_tree_manifest_summary"].get("part_node_path_count") or 0)
        for result in skipped
        if isinstance(result.get("node_tree_manifest_summary"), dict)
    )
    node_tree_manifest_sidecar_joint_path_count = sum(
        int(result["node_tree_manifest_summary"].get("joint_node_path_count") or 0)
        for result in skipped
        if isinstance(result.get("node_tree_manifest_summary"), dict)
    )
    aggregate_errors: list[str] = []
    if not input_paths:
        aggregate_errors.append("no JSON files found in input paths")
    elif args.ignore_non_gate and not args.allow_empty and not successes:
        if args.validate_validation_summary:
            aggregate_errors.append(
                "no delivery_acceptance_gate validation summary artifacts found"
            )
        else:
            aggregate_errors.append(
                "no delivery_acceptance_gate artifacts or reports found"
            )
    if args.fail_on_skipped and skipped:
        aggregate_errors.append(
            f"skipped inputs present: {len(skipped)}"
        )
    if args.fail_on_passed_false and passed_false:
        aggregate_errors.append(f"passed=false gates present: {len(passed_false)}")
    if args.fail_on_passed_unknown and passed_unknown:
        aggregate_errors.append(
            f"passed unknown gates present: {len(passed_unknown)}"
        )
    if args.fail_on_full_mechanical_gate_false and full_mechanical_gate_false:
        aggregate_errors.append(
            "full_mechanical_gate=false inputs present: "
            f"{len(full_mechanical_gate_false)}"
        )
    if args.fail_on_full_mechanical_gate_unknown and full_mechanical_gate_unknown:
        aggregate_errors.append(
            "full_mechanical_gate unknown inputs present: "
            f"{len(full_mechanical_gate_unknown)}"
        )
    if args.fail_on_smoke_report_missing and smoke_report_missing_count:
        aggregate_errors.append(
            f"smoke_report_missing_count present: {smoke_report_missing_count}"
        )
    if args.fail_on_smoke_report_read_error and smoke_report_read_error_count:
        aggregate_errors.append(
            "smoke_report_read_error_count present: "
            f"{smoke_report_read_error_count}"
        )
    if args.fail_on_control_readback_missing and control_readback_missing_count:
        aggregate_errors.append(
            "control_readback_missing_count present: "
            f"{control_readback_missing_count}"
        )
    if (
        args.fail_on_node_tree_manifest_sidecar_incomplete
        and node_tree_manifest_sidecar_incomplete_count
    ):
        aggregate_errors.append(
            "node_tree_manifest_sidecar_incomplete_count present: "
            f"{node_tree_manifest_sidecar_incomplete_count}"
        )
    if (
        args.fail_on_invalid_node_tree_manifest_sidecar
        and node_tree_manifest_sidecar_invalid_count
    ):
        aggregate_errors.append(
            "node_tree_manifest_sidecar_invalid_count present: "
            f"{node_tree_manifest_sidecar_invalid_count}"
        )
    if (
        args.fail_on_node_tree_manifest_sidecar_validation_error
        and node_tree_manifest_sidecar_validation_error_count
    ):
        aggregate_errors.append(
            "node_tree_manifest_sidecar_validation_error_count present: "
            f"{node_tree_manifest_sidecar_validation_error_count}"
        )
    if args.fail_on_node_tree_manifest_sidecar_path_incomplete:
        if node_tree_manifest_sidecar_path_incomplete_count:
            aggregate_errors.append(
                "node_tree_manifest_sidecar_path_incomplete_count present: "
                f"{node_tree_manifest_sidecar_path_incomplete_count}"
            )
    if args.fail_on_node_tree_manifest_sidecar_path_map_mismatch:
        if node_tree_manifest_sidecar_path_map_mismatch_count:
            aggregate_errors.append(
                "node_tree_manifest_sidecar_path_map_mismatch_count present: "
                f"{node_tree_manifest_sidecar_path_map_mismatch_count}"
            )
    aggregate_errors.extend(
        _expected_map_count_errors(
            "node_tree_manifest_sidecar_path_map_mismatch_kind_counts",
            expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts,
            node_tree_manifest_sidecar_path_map_mismatch_kind_counts,
        )
    )
    aggregate_errors.extend(
        _expected_count_errors(
            [
                (
                    "expanded_inputs_count",
                    args.expect_expanded_inputs_count,
                    len(expanded_input_paths),
                ),
                (
                    "excluded_output_artifacts_count",
                    args.expect_excluded_output_artifacts_count,
                    len(excluded_output_artifact_inputs),
                ),
                ("inputs_count", args.expect_inputs_count, len(results)),
                ("success_count", args.expect_success_count, len(successes)),
                ("error_count", args.expect_error_count, len(failed)),
                (
                    "passed_true_count",
                    args.expect_passed_true_count,
                    len(passed_true),
                ),
                (
                    "passed_false_count",
                    args.expect_passed_false_count,
                    len(passed_false),
                ),
                (
                    "passed_unknown_count",
                    args.expect_passed_unknown_count,
                    len(passed_unknown),
                ),
                (
                    "full_mechanical_gate_true_count",
                    args.expect_full_mechanical_gate_true_count,
                    len(full_mechanical_gate_true),
                ),
                (
                    "full_mechanical_gate_false_count",
                    args.expect_full_mechanical_gate_false_count,
                    len(full_mechanical_gate_false),
                ),
                (
                    "full_mechanical_gate_unknown_count",
                    args.expect_full_mechanical_gate_unknown_count,
                    len(full_mechanical_gate_unknown),
                ),
                (
                    "smoke_report_written_count",
                    args.expect_smoke_report_written_count,
                    smoke_report_written_count,
                ),
                (
                    "smoke_report_missing_count",
                    args.expect_smoke_report_missing_count,
                    smoke_report_missing_count,
                ),
                (
                    "smoke_report_read_error_count",
                    args.expect_smoke_report_read_error_count,
                    smoke_report_read_error_count,
                ),
                (
                    "control_configured_count",
                    args.expect_control_configured_count,
                    control_configured_count,
                ),
                (
                    "control_readback_checked_count",
                    args.expect_control_readback_checked_count,
                    control_readback_checked_count,
                ),
                (
                    "control_readback_missing_count",
                    args.expect_control_readback_missing_count,
                    control_readback_missing_count,
                ),
                (
                    "node_tree_manifest_sidecar_count",
                    args.expect_node_tree_manifest_sidecar_count,
                    node_tree_manifest_sidecar_count,
                ),
                (
                    "node_tree_manifest_sidecar_complete_count",
                    args.expect_node_tree_manifest_sidecar_complete_count,
                    node_tree_manifest_sidecar_complete_count,
                ),
                (
                    "node_tree_manifest_sidecar_incomplete_count",
                    args.expect_node_tree_manifest_sidecar_incomplete_count,
                    node_tree_manifest_sidecar_incomplete_count,
                ),
                (
                    "node_tree_manifest_sidecar_valid_count",
                    args.expect_node_tree_manifest_sidecar_valid_count,
                    node_tree_manifest_sidecar_valid_count,
                ),
                (
                    "node_tree_manifest_sidecar_invalid_count",
                    args.expect_node_tree_manifest_sidecar_invalid_count,
                    node_tree_manifest_sidecar_invalid_count,
                ),
                (
                    "node_tree_manifest_sidecar_validation_error_count",
                    args.expect_node_tree_manifest_sidecar_validation_error_count,
                    node_tree_manifest_sidecar_validation_error_count,
                ),
                (
                    "node_tree_manifest_sidecar_path_incomplete_count",
                    args.expect_node_tree_manifest_sidecar_path_incomplete_count,
                    node_tree_manifest_sidecar_path_incomplete_count,
                ),
                (
                    "node_tree_manifest_sidecar_path_map_mismatch_count",
                    args.expect_node_tree_manifest_sidecar_path_map_mismatch_count,
                    node_tree_manifest_sidecar_path_map_mismatch_count,
                ),
                (
                    "node_tree_manifest_sidecar_parts_planned_count",
                    args.expect_node_tree_manifest_sidecar_parts_planned_count,
                    node_tree_manifest_sidecar_parts_planned_count,
                ),
                (
                    "node_tree_manifest_sidecar_joints_planned_count",
                    args.expect_node_tree_manifest_sidecar_joints_planned_count,
                    node_tree_manifest_sidecar_joints_planned_count,
                ),
                (
                    "node_tree_manifest_sidecar_part_path_count",
                    args.expect_node_tree_manifest_sidecar_part_path_count,
                    node_tree_manifest_sidecar_part_path_count,
                ),
                (
                    "node_tree_manifest_sidecar_joint_path_count",
                    args.expect_node_tree_manifest_sidecar_joint_path_count,
                    node_tree_manifest_sidecar_joint_path_count,
                ),
                (
                    "summary_value_source_matched_count",
                    args.expect_summary_value_source_matched_count,
                    summary_value_source_matched_count,
                ),
                (
                    "summary_value_source_excluded_count",
                    args.expect_summary_value_source_excluded_count,
                    summary_value_source_excluded_count,
                ),
                ("skipped_count", args.expect_skipped_count, len(skipped)),
                (
                    "reason_codes_count",
                    args.expect_reason_codes_count,
                    len(
                        {
                            str(reason_code)
                            for result in results
                            for reason_code in result.get("reason_codes", [])
                        }
                    ),
                ),
                (
                    "contract_versions_count",
                    args.expect_contract_versions_count,
                    len(contract_versions),
                ),
                (
                    "summary_versions_count",
                    args.expect_summary_versions_count,
                    len(summary_versions),
                ),
                (
                    "validation_summary_statuses_count",
                    args.expect_validation_summary_statuses_count,
                    len(validation_summary_statuses),
                ),
                ("levels_count", args.expect_levels_count, len(levels)),
                ("sources_count", args.expect_sources_count, len(sources)),
                (
                    "verification_scopes_count",
                    args.expect_verification_scopes_count,
                    len(verification_scopes),
                ),
                (
                    "acceptance_profiles_count",
                    args.expect_acceptance_profiles_count,
                    len(acceptance_profiles),
                ),
                (
                    "enabled_requirements_count",
                    args.expect_enabled_requirements_count,
                    len(enabled_requirements),
                ),
                (
                    "complete_required_summary_fields_source_scope_count",
                    args.expect_complete_required_summary_fields_source_scope_count,
                    complete_required_summary_fields_source_scope_count,
                ),
                (
                    "affected_inputs_count",
                    args.expect_affected_inputs_count,
                    len(affected_inputs),
                ),
                (
                    "failed_inputs_count",
                    args.expect_failed_inputs_count,
                    len(failed_inputs),
                ),
                (
                    "skipped_inputs_count",
                    args.expect_skipped_inputs_count,
                    len(skipped_inputs),
                ),
                (
                    "skipped_reasons_count",
                    args.expect_skipped_reasons_count,
                    len(skipped_reasons),
                ),
            ]
        )
    )
    aggregate_errors.extend(
        _expected_map_count_errors(
            "node_tree_gate_check_counts",
            expected_node_tree_gate_check_counts,
            node_tree_gate_check_counts,
        )
    )
    aggregate_errors.extend(
        _expected_map_count_errors(
            "summary_counts",
            expected_summary_counts,
            actual_expected_summary_counts,
        )
    )
    aggregate_errors.extend(
        _expected_map_count_errors(
            "summary_counts",
            expected_summary_values,
            actual_expected_summary_values,
        )
    )
    aggregate_errors.extend(
        _expected_map_count_errors(
            "mechanical_gate_check_counts",
            expected_mechanical_gate_check_counts,
            mechanical_gate_check_counts,
        )
    )
    reason_codes = {
        str(reason_code)
        for result in results
        for reason_code in result.get("reason_codes", [])
    }
    aggregate_errors.extend(
        _expected_value_errors("reason_code", expected_reason_codes, list(reason_codes))
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "contract_version",
            expected_contract_versions,
            contract_versions,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "summary_version",
            expected_summary_versions,
            summary_versions,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "validation_summary_status",
            expected_validation_summary_statuses,
            validation_summary_statuses,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors("level", expected_levels, levels)
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "skipped_reason",
            expected_skipped_reasons,
            skipped_reasons,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors("source", expected_sources, sources)
    )
    if results and expected_summary_value_sources:
        aggregate_errors.extend(
            _expected_value_errors(
                "summary_value_source",
                expected_summary_value_sources,
                sources,
            )
        )
    aggregate_errors.extend(
        _expected_value_errors(
            "verification_scope",
            expected_verification_scopes,
            verification_scopes,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "acceptance_profile",
            expected_acceptance_profiles,
            acceptance_profiles,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "complete_required_summary_fields_source_scope",
            expected_complete_required_summary_fields_source_scopes,
            complete_required_summary_fields_source_scopes,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "summary_value_source_excluded_source",
            expected_summary_value_source_excluded_sources,
            summary_value_source_excluded_sources,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "summary_value_source_matched_source",
            summary_value_source_matched_sources,
            allowed_summary_value_source_matched_sources,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "summary_value_source_matched_source",
            summary_value_source_matched_sources,
            forbidden_summary_value_source_matched_sources,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "summary_value_source_excluded_source",
            summary_value_source_excluded_sources,
            allowed_summary_value_source_excluded_sources,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "summary_value_source_excluded_source",
            summary_value_source_excluded_sources,
            forbidden_summary_value_source_excluded_sources,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "enabled_requirement",
            expected_enabled_requirements,
            enabled_requirements,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "reason_code",
            sorted(reason_codes),
            allowed_reason_codes,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "contract_version",
            contract_versions,
            allowed_contract_versions,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "summary_version",
            summary_versions,
            allowed_summary_versions,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "validation_summary_status",
            validation_summary_statuses,
            allowed_validation_summary_statuses,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors("level", levels, allowed_levels)
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "skipped_reason",
            skipped_reasons,
            allowed_skipped_reasons,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors("source", sources, allowed_sources)
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "verification_scope",
            verification_scopes,
            allowed_verification_scopes,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "acceptance_profile",
            acceptance_profiles,
            allowed_acceptance_profiles,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "enabled_requirement",
            enabled_requirements,
            allowed_enabled_requirements,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "complete_required_summary_fields_source_scope",
            complete_required_summary_fields_source_scopes,
            allowed_complete_required_summary_fields_source_scopes,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "complete_required_summary_fields_source_scope",
            complete_required_summary_fields_source_scopes,
            forbidden_complete_required_summary_fields_source_scopes,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "reason_code",
            list(reason_codes),
            forbidden_reason_codes,
            order_by_forbidden=True,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "contract_version",
            contract_versions,
            forbidden_contract_versions,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "summary_version",
            summary_versions,
            forbidden_summary_versions,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "validation_summary_status",
            validation_summary_statuses,
            forbidden_validation_summary_statuses,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors("level", levels, forbidden_levels)
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "skipped_reason",
            skipped_reasons,
            forbidden_skipped_reasons,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors("source", sources, forbidden_sources)
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "verification_scope",
            verification_scopes,
            forbidden_verification_scopes,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "acceptance_profile",
            acceptance_profiles,
            forbidden_acceptance_profiles,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "enabled_requirement",
            enabled_requirements,
            forbidden_enabled_requirements,
        )
    )

    def rebuild_payloads() -> tuple[dict[str, Any], dict[str, Any]]:
        summary = _build_summary_payload(
            results=results,
            aggregate_errors=aggregate_errors,
            require_passed=args.require_passed,
            summary_version=(
                workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
            ),
            require_required=args.require_required,
            require_complete=args.require_complete,
            require_full_mechanical_restoration_gate=(
                args.require_full_mechanical_restoration_gate
            ),
            fail_on_full_mechanical_gate_false=(
                args.fail_on_full_mechanical_gate_false
            ),
            fail_on_full_mechanical_gate_unknown=(
                args.fail_on_full_mechanical_gate_unknown
            ),
            fail_on_smoke_report_missing=args.fail_on_smoke_report_missing,
            fail_on_smoke_report_read_error=(
                args.fail_on_smoke_report_read_error
            ),
            fail_on_control_readback_missing=(
                args.fail_on_control_readback_missing
            ),
            fail_on_node_tree_manifest_sidecar_incomplete=(
                args.fail_on_node_tree_manifest_sidecar_incomplete
            ),
            fail_on_invalid_node_tree_manifest_sidecar=(
                args.fail_on_invalid_node_tree_manifest_sidecar
            ),
            fail_on_node_tree_manifest_sidecar_validation_error=(
                args.fail_on_node_tree_manifest_sidecar_validation_error
            ),
            fail_on_node_tree_manifest_sidecar_path_incomplete=(
                args.fail_on_node_tree_manifest_sidecar_path_incomplete
            ),
            fail_on_node_tree_manifest_sidecar_path_map_mismatch=(
                args.fail_on_node_tree_manifest_sidecar_path_map_mismatch
            ),
            preview_limit=args.preview_limit,
            expanded_inputs_count=len(expanded_input_paths),
            expected_expanded_inputs_count=args.expect_expanded_inputs_count,
            expected_excluded_output_artifacts_count=(
                args.expect_excluded_output_artifacts_count
            ),
            expected_inputs_count=args.expect_inputs_count,
            expected_success_count=args.expect_success_count,
            expected_error_count=args.expect_error_count,
            expected_passed_true_count=args.expect_passed_true_count,
            expected_passed_false_count=args.expect_passed_false_count,
            expected_passed_unknown_count=args.expect_passed_unknown_count,
            expected_full_mechanical_gate_true_count=(
                args.expect_full_mechanical_gate_true_count
            ),
            expected_full_mechanical_gate_false_count=(
                args.expect_full_mechanical_gate_false_count
            ),
            expected_full_mechanical_gate_unknown_count=(
                args.expect_full_mechanical_gate_unknown_count
            ),
            expected_smoke_report_written_count=(
                args.expect_smoke_report_written_count
            ),
            expected_smoke_report_missing_count=(
                args.expect_smoke_report_missing_count
            ),
            expected_smoke_report_read_error_count=(
                args.expect_smoke_report_read_error_count
            ),
            expected_control_configured_count=(
                args.expect_control_configured_count
            ),
            expected_control_readback_checked_count=(
                args.expect_control_readback_checked_count
            ),
            expected_control_readback_missing_count=(
                args.expect_control_readback_missing_count
            ),
            expected_node_tree_manifest_sidecar_count=(
                args.expect_node_tree_manifest_sidecar_count
            ),
            expected_node_tree_manifest_sidecar_complete_count=(
                args.expect_node_tree_manifest_sidecar_complete_count
            ),
            expected_node_tree_manifest_sidecar_incomplete_count=(
                args.expect_node_tree_manifest_sidecar_incomplete_count
            ),
            expected_node_tree_manifest_sidecar_valid_count=(
                args.expect_node_tree_manifest_sidecar_valid_count
            ),
            expected_node_tree_manifest_sidecar_invalid_count=(
                args.expect_node_tree_manifest_sidecar_invalid_count
            ),
            expected_node_tree_manifest_sidecar_validation_error_count=(
                args.expect_node_tree_manifest_sidecar_validation_error_count
            ),
            expected_node_tree_manifest_sidecar_path_incomplete_count=(
                args.expect_node_tree_manifest_sidecar_path_incomplete_count
            ),
            expected_node_tree_manifest_sidecar_path_map_mismatch_count=(
                args.expect_node_tree_manifest_sidecar_path_map_mismatch_count
            ),
            expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts=(
                expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts
            ),
            expected_node_tree_manifest_sidecar_parts_planned_count=(
                args.expect_node_tree_manifest_sidecar_parts_planned_count
            ),
            expected_node_tree_manifest_sidecar_joints_planned_count=(
                args.expect_node_tree_manifest_sidecar_joints_planned_count
            ),
            expected_node_tree_manifest_sidecar_part_path_count=(
                args.expect_node_tree_manifest_sidecar_part_path_count
            ),
            expected_node_tree_manifest_sidecar_joint_path_count=(
                args.expect_node_tree_manifest_sidecar_joint_path_count
            ),
            expected_node_tree_gate_check_counts=(
                expected_node_tree_gate_check_counts
            ),
            expected_mechanical_gate_check_counts=(
                expected_mechanical_gate_check_counts
            ),
            expected_summary_counts=expected_summary_counts,
            expected_summary_values=expected_summary_values,
            expected_summary_value_sources=expected_summary_value_sources,
            expected_summary_value_source_matched_count=(
                args.expect_summary_value_source_matched_count
            ),
            expected_summary_value_source_excluded_count=(
                args.expect_summary_value_source_excluded_count
            ),
            expected_summary_value_source_excluded_sources=(
                expected_summary_value_source_excluded_sources
            ),
            allowed_summary_value_source_matched_sources=(
                allowed_summary_value_source_matched_sources
            ),
            forbidden_summary_value_source_matched_sources=(
                forbidden_summary_value_source_matched_sources
            ),
            allowed_summary_value_source_excluded_sources=(
                allowed_summary_value_source_excluded_sources
            ),
            forbidden_summary_value_source_excluded_sources=(
                forbidden_summary_value_source_excluded_sources
            ),
            expected_skipped_count=args.expect_skipped_count,
            expected_reason_codes_count=args.expect_reason_codes_count,
            expected_contract_versions_count=(
                args.expect_contract_versions_count
            ),
            expected_summary_versions_count=(
                args.expect_summary_versions_count
            ),
            expected_validation_summary_statuses_count=(
                args.expect_validation_summary_statuses_count
            ),
            expected_levels_count=args.expect_levels_count,
            expected_sources_count=args.expect_sources_count,
            expected_verification_scopes_count=(
                args.expect_verification_scopes_count
            ),
            expected_acceptance_profiles_count=(
                args.expect_acceptance_profiles_count
            ),
            expected_enabled_requirements_count=(
                args.expect_enabled_requirements_count
            ),
            expected_complete_required_summary_fields_source_scope_count=(
                args.expect_complete_required_summary_fields_source_scope_count
            ),
            expected_affected_inputs_count=args.expect_affected_inputs_count,
            expected_failed_inputs_count=args.expect_failed_inputs_count,
            expected_skipped_inputs_count=args.expect_skipped_inputs_count,
            expected_skipped_reasons_count=args.expect_skipped_reasons_count,
            expected_truncated_previews_count=(
                args.expect_truncated_previews_count
            ),
            expected_contract_versions=expected_contract_versions,
            expected_summary_versions=expected_summary_versions,
            expected_validation_summary_statuses=(
                expected_validation_summary_statuses
            ),
            expected_levels=expected_levels,
            expected_sources=expected_sources,
            expected_verification_scopes=expected_verification_scopes,
            expected_acceptance_profiles=expected_acceptance_profiles,
            expected_enabled_requirements=expected_enabled_requirements,
            expected_complete_required_summary_fields_source_scopes=(
                expected_complete_required_summary_fields_source_scopes
            ),
            allowed_complete_required_summary_fields_source_scopes=(
                allowed_complete_required_summary_fields_source_scopes
            ),
            forbidden_complete_required_summary_fields_source_scopes=(
                forbidden_complete_required_summary_fields_source_scopes
            ),
            expected_reason_codes=expected_reason_codes,
            expected_inputs=expected_inputs,
            expected_affected_inputs=expected_affected_inputs,
            expected_failed_inputs=expected_failed_inputs,
            expected_passed_true_inputs=expected_passed_true_inputs,
            expected_passed_false_inputs=expected_passed_false_inputs,
            expected_passed_unknown_inputs=expected_passed_unknown_inputs,
            expected_full_mechanical_gate_true_inputs=(
                expected_full_mechanical_gate_true_inputs
            ),
            expected_full_mechanical_gate_false_inputs=(
                expected_full_mechanical_gate_false_inputs
            ),
            expected_full_mechanical_gate_unknown_inputs=(
                expected_full_mechanical_gate_unknown_inputs
            ),
            expected_truncated_previews=expected_truncated_previews,
            expected_skipped_inputs=expected_skipped_inputs,
            expected_skipped_reasons=expected_skipped_reasons,
            allowed_contract_versions=allowed_contract_versions,
            allowed_summary_versions=allowed_summary_versions,
            allowed_validation_summary_statuses=(
                allowed_validation_summary_statuses
            ),
            allowed_levels=allowed_levels,
            allowed_sources=allowed_sources,
            allowed_verification_scopes=allowed_verification_scopes,
            allowed_acceptance_profiles=allowed_acceptance_profiles,
            allowed_enabled_requirements=allowed_enabled_requirements,
            allowed_reason_codes=allowed_reason_codes,
            allowed_inputs=allowed_inputs,
            allowed_affected_inputs=allowed_affected_inputs,
            allowed_failed_inputs=allowed_failed_inputs,
            allowed_passed_true_inputs=allowed_passed_true_inputs,
            allowed_passed_false_inputs=allowed_passed_false_inputs,
            allowed_passed_unknown_inputs=allowed_passed_unknown_inputs,
            allowed_full_mechanical_gate_true_inputs=(
                allowed_full_mechanical_gate_true_inputs
            ),
            allowed_full_mechanical_gate_false_inputs=(
                allowed_full_mechanical_gate_false_inputs
            ),
            allowed_full_mechanical_gate_unknown_inputs=(
                allowed_full_mechanical_gate_unknown_inputs
            ),
            allowed_truncated_previews=allowed_truncated_previews,
            allowed_skipped_inputs=allowed_skipped_inputs,
            allowed_skipped_reasons=allowed_skipped_reasons,
            forbidden_contract_versions=forbidden_contract_versions,
            forbidden_summary_versions=forbidden_summary_versions,
            forbidden_validation_summary_statuses=(
                forbidden_validation_summary_statuses
            ),
            forbidden_levels=forbidden_levels,
            forbidden_sources=forbidden_sources,
            forbidden_verification_scopes=forbidden_verification_scopes,
            forbidden_acceptance_profiles=forbidden_acceptance_profiles,
            forbidden_enabled_requirements=forbidden_enabled_requirements,
            forbidden_reason_codes=forbidden_reason_codes,
            forbidden_inputs=forbidden_inputs,
            forbidden_affected_inputs=forbidden_affected_inputs,
            forbidden_failed_inputs=forbidden_failed_inputs,
            forbidden_passed_true_inputs=forbidden_passed_true_inputs,
            forbidden_passed_false_inputs=forbidden_passed_false_inputs,
            forbidden_passed_unknown_inputs=forbidden_passed_unknown_inputs,
            forbidden_full_mechanical_gate_true_inputs=(
                forbidden_full_mechanical_gate_true_inputs
            ),
            forbidden_full_mechanical_gate_false_inputs=(
                forbidden_full_mechanical_gate_false_inputs
            ),
            forbidden_full_mechanical_gate_unknown_inputs=(
                forbidden_full_mechanical_gate_unknown_inputs
            ),
            forbidden_truncated_previews=forbidden_truncated_previews,
            forbidden_skipped_inputs=forbidden_skipped_inputs,
            forbidden_skipped_reasons=forbidden_skipped_reasons,
            excluded_output_artifact_inputs=excluded_output_artifact_inputs,
        )
        return summary, _build_result_payload(
            results=results,
            summary_payload=summary,
            aggregate_errors=aggregate_errors,
        )

    summary_payload, result = rebuild_payloads()
    aggregate_errors.extend(
        _expected_count_errors(
            [
                (
                    "truncated_previews_count",
                    args.expect_truncated_previews_count,
                    len(summary_payload["truncated_previews"]),
                )
            ]
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "truncated_preview",
            expected_truncated_previews,
            summary_payload["truncated_previews"],
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "input",
            expected_inputs,
            validation_inputs,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "skipped_input",
            expected_skipped_inputs,
            skipped_inputs,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "failed_input",
            expected_failed_inputs,
            failed_inputs,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "affected_input",
            expected_affected_inputs,
            affected_inputs,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "passed_unknown_input",
            expected_passed_unknown_inputs,
            passed_unknown_inputs,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "passed_true_input",
            expected_passed_true_inputs,
            passed_true_inputs,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "passed_false_input",
            expected_passed_false_inputs,
            passed_false_inputs,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "full_mechanical_gate_true_input",
            expected_full_mechanical_gate_true_inputs,
            full_mechanical_gate_true_inputs,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "full_mechanical_gate_false_input",
            expected_full_mechanical_gate_false_inputs,
            full_mechanical_gate_false_inputs,
        )
    )
    aggregate_errors.extend(
        _expected_value_errors(
            "full_mechanical_gate_unknown_input",
            expected_full_mechanical_gate_unknown_inputs,
            full_mechanical_gate_unknown_inputs,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "truncated_preview",
            summary_payload["truncated_previews"],
            allowed_truncated_previews,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "input",
            validation_inputs,
            allowed_inputs,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "skipped_input",
            skipped_inputs,
            allowed_skipped_inputs,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "failed_input",
            failed_inputs,
            allowed_failed_inputs,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "affected_input",
            affected_inputs,
            allowed_affected_inputs,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "passed_unknown_input",
            passed_unknown_inputs,
            allowed_passed_unknown_inputs,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "passed_true_input",
            passed_true_inputs,
            allowed_passed_true_inputs,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "passed_false_input",
            passed_false_inputs,
            allowed_passed_false_inputs,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "full_mechanical_gate_true_input",
            full_mechanical_gate_true_inputs,
            allowed_full_mechanical_gate_true_inputs,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "full_mechanical_gate_false_input",
            full_mechanical_gate_false_inputs,
            allowed_full_mechanical_gate_false_inputs,
        )
    )
    aggregate_errors.extend(
        _unexpected_value_errors(
            "full_mechanical_gate_unknown_input",
            full_mechanical_gate_unknown_inputs,
            allowed_full_mechanical_gate_unknown_inputs,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "truncated_preview",
            summary_payload["truncated_previews"],
            forbidden_truncated_previews,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "input",
            validation_inputs,
            forbidden_inputs,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "skipped_input",
            skipped_inputs,
            forbidden_skipped_inputs,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "failed_input",
            failed_inputs,
            forbidden_failed_inputs,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "affected_input",
            affected_inputs,
            forbidden_affected_inputs,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "passed_unknown_input",
            passed_unknown_inputs,
            forbidden_passed_unknown_inputs,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "passed_true_input",
            passed_true_inputs,
            forbidden_passed_true_inputs,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "passed_false_input",
            passed_false_inputs,
            forbidden_passed_false_inputs,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "full_mechanical_gate_true_input",
            full_mechanical_gate_true_inputs,
            forbidden_full_mechanical_gate_true_inputs,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "full_mechanical_gate_false_input",
            full_mechanical_gate_false_inputs,
            forbidden_full_mechanical_gate_false_inputs,
        )
    )
    aggregate_errors.extend(
        _forbidden_value_errors(
            "full_mechanical_gate_unknown_input",
            full_mechanical_gate_unknown_inputs,
            forbidden_full_mechanical_gate_unknown_inputs,
        )
    )
    if aggregate_errors:
        summary_payload, result = rebuild_payloads()
    if args.fail_on_truncated_previews and summary_payload["truncated_previews"]:
        aggregate_errors.append(
            "truncated previews present: "
            + ",".join(summary_payload["truncated_previews"])
        )
        summary_payload, result = rebuild_payloads()
    summary_contract_errors = (
        workflow_contracts.validate_delivery_acceptance_validation_summary(
            summary_payload
        )
    )
    if summary_contract_errors:
        aggregate_errors.append(
            "validation summary contract invalid: "
            + "; ".join(summary_contract_errors)
        )
        summary_payload, result = rebuild_payloads()
    output_written = False
    if args.output is not None:
        output_payload = _output_payload_for_shape(
            output_shape=args.output_shape,
            summary_only=args.summary_only,
            summary_payload=summary_payload,
            result=result,
        )
        output_error = _write_json(args.output, output_payload, label="output")
        if output_error is not None:
            aggregate_errors.append(output_error)
            summary_payload, result = rebuild_payloads()
        else:
            output_written = True
    if args.summary_output is not None:
        summary_error = _write_json(
            args.summary_output,
            summary_payload,
            label="summary output",
        )
        if summary_error is not None:
            aggregate_errors.append(summary_error)
            summary_payload, result = rebuild_payloads()
            if output_written and args.output is not None:
                output_payload = _output_payload_for_shape(
                    output_shape=args.output_shape,
                    summary_only=args.summary_only,
                    summary_payload=summary_payload,
                    result=result,
                )
                output_rewrite_error = _write_json(
                    args.output,
                    output_payload,
                    label="output",
                )
                if output_rewrite_error is not None:
                    aggregate_errors.append(output_rewrite_error)
                    summary_payload, result = rebuild_payloads()
    if args.format == "text":
        lines = []
        if _should_emit_text_summary(
            args=args,
            results=results,
            summary_payload=summary_payload,
            aggregate_errors=aggregate_errors,
        ):
            lines.append(
                _format_text_summary(
                    preview_limit=(
                        summary_payload["preview_limit"]
                        if _option_supplied(sys.argv[1:], "--preview-limit")
                        else None
                    ),
                    inputs_count=len(results),
                    expanded_inputs_count=summary_payload["expanded_inputs_count"],
                    expected_expanded_inputs_count=summary_payload[
                        "expected_expanded_inputs_count"
                    ],
                    expected_excluded_output_artifacts_count=summary_payload[
                        "expected_excluded_output_artifacts_count"
                    ],
                    success_count=len(successes),
                    error_count=len(failed),
                    passed_true_count=summary_payload["passed_true_count"],
                    passed_false_count=summary_payload["passed_false_count"],
                    passed_unknown_count=summary_payload["passed_unknown_count"],
                    requires_full_mechanical_restoration_gate_true_count=(
                        summary_payload[
                            "requires_full_mechanical_restoration_gate_true_count"
                        ]
                    ),
                    requires_full_mechanical_restoration_gate_false_count=(
                        summary_payload[
                            "requires_full_mechanical_restoration_gate_false_count"
                        ]
                    ),
                    requires_full_mechanical_restoration_gate_unknown_count=(
                        summary_payload[
                            "requires_full_mechanical_restoration_gate_unknown_count"
                        ]
                    ),
                    smoke_report_written_count=summary_payload[
                        "smoke_report_written_count"
                    ],
                    smoke_report_missing_count=summary_payload[
                        "smoke_report_missing_count"
                    ],
                    smoke_report_read_error_count=summary_payload[
                        "smoke_report_read_error_count"
                    ],
                    control_configured_count=summary_payload[
                        "control_configured_count"
                    ],
                    control_readback_checked_count=summary_payload[
                        "control_readback_checked_count"
                    ],
                    control_readback_missing_count=summary_payload[
                        "control_readback_missing_count"
                    ],
                    node_tree_gate_check_counts=summary_payload[
                        "node_tree_gate_check_counts"
                    ],
                    mechanical_gate_check_counts=summary_payload[
                        "mechanical_gate_check_counts"
                    ],
                    expected_node_tree_gate_check_counts=summary_payload[
                        "expected_node_tree_gate_check_counts"
                    ],
                    expected_mechanical_gate_check_counts=summary_payload[
                        "expected_mechanical_gate_check_counts"
                    ],
                    mismatched_expected_node_tree_gate_check_counts=(
                        summary_payload[
                            "mismatched_expected_node_tree_gate_check_counts"
                        ]
                    ),
                    mismatched_expected_mechanical_gate_check_counts=(
                        summary_payload[
                            "mismatched_expected_mechanical_gate_check_counts"
                        ]
                    ),
                    expected_summary_counts=summary_payload[
                        "expected_summary_counts"
                    ],
                    mismatched_expected_summary_counts=summary_payload[
                        "mismatched_expected_summary_counts"
                    ],
                    expected_summary_values=summary_payload[
                        "expected_summary_values"
                    ],
                    expected_summary_value_sources=summary_payload[
                        "expected_summary_value_sources"
                    ],
                    missing_expected_summary_value_sources=summary_payload[
                        "missing_expected_summary_value_sources"
                    ],
                    expected_summary_value_source_matched_count=summary_payload[
                        "expected_summary_value_source_matched_count"
                    ],
                    expected_summary_value_source_excluded_count=summary_payload[
                        "expected_summary_value_source_excluded_count"
                    ],
                    expected_summary_value_source_excluded_sources=summary_payload[
                        "expected_summary_value_source_excluded_sources"
                    ],
                    missing_expected_summary_value_source_excluded_sources=(
                        summary_payload[
                            "missing_expected_summary_value_source_excluded_sources"
                        ]
                    ),
                    allowed_summary_value_source_excluded_sources=summary_payload[
                        "allowed_summary_value_source_excluded_sources"
                    ],
                    unexpected_summary_value_source_excluded_sources=(
                        summary_payload[
                            "unexpected_summary_value_source_excluded_sources"
                        ]
                    ),
                    forbidden_summary_value_source_excluded_sources=summary_payload[
                        "forbidden_summary_value_source_excluded_sources"
                    ],
                    present_forbidden_summary_value_source_excluded_sources=(
                        summary_payload[
                            "present_forbidden_summary_value_source_excluded_sources"
                        ]
                    ),
                    summary_value_source_matched_count=summary_payload[
                        "summary_value_source_matched_count"
                    ],
                    summary_value_source_matched_sources=summary_payload[
                        "summary_value_source_matched_sources"
                    ],
                    allowed_summary_value_source_matched_sources=summary_payload[
                        "allowed_summary_value_source_matched_sources"
                    ],
                    unexpected_summary_value_source_matched_sources=(
                        summary_payload[
                            "unexpected_summary_value_source_matched_sources"
                        ]
                    ),
                    forbidden_summary_value_source_matched_sources=summary_payload[
                        "forbidden_summary_value_source_matched_sources"
                    ],
                    present_forbidden_summary_value_source_matched_sources=(
                        summary_payload[
                            "present_forbidden_summary_value_source_matched_sources"
                        ]
                    ),
                    summary_value_source_excluded_count=summary_payload[
                        "summary_value_source_excluded_count"
                    ],
                    summary_value_source_excluded_sources=summary_payload[
                        "summary_value_source_excluded_sources"
                    ],
                    mismatched_expected_summary_values=summary_payload[
                        "mismatched_expected_summary_values"
                    ],
                    skipped_count=len(skipped),
                    aggregate_errors=aggregate_errors,
                    input_paths=summary_payload["input_paths"],
                    expected_inputs=summary_payload["expected_inputs"],
                    missing_expected_inputs=summary_payload[
                        "missing_expected_inputs"
                    ],
                    allowed_inputs=summary_payload["allowed_inputs"],
                    unexpected_inputs=summary_payload["unexpected_inputs"],
                    forbidden_inputs=summary_payload["forbidden_inputs"],
                    present_forbidden_inputs=summary_payload[
                        "present_forbidden_inputs"
                    ],
                    contract_versions=summary_payload["contract_versions"],
                    levels=summary_payload["levels"],
                    sources=summary_payload["sources"],
                    verification_scopes=summary_payload["verification_scopes"],
                    acceptance_profiles=summary_payload["acceptance_profiles"],
                    enabled_requirements=summary_payload[
                        "enabled_requirements"
                    ],
                    complete_required_summary_fields_by_source_scope=(
                        summary_payload[
                            "complete_required_summary_fields_by_source_scope"
                        ]
                    ),
                    complete_required_summary_fields_source_scope_count=(
                        summary_payload[
                            "complete_required_summary_fields_source_scope_count"
                        ]
                    ),
                    complete_required_summary_fields_source_scopes=(
                        summary_payload[
                            "complete_required_summary_fields_source_scopes"
                        ]
                    ),
                    reason_codes=summary_payload["reason_codes"],
                    affected_inputs=summary_payload["affected_inputs"],
                    failed_inputs=summary_payload["failed_inputs"],
                    skipped_inputs=summary_payload["skipped_inputs"],
                    skipped_reasons=summary_payload["skipped_reasons"],
                    truncated_previews=summary_payload["truncated_previews"],
                    expected_affected_inputs=summary_payload[
                        "expected_affected_inputs"
                    ],
                    missing_expected_affected_inputs=summary_payload[
                        "missing_expected_affected_inputs"
                    ],
                    allowed_affected_inputs=summary_payload[
                        "allowed_affected_inputs"
                    ],
                    unexpected_affected_inputs=summary_payload[
                        "unexpected_affected_inputs"
                    ],
                    forbidden_affected_inputs=summary_payload[
                        "forbidden_affected_inputs"
                    ],
                    present_forbidden_affected_inputs=summary_payload[
                        "present_forbidden_affected_inputs"
                    ],
                    expected_failed_inputs=summary_payload[
                        "expected_failed_inputs"
                    ],
                    missing_expected_failed_inputs=summary_payload[
                        "missing_expected_failed_inputs"
                    ],
                    allowed_failed_inputs=summary_payload[
                        "allowed_failed_inputs"
                    ],
                    unexpected_failed_inputs=summary_payload[
                        "unexpected_failed_inputs"
                    ],
                    forbidden_failed_inputs=summary_payload[
                        "forbidden_failed_inputs"
                    ],
                    present_forbidden_failed_inputs=summary_payload[
                        "present_forbidden_failed_inputs"
                    ],
                    passed_true_inputs=summary_payload["passed_true_inputs"],
                    expected_passed_true_inputs=summary_payload[
                        "expected_passed_true_inputs"
                    ],
                    missing_expected_passed_true_inputs=summary_payload[
                        "missing_expected_passed_true_inputs"
                    ],
                    allowed_passed_true_inputs=summary_payload[
                        "allowed_passed_true_inputs"
                    ],
                    unexpected_passed_true_inputs=summary_payload[
                        "unexpected_passed_true_inputs"
                    ],
                    forbidden_passed_true_inputs=summary_payload[
                        "forbidden_passed_true_inputs"
                    ],
                    present_forbidden_passed_true_inputs=summary_payload[
                        "present_forbidden_passed_true_inputs"
                    ],
                    passed_false_inputs=summary_payload["passed_false_inputs"],
                    expected_passed_false_inputs=summary_payload[
                        "expected_passed_false_inputs"
                    ],
                    missing_expected_passed_false_inputs=summary_payload[
                        "missing_expected_passed_false_inputs"
                    ],
                    allowed_passed_false_inputs=summary_payload[
                        "allowed_passed_false_inputs"
                    ],
                    unexpected_passed_false_inputs=summary_payload[
                        "unexpected_passed_false_inputs"
                    ],
                    forbidden_passed_false_inputs=summary_payload[
                        "forbidden_passed_false_inputs"
                    ],
                    present_forbidden_passed_false_inputs=summary_payload[
                        "present_forbidden_passed_false_inputs"
                    ],
                    passed_unknown_inputs=summary_payload[
                        "passed_unknown_inputs"
                    ],
                    expected_passed_unknown_inputs=summary_payload[
                        "expected_passed_unknown_inputs"
                    ],
                    missing_expected_passed_unknown_inputs=summary_payload[
                        "missing_expected_passed_unknown_inputs"
                    ],
                    allowed_passed_unknown_inputs=summary_payload[
                        "allowed_passed_unknown_inputs"
                    ],
                    unexpected_passed_unknown_inputs=summary_payload[
                        "unexpected_passed_unknown_inputs"
                    ],
                    forbidden_passed_unknown_inputs=summary_payload[
                        "forbidden_passed_unknown_inputs"
                    ],
                    present_forbidden_passed_unknown_inputs=summary_payload[
                        "present_forbidden_passed_unknown_inputs"
                    ],
                    full_mechanical_gate_true_inputs=summary_payload[
                        "full_mechanical_gate_true_inputs"
                    ],
                    expected_full_mechanical_gate_true_inputs=summary_payload[
                        "expected_full_mechanical_gate_true_inputs"
                    ],
                    missing_expected_full_mechanical_gate_true_inputs=(
                        summary_payload[
                            "missing_expected_full_mechanical_gate_true_inputs"
                        ]
                    ),
                    allowed_full_mechanical_gate_true_inputs=summary_payload[
                        "allowed_full_mechanical_gate_true_inputs"
                    ],
                    unexpected_full_mechanical_gate_true_inputs=summary_payload[
                        "unexpected_full_mechanical_gate_true_inputs"
                    ],
                    forbidden_full_mechanical_gate_true_inputs=summary_payload[
                        "forbidden_full_mechanical_gate_true_inputs"
                    ],
                    present_forbidden_full_mechanical_gate_true_inputs=(
                        summary_payload[
                            "present_forbidden_full_mechanical_gate_true_inputs"
                        ]
                    ),
                    full_mechanical_gate_false_inputs=summary_payload[
                        "full_mechanical_gate_false_inputs"
                    ],
                    expected_full_mechanical_gate_false_inputs=summary_payload[
                        "expected_full_mechanical_gate_false_inputs"
                    ],
                    missing_expected_full_mechanical_gate_false_inputs=(
                        summary_payload[
                            "missing_expected_full_mechanical_gate_false_inputs"
                        ]
                    ),
                    allowed_full_mechanical_gate_false_inputs=summary_payload[
                        "allowed_full_mechanical_gate_false_inputs"
                    ],
                    unexpected_full_mechanical_gate_false_inputs=summary_payload[
                        "unexpected_full_mechanical_gate_false_inputs"
                    ],
                    forbidden_full_mechanical_gate_false_inputs=summary_payload[
                        "forbidden_full_mechanical_gate_false_inputs"
                    ],
                    present_forbidden_full_mechanical_gate_false_inputs=(
                        summary_payload[
                            "present_forbidden_full_mechanical_gate_false_inputs"
                        ]
                    ),
                    full_mechanical_gate_unknown_inputs=summary_payload[
                        "full_mechanical_gate_unknown_inputs"
                    ],
                    expected_full_mechanical_gate_unknown_inputs=summary_payload[
                        "expected_full_mechanical_gate_unknown_inputs"
                    ],
                    missing_expected_full_mechanical_gate_unknown_inputs=(
                        summary_payload[
                            "missing_expected_full_mechanical_gate_unknown_inputs"
                        ]
                    ),
                    allowed_full_mechanical_gate_unknown_inputs=summary_payload[
                        "allowed_full_mechanical_gate_unknown_inputs"
                    ],
                    unexpected_full_mechanical_gate_unknown_inputs=summary_payload[
                        "unexpected_full_mechanical_gate_unknown_inputs"
                    ],
                    forbidden_full_mechanical_gate_unknown_inputs=summary_payload[
                        "forbidden_full_mechanical_gate_unknown_inputs"
                    ],
                    present_forbidden_full_mechanical_gate_unknown_inputs=(
                        summary_payload[
                            "present_forbidden_full_mechanical_gate_unknown_inputs"
                        ]
                    ),
                    expected_skipped_inputs=summary_payload[
                        "expected_skipped_inputs"
                    ],
                    missing_expected_skipped_inputs=summary_payload[
                        "missing_expected_skipped_inputs"
                    ],
                    allowed_skipped_inputs=summary_payload[
                        "allowed_skipped_inputs"
                    ],
                    unexpected_skipped_inputs=summary_payload[
                        "unexpected_skipped_inputs"
                    ],
                    forbidden_skipped_inputs=summary_payload[
                        "forbidden_skipped_inputs"
                    ],
                    present_forbidden_skipped_inputs=summary_payload[
                        "present_forbidden_skipped_inputs"
                    ],
                    expected_truncated_previews=summary_payload[
                        "expected_truncated_previews"
                    ],
                    missing_expected_truncated_previews=summary_payload[
                        "missing_expected_truncated_previews"
                    ],
                    allowed_truncated_previews=summary_payload[
                        "allowed_truncated_previews"
                    ],
                    unexpected_truncated_previews=summary_payload[
                        "unexpected_truncated_previews"
                    ],
                    forbidden_truncated_previews=summary_payload[
                        "forbidden_truncated_previews"
                    ],
                    present_forbidden_truncated_previews=summary_payload[
                        "present_forbidden_truncated_previews"
                    ],
                    expected_inputs_count=summary_payload[
                        "expected_inputs_count"
                    ],
                    expected_success_count=summary_payload[
                        "expected_success_count"
                    ],
                    expected_error_count=summary_payload[
                        "expected_error_count"
                    ],
                    expected_passed_true_count=summary_payload[
                        "expected_passed_true_count"
                    ],
                    expected_passed_false_count=summary_payload[
                        "expected_passed_false_count"
                    ],
                    expected_passed_unknown_count=summary_payload[
                        "expected_passed_unknown_count"
                    ],
                    expected_full_mechanical_gate_true_count=summary_payload[
                        "expected_full_mechanical_gate_true_count"
                    ],
                    expected_full_mechanical_gate_false_count=summary_payload[
                        "expected_full_mechanical_gate_false_count"
                    ],
                    expected_full_mechanical_gate_unknown_count=summary_payload[
                        "expected_full_mechanical_gate_unknown_count"
                    ],
                    expected_smoke_report_written_count=summary_payload[
                        "expected_smoke_report_written_count"
                    ],
                    expected_smoke_report_missing_count=summary_payload[
                        "expected_smoke_report_missing_count"
                    ],
                    expected_smoke_report_read_error_count=summary_payload[
                        "expected_smoke_report_read_error_count"
                    ],
                    expected_control_configured_count=summary_payload[
                        "expected_control_configured_count"
                    ],
                    expected_control_readback_checked_count=summary_payload[
                        "expected_control_readback_checked_count"
                    ],
                    expected_control_readback_missing_count=summary_payload[
                        "expected_control_readback_missing_count"
                    ],
                    expected_node_tree_manifest_sidecar_count=summary_payload[
                        "expected_node_tree_manifest_sidecar_count"
                    ],
                    expected_node_tree_manifest_sidecar_complete_count=summary_payload[
                        "expected_node_tree_manifest_sidecar_complete_count"
                    ],
                    expected_node_tree_manifest_sidecar_incomplete_count=summary_payload[
                        "expected_node_tree_manifest_sidecar_incomplete_count"
                    ],
                    expected_node_tree_manifest_sidecar_valid_count=summary_payload[
                        "expected_node_tree_manifest_sidecar_valid_count"
                    ],
                    expected_node_tree_manifest_sidecar_invalid_count=summary_payload[
                        "expected_node_tree_manifest_sidecar_invalid_count"
                    ],
                    expected_node_tree_manifest_sidecar_validation_error_count=summary_payload[
                        "expected_node_tree_manifest_sidecar_validation_error_count"
                    ],
                    expected_node_tree_manifest_sidecar_path_incomplete_count=summary_payload[
                        "expected_node_tree_manifest_sidecar_path_incomplete_count"
                    ],
                    expected_node_tree_manifest_sidecar_path_map_mismatch_count=summary_payload[
                        "expected_node_tree_manifest_sidecar_path_map_mismatch_count"
                    ],
                    expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts=summary_payload[
                        "expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts"
                    ],
                    expected_node_tree_manifest_sidecar_parts_planned_count=summary_payload[
                        "expected_node_tree_manifest_sidecar_parts_planned_count"
                    ],
                    expected_node_tree_manifest_sidecar_joints_planned_count=summary_payload[
                        "expected_node_tree_manifest_sidecar_joints_planned_count"
                    ],
                    expected_node_tree_manifest_sidecar_part_path_count=summary_payload[
                        "expected_node_tree_manifest_sidecar_part_path_count"
                    ],
                    expected_node_tree_manifest_sidecar_joint_path_count=summary_payload[
                        "expected_node_tree_manifest_sidecar_joint_path_count"
                    ],
                    expected_skipped_count=summary_payload[
                        "expected_skipped_count"
                    ],
                    expected_reason_codes_count=summary_payload[
                        "expected_reason_codes_count"
                    ],
                    expected_contract_versions_count=summary_payload[
                        "expected_contract_versions_count"
                    ],
                    expected_levels_count=summary_payload[
                        "expected_levels_count"
                    ],
                    expected_sources_count=summary_payload[
                        "expected_sources_count"
                    ],
                    expected_verification_scopes_count=summary_payload[
                        "expected_verification_scopes_count"
                    ],
                    expected_acceptance_profiles_count=summary_payload[
                        "expected_acceptance_profiles_count"
                    ],
                    expected_enabled_requirements_count=summary_payload[
                        "expected_enabled_requirements_count"
                    ],
                    expected_complete_required_summary_fields_source_scope_count=(
                        summary_payload[
                            "expected_complete_required_summary_fields_source_scope_count"
                        ]
                    ),
                    expected_affected_inputs_count=summary_payload[
                        "expected_affected_inputs_count"
                    ],
                    expected_failed_inputs_count=summary_payload[
                        "expected_failed_inputs_count"
                    ],
                    expected_skipped_inputs_count=summary_payload[
                        "expected_skipped_inputs_count"
                    ],
                    expected_skipped_reasons_count=summary_payload[
                        "expected_skipped_reasons_count"
                    ],
                    expected_truncated_previews_count=summary_payload[
                        "expected_truncated_previews_count"
                    ],
                    expected_contract_versions=summary_payload[
                        "expected_contract_versions"
                    ],
                    missing_expected_contract_versions=summary_payload[
                        "missing_expected_contract_versions"
                    ],
                    expected_levels=summary_payload["expected_levels"],
                    missing_expected_levels=summary_payload[
                        "missing_expected_levels"
                    ],
                    expected_sources=summary_payload["expected_sources"],
                    missing_expected_sources=summary_payload[
                        "missing_expected_sources"
                    ],
                    expected_verification_scopes=summary_payload[
                        "expected_verification_scopes"
                    ],
                    missing_expected_verification_scopes=summary_payload[
                        "missing_expected_verification_scopes"
                    ],
                    expected_acceptance_profiles=summary_payload[
                        "expected_acceptance_profiles"
                    ],
                    missing_expected_acceptance_profiles=summary_payload[
                        "missing_expected_acceptance_profiles"
                    ],
                    expected_enabled_requirements=summary_payload[
                        "expected_enabled_requirements"
                    ],
                    missing_expected_enabled_requirements=summary_payload[
                        "missing_expected_enabled_requirements"
                    ],
                    expected_complete_required_summary_fields_source_scopes=(
                        summary_payload[
                            "expected_complete_required_summary_fields_source_scopes"
                        ]
                    ),
                    missing_expected_complete_required_summary_fields_source_scopes=(
                        summary_payload[
                            "missing_expected_complete_required_summary_fields_source_scopes"
                        ]
                    ),
                    allowed_complete_required_summary_fields_source_scopes=(
                        summary_payload[
                            "allowed_complete_required_summary_fields_source_scopes"
                        ]
                    ),
                    unexpected_complete_required_summary_fields_source_scopes=(
                        summary_payload[
                            "unexpected_complete_required_summary_fields_source_scopes"
                        ]
                    ),
                    forbidden_complete_required_summary_fields_source_scopes=(
                        summary_payload[
                            "forbidden_complete_required_summary_fields_source_scopes"
                        ]
                    ),
                    present_forbidden_complete_required_summary_fields_source_scopes=(
                        summary_payload[
                            "present_forbidden_complete_required_summary_fields_source_scopes"
                        ]
                    ),
                    allowed_contract_versions=summary_payload[
                        "allowed_contract_versions"
                    ],
                    unexpected_contract_versions=summary_payload[
                        "unexpected_contract_versions"
                    ],
                    allowed_levels=summary_payload["allowed_levels"],
                    unexpected_levels=summary_payload["unexpected_levels"],
                    allowed_sources=summary_payload["allowed_sources"],
                    unexpected_sources=summary_payload["unexpected_sources"],
                    allowed_verification_scopes=summary_payload[
                        "allowed_verification_scopes"
                    ],
                    unexpected_verification_scopes=summary_payload[
                        "unexpected_verification_scopes"
                    ],
                    allowed_acceptance_profiles=summary_payload[
                        "allowed_acceptance_profiles"
                    ],
                    unexpected_acceptance_profiles=summary_payload[
                        "unexpected_acceptance_profiles"
                    ],
                    allowed_enabled_requirements=summary_payload[
                        "allowed_enabled_requirements"
                    ],
                    unexpected_enabled_requirements=summary_payload[
                        "unexpected_enabled_requirements"
                    ],
                    forbidden_sources=summary_payload["forbidden_sources"],
                    present_forbidden_sources=summary_payload[
                        "present_forbidden_sources"
                    ],
                    forbidden_verification_scopes=summary_payload[
                        "forbidden_verification_scopes"
                    ],
                    present_forbidden_verification_scopes=summary_payload[
                        "present_forbidden_verification_scopes"
                    ],
                    forbidden_acceptance_profiles=summary_payload[
                        "forbidden_acceptance_profiles"
                    ],
                    present_forbidden_acceptance_profiles=summary_payload[
                        "present_forbidden_acceptance_profiles"
                    ],
                    forbidden_enabled_requirements=summary_payload[
                        "forbidden_enabled_requirements"
                    ],
                    present_forbidden_enabled_requirements=summary_payload[
                        "present_forbidden_enabled_requirements"
                    ],
                    forbidden_contract_versions=summary_payload[
                        "forbidden_contract_versions"
                    ],
                    present_forbidden_contract_versions=summary_payload[
                        "present_forbidden_contract_versions"
                    ],
                    forbidden_levels=summary_payload["forbidden_levels"],
                    present_forbidden_levels=summary_payload[
                        "present_forbidden_levels"
                    ],
                    expected_reason_codes=summary_payload[
                        "expected_reason_codes"
                    ],
                    missing_expected_reason_codes=summary_payload[
                        "missing_expected_reason_codes"
                    ],
                    allowed_reason_codes=summary_payload["allowed_reason_codes"],
                    unexpected_reason_codes=summary_payload[
                        "unexpected_reason_codes"
                    ],
                    forbidden_reason_codes=summary_payload[
                        "forbidden_reason_codes"
                    ],
                    present_forbidden_reason_codes=summary_payload[
                        "present_forbidden_reason_codes"
                    ],
                    expected_skipped_reasons=summary_payload[
                        "expected_skipped_reasons"
                    ],
                    missing_expected_skipped_reasons=summary_payload[
                        "missing_expected_skipped_reasons"
                    ],
                    allowed_skipped_reasons=summary_payload[
                        "allowed_skipped_reasons"
                    ],
                    unexpected_skipped_reasons=summary_payload[
                        "unexpected_skipped_reasons"
                    ],
                    forbidden_skipped_reasons=summary_payload[
                        "forbidden_skipped_reasons"
                    ],
                    present_forbidden_skipped_reasons=summary_payload[
                        "present_forbidden_skipped_reasons"
                    ],
                    node_tree_manifest_sidecar_count=summary_payload[
                        "node_tree_manifest_sidecar_count"
                    ],
                    node_tree_manifest_sidecar_complete_count=summary_payload[
                        "node_tree_manifest_sidecar_complete_count"
                    ],
                    node_tree_manifest_sidecar_incomplete_count=summary_payload[
                        "node_tree_manifest_sidecar_incomplete_count"
                    ],
                    node_tree_manifest_sidecar_valid_count=summary_payload[
                        "node_tree_manifest_sidecar_valid_count"
                    ],
                    node_tree_manifest_sidecar_invalid_count=summary_payload[
                        "node_tree_manifest_sidecar_invalid_count"
                    ],
                    node_tree_manifest_sidecar_validation_error_count=summary_payload[
                        "node_tree_manifest_sidecar_validation_error_count"
                    ],
                    node_tree_manifest_sidecar_path_incomplete_count=summary_payload[
                        "node_tree_manifest_sidecar_path_incomplete_count"
                    ],
                    node_tree_manifest_sidecar_path_map_mismatch_count=summary_payload[
                        "node_tree_manifest_sidecar_path_map_mismatch_count"
                    ],
                    node_tree_manifest_sidecar_path_map_mismatch_kind_counts=summary_payload[
                        "node_tree_manifest_sidecar_path_map_mismatch_kind_counts"
                    ],
                    node_tree_manifest_sidecar_parts_planned_count=summary_payload[
                        "node_tree_manifest_sidecar_parts_planned_count"
                    ],
                    node_tree_manifest_sidecar_joints_planned_count=summary_payload[
                        "node_tree_manifest_sidecar_joints_planned_count"
                    ],
                    node_tree_manifest_sidecar_part_path_count=summary_payload[
                        "node_tree_manifest_sidecar_part_path_count"
                    ],
                    node_tree_manifest_sidecar_joint_path_count=summary_payload[
                        "node_tree_manifest_sidecar_joint_path_count"
                    ],
                    excluded_output_artifact_inputs=summary_payload[
                        "excluded_output_artifact_inputs"
                    ],
                    show_passed_counts=_show_passed_counts(args),
                    show_metadata=args.show_metadata or args.show_diagnostics,
                    show_reason_codes=(
                        args.show_reason_codes or args.show_diagnostics
                    ),
                    show_inputs=args.show_inputs or args.show_diagnostics,
                    show_skipped_reasons=(
                        args.show_skipped_reasons or args.show_diagnostics
                    ),
                    show_full_mechanical_gate_counts=(
                        args.fail_on_full_mechanical_gate_false
                        or args.fail_on_full_mechanical_gate_unknown
                    ),
                    show_smoke_report_counts=(
                        args.fail_on_smoke_report_missing
                        or args.fail_on_smoke_report_read_error
                    ),
                    show_control_readback_counts=(
                        args.fail_on_control_readback_missing
                    ),
                    require_passed=args.require_passed,
                    require_full_mechanical_restoration_gate=(
                        args.require_full_mechanical_restoration_gate
                    ),
                    fail_on_full_mechanical_gate_false=(
                        args.fail_on_full_mechanical_gate_false
                    ),
                    fail_on_full_mechanical_gate_unknown=(
                        args.fail_on_full_mechanical_gate_unknown
                    ),
                    fail_on_smoke_report_missing=(
                        args.fail_on_smoke_report_missing
                    ),
                    fail_on_smoke_report_read_error=(
                        args.fail_on_smoke_report_read_error
                    ),
                    fail_on_control_readback_missing=(
                        args.fail_on_control_readback_missing
                    ),
                    fail_on_node_tree_manifest_sidecar_incomplete=(
                        args.fail_on_node_tree_manifest_sidecar_incomplete
                    ),
                    fail_on_invalid_node_tree_manifest_sidecar=(
                        args.fail_on_invalid_node_tree_manifest_sidecar
                    ),
                    fail_on_node_tree_manifest_sidecar_validation_error=(
                        args.fail_on_node_tree_manifest_sidecar_validation_error
                    ),
                    fail_on_node_tree_manifest_sidecar_path_incomplete=(
                        args.fail_on_node_tree_manifest_sidecar_path_incomplete
                    ),
                    fail_on_node_tree_manifest_sidecar_path_map_mismatch=(
                        args.fail_on_node_tree_manifest_sidecar_path_map_mismatch
                    ),
                )
            )
        if not args.summary_only:
            lines.extend(_format_text_result(item) for item in results)
        print("\n".join(lines))
    else:
        json_payload = summary_payload if args.summary_only else result
        print(json.dumps(json_payload, indent=2, ensure_ascii=False))
    return 1 if failed or aggregate_errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
