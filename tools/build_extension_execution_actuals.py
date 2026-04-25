#!/usr/bin/env python
"""Build executed extension window records and the actuals summary artifact."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for candidate in current.parents:
        if (candidate / "pyproject.toml").exists() and (candidate / "agi_walker").exists():
            return candidate
    return current.parent


def _configure_stdio() -> None:
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")


_configure_stdio()
PROJECT_ROOT = _find_repo_root()
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.core.api.release_contracts import (  # noqa: E402
    build_extension_execution_actuals_artifact,
    default_extension_execution_actuals_artifact_path,
    default_extension_execution_schedule_artifact_path,
    write_extension_execution_actuals_artifact,
)


def _resolve_project_path(path: str | Path, project_root: Path) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return project_root / candidate


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def _optional_text(value: Any) -> str | None:
    if not isinstance(value, str):
        return None
    normalized = value.strip()
    return normalized or None


def _contains_placeholder_token(value: Any) -> bool:
    """Return whether a string still contains placeholder-style tokens."""
    normalized = _optional_text(value)
    if normalized is None:
        return False
    lowered = normalized.lower()
    if lowered in {"placeholder", "<placeholder>", "tbd", "<tbd>"}:
        return True
    start = normalized.find("<")
    end = normalized.find(">", start + 1) if start != -1 else -1
    return start != -1 and end != -1 and end > start


def _confirmed_section_missing_fields(section: dict[str, Any]) -> list[str]:
    return [
        field
        for field in ("confirmed_by", "confirmed_at", "confirmation_ticket")
        if _optional_text(section.get(field)) is None
    ]


def _external_binding_confirmation(binding: dict[str, Any]) -> dict[str, str] | None:
    payload: dict[str, str] = {}
    for field in (
        "binding_state",
        "confirmed_by",
        "confirmed_at",
        "confirmation_ticket",
        "confirmation_notes",
        "confirmation_evidence",
    ):
        value = _optional_text(binding.get(field))
        if value is not None:
            payload[field] = value
    return payload or None


def _normalize_json_path(path: str | Path) -> str:
    return Path(path).as_posix()


def _load_external_bindings_config(
    config_path: str | Path,
    project_root: Path,
) -> tuple[dict[str, Any], dict[str, str | None]]:
    resolved_path = _resolve_project_path(config_path, project_root)
    try:
        payload = json.loads(resolved_path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ValueError(f"external bindings config not found: {resolved_path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(
            f"external bindings config is not valid JSON: {resolved_path}: {exc}"
        ) from exc

    if not isinstance(payload, dict):
        raise ValueError("external bindings config must be a JSON object")

    sections: dict[str, dict[str, Any]] = {}
    extracted: dict[str, str | None] = {
        "approval_identity_source_path": None,
        "approval_identity_source_type": None,
        "approval_identity_reference": None,
        "archive_target_binding_type": None,
        "archive_target_binding_reference_base": None,
        "due_trigger_binding_type": None,
        "due_trigger_binding_reference_base": None,
        "due_trigger_checked_at": None,
    }
    field_map = {
        "approval_identity": {
            "source_path": "approval_identity_source_path",
            "source_type": "approval_identity_source_type",
            "reference": "approval_identity_reference",
        },
        "archive_target": {
            "binding_type": "archive_target_binding_type",
            "binding_reference_base": "archive_target_binding_reference_base",
        },
        "due_trigger": {
            "binding_type": "due_trigger_binding_type",
            "binding_reference_base": "due_trigger_binding_reference_base",
            "checked_at": "due_trigger_checked_at",
        },
    }

    for section_name in ("approval_identity", "archive_target", "due_trigger"):
        section = payload.get(section_name)
        if section is None:
            continue
        if not isinstance(section, dict):
            raise ValueError(
                f"external bindings config section '{section_name}' must be an object"
            )
        binding_state = _optional_text(section.get("binding_state"))
        if binding_state == "confirmed":
            missing_fields = _confirmed_section_missing_fields(section)
            if missing_fields:
                raise ValueError(
                    "external bindings config section "
                    f"'{section_name}' is marked confirmed but is missing "
                    + ", ".join(missing_fields)
                )
        for source_key, target_key in field_map[section_name].items():
            extracted[target_key] = _optional_text(section.get(source_key))
        sections[section_name] = {
            str(key): value
            for key, value in section.items()
            if isinstance(key, str) and key.strip() and value is not None
        }

    return (
        {
            "config_path": _normalize_json_path(config_path),
            **sections,
        },
        extracted,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build executed extension window records and the actuals summary artifact."
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to resolve artifact paths.",
    )
    parser.add_argument(
        "--output",
        default=default_extension_execution_actuals_artifact_path(),
        help="Output path for the generated extension execution actuals artifact.",
    )
    parser.add_argument(
        "--schedule-artifact",
        default=default_extension_execution_schedule_artifact_path(),
        help="Schedule artifact used to materialize executed record files.",
    )
    parser.add_argument(
        "--external-bindings-config",
        default=None,
        help="Optional JSON file that maps approval/archive/due-trigger bindings to customer-owned external systems.",
    )
    parser.add_argument("--window-trigger-recorded-at", default=None)
    parser.add_argument("--window-trigger-recorded-by", default=None)
    parser.add_argument("--signoff-recorded-at", default=None)
    parser.add_argument("--signoff-recorded-by", default=None)
    parser.add_argument("--residual-risk-reviewed-at", default=None)
    parser.add_argument("--residual-risk-reviewed-by", default=None)
    parser.add_argument("--closure-archived-at", default=None)
    parser.add_argument("--closure-archived-by", default=None)
    parser.add_argument("--approval-identity-source-path", default=None)
    parser.add_argument("--approval-identity-source-type", default=None)
    parser.add_argument("--approval-identity-reference", default=None)
    parser.add_argument("--archive-target-binding-type", default=None)
    parser.add_argument("--archive-target-binding-reference-base", default=None)
    parser.add_argument("--exception-review-due-at", default=None)
    parser.add_argument("--closure-archive-due-at", default=None)
    parser.add_argument("--due-trigger-binding-type", default=None)
    parser.add_argument("--due-trigger-binding-reference-base", default=None)
    parser.add_argument("--due-trigger-checked-at", default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    project_root = Path(args.project_root)
    output_path = _resolve_project_path(args.output, project_root)
    external_bindings: dict[str, Any] | None = None
    external_binding_values: dict[str, str | None] = {}
    if args.external_bindings_config:
        try:
            external_bindings, external_binding_values = _load_external_bindings_config(
                args.external_bindings_config,
                project_root,
            )
        except ValueError as exc:
            parser.error(str(exc))

    payload = build_extension_execution_actuals_artifact(
        project_root=project_root,
        artifact_path=args.output,
        schedule_artifact_path=args.schedule_artifact,
        window_trigger_recorded_at=args.window_trigger_recorded_at,
        window_trigger_recorded_by=args.window_trigger_recorded_by,
        signoff_recorded_at=args.signoff_recorded_at,
        signoff_recorded_by=args.signoff_recorded_by,
        residual_risk_reviewed_at=args.residual_risk_reviewed_at,
        residual_risk_reviewed_by=args.residual_risk_reviewed_by,
        closure_archived_at=args.closure_archived_at,
        closure_archived_by=args.closure_archived_by,
        approval_identity_source_path=(
            args.approval_identity_source_path
            or external_binding_values.get("approval_identity_source_path")
        ),
        approval_identity_source_type=(
            args.approval_identity_source_type
            or external_binding_values.get("approval_identity_source_type")
        ),
        approval_identity_reference=(
            args.approval_identity_reference
            or external_binding_values.get("approval_identity_reference")
        ),
        archive_target_binding_type=(
            args.archive_target_binding_type
            or external_binding_values.get("archive_target_binding_type")
        ),
        archive_target_binding_reference_base=(
            args.archive_target_binding_reference_base
            or external_binding_values.get("archive_target_binding_reference_base")
        ),
        exception_review_due_at=args.exception_review_due_at,
        closure_archive_due_at=args.closure_archive_due_at,
        due_trigger_binding_type=(
            args.due_trigger_binding_type
            or external_binding_values.get("due_trigger_binding_type")
        ),
        due_trigger_binding_reference_base=(
            args.due_trigger_binding_reference_base
            or external_binding_values.get("due_trigger_binding_reference_base")
        ),
        due_trigger_checked_at=(
            args.due_trigger_checked_at
            or external_binding_values.get("due_trigger_checked_at")
        ),
        external_bindings=external_bindings,
    )

    payload_external_bindings = payload.get("external_bindings")
    external_binding_config_path = None
    approval_external_binding: dict[str, Any] = {}
    archive_external_binding: dict[str, Any] = {}
    due_trigger_external_binding: dict[str, Any] = {}
    if isinstance(payload_external_bindings, dict):
        external_binding_config_path = payload_external_bindings.get("config_path")
        approval_external_binding = dict(
            payload_external_bindings.get("approval_identity", {})
        ) if isinstance(payload_external_bindings.get("approval_identity"), dict) else {}
        archive_external_binding = dict(
            payload_external_bindings.get("archive_target", {})
        ) if isinstance(payload_external_bindings.get("archive_target"), dict) else {}
        due_trigger_external_binding = dict(
            payload_external_bindings.get("due_trigger", {})
        ) if isinstance(payload_external_bindings.get("due_trigger"), dict) else {}
    approval_binding_confirmation = _external_binding_confirmation(approval_external_binding)
    archive_binding_confirmation = _external_binding_confirmation(archive_external_binding)
    due_trigger_binding_confirmation = _external_binding_confirmation(due_trigger_external_binding)

    approval_identity_source_payload = {
        "schema_version": "1.0",
        "artifact_type": "extension_approval_identity_source",
        "engagement_id": payload["engagement_id"],
        "customer_name": payload["customer_name"],
        "window_id": payload["window_id"],
        "source_type": payload["approval_identity_source_type"],
        "identity_reference": payload["approval_identity_reference"],
        "binding_reference": payload["approval_identity_reference"],
        "signoff_recorded_by": payload["signoff_recorded_by"],
        "binding_config_path": external_binding_config_path,
        "external_binding": approval_external_binding,
    }
    if approval_binding_confirmation is not None:
        approval_identity_source_payload["external_binding_confirmation"] = (
            approval_binding_confirmation
        )
    skipped_artifacts: list[str] = []
    approval_identity_source_path_value = _optional_text(
        payload.get("approval_identity_source_path")
    )
    if _contains_placeholder_token(approval_identity_source_path_value):
        skipped_artifacts.append("approval_identity_source")
    elif approval_identity_source_path_value is not None:
        approval_identity_source_path = _resolve_project_path(
            approval_identity_source_path_value,
            project_root,
        )
        _write_json(approval_identity_source_path, approval_identity_source_payload)

    for item in payload.get("profiles", []):
        if not isinstance(item, dict) or item.get("actionable") is not True:
            continue
        profile_id = str(item.get("id") or "")
        window_trigger_path = _resolve_project_path(
            str(item["window_trigger_record_path"]),
            project_root,
        )
        signoff_path = _resolve_project_path(
            str(item["signoff_record_path"]),
            project_root,
        )
        residual_risk_review_path = _resolve_project_path(
            str(item["residual_risk_review_record_path"]),
            project_root,
        )
        exception_review_path = _resolve_project_path(
            str(item["exception_review_record_path"]),
            project_root,
        )
        due_trigger_check_path = _resolve_project_path(
            str(item["due_trigger_check_path"]),
            project_root,
        )
        archive_target_path = _resolve_project_path(
            str(item["archive_target_path"]),
            project_root,
        )
        closure_index_path = _resolve_project_path(
            str(item["closure_index_path"]),
            project_root,
        )
        closure_manifest_path = _resolve_project_path(
            str(item["closure_manifest_path"]),
            project_root,
        )

        _write_json(
            window_trigger_path,
            {
                "schema_version": "1.0",
                "artifact_type": "extension_window_trigger_record",
                "engagement_id": payload["engagement_id"],
                "window_id": payload["window_id"],
                "profile_id": profile_id,
                "recorded_at": payload["window_trigger_recorded_at"],
                "recorded_by": payload["window_trigger_recorded_by"],
                "status": "recorded",
            },
        )
        _write_json(
            signoff_path,
            {
                "schema_version": "1.0",
                "artifact_type": "extension_signoff_record",
                "engagement_id": payload["engagement_id"],
                "window_id": payload["window_id"],
                "profile_id": profile_id,
                "recorded_at": payload["signoff_recorded_at"],
                "recorded_by": payload["signoff_recorded_by"],
                "signoff_due_at": item["signoff_due_at"],
                "signoff_owner_role": item["signoff_owner_role"],
                "decision": "accepted",
            },
        )
        _write_json(
            residual_risk_review_path,
            {
                "schema_version": "1.0",
                "artifact_type": "extension_residual_risk_review_record",
                "engagement_id": payload["engagement_id"],
                "window_id": payload["window_id"],
                "profile_id": profile_id,
                "reviewed_at": payload["residual_risk_reviewed_at"],
                "reviewed_by": payload["residual_risk_reviewed_by"],
                "exception_review_due_at": item["exception_review_due_at"],
                "exception_review_owner_role": item["exception_review_owner_role"],
                "result": "accepted_residual_risk_tracked",
                "review_basis": "deployment/security/vulnerability_exceptions.input.json",
            },
        )
        _write_json(
            exception_review_path,
            {
                "schema_version": "1.0",
                "artifact_type": "extension_exception_review_followup_record",
                "engagement_id": payload["engagement_id"],
                "window_id": payload["window_id"],
                "profile_id": profile_id,
                "recorded_at": payload["residual_risk_reviewed_at"],
                "recorded_by": payload["residual_risk_reviewed_by"],
                "review_due_at": item["exception_review_due_at"],
                "owner_role": item["exception_review_owner_role"],
                "status": "scheduled",
                "review_basis": "deployment/security/vulnerability_exceptions.input.json",
            },
        )
        due_trigger_check_payload = {
            "schema_version": "1.0",
            "artifact_type": "extension_due_trigger_check",
            "engagement_id": payload["engagement_id"],
            "window_id": payload["window_id"],
            "profile_id": profile_id,
            "checked_at": payload["due_trigger_checked_at"],
            "exception_review_due_at": item["exception_review_due_at"],
            "closure_archive_due_at": item["closure_archive_due_at"],
            "status": "ready",
            "identity_source_path": item["approval_identity_source_path"],
            "binding_type": payload["due_trigger_binding_type"],
            "binding_reference": item["due_trigger_binding_reference"],
            "binding_config_path": external_binding_config_path,
            "external_binding": due_trigger_external_binding,
        }
        if due_trigger_binding_confirmation is not None:
            due_trigger_check_payload["external_binding_confirmation"] = (
                due_trigger_binding_confirmation
            )
        _write_json(due_trigger_check_path, due_trigger_check_payload)
        archive_target_payload = {
            "schema_version": "1.0",
            "artifact_type": "extension_closure_archive_target",
            "engagement_id": payload["engagement_id"],
            "window_id": payload["window_id"],
            "profile_id": profile_id,
            "archive_root": item["closure_archive_root"],
            "target_path": item["closure_archive_root"],
            "owner_role": item["closure_archive_owner_role"],
            "closure_manifest_path": item["closure_manifest_path"],
            "binding_type": payload["archive_target_binding_type"],
            "binding_reference": item["archive_target_binding_reference"],
            "binding_config_path": external_binding_config_path,
            "external_binding": archive_external_binding,
        }
        if archive_binding_confirmation is not None:
            archive_target_payload["external_binding_confirmation"] = (
                archive_binding_confirmation
            )
        _write_json(archive_target_path, archive_target_payload)
        _write_json(
            closure_manifest_path,
            {
                "schema_version": "1.0",
                "artifact_type": "extension_closure_manifest",
                "engagement_id": payload["engagement_id"],
                "window_id": payload["window_id"],
                "profile_id": profile_id,
                "archived_at": payload["closure_archived_at"],
                "archived_by": payload["closure_archived_by"],
                "closure_archive_due_at": item["closure_archive_due_at"],
                "closure_archive_owner_role": item["closure_archive_owner_role"],
                "closure_archive_root": item["closure_archive_root"],
                "records": [
                    item["window_trigger_record_path"],
                    item["signoff_record_path"],
                    item["exception_review_record_path"],
                    item["residual_risk_review_record_path"],
                ],
            },
        )
        _write_json(
            closure_index_path,
            {
                "schema_version": "1.0",
                "artifact_type": "extension_closure_archive_index",
                "engagement_id": payload["engagement_id"],
                "window_id": payload["window_id"],
                "profile_id": profile_id,
                "indexed_at": payload["closure_archived_at"],
                "indexed_by": payload["closure_archived_by"],
                "closure_archive_due_at": item["closure_archive_due_at"],
                "owner_role": item["closure_archive_owner_role"],
                "closure_manifest_path": item["closure_manifest_path"],
                "archive_root": item["closure_archive_root"],
            },
        )

    written_path = write_extension_execution_actuals_artifact(payload, output_path)

    print(f"extension_execution_actuals_written={written_path}")
    print(f"extension_execution_actuals_status={payload['status']}")
    print(
        "extension_execution_actuals_profiles="
        f"{payload['ready_profiles']}/{payload['actionable_profiles']}"
    )
    print(
        "extension_execution_actuals_external_bindings_status="
        f"{payload['external_bindings_status']}"
    )
    print(
        "extension_execution_actuals_external_bindings_follow_up_required="
        f"{str(payload['external_bindings_follow_up_required']).lower()}"
    )
    print(
        "extension_execution_actuals_window_trigger_recorded_at="
        f"{payload['window_trigger_recorded_at']}"
    )
    print(
        "extension_execution_actuals_signoff_recorded_by="
        f"{payload['signoff_recorded_by']}"
    )
    print(
        "extension_execution_actuals_approval_identity_source_path="
        f"{payload['approval_identity_source_path']}"
    )
    print(
        "extension_execution_actuals_archive_target_binding_type="
        f"{payload['archive_target_binding_type']}"
    )
    print(
        "extension_execution_actuals_exception_review_due_at="
        f"{payload['exception_review_due_at']}"
    )
    print(
        "extension_execution_actuals_due_trigger_binding_type="
        f"{payload['due_trigger_binding_type']}"
    )
    print(
        "extension_execution_actuals_due_trigger_checked_at="
        f"{payload['due_trigger_checked_at']}"
    )
    print(
        "extension_execution_actuals_closure_archived_at="
        f"{payload['closure_archived_at']}"
    )
    if external_binding_config_path:
        print(
            "extension_execution_actuals_external_bindings_config="
            f"{external_binding_config_path}"
        )
    for artifact_name in skipped_artifacts:
        print(f"extension_execution_actuals_skipped_artifact={artifact_name}")
    return 0 if payload["status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
