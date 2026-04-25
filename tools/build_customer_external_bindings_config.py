#!/usr/bin/env python
"""Build a customer-specific external bindings config from the instance artifact."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
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
    build_extension_execution_actuals_command,
    default_customer_external_bindings_config_path,
    default_extension_execution_instance_artifact_path,
    validate_extension_execution_instance,
)


def _resolve_project_path(path: str | Path, project_root: Path) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return project_root / candidate


def _optional_text(value: Any) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str):
        value = str(value)
    normalized = value.strip()
    return normalized or None


def _default_due_trigger_checked_at(instance_payload: dict[str, Any]) -> str:
    return (
        _optional_text(instance_payload.get("window_start_at"))
        or datetime.now(timezone.utc).replace(microsecond=0).isoformat()
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build a customer-specific external bindings config from extension_execution_instance."
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to resolve artifact paths.",
    )
    parser.add_argument(
        "--output",
        default=default_customer_external_bindings_config_path(),
        help="Output path for the generated customer-specific external bindings config.",
    )
    parser.add_argument(
        "--instance-artifact",
        default=default_extension_execution_instance_artifact_path(),
        help="Extension execution instance artifact used to derive customer/window identifiers.",
    )
    parser.add_argument(
        "--approval-operator-id",
        default="customer_operator",
        help="Operator identifier appended to the derived approval reference when --approval-reference is omitted.",
    )
    parser.add_argument(
        "--binding-state",
        default="draft",
        choices=["draft", "confirmed"],
        help="Lifecycle state recorded on each generated external-binding section. Defaults to draft until customer-owned systems are confirmed.",
    )
    parser.add_argument("--approval-source-path", default=None)
    parser.add_argument("--approval-source-type", default="customer_ticket_registry")
    parser.add_argument("--approval-reference", default=None)
    parser.add_argument("--approval-system-name", default=None)
    parser.add_argument("--approval-portal-url", default=None)
    parser.add_argument("--archive-binding-type", default="customer_archive_destination")
    parser.add_argument("--archive-binding-reference-base", default=None)
    parser.add_argument("--archive-system-name", default=None)
    parser.add_argument("--archive-portal-url", default=None)
    parser.add_argument("--due-trigger-binding-type", default="customer_due_trigger_schedule")
    parser.add_argument("--due-trigger-binding-reference-base", default=None)
    parser.add_argument("--due-trigger-checked-at", default=None)
    parser.add_argument("--due-trigger-system-name", default=None)
    parser.add_argument("--due-trigger-portal-url", default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    project_root = Path(args.project_root)
    instance_path = _resolve_project_path(args.instance_artifact, project_root)
    output_path = _resolve_project_path(args.output, project_root)

    if not instance_path.is_file():
        parser.error(f"instance artifact is missing: {instance_path}")

    try:
        instance_payload = json.loads(instance_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        parser.error(f"instance artifact is not valid JSON: {instance_path}: {exc}")

    errors = validate_extension_execution_instance(instance_payload)
    if errors:
        parser.error(
            "instance artifact is invalid: "
            + "; ".join(errors)
        )

    engagement_id = _optional_text(instance_payload.get("engagement_id"))
    customer_name = _optional_text(instance_payload.get("customer_name"))
    window_id = _optional_text(instance_payload.get("window_id"))
    delivery_root = _optional_text(instance_payload.get("delivery_root"))
    if not all([engagement_id, customer_name, window_id, delivery_root]):
        parser.error(
            "instance artifact must include engagement_id, customer_name, window_id, and delivery_root"
        )

    approval_source_path = (
        args.approval_source_path
        or Path(delivery_root).joinpath("approval_identity_source.json").as_posix()
    )
    approval_reference = (
        args.approval_reference
        or f"{engagement_id}/{window_id}/{args.approval_operator_id}"
    )
    archive_binding_reference_base = (
        args.archive_binding_reference_base
        or f"archive://{engagement_id}/windows/{window_id}"
    )
    due_trigger_binding_reference_base = (
        args.due_trigger_binding_reference_base
        or f"schedule://{engagement_id}/windows/{window_id}"
    )
    due_trigger_checked_at = (
        args.due_trigger_checked_at or _default_due_trigger_checked_at(instance_payload)
    )

    payload = {
        "approval_identity": {
            "binding_state": args.binding_state,
            "source_path": approval_source_path,
            "source_type": args.approval_source_type,
            "reference": approval_reference,
            "system_name": (
                args.approval_system_name or f"{customer_name} Approval Registry"
            ),
            "integration_notes": (
                "Replace generated approval metadata with the real customer-owned approval system "
                "before marking this binding confirmed."
            ),
        },
        "archive_target": {
            "binding_state": args.binding_state,
            "binding_type": args.archive_binding_type,
            "binding_reference_base": archive_binding_reference_base,
            "system_name": (
                args.archive_system_name or f"{customer_name} Archive Destination"
            ),
            "integration_notes": (
                "Replace generated archive metadata with the real customer-owned archive "
                "destination before marking this binding confirmed."
            ),
        },
        "due_trigger": {
            "binding_state": args.binding_state,
            "binding_type": args.due_trigger_binding_type,
            "binding_reference_base": due_trigger_binding_reference_base,
            "checked_at": due_trigger_checked_at,
            "system_name": (
                args.due_trigger_system_name or f"{customer_name} Due Trigger Schedule"
            ),
            "integration_notes": (
                "Replace generated due-trigger metadata with the real customer-owned scheduler "
                "before marking this binding confirmed."
            ),
        },
    }

    if _optional_text(args.approval_portal_url):
        payload["approval_identity"]["portal_url"] = args.approval_portal_url.strip()
    if _optional_text(args.archive_portal_url):
        payload["archive_target"]["portal_url"] = args.archive_portal_url.strip()
    if _optional_text(args.due_trigger_portal_url):
        payload["due_trigger"]["portal_url"] = args.due_trigger_portal_url.strip()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    rebuild_actuals_command = build_extension_execution_actuals_command(
        external_bindings_config_path=args.output,
    )
    print(f"customer_external_bindings_config_written={output_path}")
    print(f"customer_external_bindings_config_source_instance={instance_path}")
    print(f"customer_external_bindings_config_approval_reference={approval_reference}")
    print(
        "customer_external_bindings_config_archive_binding_reference_base="
        f"{archive_binding_reference_base}"
    )
    print(
        "customer_external_bindings_config_due_trigger_binding_reference_base="
        f"{due_trigger_binding_reference_base}"
    )
    print(
        "customer_external_bindings_config_rebuild_actuals_command="
        f"{rebuild_actuals_command}"
    )
    print(f"customer_external_bindings_config_binding_state={args.binding_state}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
