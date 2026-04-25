#!/usr/bin/env python
"""Build or refresh the managed external-mainline input file."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Mapping, Sequence
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
    EXTENSION_EXTERNAL_BINDING_SECTION_IDS,
    build_run_external_mainline_execution_plan_command,
    default_canonical_industrial_delivery_rehearsal_report_path,
    default_customer_external_bindings_closure_report_path,
    default_customer_external_bindings_config_path,
    default_external_mainline_inputs_path,
    default_vulnerability_exception_review_report_path,
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


def _load_json_object(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"JSON document must be an object: {path}")
    return dict(payload)


def _coerce_string_list(value: Any) -> list[str]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        return []
    return [str(item).strip() for item in value if _optional_text(item)]


def _existing_value(payload: Mapping[str, Any], *keys: str) -> Any:
    current: Any = payload
    for key in keys:
        if not isinstance(current, Mapping):
            return None
        current = current.get(key)
    return current


def _customer_confirmation_defaults(
    customer_config_payload: Mapping[str, Any] | None,
) -> tuple[str | None, str | None, str | None]:
    if not isinstance(customer_config_payload, Mapping):
        return None, None, None
    confirmed_sections: list[Mapping[str, Any]] = []
    for section_id in EXTENSION_EXTERNAL_BINDING_SECTION_IDS:
        section_payload = customer_config_payload.get(section_id)
        if not isinstance(section_payload, Mapping):
            continue
        if _optional_text(section_payload.get("binding_state")) != "confirmed":
            continue
        confirmed_sections.append(section_payload)
    if not confirmed_sections:
        return None, None, None

    def _consistent(field: str) -> str | None:
        values = {
            value
            for value in (
                _optional_text(item.get(field))
                for item in confirmed_sections
            )
            if value is not None
        }
        if len(values) == 1:
            return next(iter(values))
        return None

    return (
        _consistent("confirmed_by"),
        _consistent("confirmed_at"),
        _consistent("confirmation_ticket"),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build or refresh deployment/external_mainline.inputs.json."
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to resolve artifact paths.",
    )
    parser.add_argument(
        "--output",
        default=default_external_mainline_inputs_path(),
        help="Output path for the managed external-mainline inputs JSON.",
    )
    parser.add_argument(
        "--customer-config",
        default=default_customer_external_bindings_config_path(),
        help="Customer-specific external bindings config used to backfill confirmation metadata when available.",
    )
    parser.add_argument(
        "--customer-overrides-file",
        default="deployment/customer_delivery.external_bindings.customer.overrides.json",
        help="Customer overrides file path stored into the managed inputs JSON.",
    )
    parser.add_argument(
        "--customer-external-bindings-closure-report",
        default=default_customer_external_bindings_closure_report_path(),
        help="Optional closure report used to backfill selected sections.",
    )
    parser.add_argument(
        "--vulnerability-exception-review-report",
        default=default_vulnerability_exception_review_report_path(),
        help="Vulnerability exception review report path stored into the managed inputs JSON.",
    )
    parser.add_argument(
        "--industrial-delivery-rehearsal-report",
        default=default_canonical_industrial_delivery_rehearsal_report_path(),
        help="Industrial rehearsal report used to backfill version / output-root hints.",
    )
    parser.add_argument(
        "--confirmed-by-placeholder",
        default="<confirmed-by>",
        help="Fallback placeholder when the customer config does not already carry confirmed_by.",
    )
    parser.add_argument(
        "--confirmation-ticket-placeholder",
        default="<confirmation-ticket>",
        help="Fallback placeholder when the customer config does not already carry confirmation_ticket.",
    )
    parser.add_argument(
        "--confirmation-notes",
        default=(
            "Replace placeholders with the real customer confirmation metadata "
            "before running the external mainline runner."
        ),
        help="Default confirmation notes written when the existing output does not already set them.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    project_root = Path(args.project_root).resolve()
    output_path = _resolve_project_path(args.output, project_root)
    customer_config_path = _resolve_project_path(args.customer_config, project_root)
    customer_overrides_path = _resolve_project_path(args.customer_overrides_file, project_root)
    closure_report_path = _resolve_project_path(
        args.customer_external_bindings_closure_report,
        project_root,
    )
    industrial_report_path = _resolve_project_path(
        args.industrial_delivery_rehearsal_report,
        project_root,
    )

    existing_payload: dict[str, Any] = {}
    if output_path.is_file():
        try:
            existing_payload = _load_json_object(output_path)
        except Exception as exc:
            parser.error(f"existing output is not valid JSON: {output_path}: {exc}")

    customer_config_payload: dict[str, Any] | None = None
    if customer_config_path.is_file():
        try:
            customer_config_payload = _load_json_object(customer_config_path)
        except Exception as exc:
            parser.error(f"customer config is not valid JSON: {customer_config_path}: {exc}")

    confirmed_by_from_config, confirmed_at_from_config, confirmation_ticket_from_config = (
        _customer_confirmation_defaults(customer_config_payload)
    )

    selected_sections = list(EXTENSION_EXTERNAL_BINDING_SECTION_IDS)
    if closure_report_path.is_file():
        try:
            closure_payload = _load_json_object(closure_report_path)
        except Exception:
            closure_payload = {}
        selected_sections = _coerce_string_list(
            _existing_value(closure_payload, "metrics", "selected_sections")
        ) or selected_sections

    industrial_version = None
    industrial_output_root = None
    if industrial_report_path.is_file():
        try:
            industrial_payload = _load_json_object(industrial_report_path)
        except Exception:
            industrial_payload = {}
        industrial_version = _optional_text(industrial_payload.get("version"))
        industrial_output_root = _optional_text(industrial_report_path.parent.as_posix())
    if industrial_output_root is None:
        industrial_output_root = "test_env/release_rehearsal_industrial"

    existing_customer = (
        dict(existing_payload.get("customer_external_bindings"))
        if isinstance(existing_payload.get("customer_external_bindings"), Mapping)
        else {}
    )
    existing_review = (
        dict(existing_payload.get("vulnerability_exception_review"))
        if isinstance(existing_payload.get("vulnerability_exception_review"), Mapping)
        else {}
    )
    existing_industrial = (
        dict(existing_payload.get("industrial_rehearsal"))
        if isinstance(existing_payload.get("industrial_rehearsal"), Mapping)
        else {}
    )
    existing_industrial_live = (
        dict(existing_payload.get("industrial_live_evidence"))
        if isinstance(existing_payload.get("industrial_live_evidence"), Mapping)
        else {}
    )

    payload = {
        "customer_external_bindings": {
            "enabled": existing_customer.get("enabled")
            if isinstance(existing_customer.get("enabled"), bool)
            else True,
            "config": _optional_text(existing_customer.get("config")) or args.customer_config,
            "overrides_file": _optional_text(existing_customer.get("overrides_file"))
            or args.customer_overrides_file,
            "sections": _coerce_string_list(existing_customer.get("sections"))
            or selected_sections,
            "confirmed_by": _optional_text(existing_customer.get("confirmed_by"))
            or confirmed_by_from_config
            or args.confirmed_by_placeholder,
            "confirmed_at": _optional_text(existing_customer.get("confirmed_at"))
            or confirmed_at_from_config,
            "confirmation_ticket": _optional_text(existing_customer.get("confirmation_ticket"))
            or confirmation_ticket_from_config
            or args.confirmation_ticket_placeholder,
            "confirmation_notes": _optional_text(existing_customer.get("confirmation_notes"))
            or args.confirmation_notes,
            "confirmation_evidence": _optional_text(existing_customer.get("confirmation_evidence")),
            "skip_collect_release_evidence": existing_customer.get("skip_collect_release_evidence")
            if isinstance(existing_customer.get("skip_collect_release_evidence"), bool)
            else False,
        },
        "vulnerability_exception_review": {
            "enabled": existing_review.get("enabled")
            if isinstance(existing_review.get("enabled"), bool)
            else True,
            "report_output": _optional_text(existing_review.get("report_output"))
            or args.vulnerability_exception_review_report,
        },
        "industrial_rehearsal": {
            "refresh": existing_industrial.get("refresh")
            if isinstance(existing_industrial.get("refresh"), bool)
            else False,
            "version": _optional_text(existing_industrial.get("version"))
            or industrial_version
            or "2026.04.17-industrial-rehearsal",
            "build_id": _optional_text(existing_industrial.get("build_id"))
            or "release-rehearsal-industrial",
            "output_root": _optional_text(existing_industrial.get("output_root"))
            or industrial_output_root,
            "report_path": _optional_text(existing_industrial.get("report_path"))
            or args.industrial_delivery_rehearsal_report,
        },
        "industrial_live_evidence": {
            "enabled": existing_industrial_live.get("enabled")
            if isinstance(existing_industrial_live.get("enabled"), bool)
            else True,
            "target_environment": _optional_text(
                existing_industrial_live.get("target_environment")
            )
            or "<target-environment>",
            "access_method": _optional_text(
                existing_industrial_live.get("access_method")
            )
            or "<access-method>",
            "install_entrypoint": _optional_text(
                existing_industrial_live.get("install_entrypoint")
            )
            or "<install-command-or-runbook>",
            "upgrade_entrypoint": _optional_text(
                existing_industrial_live.get("upgrade_entrypoint")
            )
            or "<upgrade-command-or-runbook>",
            "rollback_entrypoint": _optional_text(
                existing_industrial_live.get("rollback_entrypoint")
            )
            or "<rollback-command-or-runbook>",
            "backup_restore_entrypoint": _optional_text(
                existing_industrial_live.get("backup_restore_entrypoint")
            )
            or "<backup-restore-command-or-runbook>",
            "closure_archive_root": _optional_text(
                existing_industrial_live.get("closure_archive_root")
            )
            or "<closure-archive-root>",
            "evidence_output_root": _optional_text(
                existing_industrial_live.get("evidence_output_root")
            )
            or "test_env/industrial_live_evidence",
            "notes": _optional_text(existing_industrial_live.get("notes"))
            or (
                "Replace placeholders with the real customer environment access, "
                "execution entrypoints, and archive location before executing "
                "industrial live evidence."
            ),
        },
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    run_command = build_run_external_mainline_execution_plan_command(
        inputs_file=args.output
    )
    print(f"external_mainline_inputs_written={output_path}")
    print(f"external_mainline_inputs_customer_config_exists={str(customer_config_path.is_file()).lower()}")
    print(f"external_mainline_inputs_customer_overrides_exists={str(customer_overrides_path.is_file()).lower()}")
    print(f"external_mainline_inputs_industrial_rehearsal_exists={str(industrial_report_path.is_file()).lower()}")
    print(
        "external_mainline_inputs_industrial_live_evidence_target="
        f"{payload['industrial_live_evidence']['target_environment']}"
    )
    print(
        "external_mainline_inputs_selected_sections="
        + ",".join(payload["customer_external_bindings"]["sections"])
    )
    print(
        "external_mainline_inputs_run_command="
        f"{run_command}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
