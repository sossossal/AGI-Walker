#!/usr/bin/env python
"""Confirm customer-specific external bindings after real customer systems are filled in."""

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
    EXTENSION_EXTERNAL_BINDING_SECTION_IDS,
    build_extension_execution_actuals_command,
    default_customer_external_bindings_config_path,
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


def _now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _section_ready(section_id: str, payload: dict[str, Any]) -> bool:
    if section_id == "approval_identity":
        return any(_optional_text(payload.get(field)) for field in ("source_path", "reference"))
    return _optional_text(payload.get("binding_reference_base")) is not None


def _parse_set_argument(raw: str) -> tuple[str, str, str]:
    if "=" not in raw or "." not in raw:
        raise ValueError(
            "set overrides must use section.field=value syntax"
        )
    left, value = raw.split("=", 1)
    section, field = left.split(".", 1)
    section = section.strip()
    field = field.strip()
    if section not in EXTENSION_EXTERNAL_BINDING_SECTION_IDS:
        raise ValueError(f"unknown section in set override: {section}")
    if not field:
        raise ValueError("set override field must not be empty")
    return section, field, value


def _load_overrides_file(
    path: str | Path,
    project_root: Path,
) -> tuple[Path, dict[str, dict[str, str]]]:
    resolved_path = _resolve_project_path(path, project_root)
    try:
        payload = json.loads(resolved_path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ValueError(f"overrides file is missing: {resolved_path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"overrides file is not valid JSON: {resolved_path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("overrides file must be a JSON object")

    overrides: dict[str, dict[str, str]] = {}
    for raw_section, raw_fields in payload.items():
        if not isinstance(raw_section, str) or not raw_section.strip():
            raise ValueError("overrides file sections must be non-empty strings")
        section = raw_section.strip()
        if section not in EXTENSION_EXTERNAL_BINDING_SECTION_IDS:
            raise ValueError(f"unknown section in overrides file: {section}")
        if not isinstance(raw_fields, dict):
            raise ValueError(
                f"overrides file section {section!r} must be a JSON object"
            )
        section_overrides: dict[str, str] = {}
        for raw_field, raw_value in raw_fields.items():
            if not isinstance(raw_field, str) or not raw_field.strip():
                raise ValueError(
                    f"overrides file fields in section {section!r} must be non-empty strings"
                )
            if isinstance(raw_value, (dict, list)):
                raise ValueError(
                    f"overrides file field {section}.{raw_field} must be a scalar value"
                )
            section_overrides[raw_field.strip()] = "" if raw_value is None else str(raw_value)
        if section_overrides:
            overrides[section] = section_overrides
    return resolved_path, overrides


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Confirm customer-specific external bindings after real customer metadata is filled in."
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to resolve config paths.",
    )
    parser.add_argument(
        "--config",
        default=default_customer_external_bindings_config_path(),
        help="Customer-specific external bindings config to confirm.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Optional output path. Defaults to overwriting --config.",
    )
    parser.add_argument(
        "--section",
        action="append",
        choices=list(EXTENSION_EXTERNAL_BINDING_SECTION_IDS),
        default=None,
        help="Binding section to confirm. Defaults to confirming every section.",
    )
    parser.add_argument(
        "--confirmed-by",
        required=True,
        help="Actor confirming that the section now points to a real customer-owned system.",
    )
    parser.add_argument(
        "--confirmed-at",
        default=None,
        help="Confirmation timestamp. Defaults to the current UTC time.",
    )
    parser.add_argument(
        "--confirmation-ticket",
        required=True,
        help="Change ticket / approval ticket used to attest the confirmation.",
    )
    parser.add_argument("--confirmation-notes", default=None)
    parser.add_argument("--confirmation-evidence", default=None)
    parser.add_argument(
        "--overrides-file",
        default=None,
        help="Optional JSON file containing section -> field -> value overrides applied before confirmation.",
    )
    parser.add_argument(
        "--set",
        action="append",
        default=None,
        help="Optional section.field=value override applied before confirming the section.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    project_root = Path(args.project_root)
    config_path = _resolve_project_path(args.config, project_root)
    output_path = _resolve_project_path(args.output, project_root) if args.output else config_path

    if not config_path.is_file():
        parser.error(f"external bindings config is missing: {config_path}")

    try:
        payload = json.loads(config_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        parser.error(f"external bindings config is not valid JSON: {config_path}: {exc}")
    if not isinstance(payload, dict):
        parser.error("external bindings config must be a JSON object")

    selected_sections = list(dict.fromkeys(args.section or list(EXTENSION_EXTERNAL_BINDING_SECTION_IDS)))
    overrides: dict[str, dict[str, str]] = {}
    resolved_overrides_path: Path | None = None
    if args.overrides_file:
        try:
            resolved_overrides_path, file_overrides = _load_overrides_file(
                args.overrides_file,
                project_root,
            )
        except ValueError as exc:
            parser.error(str(exc))
        for section, fields in file_overrides.items():
            overrides.setdefault(section, {}).update(fields)
    for raw in args.set or []:
        try:
            section, field, value = _parse_set_argument(raw)
        except ValueError as exc:
            parser.error(str(exc))
        overrides.setdefault(section, {})[field] = value

    confirmed_at = args.confirmed_at or _now_iso()
    for section in selected_sections:
        section_payload = payload.get(section)
        if not isinstance(section_payload, dict):
            parser.error(f"section {section!r} is missing from config")
        updated_section = dict(section_payload)
        for field, value in overrides.get(section, {}).items():
            updated_section[field] = value
        if not _section_ready(section, updated_section):
            parser.error(
                f"section {section!r} is missing required ready fields after overrides"
            )
        updated_section["binding_state"] = "confirmed"
        updated_section["confirmed_by"] = args.confirmed_by
        updated_section["confirmed_at"] = confirmed_at
        updated_section["confirmation_ticket"] = args.confirmation_ticket
        if _optional_text(args.confirmation_notes):
            updated_section["confirmation_notes"] = args.confirmation_notes.strip()
        if _optional_text(args.confirmation_evidence):
            updated_section["confirmation_evidence"] = args.confirmation_evidence.strip()
        payload[section] = updated_section

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    rebuild_actuals_command = build_extension_execution_actuals_command(
        external_bindings_config_path=args.output or args.config,
    )
    print(f"customer_external_bindings_confirmed={output_path}")
    print(
        "customer_external_bindings_confirmed_sections="
        + ",".join(selected_sections)
    )
    print(f"customer_external_bindings_confirmed_by={args.confirmed_by}")
    print(f"customer_external_bindings_confirmation_ticket={args.confirmation_ticket}")
    if resolved_overrides_path is not None:
        print(f"customer_external_bindings_overrides_file={resolved_overrides_path}")
    print(
        "customer_external_bindings_rebuild_actuals_command="
        f"{rebuild_actuals_command}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
