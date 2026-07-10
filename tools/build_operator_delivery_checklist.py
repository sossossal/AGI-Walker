from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence


DEFAULT_TEMPLATE = "deployment/operator_delivery_checklist.template.json"
DEFAULT_OUTPUT = "test_env/operator_delivery/operator_delivery_checklist.json"
SCHEMA_VERSION = "1.0"


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for candidate in current.parents:
        if (candidate / "pyproject.toml").exists() and (candidate / "agi_walker").exists():
            return candidate
    return current.parent


PROJECT_ROOT = _find_repo_root()


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build an operator delivery checklist from managed evidence files."
    )
    parser.add_argument("--template", default=DEFAULT_TEMPLATE)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--set-evidence",
        action="append",
        default=[],
        metavar="ITEM_ID=PATH",
        help="Override an evidence_path from the template.",
    )
    return parser.parse_args(argv)


def _load_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _resolve_evidence_path(path: str | Path) -> Path | None:
    candidate = Path(path)
    if not str(path).strip() or candidate.is_absolute() or ".." in candidate.parts:
        return None
    return PROJECT_ROOT / candidate


def _load_json_if_exists(path: str | Path) -> dict[str, Any] | None:
    json_path = _resolve_evidence_path(path)
    if json_path is None:
        return None
    if not json_path.exists():
        return None
    return _load_json(json_path)


def _value_at_path(payload: dict[str, Any], dotted_path: str) -> Any:
    current: Any = payload
    for key in dotted_path.split("."):
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


def _evidence_overrides(values: Sequence[str]) -> dict[str, str]:
    overrides: dict[str, str] = {}
    for value in values:
        if "=" not in value:
            raise ValueError(f"Invalid --set-evidence value: {value!r}")
        item_id, path = value.split("=", 1)
        item_id = item_id.strip()
        path = path.strip()
        if not item_id or not path:
            raise ValueError(f"Invalid --set-evidence value: {value!r}")
        overrides[item_id] = path
    return overrides


def _item_required(item: dict[str, Any]) -> bool:
    return item.get("required") is not False


def _build_item_result(
    item: dict[str, Any],
    *,
    evidence_path: str,
    evidence_path_valid: bool,
    evidence: dict[str, Any] | None,
) -> dict[str, Any]:
    required = _item_required(item)
    expected_statuses = item.get("expected_statuses") or []
    status_path = item.get("status_path", "status")
    actual_status = None if evidence is None else _value_at_path(evidence, status_path)

    if not evidence_path_valid:
        status = "blocked" if required else "warning"
        reason = "evidence_path_invalid"
    elif evidence is None:
        status = "blocked" if required else "warning"
        reason = "evidence_missing"
    elif expected_statuses and actual_status not in expected_statuses:
        status = "blocked" if required else "warning"
        reason = "unexpected_status"
    else:
        status = "ready"
        reason = "ok"

    return {
        "id": item["id"],
        "category": item.get("category"),
        "owner": item.get("owner"),
        "required": required,
        "status": status,
        "reason": reason,
        "evidence_path": evidence_path,
        "evidence_path_valid": evidence_path_valid,
        "status_path": status_path,
        "actual_status": actual_status,
        "expected_statuses": expected_statuses,
        "description": item.get("description", ""),
    }


def build_operator_delivery_checklist(
    *,
    template: dict[str, Any],
    template_path: str,
    evidence_overrides: dict[str, str] | None = None,
) -> dict[str, Any]:
    overrides = evidence_overrides or {}
    items = template.get("required_items") or []
    results = []
    for item in items:
        evidence_path = overrides.get(item["id"], item.get("evidence_path", ""))
        evidence_path_valid = _resolve_evidence_path(evidence_path) is not None
        results.append(
            _build_item_result(
                item,
                evidence_path=evidence_path,
                evidence_path_valid=evidence_path_valid,
                evidence=_load_json_if_exists(evidence_path),
            )
        )

    blockers = [
        result["id"]
        for result in results
        if result["required"] and result["status"] == "blocked"
    ]
    warnings = [
        result["id"]
        for result in results
        if not result["required"] and result["status"] == "warning"
    ]
    status = "ready" if not blockers else "blocked"
    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "checklist_id": template.get("checklist_id"),
        "template": template_path,
        "delivery_window": template.get("delivery_window", {}),
        "summary": {
            "ready_item_count": sum(1 for result in results if result["status"] == "ready"),
            "blocked_item_count": len(blockers),
            "warning_item_count": len(warnings),
            "required_item_count": sum(1 for result in results if result["required"]),
        },
        "blockers": blockers,
        "warnings": warnings,
        "items": results,
        "next_actions": _next_actions(status=status, blockers=blockers, warnings=warnings),
    }


def _next_actions(
    *, status: str, blockers: list[str], warnings: list[str]
) -> list[str]:
    if status == "blocked":
        return [f"Resolve blocked operator delivery items: {', '.join(blockers)}."]
    if warnings:
        return [
            "Operator delivery checklist is ready; review optional warnings before final archive."
        ]
    return ["Operator delivery checklist is ready for archive."]


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    checklist = build_operator_delivery_checklist(
        template=_load_json(args.template),
        template_path=args.template,
        evidence_overrides=_evidence_overrides(args.set_evidence),
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
