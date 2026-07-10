from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence


DEFAULT_INPUT = "deployment/ros2_typed_idl_cutover.template.json"
DEFAULT_OUTPUT = "test_env/ros2_typed_idl_cutover/ros2_typed_idl_cutover_report.json"
SCHEMA_VERSION = "1.0"
PASS_STATUSES = {"passed", "ready"}
PROJECT_ROOT = Path(__file__).resolve().parents[1]
EVIDENCE_PATH_FIELDS = (
    "live_smoke_report",
    "typed_inventory",
    "rollback_plan",
)


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a fail-closed report for ROS2 typed IDL cutover evidence."
    )
    parser.add_argument("--input", default=DEFAULT_INPUT)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--require-evidence-files", action="store_true")
    return parser.parse_args(argv)


def _load_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _load_json_if_exists(path: str | Path | None) -> dict[str, Any] | None:
    if not path:
        return None
    json_path = Path(path)
    if not json_path.exists():
        return None
    return _load_json(json_path)


def _text(value: Any) -> str:
    return value.strip() if isinstance(value, str) else ""


def _is_placeholder(value: Any) -> bool:
    text = _text(value)
    return not text or "<" in text or ">" in text


def _status(payload: dict[str, Any] | None) -> Any:
    if payload is None:
        return None
    return payload.get("status") or payload.get("summary", {}).get("status")


def _append_blocker(blockers: list[str], blocker: str) -> None:
    if blocker not in blockers:
        blockers.append(blocker)


def _resolve_evidence_path(
    value: str,
    *,
    input_path: str,
) -> tuple[bool, Path | None, str | None]:
    path = Path(value)
    if not value:
        return False, None, "empty"
    if path.is_absolute():
        return False, None, "absolute"
    if ".." in path.parts:
        return False, None, "parent_directory"

    input_relative = Path(input_path).resolve().parent / path
    if input_relative.exists():
        return True, input_relative, None
    return True, PROJECT_ROOT / path, None


def _evidence_path_status(
    evidence: dict[str, Any],
    *,
    input_path: str,
) -> dict[str, dict[str, Any]]:
    statuses: dict[str, dict[str, Any]] = {}
    for field in EVIDENCE_PATH_FIELDS:
        value = _text(evidence.get(field))
        valid, resolved_path, error = _resolve_evidence_path(value, input_path=input_path)
        statuses[field] = {
            "path": value,
            "path_valid": valid,
            "path_error": error,
            "resolved_path": str(resolved_path) if resolved_path is not None else None,
            "exists": bool(resolved_path and resolved_path.exists()),
        }
    return statuses


def _inventory_surfaces(typed_inventory: dict[str, Any] | None) -> dict[str, dict[str, Any]]:
    if typed_inventory is None:
        return {}
    surfaces = typed_inventory.get("typed_surfaces")
    if not isinstance(surfaces, list):
        return {}
    return {
        str(surface["name"]): surface
        for surface in surfaces
        if isinstance(surface, dict) and surface.get("name") is not None
    }


def _surface_results(
    surfaces: Any,
    *,
    typed_inventory: dict[str, Any] | None = None,
) -> tuple[list[dict[str, Any]], list[str]]:
    if not isinstance(surfaces, list) or not surfaces:
        return [], ["typed_surfaces_verified"]
    inventory = _inventory_surfaces(typed_inventory)
    results: list[dict[str, Any]] = []
    blockers: list[str] = []
    for index, surface in enumerate(surfaces):
        if not isinstance(surface, dict):
            blockers.append(f"typed_surfaces_verified[{index}]")
            continue
        name = _text(surface.get("name")) or f"typed_surfaces_verified[{index}]"
        actual_status = _text(surface.get("status"))
        inventory_status = _text(inventory.get(name, {}).get("status"))
        effective_status = (
            inventory_status
            if actual_status in {"", "pending"} and inventory_status in PASS_STATUSES
            else actual_status
        )
        status = "ready" if effective_status in PASS_STATUSES else "blocked"
        if status == "blocked":
            blockers.append(name)
        results.append(
            {
                "name": name,
                "legacy_surface": surface.get("legacy_surface"),
                "typed_surface": surface.get("typed_surface"),
                "status": status,
                "actual_status": actual_status,
                "inventory_status": inventory_status or None,
            }
        )
    return results, blockers


def build_ros2_typed_idl_cutover_report(
    *,
    payload: dict[str, Any],
    input_path: str,
    require_evidence_files: bool = False,
) -> dict[str, Any]:
    evidence = payload.get("evidence") if isinstance(payload.get("evidence"), dict) else {}
    evidence_path_statuses = _evidence_path_status(evidence, input_path=input_path)
    live_smoke_path = evidence_path_statuses["live_smoke_report"]["path"]
    inventory_path = evidence_path_statuses["typed_inventory"]["path"]
    rollback_plan = evidence_path_statuses["rollback_plan"]["path"]
    live_smoke = _load_json_if_exists(
        evidence_path_statuses["live_smoke_report"]["resolved_path"]
    )
    typed_inventory = _load_json_if_exists(
        evidence_path_statuses["typed_inventory"]["resolved_path"]
    )
    surface_results, surface_blockers = _surface_results(
        payload.get("typed_surfaces_verified"),
        typed_inventory=typed_inventory,
    )

    blockers: list[str] = []
    for field in ("target_environment", "operator", "rollback_owner", "launch_profile"):
        if _is_placeholder(payload.get(field)):
            _append_blocker(blockers, field)
    if payload.get("json_writers_disabled") is not True:
        _append_blocker(blockers, "json_writers_disabled")
    for field, status_item in evidence_path_statuses.items():
        if not status_item["path_valid"]:
            _append_blocker(blockers, field)
    if _status(live_smoke) != "passed":
        _append_blocker(blockers, "live_smoke_report")
    if (
        require_evidence_files
        and evidence_path_statuses["typed_inventory"]["path_valid"]
        and not evidence_path_statuses["typed_inventory"]["exists"]
    ):
        _append_blocker(blockers, "typed_inventory")
    if (
        require_evidence_files
        and evidence_path_statuses["rollback_plan"]["path_valid"]
        and not evidence_path_statuses["rollback_plan"]["exists"]
    ):
        _append_blocker(blockers, "rollback_plan")
    for blocker in surface_blockers:
        _append_blocker(blockers, blocker)

    status = "ready" if not blockers else "blocked"
    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "input": input_path,
        "summary": {
            "target_environment": payload.get("target_environment"),
            "launch_profile": payload.get("launch_profile"),
            "json_writers_disabled": payload.get("json_writers_disabled") is True,
            "live_smoke_status": _status(live_smoke),
            "typed_inventory_present": typed_inventory is not None,
            "verified_surface_count": sum(
                1 for item in surface_results if item["status"] == "ready"
            ),
            "blocked_surface_count": sum(
                1 for item in surface_results if item["status"] == "blocked"
            ),
            "require_evidence_files": require_evidence_files,
            "evidence_path_validation_error_count": sum(
                1 for item in evidence_path_statuses.values() if not item["path_valid"]
            ),
        },
        "blockers": blockers,
        "typed_surfaces": surface_results,
        "evidence": {
            "live_smoke_report": live_smoke_path,
            "typed_inventory": inventory_path,
            "rollback_plan": rollback_plan,
            "path_statuses": evidence_path_statuses,
        },
        "next_actions": _next_actions(status=status, blockers=blockers),
    }


def _next_actions(*, status: str, blockers: list[str]) -> list[str]:
    if status == "blocked":
        return [f"Resolve ROS2 typed IDL cutover blockers: {', '.join(blockers)}."]
    return ["ROS2 typed IDL cutover evidence is ready for archive."]


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    report = build_ros2_typed_idl_cutover_report(
        payload=_load_json(args.input),
        input_path=args.input,
        require_evidence_files=args.require_evidence_files,
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
