from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[1]
EXPECTED_SCHEMA_VERSION = "hardwareless_acceptance_report.v1"


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate the release gate embedded in a hardwareless acceptance report."
    )
    parser.add_argument("report")
    parser.add_argument("--expect-status", choices=["ready", "blocked"], default="ready")
    parser.add_argument("--output")
    return parser.parse_args(argv)


def _text(value: Any) -> str:
    return value.strip() if isinstance(value, str) else ""


def _resolve_report_path(value: str | Path) -> tuple[bool, Path | None, str | None]:
    text = _text(str(value))
    if not text:
        return False, None, "empty"
    path = Path(text)
    if path.is_absolute():
        return False, None, "absolute"
    if ".." in path.parts:
        return False, None, "parent_directory"
    return True, PROJECT_ROOT / path, None


def _report_path_status(value: str | Path) -> dict[str, Any]:
    valid, resolved_path, error = _resolve_report_path(value)
    return {
        "path": _text(str(value)),
        "path_valid": valid,
        "path_error": error,
        "resolved_path": str(resolved_path) if resolved_path is not None else None,
        "exists": bool(resolved_path and resolved_path.exists()),
    }


def _load_report(path_status: dict[str, Any]) -> dict[str, Any] | None:
    if (
        not path_status["path_valid"]
        or not path_status["exists"]
        or not path_status["resolved_path"]
    ):
        return None
    return json.loads(Path(path_status["resolved_path"]).read_text(encoding="utf-8"))


def validate_hardwareless_release_gate(
    report: dict[str, Any],
    *,
    expected_status: str = "ready",
) -> list[str]:
    errors: list[str] = []
    if report.get("schema_version") != EXPECTED_SCHEMA_VERSION:
        errors.append(
            "schema_version must be "
            f"{EXPECTED_SCHEMA_VERSION!r}; got {report.get('schema_version')!r}"
        )
    release_gate = report.get("release_gate")
    if not isinstance(release_gate, dict):
        return [*errors, "release_gate must be an object"]
    actual_status = release_gate.get("status")
    if actual_status != expected_status:
        errors.append(
            f"release_gate.status must be {expected_status!r}; got {actual_status!r}"
        )
    if expected_status == "ready" and release_gate.get("blockers"):
        errors.append("release_gate.blockers must be empty when release gate is ready")
    return errors


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    report_path_status = _report_path_status(args.report)
    report = _load_report(report_path_status)
    if report is None:
        errors = [
            "report path must be repository-relative, must not traverse outside the repository, and must exist"
        ]
    else:
        errors = validate_hardwareless_release_gate(
            report,
            expected_status=args.expect_status,
        )
    result = {
        "status": "success" if not errors else "failed",
        "report": args.report,
        "report_path_status": report_path_status,
        "expected_status": args.expect_status,
        "actual_status": (report.get("release_gate") or {}).get("status")
        if isinstance(report, dict) and isinstance(report.get("release_gate"), dict)
        else None,
        "errors": errors,
    }
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
