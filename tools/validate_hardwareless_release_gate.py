from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence


EXPECTED_SCHEMA_VERSION = "hardwareless_acceptance_report.v1"


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate the release gate embedded in a hardwareless acceptance report."
    )
    parser.add_argument("report")
    parser.add_argument("--expect-status", choices=["ready", "blocked"], default="ready")
    parser.add_argument("--output")
    return parser.parse_args(argv)


def _load_report(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


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
    report = _load_report(args.report)
    errors = validate_hardwareless_release_gate(
        report,
        expected_status=args.expect_status,
    )
    result = {
        "status": "success" if not errors else "failed",
        "report": args.report,
        "expected_status": args.expect_status,
        "actual_status": (report.get("release_gate") or {}).get("status")
        if isinstance(report.get("release_gate"), dict)
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
