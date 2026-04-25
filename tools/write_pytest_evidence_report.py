#!/usr/bin/env python
"""Run pytest and write a structured release evidence report."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from time import perf_counter
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
    build_release_evidence_report,
    write_release_evidence_report,
)


PYTEST_COUNT_PATTERNS = {
    "passed": re.compile(r"(\d+)\s+passed\b"),
    "failed": re.compile(r"(\d+)\s+failed\b"),
    "skipped": re.compile(r"(\d+)\s+skipped\b"),
    "deselected": re.compile(r"(\d+)\s+deselected\b"),
    "error": re.compile(r"(\d+)\s+error(?:s)?\b"),
    "xfailed": re.compile(r"(\d+)\s+xfailed\b"),
    "xpassed": re.compile(r"(\d+)\s+xpassed\b"),
}
PYTEST_DURATION_PATTERN = re.compile(r"\bin\s+([0-9]+(?:\.[0-9]+)?)s\b")


def _resolve_project_path(path: str, project_root: Path) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return project_root / candidate


def _resolve_git_commit(source_root: Path) -> str | None:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=str(source_root),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if result.returncode != 0:
        return None
    value = result.stdout.strip()
    return value or None


def _parse_pytest_metrics(output: str) -> dict[str, int]:
    metrics: dict[str, int] = {}
    for name, pattern in PYTEST_COUNT_PATTERNS.items():
        match = pattern.search(output)
        metrics[name] = int(match.group(1)) if match else 0
    return metrics


def _parse_duration_seconds(output: str, fallback: float) -> float:
    match = PYTEST_DURATION_PATTERN.search(output)
    if match:
        return float(match.group(1))
    return round(fallback, 3)


def _format_pytest_summary(name: str, status: str, metrics: dict[str, int]) -> str:
    ordered_keys = [
        "passed",
        "failed",
        "skipped",
        "deselected",
        "error",
        "xfailed",
        "xpassed",
    ]
    parts = [f"{metrics[key]} {key}" for key in ordered_keys if metrics.get(key, 0) > 0]
    if not parts:
        parts = ["no pytest summary counts detected"]
    verb = "passed" if status == "passed" else "failed"
    return f"{name} pytest evidence {verb}: {', '.join(parts)}."


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run pytest and write a structured release evidence report."
    )
    parser.add_argument("--name", required=True, help="Evidence name recorded in the report.")
    parser.add_argument("--output", required=True, help="Output JSON path.")
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used as the pytest working directory.",
    )
    parser.add_argument(
        "--source-root",
        default=str(PROJECT_ROOT),
        help="Repository root used to resolve Git commit metadata.",
    )
    parser.add_argument(
        "pytest_args",
        nargs=argparse.REMAINDER,
        help="Arguments forwarded to python -m pytest. Prefix with -- to terminate parser options.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    pytest_args = list(args.pytest_args)
    if pytest_args and pytest_args[0] == "--":
        pytest_args = pytest_args[1:]
    if not pytest_args:
        parser.error("pytest arguments are required after --")

    project_root = Path(args.project_root)
    source_root = Path(args.source_root)
    output_path = _resolve_project_path(args.output, project_root)
    stdout_path = output_path.with_suffix(".stdout.txt")
    stderr_path = output_path.with_suffix(".stderr.txt")
    command = [sys.executable, "-m", "pytest", *pytest_args]
    started = datetime.now().isoformat()
    started_perf = perf_counter()
    result = subprocess.run(
        command,
        cwd=str(project_root),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    elapsed = perf_counter() - started_perf
    stdout_path.parent.mkdir(parents=True, exist_ok=True)
    stdout_path.write_text(result.stdout, encoding="utf-8")
    stderr_path.write_text(result.stderr, encoding="utf-8")
    combined_output = "\n".join(
        part for part in [result.stdout.strip(), result.stderr.strip()] if part
    )
    metrics = _parse_pytest_metrics(combined_output)
    status = "passed" if result.returncode == 0 else "blocked"
    summary = _format_pytest_summary(args.name, status, metrics)
    duration_seconds = _parse_duration_seconds(combined_output, elapsed)

    payload = build_release_evidence_report(
        evidence_name=args.name,
        status=status,
        summary=summary,
        command=" ".join(command),
        generated_at=started,
        exit_code=result.returncode,
        duration_seconds=duration_seconds,
        metrics=metrics,
        stdout_path=str(stdout_path),
        stderr_path=str(stderr_path),
        source_commit_sha=_resolve_git_commit(source_root),
    )
    written_path = write_release_evidence_report(payload, output_path)

    print(f"release_evidence_report_written={written_path}")
    print(f"release_evidence_name={args.name}")
    print(f"release_evidence_status={status}")
    print(f"release_evidence_summary={summary}")
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
