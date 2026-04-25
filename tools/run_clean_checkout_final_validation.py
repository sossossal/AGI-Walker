#!/usr/bin/env python
"""Create a temporary clean checkout from staged changes and run final validation."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for candidate in current.parents:
        if (candidate / ".git").exists() and (candidate / "agi_walker").exists():
            return candidate
    return current.parent


def _configure_stdio() -> None:
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")


_configure_stdio()
PROJECT_ROOT = _find_repo_root()


@dataclass(frozen=True)
class CommandResult:
    command: list[str]
    cwd: str
    returncode: int
    stdout: str
    stderr: str


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def build_validation_commands(
    python_executable: str,
    *,
    skip_worktree_blocker: bool,
    skip_readiness: bool,
) -> list[list[str]]:
    commands: list[list[str]] = []
    if not skip_worktree_blocker:
        commands.append([python_executable, "tools/run_worktree_release_blocker.py"])
    if not skip_readiness:
        commands.append([python_executable, "tools/check_release_readiness.py"])
    return commands


def _run(command: list[str], *, cwd: Path) -> CommandResult:
    result = subprocess.run(
        command,
        cwd=str(cwd),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    return CommandResult(
        command=command,
        cwd=str(cwd),
        returncode=result.returncode,
        stdout=result.stdout,
        stderr=result.stderr,
    )


def _remove_tree(path: Path) -> None:
    def _onerror(function, target_path, exc_info):  # type: ignore[no-untyped-def]
        os.chmod(target_path, 0o700)
        function(target_path)

    shutil.rmtree(path, onerror=_onerror)


def _require_clean_inputs(source_root: Path) -> None:
    result = _run(["git", "diff", "--cached", "--name-only"], cwd=source_root)
    staged_paths = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    if not staged_paths:
        raise RuntimeError("No staged changes found; clean checkout validation needs staged input.")


def _write_patch_file(source_root: Path, patch_path: Path) -> int:
    patch_result = _run(["git", "diff", "--cached", "--binary", "HEAD"], cwd=source_root)
    if patch_result.returncode != 0:
        raise RuntimeError(patch_result.stderr.strip() or "Failed to export staged patch.")
    patch_path.parent.mkdir(parents=True, exist_ok=True)
    patch_path.write_text(patch_result.stdout, encoding="utf-8")
    return sum(1 for line in patch_result.stdout.splitlines() if line.startswith("diff --git "))


def _build_copy_ignore(source_root: Path, output_root: Path):
    output_root_resolved = output_root.resolve()

    def _ignore(directory: str, entries: list[str]) -> list[str]:
        directory_path = Path(directory).resolve()
        ignored: list[str] = []
        if directory_path == (source_root / "test_env").resolve():
            allowed_entries = {"release_evidence"}
            for entry in entries:
                if entry not in allowed_entries and entry != output_root_resolved.name:
                    ignored.append(entry)
        for entry in entries:
            candidate = (directory_path / entry).resolve()
            if entry in {
                ".git",
                "__pycache__",
                ".pytest_cache",
                ".pytest_tmp",
                ".mypy_cache",
            }:
                ignored.append(entry)
                continue
            if entry.startswith("codex_extbind_"):
                ignored.append(entry)
                continue
            if entry.startswith("pytest-cache-files-"):
                ignored.append(entry)
                continue
            try:
                candidate.relative_to(output_root_resolved)
                ignored.append(entry)
                continue
            except ValueError:
                pass
        return ignored

    return _ignore


def _prepare_checkout(source_root: Path, checkout_root: Path, output_root: Path) -> list[CommandResult]:
    command_results: list[CommandResult] = []
    if checkout_root.exists():
        _remove_tree(checkout_root)
    checkout_root.parent.mkdir(parents=True, exist_ok=True)

    shutil.copytree(
        source_root,
        checkout_root,
        ignore=_build_copy_ignore(source_root, output_root),
    )

    commands = [
        (["git", "init"], checkout_root),
        (["git", "config", "user.name", "AGI-Walker Clean Checkout Validator"], checkout_root),
        (["git", "config", "user.email", "clean-checkout-validator@example.com"], checkout_root),
        (["git", "add", "-A"], checkout_root),
        (["git", "commit", "-m", "clean checkout validation snapshot"], checkout_root),
    ]

    for command, command_cwd in commands:
        result = _run(command, cwd=command_cwd)
        command_results.append(result)
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip() or result.stdout.strip() or f"Command failed: {' '.join(command)}")
    return command_results


def _write_report(report_path: Path, payload: dict[str, object]) -> Path:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return report_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Materialize staged changes into a temporary clean checkout and run final validation."
    )
    parser.add_argument(
        "--source-root",
        default=str(PROJECT_ROOT),
        help="Source repository root that contains the staged changes.",
    )
    parser.add_argument(
        "--output-root",
        default=str(PROJECT_ROOT / "test_env" / "clean_checkout_final_validation"),
        help="Output root used for the clean checkout snapshot and final validation report.",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        help="Optional explicit report path. Defaults to <output-root>/clean_checkout_final_validation_report.json.",
    )
    parser.add_argument(
        "--skip-worktree-blocker",
        action="store_true",
        help="Skip running tools/run_worktree_release_blocker.py inside the clean checkout.",
    )
    parser.add_argument(
        "--skip-readiness",
        action="store_true",
        help="Skip running tools/check_release_readiness.py inside the clean checkout.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    source_root = Path(args.source_root).resolve()
    output_root = Path(args.output_root).resolve()
    run_token = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_root = output_root / run_token
    report_path = (
        Path(args.report_file).resolve()
        if args.report_file
        else run_root / "clean_checkout_final_validation_report.json"
    )
    checkout_root = run_root / "checkout"
    patch_path = run_root / "staged.patch"

    _require_clean_inputs(source_root)
    diff_count = _write_patch_file(source_root, patch_path)
    checkout_commands = _prepare_checkout(source_root, checkout_root, output_root)

    validation_results: list[CommandResult] = []
    failed_commands: list[str] = []
    for command in build_validation_commands(
        sys.executable,
        skip_worktree_blocker=args.skip_worktree_blocker,
        skip_readiness=args.skip_readiness,
    ):
        result = _run(command, cwd=checkout_root)
        validation_results.append(result)
        if result.returncode != 0:
            failed_commands.append(" ".join(command))

    payload = {
        "schema_version": "1.0",
        "artifact_type": "clean_checkout_final_validation_report",
        "generated_at": _now_iso(),
        "source_root": str(source_root),
        "output_root": str(output_root),
        "run_root": str(run_root),
        "checkout_root": str(checkout_root),
        "patch_path": str(patch_path),
        "report_path": str(report_path),
        "staged_diff_count": diff_count,
        "status": "passed" if not failed_commands else "failed",
        "failed_commands": failed_commands,
        "checkout_commands": [asdict(item) for item in checkout_commands],
        "validation_commands": [asdict(item) for item in validation_results],
    }
    written_report = _write_report(report_path, payload)

    print(f"clean_checkout_final_validation_written={written_report}")
    print(f"clean_checkout_final_validation_status={payload['status']}")
    print(f"clean_checkout_final_validation_checkout_root={checkout_root}")
    print(f"clean_checkout_final_validation_staged_diff_count={diff_count}")
    return 0 if not failed_commands else 1


if __name__ == "__main__":
    raise SystemExit(main())
