#!/usr/bin/env python
"""Build a minimal dependency SBOM artifact for AGI-Walker."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


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

from agi_walker.core.api.security_posture_contracts import (  # noqa: E402
    build_sbom_artifact,
    write_sbom_artifact,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build AGI-Walker SBOM artifact.")
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to scan pyproject and deployment requirements.",
    )
    parser.add_argument(
        "--output",
        default=str(PROJECT_ROOT / "test_env" / "security" / "sbom.json"),
        help="Output path for the generated sbom artifact.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    payload = build_sbom_artifact(project_root=args.project_root)
    output_path = write_sbom_artifact(payload, args.output)

    print(f"sbom_written={output_path}")
    print(f"sbom_components={payload['component_count']}")
    print(f"sbom_sources={len(payload['dependency_sources'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
