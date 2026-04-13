#!/usr/bin/env python
"""Build a canonical AGI-Walker release manifest."""

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

from agi_walker.core.api.release_contracts import (  # noqa: E402
    RELEASE_APPROVAL_STATUSES,
    build_release_manifest_artifact,
    default_release_test_evidence,
    write_release_manifest_artifact,
)


def _resolve_project_path(path: str, project_root: str) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return Path(project_root) / candidate


def _extract_release_notes_metadata(changelog_path: Path) -> dict[str, str]:
    if not changelog_path.is_file():
        return {}

    content = changelog_path.read_text(encoding="utf-8")
    title = ""
    summary = ""

    for line in content.splitlines():
        stripped = line.strip()
        if not title and stripped.startswith("# "):
            title = stripped[2:].strip()
        if not summary and stripped.startswith("发布摘要："):
            summary = stripped.split("：", 1)[1].strip()
        if not summary and stripped.startswith("Release Summary:"):
            summary = stripped.split(":", 1)[1].strip()
        if title and summary:
            break

    metadata: dict[str, str] = {}
    if title:
        metadata["title"] = title
    if summary:
        metadata["release_summary"] = summary
    return metadata


def _build_release_approval_payload(args: argparse.Namespace) -> dict[str, str | None] | None:
    if not any(
        [
            args.approval_status,
            args.approved_by,
            args.approved_at,
            args.commit_sha,
            args.approval_notes,
        ]
    ):
        return None

    return {
        "status": args.approval_status,
        "approved_by": args.approved_by,
        "approved_at": args.approved_at,
        "commit_sha": args.commit_sha,
        "notes": args.approval_notes,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build an AGI-Walker release manifest.")
    parser.add_argument("--version", required=True, help="Release version, e.g. 2026.04.12-rc1")
    parser.add_argument(
        "--channel",
        required=True,
        choices=("dev", "rc", "stable"),
        help="Release channel.",
    )
    parser.add_argument("--build-id", required=True, help="Stable build identifier.")
    parser.add_argument(
        "--release-summary",
        default=None,
        help="Short release summary for humans and automation.",
    )
    parser.add_argument(
        "--output",
        default=str(PROJECT_ROOT / "test_env" / "release" / "release_manifest.json"),
        help="Output path for the generated release manifest.",
    )
    parser.add_argument(
        "--changelog",
        default="RELEASE_NOTES.md",
        help="Path to the changelog/release notes file recorded in the manifest.",
    )
    parser.add_argument(
        "--changelog-title",
        default=None,
        help="Human-readable changelog title stored in the manifest.",
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Root directory used to resolve evidence artifact paths.",
    )
    parser.add_argument(
        "--source-root",
        default=str(PROJECT_ROOT),
        help="Repository root used to resolve Git release source metadata.",
    )
    parser.add_argument(
        "--distributed-report",
        default=None,
        help="Optional override path for distributed smoke report evidence.",
    )
    parser.add_argument(
        "--godot-headless-report",
        default=None,
        help="Optional override path for Godot headless smoke report evidence.",
    )
    parser.add_argument(
        "--ros2-bridge-report",
        default=None,
        help="Optional override path for ROS2 bridge smoke report evidence.",
    )
    parser.add_argument(
        "--approval-status",
        choices=sorted(RELEASE_APPROVAL_STATUSES - {"not_required"}),
        default=None,
        help="Optional release approval status override. Stable releases require approved signoff before the gate can become ready.",
    )
    parser.add_argument(
        "--approved-by",
        default=None,
        help="Approver identity recorded in release_approval.",
    )
    parser.add_argument(
        "--approved-at",
        default=None,
        help="Approval timestamp recorded in release_approval.",
    )
    parser.add_argument(
        "--commit-sha",
        default=None,
        help="Commit SHA covered by the approval record.",
    )
    parser.add_argument(
        "--approval-notes",
        default=None,
        help="Optional approval notes stored in release_approval.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    resolved_changelog = _resolve_project_path(args.changelog, args.project_root)
    changelog_metadata = _extract_release_notes_metadata(resolved_changelog)
    release_summary = args.release_summary or changelog_metadata.get("release_summary")
    changelog_title = args.changelog_title or changelog_metadata.get(
        "title",
        "AGI-Walker Release Notes",
    )

    if not release_summary:
        parser.error(
            "--release-summary is required unless the changelog contains a '发布摘要：' or 'Release Summary:' line."
        )

    test_evidence = default_release_test_evidence()
    artifact_overrides = {
        "distributed_runtime_live": args.distributed_report,
        "godot_headless_live": args.godot_headless_report,
        "ros2_bridge_live": args.ros2_bridge_report,
    }
    for item in test_evidence:
        override = artifact_overrides.get(item["name"])
        if override:
            item["artifact_path"] = override
    release_approval = _build_release_approval_payload(args)

    payload = build_release_manifest_artifact(
        build_id=args.build_id,
        version=args.version,
        channel=args.channel,
        release_summary=release_summary,
        changelog_path=args.changelog,
        changelog_title=changelog_title,
        test_evidence=test_evidence,
        release_approval=release_approval,
        project_root=args.project_root,
        source_root=args.source_root,
    )
    output_path = write_release_manifest_artifact(payload, args.output)

    print(f"release_manifest_written={output_path}")
    print(f"release_gate_status={payload['release_gate_status']}")
    print(f"release_source_commit_sha={payload['release_source']['commit_sha']}")
    print(f"release_source_worktree_clean={str(payload['release_source']['worktree_clean']).lower()}")
    print(f"known_limitations_count={len(payload['known_limitations'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
