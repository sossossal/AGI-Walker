#!/usr/bin/env python
"""Check current rc/stable release readiness and emit next actions."""

from __future__ import annotations

import argparse
import json
import re
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
    RELEASE_APPROVAL_STATUSES,
    build_release_manifest_artifact,
    validate_release_manifest_artifact,
    write_release_manifest_artifact,
)


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


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
    version = ""

    for line in content.splitlines():
        stripped = line.strip()
        if not title and stripped.startswith("# "):
            title = stripped[2:].strip()
            match = re.match(r"AGI-Walker\s+(.+?)\s+Release Notes$", title)
            if match:
                version = match.group(1).strip()
        if not summary and stripped.startswith("发布摘要："):
            summary = stripped.split("：", 1)[1].strip()
        if not summary and stripped.startswith("Release Summary:"):
            summary = stripped.split(":", 1)[1].strip()
        if title and summary and version:
            break

    metadata: dict[str, str] = {}
    if title:
        metadata["title"] = title
    if summary:
        metadata["release_summary"] = summary
    if version:
        metadata["version"] = version
    return metadata


def _derive_stable_version(current_version: str) -> str:
    stripped = re.sub(r"-(?:rc\d+|dev[\w.-]*)$", "", current_version)
    return stripped or current_version


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


def _default_build_id(channel: str, version: str) -> str:
    normalized_version = re.sub(r"[^A-Za-z0-9._-]+", "-", version)
    return f"{channel}-readiness-{normalized_version}"


def _build_preview(
    *,
    channel: str,
    version: str,
    build_id: str,
    project_root: str,
    source_root: str,
    changelog: str,
    changelog_title: str,
    release_summary: str,
    output_root: Path,
    source_root_hint: str,
    release_approval: dict[str, str | None] | None = None,
) -> dict[str, Any]:
    payload = build_release_manifest_artifact(
        build_id=build_id,
        version=version,
        channel=channel,
        release_summary=release_summary,
        changelog_path=changelog,
        changelog_title=changelog_title,
        release_approval=release_approval,
        project_root=project_root,
        source_root=source_root,
    )
    manifest_path = write_release_manifest_artifact(
        payload,
        output_root / f"{channel}_release_manifest.json",
    )
    validation_errors = validate_release_manifest_artifact(payload)
    preview = {
        "channel": channel,
        "version": version,
        "build_id": build_id,
        "manifest_path": str(manifest_path),
        "release_gate_status": payload["release_gate_status"],
        "release_gate": payload["release_gate"],
        "release_policy": payload["release_policy"],
        "release_source": payload["release_source"],
        "validation_errors": validation_errors,
        "next_actions": _build_next_actions(
            payload,
            source_root_hint=source_root_hint,
            output_root=output_root,
        ),
    }
    return preview


def _build_next_actions(
    payload: dict[str, Any],
    *,
    source_root_hint: str,
    output_root: Path,
) -> list[str]:
    actions: list[str] = []
    channel = payload["channel"]
    version = payload["version"]
    gate = payload["release_gate"]
    source = payload["release_source"]

    for item in payload.get("test_evidence", []):
        if item.get("status") == "passed":
            continue
        command = item.get("command")
        if item.get("status") == "blocked":
            actions.append(f"修复或重跑 {item.get('name')}: {command}")
        elif item.get("status") == "opt_in":
            actions.append(f"补齐可选 live 证据 {item.get('name')}: {command}")

    if gate.get("diagnostic_ready_domains", 0) > 0:
        for domain in payload.get("capability_matrix", {}).get("domains", []):
            if domain.get("status") == "diagnostic_ready":
                actions.append(
                    f"关闭诊断态域 {domain.get('id')}: {domain.get('summary')}"
                )

    if gate.get("release_approval_required", 0) > 0 and gate.get("release_approval_ready", 0) == 0:
        actions.append(
            "补齐 stable 签核: "
            f'python tools/build_release_artifact.py --version {version} --channel {channel} '
            f'--build-id {payload["build_id"]} --approval-status approved --approved-by release-manager '
            f'--approved-at {_now_iso()} --approval-notes "stable signoff"'
        )

    if gate.get("release_source_required", 0) > 0 and gate.get("release_source_ready", 0) == 0:
        actions.append("确保在目标 Git 检出上执行，并让签核 commit 与当前 HEAD 一致。")

    if gate.get("release_worktree_required", 0) > 0 and gate.get("release_worktree_ready", 0) == 0:
        summary = source.get("worktree_status_summary")
        actions.append(
            "先生成 worktree 清理审计报告: "
            f"python tools/build_worktree_cleanup_report.py --source-root {source_root_hint} "
            f"--output-root {output_root}"
            + (
                f"。当前状态: {summary}"
                if isinstance(summary, str) and summary.strip()
                else ""
            )
        )

    if gate.get("release_version_tag_required", 0) > 0 and gate.get("release_version_tag_ready", 0) == 0:
        head = source.get("commit_sha") or "HEAD"
        actions.append(
            f"为当前 HEAD 创建匹配版本 tag: git tag {version} {head}"
        )

    if not actions and payload.get("release_gate_status") == "ready":
        actions.append(
            f'门禁已就绪，可生成最终 manifest: python tools/build_release_artifact.py --version {version} --channel {channel} --build-id {payload["build_id"]}'
        )

    return actions


def _is_non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _combine_next_step_plan(previews: list[dict[str, Any]]) -> list[str]:
    plan: list[str] = []
    for preview in previews:
        if preview["channel"] == "stable":
            plan.extend(preview["next_actions"])
            break
    if not plan:
        for preview in previews:
            plan.extend(preview["next_actions"])
    return plan


def _write_report(report_path: Path, payload: dict[str, Any]) -> Path:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return report_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Check rc/stable release readiness and print the next actions."
    )
    parser.add_argument(
        "--current-version",
        default=None,
        help="Current release version. Defaults to the version inferred from RELEASE_NOTES.md.",
    )
    parser.add_argument(
        "--stable-version",
        default=None,
        help="Stable target version. Defaults to current version with rc/dev suffix stripped.",
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to resolve evidence artifact paths.",
    )
    parser.add_argument(
        "--source-root",
        default=str(PROJECT_ROOT),
        help="Repository root used to resolve Git release source metadata.",
    )
    parser.add_argument(
        "--changelog",
        default="RELEASE_NOTES.md",
        help="Changelog path used for metadata inference.",
    )
    parser.add_argument(
        "--output-root",
        default=str(PROJECT_ROOT / "test_env" / "release_readiness"),
        help="Directory used to store preview manifests and the readiness report.",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        help="Optional override path for the structured readiness report.",
    )
    parser.add_argument(
        "--approval-status",
        choices=sorted(RELEASE_APPROVAL_STATUSES - {"not_required"}),
        default=None,
        help="Optional approval status used for the stable readiness preview.",
    )
    parser.add_argument("--approved-by", default=None)
    parser.add_argument("--approved-at", default=None)
    parser.add_argument("--commit-sha", default=None)
    parser.add_argument("--approval-notes", default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    report_path = (
        Path(args.report_file)
        if args.report_file
        else Path(args.output_root) / "release_readiness_report.json"
    )
    default_output_root = PROJECT_ROOT / "test_env" / "release_readiness"
    output_root = Path(args.output_root)
    if args.report_file and output_root == default_output_root:
        output_root = report_path.parent
    output_root.mkdir(parents=True, exist_ok=True)

    changelog_path = _resolve_project_path(args.changelog, args.project_root)
    metadata = _extract_release_notes_metadata(changelog_path)
    current_version = args.current_version or metadata.get("version")
    if not current_version:
        parser.error(
            "--current-version is required when the changelog cannot provide a version."
        )
    stable_version = args.stable_version or _derive_stable_version(current_version)
    changelog_title = metadata.get("title", "AGI-Walker Release Notes")
    summary = metadata.get("release_summary", "Release readiness preview.")
    release_approval = _build_release_approval_payload(args)

    previews = [
        _build_preview(
            channel="rc",
            version=current_version,
            build_id=_default_build_id("rc", current_version),
            project_root=args.project_root,
            source_root=args.source_root,
            changelog=args.changelog,
            changelog_title=changelog_title,
            release_summary=summary,
            output_root=output_root,
            source_root_hint=args.source_root,
        ),
        _build_preview(
            channel="stable",
            version=stable_version,
            build_id=_default_build_id("stable", stable_version),
            project_root=args.project_root,
            source_root=args.source_root,
            changelog=args.changelog,
            changelog_title=changelog_title,
            release_summary=summary,
            output_root=output_root,
            source_root_hint=args.source_root,
            release_approval=release_approval,
        ),
    ]
    next_step_plan = _combine_next_step_plan(previews)

    payload = {
        "schema_version": "1.0",
        "artifact_type": "release_readiness_report",
        "generated_at": _now_iso(),
        "project_root": args.project_root,
        "source_root": args.source_root,
        "current_version": current_version,
        "stable_version": stable_version,
        "report_path": str(report_path),
        "previews": previews,
        "next_step_plan": next_step_plan,
    }
    written_report = _write_report(report_path, payload)

    rc_preview = next(item for item in previews if item["channel"] == "rc")
    stable_preview = next(item for item in previews if item["channel"] == "stable")
    print(f"release_readiness_written={written_report}")
    print(f"current_version={current_version}")
    print(f"stable_version={stable_version}")
    print(f"rc_release_gate={rc_preview['release_gate_status']}")
    print(f"stable_release_gate={stable_preview['release_gate_status']}")
    print(f"stable_next_actions={len(stable_preview['next_actions'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
