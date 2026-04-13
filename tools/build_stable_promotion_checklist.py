#!/usr/bin/env python
"""Build a structured stable-promotion checklist for the current HEAD."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
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


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _default_stable_build_id(version: str) -> str:
    normalized = re.sub(r"[^A-Za-z0-9._-]+", "-", version)
    return f"build-{normalized}-stable"


def _build_final_command(
    *,
    version: str,
    approval: dict[str, Any],
) -> str:
    approved_by = approval.get("approved_by") or "<approved-by>"
    approved_at = approval.get("approved_at") or "<approved-at-iso8601>"
    approval_notes = approval.get("notes") or "stable signoff"
    return (
        "python tools/build_release_artifact.py "
        f"--version {version} "
        "--channel stable "
        f"--build-id {_default_stable_build_id(version)} "
        "--approval-status approved "
        f"--approved-by {approved_by} "
        f"--approved-at {approved_at} "
        f'--approval-notes "{approval_notes}" '
        "--output test_env/release/release_manifest_stable.json"
    )


def _run_readiness_command(
    *,
    readiness_report_path: Path,
    current_version: str | None,
    stable_version: str | None,
    project_root: str,
    source_root: str,
    changelog: str,
    approval_args: dict[str, str | None],
    approval_manifest: str | None,
) -> tuple[dict[str, Any], str, str]:
    command = [
        sys.executable,
        str(PROJECT_ROOT / "tools" / "check_release_readiness.py"),
        "--output-root",
        str(readiness_report_path.parent),
        "--project-root",
        project_root,
        "--source-root",
        source_root,
        "--changelog",
        changelog,
        "--report-file",
        str(readiness_report_path),
    ]
    if current_version:
        command.extend(["--current-version", current_version])
    if stable_version:
        command.extend(["--stable-version", stable_version])
    if approval_manifest:
        command.extend(["--approval-manifest", approval_manifest])
    for option, value in approval_args.items():
        if value:
            command.extend([option, value])

    result = subprocess.run(
        command,
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    stdout = result.stdout.strip()
    stderr = result.stderr.strip()
    if result.returncode != 0:
        message = stderr or stdout or "release readiness command failed"
        raise RuntimeError(message)
    payload = json.loads(readiness_report_path.read_text(encoding="utf-8"))
    return payload, stdout, stderr


def _load_readiness_report(
    *,
    readiness_report_path: Path,
    refresh_readiness: bool,
    current_version: str | None,
    stable_version: str | None,
    project_root: str,
    source_root: str,
    changelog: str,
    approval_args: dict[str, str | None],
    approval_manifest: str | None,
) -> tuple[dict[str, Any], str, str]:
    if readiness_report_path.is_file() and not refresh_readiness:
        payload = json.loads(readiness_report_path.read_text(encoding="utf-8"))
        return payload, "", ""
    return _run_readiness_command(
        readiness_report_path=readiness_report_path,
        current_version=current_version,
        stable_version=stable_version,
        project_root=project_root,
        source_root=source_root,
        changelog=changelog,
        approval_args=approval_args,
        approval_manifest=approval_manifest,
    )


def _stable_preview(readiness_payload: dict[str, Any]) -> dict[str, Any]:
    return next(item for item in readiness_payload["previews"] if item["channel"] == "stable")


def _load_manifest(path: str) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _matches_ready_stable_manifest(
    approval_manifest: dict[str, Any] | None,
    *,
    stable_preview: dict[str, Any],
) -> bool:
    if not isinstance(approval_manifest, dict):
        return False
    if approval_manifest.get("artifact_type") != "release_manifest":
        return False
    if approval_manifest.get("channel") != "stable":
        return False
    if approval_manifest.get("release_gate_status") != "ready":
        return False
    if approval_manifest.get("version") != stable_preview.get("version"):
        return False
    manifest_source = approval_manifest.get("release_source", {})
    preview_source = stable_preview.get("release_source", {})
    return (
        manifest_source.get("commit_sha") == preview_source.get("commit_sha")
        and manifest_source.get("matched_version_tag")
        == preview_source.get("matched_version_tag")
    )


def _evidence_step_title(name: str) -> str:
    titles = {
        "distributed_runtime_live": "补齐 distributed live 证据",
        "godot_headless_live": "补齐 Godot headless live 证据",
        "ros2_bridge_live": "补齐 ROS2 bridge live 证据",
    }
    return titles.get(name, f"补齐证据 {name}")


def _build_evidence_steps(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    steps: list[dict[str, Any]] = []
    for item in manifest.get("test_evidence", []):
        if item.get("status") == "passed":
            continue
        status = "pending"
        steps.append(
            {
                "id": f"evidence:{item['name']}",
                "category": "evidence",
                "title": _evidence_step_title(str(item["name"])),
                "status": status,
                "required": True,
                "blocking": True,
                "summary": item.get("summary"),
                "command": item.get("command"),
                "ready_to_run": bool(item.get("command")),
                "depends_on": [],
            }
        )
    return steps


def _build_domain_steps(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    steps: list[dict[str, Any]] = []
    for domain in manifest.get("capability_matrix", {}).get("domains", []):
        if domain.get("status") != "diagnostic_ready":
            continue
        steps.append(
            {
                "id": f"domain:{domain['id']}",
                "category": "domain",
                "title": f"关闭发布面 {domain['id']} 的诊断态",
                "status": "pending",
                "required": True,
                "blocking": True,
                "summary": domain.get("summary"),
                "command": None,
                "ready_to_run": False,
                "depends_on": [],
            }
        )
    return steps


def _build_prerequisite_steps(
    *,
    manifest: dict[str, Any],
    stable_preview: dict[str, Any],
    source_root: str,
    output_root: Path,
) -> list[dict[str, Any]]:
    approval = manifest["release_approval"]
    source = stable_preview["release_source"]
    version = stable_preview["version"]
    steps: list[dict[str, Any]] = []

    approval_done = manifest["release_gate"]["release_approval_ready"] == 1
    steps.append(
        {
            "id": "stable_approval",
            "category": "approval",
            "title": "补齐 stable 签核元数据",
            "status": "done" if approval_done else "pending",
            "required": True,
            "blocking": not approval_done,
            "summary": (
                f"签核已完成: {approval.get('approved_by')} @ {approval.get('approved_at')}"
                if approval_done
                else "stable 通道要求 approved_by、approved_at、commit_sha 三项齐全。"
            ),
            "command": None,
            "ready_to_run": False,
            "depends_on": [],
        }
    )

    source_done = manifest["release_gate"]["release_source_ready"] == 1
    source_summary = (
        f"当前 HEAD 已绑定签核 SHA: {source.get('short_commit_sha')}"
        if source_done
        else (
            f"签核 commit 需与当前 HEAD 对齐: {source.get('commit_sha')}"
            if source.get("resolved_from_git")
            else "当前 source_root 未解析到有效 Git HEAD。"
        )
    )
    steps.append(
        {
            "id": "git_source_binding",
            "category": "git",
            "title": "确认 stable 签核绑定当前 Git HEAD",
            "status": "done" if source_done else "pending",
            "required": True,
            "blocking": not source_done,
            "summary": source_summary,
            "command": None,
            "ready_to_run": False,
            "depends_on": ["stable_approval"],
        }
    )

    worktree_done = manifest["release_gate"]["release_worktree_ready"] == 1
    worktree_summary = source.get("worktree_status_summary")
    steps.append(
        {
            "id": "clean_worktree",
            "category": "git",
            "title": "清理当前工作区并保持 release source 为 clean worktree",
            "status": "done" if worktree_done else "pending",
            "required": True,
            "blocking": not worktree_done,
            "summary": (
                "当前工作区已 clean。"
                if worktree_done
                else (
                    f"当前工作区未清理: {worktree_summary}"
                    if isinstance(worktree_summary, str) and worktree_summary.strip()
                    else "当前工作区未清理。"
                )
            ),
            "command": (
                None
                if worktree_done
                else (
                    "python tools/build_worktree_cleanup_report.py "
                    f"--source-root {source_root} "
                    f"--output-root {output_root}"
                )
            ),
            "ready_to_run": not worktree_done,
            "depends_on": [],
        }
    )

    tag_done = manifest["release_gate"]["release_version_tag_ready"] == 1
    steps.append(
        {
            "id": "version_tag",
            "category": "git",
            "title": "为当前 HEAD 创建匹配 stable 版本 tag",
            "status": "done" if tag_done else "pending",
            "required": True,
            "blocking": not tag_done,
            "summary": (
                f"已匹配 tag: {source.get('matched_version_tag')}"
                if tag_done
                else f"当前 HEAD 尚未匹配 {version} 或 v{version}。"
            ),
            "command": None if tag_done else f"git tag {version} {source.get('commit_sha')}",
            "ready_to_run": not tag_done,
            "depends_on": [],
        }
    )
    return steps


def _build_final_step(
    *,
    manifest: dict[str, Any],
    stable_preview: dict[str, Any],
    prerequisite_steps: list[dict[str, Any]],
    approval_manifest: dict[str, Any] | None,
    approval_manifest_path: str | None,
) -> dict[str, Any]:
    if _matches_ready_stable_manifest(approval_manifest, stable_preview=stable_preview):
        return {
            "id": "build_stable_manifest",
            "category": "release",
            "title": "生成 stable release manifest",
            "status": "done",
            "required": True,
            "blocking": False,
            "summary": (
                "已存在与当前 HEAD 匹配的 ready stable manifest。"
                + (
                    f" 来源: {approval_manifest_path}"
                    if approval_manifest_path
                    else ""
                )
            ),
            "command": None,
            "ready_to_run": False,
            "depends_on": [],
        }

    pending_blockers = [
        step["id"] for step in prerequisite_steps if step["blocking"] and step["status"] != "done"
    ]
    command = _build_final_command(
        version=stable_preview["version"],
        approval=manifest["release_approval"],
    )
    return {
        "id": "build_stable_manifest",
        "category": "release",
        "title": "生成 stable release manifest",
        "status": "pending",
        "required": True,
        "blocking": False,
        "summary": (
            "所有前置项已闭合，可直接执行 stable builder。"
            if not pending_blockers
            else "前置项未闭合，暂不应执行最终 stable builder。"
        ),
        "command": command,
        "ready_to_run": not pending_blockers,
        "depends_on": pending_blockers,
    }


def _build_summary(*, blocking_steps: int, stable_gate: str) -> str:
    if stable_gate == "ready":
        return "stable promotion 前置项已闭合，最终 manifest 生成命令可执行。"
    return f"stable promotion 仍有 {blocking_steps} 个阻塞前置项。"


def _write_report(report_path: Path, payload: dict[str, Any]) -> Path:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return report_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build a stable promotion checklist for the current HEAD."
    )
    parser.add_argument("--current-version", default=None)
    parser.add_argument("--stable-version", default=None)
    parser.add_argument("--project-root", default=str(PROJECT_ROOT))
    parser.add_argument("--source-root", default=str(PROJECT_ROOT))
    parser.add_argument("--changelog", default="RELEASE_NOTES.md")
    parser.add_argument(
        "--output-root",
        default=str(PROJECT_ROOT / "test_env" / "stable_promotion"),
    )
    parser.add_argument("--readiness-report", default=None)
    parser.add_argument("--refresh-readiness", action="store_true")
    parser.add_argument("--report-file", default=None)
    parser.add_argument("--approval-status", default=None)
    parser.add_argument("--approved-by", default=None)
    parser.add_argument("--approved-at", default=None)
    parser.add_argument("--commit-sha", default=None)
    parser.add_argument("--approval-notes", default=None)
    parser.add_argument(
        "--approval-manifest",
        default=None,
        help="Optional ready stable release manifest used to import approval metadata.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    output_root = Path(args.output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    report_path = (
        Path(args.report_file)
        if args.report_file
        else output_root / "stable_promotion_checklist.json"
    )
    readiness_report_path = (
        Path(args.readiness_report)
        if args.readiness_report
        else report_path.parent / "release_readiness_report.json"
    )

    approval_args = {
        "--approval-status": args.approval_status,
        "--approved-by": args.approved_by,
        "--approved-at": args.approved_at,
        "--commit-sha": args.commit_sha,
        "--approval-notes": args.approval_notes,
    }
    readiness_payload, readiness_stdout, readiness_stderr = _load_readiness_report(
        readiness_report_path=readiness_report_path,
        refresh_readiness=args.refresh_readiness or not bool(args.readiness_report),
        current_version=args.current_version,
        stable_version=args.stable_version,
        project_root=args.project_root,
        source_root=args.source_root,
        changelog=args.changelog,
        approval_args=approval_args,
        approval_manifest=args.approval_manifest,
    )
    stable_preview = _stable_preview(readiness_payload)
    manifest = _load_manifest(stable_preview["manifest_path"])
    approval_manifest = (
        _load_manifest(args.approval_manifest) if args.approval_manifest else None
    )

    evidence_steps = _build_evidence_steps(manifest)
    domain_steps = _build_domain_steps(manifest)
    prerequisite_steps = _build_prerequisite_steps(
        manifest=manifest,
        stable_preview=stable_preview,
        source_root=args.source_root,
        output_root=report_path.parent,
    )
    blocking_steps_list = evidence_steps + domain_steps + prerequisite_steps
    final_step = _build_final_step(
        manifest=manifest,
        stable_preview=stable_preview,
        prerequisite_steps=blocking_steps_list,
        approval_manifest=approval_manifest,
        approval_manifest_path=args.approval_manifest,
    )
    steps = blocking_steps_list + [final_step]
    blocking_steps = sum(
        1 for step in steps if step["blocking"] and step["status"] != "done"
    )
    completed_steps = sum(1 for step in steps if step["status"] == "done")
    pending_steps = sum(1 for step in steps if step["status"] != "done")

    payload = {
        "schema_version": "1.0",
        "artifact_type": "stable_promotion_checklist",
        "generated_at": _now_iso(),
        "project_root": args.project_root,
        "source_root": args.source_root,
        "current_version": readiness_payload["current_version"],
        "stable_version": stable_preview["version"],
        "stable_release_gate": stable_preview["release_gate_status"],
        "current_head_commit": stable_preview["release_source"].get("commit_sha"),
        "current_head_short_commit": stable_preview["release_source"].get("short_commit_sha"),
        "matched_version_tag": stable_preview["release_source"].get("matched_version_tag"),
        "readiness_report_path": str(readiness_report_path),
        "stable_preview_manifest_path": stable_preview["manifest_path"],
        "approval_manifest_path": args.approval_manifest,
        "blocking_steps": blocking_steps,
        "completed_steps": completed_steps,
        "pending_steps": pending_steps,
        "ready_to_promote": blocking_steps == 0,
        "summary": _build_summary(
            blocking_steps=blocking_steps,
            stable_gate=stable_preview["release_gate_status"],
        ),
        "next_step_plan": [
            step["title"]
            for step in steps
            if step["blocking"] and step["status"] != "done"
        ] + (
            [final_step["title"]] if final_step["ready_to_run"] else []
        ),
        "steps": steps,
        "readiness_stdout": readiness_stdout,
        "readiness_stderr": readiness_stderr,
    }
    written_report = _write_report(report_path, payload)

    print(f"stable_promotion_checklist_written={written_report}")
    print(f"stable_promotion_gate={stable_preview['release_gate_status']}")
    print(f"stable_promotion_blocking_steps={blocking_steps}")
    print(f"stable_promotion_ready_to_promote={str(blocking_steps == 0).lower()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
