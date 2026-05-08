"""Deterministic orchestration for stable and industrial promotion checklists."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

from agi_walker.core.api.release_ops_contracts import (
    IndustrialPromotionChecklistRequest,
    IndustrialPromotionChecklistResult,
    StablePromotionChecklistRequest,
    StablePromotionChecklistResult,
)


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for candidate in current.parents:
        if (candidate / "pyproject.toml").exists() and (
            candidate / "agi_walker"
        ).exists():
            return candidate
    return current.parent


PROJECT_ROOT = _find_repo_root()
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def _load_repo_tool_module(module_name: str, relative_path: str):
    module_path = (PROJECT_ROOT / relative_path).resolve()
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"could not load tool module from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def execute_stable_promotion_checklist(
    request: StablePromotionChecklistRequest,
) -> StablePromotionChecklistResult:
    stable_tool = _load_repo_tool_module(
        "agi_walker_tools_build_stable_promotion_checklist",
        "tools/build_stable_promotion_checklist.py",
    )
    output_root = Path(request.output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    report_path = (
        Path(request.report_file)
        if request.report_file
        else output_root / "stable_promotion_checklist.json"
    )
    readiness_report_path = (
        Path(request.readiness_report)
        if request.readiness_report
        else report_path.parent / "release_readiness_report.json"
    )
    approval_args = {
        "--approval-status": request.approval_status,
        "--approved-by": request.approved_by,
        "--approved-at": request.approved_at,
        "--commit-sha": request.commit_sha,
        "--approval-notes": request.approval_notes,
    }
    readiness_payload, readiness_stdout, readiness_stderr = (
        stable_tool._load_readiness_report(
            readiness_report_path=readiness_report_path,
            refresh_readiness=(
                request.refresh_readiness
                or not bool(request.readiness_report)
                or bool(request.security_preflight_report)
            ),
            current_version=request.current_version,
            stable_version=request.stable_version,
            project_root=request.project_root,
            source_root=request.source_root,
            changelog=request.changelog,
            security_preflight_report=request.security_preflight_report,
            approval_args=approval_args,
            approval_manifest=request.approval_manifest,
        )
    )
    stable_preview = stable_tool._stable_preview(readiness_payload)
    manifest = stable_tool._load_manifest(stable_preview["manifest_path"])
    approval_manifest = (
        stable_tool._load_manifest(request.approval_manifest)
        if request.approval_manifest
        else None
    )

    evidence_steps = stable_tool._build_evidence_steps(manifest)
    security_preflight_step = stable_tool._build_security_preflight_step(stable_preview)
    vulnerability_exception_review_step = (
        stable_tool._build_vulnerability_exception_review_step(stable_preview)
    )
    (
        external_mainline_execution_plan,
        external_mainline_execution_plan_step,
    ) = stable_tool._build_external_mainline_execution_plan_step(stable_preview)
    (
        external_mainline_input_checklist,
        external_mainline_input_checklist_step,
    ) = stable_tool._build_external_mainline_input_checklist_step(stable_preview)
    worktree_release_blocker = stable_tool._build_worktree_release_blocker_preview(
        stable_preview,
        output_root=report_path.parent,
    )
    customer_delivery_step = stable_tool._build_customer_delivery_step(stable_preview)
    industrial_delivery_step = stable_tool._build_industrial_delivery_step(
        stable_preview
    )
    extension_execution_actuals, extension_execution_actuals_step = (
        stable_tool._build_extension_execution_actuals_step(stable_preview)
    )
    extension_external_bindings_step = (
        stable_tool._build_extension_external_bindings_step(
            project_root=request.project_root,
            actuals=extension_execution_actuals,
        )
    )
    domain_steps = stable_tool._build_domain_steps(manifest)
    prerequisite_steps = stable_tool._build_prerequisite_steps(
        manifest=manifest,
        stable_preview=stable_preview,
        worktree_release_blocker=worktree_release_blocker,
        source_root=request.source_root,
        output_root=report_path.parent,
    )
    blocking_steps_list = (
        evidence_steps
        + [
            security_preflight_step,
            vulnerability_exception_review_step,
            external_mainline_execution_plan_step,
            customer_delivery_step,
            industrial_delivery_step,
            extension_execution_actuals_step,
        ]
        + domain_steps
        + prerequisite_steps
    )
    optional_steps = (
        [extension_external_bindings_step]
        if extension_external_bindings_step is not None
        else []
    )
    if external_mainline_input_checklist_step is not None:
        optional_steps.append(external_mainline_input_checklist_step)
    final_step = stable_tool._build_final_step(
        manifest=manifest,
        stable_preview=stable_preview,
        prerequisite_steps=blocking_steps_list,
        approval_manifest=approval_manifest,
        approval_manifest_path=request.approval_manifest,
    )
    steps = blocking_steps_list + optional_steps + [final_step]
    blocking_steps = sum(
        1 for step in steps if step["blocking"] and step["status"] != "done"
    )
    completed_steps = sum(1 for step in steps if step["status"] == "done")
    pending_steps = sum(1 for step in steps if step["status"] != "done")

    payload = {
        "schema_version": "1.0",
        "artifact_type": "stable_promotion_checklist",
        "generated_at": stable_tool._now_iso(),
        "project_root": request.project_root,
        "source_root": request.source_root,
        "current_version": readiness_payload["current_version"],
        "stable_version": stable_preview["version"],
        "stable_release_gate": stable_preview["release_gate_status"],
        "current_head_commit": stable_preview["release_source"].get("commit_sha"),
        "current_head_short_commit": stable_preview["release_source"].get(
            "short_commit_sha"
        ),
        "matched_version_tag": stable_preview["release_source"].get(
            "matched_version_tag"
        ),
        "readiness_report_path": str(readiness_report_path),
        "stable_preview_manifest_path": stable_preview["manifest_path"],
        "approval_manifest_path": request.approval_manifest,
        "blocking_steps": blocking_steps,
        "completed_steps": completed_steps,
        "pending_steps": pending_steps,
        "ready_to_promote": blocking_steps == 0,
        "customer_delivery_surface": stable_preview.get(
            "customer_delivery_surface", {}
        ),
        "industrial_delivery_gate": stable_preview.get("industrial_delivery_gate", {}),
        "worktree_release_blocker": worktree_release_blocker,
        "external_mainline_execution_plan": external_mainline_execution_plan,
        "external_mainline_input_checklist": external_mainline_input_checklist,
        "extension_execution_actuals": extension_execution_actuals,
        "summary": stable_tool._build_summary(
            blocking_steps=blocking_steps,
            stable_gate=stable_preview["release_gate_status"],
            worktree_release_blocker=worktree_release_blocker,
            external_mainline_execution_plan=external_mainline_execution_plan,
            external_mainline_input_checklist=external_mainline_input_checklist,
        ),
        "next_step_plan": [
            step["title"]
            for step in steps
            if step["blocking"] and step["status"] != "done"
        ]
        + [
            step["title"]
            for step in steps
            if step["category"] == "customer_delivery" and step["status"] != "done"
        ]
        + [
            step["title"]
            for step in steps
            if step["category"] == "industrial_delivery" and step["status"] != "done"
        ]
        + [
            step["title"]
            for step in steps
            if step["category"] == "extension_external_bindings"
            and step["status"] != "done"
        ]
        + [
            step["title"]
            for step in steps
            if step["category"] == "external_mainline" and step["status"] != "done"
        ]
        + [
            step["title"]
            for step in steps
            if step["category"] == "external_mainline_input_checklist"
            and step["status"] != "done"
        ]
        + ([final_step["title"]] if final_step["ready_to_run"] else []),
        "steps": steps,
        "readiness_stdout": readiness_stdout,
        "readiness_stderr": readiness_stderr,
    }
    written_report = stable_tool._write_report(report_path, payload)
    return StablePromotionChecklistResult(
        payload=payload,
        report_path=Path(written_report),
        readiness_payload=readiness_payload,
        readiness_report_path=readiness_report_path,
        stable_preview=stable_preview,
    )


def execute_industrial_promotion_checklist(
    request: IndustrialPromotionChecklistRequest,
) -> IndustrialPromotionChecklistResult:
    industrial_tool = _load_repo_tool_module(
        "agi_walker_tools_build_industrial_promotion_checklist",
        "tools/build_industrial_promotion_checklist.py",
    )
    output_root = Path(request.output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    report_path = (
        Path(request.report_file)
        if request.report_file
        else output_root / "industrial_promotion_checklist.json"
    )
    readiness_report_path = (
        Path(request.readiness_report)
        if request.readiness_report
        else report_path.parent / "industrial_release_readiness_report.json"
    )
    approval_args = {
        "--approval-status": request.approval_status,
        "--approved-by": request.approved_by,
        "--approved-at": request.approved_at,
        "--commit-sha": request.commit_sha,
        "--approval-notes": request.approval_notes,
    }
    readiness_payload, readiness_stdout, readiness_stderr = (
        industrial_tool._load_readiness_report(
            readiness_report_path=readiness_report_path,
            refresh_readiness=(
                request.refresh_readiness
                or not bool(request.readiness_report)
                or bool(request.security_preflight_report)
            ),
            current_version=request.current_version,
            industrial_version=request.industrial_version,
            project_root=request.project_root,
            source_root=request.source_root,
            changelog=request.changelog,
            security_preflight_report=request.security_preflight_report,
            approval_args=approval_args,
            approval_manifest=request.approval_manifest,
        )
    )
    industrial_preview = industrial_tool._industrial_preview(readiness_payload)
    manifest = industrial_tool._load_manifest(industrial_preview["manifest_path"])
    approval_manifest = (
        industrial_tool._load_manifest(request.approval_manifest)
        if request.approval_manifest
        else None
    )

    evidence_steps = industrial_tool._build_evidence_steps(manifest)
    security_preflight_step = industrial_tool._build_security_preflight_step(
        industrial_preview
    )
    vulnerability_exception_review_step = (
        industrial_tool._build_vulnerability_exception_review_step(industrial_preview)
    )
    (
        external_mainline_execution_plan,
        external_mainline_execution_plan_step,
    ) = industrial_tool._build_external_mainline_execution_plan_step(industrial_preview)
    (
        external_mainline_input_checklist,
        external_mainline_input_checklist_step,
    ) = industrial_tool._build_external_mainline_input_checklist_step(
        industrial_preview
    )
    worktree_release_blocker = industrial_tool._build_worktree_release_blocker_preview(
        industrial_preview,
        output_root=report_path.parent,
    )
    customer_delivery_step = industrial_tool._build_customer_delivery_step(
        industrial_preview
    )
    industrial_delivery_step = industrial_tool._build_industrial_delivery_step(
        industrial_preview
    )
    extension_execution_plan, extension_execution_step = (
        industrial_tool._build_extension_execution_step(industrial_preview)
    )
    (
        extension_execution_evidence,
        extension_execution_evidence_step,
    ) = industrial_tool._build_extension_execution_evidence_step(
        project_root=request.project_root,
        industrial_preview=industrial_preview,
    )
    (
        extension_execution_instance,
        extension_execution_instance_step,
    ) = industrial_tool._build_extension_execution_instance_step(
        project_root=request.project_root,
        industrial_preview=industrial_preview,
    )
    (
        extension_execution_schedule,
        extension_execution_schedule_step,
    ) = industrial_tool._build_extension_execution_schedule_step(
        project_root=request.project_root,
        industrial_preview=industrial_preview,
    )
    (
        extension_execution_actuals,
        extension_execution_actuals_step,
    ) = industrial_tool._build_extension_execution_actuals_step(
        project_root=request.project_root,
        industrial_preview=industrial_preview,
    )
    extension_external_bindings_step = (
        industrial_tool._build_extension_external_bindings_step(
            project_root=request.project_root,
            actuals=extension_execution_actuals,
        )
    )
    domain_steps = industrial_tool._build_domain_steps(manifest)
    prerequisite_steps = industrial_tool._build_prerequisite_steps(
        manifest=manifest,
        industrial_preview=industrial_preview,
        worktree_release_blocker=worktree_release_blocker,
        source_root=request.source_root,
        output_root=report_path.parent,
    )
    blocking_steps_list = (
        evidence_steps
        + [
            security_preflight_step,
            vulnerability_exception_review_step,
            external_mainline_execution_plan_step,
            customer_delivery_step,
            industrial_delivery_step,
            extension_execution_step,
            extension_execution_instance_step,
            extension_execution_schedule_step,
            extension_execution_actuals_step,
        ]
        + (
            [extension_external_bindings_step]
            if extension_external_bindings_step is not None
            else []
        )
        + (
            [external_mainline_input_checklist_step]
            if external_mainline_input_checklist_step is not None
            else []
        )
        + [extension_execution_evidence_step]
        + domain_steps
        + prerequisite_steps
    )
    final_step = industrial_tool._build_final_step(
        manifest=manifest,
        industrial_preview=industrial_preview,
        prerequisite_steps=blocking_steps_list,
        approval_manifest=approval_manifest,
        approval_manifest_path=request.approval_manifest,
    )
    steps = blocking_steps_list + [final_step]
    blocking_steps = sum(
        1 for step in steps if step["blocking"] and step["status"] != "done"
    )
    completed_steps = sum(1 for step in steps if step["status"] == "done")
    pending_steps = sum(1 for step in steps if step["status"] != "done")

    payload = {
        "schema_version": "1.0",
        "artifact_type": "industrial_promotion_checklist",
        "generated_at": industrial_tool._now_iso(),
        "project_root": request.project_root,
        "source_root": request.source_root,
        "current_version": readiness_payload["current_version"],
        "industrial_version": industrial_preview["version"],
        "industrial_release_gate": industrial_preview["release_gate_status"],
        "current_head_commit": industrial_preview["release_source"].get("commit_sha"),
        "current_head_short_commit": industrial_preview["release_source"].get(
            "short_commit_sha"
        ),
        "matched_version_tag": industrial_preview["release_source"].get(
            "matched_version_tag"
        ),
        "readiness_report_path": str(readiness_report_path),
        "industrial_preview_manifest_path": industrial_preview["manifest_path"],
        "approval_manifest_path": request.approval_manifest,
        "blocking_steps": blocking_steps,
        "completed_steps": completed_steps,
        "pending_steps": pending_steps,
        "ready_to_promote": blocking_steps == 0,
        "customer_delivery_surface": industrial_preview.get(
            "customer_delivery_surface", {}
        ),
        "industrial_delivery_gate": industrial_preview.get(
            "industrial_delivery_gate", {}
        ),
        "worktree_release_blocker": worktree_release_blocker,
        "external_mainline_execution_plan": external_mainline_execution_plan,
        "external_mainline_input_checklist": external_mainline_input_checklist,
        "extension_execution_plan": extension_execution_plan,
        "extension_execution_evidence": extension_execution_evidence,
        "extension_execution_instance": extension_execution_instance,
        "extension_execution_schedule": extension_execution_schedule,
        "extension_execution_actuals": extension_execution_actuals,
        "summary": industrial_tool._build_summary(
            blocking_steps=blocking_steps,
            worktree_release_blocker=worktree_release_blocker,
            external_mainline_execution_plan=external_mainline_execution_plan,
            external_mainline_input_checklist=external_mainline_input_checklist,
        ),
        "next_step_plan": [
            step["title"]
            for step in steps
            if step["blocking"] and step["status"] != "done"
        ]
        + [
            step["title"]
            for step in steps
            if step["category"] == "external_mainline" and step["status"] != "done"
        ]
        + [
            step["title"]
            for step in steps
            if step["category"] == "external_mainline_input_checklist"
            and step["status"] != "done"
        ]
        + ([final_step["title"]] if final_step["ready_to_run"] else []),
        "steps": steps,
        "readiness_stdout": readiness_stdout,
        "readiness_stderr": readiness_stderr,
    }
    written_report = industrial_tool._write_report(report_path, payload)
    return IndustrialPromotionChecklistResult(
        payload=payload,
        report_path=Path(written_report),
        readiness_payload=readiness_payload,
        readiness_report_path=readiness_report_path,
        industrial_preview=industrial_preview,
    )


__all__ = [
    "execute_industrial_promotion_checklist",
    "execute_stable_promotion_checklist",
]
