"""Unified control-plane entry for deterministic release-ops actions."""

from __future__ import annotations

from dataclasses import MISSING, asdict, dataclass, fields
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
from typing import Any, Callable

from agi_walker.core.api.release_contracts import (
    EXTERNAL_MAINLINE_EXECUTION_PLAN_ARTIFACT_TYPE,
    build_release_evidence_report,
    default_canonical_industrial_delivery_rehearsal_report_path,
    default_customer_external_bindings_closure_report_path,
    default_customer_external_bindings_config_path,
    default_external_mainline_execution_plan_path,
    default_external_mainline_input_checklist_report_path,
    default_external_mainline_inputs_path,
    default_vulnerability_exception_review_report_path,
    write_external_mainline_execution_plan_artifact,
    write_release_evidence_report,
)
from agi_walker.core.api.release_ops_contracts import (
    CustomerAcceptanceBundleRequest,
    DEFAULT_RELEASE_OP_POLICY_PROFILE,
    ExternalMainlineExecutionRequest,
    IndustrialDeliveryRehearsalReportRequest,
    IndustrialPromotionChecklistRequest,
    IndustrialReleaseReadinessRequest,
    RELEASE_OP_POLICY_LEVELS,
    RELEASE_OP_POLICY_ORDER,
    RELEASE_OP_POLICY_PROFILES,
    ReleaseOpRequest,
    ReleaseOpResult,
    ReleaseOpSessionContext,
    ReleaseReadinessRequest,
    ReleaseRehearsalRequest,
    StablePromotionChecklistRequest,
    WorktreeReleaseBlockerRequest,
)
from agi_walker.ops.acceptance import execute_customer_acceptance_bundle
from agi_walker.ops.external_mainline import execute_external_mainline_execution
from agi_walker.ops.industrial_delivery import (
    execute_industrial_delivery_rehearsal_report,
)
from agi_walker.ops.promotion import (
    execute_industrial_promotion_checklist,
    execute_stable_promotion_checklist,
)
from agi_walker.ops.readiness import (
    execute_industrial_release_readiness,
    execute_release_readiness,
)
from agi_walker.ops.rehearsal import execute_release_rehearsal
from agi_walker.ops.worktree import (
    execute_worktree_release_blocker,
    write_worktree_release_blocker_report,
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
_POLICY_RANK = {level: index for index, level in enumerate(RELEASE_OP_POLICY_ORDER)}
_POLICY_PROFILE_DESCRIPTIONS = {
    "read_only": "Only read-only inspection and report lookup actions are allowed.",
    "local_safe_refresh": (
        "Allow deterministic local report refresh and checklist generation."
    ),
    "external_mutation": (
        "Allow local-safe refresh actions plus future local mutation actions."
    ),
    "requires_attestation": (
        "Allow attested actions such as external-mainline orchestration."
    ),
}

_TOP_LEVEL_CONTROL_PLANE_ARTIFACT_TYPES = frozenset(
    {
        "release_readiness_report",
        "industrial_release_readiness_report",
        "stable_promotion_checklist",
        "industrial_promotion_checklist",
    }
)


@dataclass(frozen=True, slots=True)
class _ReleaseOpDefinition:
    action: str
    description: str
    policy_level: str
    request_type: type
    build_default_request: Callable[[], Any]
    execute: Callable[[Any], Any]


def _default_release_readiness_request() -> ReleaseReadinessRequest:
    return ReleaseReadinessRequest(
        current_version=None,
        stable_version=None,
        project_root=".",
        source_root=".",
    )


def _default_industrial_release_readiness_request() -> (
    IndustrialReleaseReadinessRequest
):
    return IndustrialReleaseReadinessRequest(
        current_version=None,
        industrial_version=None,
        project_root=".",
        source_root=".",
    )


def _default_external_mainline_request() -> ExternalMainlineExecutionRequest:
    return ExternalMainlineExecutionRequest(
        project_root=".",
        inputs_file=default_external_mainline_inputs_path(),
        skip_managed_inputs=False,
        output=default_external_mainline_execution_plan_path(),
        external_mainline_input_checklist_report=(
            default_external_mainline_input_checklist_report_path()
        ),
        customer_config=default_customer_external_bindings_config_path(),
        customer_external_bindings_closure_report=(
            default_customer_external_bindings_closure_report_path()
        ),
        vulnerability_exception_review_report=(
            default_vulnerability_exception_review_report_path()
        ),
        industrial_delivery_rehearsal_report=(
            default_canonical_industrial_delivery_rehearsal_report_path()
        ),
    )


def _run_release_op_command(command: list[str]) -> int:
    result = subprocess.run(
        command,
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    return result.returncode


def _execute_external_mainline_action(request: ExternalMainlineExecutionRequest):
    return execute_external_mainline_execution(
        request,
        run_command=_run_release_op_command,
        python_executable=sys.executable,
    )


def _default_worktree_release_blocker_request() -> WorktreeReleaseBlockerRequest:
    return WorktreeReleaseBlockerRequest(source_root=".")


def _normalize_status(payload: dict[str, Any]) -> str:
    for field in [
        "status",
        "release_gate_status",
        "stable_release_gate",
        "industrial_release_gate",
        "bundle_status",
    ]:
        value = payload.get(field)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return "unknown"


def _normalize_summary(payload: dict[str, Any], *, action: str) -> str:
    summary = payload.get("summary")
    if isinstance(summary, str) and summary.strip():
        return summary.strip()
    artifact_type = payload.get("artifact_type")
    if isinstance(artifact_type, str) and artifact_type.strip():
        return f"{artifact_type.strip()} action={action} completed."
    return f"release op {action} completed."


def _normalize_output_path(result: Any) -> Path | None:
    for field in ["output_path", "report_path"]:
        value = getattr(result, field, None)
        if isinstance(value, Path):
            return value
    return None


def _instantiate_request(
    request_type: type, payload: dict[str, Any], default_request: Any
) -> Any:
    defaults = asdict(default_request)
    defaults.update(payload)
    try:
        return request_type(**defaults)
    except TypeError as exc:
        raise ValueError(
            f"invalid request payload for {request_type.__name__}: {exc}"
        ) from exc


def _normalize_session_context(
    session: ReleaseOpSessionContext | dict[str, Any],
) -> dict[str, str]:
    raw = asdict(session) if isinstance(session, ReleaseOpSessionContext) else session
    if not isinstance(raw, dict):
        raise ValueError("release op session context must be a JSON object")
    normalized: dict[str, str] = {}
    for field in ("engagement_id", "window_id", "change_ticket", "channel"):
        value = raw.get(field)
        if value is None:
            continue
        if not isinstance(value, str):
            raise ValueError(f"release op session field {field} must be a string")
        value = value.strip()
        if value:
            normalized[field] = value
    return normalized


def _resolve_event_stream_path(event_stream_file: str | None) -> Path | None:
    if event_stream_file is None:
        return None
    if not isinstance(event_stream_file, str):
        raise ValueError("release op event stream path must be a string")
    event_stream_file = event_stream_file.strip()
    if not event_stream_file:
        raise ValueError("release op event stream path must not be empty")
    return Path(event_stream_file)


def _resolve_evidence_report_path(evidence_report_file: str | None) -> Path | None:
    if evidence_report_file is None:
        return None
    if not isinstance(evidence_report_file, str):
        raise ValueError("release op evidence report path must be a string")
    evidence_report_file = evidence_report_file.strip()
    if not evidence_report_file:
        raise ValueError("release op evidence report path must not be empty")
    return Path(evidence_report_file)


def _emit_release_op_event(
    event_stream_path: Path | None,
    *,
    action: str,
    event_type: str,
    session: dict[str, str],
    payload: dict[str, Any],
    count: int,
) -> int:
    if event_stream_path is None:
        return count
    event_stream_path.parent.mkdir(parents=True, exist_ok=True)
    event = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "action": action,
        "event_type": event_type,
        "session": session,
        "payload": payload,
    }
    with event_stream_path.open("a", encoding="utf-8") as handle:
        handle.write(f"{json.dumps(event, ensure_ascii=False)}\n")
    return count + 1


def _enrich_external_mainline_execution_plan_payload(
    payload: dict[str, Any],
    *,
    session: dict[str, str],
    event_stream_path: Path | None,
    event_count: int,
) -> dict[str, Any]:
    if payload.get("artifact_type") != EXTERNAL_MAINLINE_EXECUTION_PLAN_ARTIFACT_TYPE:
        return payload
    enriched = dict(payload)
    if session:
        enriched["control_plane_session"] = dict(session)
    if event_stream_path is not None:
        enriched["control_plane_event_stream"] = {
            "path": str(event_stream_path),
            "event_count": event_count,
        }
    return enriched


def _enrich_release_evidence_payload(
    payload: dict[str, Any],
    *,
    session: dict[str, str],
    event_stream_path: Path | None,
    event_count: int,
) -> dict[str, Any]:
    if payload.get("artifact_type") != "release_evidence_report":
        return payload
    enriched = dict(payload)
    if session:
        enriched["control_plane_session"] = dict(session)
    if event_stream_path is not None:
        enriched["control_plane_event_stream"] = {
            "path": str(event_stream_path),
            "event_count": event_count,
        }
    return enriched


def _enrich_worktree_release_blocker_payload(
    payload: dict[str, Any],
    *,
    session: dict[str, str],
    event_stream_path: Path | None,
    event_count: int,
) -> dict[str, Any]:
    if payload.get("artifact_type") != "worktree_release_blocker_report":
        return payload
    enriched = dict(payload)
    if session:
        enriched["control_plane_session"] = dict(session)
    if event_stream_path is not None:
        enriched["control_plane_event_stream"] = {
            "path": str(event_stream_path),
            "event_count": event_count,
        }
    return enriched


def _enrich_top_level_control_plane_payload(
    payload: dict[str, Any],
    *,
    session: dict[str, str],
    event_stream_path: Path | None,
    event_count: int,
) -> dict[str, Any]:
    if payload.get("artifact_type") not in _TOP_LEVEL_CONTROL_PLANE_ARTIFACT_TYPES:
        return payload
    enriched = dict(payload)
    if session:
        enriched["control_plane_session"] = dict(session)
    if event_stream_path is not None:
        enriched["control_plane_event_stream"] = {
            "path": str(event_stream_path),
            "event_count": event_count,
        }
    return enriched


def _rewrite_json_payload(path: Path, payload: dict[str, Any]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return path


def _build_release_op_evidence_report_payload(
    *,
    action: str,
    policy_level: str,
    policy_profile: str,
    request_type: str,
    status: str,
    summary: str,
    payload: dict[str, Any],
    output_path: Path | None,
    session: dict[str, str],
    event_stream_path: Path | None,
    event_count: int,
) -> dict[str, Any]:
    metrics: dict[str, Any] = {
        "action": action,
        "policy_level": policy_level,
        "policy_profile": policy_profile,
        "request_type": request_type,
        "status": status,
        "event_count": event_count,
    }
    artifact_type = payload.get("artifact_type")
    if isinstance(artifact_type, str) and artifact_type.strip():
        metrics["artifact_type"] = artifact_type.strip()
    if output_path is not None:
        metrics["output_path"] = str(output_path)
    return build_release_evidence_report(
        evidence_name="release_ops_execution",
        status="passed" if status in {"ready", "passed"} else "blocked",
        summary=summary,
        command=f"release_ops::{action}",
        metrics=metrics,
        control_plane_session=session or None,
        control_plane_event_stream=(
            {
                "path": str(event_stream_path),
                "event_count": event_count,
            }
            if event_stream_path is not None
            else None
        ),
    )


_RELEASE_OP_DEFINITIONS = {
    "release_readiness": _ReleaseOpDefinition(
        action="release_readiness",
        description="Build the rc/stable release readiness report.",
        policy_level="local_safe_refresh",
        request_type=ReleaseReadinessRequest,
        build_default_request=_default_release_readiness_request,
        execute=execute_release_readiness,
    ),
    "industrial_release_readiness": _ReleaseOpDefinition(
        action="industrial_release_readiness",
        description="Build the industrial release readiness report.",
        policy_level="local_safe_refresh",
        request_type=IndustrialReleaseReadinessRequest,
        build_default_request=_default_industrial_release_readiness_request,
        execute=execute_industrial_release_readiness,
    ),
    "stable_promotion_checklist": _ReleaseOpDefinition(
        action="stable_promotion_checklist",
        description="Build the stable promotion checklist.",
        policy_level="local_safe_refresh",
        request_type=StablePromotionChecklistRequest,
        build_default_request=StablePromotionChecklistRequest,
        execute=execute_stable_promotion_checklist,
    ),
    "industrial_promotion_checklist": _ReleaseOpDefinition(
        action="industrial_promotion_checklist",
        description="Build the industrial promotion checklist.",
        policy_level="local_safe_refresh",
        request_type=IndustrialPromotionChecklistRequest,
        build_default_request=IndustrialPromotionChecklistRequest,
        execute=execute_industrial_promotion_checklist,
    ),
    "external_mainline_execution": _ReleaseOpDefinition(
        action="external_mainline_execution",
        description="Refresh the external-mainline plan and safe local substeps.",
        policy_level="requires_attestation",
        request_type=ExternalMainlineExecutionRequest,
        build_default_request=_default_external_mainline_request,
        execute=_execute_external_mainline_action,
    ),
    "release_rehearsal": _ReleaseOpDefinition(
        action="release_rehearsal",
        description="Run the stable release rehearsal flow.",
        policy_level="local_safe_refresh",
        request_type=ReleaseRehearsalRequest,
        build_default_request=ReleaseRehearsalRequest,
        execute=execute_release_rehearsal,
    ),
    "worktree_release_blocker": _ReleaseOpDefinition(
        action="worktree_release_blocker",
        description="Build the unified worktree cleanup/review blocker report.",
        policy_level="local_safe_refresh",
        request_type=WorktreeReleaseBlockerRequest,
        build_default_request=_default_worktree_release_blocker_request,
        execute=execute_worktree_release_blocker,
    ),
    "customer_acceptance_bundle": _ReleaseOpDefinition(
        action="customer_acceptance_bundle",
        description="Build the customer acceptance bundle.",
        policy_level="local_safe_refresh",
        request_type=CustomerAcceptanceBundleRequest,
        build_default_request=CustomerAcceptanceBundleRequest,
        execute=execute_customer_acceptance_bundle,
    ),
    "industrial_delivery_rehearsal_report": _ReleaseOpDefinition(
        action="industrial_delivery_rehearsal_report",
        description="Build the industrial delivery rehearsal report.",
        policy_level="local_safe_refresh",
        request_type=IndustrialDeliveryRehearsalReportRequest,
        build_default_request=IndustrialDeliveryRehearsalReportRequest,
        execute=execute_industrial_delivery_rehearsal_report,
    ),
}


def list_release_ops_actions() -> list[dict[str, str]]:
    actions: list[dict[str, str]] = []
    for definition in _RELEASE_OP_DEFINITIONS.values():
        if definition.policy_level not in RELEASE_OP_POLICY_LEVELS:
            raise ValueError(
                f"unsupported release op policy: {definition.policy_level}"
            )
        actions.append(
            {
                "action": definition.action,
                "description": definition.description,
                "policy_level": definition.policy_level,
                "request_type": definition.request_type.__name__,
            }
        )
    return actions


def list_release_ops_policy_profiles() -> list[dict[str, str]]:
    profiles: list[dict[str, str]] = []
    for profile in RELEASE_OP_POLICY_ORDER:
        if profile not in RELEASE_OP_POLICY_PROFILES:
            raise ValueError(f"unsupported release op policy profile: {profile}")
        profiles.append(
            {
                "policy_profile": profile,
                "description": _POLICY_PROFILE_DESCRIPTIONS.get(
                    profile,
                    f"Allow actions up to policy level {profile}.",
                ),
                "default": str(profile == DEFAULT_RELEASE_OP_POLICY_PROFILE).lower(),
            }
        )
    return profiles


def list_release_ops_request_templates(
    action: str | None = None,
) -> list[dict[str, Any]]:
    requested_actions: list[str]
    if action is None:
        requested_actions = list(_RELEASE_OP_DEFINITIONS)
    else:
        if action not in _RELEASE_OP_DEFINITIONS:
            raise ValueError(f"unsupported release op action: {action}")
        requested_actions = [action]

    templates: list[dict[str, Any]] = []
    for action_name in requested_actions:
        definition = _RELEASE_OP_DEFINITIONS[action_name]
        required_fields: list[str] = []
        optional_fields: list[str] = []
        for field_definition in fields(definition.request_type):
            if (
                field_definition.default is MISSING
                and field_definition.default_factory is MISSING
            ):
                required_fields.append(field_definition.name)
            else:
                optional_fields.append(field_definition.name)
        templates.append(
            {
                "action": definition.action,
                "description": definition.description,
                "policy_level": definition.policy_level,
                "default_policy_profile": DEFAULT_RELEASE_OP_POLICY_PROFILE,
                "request_type": definition.request_type.__name__,
                "required_fields": required_fields,
                "optional_fields": optional_fields,
                "request_template": asdict(definition.build_default_request()),
            }
        )
    return templates


def _validate_policy_profile(policy_profile: str) -> str:
    if policy_profile not in RELEASE_OP_POLICY_PROFILES:
        raise ValueError(f"unsupported release op policy profile: {policy_profile}")
    return policy_profile


def _policy_profile_allows(*, policy_profile: str, policy_level: str) -> bool:
    return _POLICY_RANK[policy_profile] >= _POLICY_RANK[policy_level]


def execute_release_op(request: ReleaseOpRequest) -> ReleaseOpResult:
    definition = _RELEASE_OP_DEFINITIONS.get(request.action)
    if definition is None:
        raise ValueError(f"unsupported release op action: {request.action}")
    if not isinstance(request.request, dict):
        raise ValueError("release op request payload must be a JSON object")
    policy_profile = _validate_policy_profile(request.policy_profile)
    session = _normalize_session_context(request.session)
    event_stream_path = _resolve_event_stream_path(request.event_stream_file)
    evidence_report_path = _resolve_evidence_report_path(request.evidence_report_file)
    event_count = 0
    event_count = _emit_release_op_event(
        event_stream_path,
        action=definition.action,
        event_type="action_started",
        session=session,
        payload={
            "policy_level": definition.policy_level,
            "policy_profile": policy_profile,
            "request_type": definition.request_type.__name__,
        },
        count=event_count,
    )
    if not _policy_profile_allows(
        policy_profile=policy_profile,
        policy_level=definition.policy_level,
    ):
        event_count = _emit_release_op_event(
            event_stream_path,
            action=definition.action,
            event_type="policy_denied",
            session=session,
            payload={
                "required_policy_level": definition.policy_level,
                "policy_profile": policy_profile,
            },
            count=event_count,
        )
        raise ValueError(
            "release op "
            f"{request.action} requires policy profile {definition.policy_level} "
            f"or higher; current profile is {policy_profile}"
        )
    typed_request = _instantiate_request(
        definition.request_type,
        request.request,
        definition.build_default_request(),
    )
    try:
        result = definition.execute(typed_request)
    except Exception as exc:
        event_count = _emit_release_op_event(
            event_stream_path,
            action=definition.action,
            event_type="action_failed",
            session=session,
            payload={
                "error_type": type(exc).__name__,
                "message": str(exc),
            },
            count=event_count,
        )
        raise
    payload = getattr(result, "payload", None)
    if not isinstance(payload, dict):
        raise ValueError(f"release op {request.action} did not return a payload object")
    output_path = _normalize_output_path(result)
    status = _normalize_status(payload)
    summary = _normalize_summary(payload, action=definition.action)
    event_count = _emit_release_op_event(
        event_stream_path,
        action=definition.action,
        event_type="action_completed",
        session=session,
        payload={
            "status": status,
            "summary": summary,
        },
        count=event_count,
    )
    if output_path is not None:
        event_count = _emit_release_op_event(
            event_stream_path,
            action=definition.action,
            event_type="artifact_written",
            session=session,
            payload={"output_path": str(output_path)},
            count=event_count,
        )
    enriched_payload = _enrich_external_mainline_execution_plan_payload(
        payload,
        session=session,
        event_stream_path=event_stream_path,
        event_count=event_count,
    )
    if enriched_payload is not payload:
        payload = enriched_payload
        if output_path is not None and output_path.is_file():
            write_external_mainline_execution_plan_artifact(payload, output_path)
    checklist_payload = getattr(result, "checklist_payload", None)
    checklist_path = getattr(result, "checklist_path", None)
    if isinstance(checklist_payload, dict) and isinstance(checklist_path, Path):
        enriched_checklist_payload = _enrich_release_evidence_payload(
            checklist_payload,
            session=session,
            event_stream_path=event_stream_path,
            event_count=event_count,
        )
        if (
            enriched_checklist_payload is not checklist_payload
            and checklist_path.is_file()
        ):
            write_release_evidence_report(enriched_checklist_payload, checklist_path)
    enriched_worktree_payload = _enrich_worktree_release_blocker_payload(
        payload,
        session=session,
        event_stream_path=event_stream_path,
        event_count=event_count,
    )
    if enriched_worktree_payload is not payload:
        payload = enriched_worktree_payload
        if output_path is not None and output_path.is_file():
            write_worktree_release_blocker_report(payload, output_path)
    enriched_top_level_payload = _enrich_top_level_control_plane_payload(
        payload,
        session=session,
        event_stream_path=event_stream_path,
        event_count=event_count,
    )
    if enriched_top_level_payload is not payload:
        payload = enriched_top_level_payload
        if output_path is not None and output_path.is_file():
            _rewrite_json_payload(output_path, payload)
    evidence_report_payload: dict[str, Any] = {}
    if evidence_report_path is not None:
        evidence_report_payload = _build_release_op_evidence_report_payload(
            action=definition.action,
            policy_level=definition.policy_level,
            policy_profile=policy_profile,
            request_type=definition.request_type.__name__,
            status=status,
            summary=summary,
            payload=payload,
            output_path=output_path,
            session=session,
            event_stream_path=event_stream_path,
            event_count=event_count,
        )
        write_release_evidence_report(evidence_report_payload, evidence_report_path)
    return ReleaseOpResult(
        action=definition.action,
        policy_level=definition.policy_level,
        policy_profile=policy_profile,
        request_type=definition.request_type.__name__,
        status=status,
        summary=summary,
        payload=payload,
        output_path=output_path,
        evidence_report_path=evidence_report_path,
        evidence_report_payload=evidence_report_payload,
        session=session,
        event_stream_path=event_stream_path,
        event_count=event_count,
    )


__all__ = [
    "execute_release_op",
    "list_release_ops_actions",
    "list_release_ops_policy_profiles",
    "list_release_ops_request_templates",
]
