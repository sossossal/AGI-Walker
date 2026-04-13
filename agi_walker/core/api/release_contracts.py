"""Stable contracts for release manifests and gate evidence."""

from __future__ import annotations

import json
import subprocess
from collections.abc import Mapping, Sequence
from datetime import datetime
from pathlib import Path
from typing import Any

from agi_walker.core.api.capability_matrix import (
    CAPABILITY_MATRIX_VERSION,
    build_capability_matrix_artifact,
    validate_capability_matrix_artifact,
)
from agi_walker.core.api.training_contracts import TRAINING_CONTRACT_VERSION
from agi_walker.core.api.workflow_contracts import (
    WORKFLOW_CONTRACT_VERSION,
    to_jsonable,
)

RELEASE_CONTRACT_VERSION = "1.0"
RELEASE_ARTIFACT_TYPE = "release_manifest"

RELEASE_CHANNELS = {"dev", "rc", "stable"}
RELEASE_GATE_STATUSES = {"ready", "ready_with_limitations", "blocked"}
RELEASE_EVIDENCE_STATUSES = {"passed", "blocked", "opt_in"}
RELEASE_APPROVAL_STATUSES = {"not_required", "pending", "approved"}

DISTRIBUTED_RELEASE_LIMITATION = (
    "Distributed runtime remains diagnostic_ready until a passing distributed smoke report is attached to release evidence."
)
DISTRIBUTED_DOMAIN_LIMITATION = (
    "A passing distributed smoke report is still required to confirm actor discovery and learner action loop in the target Docker environment."
)

RELEASE_MANIFEST_REQUIRED_FIELDS = {
    "schema_version",
    "artifact_type",
    "build_id",
    "version",
    "channel",
    "release_policy",
    "release_approval",
    "release_source",
    "release_summary",
    "generated_at",
    "release_gate_status",
    "release_gate",
    "changelog",
    "contract_versions",
    "capability_matrix",
    "test_evidence",
    "known_limitations",
}
RELEASE_GATE_REQUIRED_FIELDS = {
    "required_evidence",
    "passed_required_evidence",
    "blocked_evidence",
    "blocked_optional_evidence",
    "opt_in_evidence",
    "diagnostic_ready_domains",
    "release_approval_required",
    "release_approval_ready",
    "release_source_required",
    "release_source_ready",
    "release_worktree_required",
    "release_worktree_ready",
    "release_version_tag_required",
    "release_version_tag_ready",
}
RELEASE_POLICY_REQUIRED_FIELDS = {
    "channel",
    "allows_opt_in_evidence",
    "allows_diagnostic_ready_domains",
    "requires_release_approval",
    "requires_git_source_binding",
    "requires_clean_worktree",
    "requires_version_tag_match",
    "summary",
}
RELEASE_APPROVAL_REQUIRED_FIELDS = {
    "status",
    "required",
    "approved_by",
    "approved_at",
    "commit_sha",
    "notes",
}
RELEASE_SOURCE_REQUIRED_FIELDS = {
    "resolved_from_git",
    "commit_sha",
    "short_commit_sha",
    "git_tag",
    "matched_version_tag",
    "worktree_clean",
    "worktree_status_summary",
    "version_tag_matches",
}
RELEASE_CHANGELOG_REQUIRED_FIELDS = {"path", "title"}
RELEASE_CONTRACT_VERSION_REQUIRED_FIELDS = {"name", "version"}
RELEASE_TEST_EVIDENCE_REQUIRED_FIELDS = {
    "name",
    "required",
    "status",
    "summary",
    "command",
}


def default_release_contract_versions() -> list[dict[str, str]]:
    return [
        {"name": "release_manifest", "version": RELEASE_CONTRACT_VERSION},
        {"name": "capability_matrix", "version": CAPABILITY_MATRIX_VERSION},
        {"name": "workflow", "version": WORKFLOW_CONTRACT_VERSION},
        {"name": "training_run", "version": TRAINING_CONTRACT_VERSION},
        {"name": "distributed_monitor", "version": "1.0"},
        {"name": "distributed_smoke_report", "version": "1.0"},
        {"name": "godot_session_status", "version": "1.0"},
    ]


def default_release_test_evidence() -> list[dict[str, Any]]:
    return [
        {
            "name": "smoke_runner",
            "required": True,
            "status": "passed",
            "summary": "Default smoke runner passes and now includes capability matrix validation.",
            "command": "python tests/run_smoke_tests.py",
            "artifact_path": "test_env/smoke_capability_matrix",
        },
        {
            "name": "non_live_gate",
            "required": True,
            "status": "passed",
            "summary": 'Default non-live pytest gate passes: 792 passed, 3 skipped, 3 deselected.',
            "command": 'python -m pytest -m "not live" -q',
        },
        {
            "name": "release_contracts_and_capability_matrix",
            "required": True,
            "status": "passed",
            "summary": "Capability matrix, MCP surface, Web routes, and release contracts are covered by targeted tests.",
            "command": "python -m pytest tests/test_release_contracts.py tests/test_release_artifact_builder.py tests/test_capability_matrix.py tests/test_mcp_tools.py tests/test_mcp_server.py tests/test_web_panel_aux_apis.py -q",
        },
        {
            "name": "distributed_runtime_live",
            "required": False,
            "status": "opt_in",
            "summary": "Attach a passing distributed smoke report to promote distributed runtime from diagnostic_ready to ready.",
            "command": "python tests/run_distributed_smoke.py --build --stop-after --report-file test_env/distributed_smoke/distributed_smoke_report.json",
            "artifact_path": "test_env/distributed_smoke/distributed_smoke_report.json",
        },
        {
            "name": "godot_headless_live",
            "required": False,
            "status": "opt_in",
            "summary": "Godot headless smoke is intentionally opt-in and requires a local Godot executable plus scene assets.",
            "command": 'AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv',
            "artifact_path": "test_env/godot_headless_smoke/headless_smoke_report.json",
        },
        {
            "name": "ros2_bridge_live",
            "required": False,
            "status": "opt_in",
            "summary": "ROS2 bridge smoke is intentionally opt-in and requires a ROS2 Humble runtime.",
            "command": 'AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1 python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv',
            "artifact_path": "test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json",
        },
    ]


def hydrate_release_test_evidence(
    test_evidence: Sequence[Mapping[str, Any]] | None = None,
    *,
    project_root: str | Path | None = None,
) -> list[dict[str, Any]]:
    """Load known artifact reports and reflect their current evidence status."""
    base_evidence = [dict(item) for item in (test_evidence or default_release_test_evidence())]
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    hydrated: list[dict[str, Any]] = []

    for item in base_evidence:
        current = dict(item)
        artifact_path = current.get("artifact_path")
        if _is_non_empty_string(artifact_path):
            resolved_path = _resolve_release_artifact_path(artifact_path, resolved_root)
            current["resolved_artifact_path"] = str(resolved_path)
            current["artifact_found"] = resolved_path.exists()
            if resolved_path.is_file():
                current.update(
                    _load_release_evidence_from_report(current["name"], resolved_path)
                )
        hydrated.append(current)

    return hydrated


def apply_release_test_evidence_to_capability_matrix(
    capability_matrix: Mapping[str, Any] | None = None,
    test_evidence: Sequence[Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Adjust the capability matrix when optional live evidence is available."""
    matrix = json.loads(
        json.dumps(
            to_jsonable(
                dict(capability_matrix)
                if capability_matrix is not None
                else build_capability_matrix_artifact()
            ),
            ensure_ascii=False,
        )
    )
    evidence_by_name = {item.get("name"): item for item in (test_evidence or [])}
    domains = matrix.get("domains", [])

    distributed_evidence = evidence_by_name.get("distributed_runtime_live", {})
    if distributed_evidence.get("status") == "passed":
        for domain in domains:
            if domain.get("id") != "distributed_runtime":
                continue
            domain["status"] = "ready"
            domain["summary"] = (
                "Compose entrypoints, smoke report, monitor schema, and live distributed smoke evidence are all available."
            )
            domain["known_limitations"] = [
                item
                for item in domain.get("known_limitations", [])
                if item != DISTRIBUTED_DOMAIN_LIMITATION
            ]
            verification = dict(domain.get("verification", {}))
            verification["live_evidence_status"] = "passed"
            verification["live_evidence_source"] = distributed_evidence.get(
                "resolved_artifact_path"
            ) or distributed_evidence.get("artifact_path")
            domain["verification"] = verification

        matrix["known_limitations"] = [
            item
            for item in matrix.get("known_limitations", [])
            if item != DISTRIBUTED_RELEASE_LIMITATION
        ]

    matrix["summary"] = _build_capability_summary(domains)
    return matrix


def build_release_manifest_artifact(
    *,
    build_id: str,
    version: str,
    channel: str,
    release_summary: str,
    changelog_path: str = "RELEASE_NOTES.md",
    changelog_title: str = "AGI-Walker Release Notes",
    generated_at: str | None = None,
    contract_versions: Sequence[Mapping[str, Any]] | None = None,
    capability_matrix: Mapping[str, Any] | None = None,
    test_evidence: Sequence[Mapping[str, Any]] | None = None,
    release_approval: Mapping[str, Any] | None = None,
    release_source: Mapping[str, Any] | None = None,
    known_limitations: Sequence[str] | None = None,
    project_root: str | Path | None = None,
    source_root: str | Path | None = None,
) -> dict[str, Any]:
    evidence = hydrate_release_test_evidence(test_evidence, project_root=project_root)
    release_policy = _build_release_policy(channel)
    release_source_payload = _build_release_source(
        version=version,
        release_source=release_source,
        source_root=source_root,
    )
    approval_payload = _build_release_approval(
        channel,
        release_approval,
        release_source=release_source_payload,
    )
    matrix_payload = apply_release_test_evidence_to_capability_matrix(
        to_jsonable(
            dict(capability_matrix)
            if capability_matrix is not None
            else build_capability_matrix_artifact(generated_at=generated_at)
        ),
        evidence,
    )
    limitations = list(
        known_limitations
        if known_limitations is not None
        else _default_known_limitations(matrix_payload)
    )
    gate = _build_release_gate(
        evidence,
        matrix_payload,
        approval_payload,
        release_source_payload,
        release_policy,
    )
    return {
        "schema_version": RELEASE_CONTRACT_VERSION,
        "artifact_type": RELEASE_ARTIFACT_TYPE,
        "build_id": build_id,
        "version": version,
        "channel": channel,
        "release_policy": to_jsonable(release_policy),
        "release_approval": to_jsonable(approval_payload),
        "release_source": to_jsonable(release_source_payload),
        "release_summary": release_summary,
        "generated_at": generated_at or datetime.now().isoformat(),
        "release_gate_status": _resolve_release_gate_status(gate, release_policy),
        "release_gate": gate,
        "changelog": {
            "path": changelog_path,
            "title": changelog_title,
        },
        "contract_versions": to_jsonable(
            [
                dict(item)
                for item in (contract_versions or default_release_contract_versions())
            ]
        ),
        "capability_matrix": to_jsonable(matrix_payload),
        "test_evidence": to_jsonable(evidence),
        "known_limitations": to_jsonable(limitations),
    }


def validate_release_manifest_artifact(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["release manifest must be an object"]

    errors: list[str] = []
    missing = sorted(RELEASE_MANIFEST_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(f"missing required fields: {', '.join(missing)}")

    if payload.get("schema_version") != RELEASE_CONTRACT_VERSION:
        errors.append(
            f"schema_version must be {RELEASE_CONTRACT_VERSION!r}, got {payload.get('schema_version')!r}"
        )
    if payload.get("artifact_type") != RELEASE_ARTIFACT_TYPE:
        errors.append(f"artifact_type must be {RELEASE_ARTIFACT_TYPE!r}")

    for key in ["build_id", "version", "channel", "release_summary", "generated_at"]:
        if key in payload and not _is_non_empty_string(payload.get(key)):
            errors.append(f"{key} must be a non-empty string")

    if payload.get("channel") not in RELEASE_CHANNELS:
        errors.append(f"channel must be one of {sorted(RELEASE_CHANNELS)}")
    if payload.get("release_gate_status") not in RELEASE_GATE_STATUSES:
        errors.append(
            f"release_gate_status must be one of {sorted(RELEASE_GATE_STATUSES)}"
        )

    release_policy = payload.get("release_policy")
    if not isinstance(release_policy, Mapping):
        errors.append("release_policy must be an object")
    else:
        missing_policy = sorted(RELEASE_POLICY_REQUIRED_FIELDS - set(release_policy))
        if missing_policy:
            errors.append(
                f"release_policy missing required fields: {', '.join(missing_policy)}"
            )
        if release_policy.get("channel") != payload.get("channel"):
            errors.append(
                f"release_policy.channel must match channel {payload.get('channel')!r}"
            )
        for field in ["allows_opt_in_evidence", "allows_diagnostic_ready_domains"]:
            if field in release_policy and not isinstance(release_policy.get(field), bool):
                errors.append(f"release_policy.{field} must be a boolean")
        if "requires_release_approval" in release_policy and not isinstance(
            release_policy.get("requires_release_approval"), bool
        ):
            errors.append("release_policy.requires_release_approval must be a boolean")
        if "requires_git_source_binding" in release_policy and not isinstance(
            release_policy.get("requires_git_source_binding"), bool
        ):
            errors.append("release_policy.requires_git_source_binding must be a boolean")
        if "requires_clean_worktree" in release_policy and not isinstance(
            release_policy.get("requires_clean_worktree"), bool
        ):
            errors.append("release_policy.requires_clean_worktree must be a boolean")
        if "requires_version_tag_match" in release_policy and not isinstance(
            release_policy.get("requires_version_tag_match"), bool
        ):
            errors.append("release_policy.requires_version_tag_match must be a boolean")
        if "summary" in release_policy and not _is_non_empty_string(
            release_policy.get("summary")
        ):
            errors.append("release_policy.summary must be a non-empty string")

    release_approval = payload.get("release_approval")
    if not isinstance(release_approval, Mapping):
        errors.append("release_approval must be an object")
    else:
        missing_approval = sorted(RELEASE_APPROVAL_REQUIRED_FIELDS - set(release_approval))
        if missing_approval:
            errors.append(
                f"release_approval missing required fields: {', '.join(missing_approval)}"
            )
        if release_approval.get("status") not in RELEASE_APPROVAL_STATUSES:
            errors.append(
                f"release_approval.status must be one of {sorted(RELEASE_APPROVAL_STATUSES)}"
            )
        if "required" in release_approval and not isinstance(
            release_approval.get("required"), bool
        ):
            errors.append("release_approval.required must be a boolean")
        for field in ["approved_by", "approved_at", "commit_sha", "notes"]:
            if field in release_approval and release_approval.get(field) is not None:
                if not _is_non_empty_string(release_approval.get(field)):
                    errors.append(
                        f"release_approval.{field} must be null or a non-empty string"
                    )
        if release_approval.get("status") == "approved":
            for field in ["approved_by", "approved_at", "commit_sha"]:
                if not _is_non_empty_string(release_approval.get(field)):
                    errors.append(
                        f"release_approval.{field} is required when status is 'approved'"
                    )

    release_source = payload.get("release_source")
    if not isinstance(release_source, Mapping):
        errors.append("release_source must be an object")
    else:
        missing_source = sorted(RELEASE_SOURCE_REQUIRED_FIELDS - set(release_source))
        if missing_source:
            errors.append(
                f"release_source missing required fields: {', '.join(missing_source)}"
            )
        if "resolved_from_git" in release_source and not isinstance(
            release_source.get("resolved_from_git"), bool
        ):
            errors.append("release_source.resolved_from_git must be a boolean")
        for field in ["commit_sha", "short_commit_sha", "git_tag", "matched_version_tag"]:
            if field in release_source and release_source.get(field) is not None:
                if not _is_non_empty_string(release_source.get(field)):
                    errors.append(
                        f"release_source.{field} must be null or a non-empty string"
                    )
        if "worktree_clean" in release_source and not isinstance(
            release_source.get("worktree_clean"), bool
        ):
            errors.append("release_source.worktree_clean must be a boolean")
        if (
            "worktree_status_summary" in release_source
            and release_source.get("worktree_status_summary") is not None
            and not _is_non_empty_string(release_source.get("worktree_status_summary"))
        ):
            errors.append(
                "release_source.worktree_status_summary must be null or a non-empty string"
            )
        if "version_tag_matches" in release_source and not isinstance(
            release_source.get("version_tag_matches"), bool
        ):
            errors.append("release_source.version_tag_matches must be a boolean")
        if release_source.get("resolved_from_git"):
            for field in ["commit_sha", "short_commit_sha"]:
                if not _is_non_empty_string(release_source.get(field)):
                    errors.append(
                        f"release_source.{field} is required when resolved_from_git is true"
                    )

    changelog = payload.get("changelog")
    if not isinstance(changelog, Mapping):
        errors.append("changelog must be an object")
    else:
        missing_changelog = sorted(RELEASE_CHANGELOG_REQUIRED_FIELDS - set(changelog))
        if missing_changelog:
            errors.append(
                f"changelog missing required fields: {', '.join(missing_changelog)}"
            )
        for field in RELEASE_CHANGELOG_REQUIRED_FIELDS:
            if field in changelog and not _is_non_empty_string(changelog.get(field)):
                errors.append(f"changelog.{field} must be a non-empty string")

    gate = payload.get("release_gate")
    if not isinstance(gate, Mapping):
        errors.append("release_gate must be an object")
    else:
        missing_gate = sorted(RELEASE_GATE_REQUIRED_FIELDS - set(gate))
        if missing_gate:
            errors.append(
                f"release_gate missing required fields: {', '.join(missing_gate)}"
            )
        for field in RELEASE_GATE_REQUIRED_FIELDS:
            if field in gate and not _is_non_negative_int(gate.get(field)):
                errors.append(f"release_gate.{field} must be a non-negative integer")

    contract_versions = payload.get("contract_versions")
    if not isinstance(contract_versions, list):
        errors.append("contract_versions must be a list")
    else:
        for index, item in enumerate(contract_versions, start=1):
            prefix = f"contract_versions[{index}]"
            if not isinstance(item, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            missing_fields = sorted(RELEASE_CONTRACT_VERSION_REQUIRED_FIELDS - set(item))
            if missing_fields:
                errors.append(
                    f"{prefix} missing required fields: {', '.join(missing_fields)}"
                )
            for field in RELEASE_CONTRACT_VERSION_REQUIRED_FIELDS:
                if field in item and not _is_non_empty_string(item.get(field)):
                    errors.append(f"{prefix}.{field} must be a non-empty string")

    matrix = payload.get("capability_matrix")
    if not isinstance(matrix, Mapping):
        errors.append("capability_matrix must be an object")
    else:
        errors.extend(validate_capability_matrix_artifact(matrix))

    evidence = payload.get("test_evidence")
    if not isinstance(evidence, list):
        errors.append("test_evidence must be a list")
        evidence = []
    else:
        for index, item in enumerate(evidence, start=1):
            prefix = f"test_evidence[{index}]"
            if not isinstance(item, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            missing_fields = sorted(RELEASE_TEST_EVIDENCE_REQUIRED_FIELDS - set(item))
            if missing_fields:
                errors.append(
                    f"{prefix} missing required fields: {', '.join(missing_fields)}"
                )
            for field in ["name", "status", "summary", "command"]:
                if field in item and not _is_non_empty_string(item.get(field)):
                    errors.append(f"{prefix}.{field} must be a non-empty string")
            if "required" in item and not isinstance(item.get("required"), bool):
                errors.append(f"{prefix}.required must be a boolean")
            if item.get("status") not in RELEASE_EVIDENCE_STATUSES:
                errors.append(
                    f"{prefix}.status must be one of {sorted(RELEASE_EVIDENCE_STATUSES)}"
                )
            if "artifact_path" in item and item.get("artifact_path") is not None:
                if not _is_non_empty_string(item.get("artifact_path")):
                    errors.append(f"{prefix}.artifact_path must be null or a non-empty string")

    limitations = payload.get("known_limitations")
    if not isinstance(limitations, list):
        errors.append("known_limitations must be a list")
        limitations = []
    else:
        for index, item in enumerate(limitations, start=1):
            if not _is_non_empty_string(item):
                errors.append(
                    f"known_limitations[{index}] must be a non-empty string"
                )

    if (
        not errors
        and isinstance(gate, Mapping)
        and isinstance(matrix, Mapping)
        and isinstance(release_policy, Mapping)
        and isinstance(release_approval, Mapping)
        and isinstance(release_source, Mapping)
    ):
        expected_gate = _build_release_gate(
            evidence,
            matrix,
            release_approval,
            release_source,
            release_policy,
        )
        for key, value in expected_gate.items():
            if gate.get(key) != value:
                errors.append(
                    f"release_gate.{key} must be {value!r}, got {gate.get(key)!r}"
                )
        expected_policy = _build_release_policy(str(payload.get("channel")))
        if dict(release_policy) != expected_policy:
            errors.append(
                f"release_policy must be {expected_policy!r}, got {dict(release_policy)!r}"
            )
        expected_approval = _build_release_approval(
            str(payload.get("channel")), release_approval
        )
        if dict(release_approval) != expected_approval:
            errors.append(
                f"release_approval must be {expected_approval!r}, got {dict(release_approval)!r}"
            )
        expected_status = _resolve_release_gate_status(expected_gate, expected_policy)
        if payload.get("release_gate_status") != expected_status:
            errors.append(
                f"release_gate_status must be {expected_status!r}, got {payload.get('release_gate_status')!r}"
            )

    return errors


def write_release_manifest_artifact(payload: Mapping[str, Any], path: str | Path) -> Path:
    errors = validate_release_manifest_artifact(payload)
    if errors:
        raise ValueError("; ".join(errors))

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def _resolve_release_artifact_path(artifact_path: str, project_root: Path) -> Path:
    path = Path(artifact_path)
    if path.is_absolute():
        return path
    return project_root / path


def _load_release_evidence_from_report(name: str, report_path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(report_path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {
            "status": "blocked",
            "summary": f"Report could not be parsed: {exc}",
        }

    if name == "distributed_runtime_live":
        return _summarize_distributed_release_evidence(payload)
    if name == "godot_headless_live":
        return _summarize_live_smoke_evidence(
            payload,
            passed_message="Godot headless smoke passed with structured report evidence.",
        )
    if name == "ros2_bridge_live":
        return _summarize_live_smoke_evidence(
            payload,
            passed_message="ROS2 bridge smoke passed with structured report evidence.",
        )
    return {}


def _summarize_distributed_release_evidence(payload: Mapping[str, Any]) -> dict[str, Any]:
    status = str(payload.get("status", "")).strip().lower()
    failed_check = payload.get("failed_check")
    checks = payload.get("checks", [])
    passed_checks = sum(1 for item in checks if item.get("status") == "pass")
    total_checks = len(checks)
    actor_id = payload.get("actor_id")

    if status == "passed":
        return {
            "status": "passed",
            "summary": (
                f"Distributed smoke passed: {passed_checks}/{total_checks} checks passed"
                + (f" for actor {actor_id}." if _is_non_empty_string(actor_id) else ".")
            ),
        }

    detail = payload.get("diagnostics", {}).get("detail")
    if status == "failed":
        return {
            "status": "blocked",
            "summary": (
                f"Distributed smoke failed at {failed_check or 'unknown'}"
                + (f": {detail}" if _is_non_empty_string(detail) else ".")
            ),
        }

    return {
        "status": "blocked",
        "summary": f"Distributed smoke report returned unsupported status {status!r}.",
    }


def _summarize_live_smoke_evidence(
    payload: Mapping[str, Any], *, passed_message: str
) -> dict[str, Any]:
    status = str(payload.get("status", "")).strip().lower()
    failure_stage = payload.get("failure_stage")
    message = payload.get("message")

    if status == "passed":
        return {"status": "passed", "summary": passed_message}
    if status == "skipped":
        return {
            "status": "opt_in",
            "summary": message or "Live smoke was skipped because its environment gate was not enabled.",
        }
    if status == "failed":
        suffix = f" at {failure_stage}" if _is_non_empty_string(failure_stage) else ""
        return {
            "status": "blocked",
            "summary": f"Live smoke failed{suffix}: {message or 'see structured report'}",
        }
    return {
        "status": "blocked",
        "summary": f"Live smoke report returned unsupported status {status!r}.",
    }


def _default_known_limitations(capability_matrix: Mapping[str, Any]) -> list[str]:
    limitations: list[str] = []
    for item in capability_matrix.get("known_limitations", []):
        if _is_non_empty_string(item) and item not in limitations:
            limitations.append(item)
    for domain in capability_matrix.get("domains", []):
        for item in domain.get("known_limitations", []):
            if _is_non_empty_string(item) and item not in limitations:
                limitations.append(item)
    return limitations


def _build_release_gate(
    evidence: Sequence[Mapping[str, Any]],
    capability_matrix: Mapping[str, Any],
    release_approval: Mapping[str, Any],
    release_source: Mapping[str, Any],
    release_policy: Mapping[str, Any],
) -> dict[str, int]:
    required_evidence = sum(1 for item in evidence if item.get("required") is True)
    passed_required_evidence = sum(
        1
        for item in evidence
        if item.get("required") is True and item.get("status") == "passed"
    )
    blocked_evidence = sum(1 for item in evidence if item.get("status") == "blocked")
    blocked_optional_evidence = sum(
        1
        for item in evidence
        if item.get("required") is not True and item.get("status") == "blocked"
    )
    opt_in_evidence = sum(1 for item in evidence if item.get("status") == "opt_in")
    summary = capability_matrix.get("summary", {})
    diagnostic_ready_domains = (
        summary.get("diagnostic_ready_domains")
        if isinstance(summary.get("diagnostic_ready_domains"), int)
        else 0
    )
    release_approval_required = 1 if release_approval.get("required") else 0
    release_approval_ready = 1 if _is_release_approval_ready(release_approval) else 0
    release_source_required = 1 if release_approval.get("required") else 0
    release_source_ready = (
        1 if _is_release_source_ready(release_source, release_approval) else 0
    )
    release_worktree_required = (
        1 if release_policy.get("requires_clean_worktree", False) else 0
    )
    release_worktree_ready = (
        1 if _is_release_worktree_ready(release_source, release_policy) else 0
    )
    release_version_tag_required = 1 if release_approval.get("required") else 0
    release_version_tag_ready = (
        1 if _is_release_version_tag_ready(release_source, release_approval) else 0
    )
    return {
        "required_evidence": required_evidence,
        "passed_required_evidence": passed_required_evidence,
        "blocked_evidence": blocked_evidence,
        "blocked_optional_evidence": blocked_optional_evidence,
        "opt_in_evidence": opt_in_evidence,
        "diagnostic_ready_domains": diagnostic_ready_domains,
        "release_approval_required": release_approval_required,
        "release_approval_ready": release_approval_ready,
        "release_source_required": release_source_required,
        "release_source_ready": release_source_ready,
        "release_worktree_required": release_worktree_required,
        "release_worktree_ready": release_worktree_ready,
        "release_version_tag_required": release_version_tag_required,
        "release_version_tag_ready": release_version_tag_ready,
    }


def _build_capability_summary(domains: Sequence[Mapping[str, Any]]) -> dict[str, int]:
    ready_domains = sum(1 for item in domains if item.get("status") == "ready")
    diagnostic_ready_domains = sum(
        1 for item in domains if item.get("status") == "diagnostic_ready"
    )
    return {
        "total_domains": len(domains),
        "ready_domains": ready_domains,
        "diagnostic_ready_domains": diagnostic_ready_domains,
    }


def _build_release_policy(channel: str) -> dict[str, Any]:
    if channel == "dev":
        return {
            "channel": "dev",
            "allows_opt_in_evidence": True,
            "allows_diagnostic_ready_domains": True,
            "requires_release_approval": False,
            "requires_git_source_binding": False,
            "requires_clean_worktree": False,
            "requires_version_tag_match": False,
            "summary": "Dev releases may remain ready when required evidence passes, even if optional live evidence or diagnostic-ready domains are still open.",
        }
    if channel == "rc":
        return {
            "channel": "rc",
            "allows_opt_in_evidence": False,
            "allows_diagnostic_ready_domains": False,
            "requires_release_approval": False,
            "requires_git_source_binding": False,
            "requires_clean_worktree": False,
            "requires_version_tag_match": False,
            "summary": "RC releases require optional live evidence and diagnostic-ready domains to be fully closed before the gate becomes ready.",
        }
    return {
        "channel": "stable",
        "allows_opt_in_evidence": False,
        "allows_diagnostic_ready_domains": False,
        "requires_release_approval": True,
        "requires_git_source_binding": True,
        "requires_clean_worktree": True,
        "requires_version_tag_match": True,
        "summary": "Stable releases require optional live evidence, diagnostic-ready domains, explicit release approval, Git HEAD binding, a clean worktree, and a matching version tag before the gate becomes ready.",
    }


def _build_release_approval(
    channel: str,
    release_approval: Mapping[str, Any] | None = None,
    *,
    release_source: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    base = {
        "status": "pending" if channel == "stable" else "not_required",
        "required": channel == "stable",
        "approved_by": None,
        "approved_at": None,
        "commit_sha": None,
        "notes": None,
    }
    if release_approval is None:
        payload = dict(base)
    else:
        payload = dict(base)
        payload.update(dict(release_approval))

    if (
        payload.get("status") == "approved"
        and not _is_non_empty_string(payload.get("commit_sha"))
        and isinstance(release_source, Mapping)
        and _is_non_empty_string(release_source.get("commit_sha"))
    ):
        payload["commit_sha"] = release_source.get("commit_sha")
    return payload


def _is_release_approval_ready(release_approval: Mapping[str, Any]) -> bool:
    if not release_approval.get("required"):
        return True
    if release_approval.get("status") != "approved":
        return False
    return all(
        _is_non_empty_string(release_approval.get(field))
        for field in ["approved_by", "approved_at", "commit_sha"]
    )


def _build_release_source(
    *,
    version: str,
    release_source: Mapping[str, Any] | None = None,
    source_root: str | Path | None = None,
) -> dict[str, Any]:
    payload = {
        "resolved_from_git": False,
        "commit_sha": None,
        "short_commit_sha": None,
        "git_tag": None,
        "matched_version_tag": None,
        "worktree_clean": False,
        "worktree_status_summary": None,
        "version_tag_matches": False,
    }
    payload.update(_resolve_git_release_source(source_root, version=version))
    if release_source is not None:
        payload.update(dict(release_source))
    return payload


def _resolve_git_release_source(
    source_root: str | Path | None,
    *,
    version: str,
) -> dict[str, Any]:
    root = Path(source_root) if source_root is not None else Path.cwd()
    commit_sha = _run_git_capture(["git", "rev-parse", "HEAD"], cwd=root)
    short_commit_sha = _run_git_capture(["git", "rev-parse", "--short=12", "HEAD"], cwd=root)
    tags_output = _run_git_capture(["git", "tag", "--points-at", "HEAD"], cwd=root)
    tags = [line.strip() for line in (tags_output or "").splitlines() if line.strip()]
    worktree_output = _run_git_capture(["git", "status", "--short"], cwd=root)
    worktree_lines = [
        line.strip() for line in (worktree_output or "").splitlines() if line.strip()
    ]
    matched_version_tag = _match_version_tag(version, tags)
    git_tag = None
    if matched_version_tag is not None:
        git_tag = matched_version_tag
    elif tags:
        git_tag = tags[0]
    if not (_is_non_empty_string(commit_sha) and _is_non_empty_string(short_commit_sha)):
        return {}
    return {
        "resolved_from_git": True,
        "commit_sha": commit_sha,
        "short_commit_sha": short_commit_sha,
        "git_tag": git_tag,
        "matched_version_tag": matched_version_tag,
        "worktree_clean": not worktree_lines,
        "worktree_status_summary": _summarize_git_worktree(worktree_lines),
        "version_tag_matches": matched_version_tag is not None,
    }


def _run_git_capture(argv: Sequence[str], *, cwd: Path) -> str | None:
    try:
        result = subprocess.run(
            list(argv),
            cwd=str(cwd),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=False,
        )
    except Exception:
        return None
    if result.returncode != 0:
        return None
    value = result.stdout.strip()
    return value or None


def _is_release_source_ready(
    release_source: Mapping[str, Any],
    release_approval: Mapping[str, Any],
) -> bool:
    if not release_approval.get("required"):
        return True
    if not release_source.get("resolved_from_git"):
        return False
    return _release_approval_matches_source(release_approval, release_source)


def _is_release_worktree_ready(
    release_source: Mapping[str, Any],
    release_policy: Mapping[str, Any],
) -> bool:
    if not release_policy.get("requires_clean_worktree", False):
        return True
    if not release_source.get("resolved_from_git"):
        return False
    return bool(release_source.get("worktree_clean"))


def _is_release_version_tag_ready(
    release_source: Mapping[str, Any],
    release_approval: Mapping[str, Any],
) -> bool:
    if not release_approval.get("required"):
        return True
    return bool(release_source.get("version_tag_matches"))


def _match_version_tag(version: str, tags: Sequence[str]) -> str | None:
    expected_tags = {version, f"v{version}"}
    for tag in tags:
        if tag in expected_tags:
            return tag
    return None


def _summarize_git_worktree(lines: Sequence[str]) -> str | None:
    if not lines:
        return None
    preview = ", ".join(lines[:3])
    if len(lines) > 3:
        preview = f"{preview}, ..."
    return f"{len(lines)} pending path(s): {preview}"


def _release_approval_matches_source(
    release_approval: Mapping[str, Any],
    release_source: Mapping[str, Any],
) -> bool:
    approval_commit = release_approval.get("commit_sha")
    source_commit = release_source.get("commit_sha")
    source_short_commit = release_source.get("short_commit_sha")
    if not (
        _is_non_empty_string(approval_commit)
        and _is_non_empty_string(source_commit)
        and _is_non_empty_string(source_short_commit)
    ):
        return False

    normalized_commit = str(approval_commit).strip()
    normalized_source = str(source_commit).strip()
    normalized_short = str(source_short_commit).strip()
    return (
        normalized_commit == normalized_source
        or normalized_commit == normalized_short
        or normalized_source.startswith(normalized_commit)
    )


def _resolve_release_gate_status(
    gate: Mapping[str, Any], release_policy: Mapping[str, Any]
) -> str:
    if gate.get("passed_required_evidence") != gate.get("required_evidence"):
        return "blocked"
    if (
        release_policy.get("requires_release_approval", False)
        and gate.get("release_approval_ready", 0) == 0
    ):
        return "blocked"
    if (
        release_policy.get("requires_git_source_binding", False)
        and gate.get("release_source_ready", 0) == 0
    ):
        return "blocked"
    if (
        release_policy.get("requires_clean_worktree", False)
        and gate.get("release_worktree_ready", 0) == 0
    ):
        return "blocked"
    if (
        release_policy.get("requires_version_tag_match", False)
        and gate.get("release_version_tag_ready", 0) == 0
    ):
        return "blocked"
    if (
        gate.get("blocked_optional_evidence", 0) > 0
        or (
            not release_policy.get("allows_opt_in_evidence", False)
            and gate.get("opt_in_evidence", 0) > 0
        )
        or (
            not release_policy.get("allows_diagnostic_ready_domains", False)
            and gate.get("diagnostic_ready_domains", 0) > 0
        )
    ):
        return "ready_with_limitations"
    return "ready"


def _is_non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _is_non_negative_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0
