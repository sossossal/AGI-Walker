"""Stable contracts for release-facing capability matrix metadata."""

from __future__ import annotations

import json
from collections.abc import Mapping
from datetime import datetime
from pathlib import Path
from typing import Any

from agi_walker.core.api.training_contracts import TRAINING_CONTRACT_VERSION
from agi_walker.core.api.workflow_contracts import (
    WORKFLOW_CONTRACT_VERSION,
    to_jsonable,
)

CAPABILITY_MATRIX_VERSION = "1.0"
CAPABILITY_MATRIX_ARTIFACT_TYPE = "capability_matrix"
CAPABILITY_MATRIX_ROUTE = "/api/capabilities/matrix"

CAPABILITY_DOMAIN_IDS = {
    "cli",
    "web_panel",
    "mcp",
    "distributed_runtime",
    "godot_integration",
}
CAPABILITY_DOMAIN_STATUSES = {"ready", "diagnostic_ready"}

CAPABILITY_MATRIX_REQUIRED_FIELDS = {
    "schema_version",
    "artifact_type",
    "generated_at",
    "summary",
    "domains",
    "known_limitations",
}
CAPABILITY_MATRIX_SUMMARY_REQUIRED_FIELDS = {
    "total_domains",
    "ready_domains",
    "diagnostic_ready_domains",
}
CAPABILITY_DOMAIN_REQUIRED_FIELDS = {
    "id",
    "display_name",
    "status",
    "summary",
    "entrypoints",
    "contracts",
    "verification",
    "known_limitations",
}
CAPABILITY_CONTRACT_REQUIRED_FIELDS = {"name", "version"}


def build_capability_matrix_artifact(
    *,
    generated_at: str | None = None,
) -> dict[str, Any]:
    """Return the canonical release-facing capability matrix payload."""
    domains = [
        {
            "id": "cli",
            "display_name": "CLI",
            "status": "ready",
            "summary": "Primary operator entrypoint for skills, workflows, validation, and doctor checks.",
            "entrypoints": [
                "python -m agi_walker.cli",
            ],
            "contracts": [
                {"name": "workflow", "version": WORKFLOW_CONTRACT_VERSION},
                {"name": "capability_matrix", "version": CAPABILITY_MATRIX_VERSION},
            ],
            "verification": {
                "smoke_command": "python tests/run_smoke_tests.py",
                "non_live_gate": 'python -m pytest -m "not live" -q',
            },
            "known_limitations": [],
        },
        {
            "id": "web_panel",
            "display_name": "Web Panel",
            "status": "ready",
            "summary": "FastAPI control plane exposes stable system, workflow, nightly, distributed, and capability routes.",
            "entrypoints": [
                "python -m web_panel.server",
                "GET /api/system/status",
                CAPABILITY_MATRIX_ROUTE,
            ],
            "contracts": [
                {"name": "workflow", "version": WORKFLOW_CONTRACT_VERSION},
                {"name": "capability_matrix", "version": CAPABILITY_MATRIX_VERSION},
                {"name": "distributed_status", "version": "1.0"},
                {"name": "godot_session_status", "version": "1.0"},
            ],
            "verification": {
                "route_tests": [
                    "tests/test_web_panel_aux_apis.py",
                    "tests/test_web_godot_session_bridge.py",
                ],
                "nightly_jobs": [
                    "smoke",
                    "distributed-smoke",
                    "godot-headless-smoke",
                    "ros2-bridge-smoke",
                ],
            },
            "known_limitations": [],
        },
        {
            "id": "mcp",
            "display_name": "MCP",
            "status": "ready",
            "summary": "Structured stdio server exposes workflow, skills, telemetry, Godot, and capability matrix surfaces.",
            "entrypoints": [
                "agi-walker-mcp",
                "python -m agi_walker.mcp.server",
            ],
            "contracts": [
                {"name": "workflow", "version": WORKFLOW_CONTRACT_VERSION},
                {"name": "capability_matrix", "version": CAPABILITY_MATRIX_VERSION},
            ],
            "verification": {
                "tool_tests": [
                    "tests/test_mcp_tools.py",
                    "tests/test_mcp_server.py",
                ],
                "transport": "stdio",
            },
            "known_limitations": [
                "Only stdio transport is productized; alternate MCP transports are not declared here.",
            ],
        },
        {
            "id": "distributed_runtime",
            "display_name": "Distributed Runtime",
            "status": "diagnostic_ready",
            "summary": "Compose entrypoints, smoke report, and monitor schema are versioned; a passing distributed smoke report promotes this domain to ready.",
            "entrypoints": [
                "python tests/run_distributed_smoke.py --build --stop-after --report-file test_env/distributed_smoke/distributed_smoke_report.json",
                "GET /api/distributed/status",
            ],
            "contracts": [
                {"name": "distributed_smoke_report", "version": "1.0"},
                {"name": "distributed_monitor", "version": "1.0"},
                {"name": "capability_matrix", "version": CAPABILITY_MATRIX_VERSION},
            ],
            "verification": {
                "nightly_job": "distributed-smoke",
                "report_file": "test_env/distributed_smoke/distributed_smoke_report.json",
                "runner_test": "tests/test_distributed_smoke_runner.py",
            },
            "known_limitations": [
                "A passing distributed smoke report is still required to confirm actor discovery and learner action loop in the target Docker environment.",
            ],
        },
        {
            "id": "godot_integration",
            "display_name": "Godot Integration",
            "status": "ready",
            "summary": "Legacy controller and session bridge are both contract-backed, with headless and ROS2 verification isolated as opt-in live smoke.",
            "entrypoints": [
                "GET /api/godot/capabilities",
                "GET /api/godot/{session_id}/status",
                "WS /ws/{session_id}",
            ],
            "contracts": [
                {"name": "workflow", "version": WORKFLOW_CONTRACT_VERSION},
                {"name": "training_run", "version": TRAINING_CONTRACT_VERSION},
                {"name": "godot_session_status", "version": "1.0"},
                {"name": "capability_matrix", "version": CAPABILITY_MATRIX_VERSION},
            ],
            "verification": {
                "route_tests": [
                    "tests/test_web_godot_session_bridge.py",
                    "tests/test_web_godot_integration.py",
                ],
                "live_jobs": [
                    "godot-headless-smoke",
                    "ros2-bridge-smoke",
                ],
            },
            "known_limitations": [
                "Headless Godot and ROS2 bridge smokes require explicit live environment enablement.",
            ],
        },
    ]
    payload = {
        "schema_version": CAPABILITY_MATRIX_VERSION,
        "artifact_type": CAPABILITY_MATRIX_ARTIFACT_TYPE,
        "generated_at": generated_at or datetime.now().isoformat(),
        "summary": _build_summary(domains),
        "domains": domains,
        "known_limitations": [
            "Distributed runtime remains diagnostic_ready until a passing distributed smoke report is attached to release evidence.",
            "Live Godot and ROS2 validation are intentionally excluded from the default non-live gate.",
        ],
    }
    return to_jsonable(payload)


def build_capability_matrix_summary(
    payload: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Return a compact summary suitable for status payloads."""
    matrix = (
        dict(payload) if payload is not None else build_capability_matrix_artifact()
    )
    summary = dict(matrix["summary"])
    return {
        "schema_version": CAPABILITY_MATRIX_VERSION,
        "artifact_type": CAPABILITY_MATRIX_ARTIFACT_TYPE,
        "route": CAPABILITY_MATRIX_ROUTE,
        "summary": summary,
    }


def validate_capability_matrix_artifact(payload: Any) -> list[str]:
    """Return validation errors for a capability-matrix payload."""
    if not isinstance(payload, Mapping):
        return ["capability matrix must be an object"]

    errors: list[str] = []
    missing = sorted(CAPABILITY_MATRIX_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(f"missing required fields: {', '.join(missing)}")

    if payload.get("schema_version") != CAPABILITY_MATRIX_VERSION:
        errors.append(
            f"schema_version must be {CAPABILITY_MATRIX_VERSION!r}, got {payload.get('schema_version')!r}"
        )
    if payload.get("artifact_type") != CAPABILITY_MATRIX_ARTIFACT_TYPE:
        errors.append(f"artifact_type must be {CAPABILITY_MATRIX_ARTIFACT_TYPE!r}")
    if "generated_at" in payload and not _is_non_empty_string(
        payload.get("generated_at")
    ):
        errors.append("generated_at must be a non-empty string")

    summary = payload.get("summary")
    if not isinstance(summary, Mapping):
        errors.append("summary must be an object")
    else:
        missing_summary = sorted(
            CAPABILITY_MATRIX_SUMMARY_REQUIRED_FIELDS - set(summary)
        )
        if missing_summary:
            errors.append(
                f"summary missing required fields: {', '.join(missing_summary)}"
            )
        for key in CAPABILITY_MATRIX_SUMMARY_REQUIRED_FIELDS:
            if key in summary and not _is_non_negative_int(summary.get(key)):
                errors.append(f"summary.{key} must be a non-negative integer")

    domains = payload.get("domains")
    if not isinstance(domains, list):
        errors.append("domains must be a list")
        domains = []
    else:
        seen_ids: set[str] = set()
        for index, domain in enumerate(domains, start=1):
            prefix = f"domains[{index}]"
            if not isinstance(domain, Mapping):
                errors.append(f"{prefix} must be an object")
                continue

            missing_domain = sorted(CAPABILITY_DOMAIN_REQUIRED_FIELDS - set(domain))
            if missing_domain:
                errors.append(
                    f"{prefix} missing required fields: {', '.join(missing_domain)}"
                )

            domain_id = domain.get("id")
            if not _is_non_empty_string(domain_id):
                errors.append(f"{prefix}.id must be a non-empty string")
            elif domain_id in seen_ids:
                errors.append(f"{prefix}.id duplicates {domain_id!r}")
            else:
                seen_ids.add(domain_id)
                if domain_id not in CAPABILITY_DOMAIN_IDS:
                    errors.append(
                        f"{prefix}.id must be one of {sorted(CAPABILITY_DOMAIN_IDS)}"
                    )

            for key in ["display_name", "summary", "status"]:
                if key in domain and not _is_non_empty_string(domain.get(key)):
                    errors.append(f"{prefix}.{key} must be a non-empty string")

            if domain.get("status") not in CAPABILITY_DOMAIN_STATUSES:
                errors.append(
                    f"{prefix}.status must be one of {sorted(CAPABILITY_DOMAIN_STATUSES)}"
                )

            for key in ["entrypoints", "known_limitations"]:
                value = domain.get(key)
                if not isinstance(value, list):
                    errors.append(f"{prefix}.{key} must be a list")
                    continue
                for item_index, item in enumerate(value, start=1):
                    if not _is_non_empty_string(item):
                        errors.append(
                            f"{prefix}.{key}[{item_index}] must be a non-empty string"
                        )

            contracts = domain.get("contracts")
            if not isinstance(contracts, list):
                errors.append(f"{prefix}.contracts must be a list")
            else:
                for contract_index, contract in enumerate(contracts, start=1):
                    contract_prefix = f"{prefix}.contracts[{contract_index}]"
                    if not isinstance(contract, Mapping):
                        errors.append(f"{contract_prefix} must be an object")
                        continue
                    missing_contract = sorted(
                        CAPABILITY_CONTRACT_REQUIRED_FIELDS - set(contract)
                    )
                    if missing_contract:
                        errors.append(
                            f"{contract_prefix} missing required fields: {', '.join(missing_contract)}"
                        )
                    for field in CAPABILITY_CONTRACT_REQUIRED_FIELDS:
                        if field in contract and not _is_non_empty_string(
                            contract.get(field)
                        ):
                            errors.append(
                                f"{contract_prefix}.{field} must be a non-empty string"
                            )

            if not isinstance(domain.get("verification"), Mapping):
                errors.append(f"{prefix}.verification must be an object")

        missing_ids = sorted(CAPABILITY_DOMAIN_IDS - seen_ids)
        if missing_ids:
            errors.append(
                f"domains must include all required ids: {', '.join(missing_ids)}"
            )

    known_limitations = payload.get("known_limitations")
    if not isinstance(known_limitations, list):
        errors.append("known_limitations must be a list")
    else:
        for index, item in enumerate(known_limitations, start=1):
            if not _is_non_empty_string(item):
                errors.append(f"known_limitations[{index}] must be a non-empty string")

    if not errors and isinstance(summary, Mapping) and isinstance(domains, list):
        calculated_summary = _build_summary(domains)
        for key, value in calculated_summary.items():
            if summary.get(key) != value:
                errors.append(
                    f"summary.{key} must be {value!r}, got {summary.get(key)!r}"
                )

    return errors


def write_capability_matrix_artifact(
    payload: Mapping[str, Any], path: str | Path
) -> Path:
    """Validate and write a capability-matrix artifact to disk."""
    errors = validate_capability_matrix_artifact(payload)
    if errors:
        raise ValueError("; ".join(errors))

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def _build_summary(domains: list[Mapping[str, Any]]) -> dict[str, int]:
    ready_domains = sum(1 for item in domains if item.get("status") == "ready")
    diagnostic_ready_domains = sum(
        1 for item in domains if item.get("status") == "diagnostic_ready"
    )
    return {
        "total_domains": len(domains),
        "ready_domains": ready_domains,
        "diagnostic_ready_domains": diagnostic_ready_domains,
    }


def _is_non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _is_non_negative_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0
