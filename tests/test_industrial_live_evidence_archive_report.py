from __future__ import annotations

import json
from pathlib import Path

from tools.build_industrial_live_evidence_archive_report import main


ROOT = Path(__file__).resolve().parents[1]
README = ROOT / "README.md"
NEXT_STAGE_PLAN = ROOT / "docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md"
GUIDE = ROOT / "docs/guides/INDUSTRIAL_LIVE_EVIDENCE_ARCHIVE_20260427.md"
SLO_VIEW = ROOT / "docs/guides/SLA_SLO_VIEW_20260427.md"
OPERATOR_CHECKLIST_GUIDE = (
    ROOT / "docs/guides/OPERATOR_DELIVERY_CHECKLIST_AUTOMATION_20260427.md"
)


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def _ready_inputs() -> dict[str, object]:
    return {
        "industrial_live_evidence": {
            "enabled": True,
            "target_environment": "customer-prod-line-1",
            "access_method": "VPN + bastion + customer SSO",
            "install_entrypoint": "docs/runbooks/customer/install.md",
            "upgrade_entrypoint": "docs/runbooks/customer/upgrade.md",
            "rollback_entrypoint": "docs/runbooks/customer/rollback.md",
            "backup_restore_entrypoint": "docs/runbooks/customer/backup_restore.md",
            "closure_archive_root": "archive://customer-prod-line-1/window-001",
            "evidence_output_root": "test_env/industrial_live_evidence/customer-prod-line-1",
        }
    }


def _write_ready_required_evidence(tmp_path: Path) -> dict[str, Path]:
    paths = {
        "inputs": tmp_path / "external_mainline.inputs.json",
        "operator_checklist": tmp_path / "operator_delivery_checklist.json",
        "external_mainline_plan": tmp_path / "external_mainline_execution_plan.json",
        "output": tmp_path / "industrial_live_evidence_archive_report.json",
    }
    _write_json(paths["inputs"], _ready_inputs())
    _write_json(paths["operator_checklist"], {"schema_version": "1.0", "status": "ready"})
    _write_json(
        paths["external_mainline_plan"],
        {
            "schema_version": "1.0",
            "status": "ready",
            "steps": {
                "industrial_delivery_live_evidence": {
                    "status": "ready_to_run",
                    "managed_inputs_ready": True,
                }
            },
        },
    )
    return paths


def test_industrial_live_evidence_archive_report_ready(tmp_path: Path) -> None:
    paths = _write_ready_required_evidence(tmp_path)

    exit_code = main(
        [
            "--inputs-file",
            str(paths["inputs"]),
            "--operator-checklist",
            str(paths["operator_checklist"]),
            "--external-mainline-plan",
            str(paths["external_mainline_plan"]),
            "--output",
            str(paths["output"]),
        ]
    )

    assert exit_code == 0
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["status"] == "ready"
    assert payload["missing_fields"] == []
    assert payload["blockers"] == []
    assert payload["summary"]["target_environment"] == "customer-prod-line-1"
    assert payload["summary"]["external_mainline_managed_inputs_ready"] is True
    assert payload["summary"]["require_customer_site_smoke"] is False


def test_industrial_live_evidence_archive_strict_customer_site_smoke_blocks_missing(
    tmp_path: Path,
) -> None:
    paths = _write_ready_required_evidence(tmp_path)

    exit_code = main(
        [
            "--inputs-file",
            str(paths["inputs"]),
            "--operator-checklist",
            str(paths["operator_checklist"]),
            "--external-mainline-plan",
            str(paths["external_mainline_plan"]),
            "--require-customer-site-smoke",
            "--output",
            str(paths["output"]),
        ]
    )

    assert exit_code == 1
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert "customer_site_smoke" in payload["blockers"]
    assert payload["summary"]["require_customer_site_smoke"] is True
    customer_site_evidence = [
        item for item in payload["evidence"] if item["id"] == "customer_site_smoke"
    ][0]
    assert customer_site_evidence["required"] is True
    assert customer_site_evidence["reason"] == "evidence_missing"


def test_industrial_live_evidence_archive_strict_customer_site_smoke_passes(
    tmp_path: Path,
) -> None:
    paths = _write_ready_required_evidence(tmp_path)
    customer_site_smoke = tmp_path / "customer_site_live_smoke_report.json"
    _write_json(customer_site_smoke, {"schema_version": "1.0", "status": "passed"})

    exit_code = main(
        [
            "--inputs-file",
            str(paths["inputs"]),
            "--operator-checklist",
            str(paths["operator_checklist"]),
            "--external-mainline-plan",
            str(paths["external_mainline_plan"]),
            "--customer-site-smoke",
            str(customer_site_smoke),
            "--require-customer-site-smoke",
            "--output",
            str(paths["output"]),
        ]
    )

    assert exit_code == 0
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["status"] == "ready"
    assert payload["blockers"] == []
    customer_site_evidence = [
        item for item in payload["evidence"] if item["id"] == "customer_site_smoke"
    ][0]
    assert customer_site_evidence["required"] is True
    assert customer_site_evidence["status"] == "ready"


def test_industrial_live_evidence_archive_blocks_placeholder_fields(
    tmp_path: Path,
) -> None:
    paths = _write_ready_required_evidence(tmp_path)
    inputs = _ready_inputs()
    inputs["industrial_live_evidence"]["target_environment"] = (
        "<customer-production-environment-id>"
    )
    _write_json(paths["inputs"], inputs)

    exit_code = main(
        [
            "--inputs-file",
            str(paths["inputs"]),
            "--operator-checklist",
            str(paths["operator_checklist"]),
            "--external-mainline-plan",
            str(paths["external_mainline_plan"]),
            "--output",
            str(paths["output"]),
        ]
    )

    assert exit_code == 1
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert payload["missing_fields"] == ["target_environment"]
    assert "industrial_live_evidence_fields" in payload["blockers"]


def test_industrial_live_evidence_archive_blocks_waiting_external_mainline(
    tmp_path: Path,
) -> None:
    paths = _write_ready_required_evidence(tmp_path)
    _write_json(
        paths["external_mainline_plan"],
        {
            "schema_version": "1.0",
            "status": "ready",
            "steps": {
                "industrial_delivery_live_evidence": {
                    "status": "waiting_external_input",
                    "managed_inputs_ready": False,
                }
            },
        },
    )

    exit_code = main(
        [
            "--inputs-file",
            str(paths["inputs"]),
            "--operator-checklist",
            str(paths["operator_checklist"]),
            "--external-mainline-plan",
            str(paths["external_mainline_plan"]),
            "--output",
            str(paths["output"]),
        ]
    )

    assert exit_code == 1
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert "external_mainline_industrial_live_evidence_waiting" in payload["blockers"]
    assert payload["summary"]["external_mainline_managed_inputs_ready"] is False


def test_industrial_live_evidence_archive_docs_are_linked() -> None:
    path = "docs/guides/INDUSTRIAL_LIVE_EVIDENCE_ARCHIVE_20260427.md"
    tool = "tools/build_industrial_live_evidence_archive_report.py"

    assert tool in README.read_text(encoding="utf-8")
    assert path in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
    assert tool in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
    assert tool in GUIDE.read_text(encoding="utf-8")
    assert tool in SLO_VIEW.read_text(encoding="utf-8")
    assert tool in OPERATOR_CHECKLIST_GUIDE.read_text(encoding="utf-8")
