from __future__ import annotations

import json
from pathlib import Path

from agi_walker.core.api.release_contracts import (
    build_release_evidence_report,
    write_release_evidence_report,
)
from agi_walker.core.api.release_ops_contracts import (
    ExternalMainlineExecutionRequest,
)
from agi_walker.ops.external_mainline import execute_external_mainline_execution
from tests.test_external_mainline_execution_plan import _write_industrial_report


def _write_review_report(project_root: Path) -> Path:
    report_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "security"
        / "vulnerability_exception_review_report.json"
    )
    return write_release_evidence_report(
        build_release_evidence_report(
            evidence_name="vulnerability_exception_review",
            status="passed",
            summary=(
                "vulnerability exception review evidence passed: "
                "31 active exception(s) require review."
            ),
            command="python tools/build_vulnerability_exception_review_report.py",
            metrics={
                "review_candidate_count": 31,
                "review_follow_up_required": True,
            },
        ),
        report_path,
    )


def test_execute_external_mainline_execution_returns_structured_result(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _write_industrial_report(project_root)
    commands: list[list[str]] = []

    def fake_run(command: list[str]) -> int:
        commands.append(command)
        if "build_vulnerability_exception_review_report.py" in command:
            _write_review_report(project_root)
        return 0

    request = ExternalMainlineExecutionRequest(
        project_root=str(project_root),
        inputs_file=None,
        skip_managed_inputs=True,
        output="test_env/release_evidence/operations/external_mainline_execution_plan.json",
        external_mainline_input_checklist_report=(
            "test_env/release_evidence/operations/"
            "external_mainline_input_checklist_report.json"
        ),
        customer_config="deployment/customer_delivery.external_bindings.customer.json",
        customer_external_bindings_closure_report=(
            "test_env/release_evidence/operations/"
            "customer_external_bindings_closure_report.json"
        ),
        vulnerability_exception_review_report=(
            "test_env/release_evidence/security/"
            "vulnerability_exception_review_report.json"
        ),
        industrial_delivery_rehearsal_report=(
            "test_env/release_rehearsal_industrial/"
            "industrial_delivery_rehearsal_report.json"
        ),
        skip_customer_external_bindings_closure=True,
    )

    result = execute_external_mainline_execution(
        request,
        run_command=fake_run,
        python_executable="python",
    )

    assert commands == [
        [
            "python",
            "tools/build_vulnerability_exception_review_report.py",
            "--project-root",
            str(project_root),
            "--output",
            "test_env/release_evidence/security/vulnerability_exception_review_report.json",
            "--exception-report",
            "test_env/release_evidence/security/vulnerability_exception_report.json",
        ]
    ]
    assert result.executed_steps == ["vulnerability_exception_review_refresh"]
    assert result.skipped_steps == [
        "industrial_rehearsal_refresh",
        "customer_external_bindings_closure",
    ]
    assert result.failures == []
    assert result.payload["status"] == "ready"
    assert result.output_path.is_file()
    assert result.checklist_path.is_file()
    assert result.checklist_payload["status"] == "blocked"
    assert result.industrial_live_evidence_inputs_ready is False


def test_execute_external_mainline_execution_requires_industrial_refresh_metadata(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)

    request = ExternalMainlineExecutionRequest(
        project_root=str(project_root),
        inputs_file=None,
        skip_managed_inputs=True,
        output="test_env/release_evidence/operations/external_mainline_execution_plan.json",
        external_mainline_input_checklist_report=(
            "test_env/release_evidence/operations/"
            "external_mainline_input_checklist_report.json"
        ),
        customer_config="deployment/customer_delivery.external_bindings.customer.json",
        customer_external_bindings_closure_report=(
            "test_env/release_evidence/operations/"
            "customer_external_bindings_closure_report.json"
        ),
        vulnerability_exception_review_report=(
            "test_env/release_evidence/security/"
            "vulnerability_exception_review_report.json"
        ),
        industrial_delivery_rehearsal_report=(
            "test_env/release_rehearsal_industrial/"
            "industrial_delivery_rehearsal_report.json"
        ),
        skip_vulnerability_exception_review_refresh=True,
        refresh_industrial_rehearsal=True,
    )

    try:
        execute_external_mainline_execution(
            request,
            run_command=lambda command: 0,
            python_executable="python",
        )
    except ValueError as exc:
        assert (
            str(exc)
            == "--refresh-industrial-rehearsal requires both "
            "--industrial-rehearsal-version and --industrial-rehearsal-build-id"
        )
    else:
        raise AssertionError("expected ValueError for missing industrial rehearsal metadata")


def test_execute_external_mainline_execution_tracks_managed_inputs_bootstrap(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _write_industrial_report(project_root)

    def fake_run(command: list[str]) -> int:
        if "tools/build_external_mainline_inputs.py" in command:
            output_path = project_root / "deployment" / "external_mainline.inputs.json"
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_text(
                json.dumps(
                    {
                        "customer_external_bindings": {
                            "enabled": True,
                            "config": "deployment/customer_delivery.external_bindings.customer.json",
                        },
                        "vulnerability_exception_review": {
                            "enabled": False,
                            "report_output": (
                                "test_env/release_evidence/security/"
                                "vulnerability_exception_review_report.json"
                            ),
                        },
                        "industrial_rehearsal": {
                            "refresh": False,
                            "report_path": (
                                "test_env/release_rehearsal_industrial/"
                                "industrial_delivery_rehearsal_report.json"
                            ),
                        },
                        "industrial_live_evidence": {},
                    }
                ),
                encoding="utf-8",
            )
        return 0

    request = ExternalMainlineExecutionRequest(
        project_root=str(project_root),
        inputs_file="deployment/external_mainline.inputs.json",
        skip_managed_inputs=False,
        output="test_env/release_evidence/operations/external_mainline_execution_plan.json",
        external_mainline_input_checklist_report=(
            "test_env/release_evidence/operations/"
            "external_mainline_input_checklist_report.json"
        ),
        customer_config="deployment/customer_delivery.external_bindings.customer.json",
        customer_external_bindings_closure_report=(
            "test_env/release_evidence/operations/"
            "customer_external_bindings_closure_report.json"
        ),
        vulnerability_exception_review_report=(
            "test_env/release_evidence/security/"
            "vulnerability_exception_review_report.json"
        ),
        industrial_delivery_rehearsal_report=(
            "test_env/release_rehearsal_industrial/"
            "industrial_delivery_rehearsal_report.json"
        ),
        skip_customer_external_bindings_closure=True,
    )

    result = execute_external_mainline_execution(
        request,
        run_command=fake_run,
        python_executable="python",
    )

    assert result.resolved_inputs_path == (
        project_root / "deployment" / "external_mainline.inputs.json"
    )
    assert result.managed_inputs_sync_status == "bootstrapped"
