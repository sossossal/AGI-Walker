from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from tests.run_smoke_tests import (
    PROJECT_ROOT,
    SmokeCheck,
    _build_env,
    _godot_instruction_set_smoke_script,
    _ros2_instruction_set_smoke_script,
    _run_check,
    _simulated_circuit_replay_smoke_script,
    _write_clean_checkout_smoke_evidence_script,
    _write_external_mainline_ready_script,
    _write_live_release_evidence_script,
)


def test_run_check_reports_release_summaries_in_artifact_summary(
    tmp_path: Path,
) -> None:
    artifact_dir = tmp_path / "artifacts"
    artifact_dir.mkdir(parents=True, exist_ok=True)
    for file_name in [
        "release_readiness_report.json",
        "industrial_release_readiness_report.json",
        "stable_promotion_checklist.json",
        "industrial_promotion_checklist.json",
        "customer_acceptance_bundle.json",
        "customer_acceptance_bundle_industrial.json",
        "release_rehearsal_report.json",
        "industrial_delivery_rehearsal_report.json",
    ]:
        (artifact_dir / file_name).write_text(
            json.dumps(
                {
                    "vulnerability_exception_review": {
                        "status": "passed",
                        "review_candidate_count": 5,
                    },
                    "control_plane_event_stream": {
                        "path": "test_env/release_evidence/operations/release_ops.events.jsonl",
                        "event_count": 3,
                    },
                    "external_mainline_execution_plan": {
                        "status": "ready",
                        "completed_steps": 0,
                        "ready_to_run_steps": 1,
                        "waiting_external_input_steps": 2,
                        "blocked_steps": 0,
                    },
                    "external_mainline_input_checklist": {
                        "status": "blocked",
                        "metrics": {
                            "missing_input_count": 4,
                            "waiting_external_input_steps": [
                                "customer_external_bindings_closure",
                                "industrial_delivery_live_evidence",
                            ],
                            "ready_to_run_steps": ["vulnerability_exception_replacement"],
                            "completed_steps": [],
                        },
                    },
                    "worktree_release_blocker": {
                        "status": "blocked",
                        "total_paths": 7,
                        "tracked_review_candidate_count": 2,
                    },
                },
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )

    check = SmokeCheck(
        name="smoke-summary",
        command=[sys.executable, "-c", "print('smoke_summary_ok')"],
        expected_tokens=["smoke_summary_ok"],
        artifact_dir=artifact_dir,
    )

    ok, details = _run_check(check, _build_env())

    assert ok is True
    assert "release_readiness_report=" in details
    assert "industrial_release_readiness_report=" in details
    assert "stable_promotion_checklist=" in details
    assert "industrial_promotion_checklist=" in details
    assert "customer_acceptance_bundle=" in details
    assert "customer_acceptance_bundle_industrial=" in details
    assert "release_rehearsal_report=" in details
    assert "industrial_delivery_rehearsal_report=" in details
    assert "exception_review=passed/5" in details
    assert "external_mainline=ready/0/1/2/0" in details
    assert "external_mainline_input_checklist=blocked/4/2/1/0" in details
    assert "worktree=blocked/7/2" in details
    assert "control_plane_events=3" in details


def test_smoke_seed_scripts_write_self_contained_release_evidence(tmp_path: Path) -> None:
    env = _build_env()
    for script in [
        _write_clean_checkout_smoke_evidence_script(),
        _write_live_release_evidence_script(),
        _write_external_mainline_ready_script(),
    ]:
        result = subprocess.run(
            [sys.executable, "-c", script],
            cwd=tmp_path,
            env=env,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=False,
        )
        assert result.returncode == 0, result.stderr or result.stdout

    clean_checkout_report = json.loads(
        (
            tmp_path / "test_env" / "release_evidence" / "clean_checkout_smoke_report.json"
        ).read_text(encoding="utf-8")
    )
    assert clean_checkout_report["status"] == "passed"
    assert clean_checkout_report["runs"] == 1
    assert clean_checkout_report["run_reports"][0]["worktree_clean"] is True

    distributed_report = json.loads(
        (
            tmp_path / "test_env" / "distributed_smoke" / "distributed_smoke_report.json"
        ).read_text(encoding="utf-8")
    )
    assert distributed_report["status"] == "passed"

    external_mainline_report = json.loads(
        (
            tmp_path
            / "test_env"
            / "release_evidence"
            / "operations"
            / "external_mainline_execution_plan.json"
        ).read_text(encoding="utf-8")
    )
    assert external_mainline_report["status"] == "ready"
    assert external_mainline_report["waiting_external_input_steps"] == 0


def test_instruction_and_circuit_smoke_scripts_write_reports(tmp_path: Path) -> None:
    env = _build_env()
    script_envs = [
        (
            _godot_instruction_set_smoke_script(),
            {"AGI_WALKER_GODOT_INSTRUCTION_SMOKE_ARTIFACT_DIR": str(tmp_path / "godot")},
            tmp_path / "godot" / "godot_instruction_smoke_report.json",
            "smoke-demo",
        ),
        (
            _ros2_instruction_set_smoke_script(),
            {"AGI_WALKER_ROS2_INSTRUCTION_SMOKE_ARTIFACT_DIR": str(tmp_path / "ros2")},
            tmp_path / "ros2" / "ros2_instruction_smoke_report.json",
            "instruction_set_applied",
        ),
        (
            _simulated_circuit_replay_smoke_script(),
            {
                "AGI_WALKER_SIMULATED_CIRCUIT_SMOKE_ARTIFACT_DIR": str(
                    tmp_path / "circuit"
                )
            },
            tmp_path / "circuit" / "simulated_circuit_replay_smoke_report.json",
            "passed",
        ),
    ]

    for script, overrides, report_path, expected_value in script_envs:
        script_env = dict(env)
        script_env.update(overrides)
        result = subprocess.run(
            [sys.executable, "-c", script],
            cwd=PROJECT_ROOT,
            env=script_env,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=False,
        )
        assert result.returncode == 0, result.stderr or result.stdout
        payload = json.loads(report_path.read_text(encoding="utf-8"))
        flat_json = json.dumps(payload, ensure_ascii=False)
        assert expected_value in flat_json
