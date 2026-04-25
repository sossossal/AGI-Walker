import json
from pathlib import Path

from agi_walker.core.api.release_contracts import (
    build_release_evidence_report,
    build_release_manifest_artifact,
    validate_release_evidence_report,
    write_release_evidence_report,
)


def _clean_checkout_smoke_report() -> dict[str, object]:
    return {
        "schema_version": "1.0",
        "artifact_type": "clean_checkout_smoke_report",
        "status": "passed",
        "generated_at": "2026-04-13T10:05:00+00:00",
        "source_root": "D:/repo",
        "checkout_root": "D:/repo/test_env/clean_checkout_smoke/checkout",
        "output_root": "D:/repo/test_env/clean_checkout_smoke",
        "version": "2026.04.12",
        "tag": "2026.04.12",
        "runs": 2,
        "command_template": [
            "python",
            "tests/run_smoke_tests.py",
            "--output-root",
            "{run_output_root}",
        ],
        "checkout_commit_sha": "commit-1",
        "seeded_evidence_paths": [
            "test_env/release_evidence/non_live_gate_report.json",
        ],
        "checks": [
            {"name": "workspace_snapshot_copied", "status": "pass", "detail": "copied"},
            {
                "name": "clean_checkout_initialized",
                "status": "pass",
                "detail": "initialized",
            },
            {
                "name": "sequential_smoke_runs_are_clean",
                "status": "pass",
                "detail": "2 sequential smoke run(s) completed with empty git status after each run",
            },
        ],
        "run_reports": [
            {
                "run_index": 1,
                "status": "passed",
                "command": ["python", "tests/run_smoke_tests.py"],
                "run_output_root": "run_01",
                "exit_code": 0,
                "stdout_path": "logs/run_01.stdout.txt",
                "stderr_path": "logs/run_01.stderr.txt",
                "worktree_clean": True,
                "dirty_paths": [],
            },
            {
                "run_index": 2,
                "status": "passed",
                "command": ["python", "tests/run_smoke_tests.py"],
                "run_output_root": "run_02",
                "exit_code": 0,
                "stdout_path": "logs/run_02.stdout.txt",
                "stderr_path": "logs/run_02.stderr.txt",
                "worktree_clean": True,
                "dirty_paths": [],
            },
        ],
        "failure_reason": None,
    }


def test_release_evidence_report_accepts_canonical_payload(tmp_path: Path) -> None:
    payload = build_release_evidence_report(
        evidence_name="non_live_gate",
        status="passed",
        summary="non_live_gate pytest evidence passed: 795 passed, 3 skipped, 3 deselected.",
        command='python -m pytest -m "not live" -q',
        generated_at="2026-04-13T10:00:00+00:00",
        exit_code=0,
        duration_seconds=12.34,
        metrics={"passed": 795, "skipped": 3, "deselected": 3},
        source_commit_sha="abc123",
    )

    assert validate_release_evidence_report(payload) == []
    output_path = write_release_evidence_report(payload, tmp_path / "evidence.json")
    saved = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_release_evidence_report(saved) == []
    assert saved["artifact_type"] == "release_evidence_report"
    assert saved["metrics"]["passed"] == 795


def test_release_manifest_prefers_structured_required_evidence_reports(
    tmp_path: Path,
) -> None:
    report_root = tmp_path / "test_env" / "release_evidence"
    report_root.mkdir(parents=True, exist_ok=True)
    reports = {
        "clean_checkout_smoke_report.json": _clean_checkout_smoke_report(),
        "non_live_gate_report.json": build_release_evidence_report(
            evidence_name="non_live_gate",
            status="passed",
            summary="non_live_gate pytest evidence passed: 795 passed, 3 skipped, 3 deselected.",
            command='python -m pytest -m "not live" -q',
            generated_at="2026-04-13T10:01:00+00:00",
            metrics={"passed": 795, "skipped": 3, "deselected": 3},
            source_commit_sha="commit-1",
        ),
        "release_contracts_and_capability_matrix_report.json": build_release_evidence_report(
            evidence_name="release_contracts_and_capability_matrix",
            status="passed",
            summary="release_contracts_and_capability_matrix pytest evidence passed: 42 passed.",
            command="python -m pytest tests/test_release_contracts.py -q",
            generated_at="2026-04-13T10:02:00+00:00",
            metrics={"passed": 42},
            source_commit_sha="commit-1",
        ),
    }
    for name, payload in reports.items():
        report_path = report_root / name
        if payload.get("artifact_type") == "release_evidence_report":
            write_release_evidence_report(payload, report_path)
        else:
            report_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )

    manifest = build_release_manifest_artifact(
        build_id="build-20260413-001",
        version="2026.04.13-rc1",
        channel="rc",
        release_summary="release evidence attestation test",
        project_root=tmp_path,
        source_root=tmp_path,
    )

    evidence_by_name = {item["name"]: item for item in manifest["test_evidence"]}
    assert evidence_by_name["clean_checkout_smoke"]["attested"] is True
    assert evidence_by_name["clean_checkout_smoke"]["metrics"]["runs"] == 2
    assert evidence_by_name["non_live_gate"]["attested"] is True
    assert (
        evidence_by_name["non_live_gate"]["summary"]
        == "non_live_gate pytest evidence passed: 795 passed, 3 skipped, 3 deselected."
    )
    assert evidence_by_name["release_contracts_and_capability_matrix"]["attested"] is True


def test_release_manifest_blocks_mismatched_structured_evidence_name(
    tmp_path: Path,
) -> None:
    report_root = tmp_path / "test_env" / "release_evidence"
    report_root.mkdir(parents=True, exist_ok=True)
    write_release_evidence_report(
        build_release_evidence_report(
            evidence_name="wrong_name",
            status="passed",
            summary="wrong report",
            command="python -m pytest tests/test_docs_utf8.py -q",
            generated_at="2026-04-13T10:03:00+00:00",
        ),
        report_root / "non_live_gate_report.json",
    )

    manifest = build_release_manifest_artifact(
        build_id="build-20260413-002",
        version="2026.04.13-rc1",
        channel="rc",
        release_summary="release evidence mismatch test",
        project_root=tmp_path,
        source_root=tmp_path,
    )

    evidence_by_name = {item["name"]: item for item in manifest["test_evidence"]}
    assert evidence_by_name["non_live_gate"]["status"] == "blocked"
    assert evidence_by_name["non_live_gate"]["attested"] is False
