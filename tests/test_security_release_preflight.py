from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from types import SimpleNamespace
from pathlib import Path

from agi_walker.core.api.release_contracts import (
    build_release_evidence_report,
    write_release_evidence_report,
)
from agi_walker.core.api.security_posture_contracts import (
    build_vulnerability_exception_burndown_report,
    build_vulnerability_exception_report,
    build_security_posture_report,
    write_vulnerability_exception_burndown_report,
    write_security_posture_report,
)


PROJECT_ROOT = Path(__file__).resolve().parent.parent
SECURITY_PREFLIGHT_PATH = PROJECT_ROOT / "tools" / "run_security_release_preflight.py"


def _load_security_preflight_module():
    spec = importlib.util.spec_from_file_location(
        "run_security_release_preflight_module",
        SECURITY_PREFLIGHT_PATH,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _seed_ready_security_posture_report(
    report_path: Path,
    *,
    review_due_count: int = 0,
    review_status: str = "tracked",
) -> None:
    payload = build_security_posture_report(project_root=PROJECT_ROOT)
    payload["posture_status"] = "ready"
    payload["summary"] = "Security posture is ready."
    payload["sbom"] = {
        "path": str(report_path.parent / "sbom.json"),
        "exists": True,
        "component_count": 10,
    }
    payload["vulnerability_reports"] = [
        {
            "name": "python_dependencies",
            "path": str(report_path.parent / "python_vuln_scan_report.json"),
            "required": True,
            "exists": True,
            "status": "passed",
            "scanner": "pip-audit",
        },
        {
            "name": "container_images",
            "path": str(report_path.parent / "container_vuln_scan_report.json"),
            "required": True,
            "exists": True,
            "status": "passed",
            "scanner": "trivy",
        },
    ]
    payload["baseline_documents"] = [
        {
            "name": "security_baseline",
            "path": str(PROJECT_ROOT / "docs" / "guides" / "SECURITY_BASELINE.md"),
            "required": True,
            "exists": True,
        }
    ]
    payload["backup_restore_rehearsal"] = {
        "path": str(report_path.parent / "backup_restore_rehearsal_report.json"),
        "exists": True,
        "status": "passed",
        "rehearsal_duration_seconds": 1.0,
    }
    payload["vulnerability_exception_report"] = {
        "path": str(report_path.parent / "vulnerability_exception_report.json"),
        "exists": True,
        "active_exception_count": 31,
        "expired_exception_count": 0,
        "stale_exception_count": 0,
        "stale_exception_ids": [],
        "review_window_days": 30,
        "review_due_exception_count": review_due_count,
        "next_exception_expiry": "2026-05-15T00:00:00+00:00",
        "review_status": review_status,
    }
    payload["accepted_vulnerability_findings"] = 104
    payload["unresolved_vulnerability_findings"] = 0
    payload["missing_vulnerability_reports"] = 0
    payload["blocked_vulnerability_reports"] = 0
    payload["missing_documents"] = 0
    payload["missing_backup_restore_rehearsal_reports"] = 0
    payload["blocked_backup_restore_rehearsal_reports"] = 0
    payload["next_actions"] = []
    write_security_posture_report(payload, report_path)


def _seed_blocked_security_posture_report(report_path: Path) -> None:
    payload = build_security_posture_report(project_root=PROJECT_ROOT)
    payload["summary"] = "Security posture remains blocked until vulnerability scans pass."
    write_security_posture_report(payload, report_path)


def _seed_exception_review_report(
    report_path: Path,
    *,
    status: str = "passed",
    review_candidate_count: int = 0,
) -> None:
    write_release_evidence_report(
        build_release_evidence_report(
            evidence_name="vulnerability_exception_review",
            status=status,
            summary=(
                "vulnerability_exception_review evidence passed: "
                "31 active exception(s) require review inside the 30-day window "
                "before 2026-05-15T00:00:00+00:00."
                if status == "passed"
                else "vulnerability_exception_review evidence blocked: 2 vulnerability exception(s) are expired."
            ),
            command=(
                "python tools/build_vulnerability_exception_review_report.py "
                "--output test_env/release_evidence/security/"
                "vulnerability_exception_review_report.json"
            ),
            metrics={
                "review_due_exception_count": (
                    review_candidate_count if status == "passed" else 0
                ),
                "review_due_exception_ids": [
                    "webpanel-distributed-libsystemd0-no-fix",
                    "webpanel-distributed-libudev1-no-fix",
                ]
                if status == "passed" and review_candidate_count
                else [],
                "review_due_exception_tickets": (
                    ["SEC-201"] if status == "passed" and review_candidate_count else []
                ),
                "expired_exception_ids": []
                if status == "passed"
                else [
                    "webpanel-distributed-libsystemd0-no-fix",
                    "webpanel-distributed-libudev1-no-fix",
                ],
                "review_follow_up_required": bool(review_candidate_count),
                "review_candidate_count": review_candidate_count
                if status == "passed"
                else 2,
            },
        ),
        report_path,
    )


def _seed_exception_burndown_report(report_path: Path) -> None:
    exception_report = build_vulnerability_exception_report(
        project_root=PROJECT_ROOT,
        generated_at="2026-04-15T12:00:00+00:00",
        exceptions=[
            {
                "id": "webpanel-libsystemd0-no-fix",
                "scope": "container_images",
                "component": "libsystemd0",
                "image_refs": ["deployment-web-panel-distributed"],
                "vulnerability_ids": ["CVE-DEMO-NO-FIX"],
                "severities": ["LOW"],
                "only_without_fix_version": True,
                "justification": "Temporary exception while upstream fix is unavailable.",
                "approved_by": "security-reviewer",
                "approved_at": "2026-04-15T11:00:00+00:00",
                "expires_at": "2026-06-30T00:00:00+00:00",
                "ticket": "SEC-301",
            }
        ],
    )
    write_vulnerability_exception_burndown_report(
        build_vulnerability_exception_burndown_report(
            project_root=PROJECT_ROOT,
            exception_report=exception_report,
            generated_at="2026-04-15T12:00:00+00:00",
        ),
        report_path,
    )


def test_security_release_preflight_passes_with_ready_security_posture(
    tmp_path: Path,
) -> None:
    report_path = tmp_path / "release_evidence" / "security" / "security_posture_report.json"
    evidence_report_path = tmp_path / "release_evidence" / "security_release_preflight_report.json"
    _seed_ready_security_posture_report(report_path)
    _seed_exception_review_report(
        tmp_path
        / "release_evidence"
        / "security"
        / "vulnerability_exception_review_report.json"
    )
    _seed_exception_burndown_report(
        tmp_path
        / "release_evidence"
        / "security"
        / "vulnerability_exception_burndown_report.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_security_release_preflight.py",
            "--skip-collect",
            "--security-posture-report",
            str(report_path),
            "--report-file",
            str(evidence_report_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "security_release_preflight_status=passed" in result.stdout
    assert "security_release_preflight_stale_exceptions=0" in result.stdout
    assert "security_release_preflight_exception_review_status=tracked" in result.stdout
    assert "security_release_preflight_review_report_status=passed" in result.stdout
    assert "security_release_preflight_burndown_status=tracked" in result.stdout
    assert "security_release_preflight_burndown_active=1" in result.stdout
    payload = json.loads(evidence_report_path.read_text(encoding="utf-8"))
    assert payload["status"] == "passed"
    assert payload["evidence_name"] == "security_release_preflight"
    assert payload["metrics"]["stale_vulnerability_exceptions"] == 0
    assert payload["metrics"]["vulnerability_exception_review_status"] == "tracked"
    assert payload["metrics"]["vulnerability_exception_review_due"] == 0
    assert payload["metrics"]["vulnerability_exception_review_report_status"] == "passed"
    assert payload["metrics"]["review_due_vulnerability_exception_ids"] == []
    assert (
        payload["metrics"]["vulnerability_exception_next_expiry"]
        == "2026-05-15T00:00:00+00:00"
    )
    assert payload["metrics"]["vulnerability_exception_burndown_report_status"] == "tracked"
    assert payload["metrics"]["vulnerability_exception_burndown_active"] == 1


def test_security_release_preflight_blocks_when_exception_review_is_due(
    tmp_path: Path,
) -> None:
    report_path = tmp_path / "release_evidence" / "security" / "security_posture_report.json"
    evidence_report_path = tmp_path / "release_evidence" / "security_release_preflight_report.json"
    _seed_ready_security_posture_report(
        report_path,
        review_due_count=31,
        review_status="review_due",
    )
    _seed_exception_review_report(
        tmp_path
        / "release_evidence"
        / "security"
        / "vulnerability_exception_review_report.json",
        review_candidate_count=31,
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_security_release_preflight.py",
            "--skip-collect",
            "--security-posture-report",
            str(report_path),
            "--report-file",
            str(evidence_report_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 1
    assert "security_release_preflight_status=blocked" in result.stdout
    assert "review_due=31" in result.stdout
    payload = json.loads(evidence_report_path.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert payload["metrics"]["vulnerability_exception_review_status"] == "review_due"
    assert payload["metrics"]["vulnerability_exception_review_due"] == 31
    assert payload["metrics"]["vulnerability_exception_review_candidate_count"] == 31


def test_security_release_preflight_forwards_security_only_collect_profile(
    tmp_path: Path, monkeypatch
) -> None:
    module = _load_security_preflight_module()
    captured: dict[str, object] = {}

    def fake_run(command, **kwargs):
        captured["command"] = list(command)
        captured["cwd"] = kwargs.get("cwd")
        return SimpleNamespace(returncode=0, stdout="release_evidence_status=passed\n", stderr="")

    monkeypatch.setattr(module.subprocess, "run", fake_run)
    args = SimpleNamespace(
        python_vuln_report_source=None,
        python_vuln_raw_report=None,
        python_vuln_raw_format="pip-audit-json",
        python_vuln_command="pip-audit --format json",
        python_vuln_raw_output=None,
        container_vuln_report_source=None,
        container_vuln_raw_report=None,
        container_vuln_raw_format="trivy-json",
        container_vuln_command="trivy image --scanners vuln --timeout 15m --format json",
        container_vuln_raw_output_dir=None,
        vulnerability_exception_report_source=None,
        vulnerability_exception_input_source="deployment/security/vulnerability_exceptions.input.json",
        run_python_vuln_scan=True,
        run_container_vuln_scan=True,
        container_image_ref=["deployment-web-panel-distributed"],
        security_only=True,
    )

    code, summary = module._run_collect_release_evidence(
        args,
        tmp_path / "release_evidence",
    )

    assert code == 0
    assert "release_evidence_status=passed" in summary
    command = captured["command"]
    assert isinstance(command, list)
    assert "tools/collect_release_evidence.py" in command
    assert "--security-only" in command
    assert "--run-python-vuln-scan" in command
    assert "--run-container-vuln-scan" in command


def test_security_release_preflight_blocks_with_blocked_security_posture(
    tmp_path: Path,
) -> None:
    report_path = tmp_path / "release_evidence" / "security" / "security_posture_report.json"
    evidence_report_path = tmp_path / "release_evidence" / "security_release_preflight_report.json"
    _seed_blocked_security_posture_report(report_path)

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_security_release_preflight.py",
            "--skip-collect",
            "--security-posture-report",
            str(report_path),
            "--report-file",
            str(evidence_report_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 1
    assert "security_release_preflight_status=blocked" in result.stdout
    payload = json.loads(evidence_report_path.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"


def test_security_release_preflight_surfaces_stale_exception_metrics_when_blocked(
    tmp_path: Path,
) -> None:
    report_path = tmp_path / "release_evidence" / "security" / "security_posture_report.json"
    evidence_report_path = tmp_path / "release_evidence" / "security_release_preflight_report.json"
    _seed_blocked_security_posture_report(report_path)

    payload = json.loads(report_path.read_text(encoding="utf-8"))
    payload["summary"] = (
        "Security posture remains blocked because some active no-fix vulnerability "
        "exceptions are stale and the matching findings now advertise fix versions."
    )
    payload["vulnerability_exception_report"] = {
        "path": str(report_path.parent / "vulnerability_exception_report.json"),
        "exists": True,
        "active_exception_count": 31,
        "expired_exception_count": 0,
        "stale_exception_count": 2,
        "stale_exception_ids": [
            "webpanel-distributed-libsystemd0-no-fix",
            "webpanel-distributed-libudev1-no-fix",
        ],
        "review_window_days": 30,
        "review_due_exception_count": 0,
        "next_exception_expiry": "2026-05-15T00:00:00+00:00",
        "review_status": "tracked",
    }
    write_security_posture_report(payload, report_path)
    _seed_exception_review_report(
        tmp_path
        / "release_evidence"
        / "security"
        / "vulnerability_exception_review_report.json",
        status="blocked",
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_security_release_preflight.py",
            "--skip-collect",
            "--security-posture-report",
            str(report_path),
            "--report-file",
            str(evidence_report_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 1
    assert "security_release_preflight_stale_exceptions=2" in result.stdout
    payload = json.loads(evidence_report_path.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert payload["metrics"]["stale_vulnerability_exceptions"] == 2
    assert payload["metrics"]["stale_vulnerability_exception_ids"] == [
        "webpanel-distributed-libsystemd0-no-fix",
        "webpanel-distributed-libudev1-no-fix",
    ]
    assert payload["metrics"]["vulnerability_exception_review_report_status"] == "blocked"


def test_security_release_preflight_blocks_when_report_is_missing(tmp_path: Path) -> None:
    report_path = tmp_path / "release_evidence" / "security" / "security_posture_report.json"
    evidence_report_path = tmp_path / "release_evidence" / "security_release_preflight_report.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_security_release_preflight.py",
            "--skip-collect",
            "--security-posture-report",
            str(report_path),
            "--report-file",
            str(evidence_report_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 1
    assert "security_release_preflight_status=blocked" in result.stdout
    payload = json.loads(evidence_report_path.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert "missing" in payload["summary"]
