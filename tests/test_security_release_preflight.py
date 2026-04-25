from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from agi_walker.core.api.release_contracts import (
    build_release_evidence_report,
    write_release_evidence_report,
)
from agi_walker.core.api.security_posture_contracts import (
    build_security_posture_report,
    write_security_posture_report,
)


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _seed_ready_security_posture_report(report_path: Path) -> None:
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
        "review_due_exception_count": 31,
        "next_exception_expiry": "2026-05-15T00:00:00+00:00",
        "review_status": "review_due",
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


def _seed_exception_review_report(report_path: Path, *, status: str = "passed") -> None:
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
                "review_due_exception_count": 31 if status == "passed" else 0,
                "review_due_exception_ids": [
                    "webpanel-distributed-libsystemd0-no-fix",
                    "webpanel-distributed-libudev1-no-fix",
                ]
                if status == "passed"
                else [],
                "review_due_exception_tickets": ["SEC-201"] if status == "passed" else [],
                "expired_exception_ids": []
                if status == "passed"
                else [
                    "webpanel-distributed-libsystemd0-no-fix",
                    "webpanel-distributed-libudev1-no-fix",
                ],
                "review_follow_up_required": True,
                "review_candidate_count": 31 if status == "passed" else 2,
            },
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
    assert "security_release_preflight_exception_review_status=review_due" in result.stdout
    assert "security_release_preflight_review_report_status=passed" in result.stdout
    payload = json.loads(evidence_report_path.read_text(encoding="utf-8"))
    assert payload["status"] == "passed"
    assert payload["evidence_name"] == "security_release_preflight"
    assert payload["metrics"]["stale_vulnerability_exceptions"] == 0
    assert payload["metrics"]["vulnerability_exception_review_status"] == "review_due"
    assert payload["metrics"]["vulnerability_exception_review_due"] == 31
    assert payload["metrics"]["vulnerability_exception_review_report_status"] == "passed"
    assert payload["metrics"]["review_due_vulnerability_exception_ids"] == [
        "webpanel-distributed-libsystemd0-no-fix",
        "webpanel-distributed-libudev1-no-fix",
    ]
    assert (
        payload["metrics"]["vulnerability_exception_next_expiry"]
        == "2026-05-15T00:00:00+00:00"
    )


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
