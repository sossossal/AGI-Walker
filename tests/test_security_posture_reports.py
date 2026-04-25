from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path

from agi_walker.core.api.security_posture_contracts import (
    build_backup_restore_rehearsal_report,
    build_sbom_artifact,
    build_security_posture_report,
    build_vulnerability_exception_report,
    build_vulnerability_remediation_report,
    build_vulnerability_scan_report,
    build_vulnerability_scan_report_from_raw,
    validate_backup_restore_rehearsal_report,
    validate_sbom_artifact,
    validate_security_posture_report,
    validate_vulnerability_exception_report,
    validate_vulnerability_scan_report,
    write_backup_restore_rehearsal_report,
    write_sbom_artifact,
    write_vulnerability_exception_report,
    write_vulnerability_remediation_report,
    write_vulnerability_scan_report,
)


PROJECT_ROOT = Path(__file__).resolve().parent.parent
FIXTURES_ROOT = PROJECT_ROOT / "tests" / "fixtures"


def _seed_project_root(project_root: Path, *, include_docs: bool = False) -> None:
    (project_root / "deployment").mkdir(parents=True, exist_ok=True)
    (project_root / "docs" / "guides").mkdir(parents=True, exist_ok=True)
    (project_root / "pyproject.toml").write_text(
        """
[project]
name = "agi-walker-test"
version = "0.1.0"
dependencies = [
  "fastapi>=0.110",
  "uvicorn>=0.29",
]

[project.optional-dependencies]
distributed = ["zenoh>=1.0"]
""".strip()
        + "\n",
        encoding="utf-8",
    )
    (project_root / "deployment" / "requirements.common.txt").write_text(
        "aiosqlite>=0.19\n",
        encoding="utf-8",
    )
    (project_root / "deployment" / "requirements.web_panel.txt").write_text(
        "-r requirements.common.txt\nsqlalchemy>=2.0\n",
        encoding="utf-8",
    )
    (project_root / "deployment" / "requirements.web_panel.distributed.txt").write_text(
        "-r requirements.common.txt\nzenoh>=1.0\n",
        encoding="utf-8",
    )
    (project_root / "deployment" / "requirements.distributed_runtime.txt").write_text(
        "psutil>=5.9\npyyaml>=6.0\n",
        encoding="utf-8",
    )

    if include_docs:
        docs = {
            "SECURITY_BASELINE.md": "# Security Baseline\n",
            "AUDIT_TRAIL_POLICY.md": "# Audit Trail Policy\n",
            "BACKUP_RESTORE_RUNBOOK.md": "# Backup Restore Runbook\n",
            "INCIDENT_RESPONSE_MATRIX.md": "# Incident Response Matrix\n",
        }
        for name, content in docs.items():
            (project_root / "docs" / "guides" / name).write_text(
                content,
                encoding="utf-8",
            )
    (project_root / "deployment" / "compose.env.example").write_text(
        "AGI_WALKER_RUNTIME_ROOT=./runtime\n",
        encoding="utf-8",
    )
    (project_root / "deployment" / "web_panel.env.example").write_text(
        "AGI_WALKER_WEB_PORT=8000\n",
        encoding="utf-8",
    )


def _seed_backup_restore_rehearsal(project_root: Path) -> Path:
    security_root = project_root / "test_env" / "security"
    source_runtime_root = security_root / "backup_restore_rehearsal" / "source_runtime"
    source_config_root = security_root / "backup_restore_rehearsal" / "source_config"
    backup_snapshot_root = (
        security_root / "backup_restore_rehearsal" / "backup_snapshot"
    )
    restored_runtime_root = (
        security_root / "backup_restore_rehearsal" / "restored_runtime"
    )
    restored_config_root = (
        security_root / "backup_restore_rehearsal" / "restored_config"
    )

    for path in [
        source_runtime_root / "db",
        source_runtime_root / "workflow_runs" / "run-001",
        source_runtime_root / "workflow_archive" / "archive-001",
        source_runtime_root / "backups",
        source_config_root,
        backup_snapshot_root,
        restored_runtime_root,
        restored_config_root,
    ]:
        path.mkdir(parents=True, exist_ok=True)

    (source_runtime_root / "db" / "app.sqlite3").write_text("db\n", encoding="utf-8")
    (source_runtime_root / "workflow_runs" / "run-001" / "result.json").write_text(
        '{"status":"passed"}\n',
        encoding="utf-8",
    )
    (
        source_runtime_root / "workflow_archive" / "archive-001" / "manifest.json"
    ).write_text('{"archived":true}\n', encoding="utf-8")
    (source_runtime_root / "backups" / "README.txt").write_text(
        "backup seed\n", encoding="utf-8"
    )
    (source_config_root / "compose.env").write_text(
        "AGI_WALKER_RUNTIME_ROOT=./runtime\n", encoding="utf-8"
    )
    (source_config_root / "web_panel.env").write_text(
        "AGI_WALKER_WEB_PORT=8000\n", encoding="utf-8"
    )

    shutil.copytree(source_runtime_root / "db", backup_snapshot_root / "db")
    shutil.copytree(
        source_runtime_root / "workflow_runs",
        backup_snapshot_root / "workflow_runs",
    )
    shutil.copytree(
        source_runtime_root / "workflow_archive",
        backup_snapshot_root / "workflow_archive",
    )
    shutil.copytree(source_runtime_root / "backups", backup_snapshot_root / "backups")
    shutil.copyfile(source_config_root / "compose.env", backup_snapshot_root / "compose.env")
    shutil.copyfile(
        source_config_root / "web_panel.env",
        backup_snapshot_root / "web_panel.env",
    )

    shutil.copytree(backup_snapshot_root / "db", restored_runtime_root / "db")
    shutil.copytree(
        backup_snapshot_root / "workflow_runs",
        restored_runtime_root / "workflow_runs",
    )
    shutil.copytree(
        backup_snapshot_root / "workflow_archive",
        restored_runtime_root / "workflow_archive",
    )
    shutil.copytree(backup_snapshot_root / "backups", restored_runtime_root / "backups")
    shutil.copyfile(backup_snapshot_root / "compose.env", restored_config_root / "compose.env")
    shutil.copyfile(
        backup_snapshot_root / "web_panel.env",
        restored_config_root / "web_panel.env",
    )

    payload = build_backup_restore_rehearsal_report(
        project_root=project_root,
        actor="test-runner",
        source_runtime_root=source_runtime_root,
        source_config_root=source_config_root,
        backup_snapshot_root=backup_snapshot_root,
        restored_runtime_root=restored_runtime_root,
        restored_config_root=restored_config_root,
        rehearsal_duration_seconds=1.25,
    )
    assert validate_backup_restore_rehearsal_report(payload) == []
    output_path = (
        security_root / "backup_restore_rehearsal_report.json"
    )
    return write_backup_restore_rehearsal_report(payload, output_path)


def _seed_vulnerability_exception_report(project_root: Path) -> Path:
    security_root = project_root / "test_env" / "security"
    payload = build_vulnerability_exception_report(
        project_root=project_root,
        generated_at="2026-04-15T12:00:00+00:00",
        exceptions=[
            {
                "id": "container-libsystemd0-no-fix",
                "scope": "container_images",
                "component": "libsystemd0",
                "image_refs": ["deployment-web-panel-distributed"],
                "only_without_fix_version": True,
                "justification": "Upstream vendor has not published a fixed package yet.",
                "approved_by": "security-reviewer",
                "approved_at": "2026-04-15T11:00:00+00:00",
                "expires_at": "2026-05-15T00:00:00+00:00",
                "ticket": "SEC-101",
            }
        ],
    )
    assert validate_vulnerability_exception_report(payload) == []
    output_path = security_root / "vulnerability_exception_report.json"
    return write_vulnerability_exception_report(payload, output_path)


def test_build_sbom_artifact_collects_pyproject_and_requirements(
    tmp_path: Path,
) -> None:
    _seed_project_root(tmp_path)

    payload = build_sbom_artifact(project_root=tmp_path)
    dependency_sources = {Path(item).as_posix() for item in payload["dependency_sources"]}

    assert validate_sbom_artifact(payload) == []
    assert payload["project_name"] == "agi-walker-test"
    assert payload["project_version"] == "0.1.0"
    assert "pyproject.toml" in dependency_sources
    assert "deployment/requirements.web_panel.txt" in dependency_sources
    assert "deployment/requirements.common.txt" in dependency_sources
    assert any(item["name"] == "fastapi" for item in payload["components"])
    assert any(item["name"] == "sqlalchemy" for item in payload["components"])
    assert any(item["name"] == "aiosqlite" for item in payload["components"])

    output_path = write_sbom_artifact(payload, tmp_path / "test_env" / "security" / "sbom.json")
    saved = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_sbom_artifact(saved) == []
    assert saved["component_count"] == len(saved["components"])


def test_vulnerability_scan_report_round_trip(tmp_path: Path) -> None:
    payload = build_vulnerability_scan_report(
        scan_name="python_dependencies",
        target="pyproject.toml",
        status="passed",
        summary="Python dependency review completed.",
        command="pip-audit --requirement ...",
        scanner="manual-review",
    )

    assert validate_vulnerability_scan_report(payload) == []
    output_path = write_vulnerability_scan_report(
        payload,
        tmp_path / "test_env" / "security" / "python_vuln_scan_report.json",
    )
    saved = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_vulnerability_scan_report(saved) == []
    assert saved["status"] == "passed"


def test_vulnerability_scan_report_can_normalize_pip_audit_json() -> None:
    payload = build_vulnerability_scan_report_from_raw(
        scan_name="python_dependencies",
        target="pyproject.toml",
        raw_report_path=FIXTURES_ROOT / "pip_audit_clean_report.json",
        raw_format="pip-audit-json",
        command="pip-audit --format json",
    )

    assert validate_vulnerability_scan_report(payload) == []
    assert payload["status"] == "passed"
    assert payload["scanner"] == "pip-audit"
    assert payload["report_format"] == "pip-audit-json"
    assert payload["finding_count"] == 0
    assert payload["affected_component_count"] == 0


def test_vulnerability_scan_report_can_normalize_trivy_json() -> None:
    payload = build_vulnerability_scan_report_from_raw(
        scan_name="container_images",
        target="deployment/docker-compose.yml",
        raw_report_path=FIXTURES_ROOT / "trivy_clean_report.json",
        raw_format="trivy-json",
        command="trivy image --format json",
    )

    assert validate_vulnerability_scan_report(payload) == []
    assert payload["status"] == "passed"
    assert payload["scanner"] == "trivy"
    assert payload["report_format"] == "trivy-json"
    assert payload["finding_count"] == 0
    assert payload["affected_component_count"] == 0


def test_security_posture_report_blocks_when_reports_and_docs_are_missing(
    tmp_path: Path,
) -> None:
    _seed_project_root(tmp_path, include_docs=False)

    posture = build_security_posture_report(project_root=tmp_path)

    assert validate_security_posture_report(posture) == []
    assert posture["posture_status"] == "blocked"
    assert posture["sbom"]["exists"] is False
    assert posture["missing_vulnerability_reports"] == 2
    assert posture["blocked_vulnerability_execution_reports"] == 0
    assert posture["blocked_vulnerability_finding_reports"] == 0
    assert posture["missing_documents"] == 4
    assert posture["missing_backup_restore_rehearsal_reports"] == 1
    assert len(posture["next_actions"]) >= 3


def test_security_posture_report_is_ready_when_sbom_reports_and_docs_exist(
    tmp_path: Path,
) -> None:
    _seed_project_root(tmp_path, include_docs=True)

    security_root = tmp_path / "test_env" / "security"
    sbom_payload = build_sbom_artifact(project_root=tmp_path)
    write_sbom_artifact(sbom_payload, security_root / "sbom.json")
    write_vulnerability_scan_report(
        build_vulnerability_scan_report(
            scan_name="python_dependencies",
            target="pyproject.toml",
            status="passed",
            summary="Python dependency review completed.",
            command="manual placeholder",
            scanner="manual-review",
        ),
        security_root / "python_vuln_scan_report.json",
    )
    write_vulnerability_scan_report(
        build_vulnerability_scan_report(
            scan_name="container_images",
            target="deployment/docker-compose.yml",
            status="passed",
            summary="Container image review completed.",
            command="manual placeholder",
            scanner="manual-review",
        ),
        security_root / "container_vuln_scan_report.json",
    )
    _seed_backup_restore_rehearsal(tmp_path)

    posture = build_security_posture_report(project_root=tmp_path)

    assert validate_security_posture_report(posture) == []
    assert posture["posture_status"] == "ready"
    assert posture["sbom"]["exists"] is True
    assert posture["missing_vulnerability_reports"] == 0
    assert posture["blocked_vulnerability_reports"] == 0
    assert posture["blocked_vulnerability_execution_reports"] == 0
    assert posture["blocked_vulnerability_finding_reports"] == 0
    assert posture["missing_documents"] == 0
    assert posture["missing_backup_restore_rehearsal_reports"] == 0
    assert posture["blocked_backup_restore_rehearsal_reports"] == 0
    assert posture["next_actions"] == []


def test_security_posture_report_distinguishes_scan_execution_from_findings(
    tmp_path: Path,
) -> None:
    _seed_project_root(tmp_path, include_docs=True)

    security_root = tmp_path / "test_env" / "security"
    write_sbom_artifact(build_sbom_artifact(project_root=tmp_path), security_root / "sbom.json")
    write_vulnerability_scan_report(
        build_vulnerability_scan_report(
            scan_name="python_dependencies",
            target="pyproject.toml",
            status="blocked",
            summary="pip-audit execution failed: docker permission denied",
            command="pip-audit --format json",
            scanner="pip-audit",
            finding_count=0,
            affected_component_count=0,
        ),
        security_root / "python_vuln_scan_report.json",
    )
    write_vulnerability_scan_report(
        build_vulnerability_scan_report(
            scan_name="container_images",
            target="deployment/docker-compose.yml",
            status="blocked",
            summary="trivy reported 4 finding(s) across 2 affected component(s).",
            command="trivy image --format json",
            scanner="trivy",
            finding_count=4,
            affected_component_count=2,
        ),
        security_root / "container_vuln_scan_report.json",
    )
    _seed_backup_restore_rehearsal(tmp_path)

    posture = build_security_posture_report(project_root=tmp_path)

    assert validate_security_posture_report(posture) == []
    assert posture["posture_status"] == "blocked"
    assert posture["blocked_vulnerability_reports"] == 2
    assert posture["blocked_vulnerability_execution_reports"] == 1
    assert posture["blocked_vulnerability_finding_reports"] == 1
    assert "execute successfully" in posture["summary"]
    assert "reported findings are remediated" in posture["summary"]
    assert any("re-run the blocked vulnerability scanners" in item for item in posture["next_actions"])
    assert any("resolve reported vulnerability scan findings" in item for item in posture["next_actions"])


def test_security_posture_report_can_be_ready_with_active_vulnerability_exceptions(
    tmp_path: Path,
) -> None:
    _seed_project_root(tmp_path, include_docs=True)

    security_root = tmp_path / "test_env" / "security"
    write_sbom_artifact(build_sbom_artifact(project_root=tmp_path), security_root / "sbom.json")
    write_vulnerability_scan_report(
        build_vulnerability_scan_report(
            scan_name="python_dependencies",
            target="pyproject.toml",
            status="passed",
            summary="python clean",
            command="pip-audit --format json",
            scanner="pip-audit",
            finding_count=0,
            affected_component_count=0,
        ),
        security_root / "python_vuln_scan_report.json",
    )
    container_raw_root = security_root / "container_vuln_scan_report_raw"
    container_raw_root.mkdir(parents=True, exist_ok=True)
    (container_raw_root / "deployment-web-panel-distributed.json").write_text(
        json.dumps(
            {
                "ArtifactName": "deployment-web-panel-distributed",
                "Results": [
                    {
                        "Target": "debian:stable",
                        "Vulnerabilities": [
                            {
                                "PkgName": "libsystemd0",
                                "InstalledVersion": "257.8-1",
                                "FixedVersion": "",
                                "VulnerabilityID": "CVE-1",
                                "Severity": "HIGH",
                            }
                        ],
                    }
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    write_vulnerability_scan_report(
        build_vulnerability_scan_report(
            scan_name="container_images",
            target="deployment/docker-compose.yml",
            status="blocked",
            summary="trivy reported 1 finding(s) across 1 affected component(s).",
            command="trivy image --format json",
            scanner="trivy",
            report_format="trivy-json",
            raw_report_path=str(container_raw_root.relative_to(tmp_path)),
            finding_count=1,
            affected_component_count=1,
        ),
        security_root / "container_vuln_scan_report.json",
    )
    _seed_backup_restore_rehearsal(tmp_path)
    exception_report_path = _seed_vulnerability_exception_report(tmp_path)
    remediation_payload = build_vulnerability_remediation_report(
        project_root=tmp_path,
        python_vuln_report_path=security_root / "python_vuln_scan_report.json",
        container_vuln_report_path=security_root / "container_vuln_scan_report.json",
        vulnerability_exception_report_path=exception_report_path,
    )
    remediation_path = security_root / "vulnerability_remediation_report.json"
    write_vulnerability_remediation_report(remediation_payload, remediation_path)

    posture = build_security_posture_report(
        project_root=tmp_path,
        vulnerability_remediation_report_path=remediation_path,
        vulnerability_exception_report_path=exception_report_path,
    )

    assert validate_security_posture_report(posture) == []
    assert posture["posture_status"] == "ready"
    assert posture["blocked_vulnerability_reports"] == 0
    assert posture["accepted_vulnerability_findings"] == 1
    assert posture["unresolved_vulnerability_findings"] == 0
    assert posture["vulnerability_remediation"]["status"] == "ready"
    assert posture["vulnerability_remediation"]["stale_exception_count"] == 0
    assert posture["vulnerability_exception_report"]["active_exception_count"] == 1
    assert posture["vulnerability_exception_report"]["review_due_exception_count"] == 1
    assert posture["vulnerability_exception_report"]["review_due_exception_ids"] == [
        "container-libsystemd0-no-fix"
    ]
    assert posture["vulnerability_exception_report"][
        "review_due_exception_tickets"
    ] == ["SEC-101"]
    assert posture["vulnerability_exception_report"]["next_exception_expiry"] == "2026-05-15T00:00:00+00:00"
    assert posture["vulnerability_exception_report"]["review_status"] == "review_due"
    assert posture["vulnerability_exception_report"]["stale_exception_count"] == 0
    assert posture["vulnerability_exception_report"]["stale_exception_ids"] == []
    assert "30-day review window before 2026-05-15T00:00:00+00:00" in posture["summary"]
    assert any(
        "review approved vulnerability exceptions" in item
        for item in posture["next_actions"]
    )


def test_security_posture_report_surfaces_stale_no_fix_exceptions(
    tmp_path: Path,
) -> None:
    _seed_project_root(tmp_path, include_docs=True)

    security_root = tmp_path / "test_env" / "security"
    write_sbom_artifact(build_sbom_artifact(project_root=tmp_path), security_root / "sbom.json")
    write_vulnerability_scan_report(
        build_vulnerability_scan_report(
            scan_name="python_dependencies",
            target="pyproject.toml",
            status="passed",
            summary="python clean",
            command="pip-audit --format json",
            scanner="pip-audit",
            finding_count=0,
            affected_component_count=0,
        ),
        security_root / "python_vuln_scan_report.json",
    )
    container_raw_root = security_root / "container_vuln_scan_report_raw"
    container_raw_root.mkdir(parents=True, exist_ok=True)
    (container_raw_root / "deployment-web-panel-distributed.json").write_text(
        json.dumps(
            {
                "ArtifactName": "deployment-web-panel-distributed",
                "Results": [
                    {
                        "Target": "debian:stable",
                        "Vulnerabilities": [
                            {
                                "PkgName": "libsystemd0",
                                "InstalledVersion": "257.8-1",
                                "FixedVersion": "257.11-1",
                                "VulnerabilityID": "CVE-1",
                                "Severity": "HIGH",
                            }
                        ],
                    }
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    write_vulnerability_scan_report(
        build_vulnerability_scan_report(
            scan_name="container_images",
            target="deployment/docker-compose.yml",
            status="blocked",
            summary="trivy reported 1 finding(s) across 1 affected component(s).",
            command="trivy image --format json",
            scanner="trivy",
            report_format="trivy-json",
            raw_report_path=str(container_raw_root.relative_to(tmp_path)),
            finding_count=1,
            affected_component_count=1,
        ),
        security_root / "container_vuln_scan_report.json",
    )
    _seed_backup_restore_rehearsal(tmp_path)
    exception_report_path = _seed_vulnerability_exception_report(tmp_path)
    remediation_payload = build_vulnerability_remediation_report(
        project_root=tmp_path,
        python_vuln_report_path=security_root / "python_vuln_scan_report.json",
        container_vuln_report_path=security_root / "container_vuln_scan_report.json",
        vulnerability_exception_report_path=exception_report_path,
    )
    remediation_path = security_root / "vulnerability_remediation_report.json"
    write_vulnerability_remediation_report(remediation_payload, remediation_path)

    posture = build_security_posture_report(
        project_root=tmp_path,
        vulnerability_remediation_report_path=remediation_path,
        vulnerability_exception_report_path=exception_report_path,
    )

    assert validate_security_posture_report(posture) == []
    assert posture["posture_status"] == "blocked"
    assert posture["blocked_vulnerability_reports"] == 1
    assert posture["blocked_vulnerability_finding_reports"] == 1
    assert posture["vulnerability_remediation"]["status"] == "needs_remediation"
    assert posture["vulnerability_remediation"]["stale_exception_count"] == 1
    assert posture["vulnerability_exception_report"]["stale_exception_count"] == 1
    assert posture["vulnerability_exception_report"]["stale_exception_ids"] == [
        "container-libsystemd0-no-fix"
    ]
    assert posture["vulnerability_exception_report"]["review_due_exception_ids"] == [
        "container-libsystemd0-no-fix"
    ]
    assert "stale" in posture["summary"]
    assert any(
        "stale no-fix vulnerability exceptions" in item
        for item in posture["next_actions"]
    )


def test_security_posture_cli_tools_write_expected_artifacts(tmp_path: Path) -> None:
    _seed_project_root(tmp_path, include_docs=True)
    security_root = tmp_path / "test_env" / "security"
    sbom_path = security_root / "sbom.json"
    python_report_path = security_root / "python_vuln_scan_report.json"
    container_report_path = security_root / "container_vuln_scan_report.json"
    backup_restore_report_path = security_root / "backup_restore_rehearsal_report.json"
    posture_path = security_root / "security_posture_report.json"

    commands = [
        [
            sys.executable,
            "tools/build_sbom_artifact.py",
            "--project-root",
            str(tmp_path),
            "--output",
            str(sbom_path),
        ],
        [
            sys.executable,
            "tools/write_vulnerability_scan_report.py",
            "--scan-name",
            "python_dependencies",
            "--target",
            "pyproject.toml",
            "--status",
            "passed",
            "--scanner",
            "manual-review",
            "--summary",
            "Python dependency review completed.",
            "--command",
            "manual placeholder",
            "--output",
            str(python_report_path),
        ],
        [
            sys.executable,
            "tools/write_vulnerability_scan_report.py",
            "--scan-name",
            "container_images",
            "--target",
            "deployment/docker-compose.yml",
            "--status",
            "passed",
            "--scanner",
            "manual-review",
            "--summary",
            "Container image review completed.",
            "--command",
            "manual placeholder",
            "--output",
            str(container_report_path),
        ],
        [
            sys.executable,
            "tools/run_backup_restore_rehearsal.py",
            "--project-root",
            str(tmp_path),
            "--output-root",
            str(security_root / "backup_restore_rehearsal"),
            "--report-file",
            str(backup_restore_report_path),
        ],
        [
            sys.executable,
            "tools/build_security_posture_report.py",
            "--project-root",
            str(tmp_path),
            "--sbom",
            str(sbom_path),
            "--python-vuln-report",
            str(python_report_path),
            "--container-vuln-report",
            str(container_report_path),
            "--backup-restore-report",
            str(backup_restore_report_path),
            "--output",
            str(posture_path),
        ],
    ]

    expected_tokens = [
        "sbom_written=",
        "vulnerability_scan_report_written=",
        "vulnerability_scan_report_written=",
        "backup_restore_rehearsal_status=passed",
        "security_posture_status=ready",
    ]

    for command, token in zip(commands, expected_tokens, strict=True):
        result = subprocess.run(
            command,
            cwd=str(PROJECT_ROOT),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=False,
        )
        assert result.returncode == 0, result.stderr
        assert token in result.stdout

    posture_payload = json.loads(posture_path.read_text(encoding="utf-8"))
    assert validate_security_posture_report(posture_payload) == []
    assert posture_payload["posture_status"] == "ready"
    assert posture_payload["backup_restore_rehearsal"]["status"] == "passed"
    assert posture_payload["vulnerability_exception_report"]["exists"] is False
    assert posture_payload["vulnerability_exception_report"]["stale_exception_count"] == 0
    assert posture_payload["vulnerability_exception_report"]["stale_exception_ids"] == []
    assert "review_due_exception_count" not in posture_payload["vulnerability_exception_report"]


def test_vulnerability_scan_cli_supports_raw_report_mode(tmp_path: Path) -> None:
    output_path = tmp_path / "test_env" / "security" / "python_vuln_scan_report.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/write_vulnerability_scan_report.py",
            "--scan-name",
            "python_dependencies",
            "--target",
            "pyproject.toml",
            "--raw-report",
            str(FIXTURES_ROOT / "pip_audit_clean_report.json"),
            "--raw-format",
            "pip-audit-json",
            "--command",
            "pip-audit --format json",
            "--output",
            str(output_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "vulnerability_scan_status=passed" in result.stdout
    assert "vulnerability_scan_findings=0" in result.stdout
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_vulnerability_scan_report(payload) == []
    assert payload["report_format"] == "pip-audit-json"
