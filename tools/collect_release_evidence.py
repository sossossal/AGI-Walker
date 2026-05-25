#!/usr/bin/env python
"""Collect required release evidence reports into a canonical directory."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for candidate in current.parents:
        if (candidate / "pyproject.toml").exists() and (candidate / "agi_walker").exists():
            return candidate
    return current.parent


def _configure_stdio() -> None:
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")


_configure_stdio()
PROJECT_ROOT = _find_repo_root()
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.core.api.release_contracts import (  # noqa: E402
    default_customer_external_bindings_config_path,
    default_placeholder_external_bindings_config_path,
    default_release_ops_execution_report_path,
)


def _run(command: list[str]) -> int:
    print(f"command={' '.join(command)}")
    result = subprocess.run(command, cwd=str(PROJECT_ROOT), check=False)
    return result.returncode


def _resolve_source_path(path: str | None) -> Path | None:
    if not path:
        return None
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return PROJECT_ROOT / candidate


def _select_external_bindings_config_source(path: str | None) -> str | None:
    requested_path = path or default_placeholder_external_bindings_config_path()
    normalized_requested = Path(requested_path).as_posix()
    resolved_requested = _resolve_source_path(requested_path)
    default_customer_path = default_customer_external_bindings_config_path()
    resolved_customer = _resolve_source_path(default_customer_path)

    if (
        normalized_requested == default_placeholder_external_bindings_config_path()
        and resolved_customer is not None
        and resolved_customer.is_file()
    ):
        return default_customer_path
    if resolved_requested is not None and resolved_requested.is_file():
        return requested_path
    if resolved_customer is not None and resolved_customer.is_file():
        return default_customer_path
    return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Collect canonical release evidence reports."
    )
    parser.add_argument(
        "--output-root",
        default=str(PROJECT_ROOT / "test_env" / "release_evidence"),
        help="Directory used to store structured release evidence reports.",
    )
    parser.add_argument(
        "--python-vuln-report-source",
        default=None,
        help="Optional existing structured python vulnerability report to copy into the canonical release evidence directory.",
    )
    parser.add_argument(
        "--python-vuln-raw-report",
        default=None,
        help="Optional raw scanner JSON report for Python dependencies.",
    )
    parser.add_argument(
        "--python-vuln-raw-format",
        default="pip-audit-json",
        choices=["pip-audit-json", "trivy-json"],
        help="Raw scanner format used with --python-vuln-raw-report.",
    )
    parser.add_argument(
        "--python-vuln-command",
        default="pip-audit --format json",
        help="Scanner command recorded when --python-vuln-raw-report is normalized.",
    )
    parser.add_argument(
        "--run-python-vuln-scan",
        action="store_true",
        help="Execute tools/run_python_vulnerability_scan.py when no structured or raw Python vulnerability report is supplied.",
    )
    parser.add_argument(
        "--python-vuln-raw-output",
        default=None,
        help="Optional path used to persist raw pip-audit JSON when --run-python-vuln-scan executes the scanner.",
    )
    parser.add_argument(
        "--container-vuln-report-source",
        default=None,
        help="Optional existing structured container vulnerability report to copy into the canonical release evidence directory.",
    )
    parser.add_argument(
        "--container-vuln-raw-report",
        default=None,
        help="Optional raw scanner JSON report for container images.",
    )
    parser.add_argument(
        "--container-vuln-raw-format",
        default="trivy-json",
        choices=["pip-audit-json", "trivy-json"],
        help="Raw scanner format used with --container-vuln-raw-report.",
    )
    parser.add_argument(
        "--container-vuln-command",
        default="trivy image --scanners vuln --timeout 15m --format json",
        help="Scanner command recorded when --container-vuln-raw-report is normalized.",
    )
    parser.add_argument(
        "--run-container-vuln-scan",
        action="store_true",
        help="Execute tools/run_container_vulnerability_scan.py when no structured or raw container vulnerability report is supplied.",
    )
    parser.add_argument(
        "--container-image-ref",
        action="append",
        default=[],
        help="Explicit container image reference passed through when --run-container-vuln-scan is enabled. May be supplied multiple times.",
    )
    parser.add_argument(
        "--container-vuln-raw-output-dir",
        default=None,
        help="Optional directory used to persist raw trivy JSON when --run-container-vuln-scan executes the scanner.",
    )
    parser.add_argument(
        "--vulnerability-exception-report-source",
        default=None,
        help="Optional existing structured vulnerability exception report to copy into the canonical release evidence directory.",
    )
    parser.add_argument(
        "--vulnerability-exception-input-source",
        default=str(
            Path("deployment") / "security" / "vulnerability_exceptions.input.json"
        ),
        help="Optional JSON input used to build the canonical vulnerability exception report when a structured report is not supplied.",
    )
    parser.add_argument(
        "--external-bindings-config-source",
        default=str(Path("deployment") / "customer_delivery.external_bindings.json"),
        help="Managed external bindings config passed into extension execution actuals generation when present.",
    )
    parser.add_argument(
        "--release-ops-execution-report-source",
        default=None,
        help="Optional existing release_ops_execution_report.json copied into the canonical operations evidence directory.",
    )
    parser.add_argument(
        "--security-only",
        action="store_true",
        help=(
            "Collect only security posture evidence: SBOM, vulnerability reports, "
            "exceptions, review, backup/restore, remediation and security posture. "
            "Skips broad release gates such as non-live pytest, targeted release "
            "contracts, clean checkout smoke and operations evidence."
        ),
    )
    return parser


def _copy_if_present(source: str | None, destination: Path) -> bool:
    if not source:
        return False
    source_path = Path(source)
    if not source_path.is_file():
        return False
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source_path, destination)
    print(f"copied_report={destination}")
    return True


def _append_vulnerability_report_command(
    commands: list[tuple[str, list[str]]],
    *,
    report_name: str,
    scan_name: str,
    target: str,
    structured_source: str | None,
    raw_source: str | None,
    raw_format: str,
    command_text: str,
    execute_scan: bool,
    execution_command: list[str],
    output_path: Path,
) -> None:
    if _copy_if_present(structured_source, output_path):
        return
    if not raw_source:
        if execute_scan:
            commands.append((report_name, execution_command))
        return
    commands.append(
        (
            report_name,
            [
                sys.executable,
                "tools/write_vulnerability_scan_report.py",
                "--scan-name",
                scan_name,
                "--target",
                target,
                "--raw-report",
                raw_source,
                "--raw-format",
                raw_format,
                "--command",
                command_text,
                "--output",
                str(output_path),
            ],
        )
    )


def _append_vulnerability_exception_command(
    commands: list[tuple[str, list[str]]],
    *,
    project_root: Path,
    structured_source: str | None,
    input_source: str | None,
    output_path: Path,
) -> None:
    if _copy_if_present(structured_source, output_path):
        return
    if not input_source:
        return
    input_path = Path(input_source)
    if not input_path.is_file():
        return
    commands.append(
        (
            "vulnerability_exception_report",
            [
                sys.executable,
                "tools/build_vulnerability_exception_report.py",
                "--project-root",
                str(project_root),
                "--input",
                str(input_path),
                "--output",
                str(output_path),
            ],
        )
    )


def _append_vulnerability_exception_review_command(
    commands: list[tuple[str, list[str]]],
    *,
    project_root: Path,
    exception_report_output_path: Path,
    output_path: Path,
) -> None:
    commands.append(
        (
            "vulnerability_exception_review",
            [
                sys.executable,
                "tools/build_vulnerability_exception_review_report.py",
                "--project-root",
                str(project_root),
                "--exception-report",
                str(exception_report_output_path),
                "--output",
                str(output_path),
            ],
        )
    )


def _append_extension_execution_commands(
    commands: list[tuple[str, list[str]]],
    *,
    output_root: Path,
    vulnerability_exception_report: Path,
    external_bindings_config_source: str | None,
) -> None:
    operations_root = output_root / "operations"
    instance_artifact = operations_root / "extension_execution_instance.json"
    schedule_artifact = operations_root / "extension_execution_schedule.json"
    actuals_artifact = operations_root / "extension_execution_actuals.json"
    actuals_command = [
        sys.executable,
        "tools/build_extension_execution_actuals.py",
        "--project-root",
        str(PROJECT_ROOT),
        "--output",
        str(actuals_artifact),
        "--schedule-artifact",
        str(schedule_artifact),
    ]
    selected_external_bindings = _select_external_bindings_config_source(
        external_bindings_config_source
    )
    resolved_external_bindings = _resolve_source_path(selected_external_bindings)
    if resolved_external_bindings and resolved_external_bindings.is_file():
        actuals_command.extend(
            [
                "--external-bindings-config",
                selected_external_bindings or str(resolved_external_bindings),
            ]
        )

    commands.extend(
        [
            (
                "extension_execution_instance",
                [
                    sys.executable,
                    "tools/build_extension_execution_instance.py",
                    "--project-root",
                    str(PROJECT_ROOT),
                    "--output",
                    str(instance_artifact),
                    "--vulnerability-exception-report",
                    str(vulnerability_exception_report),
                ],
            ),
            (
                "extension_execution_schedule",
                [
                    sys.executable,
                    "tools/build_extension_execution_schedule.py",
                    "--project-root",
                    str(PROJECT_ROOT),
                    "--output",
                    str(schedule_artifact),
                    "--instance-artifact",
                    str(instance_artifact),
                ],
            ),
            ("extension_execution_actuals", actuals_command),
            (
                "extension_execution_evidence",
                [
                    sys.executable,
                    "tools/build_extension_execution_evidence.py",
                    "--project-root",
                    str(PROJECT_ROOT),
                    "--source-root",
                    str(PROJECT_ROOT),
                    "--output-root",
                    str(operations_root),
                    "--vulnerability-exception-report",
                    str(vulnerability_exception_report),
                    "--actuals-artifact",
                    str(actuals_artifact),
                ],
            ),
        ]
    )


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    output_root = Path(args.output_root)
    output_root.mkdir(parents=True, exist_ok=True)

    non_live_report = output_root / "non_live_gate_report.json"
    targeted_report = output_root / "release_contracts_and_capability_matrix_report.json"
    clean_checkout_output_root = output_root / "clean_checkout_smoke"
    clean_checkout_report = output_root / "clean_checkout_smoke_report.json"
    security_root = output_root / "security"
    sbom_report = security_root / "sbom.json"
    python_vuln_report = security_root / "python_vuln_scan_report.json"
    container_vuln_report = security_root / "container_vuln_scan_report.json"
    vulnerability_exception_report = security_root / "vulnerability_exception_report.json"
    vulnerability_remediation_report = security_root / "vulnerability_remediation_report.json"
    backup_restore_rehearsal_output_root = security_root / "backup_restore_rehearsal"
    backup_restore_rehearsal_report = (
        security_root / "backup_restore_rehearsal_report.json"
    )
    security_posture_report = security_root / "security_posture_report.json"
    release_ops_execution_report = (
        output_root / "operations" / Path(default_release_ops_execution_report_path()).name
    )

    commands = [
        (
            "sbom_artifact",
            [
                sys.executable,
                "tools/build_sbom_artifact.py",
                "--project-root",
                str(PROJECT_ROOT),
                "--output",
                str(sbom_report),
            ],
        ),
    ]

    _append_vulnerability_report_command(
        commands,
        report_name="python_vulnerability_report",
        scan_name="python_dependencies",
        target="pyproject.toml",
        structured_source=args.python_vuln_report_source,
        raw_source=args.python_vuln_raw_report,
        raw_format=args.python_vuln_raw_format,
        command_text=args.python_vuln_command,
        execute_scan=args.run_python_vuln_scan,
        execution_command=[
            sys.executable,
            "tools/run_python_vulnerability_scan.py",
            "--project-root",
            str(PROJECT_ROOT),
            "--scan-name",
            "python_dependencies",
            "--target",
            "pyproject.toml",
            "--command",
            args.python_vuln_command,
            *(
                ["--raw-output", args.python_vuln_raw_output]
                if args.python_vuln_raw_output
                else []
            ),
            "--output",
            str(python_vuln_report),
        ],
        output_path=python_vuln_report,
    )
    _append_vulnerability_report_command(
        commands,
        report_name="container_vulnerability_report",
        scan_name="container_images",
        target="deployment/docker-compose.yml",
        structured_source=args.container_vuln_report_source,
        raw_source=args.container_vuln_raw_report,
        raw_format=args.container_vuln_raw_format,
        command_text=args.container_vuln_command,
        execute_scan=args.run_container_vuln_scan,
        execution_command=[
            sys.executable,
            "tools/run_container_vulnerability_scan.py",
            "--project-root",
            str(PROJECT_ROOT),
            "--scan-name",
            "container_images",
            "--target",
            "deployment/docker-compose.yml",
            "--command",
            args.container_vuln_command,
            *(
                ["--raw-output-dir", args.container_vuln_raw_output_dir]
                if args.container_vuln_raw_output_dir
                else []
            ),
            *[
                item
                for image_ref in args.container_image_ref
                for item in ("--image-ref", image_ref)
            ],
            "--output",
            str(container_vuln_report),
        ],
        output_path=container_vuln_report,
    )
    _append_vulnerability_exception_command(
        commands,
        project_root=PROJECT_ROOT,
        structured_source=args.vulnerability_exception_report_source,
        input_source=args.vulnerability_exception_input_source,
        output_path=vulnerability_exception_report,
    )
    resolved_exception_report_source = _resolve_source_path(
        args.vulnerability_exception_report_source
    )
    resolved_exception_input_source = _resolve_source_path(
        args.vulnerability_exception_input_source
    )
    if (
        resolved_exception_report_source is not None
        and resolved_exception_report_source.is_file()
    ) or (
        resolved_exception_input_source is not None
        and resolved_exception_input_source.is_file()
    ):
        _append_vulnerability_exception_review_command(
            commands,
            project_root=PROJECT_ROOT,
            exception_report_output_path=vulnerability_exception_report,
            output_path=security_root / "vulnerability_exception_review_report.json",
        )

    if not args.security_only:
        commands.extend([
            (
                "non_live_gate",
                [
                    sys.executable,
                    "tools/write_pytest_evidence_report.py",
                    "--name",
                    "non_live_gate",
                    "--output",
                    str(non_live_report),
                    "--",
                    "-m",
                    "not live",
                    "-q",
                ],
            ),
            (
                "release_contracts_and_capability_matrix",
                [
                    sys.executable,
                    "tools/write_pytest_evidence_report.py",
                    "--name",
                    "release_contracts_and_capability_matrix",
                    "--output",
                    str(targeted_report),
                    "--",
                    "tests/test_release_contracts.py",
                    "tests/test_release_artifact_builder.py",
                    "tests/test_capability_matrix.py",
                    "tests/test_mcp_tools.py",
                    "tests/test_mcp_server.py",
                    "tests/test_web_panel_aux_apis.py",
                    "-q",
                ],
            ),
        ])

    commands.extend([
        (
            "backup_restore_rehearsal",
            [
                sys.executable,
                "tools/run_backup_restore_rehearsal.py",
                "--project-root",
                str(PROJECT_ROOT),
                "--output-root",
                str(backup_restore_rehearsal_output_root),
                "--report-file",
                str(backup_restore_rehearsal_report),
            ],
        ),
        (
            "vulnerability_remediation",
            [
                sys.executable,
                "tools/build_vulnerability_remediation_report.py",
                "--project-root",
                str(PROJECT_ROOT),
                "--python-vuln-report",
                str(python_vuln_report),
                "--container-vuln-report",
                str(container_vuln_report),
                "--vulnerability-exception-report",
                str(vulnerability_exception_report),
                "--output",
                str(vulnerability_remediation_report),
            ],
        ),
        (
            "security_posture",
            [
                sys.executable,
                "tools/build_security_posture_report.py",
                "--project-root",
                str(PROJECT_ROOT),
                "--sbom",
                str(sbom_report),
                "--python-vuln-report",
                str(python_vuln_report),
                "--container-vuln-report",
                str(container_vuln_report),
                "--backup-restore-report",
                str(backup_restore_rehearsal_report),
                "--vulnerability-remediation-report",
                str(vulnerability_remediation_report),
                "--vulnerability-exception-report",
                str(vulnerability_exception_report),
                "--output",
                str(security_posture_report),
            ],
        ),
    ])

    if not args.security_only:
        commands.append(
            (
                "clean_checkout_smoke",
                [
                    sys.executable,
                    "tools/run_clean_checkout_smoke.py",
                    "--output-root",
                    str(clean_checkout_output_root),
                    "--report-file",
                    str(clean_checkout_report),
                ],
            )
        )
        _append_extension_execution_commands(
            commands,
            output_root=output_root,
            vulnerability_exception_report=vulnerability_exception_report,
            external_bindings_config_source=args.external_bindings_config_source,
        )

        _copy_if_present(
            args.release_ops_execution_report_source,
            release_ops_execution_report,
        )

    failures: list[str] = []
    for name, command in commands:
        print(f"[collect] {name}")
        code = _run(command)
        if code != 0:
            failures.append(name)
        print()

    print(f"release_evidence_output_root={output_root}")
    print(f"release_evidence_failures={len(failures)}")
    if failures:
        print(f"release_evidence_failed_checks={','.join(failures)}")
        return 1

    print("release_evidence_status=passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
