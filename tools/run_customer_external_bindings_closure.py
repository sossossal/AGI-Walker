#!/usr/bin/env python
"""Run the managed customer external-bindings closure chain."""

from __future__ import annotations

import argparse
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
    EXTENSION_EXTERNAL_BINDING_SECTION_IDS,
    build_release_evidence_report,
    build_run_customer_external_bindings_closure_command,
    default_customer_external_bindings_config_path,
    default_customer_external_bindings_closure_report_path,
    default_customer_external_bindings_confirmation_report_path,
    default_extension_execution_actuals_artifact_path,
    default_extension_execution_instance_artifact_path,
    default_extension_execution_schedule_artifact_path,
    resolve_customer_external_bindings_config_path,
    write_release_evidence_report,
)


def _resolve_project_path(path: str | Path, project_root: Path) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return project_root / candidate


def _run(command: list[str]) -> int:
    print(f"command={' '.join(command)}")
    result = subprocess.run(
        command,
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if result.stdout.strip():
        print(result.stdout.strip())
    if result.stderr.strip():
        print(result.stderr.strip(), file=sys.stderr)
    return result.returncode


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run the managed customer external-bindings closure chain: generate config "
            "if needed, confirm bindings, rebuild actuals, emit confirmation evidence, "
            "and optionally refresh canonical release evidence."
        )
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to resolve customer delivery artifact paths.",
    )
    parser.add_argument(
        "--config",
        default=default_customer_external_bindings_config_path(),
        help="Customer-specific external bindings config to generate/confirm.",
    )
    parser.add_argument(
        "--instance-artifact",
        default=default_extension_execution_instance_artifact_path(),
        help="Extension execution instance artifact. Auto-generated when missing.",
    )
    parser.add_argument(
        "--schedule-artifact",
        default=default_extension_execution_schedule_artifact_path(),
        help="Extension execution schedule artifact. Auto-generated when missing.",
    )
    parser.add_argument(
        "--actuals-output",
        default=default_extension_execution_actuals_artifact_path(),
        help="Output path used when rebuilding extension_execution_actuals.",
    )
    parser.add_argument(
        "--confirmation-report-output",
        default=default_customer_external_bindings_confirmation_report_path(),
        help="Output path for customer_external_bindings_confirmation_report.json.",
    )
    parser.add_argument(
        "--closure-report-output",
        default=default_customer_external_bindings_closure_report_path(),
        help="Output path for customer_external_bindings_closure_report.json.",
    )
    parser.add_argument(
        "--collect-output-root",
        default="test_env/release_evidence",
        help="Output root passed to tools/collect_release_evidence.py unless skipped.",
    )
    parser.add_argument(
        "--skip-collect-release-evidence",
        action="store_true",
        help="Do not run tools/collect_release_evidence.py after rebuilding actuals and confirmation evidence.",
    )
    parser.add_argument(
        "--vulnerability-exception-report",
        default="test_env/release_evidence/security/vulnerability_exception_report.json",
        help="Structured vulnerability exception report used when auto-generating extension_execution_instance.",
    )
    parser.add_argument(
        "--section",
        action="append",
        choices=list(EXTENSION_EXTERNAL_BINDING_SECTION_IDS),
        default=None,
        help="Binding section to confirm. Defaults to all sections.",
    )
    parser.add_argument(
        "--confirmed-by",
        required=True,
        help="Actor confirming the customer-owned system mapping.",
    )
    parser.add_argument("--confirmed-at", default=None)
    parser.add_argument(
        "--confirmation-ticket",
        required=True,
        help="Change ticket / approval ticket used to attest the confirmation.",
    )
    parser.add_argument("--confirmation-notes", default=None)
    parser.add_argument("--confirmation-evidence", default=None)
    parser.add_argument(
        "--overrides-file",
        default=None,
        help="Optional JSON file containing section -> field -> value overrides passed through to confirm_customer_external_bindings.py.",
    )
    parser.add_argument(
        "--set",
        action="append",
        default=None,
        help="Optional section.field=value override applied before confirming the section.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    project_root = Path(args.project_root).resolve()
    config_path = resolve_customer_external_bindings_config_path(args.config)
    resolved_config_path = _resolve_project_path(config_path, project_root)
    resolved_instance_path = _resolve_project_path(args.instance_artifact, project_root)
    resolved_schedule_path = _resolve_project_path(args.schedule_artifact, project_root)
    resolved_actuals_output = _resolve_project_path(args.actuals_output, project_root)
    resolved_confirmation_output = _resolve_project_path(
        args.confirmation_report_output,
        project_root,
    )
    resolved_closure_report_output = _resolve_project_path(
        args.closure_report_output,
        project_root,
    )

    if (
        not args.skip_collect_release_evidence
        and project_root != PROJECT_ROOT.resolve()
    ):
        parser.error(
            "collect_release_evidence.py only supports the repository project root; "
            "use --skip-collect-release-evidence when --project-root points elsewhere"
        )

    selected_sections = list(
        dict.fromkeys(args.section or list(EXTENSION_EXTERNAL_BINDING_SECTION_IDS))
    )
    generated_instance = False
    generated_schedule = False
    generated_config = False
    failures: list[str] = []

    if not resolved_instance_path.is_file():
        generated_instance = True
        command = [
            sys.executable,
            "tools/build_extension_execution_instance.py",
            "--project-root",
            str(project_root),
            "--output",
            args.instance_artifact,
            "--vulnerability-exception-report",
            args.vulnerability_exception_report,
        ]
        if _run(command) != 0 and not resolved_instance_path.is_file():
            failures.append("extension_execution_instance")

    if not resolved_schedule_path.is_file():
        generated_schedule = True
        command = [
            sys.executable,
            "tools/build_extension_execution_schedule.py",
            "--project-root",
            str(project_root),
            "--output",
            args.schedule_artifact,
            "--instance-artifact",
            args.instance_artifact,
        ]
        if _run(command) != 0 and not resolved_schedule_path.is_file():
            failures.append("extension_execution_schedule")

    if not resolved_config_path.is_file():
        generated_config = True
        command = [
            sys.executable,
            "tools/build_customer_external_bindings_config.py",
            "--project-root",
            str(project_root),
            "--output",
            config_path,
            "--instance-artifact",
            args.instance_artifact,
        ]
        if _run(command) != 0 and not resolved_config_path.is_file():
            failures.append("customer_external_bindings_config")

    if generated_config and not (args.set or [] or args.overrides_file):
        failures.append("customer_external_bindings_overrides_missing")
    else:
        confirm_command = [
            sys.executable,
            "tools/confirm_customer_external_bindings.py",
            "--project-root",
            str(project_root),
            "--config",
            config_path,
            "--confirmed-by",
            args.confirmed_by,
            "--confirmation-ticket",
            args.confirmation_ticket,
        ]
        if args.confirmed_at:
            confirm_command.extend(["--confirmed-at", args.confirmed_at])
        if args.confirmation_notes:
            confirm_command.extend(["--confirmation-notes", args.confirmation_notes])
        if args.confirmation_evidence:
            confirm_command.extend(["--confirmation-evidence", args.confirmation_evidence])
        if args.overrides_file:
            confirm_command.extend(["--overrides-file", args.overrides_file])
        for section in selected_sections:
            confirm_command.extend(["--section", section])
        for override in args.set or []:
            confirm_command.extend(["--set", override])
        if _run(confirm_command) != 0:
            failures.append("confirm_customer_external_bindings")

        actuals_command = [
            sys.executable,
            "tools/build_extension_execution_actuals.py",
            "--project-root",
            str(project_root),
            "--output",
            args.actuals_output,
            "--schedule-artifact",
            args.schedule_artifact,
            "--external-bindings-config",
            config_path,
        ]
        if _run(actuals_command) != 0 and not resolved_actuals_output.is_file():
            failures.append("extension_execution_actuals")

        confirmation_report_command = [
            sys.executable,
            "tools/build_customer_external_bindings_confirmation_report.py",
            "--project-root",
            str(project_root),
            "--actuals-artifact",
            args.actuals_output,
            "--output",
            args.confirmation_report_output,
        ]
        if _run(confirmation_report_command) != 0:
            failures.append("customer_external_bindings_confirmation_report")

        if not args.skip_collect_release_evidence:
            collect_command = [
                sys.executable,
                "tools/collect_release_evidence.py",
                "--output-root",
                args.collect_output_root,
                "--external-bindings-config-source",
                config_path,
            ]
            if _run(collect_command) != 0:
                failures.append("collect_release_evidence")

    closure_status = "passed" if not failures else "blocked"
    if failures:
        if "customer_external_bindings_overrides_missing" in failures:
            closure_summary = (
                "customer external bindings closure blocked: generated a draft "
                "customer-specific config, but real customer overrides are still required "
                "before confirmation."
            )
        else:
            closure_summary = (
                "customer external bindings closure blocked: "
                f"failed_steps={', '.join(failures)}."
            )
    else:
        closure_summary = (
            "customer external bindings closure passed: rebuilt actuals and "
            f"confirmation evidence for sections={', '.join(selected_sections)}."
        )
    closure_command = build_run_customer_external_bindings_closure_command(
        config_path=config_path,
        instance_artifact_path=args.instance_artifact,
        schedule_artifact_path=args.schedule_artifact,
        actuals_artifact_path=args.actuals_output,
        confirmation_report_output_path=args.confirmation_report_output,
        closure_report_output_path=args.closure_report_output,
        sections=selected_sections,
        collect_output_root=args.collect_output_root,
        skip_collect_release_evidence=args.skip_collect_release_evidence,
    )
    closure_report = build_release_evidence_report(
        evidence_name="customer_external_bindings_closure",
        status=closure_status,
        summary=closure_summary,
        command=closure_command,
        exit_code=0 if closure_status == "passed" else 1,
        metrics={
            "project_root": str(project_root),
            "config_path": str(resolved_config_path),
            "config_exists": resolved_config_path.is_file(),
            "instance_artifact_path": str(resolved_instance_path),
            "instance_exists": resolved_instance_path.is_file(),
            "schedule_artifact_path": str(resolved_schedule_path),
            "schedule_exists": resolved_schedule_path.is_file(),
            "actuals_output_path": str(resolved_actuals_output),
            "actuals_exists": resolved_actuals_output.is_file(),
            "confirmation_report_output_path": str(resolved_confirmation_output),
            "confirmation_report_exists": resolved_confirmation_output.is_file(),
            "closure_report_output_path": str(resolved_closure_report_output),
            "selected_sections": selected_sections,
            "confirmed_by": args.confirmed_by,
            "confirmation_ticket": args.confirmation_ticket,
            "generated_instance": generated_instance,
            "generated_schedule": generated_schedule,
            "generated_config": generated_config,
            "collect_release_evidence": not args.skip_collect_release_evidence,
            "failure_count": len(failures),
            "failed_steps": failures,
            "overrides_file": (
                str(_resolve_project_path(args.overrides_file, project_root))
                if args.overrides_file
                else None
            ),
        },
    )
    written_closure_report = write_release_evidence_report(
        closure_report,
        resolved_closure_report_output,
    )

    print(f"customer_external_bindings_closure_report_written={written_closure_report}")
    print(f"customer_external_bindings_closure_instance_generated={str(generated_instance).lower()}")
    print(f"customer_external_bindings_closure_schedule_generated={str(generated_schedule).lower()}")
    print(f"customer_external_bindings_closure_config_generated={str(generated_config).lower()}")
    print(
        "customer_external_bindings_closure_confirmed_sections="
        + ",".join(selected_sections)
    )
    print(f"customer_external_bindings_closure_config_path={resolved_config_path}")
    if args.overrides_file:
        print(
            "customer_external_bindings_closure_overrides_file="
            f"{_resolve_project_path(args.overrides_file, project_root)}"
        )
    print(f"customer_external_bindings_closure_actuals_output={resolved_actuals_output}")
    print(
        "customer_external_bindings_closure_confirmation_report_output="
        f"{resolved_confirmation_output}"
    )
    print(
        "customer_external_bindings_closure_report_output="
        f"{resolved_closure_report_output}"
    )
    print(
        "customer_external_bindings_closure_collect_release_evidence="
        f"{str(not args.skip_collect_release_evidence).lower()}"
    )
    print(f"customer_external_bindings_closure_failures={len(failures)}")
    if failures:
        if "customer_external_bindings_overrides_missing" in failures:
            print(
                "customer_external_bindings_closure_summary="
                "Generated a draft customer-specific config. Fill the real customer system metadata "
                "in the config or rerun with --overrides-file / --set overrides before confirmation."
            )
        print(
            "customer_external_bindings_closure_failed_steps="
            + ",".join(failures)
        )
        print("customer_external_bindings_closure_status=blocked")
        return 1

    print("customer_external_bindings_closure_status=passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
