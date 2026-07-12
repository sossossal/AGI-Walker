"""Deterministic orchestration for the external-mainline release flow."""

from __future__ import annotations

import json
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any

from agi_walker.core.api.release_contracts import (
    EXTENSION_EXTERNAL_BINDING_SECTION_IDS,
    build_external_mainline_execution_plan_artifact,
    build_external_mainline_input_checklist_report,
    default_canonical_industrial_delivery_rehearsal_report_path,
    default_customer_external_bindings_closure_report_path,
    default_customer_external_bindings_config_path,
    default_external_mainline_execution_plan_path,
    default_external_mainline_inputs_path,
    default_vulnerability_exception_review_report_path,
    write_external_mainline_execution_plan_artifact,
    write_release_evidence_report,
)
from agi_walker.core.api.release_ops_contracts import (
    ExternalMainlineExecutionRequest,
    ExternalMainlineExecutionResult,
)

CommandRunner = Callable[[list[str]], int]


def _validate_project_relative_path(path: str | Path, *, field_name: str) -> Path:
    raw_path = str(path).strip()
    if not raw_path:
        raise ValueError(f"{field_name} must be a non-empty project-relative path")
    candidate = Path(raw_path)
    if candidate.is_absolute():
        raise ValueError(f"{field_name} must be project-relative")
    if any(part == ".." for part in raw_path.replace("\\", "/").split("/")):
        raise ValueError(f"{field_name} must stay within the project root")
    return candidate


def _resolve_project_path(
    path: str | Path,
    project_root: str | Path,
    *,
    field_name: str = "path",
) -> Path:
    project_root_path = Path(project_root).resolve()
    candidate = _validate_project_relative_path(path, field_name=field_name)
    resolved_path = (project_root_path / candidate).resolve()
    if not resolved_path.is_relative_to(project_root_path):
        raise ValueError(f"{field_name} must stay within the project root")
    return resolved_path


def _validate_local_request_paths(request: ExternalMainlineExecutionRequest) -> None:
    path_fields = [
        ("--output", request.output),
        (
            "--external-mainline-input-checklist-report",
            request.external_mainline_input_checklist_report,
        ),
        ("--customer-config", request.customer_config),
        (
            "--customer-external-bindings-closure-report",
            request.customer_external_bindings_closure_report,
        ),
        (
            "--vulnerability-exception-review-report",
            request.vulnerability_exception_review_report,
        ),
        (
            "--industrial-delivery-rehearsal-report",
            request.industrial_delivery_rehearsal_report,
        ),
    ]
    if request.refresh_industrial_rehearsal:
        path_fields.append(
            (
                "--industrial-rehearsal-output-root",
                request.industrial_rehearsal_output_root,
            )
        )
    if not request.skip_managed_inputs:
        path_fields.append(("--inputs-file", request.inputs_file))
    if request.customer_overrides_file:
        path_fields.append(
            ("--customer-overrides-file", request.customer_overrides_file)
        )
    for field_name, path in path_fields:
        if path is None:
            raise ValueError(f"{field_name} must be set")
        _resolve_project_path(path, request.project_root, field_name=field_name)


def _is_non_empty_string(value: object) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _is_placeholder_string(value: object) -> bool:
    if not isinstance(value, str):
        return False
    normalized = value.strip()
    if not normalized:
        return False
    lowered = normalized.lower()
    if lowered in {"placeholder", "<placeholder>", "tbd", "<tbd>"}:
        return True
    return normalized.startswith("<") and normalized.endswith(">")


def _has_effective_confirmation_value(value: object) -> bool:
    return _is_non_empty_string(value) and not _is_placeholder_string(value)


def _coerce_string_list(value: object) -> list[str]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        return []
    return [str(item).strip() for item in value if _is_non_empty_string(item)]


def _load_inputs_file(
    path: str | Path, *, project_root: str | Path
) -> dict[str, object]:
    resolved_path = _resolve_project_path(
        path,
        project_root,
        field_name="--inputs-file",
    )
    try:
        payload = json.loads(resolved_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(
            f"--inputs-file is not valid JSON: {resolved_path}: {exc}"
        ) from exc
    if not isinstance(payload, Mapping):
        raise ValueError("external mainline inputs file must be a JSON object")
    return dict(payload)


def _is_default_managed_inputs_path(
    request: ExternalMainlineExecutionRequest,
    resolved_inputs_path: Path,
) -> bool:
    default_inputs_path = _resolve_project_path(
        default_external_mainline_inputs_path(),
        request.project_root,
        field_name="default inputs file",
    )
    return resolved_inputs_path == default_inputs_path


def _synchronize_inputs_file(
    request: ExternalMainlineExecutionRequest,
    *,
    run_command: CommandRunner,
    python_executable: str,
) -> tuple[Path, str | None]:
    if not request.inputs_file:
        raise ValueError(
            "--inputs-file must be set unless --skip-managed-inputs is used"
        )
    resolved_inputs_path = _resolve_project_path(
        request.inputs_file,
        request.project_root,
        field_name="--inputs-file",
    )
    had_existing_file = resolved_inputs_path.is_file()
    if had_existing_file and not _is_default_managed_inputs_path(
        request, resolved_inputs_path
    ):
        return resolved_inputs_path, None

    synchronize_command = [
        python_executable,
        "tools/build_external_mainline_inputs.py",
        "--project-root",
        request.project_root,
        "--output",
        request.inputs_file,
        "--customer-config",
        request.customer_config,
        "--customer-external-bindings-closure-report",
        request.customer_external_bindings_closure_report,
        "--vulnerability-exception-review-report",
        request.vulnerability_exception_review_report,
        "--industrial-delivery-rehearsal-report",
        request.industrial_delivery_rehearsal_report,
    ]
    if request.customer_overrides_file:
        synchronize_command.extend(
            ["--customer-overrides-file", request.customer_overrides_file]
        )
    if run_command(synchronize_command) != 0 or not resolved_inputs_path.is_file():
        raise ValueError(
            "--inputs-file automatic synchronization failed: " f"{resolved_inputs_path}"
        )
    return resolved_inputs_path, "refreshed" if had_existing_file else "bootstrapped"


def _apply_inputs_file(
    request: ExternalMainlineExecutionRequest,
    *,
    run_command: CommandRunner,
    python_executable: str,
) -> tuple[Path | None, str | None]:
    if not request.inputs_file:
        return None, None

    resolved_inputs_path, sync_status = _synchronize_inputs_file(
        request,
        run_command=run_command,
        python_executable=python_executable,
    )
    payload = _load_inputs_file(request.inputs_file, project_root=request.project_root)

    customer = payload.get("customer_external_bindings")
    if isinstance(customer, Mapping):
        if customer.get("enabled") is False:
            request.skip_customer_external_bindings_closure = True
        if (
            request.customer_config == default_customer_external_bindings_config_path()
            and _is_non_empty_string(customer.get("config"))
        ):
            request.customer_config = str(customer.get("config")).strip()
        if (
            request.customer_external_bindings_closure_report
            == default_customer_external_bindings_closure_report_path()
            and _is_non_empty_string(customer.get("closure_report_output"))
        ):
            request.customer_external_bindings_closure_report = str(
                customer.get("closure_report_output")
            ).strip()
        if request.customer_section is None:
            customer_sections = _coerce_string_list(customer.get("sections"))
            invalid_sections = [
                item
                for item in customer_sections
                if item not in EXTENSION_EXTERNAL_BINDING_SECTION_IDS
            ]
            if invalid_sections:
                raise ValueError(
                    "--inputs-file customer_external_bindings.sections contains invalid values: "
                    + ", ".join(invalid_sections)
                )
            if customer_sections:
                request.customer_section = customer_sections
        if request.customer_confirmed_by is None and _has_effective_confirmation_value(
            customer.get("confirmed_by")
        ):
            request.customer_confirmed_by = str(customer.get("confirmed_by")).strip()
        if request.customer_confirmed_at is None and _is_non_empty_string(
            customer.get("confirmed_at")
        ):
            request.customer_confirmed_at = str(customer.get("confirmed_at")).strip()
        if (
            request.customer_confirmation_ticket is None
            and _has_effective_confirmation_value(customer.get("confirmation_ticket"))
        ):
            request.customer_confirmation_ticket = str(
                customer.get("confirmation_ticket")
            ).strip()
        if request.customer_confirmation_notes is None and _is_non_empty_string(
            customer.get("confirmation_notes")
        ):
            request.customer_confirmation_notes = str(
                customer.get("confirmation_notes")
            ).strip()
        if request.customer_confirmation_evidence is None and _is_non_empty_string(
            customer.get("confirmation_evidence")
        ):
            request.customer_confirmation_evidence = str(
                customer.get("confirmation_evidence")
            ).strip()
        if request.customer_overrides_file is None and _is_non_empty_string(
            customer.get("overrides_file")
        ):
            request.customer_overrides_file = str(
                customer.get("overrides_file")
            ).strip()
        if request.customer_set is None:
            customer_set = _coerce_string_list(customer.get("set"))
            if customer_set:
                request.customer_set = customer_set
        if (
            request.skip_customer_collect_release_evidence is False
            and customer.get("skip_collect_release_evidence") is True
        ):
            request.skip_customer_collect_release_evidence = True

    vulnerability_review = payload.get("vulnerability_exception_review")
    if isinstance(vulnerability_review, Mapping):
        if vulnerability_review.get("enabled") is False:
            request.skip_vulnerability_exception_review_refresh = True
        if (
            request.vulnerability_exception_review_report
            == default_vulnerability_exception_review_report_path()
            and _is_non_empty_string(vulnerability_review.get("report_output"))
        ):
            request.vulnerability_exception_review_report = str(
                vulnerability_review.get("report_output")
            ).strip()

    industrial = payload.get("industrial_rehearsal")
    if isinstance(industrial, Mapping):
        if (
            request.industrial_delivery_rehearsal_report
            == default_canonical_industrial_delivery_rehearsal_report_path()
            and _is_non_empty_string(industrial.get("report_path"))
        ):
            request.industrial_delivery_rehearsal_report = str(
                industrial.get("report_path")
            ).strip()
        if (
            request.refresh_industrial_rehearsal is False
            and industrial.get("refresh") is True
        ):
            request.refresh_industrial_rehearsal = True
        if request.industrial_rehearsal_version is None and _is_non_empty_string(
            industrial.get("version")
        ):
            request.industrial_rehearsal_version = str(
                industrial.get("version")
            ).strip()
        if request.industrial_rehearsal_build_id is None and _is_non_empty_string(
            industrial.get("build_id")
        ):
            request.industrial_rehearsal_build_id = str(
                industrial.get("build_id")
            ).strip()
        if (
            request.refresh_industrial_rehearsal is True
            and request.industrial_rehearsal_output_root
            == "test_env/release_rehearsal_industrial"
            and _is_non_empty_string(industrial.get("output_root"))
        ):
            request.industrial_rehearsal_output_root = str(
                industrial.get("output_root")
            ).strip()

    industrial_live_evidence = payload.get("industrial_live_evidence")
    if isinstance(industrial_live_evidence, Mapping):
        request.industrial_live_evidence_inputs = dict(industrial_live_evidence)

    return resolved_inputs_path, sync_status


def execute_external_mainline_execution(
    request: ExternalMainlineExecutionRequest,
    *,
    run_command: CommandRunner,
    python_executable: str,
) -> ExternalMainlineExecutionResult:
    _validate_local_request_paths(request)
    resolved_inputs_path = None
    managed_inputs_sync_status = None
    if not request.skip_managed_inputs:
        resolved_inputs_path, managed_inputs_sync_status = _apply_inputs_file(
            request,
            run_command=run_command,
            python_executable=python_executable,
        )
    _validate_local_request_paths(request)

    failures: list[str] = []
    executed_steps: list[str] = []
    skipped_steps: list[str] = []

    if not request.skip_vulnerability_exception_review_refresh:
        review_command = [
            python_executable,
            "tools/build_vulnerability_exception_review_report.py",
            "--project-root",
            request.project_root,
            "--output",
            request.vulnerability_exception_review_report,
            "--exception-report",
            "test_env/release_evidence/security/vulnerability_exception_report.json",
        ]
        if run_command(review_command) != 0:
            failures.append("vulnerability_exception_review_refresh")
        else:
            executed_steps.append("vulnerability_exception_review_refresh")
    else:
        skipped_steps.append("vulnerability_exception_review_refresh")

    if request.refresh_industrial_rehearsal:
        if (
            not request.industrial_rehearsal_version
            or not request.industrial_rehearsal_build_id
        ):
            raise ValueError(
                "--refresh-industrial-rehearsal requires both "
                "--industrial-rehearsal-version and --industrial-rehearsal-build-id"
            )
        rehearsal_command = [
            python_executable,
            "tools/run_release_rehearsal.py",
            "--version",
            request.industrial_rehearsal_version,
            "--build-id",
            request.industrial_rehearsal_build_id,
            "--output-root",
            request.industrial_rehearsal_output_root,
        ]
        if run_command(rehearsal_command) != 0:
            failures.append("industrial_rehearsal_refresh")
        else:
            executed_steps.append("industrial_rehearsal_refresh")
    else:
        skipped_steps.append("industrial_rehearsal_refresh")

    if request.skip_customer_external_bindings_closure:
        skipped_steps.append("customer_external_bindings_closure")
    elif _has_effective_confirmation_value(
        request.customer_confirmed_by
    ) and _has_effective_confirmation_value(request.customer_confirmation_ticket):
        closure_command = [
            python_executable,
            "tools/run_customer_external_bindings_closure.py",
            "--project-root",
            request.project_root,
            "--config",
            request.customer_config,
            "--closure-report-output",
            request.customer_external_bindings_closure_report,
            "--confirmed-by",
            str(request.customer_confirmed_by),
            "--confirmation-ticket",
            str(request.customer_confirmation_ticket),
        ]
        if request.customer_confirmed_at:
            closure_command.extend(["--confirmed-at", request.customer_confirmed_at])
        if request.customer_confirmation_notes:
            closure_command.extend(
                ["--confirmation-notes", request.customer_confirmation_notes]
            )
        if request.customer_confirmation_evidence:
            closure_command.extend(
                ["--confirmation-evidence", request.customer_confirmation_evidence]
            )
        if request.customer_overrides_file:
            closure_command.extend(
                ["--overrides-file", request.customer_overrides_file]
            )
        for section in request.customer_section or []:
            closure_command.extend(["--section", section])
        for override in request.customer_set or []:
            closure_command.extend(["--set", override])
        if request.skip_customer_collect_release_evidence:
            closure_command.append("--skip-collect-release-evidence")
        if run_command(closure_command) != 0:
            failures.append("customer_external_bindings_closure")
        else:
            executed_steps.append("customer_external_bindings_closure")
    else:
        skipped_steps.append("customer_external_bindings_closure")

    payload = build_external_mainline_execution_plan_artifact(
        project_root=request.project_root,
        customer_config_path=request.customer_config,
        customer_external_bindings_closure_report_path=request.customer_external_bindings_closure_report,
        vulnerability_exception_review_report_path=request.vulnerability_exception_review_report,
        industrial_delivery_rehearsal_report_path=request.industrial_delivery_rehearsal_report,
        industrial_live_evidence_inputs=request.industrial_live_evidence_inputs,
    )
    resolved_output_path = _resolve_project_path(
        request.output,
        request.project_root,
        field_name="--output",
    )
    output_path = write_external_mainline_execution_plan_artifact(
        payload,
        resolved_output_path,
    )
    checklist_payload = build_external_mainline_input_checklist_report(
        project_root=request.project_root,
        inputs_file_path=resolved_inputs_path or request.inputs_file,
        external_mainline_execution_plan_path=request.output,
        output_path=request.external_mainline_input_checklist_report,
    )
    checklist_path = write_release_evidence_report(
        checklist_payload,
        _resolve_project_path(
            request.external_mainline_input_checklist_report,
            request.project_root,
            field_name="--external-mainline-input-checklist-report",
        ),
    )
    industrial_step = next(
        (
            item
            for item in payload["steps"]
            if item.get("id") == "industrial_delivery_live_evidence"
        ),
        None,
    )
    industrial_live_evidence_inputs_ready = None
    if isinstance(industrial_step, Mapping):
        industrial_live_evidence_inputs_ready = (
            industrial_step.get("managed_inputs_ready") is True
        )

    return ExternalMainlineExecutionResult(
        payload=payload,
        output_path=Path(output_path),
        checklist_payload=checklist_payload,
        checklist_path=Path(checklist_path),
        resolved_inputs_path=resolved_inputs_path,
        managed_inputs_sync_status=managed_inputs_sync_status,
        executed_steps=executed_steps,
        skipped_steps=skipped_steps,
        failures=failures,
        industrial_live_evidence_inputs_ready=industrial_live_evidence_inputs_ready,
    )


__all__ = ["execute_external_mainline_execution"]
