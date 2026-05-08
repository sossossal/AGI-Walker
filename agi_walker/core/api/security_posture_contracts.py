"""Contracts for security posture, SBOM, and vulnerability scan artifacts."""

from __future__ import annotations

import json
import re
import tomllib
from collections.abc import Mapping, Sequence
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any

from agi_walker.core.api.workflow_contracts import to_jsonable

SBOM_ARTIFACT_VERSION = "1.0"
SBOM_ARTIFACT_TYPE = "sbom_artifact"
VULNERABILITY_SCAN_REPORT_VERSION = "1.0"
VULNERABILITY_SCAN_REPORT_ARTIFACT_TYPE = "vulnerability_scan_report"
VULNERABILITY_EXCEPTION_REPORT_VERSION = "1.0"
VULNERABILITY_EXCEPTION_REPORT_ARTIFACT_TYPE = "vulnerability_exception_report"
VULNERABILITY_REMEDIATION_REPORT_VERSION = "1.0"
VULNERABILITY_REMEDIATION_REPORT_ARTIFACT_TYPE = "vulnerability_remediation_report"
BACKUP_RESTORE_REHEARSAL_REPORT_VERSION = "1.0"
BACKUP_RESTORE_REHEARSAL_REPORT_ARTIFACT_TYPE = "backup_restore_rehearsal_report"
SECURITY_POSTURE_REPORT_VERSION = "1.0"
SECURITY_POSTURE_REPORT_ARTIFACT_TYPE = "security_posture_report"

VULNERABILITY_SCAN_STATUSES = {"passed", "blocked", "not_run"}
VULNERABILITY_SCAN_RAW_FORMATS = {"pip-audit-json", "trivy-json"}
VULNERABILITY_EXCEPTION_SCOPES = {"python_dependencies", "container_images"}
VULNERABILITY_EXCEPTION_STATUSES = {"active", "expired"}
VULNERABILITY_EXCEPTION_REVIEW_STATUSES = {
    "no_active_exceptions",
    "tracked",
    "review_due",
    "expired",
}
VULNERABILITY_REMEDIATION_STATUSES = {"ready", "needs_remediation", "blocked"}
BACKUP_RESTORE_REHEARSAL_STATUSES = {"passed", "blocked"}
SECURITY_POSTURE_STATUSES = {"ready", "blocked"}
DEFAULT_VULNERABILITY_EXCEPTION_REVIEW_WINDOW_DAYS = 30


def _now_iso() -> str:
    return datetime.now().isoformat()


def _is_non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _parse_iso_datetime(value: str | None) -> datetime | None:
    if not _is_non_empty_string(value):
        return None
    try:
        return datetime.fromisoformat(str(value).strip())
    except ValueError:
        return None


def _datetime_is_before(left: datetime | None, right: datetime | None) -> bool:
    if left is None or right is None:
        return False
    normalized_left = left
    normalized_right = right
    if normalized_left.tzinfo is None and normalized_right.tzinfo is not None:
        normalized_left = normalized_left.replace(tzinfo=normalized_right.tzinfo)
    elif normalized_left.tzinfo is not None and normalized_right.tzinfo is None:
        normalized_right = normalized_right.replace(tzinfo=normalized_left.tzinfo)
    return normalized_left < normalized_right


def _normalize_comparable_datetimes(
    left: datetime | None, right: datetime | None
) -> tuple[datetime | None, datetime | None]:
    if left is None or right is None:
        return left, right
    normalized_left = left
    normalized_right = right
    if normalized_left.tzinfo is None and normalized_right.tzinfo is not None:
        normalized_left = normalized_left.replace(tzinfo=normalized_right.tzinfo)
    elif normalized_left.tzinfo is not None and normalized_right.tzinfo is None:
        normalized_right = normalized_right.replace(tzinfo=normalized_left.tzinfo)
    return normalized_left, normalized_right


def _days_until_datetime(
    target: datetime | None, reference: datetime | None
) -> float | None:
    normalized_target, normalized_reference = _normalize_comparable_datetimes(
        target, reference
    )
    if normalized_target is None or normalized_reference is None:
        return None
    return (normalized_target - normalized_reference).total_seconds() / 86400.0


def _build_vulnerability_exception_review_metrics(
    exceptions: Sequence[Mapping[str, Any]],
    *,
    generated_datetime: datetime,
    review_window_days: int,
) -> dict[str, Any]:
    active_exception_count = 0
    expired_exception_count = 0
    review_due_exception_count = 0
    active_expiries: list[tuple[datetime, str]] = []
    review_due_exception_ids: list[str] = []
    review_due_exception_components: list[str] = []
    review_due_exception_tickets: list[str] = []
    expired_exception_ids: list[str] = []
    for item in exceptions:
        status = str(item.get("status") or "").strip()
        expires_at = str(item.get("expires_at") or "").strip()
        expires_datetime = _parse_iso_datetime(expires_at)
        exception_id = str(item.get("id") or "").strip()
        component = str(item.get("component") or "").strip()
        ticket = str(item.get("ticket") or "").strip()
        if status == "active":
            active_exception_count += 1
            if expires_datetime is not None:
                active_expiries.append((expires_datetime, expires_at))
                days_until_expiry = _days_until_datetime(
                    expires_datetime, generated_datetime
                )
                if (
                    days_until_expiry is not None
                    and days_until_expiry >= 0
                    and days_until_expiry <= review_window_days
                ):
                    review_due_exception_count += 1
                    if exception_id:
                        review_due_exception_ids.append(exception_id)
                    if component:
                        review_due_exception_components.append(component)
                    if ticket:
                        review_due_exception_tickets.append(ticket)
        elif status == "expired":
            expired_exception_count += 1
            if exception_id:
                expired_exception_ids.append(exception_id)

    active_expiries.sort(key=lambda item: item[0])
    next_exception_expiry = active_expiries[0][1] if active_expiries else None
    if expired_exception_count:
        review_status = "expired"
    elif active_exception_count == 0:
        review_status = "no_active_exceptions"
    elif review_due_exception_count:
        review_status = "review_due"
    else:
        review_status = "tracked"

    return {
        "active_exception_count": active_exception_count,
        "expired_exception_count": expired_exception_count,
        "review_window_days": review_window_days,
        "review_due_exception_count": review_due_exception_count,
        "review_due_exception_ids": review_due_exception_ids,
        "review_due_exception_components": review_due_exception_components,
        "review_due_exception_tickets": review_due_exception_tickets,
        "expired_exception_ids": expired_exception_ids,
        "next_exception_expiry": next_exception_expiry,
        "review_status": review_status,
    }


def _derive_vulnerability_exception_review_metrics(
    payload: Mapping[str, Any],
) -> dict[str, Any]:
    review_window_days = payload.get("review_window_days")
    if not isinstance(review_window_days, int) or review_window_days < 0:
        review_window_days = DEFAULT_VULNERABILITY_EXCEPTION_REVIEW_WINDOW_DAYS
    generated_datetime = _parse_iso_datetime(str(payload.get("generated_at") or ""))
    if generated_datetime is None:
        generated_datetime = datetime.now()
    exceptions = payload.get("exceptions")
    if not isinstance(exceptions, list):
        exceptions = []
    return _build_vulnerability_exception_review_metrics(
        exceptions,
        generated_datetime=generated_datetime,
        review_window_days=review_window_days,
    )


def _resolve_project_root(project_root: str | Path | None) -> Path:
    return Path(project_root) if project_root is not None else Path.cwd()


def _normalize_component_name(requirement: str) -> str:
    normalized = requirement.strip()
    if not normalized:
        return ""
    return re.split(r"[<>=!~\\[]", normalized, maxsplit=1)[0].strip()


def _iter_requirements_file(
    path: Path,
    project_root: Path,
    *,
    seen_files: set[Path],
) -> list[dict[str, Any]]:
    resolved = path.resolve()
    if resolved in seen_files or not path.is_file():
        return []
    seen_files.add(resolved)

    components: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped.startswith("-r "):
            include_target = stripped[3:].strip()
            components.extend(
                _iter_requirements_file(
                    (path.parent / include_target).resolve(),
                    project_root,
                    seen_files=seen_files,
                )
            )
            continue

        components.append(
            {
                "name": _normalize_component_name(stripped),
                "requirement": stripped,
                "source": str(path.relative_to(project_root)),
                "group": "requirements",
            }
        )
    return components


def build_sbom_artifact(
    *, project_root: str | Path | None = None, generated_at: str | None = None
) -> dict[str, Any]:
    root = _resolve_project_root(project_root)
    pyproject_path = root / "pyproject.toml"
    if not pyproject_path.is_file():
        raise FileNotFoundError(f"missing pyproject.toml under {root}")

    pyproject = tomllib.loads(pyproject_path.read_text(encoding="utf-8"))
    project_data = pyproject.get("project", {})
    components: list[dict[str, Any]] = []
    sources: set[str] = {str(pyproject_path.relative_to(root))}

    for requirement in project_data.get("dependencies", []):
        components.append(
            {
                "name": _normalize_component_name(requirement),
                "requirement": requirement,
                "source": str(pyproject_path.relative_to(root)),
                "group": "project.dependencies",
            }
        )

    for optional_group, requirements in project_data.get(
        "optional-dependencies", {}
    ).items():
        for requirement in requirements:
            components.append(
                {
                    "name": _normalize_component_name(requirement),
                    "requirement": requirement,
                    "source": str(pyproject_path.relative_to(root)),
                    "group": f"project.optional-dependencies.{optional_group}",
                }
            )

    deployment_requirements = [
        root / "deployment" / "requirements.web_panel.txt",
        root / "deployment" / "requirements.web_panel.distributed.txt",
        root / "deployment" / "requirements.distributed_runtime.txt",
    ]
    seen_files: set[Path] = set()
    for requirement_file in deployment_requirements:
        if requirement_file.is_file():
            sources.add(str(requirement_file.relative_to(root)))
            components.extend(
                _iter_requirements_file(requirement_file, root, seen_files=seen_files)
            )
            for nested in seen_files:
                try:
                    sources.add(str(nested.relative_to(root)))
                except ValueError:
                    continue

    deduplicated: list[dict[str, Any]] = []
    seen_components: set[tuple[str, str, str, str]] = set()
    for item in components:
        key = (
            item["name"],
            item["requirement"],
            item["source"],
            item["group"],
        )
        if key in seen_components:
            continue
        seen_components.add(key)
        deduplicated.append(item)

    return {
        "schema_version": SBOM_ARTIFACT_VERSION,
        "artifact_type": SBOM_ARTIFACT_TYPE,
        "generated_at": generated_at or _now_iso(),
        "project_root": str(root),
        "project_name": project_data.get("name", "agi-walker"),
        "project_version": project_data.get("version", "unknown"),
        "dependency_sources": sorted(sources),
        "components": to_jsonable(deduplicated),
        "component_count": len(deduplicated),
    }


def validate_sbom_artifact(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["sbom artifact must be an object"]

    errors: list[str] = []
    for field in [
        "schema_version",
        "artifact_type",
        "generated_at",
        "project_root",
        "project_name",
        "project_version",
        "dependency_sources",
        "components",
        "component_count",
    ]:
        if field not in payload:
            errors.append(f"missing required field: {field}")

    if payload.get("schema_version") != SBOM_ARTIFACT_VERSION:
        errors.append(f"schema_version must be {SBOM_ARTIFACT_VERSION!r}")
    if payload.get("artifact_type") != SBOM_ARTIFACT_TYPE:
        errors.append(f"artifact_type must be {SBOM_ARTIFACT_TYPE!r}")

    for field in ["generated_at", "project_root", "project_name", "project_version"]:
        if field in payload and not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")

    dependency_sources = payload.get("dependency_sources")
    if not isinstance(dependency_sources, list):
        errors.append("dependency_sources must be a list")
    else:
        for index, item in enumerate(dependency_sources, start=1):
            if not _is_non_empty_string(item):
                errors.append(f"dependency_sources[{index}] must be a non-empty string")

    components = payload.get("components")
    if not isinstance(components, list):
        errors.append("components must be a list")
    else:
        for index, item in enumerate(components, start=1):
            prefix = f"components[{index}]"
            if not isinstance(item, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            for field in ["name", "requirement", "source", "group"]:
                if not _is_non_empty_string(item.get(field)):
                    errors.append(f"{prefix}.{field} must be a non-empty string")

    if (
        not isinstance(payload.get("component_count"), int)
        or payload.get("component_count") < 0
    ):
        errors.append("component_count must be a non-negative integer")

    return errors


def write_sbom_artifact(payload: Mapping[str, Any], path: str | Path) -> Path:
    errors = validate_sbom_artifact(payload)
    if errors:
        raise ValueError(f"invalid sbom artifact: {'; '.join(errors)}")

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def build_vulnerability_scan_report(
    *,
    scan_name: str,
    target: str,
    status: str,
    summary: str,
    command: str,
    scanner: str,
    report_format: str | None = None,
    raw_report_path: str | None = None,
    finding_count: int = 0,
    affected_component_count: int = 0,
    generated_at: str | None = None,
) -> dict[str, Any]:
    return {
        "schema_version": VULNERABILITY_SCAN_REPORT_VERSION,
        "artifact_type": VULNERABILITY_SCAN_REPORT_ARTIFACT_TYPE,
        "generated_at": generated_at or _now_iso(),
        "scan_name": scan_name,
        "target": target,
        "status": status,
        "summary": summary,
        "command": command,
        "scanner": scanner,
        "report_format": report_format,
        "raw_report_path": raw_report_path,
        "finding_count": finding_count,
        "affected_component_count": affected_component_count,
    }


def _load_raw_json(path: str | Path) -> dict[str, Any]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError("raw scan report payload must be a JSON object")
    return dict(payload)


def _normalize_pip_audit_json_report(payload: Mapping[str, Any]) -> dict[str, Any]:
    dependencies = payload.get("dependencies")
    if not isinstance(dependencies, list):
        raise ValueError("pip-audit JSON must include a dependencies list")

    finding_count = 0
    affected_components: set[str] = set()
    for dependency in dependencies:
        if not isinstance(dependency, Mapping):
            continue
        vulnerabilities = (
            dependency.get("vulns") or dependency.get("vulnerabilities") or []
        )
        if not isinstance(vulnerabilities, list):
            continue
        if vulnerabilities:
            name = dependency.get("name")
            if _is_non_empty_string(name):
                affected_components.add(str(name))
            finding_count += len(vulnerabilities)

    status = "passed" if finding_count == 0 else "blocked"
    summary = (
        f"pip-audit reported no known vulnerabilities across {len(dependencies)} dependencies."
        if status == "passed"
        else "pip-audit reported "
        f"{finding_count} finding(s) across {len(affected_components)} affected dependency component(s)."
    )
    return {
        "status": status,
        "summary": summary,
        "finding_count": finding_count,
        "affected_component_count": len(affected_components),
    }


def _normalize_trivy_json_report(payload: Mapping[str, Any]) -> dict[str, Any]:
    results = payload.get("Results") or payload.get("results")
    if not isinstance(results, list):
        raise ValueError("trivy JSON must include a Results list")

    finding_count = 0
    affected_components: set[str] = set()
    targets: set[str] = set()
    for result in results:
        if not isinstance(result, Mapping):
            continue
        target = result.get("Target") or result.get("target")
        if _is_non_empty_string(target):
            targets.add(str(target))
        vulnerabilities = (
            result.get("Vulnerabilities") or result.get("vulnerabilities") or []
        )
        if not isinstance(vulnerabilities, list):
            continue
        finding_count += len(vulnerabilities)
        for vulnerability in vulnerabilities:
            if not isinstance(vulnerability, Mapping):
                continue
            component = vulnerability.get("PkgName") or vulnerability.get("pkgName")
            if _is_non_empty_string(component):
                affected_components.add(str(component))

    status = "passed" if finding_count == 0 else "blocked"
    summary = (
        "trivy reported no known vulnerabilities across "
        f"{len(targets) or len(results)} scanned target(s)."
        if status == "passed"
        else "trivy reported "
        f"{finding_count} finding(s) across {len(affected_components)} affected component(s)."
    )
    return {
        "status": status,
        "summary": summary,
        "finding_count": finding_count,
        "affected_component_count": len(affected_components),
    }


def build_vulnerability_scan_report_from_raw(
    *,
    scan_name: str,
    target: str,
    raw_report_path: str | Path,
    raw_format: str,
    command: str,
    scanner: str | None = None,
    generated_at: str | None = None,
) -> dict[str, Any]:
    if raw_format not in VULNERABILITY_SCAN_RAW_FORMATS:
        raise ValueError(
            f"raw_format must be one of {sorted(VULNERABILITY_SCAN_RAW_FORMATS)}"
        )

    resolved_raw_report_path = Path(raw_report_path)
    raw_payload = _load_raw_json(resolved_raw_report_path)
    if raw_format == "pip-audit-json":
        normalized = _normalize_pip_audit_json_report(raw_payload)
        resolved_scanner = scanner or "pip-audit"
    else:
        normalized = _normalize_trivy_json_report(raw_payload)
        resolved_scanner = scanner or "trivy"

    return build_vulnerability_scan_report(
        scan_name=scan_name,
        target=target,
        status=str(normalized["status"]),
        summary=str(normalized["summary"]),
        command=command,
        scanner=resolved_scanner,
        report_format=raw_format,
        raw_report_path=str(resolved_raw_report_path),
        finding_count=int(normalized["finding_count"]),
        affected_component_count=int(normalized["affected_component_count"]),
        generated_at=generated_at,
    )


def validate_vulnerability_scan_report(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["vulnerability scan report must be an object"]

    errors: list[str] = []
    for field in [
        "schema_version",
        "artifact_type",
        "generated_at",
        "scan_name",
        "target",
        "status",
        "summary",
        "command",
        "scanner",
    ]:
        if field not in payload:
            errors.append(f"missing required field: {field}")

    if payload.get("schema_version") != VULNERABILITY_SCAN_REPORT_VERSION:
        errors.append(f"schema_version must be {VULNERABILITY_SCAN_REPORT_VERSION!r}")
    if payload.get("artifact_type") != VULNERABILITY_SCAN_REPORT_ARTIFACT_TYPE:
        errors.append(
            f"artifact_type must be {VULNERABILITY_SCAN_REPORT_ARTIFACT_TYPE!r}"
        )

    for field in [
        "generated_at",
        "scan_name",
        "target",
        "summary",
        "command",
        "scanner",
    ]:
        if field in payload and not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")

    if payload.get("status") not in VULNERABILITY_SCAN_STATUSES:
        errors.append(f"status must be one of {sorted(VULNERABILITY_SCAN_STATUSES)}")

    report_format = payload.get("report_format")
    if (
        report_format is not None
        and report_format not in VULNERABILITY_SCAN_RAW_FORMATS
    ):
        errors.append(
            f"report_format must be null or one of {sorted(VULNERABILITY_SCAN_RAW_FORMATS)}"
        )

    raw_report_path = payload.get("raw_report_path")
    if raw_report_path is not None and not _is_non_empty_string(raw_report_path):
        errors.append("raw_report_path must be null or a non-empty string")

    for field in ["finding_count", "affected_component_count"]:
        value = payload.get(field)
        if value is None:
            continue
        if not isinstance(value, int) or value < 0:
            errors.append(f"{field} must be a non-negative integer")
    return errors


def write_vulnerability_scan_report(
    payload: Mapping[str, Any], path: str | Path
) -> Path:
    errors = validate_vulnerability_scan_report(payload)
    if errors:
        raise ValueError(f"invalid vulnerability scan report: {'; '.join(errors)}")

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def build_vulnerability_exception_report(
    *,
    project_root: str | Path | None = None,
    exceptions: Sequence[Mapping[str, Any]] | None = None,
    generated_at: str | None = None,
    review_window_days: int = DEFAULT_VULNERABILITY_EXCEPTION_REVIEW_WINDOW_DAYS,
) -> dict[str, Any]:
    root = _resolve_project_root(project_root)
    generated_timestamp = generated_at or _now_iso()
    generated_datetime = _parse_iso_datetime(generated_timestamp) or datetime.now()
    if review_window_days < 0:
        raise ValueError("review_window_days must be a non-negative integer")

    normalized_exceptions: list[dict[str, Any]] = []
    for index, raw_item in enumerate(exceptions or [], start=1):
        item = dict(raw_item)
        expires_at = str(item.get("expires_at") or "").strip() or None
        expires_datetime = _parse_iso_datetime(expires_at)
        status = (
            "expired"
            if _datetime_is_before(expires_datetime, generated_datetime)
            else "active"
        )

        normalized_exceptions.append(
            {
                "id": str(item.get("id") or f"exception-{index:03d}"),
                "scope": item.get("scope"),
                "component": item.get("component"),
                "image_refs": _non_empty_strings(item.get("image_refs", [])),
                "current_version": (
                    str(item.get("current_version")).strip()
                    if _is_non_empty_string(item.get("current_version"))
                    else None
                ),
                "vulnerability_ids": _non_empty_strings(
                    item.get("vulnerability_ids", [])
                ),
                "severities": _non_empty_strings(item.get("severities", [])),
                "only_without_fix_version": bool(
                    item.get("only_without_fix_version", False)
                ),
                "justification": item.get("justification"),
                "ticket": (
                    str(item.get("ticket")).strip()
                    if _is_non_empty_string(item.get("ticket"))
                    else None
                ),
                "approved_by": item.get("approved_by"),
                "approved_at": item.get("approved_at"),
                "expires_at": expires_at,
                "status": status,
            }
        )

    review_metrics = _build_vulnerability_exception_review_metrics(
        normalized_exceptions,
        generated_datetime=generated_datetime,
        review_window_days=review_window_days,
    )
    summary = (
        "No vulnerability exceptions are currently recorded."
        if not normalized_exceptions
        else "Vulnerability exception report contains "
        f"{review_metrics['active_exception_count']} active exception(s) and "
        f"{review_metrics['expired_exception_count']} expired exception(s)."
    )
    if normalized_exceptions and review_metrics["review_status"] == "review_due":
        summary += (
            f" {review_metrics['review_due_exception_count']} active exception(s) "
            f"enter the {review_window_days}-day review window before "
            f"{review_metrics['next_exception_expiry']}."
        )
    elif (
        normalized_exceptions
        and review_metrics["review_status"] == "tracked"
        and _is_non_empty_string(review_metrics["next_exception_expiry"])
    ):
        summary += f" Next active exception expiry is {review_metrics['next_exception_expiry']}."
    elif normalized_exceptions and review_metrics["review_status"] == "expired":
        summary += " Expired exceptions must be refreshed or removed before reuse."
    return {
        "schema_version": VULNERABILITY_EXCEPTION_REPORT_VERSION,
        "artifact_type": VULNERABILITY_EXCEPTION_REPORT_ARTIFACT_TYPE,
        "generated_at": generated_timestamp,
        "project_root": str(root),
        "summary": summary,
        "active_exception_count": review_metrics["active_exception_count"],
        "expired_exception_count": review_metrics["expired_exception_count"],
        "review_window_days": review_metrics["review_window_days"],
        "review_due_exception_count": review_metrics["review_due_exception_count"],
        "review_due_exception_ids": review_metrics["review_due_exception_ids"],
        "review_due_exception_components": review_metrics[
            "review_due_exception_components"
        ],
        "review_due_exception_tickets": review_metrics["review_due_exception_tickets"],
        "expired_exception_ids": review_metrics["expired_exception_ids"],
        "next_exception_expiry": review_metrics["next_exception_expiry"],
        "review_status": review_metrics["review_status"],
        "exceptions": to_jsonable(normalized_exceptions),
    }


def validate_vulnerability_exception_report(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["vulnerability exception report must be an object"]

    errors: list[str] = []
    for field in [
        "schema_version",
        "artifact_type",
        "generated_at",
        "project_root",
        "summary",
        "active_exception_count",
        "expired_exception_count",
        "exceptions",
    ]:
        if field not in payload:
            errors.append(f"missing required field: {field}")

    if payload.get("schema_version") != VULNERABILITY_EXCEPTION_REPORT_VERSION:
        errors.append(
            f"schema_version must be {VULNERABILITY_EXCEPTION_REPORT_VERSION!r}"
        )
    if payload.get("artifact_type") != VULNERABILITY_EXCEPTION_REPORT_ARTIFACT_TYPE:
        errors.append(
            "artifact_type must be " f"{VULNERABILITY_EXCEPTION_REPORT_ARTIFACT_TYPE!r}"
        )
    for field in ["generated_at", "project_root", "summary"]:
        if field in payload and not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    if _parse_iso_datetime(payload.get("generated_at")) is None:
        errors.append("generated_at must be a valid ISO 8601 datetime string")

    for field in ["active_exception_count", "expired_exception_count"]:
        value = payload.get(field)
        if not isinstance(value, int) or value < 0:
            errors.append(f"{field} must be a non-negative integer")
    review_window_days = payload.get("review_window_days")
    if review_window_days is not None and (
        not isinstance(review_window_days, int) or review_window_days < 0
    ):
        errors.append("review_window_days must be a non-negative integer")
    review_due_exception_count = payload.get("review_due_exception_count")
    if review_due_exception_count is not None and (
        not isinstance(review_due_exception_count, int)
        or review_due_exception_count < 0
    ):
        errors.append("review_due_exception_count must be a non-negative integer")
    next_exception_expiry = payload.get("next_exception_expiry")
    if (
        next_exception_expiry is not None
        and next_exception_expiry != ""
        and _parse_iso_datetime(str(next_exception_expiry)) is None
    ):
        errors.append(
            "next_exception_expiry must be null or a valid ISO 8601 datetime string"
        )
    review_status = payload.get("review_status")
    if (
        review_status is not None
        and review_status not in VULNERABILITY_EXCEPTION_REVIEW_STATUSES
    ):
        errors.append(
            "review_status must be null or one of "
            f"{sorted(VULNERABILITY_EXCEPTION_REVIEW_STATUSES)}"
        )
    for field in [
        "review_due_exception_ids",
        "review_due_exception_components",
        "review_due_exception_tickets",
        "expired_exception_ids",
    ]:
        values = payload.get(field)
        if values is None:
            continue
        if not isinstance(values, list):
            errors.append(f"{field} must be a list")
            continue
        for index, value in enumerate(values, start=1):
            if not _is_non_empty_string(value):
                errors.append(f"{field}[{index}] must be a non-empty string")

    exceptions = payload.get("exceptions")
    if not isinstance(exceptions, list):
        errors.append("exceptions must be a list")
        return errors

    counted_active = 0
    counted_expired = 0
    for index, item in enumerate(exceptions, start=1):
        prefix = f"exceptions[{index}]"
        if not isinstance(item, Mapping):
            errors.append(f"{prefix} must be an object")
            continue
        for field in [
            "id",
            "scope",
            "component",
            "justification",
            "approved_by",
            "approved_at",
            "expires_at",
            "status",
        ]:
            if not _is_non_empty_string(item.get(field)):
                errors.append(f"{prefix}.{field} must be a non-empty string")
        if item.get("scope") not in VULNERABILITY_EXCEPTION_SCOPES:
            errors.append(
                f"{prefix}.scope must be one of {sorted(VULNERABILITY_EXCEPTION_SCOPES)}"
            )
        if item.get("status") not in VULNERABILITY_EXCEPTION_STATUSES:
            errors.append(
                f"{prefix}.status must be one of {sorted(VULNERABILITY_EXCEPTION_STATUSES)}"
            )
        if item.get("status") == "active":
            counted_active += 1
        elif item.get("status") == "expired":
            counted_expired += 1
        if _parse_iso_datetime(item.get("approved_at")) is None:
            errors.append(
                f"{prefix}.approved_at must be a valid ISO 8601 datetime string"
            )
        expires_datetime = _parse_iso_datetime(item.get("expires_at"))
        if expires_datetime is None:
            errors.append(
                f"{prefix}.expires_at must be a valid ISO 8601 datetime string"
            )

        image_refs = item.get("image_refs", [])
        if not isinstance(image_refs, list):
            errors.append(f"{prefix}.image_refs must be a list")
        else:
            for image_index, image_ref in enumerate(image_refs, start=1):
                if not _is_non_empty_string(image_ref):
                    errors.append(
                        f"{prefix}.image_refs[{image_index}] must be a non-empty string"
                    )
        current_version = item.get("current_version")
        if current_version is not None and not _is_non_empty_string(current_version):
            errors.append(
                f"{prefix}.current_version must be null or a non-empty string"
            )
        for list_name in ["vulnerability_ids", "severities"]:
            values = item.get(list_name, [])
            if not isinstance(values, list):
                errors.append(f"{prefix}.{list_name} must be a list")
                continue
            for value_index, value in enumerate(values, start=1):
                if not _is_non_empty_string(value):
                    errors.append(
                        f"{prefix}.{list_name}[{value_index}] must be a non-empty string"
                    )
        if not isinstance(item.get("only_without_fix_version"), bool):
            errors.append(f"{prefix}.only_without_fix_version must be a boolean")
        ticket = item.get("ticket")
        if ticket is not None and not _is_non_empty_string(ticket):
            errors.append(f"{prefix}.ticket must be null or a non-empty string")

    if (
        isinstance(payload.get("active_exception_count"), int)
        and payload["active_exception_count"] != counted_active
    ):
        errors.append(
            "active_exception_count must equal the number of active exceptions"
        )
    if (
        isinstance(payload.get("expired_exception_count"), int)
        and payload["expired_exception_count"] != counted_expired
    ):
        errors.append(
            "expired_exception_count must equal the number of expired exceptions"
        )
    if (
        not errors
        and isinstance(payload.get("review_window_days"), int)
        and isinstance(payload.get("generated_at"), str)
    ):
        review_metrics = _derive_vulnerability_exception_review_metrics(payload)
        if (
            isinstance(payload.get("review_due_exception_count"), int)
            and payload["review_due_exception_count"]
            != review_metrics["review_due_exception_count"]
        ):
            errors.append(
                "review_due_exception_count must equal the number of active exceptions "
                "inside the review window"
            )
        if (
            payload.get("next_exception_expiry")
            != review_metrics["next_exception_expiry"]
        ):
            errors.append(
                "next_exception_expiry must equal the earliest active exception expiry"
            )
        if payload.get("review_status") != review_metrics["review_status"]:
            errors.append(
                "review_status must match the derived vulnerability exception review status"
            )
        for field in [
            "review_due_exception_ids",
            "review_due_exception_components",
            "review_due_exception_tickets",
            "expired_exception_ids",
        ]:
            if field in payload and payload.get(field) != review_metrics[field]:
                errors.append(f"{field} must match the derived review metrics")
    return errors


def write_vulnerability_exception_report(
    payload: Mapping[str, Any], path: str | Path
) -> Path:
    errors = validate_vulnerability_exception_report(payload)
    if errors:
        raise ValueError(f"invalid vulnerability exception report: {'; '.join(errors)}")

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def _non_empty_strings(items: Sequence[Any]) -> list[str]:
    values: list[str] = []
    for item in items:
        if _is_non_empty_string(item):
            values.append(str(item).strip())
    return sorted(dict.fromkeys(values))


def _severity_score(severity: str | None) -> int:
    order = {
        "CRITICAL": 5,
        "HIGH": 4,
        "MEDIUM": 3,
        "LOW": 2,
        "UNKNOWN": 1,
    }
    return order.get(str(severity or "UNKNOWN").upper(), 0)


def _load_valid_vulnerability_scan_report(path: Path) -> dict[str, Any] | None:
    payload = _load_json(path)
    if payload is None:
        return None
    if validate_vulnerability_scan_report(payload):
        return None
    return payload


def _load_valid_vulnerability_exception_report(path: Path) -> dict[str, Any] | None:
    payload = _load_json(path)
    if payload is None:
        return None
    if validate_vulnerability_exception_report(payload):
        return None
    return payload


def _load_trivy_raw_payloads(raw_path: Path) -> list[dict[str, Any]]:
    if raw_path.is_file():
        payload = _load_json(raw_path)
        if payload is None:
            return []
        annotated_payload = dict(payload)
        annotated_payload["_source_report_name"] = raw_path.stem
        return [annotated_payload]
    if not raw_path.is_dir():
        return []

    payloads: list[dict[str, Any]] = []
    for candidate in sorted(raw_path.glob("*.json")):
        payload = _load_json(candidate)
        if payload is not None:
            annotated_payload = dict(payload)
            annotated_payload["_source_report_name"] = candidate.stem
            payloads.append(annotated_payload)
    return payloads


def _slugify_image_ref(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value)


def _resolve_container_image_ref(
    raw_payload: Mapping[str, Any],
    *,
    structured_scanned_images: Sequence[str] | None = None,
) -> str | None:
    artifact_name = (
        str(raw_payload.get("ArtifactName")).strip()
        if _is_non_empty_string(raw_payload.get("ArtifactName"))
        else None
    )
    if artifact_name and artifact_name != "/scan/image.tar":
        return artifact_name

    source_report_name = (
        str(raw_payload.get("_source_report_name")).strip()
        if _is_non_empty_string(raw_payload.get("_source_report_name"))
        else None
    )
    if source_report_name:
        for image_ref in structured_scanned_images or []:
            if (
                image_ref == source_report_name
                or _slugify_image_ref(image_ref) == source_report_name
            ):
                return str(image_ref)
        return source_report_name

    labels = raw_payload.get("Metadata", {}).get("ImageConfig", {}).get("Labels", {})
    compose_service = (
        str(labels.get("com.docker.compose.service")).strip()
        if isinstance(labels, Mapping)
        and _is_non_empty_string(labels.get("com.docker.compose.service"))
        else None
    )
    if compose_service:
        matching_images = [
            str(image_ref)
            for image_ref in structured_scanned_images or []
            if image_ref == compose_service
            or image_ref.endswith(f"-{compose_service}")
            or compose_service in image_ref
        ]
        if len(matching_images) == 1:
            return matching_images[0]

    if len(structured_scanned_images or []) == 1:
        return str((structured_scanned_images or [None])[0])
    return artifact_name


def _classify_blocked_vulnerability_report(
    payload: Mapping[str, Any] | None,
) -> str | None:
    if not isinstance(payload, Mapping):
        return None
    if payload.get("status") != "blocked":
        return None
    finding_count = payload.get("finding_count", 0)
    if isinstance(finding_count, int) and finding_count > 0:
        return "findings"
    return "execution"


def _build_python_remediation_findings(
    raw_payload: Mapping[str, Any],
) -> list[dict[str, Any]]:
    dependencies = raw_payload.get("dependencies")
    if not isinstance(dependencies, list):
        return []

    findings: list[dict[str, Any]] = []
    for dependency in dependencies:
        if not isinstance(dependency, Mapping):
            continue
        vulnerabilities = (
            dependency.get("vulns") or dependency.get("vulnerabilities") or []
        )
        if not isinstance(vulnerabilities, list) or not vulnerabilities:
            continue

        component_name = str(dependency.get("name") or "")
        if not _is_non_empty_string(component_name):
            continue
        current_version = (
            str(dependency.get("version"))
            if _is_non_empty_string(dependency.get("version"))
            else None
        )
        for vulnerability in vulnerabilities:
            if not isinstance(vulnerability, Mapping):
                continue
            recommended_fix_versions = _non_empty_strings(
                vulnerability.get("fix_versions", [])
            )
            findings.append(
                {
                    "scope": "python_dependencies",
                    "component": component_name,
                    "current_version": current_version,
                    "image_ref": None,
                    "vulnerability_id": vulnerability.get("id"),
                    "severity": (
                        str(vulnerability.get("severity")).upper()
                        if _is_non_empty_string(vulnerability.get("severity"))
                        else None
                    ),
                    "recommended_fix_versions": recommended_fix_versions,
                }
            )
    return findings


def _build_container_remediation_findings(
    raw_payloads: Sequence[Mapping[str, Any]],
    *,
    structured_scanned_images: Sequence[str] | None = None,
) -> tuple[list[dict[str, Any]], list[str]]:
    findings: list[dict[str, Any]] = []
    scanned_images: set[str] = set()

    for raw_payload in raw_payloads:
        resolved_image_ref = _resolve_container_image_ref(
            raw_payload,
            structured_scanned_images=structured_scanned_images,
        )
        if _is_non_empty_string(resolved_image_ref):
            scanned_images.add(str(resolved_image_ref))

        results = raw_payload.get("Results") or raw_payload.get("results") or []
        if not isinstance(results, list):
            continue

        for result in results:
            if not isinstance(result, Mapping):
                continue
            vulnerabilities = (
                result.get("Vulnerabilities") or result.get("vulnerabilities") or []
            )
            if not isinstance(vulnerabilities, list):
                continue

            for vulnerability in vulnerabilities:
                if not isinstance(vulnerability, Mapping):
                    continue
                package_name = vulnerability.get("PkgName") or vulnerability.get(
                    "pkgName"
                )
                if not _is_non_empty_string(package_name):
                    continue
                installed_version = vulnerability.get("InstalledVersion")
                fixed_version = vulnerability.get("FixedVersion")
                vulnerability_id = vulnerability.get(
                    "VulnerabilityID"
                ) or vulnerability.get("vulnerabilityID")
                findings.append(
                    {
                        "scope": "container_images",
                        "component": str(package_name),
                        "current_version": (
                            str(installed_version)
                            if _is_non_empty_string(installed_version)
                            else None
                        ),
                        "image_ref": resolved_image_ref,
                        "vulnerability_id": vulnerability_id,
                        "severity": str(
                            vulnerability.get("Severity") or "UNKNOWN"
                        ).upper(),
                        "recommended_fix_versions": (
                            [str(fixed_version)]
                            if _is_non_empty_string(fixed_version)
                            else []
                        ),
                    }
                )

    return findings, sorted(scanned_images)


def _finding_matches_vulnerability_exception(
    finding: Mapping[str, Any],
    exception: Mapping[str, Any],
    *,
    enforce_no_fix_version: bool = True,
) -> bool:
    if exception.get("status") != "active":
        return False
    if exception.get("scope") != finding.get("scope"):
        return False
    if str(exception.get("component") or "") != str(finding.get("component") or ""):
        return False

    image_refs = [
        str(item)
        for item in exception.get("image_refs", [])
        if _is_non_empty_string(item)
    ]
    if image_refs:
        if not _is_non_empty_string(finding.get("image_ref")):
            return False
        if str(finding.get("image_ref")) not in image_refs:
            return False

    current_version = exception.get("current_version")
    if _is_non_empty_string(current_version):
        if str(finding.get("current_version") or "") != str(current_version):
            return False

    vulnerability_ids = [
        str(item)
        for item in exception.get("vulnerability_ids", [])
        if _is_non_empty_string(item)
    ]
    if vulnerability_ids:
        if str(finding.get("vulnerability_id") or "") not in vulnerability_ids:
            return False

    severities = {
        str(item).upper()
        for item in exception.get("severities", [])
        if _is_non_empty_string(item)
    }
    if severities:
        if str(finding.get("severity") or "").upper() not in severities:
            return False

    if (
        enforce_no_fix_version
        and bool(exception.get("only_without_fix_version", False))
        and finding.get("recommended_fix_versions")
    ):
        return False
    return True


def _build_stale_vulnerability_exception_entries(
    findings: Sequence[Mapping[str, Any]],
    exception_payload: Mapping[str, Any] | None,
) -> list[dict[str, Any]]:
    active_exceptions = [
        dict(item)
        for item in (exception_payload or {}).get("exceptions", [])
        if isinstance(item, Mapping) and item.get("status") == "active"
    ]
    stale_entries_by_id: dict[str, dict[str, Any]] = {}
    stale_recommended_fix_versions: dict[str, set[str]] = {}
    stale_vulnerability_ids: dict[str, set[str]] = {}

    for raw_finding in findings:
        finding = dict(raw_finding)
        recommended_fix_versions = sorted(
            {
                str(item).strip()
                for item in finding.get("recommended_fix_versions", [])
                if _is_non_empty_string(item)
            }
        )
        if not recommended_fix_versions:
            continue

        for exception in active_exceptions:
            if not bool(exception.get("only_without_fix_version", False)):
                continue
            if not _finding_matches_vulnerability_exception(
                finding,
                exception,
                enforce_no_fix_version=False,
            ):
                continue

            exception_id = str(exception.get("id") or "").strip()
            if not exception_id:
                continue

            entry = stale_entries_by_id.setdefault(
                exception_id,
                {
                    "id": exception_id,
                    "scope": str(exception.get("scope") or ""),
                    "component": str(exception.get("component") or ""),
                    "current_version": exception.get("current_version"),
                    "image_refs": [
                        str(item)
                        for item in exception.get("image_refs", [])
                        if _is_non_empty_string(item)
                    ],
                    "ticket": exception.get("ticket"),
                    "expires_at": exception.get("expires_at"),
                    "finding_count": 0,
                },
            )
            entry["finding_count"] = int(entry.get("finding_count", 0)) + 1
            stale_recommended_fix_versions.setdefault(exception_id, set()).update(
                recommended_fix_versions
            )
            vulnerability_id = finding.get("vulnerability_id")
            if _is_non_empty_string(vulnerability_id):
                stale_vulnerability_ids.setdefault(exception_id, set()).add(
                    str(vulnerability_id).strip()
                )

    normalized_entries: list[dict[str, Any]] = []
    for exception_id, entry in stale_entries_by_id.items():
        normalized_entries.append(
            {
                **entry,
                "recommended_fix_versions": sorted(
                    stale_recommended_fix_versions.get(exception_id, set())
                ),
                "vulnerability_ids": sorted(
                    stale_vulnerability_ids.get(exception_id, set())
                ),
            }
        )
    normalized_entries.sort(
        key=lambda item: (
            str(item.get("scope") or ""),
            str(item.get("component") or ""),
            str(item.get("id") or ""),
        )
    )
    return normalized_entries


def _apply_vulnerability_exceptions(
    findings: Sequence[Mapping[str, Any]],
    exception_payload: Mapping[str, Any] | None,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], set[str], int]:
    active_exceptions = [
        dict(item)
        for item in (exception_payload or {}).get("exceptions", [])
        if isinstance(item, Mapping) and item.get("status") == "active"
    ]
    expired_exception_count = (
        int(exception_payload.get("expired_exception_count", 0))
        if isinstance(exception_payload, Mapping)
        else 0
    )
    accepted: list[dict[str, Any]] = []
    unresolved: list[dict[str, Any]] = []
    matched_exception_ids: set[str] = set()
    for raw_finding in findings:
        finding = dict(raw_finding)
        matched_exception = next(
            (
                item
                for item in active_exceptions
                if _finding_matches_vulnerability_exception(finding, item)
            ),
            None,
        )
        if matched_exception is None:
            unresolved.append(finding)
            continue
        finding["accepted_exception_id"] = str(matched_exception.get("id") or "")
        accepted.append(finding)
        if _is_non_empty_string(matched_exception.get("id")):
            matched_exception_ids.add(str(matched_exception["id"]))
    return accepted, unresolved, matched_exception_ids, expired_exception_count


def _build_remediation_components_from_findings(
    findings: Sequence[Mapping[str, Any]],
    *,
    scope: str,
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    components: dict[tuple[str, str | None], dict[str, Any]] = {}
    severity_counts: dict[str, int] = {}
    for finding in findings:
        severity = (
            str(finding.get("severity") or "UNKNOWN").upper()
            if scope == "container_images"
            else None
        )
        if severity:
            severity_counts[severity] = severity_counts.get(severity, 0) + 1

        key = (
            str(finding.get("component") or ""),
            (
                str(finding.get("image_ref"))
                if scope == "container_images"
                and _is_non_empty_string(finding.get("image_ref"))
                else None
            ),
        )
        if not _is_non_empty_string(key[0]):
            continue
        entry = components.setdefault(
            key,
            {
                "name": key[0],
                "scope": scope,
                "image_ref": key[1],
                "current_version": None,
                "finding_count": 0,
                "recommended_fix_versions": set(),
                "vulnerability_ids": set(),
                "highest_severity": None,
            },
        )
        entry["finding_count"] += 1
        if _is_non_empty_string(finding.get("current_version")):
            entry["current_version"] = str(finding["current_version"])
        for version in finding.get("recommended_fix_versions", []):
            if _is_non_empty_string(version):
                entry["recommended_fix_versions"].add(str(version))
        if _is_non_empty_string(finding.get("vulnerability_id")):
            entry["vulnerability_ids"].add(str(finding["vulnerability_id"]))
        if severity and _severity_score(severity) > _severity_score(
            entry.get("highest_severity")
        ):
            entry["highest_severity"] = severity

    normalized_components: list[dict[str, Any]] = []
    for item in components.values():
        normalized_components.append(
            {
                "name": item["name"],
                "scope": item["scope"],
                "image_ref": item["image_ref"],
                "current_version": item["current_version"],
                "finding_count": item["finding_count"],
                "recommended_fix_versions": sorted(item["recommended_fix_versions"]),
                "vulnerability_ids": sorted(item["vulnerability_ids"]),
                "highest_severity": item["highest_severity"],
            }
        )

    normalized_components.sort(
        key=lambda item: (
            -int(item["finding_count"]),
            -_severity_score(item.get("highest_severity")),
            str(item.get("image_ref") or ""),
            item["name"],
        )
    )
    return normalized_components, severity_counts


def _build_prioritized_remediation_actions(
    python_components: Sequence[Mapping[str, Any]],
    container_components: Sequence[Mapping[str, Any]],
    *,
    limit: int = 10,
) -> list[dict[str, Any]]:
    def _action_from_component(component: Mapping[str, Any]) -> dict[str, Any]:
        recommended_fix_versions = [
            str(item)
            for item in component.get("recommended_fix_versions", [])
            if _is_non_empty_string(item)
        ]
        recommended_fix_version = (
            recommended_fix_versions[0] if recommended_fix_versions else None
        )
        highest_severity = (
            str(component.get("highest_severity"))
            if _is_non_empty_string(component.get("highest_severity"))
            else None
        )
        finding_count = int(component.get("finding_count", 0))
        priority = "P1"
        if highest_severity in {"CRITICAL", "HIGH"} or finding_count >= 5:
            priority = "P0"
        rationale = (
            f"{component.get('scope')} 上该组件当前有 {finding_count} 个待修复 finding"
        )
        if _is_non_empty_string(component.get("image_ref")):
            rationale += f"，镜像为 {component.get('image_ref')}"
        if highest_severity:
            rationale += f"，最高严重级别为 {highest_severity}"
        return {
            "priority": priority,
            "scope": str(component.get("scope") or ""),
            "component": str(component.get("name") or ""),
            "image_ref": (
                str(component.get("image_ref"))
                if _is_non_empty_string(component.get("image_ref"))
                else None
            ),
            "current_version": component.get("current_version"),
            "finding_count": finding_count,
            "highest_severity": highest_severity,
            "recommended_fix_version": recommended_fix_version,
            "rationale": rationale,
        }

    actions = [
        _action_from_component(component)
        for component in [*python_components[:5], *container_components[:5]]
        if _is_non_empty_string(component.get("name"))
    ]
    actions.sort(
        key=lambda item: (
            0 if item["priority"] == "P0" else 1,
            -int(item["finding_count"]),
            -_severity_score(item.get("highest_severity")),
            item["component"],
        )
    )
    return actions[:limit]


def build_vulnerability_remediation_report(
    *,
    project_root: str | Path | None = None,
    python_vuln_report_path: str | Path | None = None,
    container_vuln_report_path: str | Path | None = None,
    vulnerability_exception_report_path: str | Path | None = None,
    generated_at: str | None = None,
) -> dict[str, Any]:
    root = _resolve_project_root(project_root)
    resolved_python_report_path = _resolve_path(
        python_vuln_report_path or "test_env/security/python_vuln_scan_report.json",
        root,
    )
    resolved_container_report_path = _resolve_path(
        container_vuln_report_path
        or "test_env/security/container_vuln_scan_report.json",
        root,
    )
    resolved_exception_report_path = _resolve_path(
        vulnerability_exception_report_path
        or "test_env/security/vulnerability_exception_report.json",
        root,
    )

    python_report = _load_valid_vulnerability_scan_report(resolved_python_report_path)
    container_report = _load_valid_vulnerability_scan_report(
        resolved_container_report_path
    )
    exception_report = _load_valid_vulnerability_exception_report(
        resolved_exception_report_path
    )

    python_raw_path = (
        _resolve_path(str(python_report.get("raw_report_path")), root)
        if python_report and _is_non_empty_string(python_report.get("raw_report_path"))
        else None
    )
    container_raw_path = (
        _resolve_path(str(container_report.get("raw_report_path")), root)
        if container_report
        and _is_non_empty_string(container_report.get("raw_report_path"))
        else None
    )

    python_raw_payload = (
        _load_json(python_raw_path)
        if python_raw_path and python_raw_path.is_file()
        else None
    )
    container_raw_payloads = (
        _load_trivy_raw_payloads(container_raw_path)
        if container_raw_path is not None
        else []
    )

    python_findings_raw = (
        _build_python_remediation_findings(python_raw_payload)
        if python_raw_payload is not None
        else []
    )
    container_findings_raw, container_scanned_images = (
        _build_container_remediation_findings(
            container_raw_payloads,
            structured_scanned_images=(
                container_report.get("scanned_images", [])
                if isinstance(container_report, Mapping)
                else []
            ),
        )
        if container_raw_payloads
        else ([], [])
    )

    (
        accepted_python_findings,
        unresolved_python_findings,
        matched_exception_ids,
        expired_exception_count,
    ) = _apply_vulnerability_exceptions(python_findings_raw, exception_report)
    (
        accepted_container_findings,
        unresolved_container_findings,
        matched_container_exception_ids,
        _,
    ) = _apply_vulnerability_exceptions(container_findings_raw, exception_report)
    matched_exception_ids.update(matched_container_exception_ids)
    stale_exception_entries = _build_stale_vulnerability_exception_entries(
        [*python_findings_raw, *container_findings_raw],
        exception_report,
    )
    stale_exception_ids = [str(item["id"]) for item in stale_exception_entries]
    stale_exception_count = len(stale_exception_entries)

    python_components, _ = _build_remediation_components_from_findings(
        unresolved_python_findings,
        scope="python_dependencies",
    )
    container_components, container_severity_counts = (
        _build_remediation_components_from_findings(
            unresolved_container_findings,
            scope="container_images",
        )
    )

    missing_inputs: list[str] = []
    if python_report is None:
        missing_inputs.append("python_vuln_scan_report")
    elif python_report.get("status") == "blocked" and python_raw_payload is None:
        missing_inputs.append("python_vuln_scan_raw_report")
    if container_report is None:
        missing_inputs.append("container_vuln_scan_report")
    elif container_report.get("status") == "blocked" and not container_raw_payloads:
        missing_inputs.append("container_vuln_scan_raw_report")

    python_findings = int(python_report.get("finding_count", 0)) if python_report else 0
    container_findings = (
        int(container_report.get("finding_count", 0)) if container_report else 0
    )
    accepted_finding_count = len(accepted_python_findings) + len(
        accepted_container_findings
    )
    unresolved_finding_count = len(unresolved_python_findings) + len(
        unresolved_container_findings
    )

    actionable_components_available = bool(python_components or container_components)
    if python_report is None or container_report is None:
        remediation_status = "blocked"
        summary = "Vulnerability remediation report is blocked until both structured vulnerability scan reports are available."
    elif unresolved_finding_count == 0 and not missing_inputs:
        remediation_status = "ready"
        if accepted_finding_count:
            summary = "No remediation actions are pending because the remaining findings are covered by active vulnerability exceptions."
        else:
            summary = "No remediation actions are pending because both vulnerability scans passed."
        if stale_exception_count:
            summary += (
                f" {stale_exception_count} active no-fix exception(s) are stale because "
                "matching findings now advertise fix versions."
            )
    elif actionable_components_available:
        remediation_status = "needs_remediation"
        if missing_inputs:
            summary = (
                "Remediation planning is partially available. Some scanner raw payloads are still missing, "
                f"but {unresolved_finding_count} unresolved finding(s) already have actionable remediation data."
            )
        else:
            summary = (
                "Remediation planning is required for "
                f"{unresolved_finding_count} unresolved finding(s) across Python dependencies "
                "and container images."
            )
            if accepted_finding_count:
                summary += f" {accepted_finding_count} finding(s) are already covered by active vulnerability exceptions."
        if stale_exception_count:
            summary += (
                f" {stale_exception_count} active no-fix exception(s) are stale because "
                "matching findings now advertise fix versions."
            )
    else:
        remediation_status = "blocked"
        summary = "Vulnerability remediation report is blocked until raw scanner payloads are available for the blocked scans."
        if stale_exception_count:
            summary += (
                f" {stale_exception_count} active no-fix exception(s) are already stale "
                "because matching findings advertise fix versions."
            )

    prioritized_actions = (
        _build_prioritized_remediation_actions(python_components, container_components)
        if remediation_status == "needs_remediation"
        else []
    )
    next_actions: list[str] = []
    if remediation_status == "blocked":
        next_actions.append(
            "refresh structured vulnerability scan reports and keep raw scanner payloads so remediation planning stays reproducible"
        )
    elif remediation_status == "needs_remediation":
        if missing_inputs:
            next_actions.append(
                "refresh the missing raw scanner payloads so remediation planning covers every blocked scan"
            )
        if stale_exception_count:
            stale_preview = ", ".join(stale_exception_ids[:3])
            if stale_exception_count > 3:
                stale_preview += ", ..."
            next_actions.append(
                "remove or replace stale no-fix vulnerability exceptions in the approved exception input source "
                f"before relying on them again ({stale_exception_count} impacted"
                + (f": {stale_preview}" if stale_preview else "")
                + ")"
            )
        next_actions.append(
            "upgrade the highest-priority Python dependencies and rebuild the distributed/web images with patched packages"
        )
        next_actions.append(
            "re-run python/container vulnerability scans and refresh security_release_preflight after remediation"
        )
    elif stale_exception_count:
        stale_preview = ", ".join(stale_exception_ids[:3])
        if stale_exception_count > 3:
            stale_preview += ", ..."
        next_actions.append(
            "remove or replace stale no-fix vulnerability exceptions in the approved exception input source "
            f"({stale_exception_count} impacted"
            + (f": {stale_preview}" if stale_preview else "")
            + ")"
        )
    if accepted_finding_count:
        next_actions.append(
            "review active vulnerability exceptions before expiry and remove them once upstream fixes are available"
        )
    if expired_exception_count:
        next_actions.append(
            "refresh or remove expired vulnerability exceptions before relying on them in security posture reviews"
        )

    return {
        "schema_version": VULNERABILITY_REMEDIATION_REPORT_VERSION,
        "artifact_type": VULNERABILITY_REMEDIATION_REPORT_ARTIFACT_TYPE,
        "generated_at": generated_at or _now_iso(),
        "project_root": str(root),
        "remediation_status": remediation_status,
        "summary": summary,
        "python_dependencies": {
            "report_path": str(resolved_python_report_path),
            "raw_report_path": (
                str(python_raw_path) if python_raw_path is not None else None
            ),
            "status": python_report.get("status") if python_report else "not_run",
            "finding_count": python_findings,
            "accepted_finding_count": len(accepted_python_findings),
            "unresolved_finding_count": len(unresolved_python_findings),
            "affected_component_count": (
                int(python_report.get("affected_component_count", 0))
                if python_report
                else 0
            ),
            "top_components": to_jsonable(python_components[:10]),
        },
        "container_images": {
            "report_path": str(resolved_container_report_path),
            "raw_report_path": (
                str(container_raw_path) if container_raw_path is not None else None
            ),
            "status": container_report.get("status") if container_report else "not_run",
            "finding_count": container_findings,
            "accepted_finding_count": len(accepted_container_findings),
            "unresolved_finding_count": len(unresolved_container_findings),
            "affected_component_count": (
                int(container_report.get("affected_component_count", 0))
                if container_report
                else 0
            ),
            "severity_counts": to_jsonable(container_severity_counts),
            "scanned_images": to_jsonable(container_scanned_images),
            "top_components": to_jsonable(container_components[:10]),
        },
        "vulnerability_exception_report": {
            "path": str(resolved_exception_report_path),
            "exists": exception_report is not None,
            "active_exception_count": (
                int(exception_report.get("active_exception_count", 0))
                if exception_report
                else 0
            ),
            "expired_exception_count": expired_exception_count,
            "matched_exception_count": len(matched_exception_ids),
            "stale_exception_count": stale_exception_count,
            "stale_exception_ids": stale_exception_ids,
            "stale_exceptions": to_jsonable(stale_exception_entries),
        },
        "accepted_finding_count": accepted_finding_count,
        "unresolved_finding_count": unresolved_finding_count,
        "prioritized_actions": to_jsonable(prioritized_actions),
        "missing_inputs": missing_inputs,
        "next_actions": next_actions,
    }


def validate_vulnerability_remediation_report(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["vulnerability remediation report must be an object"]

    errors: list[str] = []
    for field in [
        "schema_version",
        "artifact_type",
        "generated_at",
        "project_root",
        "remediation_status",
        "summary",
        "python_dependencies",
        "container_images",
        "vulnerability_exception_report",
        "accepted_finding_count",
        "unresolved_finding_count",
        "prioritized_actions",
        "missing_inputs",
        "next_actions",
    ]:
        if field not in payload:
            errors.append(f"missing required field: {field}")

    if payload.get("schema_version") != VULNERABILITY_REMEDIATION_REPORT_VERSION:
        errors.append(
            f"schema_version must be {VULNERABILITY_REMEDIATION_REPORT_VERSION!r}"
        )
    if payload.get("artifact_type") != VULNERABILITY_REMEDIATION_REPORT_ARTIFACT_TYPE:
        errors.append(
            f"artifact_type must be {VULNERABILITY_REMEDIATION_REPORT_ARTIFACT_TYPE!r}"
        )
    for field in ["generated_at", "project_root", "summary"]:
        if field in payload and not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    if payload.get("remediation_status") not in VULNERABILITY_REMEDIATION_STATUSES:
        errors.append(
            f"remediation_status must be one of {sorted(VULNERABILITY_REMEDIATION_STATUSES)}"
        )

    for field in ["python_dependencies", "container_images"]:
        section = payload.get(field)
        if not isinstance(section, Mapping):
            errors.append(f"{field} must be an object")
            continue
        for key in [
            "report_path",
            "status",
            "finding_count",
            "affected_component_count",
            "top_components",
        ]:
            if key not in section:
                errors.append(f"{field}.{key} is required")
        if "report_path" in section and not _is_non_empty_string(
            section.get("report_path")
        ):
            errors.append(f"{field}.report_path must be a non-empty string")
        raw_report_path = section.get("raw_report_path")
        if raw_report_path is not None and not _is_non_empty_string(raw_report_path):
            errors.append(f"{field}.raw_report_path must be null or a non-empty string")
        if section.get("status") not in VULNERABILITY_SCAN_STATUSES:
            errors.append(
                f"{field}.status must be one of {sorted(VULNERABILITY_SCAN_STATUSES)}"
            )
        for key in [
            "finding_count",
            "accepted_finding_count",
            "unresolved_finding_count",
            "affected_component_count",
        ]:
            value = section.get(key)
            if not isinstance(value, int) or value < 0:
                errors.append(f"{field}.{key} must be a non-negative integer")
        top_components = section.get("top_components")
        if not isinstance(top_components, list):
            errors.append(f"{field}.top_components must be a list")
        else:
            for index, item in enumerate(top_components, start=1):
                prefix = f"{field}.top_components[{index}]"
                if not isinstance(item, Mapping):
                    errors.append(f"{prefix} must be an object")
                    continue
                for key in [
                    "name",
                    "scope",
                    "finding_count",
                    "recommended_fix_versions",
                    "vulnerability_ids",
                ]:
                    if key not in item:
                        errors.append(f"{prefix}.{key} is required")
                if not _is_non_empty_string(item.get("name")):
                    errors.append(f"{prefix}.name must be a non-empty string")
                if item.get("scope") not in {"python_dependencies", "container_images"}:
                    errors.append(
                        f"{prefix}.scope must be 'python_dependencies' or 'container_images'"
                    )
                if (
                    not isinstance(item.get("finding_count"), int)
                    or item.get("finding_count") < 0
                ):
                    errors.append(
                        f"{prefix}.finding_count must be a non-negative integer"
                    )
                for key in ["recommended_fix_versions", "vulnerability_ids"]:
                    values = item.get(key)
                    if not isinstance(values, list):
                        errors.append(f"{prefix}.{key} must be a list")
                    else:
                        for value_index, value in enumerate(values, start=1):
                            if not _is_non_empty_string(value):
                                errors.append(
                                    f"{prefix}.{key}[{value_index}] must be a non-empty string"
                                )
        if field == "container_images":
            severity_counts = section.get("severity_counts")
            if not isinstance(severity_counts, Mapping):
                errors.append("container_images.severity_counts must be an object")
            scanned_images = section.get("scanned_images")
            if not isinstance(scanned_images, list):
                errors.append("container_images.scanned_images must be a list")

    exception_report = payload.get("vulnerability_exception_report")
    if not isinstance(exception_report, Mapping):
        errors.append("vulnerability_exception_report must be an object")
    else:
        if not _is_non_empty_string(exception_report.get("path")):
            errors.append(
                "vulnerability_exception_report.path must be a non-empty string"
            )
        if not isinstance(exception_report.get("exists"), bool):
            errors.append("vulnerability_exception_report.exists must be a boolean")
        for key in [
            "active_exception_count",
            "expired_exception_count",
            "matched_exception_count",
            "stale_exception_count",
        ]:
            value = exception_report.get(key)
            if not isinstance(value, int) or value < 0:
                errors.append(
                    f"vulnerability_exception_report.{key} must be a non-negative integer"
                )
        stale_exception_ids = exception_report.get("stale_exception_ids")
        if not isinstance(stale_exception_ids, list):
            errors.append(
                "vulnerability_exception_report.stale_exception_ids must be a list"
            )
        else:
            for index, value in enumerate(stale_exception_ids, start=1):
                if not _is_non_empty_string(value):
                    errors.append(
                        "vulnerability_exception_report.stale_exception_ids"
                        f"[{index}] must be a non-empty string"
                    )
        stale_exceptions = exception_report.get("stale_exceptions")
        if not isinstance(stale_exceptions, list):
            errors.append(
                "vulnerability_exception_report.stale_exceptions must be a list"
            )
        else:
            for index, item in enumerate(stale_exceptions, start=1):
                prefix = f"vulnerability_exception_report.stale_exceptions[{index}]"
                if not isinstance(item, Mapping):
                    errors.append(f"{prefix} must be an object")
                    continue
                for key in [
                    "id",
                    "scope",
                    "component",
                    "finding_count",
                    "recommended_fix_versions",
                    "vulnerability_ids",
                ]:
                    if key not in item:
                        errors.append(f"{prefix}.{key} is required")
                for key in ["id", "scope", "component"]:
                    if not _is_non_empty_string(item.get(key)):
                        errors.append(f"{prefix}.{key} must be a non-empty string")
                current_version = item.get("current_version")
                if current_version is not None and not _is_non_empty_string(
                    current_version
                ):
                    errors.append(
                        f"{prefix}.current_version must be null or a non-empty string"
                    )
                ticket = item.get("ticket")
                if ticket is not None and not _is_non_empty_string(ticket):
                    errors.append(f"{prefix}.ticket must be null or a non-empty string")
                expires_at = item.get("expires_at")
                if expires_at is not None and not _is_non_empty_string(expires_at):
                    errors.append(
                        f"{prefix}.expires_at must be null or a non-empty string"
                    )
                if (
                    not isinstance(item.get("finding_count"), int)
                    or item.get("finding_count") < 0
                ):
                    errors.append(
                        f"{prefix}.finding_count must be a non-negative integer"
                    )
                for key in [
                    "image_refs",
                    "recommended_fix_versions",
                    "vulnerability_ids",
                ]:
                    values = item.get(key)
                    if not isinstance(values, list):
                        errors.append(f"{prefix}.{key} must be a list")
                    else:
                        for value_index, value in enumerate(values, start=1):
                            if not _is_non_empty_string(value):
                                errors.append(
                                    f"{prefix}.{key}[{value_index}] must be a non-empty string"
                                )

    for key in ["accepted_finding_count", "unresolved_finding_count"]:
        value = payload.get(key)
        if not isinstance(value, int) or value < 0:
            errors.append(f"{key} must be a non-negative integer")

    prioritized_actions = payload.get("prioritized_actions")
    if not isinstance(prioritized_actions, list):
        errors.append("prioritized_actions must be a list")
    else:
        for index, item in enumerate(prioritized_actions, start=1):
            prefix = f"prioritized_actions[{index}]"
            if not isinstance(item, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            if item.get("priority") not in {"P0", "P1", "P2"}:
                errors.append(f"{prefix}.priority must be one of ['P0', 'P1', 'P2']")
            for key in ["scope", "component", "finding_count", "rationale"]:
                if key not in item:
                    errors.append(f"{prefix}.{key} is required")
            if not _is_non_empty_string(item.get("scope")):
                errors.append(f"{prefix}.scope must be a non-empty string")
            if not _is_non_empty_string(item.get("component")):
                errors.append(f"{prefix}.component must be a non-empty string")
            if (
                not isinstance(item.get("finding_count"), int)
                or item.get("finding_count") < 0
            ):
                errors.append(f"{prefix}.finding_count must be a non-negative integer")
            if not _is_non_empty_string(item.get("rationale")):
                errors.append(f"{prefix}.rationale must be a non-empty string")

    for field in ["missing_inputs", "next_actions"]:
        values = payload.get(field)
        if not isinstance(values, list):
            errors.append(f"{field} must be a list")
            continue
        for index, value in enumerate(values, start=1):
            if not _is_non_empty_string(value):
                errors.append(f"{field}[{index}] must be a non-empty string")

    return errors


def write_vulnerability_remediation_report(
    payload: Mapping[str, Any], path: str | Path
) -> Path:
    errors = validate_vulnerability_remediation_report(payload)
    if errors:
        raise ValueError(
            f"invalid vulnerability remediation report: {'; '.join(errors)}"
        )

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def default_security_baseline_documents() -> list[dict[str, Any]]:
    return [
        {
            "name": "security_baseline",
            "path": "docs/guides/SECURITY_BASELINE.md",
            "required": True,
        },
        {
            "name": "audit_trail_policy",
            "path": "docs/guides/AUDIT_TRAIL_POLICY.md",
            "required": True,
        },
        {
            "name": "backup_restore_runbook",
            "path": "docs/guides/BACKUP_RESTORE_RUNBOOK.md",
            "required": True,
        },
        {
            "name": "incident_response_matrix",
            "path": "docs/guides/INCIDENT_RESPONSE_MATRIX.md",
            "required": True,
        },
    ]


def _resolve_path(path: str | Path, project_root: Path) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return project_root / candidate


def _load_json(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    if isinstance(payload, Mapping):
        return dict(payload)
    return None


def _collect_relative_file_fingerprints(path: Path) -> dict[str, int] | None:
    if path.is_file():
        return {path.name: path.stat().st_size}
    if not path.is_dir():
        return None

    fingerprints: dict[str, int] = {}
    for file_path in sorted(item for item in path.rglob("*") if item.is_file()):
        fingerprints[str(file_path.relative_to(path)).replace("\\", "/")] = (
            file_path.stat().st_size
        )
    return fingerprints


def _paths_match(source: Path, target: Path) -> bool:
    source_fingerprints = _collect_relative_file_fingerprints(source)
    target_fingerprints = _collect_relative_file_fingerprints(target)
    if source_fingerprints is None or target_fingerprints is None:
        return False
    return source_fingerprints == target_fingerprints


def build_backup_restore_rehearsal_report(
    *,
    project_root: str | Path | None = None,
    actor: str,
    source_runtime_root: str | Path,
    source_config_root: str | Path,
    backup_snapshot_root: str | Path,
    restored_runtime_root: str | Path,
    restored_config_root: str | Path,
    release_manifest_path: str | Path | None = None,
    rehearsal_duration_seconds: float | None = None,
    generated_at: str | None = None,
) -> dict[str, Any]:
    root = _resolve_project_root(project_root)
    source_runtime = _resolve_path(source_runtime_root, root)
    source_config = _resolve_path(source_config_root, root)
    backup_snapshot = _resolve_path(backup_snapshot_root, root)
    restored_runtime = _resolve_path(restored_runtime_root, root)
    restored_config = _resolve_path(restored_config_root, root)
    release_manifest = (
        _resolve_path(release_manifest_path, root)
        if release_manifest_path is not None
        else None
    )

    backup_specs = [
        ("db", source_runtime / "db", backup_snapshot / "db"),
        (
            "workflow_runs",
            source_runtime / "workflow_runs",
            backup_snapshot / "workflow_runs",
        ),
        (
            "workflow_archive",
            source_runtime / "workflow_archive",
            backup_snapshot / "workflow_archive",
        ),
        ("backups", source_runtime / "backups", backup_snapshot / "backups"),
        ("compose_env", source_config / "compose.env", backup_snapshot / "compose.env"),
        (
            "web_panel_env",
            source_config / "web_panel.env",
            backup_snapshot / "web_panel.env",
        ),
    ]

    backup_items: list[dict[str, Any]] = []
    missing_backup_items = 0
    for name, source_path, backup_path in backup_specs:
        exists = backup_path.exists()
        if not exists:
            missing_backup_items += 1
        backup_items.append(
            {
                "name": name,
                "source_path": str(source_path),
                "backup_path": str(backup_path),
                "required": True,
                "exists": exists,
            }
        )

    restore_specs = [
        ("db", source_runtime / "db", restored_runtime / "db"),
        (
            "workflow_runs",
            source_runtime / "workflow_runs",
            restored_runtime / "workflow_runs",
        ),
        (
            "workflow_archive",
            source_runtime / "workflow_archive",
            restored_runtime / "workflow_archive",
        ),
        ("backups", source_runtime / "backups", restored_runtime / "backups"),
        ("compose_env", source_config / "compose.env", restored_config / "compose.env"),
        (
            "web_panel_env",
            source_config / "web_panel.env",
            restored_config / "web_panel.env",
        ),
    ]

    restore_checks: list[dict[str, Any]] = []
    failed_restore_checks = 0
    for name, source_path, restored_path in restore_specs:
        passed = _paths_match(source_path, restored_path)
        if not passed:
            failed_restore_checks += 1
        restore_checks.append(
            {
                "name": name,
                "source_path": str(source_path),
                "restored_path": str(restored_path),
                "required": True,
                "passed": passed,
            }
        )

    next_actions: list[str] = []
    if missing_backup_items:
        next_actions.append(
            "ensure runtime data and environment snapshots are copied into the backup snapshot before declaring the rehearsal complete"
        )
    if failed_restore_checks:
        next_actions.append(
            "re-run the restore flow until restored runtime data and config files match the backed-up source material"
        )

    status = (
        "passed"
        if missing_backup_items == 0 and failed_restore_checks == 0
        else "blocked"
    )
    summary = (
        "Backup and restore rehearsal passed."
        if status == "passed"
        else "Backup and restore rehearsal is blocked until all required items are backed up and restored successfully."
    )

    return {
        "schema_version": BACKUP_RESTORE_REHEARSAL_REPORT_VERSION,
        "artifact_type": BACKUP_RESTORE_REHEARSAL_REPORT_ARTIFACT_TYPE,
        "generated_at": generated_at or _now_iso(),
        "project_root": str(root),
        "actor": actor,
        "status": status,
        "summary": summary,
        "release_manifest_path": (
            str(release_manifest) if release_manifest is not None else None
        ),
        "source_runtime_root": str(source_runtime),
        "source_config_root": str(source_config),
        "backup_snapshot_root": str(backup_snapshot),
        "restored_runtime_root": str(restored_runtime),
        "restored_config_root": str(restored_config),
        "backup_items": to_jsonable(backup_items),
        "restore_checks": to_jsonable(restore_checks),
        "missing_backup_items": missing_backup_items,
        "failed_restore_checks": failed_restore_checks,
        "rpo_target": "24 hours",
        "rto_target": "4 hours",
        "rehearsal_duration_seconds": round(
            float(rehearsal_duration_seconds or 0.0), 6
        ),
        "next_actions": next_actions,
    }


def validate_backup_restore_rehearsal_report(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["backup restore rehearsal report must be an object"]

    errors: list[str] = []
    for field in [
        "schema_version",
        "artifact_type",
        "generated_at",
        "project_root",
        "actor",
        "status",
        "summary",
        "source_runtime_root",
        "source_config_root",
        "backup_snapshot_root",
        "restored_runtime_root",
        "restored_config_root",
        "backup_items",
        "restore_checks",
        "missing_backup_items",
        "failed_restore_checks",
        "rpo_target",
        "rto_target",
        "rehearsal_duration_seconds",
        "next_actions",
    ]:
        if field not in payload:
            errors.append(f"missing required field: {field}")

    if payload.get("schema_version") != BACKUP_RESTORE_REHEARSAL_REPORT_VERSION:
        errors.append(
            f"schema_version must be {BACKUP_RESTORE_REHEARSAL_REPORT_VERSION!r}"
        )
    if payload.get("artifact_type") != BACKUP_RESTORE_REHEARSAL_REPORT_ARTIFACT_TYPE:
        errors.append(
            f"artifact_type must be {BACKUP_RESTORE_REHEARSAL_REPORT_ARTIFACT_TYPE!r}"
        )
    for field in [
        "generated_at",
        "project_root",
        "actor",
        "summary",
        "source_runtime_root",
        "source_config_root",
        "backup_snapshot_root",
        "restored_runtime_root",
        "restored_config_root",
        "rpo_target",
        "rto_target",
    ]:
        if field in payload and not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    if payload.get("status") not in BACKUP_RESTORE_REHEARSAL_STATUSES:
        errors.append(
            f"status must be one of {sorted(BACKUP_RESTORE_REHEARSAL_STATUSES)}"
        )
    release_manifest_path = payload.get("release_manifest_path")
    if release_manifest_path is not None and not _is_non_empty_string(
        release_manifest_path
    ):
        errors.append("release_manifest_path must be null or a non-empty string")

    backup_items = payload.get("backup_items")
    if not isinstance(backup_items, list):
        errors.append("backup_items must be a list")
    else:
        for index, item in enumerate(backup_items, start=1):
            prefix = f"backup_items[{index}]"
            if not isinstance(item, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            for field in ["name", "source_path", "backup_path"]:
                if not _is_non_empty_string(item.get(field)):
                    errors.append(f"{prefix}.{field} must be a non-empty string")
            if not isinstance(item.get("required"), bool):
                errors.append(f"{prefix}.required must be a boolean")
            if not isinstance(item.get("exists"), bool):
                errors.append(f"{prefix}.exists must be a boolean")

    restore_checks = payload.get("restore_checks")
    if not isinstance(restore_checks, list):
        errors.append("restore_checks must be a list")
    else:
        for index, item in enumerate(restore_checks, start=1):
            prefix = f"restore_checks[{index}]"
            if not isinstance(item, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            for field in ["name", "source_path", "restored_path"]:
                if not _is_non_empty_string(item.get(field)):
                    errors.append(f"{prefix}.{field} must be a non-empty string")
            if not isinstance(item.get("required"), bool):
                errors.append(f"{prefix}.required must be a boolean")
            if not isinstance(item.get("passed"), bool):
                errors.append(f"{prefix}.passed must be a boolean")

    for field in ["missing_backup_items", "failed_restore_checks"]:
        value = payload.get(field)
        if not isinstance(value, int) or value < 0:
            errors.append(f"{field} must be a non-negative integer")

    rehearsal_duration_seconds = payload.get("rehearsal_duration_seconds")
    if (
        not isinstance(rehearsal_duration_seconds, (int, float))
        or rehearsal_duration_seconds < 0
    ):
        errors.append("rehearsal_duration_seconds must be a non-negative number")

    next_actions = payload.get("next_actions")
    if not isinstance(next_actions, list):
        errors.append("next_actions must be a list")
    else:
        for index, item in enumerate(next_actions, start=1):
            if not _is_non_empty_string(item):
                errors.append(f"next_actions[{index}] must be a non-empty string")

    return errors


def write_backup_restore_rehearsal_report(
    payload: Mapping[str, Any], path: str | Path
) -> Path:
    errors = validate_backup_restore_rehearsal_report(payload)
    if errors:
        raise ValueError(
            f"invalid backup restore rehearsal report: {'; '.join(errors)}"
        )

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def build_security_posture_report(
    *,
    project_root: str | Path | None = None,
    sbom_path: str | Path | None = None,
    python_vuln_report_path: str | Path | None = None,
    container_vuln_report_path: str | Path | None = None,
    backup_restore_rehearsal_report_path: str | Path | None = None,
    vulnerability_remediation_report_path: str | Path | None = None,
    vulnerability_exception_report_path: str | Path | None = None,
    generated_at: str | None = None,
) -> dict[str, Any]:
    root = _resolve_project_root(project_root)
    resolved_sbom_path = _resolve_path(sbom_path or "test_env/security/sbom.json", root)
    resolved_python_vuln_path = _resolve_path(
        python_vuln_report_path or "test_env/security/python_vuln_scan_report.json",
        root,
    )
    resolved_container_vuln_path = _resolve_path(
        container_vuln_report_path
        or "test_env/security/container_vuln_scan_report.json",
        root,
    )
    resolved_backup_restore_rehearsal_path = _resolve_path(
        backup_restore_rehearsal_report_path
        or "test_env/security/backup_restore_rehearsal_report.json",
        root,
    )
    resolved_vulnerability_remediation_path = _resolve_path(
        vulnerability_remediation_report_path
        or "test_env/security/vulnerability_remediation_report.json",
        root,
    )
    resolved_vulnerability_exception_path = _resolve_path(
        vulnerability_exception_report_path
        or "test_env/security/vulnerability_exception_report.json",
        root,
    )

    sbom_payload = _load_json(resolved_sbom_path)
    sbom_exists = sbom_payload is not None and not validate_sbom_artifact(sbom_payload)

    vuln_reports = []
    missing_vuln_reports = 0
    blocked_vuln_reports = 0
    blocked_vuln_execution_reports = 0
    blocked_vuln_finding_reports = 0
    for scan_name, report_path in [
        ("python_dependencies", resolved_python_vuln_path),
        ("container_images", resolved_container_vuln_path),
    ]:
        payload = _load_json(report_path)
        valid_payload = payload is not None and not validate_vulnerability_scan_report(
            payload
        )
        status = payload.get("status") if valid_payload else "not_run"
        exists = valid_payload
        if not exists:
            missing_vuln_reports += 1
        elif status != "passed":
            blocked_vuln_reports += 1
            blocked_reason = _classify_blocked_vulnerability_report(payload)
            if blocked_reason == "findings":
                blocked_vuln_finding_reports += 1
            elif blocked_reason == "execution":
                blocked_vuln_execution_reports += 1
        vuln_reports.append(
            {
                "name": scan_name,
                "path": str(report_path),
                "required": True,
                "exists": exists,
                "status": status,
                "scanner": payload.get("scanner") if valid_payload else None,
            }
        )

    baseline_documents = []
    missing_documents = 0
    for item in default_security_baseline_documents():
        resolved = _resolve_path(item["path"], root)
        exists = resolved.is_file()
        if item["required"] and not exists:
            missing_documents += 1
        baseline_documents.append(
            {
                "name": item["name"],
                "path": str(resolved),
                "required": item["required"],
                "exists": exists,
            }
        )

    rehearsal_payload = _load_json(resolved_backup_restore_rehearsal_path)
    rehearsal_exists = (
        rehearsal_payload is not None
        and not validate_backup_restore_rehearsal_report(rehearsal_payload)
    )
    rehearsal_status = (
        rehearsal_payload.get("status") if rehearsal_exists else "blocked"
    )
    missing_backup_restore_rehearsal_reports = 0 if rehearsal_exists else 1
    blocked_backup_restore_rehearsal_reports = (
        0 if not rehearsal_exists else 0 if rehearsal_status == "passed" else 1
    )
    remediation_payload = _load_json(resolved_vulnerability_remediation_path)
    remediation_exists = (
        remediation_payload is not None
        and not validate_vulnerability_remediation_report(remediation_payload)
    )
    remediation_status = (
        str(remediation_payload.get("remediation_status"))
        if remediation_exists
        else "missing"
    )
    accepted_vulnerability_findings = (
        int(remediation_payload.get("accepted_finding_count", 0))
        if remediation_exists
        else 0
    )
    unresolved_vulnerability_findings = (
        int(remediation_payload.get("unresolved_finding_count", 0))
        if remediation_exists
        else 0
    )
    remediation_exception_report = (
        remediation_payload.get("vulnerability_exception_report", {})
        if remediation_exists
        else {}
    )
    stale_exception_count = (
        int(remediation_exception_report.get("stale_exception_count", 0))
        if isinstance(remediation_exception_report, Mapping)
        else 0
    )
    stale_exception_ids = (
        [
            str(item).strip()
            for item in remediation_exception_report.get("stale_exception_ids", [])
            if _is_non_empty_string(item)
        ]
        if isinstance(remediation_exception_report, Mapping)
        else []
    )
    exception_payload = _load_valid_vulnerability_exception_report(
        resolved_vulnerability_exception_path
    )
    exception_exists = exception_payload is not None
    active_exception_count = (
        int(exception_payload.get("active_exception_count", 0))
        if exception_payload
        else 0
    )
    expired_exception_count = (
        int(exception_payload.get("expired_exception_count", 0))
        if exception_payload
        else 0
    )
    exception_review_metrics = (
        _derive_vulnerability_exception_review_metrics(exception_payload)
        if exception_payload
        else None
    )
    review_window_days = (
        int(exception_review_metrics["review_window_days"])
        if exception_review_metrics
        else None
    )
    review_due_exception_count = (
        int(exception_review_metrics["review_due_exception_count"])
        if exception_review_metrics
        else 0
    )
    next_exception_expiry = (
        exception_review_metrics.get("next_exception_expiry")
        if exception_review_metrics
        else None
    )
    review_status = (
        str(exception_review_metrics["review_status"])
        if exception_review_metrics
        else None
    )

    effective_blocked_vuln_reports = blocked_vuln_execution_reports
    effective_blocked_vuln_finding_reports = blocked_vuln_finding_reports
    if blocked_vuln_finding_reports and remediation_exists:
        effective_blocked_vuln_finding_reports = (
            0
            if unresolved_vulnerability_findings == 0
            else blocked_vuln_finding_reports
        )
    effective_blocked_vuln_reports += effective_blocked_vuln_finding_reports

    next_actions: list[str] = []
    if not sbom_exists:
        next_actions.append(
            "generate sbom artifact with tools/build_sbom_artifact.py before promoting the release"
        )
    if missing_vuln_reports:
        next_actions.append(
            "attach structured python and container vulnerability scan reports"
        )
    if blocked_vuln_execution_reports:
        next_actions.append(
            "re-run the blocked vulnerability scanners until structured reports are produced successfully"
        )
    if blocked_vuln_finding_reports and not remediation_exists:
        next_actions.append(
            "build vulnerability remediation data before resolving or accepting the remaining findings"
        )
    if stale_exception_count:
        stale_preview = ", ".join(stale_exception_ids[:3])
        if stale_exception_count > 3:
            stale_preview += ", ..."
        next_actions.append(
            "remove or replace stale no-fix vulnerability exceptions now that matching findings publish fix versions"
            + (
                f" ({stale_exception_count} impacted: {stale_preview})"
                if stale_preview
                else f" ({stale_exception_count} impacted)"
            )
        )
    if effective_blocked_vuln_finding_reports:
        next_actions.append(
            "resolve reported vulnerability scan findings before marking the security posture ready"
        )
    elif accepted_vulnerability_findings and review_status == "review_due":
        next_actions.append(
            "review approved vulnerability exceptions that are now inside the review window "
            f"and replace them before {next_exception_expiry}"
        )
    elif accepted_vulnerability_findings:
        next_actions.append(
            "track approved vulnerability exceptions until upstream fixes are available and remove them before expiry"
        )
    if expired_exception_count:
        next_actions.append(
            "refresh or remove expired vulnerability exceptions before relying on them in future security posture reviews"
        )
    if missing_documents:
        next_actions.append(
            "complete the required security baseline documents under docs/guides/"
        )
    if missing_backup_restore_rehearsal_reports:
        next_actions.append("attach a structured backup and restore rehearsal report")
    if blocked_backup_restore_rehearsal_reports:
        next_actions.append(
            "resolve blocked backup and restore rehearsal checks before marking the security posture ready"
        )

    posture_status = (
        "ready"
        if sbom_exists
        and missing_vuln_reports == 0
        and effective_blocked_vuln_reports == 0
        and missing_documents == 0
        and missing_backup_restore_rehearsal_reports == 0
        and blocked_backup_restore_rehearsal_reports == 0
        else "blocked"
    )
    if (
        posture_status == "ready"
        and accepted_vulnerability_findings
        and review_status == "review_due"
    ):
        summary = (
            "Security posture is ready with approved vulnerability exceptions covering the remaining reported findings, "
            f"but {review_due_exception_count} active exception(s) enter the "
            f"{review_window_days}-day review window before {next_exception_expiry}."
        )
    elif posture_status == "ready" and stale_exception_count:
        summary = (
            "Security posture is ready, but some active no-fix vulnerability exceptions are stale "
            "because matching findings now advertise fix versions."
        )
    elif posture_status == "ready":
        summary = "Security posture is ready."
    elif blocked_vuln_execution_reports and blocked_vuln_finding_reports:
        summary = (
            "Security posture remains blocked until required vulnerability scans execute successfully "
            "and the remaining reported findings are remediated."
        )
    elif blocked_vuln_execution_reports:
        summary = "Security posture remains blocked until the required vulnerability scans execute successfully."
    elif effective_blocked_vuln_finding_reports and stale_exception_count:
        summary = (
            "Security posture remains blocked because some active no-fix vulnerability exceptions are stale "
            "and the matching findings now advertise fix versions."
        )
    elif effective_blocked_vuln_finding_reports:
        summary = "Security posture remains blocked until the reported vulnerability findings are remediated."
    elif accepted_vulnerability_findings:
        summary = "Security posture is ready with approved vulnerability exceptions covering the remaining reported findings."
    else:
        summary = (
            "Security posture remains blocked until SBOM, vulnerability scans, backup and restore rehearsal, "
            "and required baseline documents are all present."
        )

    return {
        "schema_version": SECURITY_POSTURE_REPORT_VERSION,
        "artifact_type": SECURITY_POSTURE_REPORT_ARTIFACT_TYPE,
        "generated_at": generated_at or _now_iso(),
        "project_root": str(root),
        "posture_status": posture_status,
        "summary": summary,
        "sbom": {
            "path": str(resolved_sbom_path),
            "exists": sbom_exists,
            "component_count": (
                sbom_payload.get("component_count", 0) if sbom_exists else 0
            ),
        },
        "vulnerability_reports": to_jsonable(vuln_reports),
        "baseline_documents": to_jsonable(baseline_documents),
        "backup_restore_rehearsal": {
            "path": str(resolved_backup_restore_rehearsal_path),
            "exists": rehearsal_exists,
            "status": rehearsal_status,
            "rehearsal_duration_seconds": (
                rehearsal_payload.get("rehearsal_duration_seconds", 0.0)
                if rehearsal_exists
                else 0.0
            ),
        },
        "vulnerability_remediation": {
            "path": str(resolved_vulnerability_remediation_path),
            "exists": remediation_exists,
            "status": remediation_status,
            "accepted_finding_count": accepted_vulnerability_findings,
            "unresolved_finding_count": unresolved_vulnerability_findings,
            "stale_exception_count": stale_exception_count,
        },
        "vulnerability_exception_report": {
            "path": str(resolved_vulnerability_exception_path),
            "exists": exception_exists,
            "active_exception_count": active_exception_count,
            "expired_exception_count": expired_exception_count,
            "stale_exception_count": stale_exception_count,
            "stale_exception_ids": stale_exception_ids,
            **(
                {
                    "review_window_days": review_window_days,
                    "review_due_exception_count": review_due_exception_count,
                    "review_due_exception_ids": exception_payload.get(
                        "review_due_exception_ids",
                        [],
                    ),
                    "review_due_exception_components": exception_payload.get(
                        "review_due_exception_components",
                        [],
                    ),
                    "review_due_exception_tickets": exception_payload.get(
                        "review_due_exception_tickets",
                        [],
                    ),
                    "expired_exception_ids": exception_payload.get(
                        "expired_exception_ids",
                        [],
                    ),
                    "next_exception_expiry": next_exception_expiry,
                    "review_status": review_status,
                }
                if exception_exists
                else {}
            ),
        },
        "missing_vulnerability_reports": missing_vuln_reports,
        "blocked_vulnerability_reports": effective_blocked_vuln_reports,
        "blocked_vulnerability_execution_reports": blocked_vuln_execution_reports,
        "blocked_vulnerability_finding_reports": effective_blocked_vuln_finding_reports,
        "accepted_vulnerability_findings": accepted_vulnerability_findings,
        "unresolved_vulnerability_findings": unresolved_vulnerability_findings,
        "missing_documents": missing_documents,
        "missing_backup_restore_rehearsal_reports": (
            missing_backup_restore_rehearsal_reports
        ),
        "blocked_backup_restore_rehearsal_reports": (
            blocked_backup_restore_rehearsal_reports
        ),
        "next_actions": next_actions,
    }


def validate_security_posture_report(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["security posture report must be an object"]

    errors: list[str] = []
    for field in [
        "schema_version",
        "artifact_type",
        "generated_at",
        "project_root",
        "posture_status",
        "summary",
        "sbom",
        "vulnerability_reports",
        "baseline_documents",
        "backup_restore_rehearsal",
        "missing_vulnerability_reports",
        "blocked_vulnerability_reports",
        "next_actions",
    ]:
        if field not in payload:
            errors.append(f"missing required field: {field}")

    if payload.get("schema_version") != SECURITY_POSTURE_REPORT_VERSION:
        errors.append(f"schema_version must be {SECURITY_POSTURE_REPORT_VERSION!r}")
    if payload.get("artifact_type") != SECURITY_POSTURE_REPORT_ARTIFACT_TYPE:
        errors.append(
            f"artifact_type must be {SECURITY_POSTURE_REPORT_ARTIFACT_TYPE!r}"
        )
    for field in ["generated_at", "project_root", "summary"]:
        if field in payload and not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    if payload.get("posture_status") not in SECURITY_POSTURE_STATUSES:
        errors.append(
            f"posture_status must be one of {sorted(SECURITY_POSTURE_STATUSES)}"
        )

    sbom = payload.get("sbom")
    if not isinstance(sbom, Mapping):
        errors.append("sbom must be an object")
    else:
        if not _is_non_empty_string(sbom.get("path")):
            errors.append("sbom.path must be a non-empty string")
        if not isinstance(sbom.get("exists"), bool):
            errors.append("sbom.exists must be a boolean")
        if (
            not isinstance(sbom.get("component_count"), int)
            or sbom.get("component_count") < 0
        ):
            errors.append("sbom.component_count must be a non-negative integer")

    for list_name in ["vulnerability_reports", "baseline_documents"]:
        items = payload.get(list_name)
        if not isinstance(items, list):
            errors.append(f"{list_name} must be a list")
            continue
        for index, item in enumerate(items, start=1):
            prefix = f"{list_name}[{index}]"
            if not isinstance(item, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            for field in ["name", "path"]:
                if not _is_non_empty_string(item.get(field)):
                    errors.append(f"{prefix}.{field} must be a non-empty string")
            if not isinstance(item.get("required"), bool):
                errors.append(f"{prefix}.required must be a boolean")
            if not isinstance(item.get("exists"), bool):
                errors.append(f"{prefix}.exists must be a boolean")
            if list_name == "vulnerability_reports":
                if item.get("status") not in VULNERABILITY_SCAN_STATUSES:
                    errors.append(
                        f"{prefix}.status must be one of {sorted(VULNERABILITY_SCAN_STATUSES)}"
                    )

    rehearsal = payload.get("backup_restore_rehearsal")
    if not isinstance(rehearsal, Mapping):
        errors.append("backup_restore_rehearsal must be an object")
    else:
        if not _is_non_empty_string(rehearsal.get("path")):
            errors.append("backup_restore_rehearsal.path must be a non-empty string")
        if not isinstance(rehearsal.get("exists"), bool):
            errors.append("backup_restore_rehearsal.exists must be a boolean")
        if rehearsal.get("status") not in BACKUP_RESTORE_REHEARSAL_STATUSES:
            errors.append(
                "backup_restore_rehearsal.status must be one of "
                f"{sorted(BACKUP_RESTORE_REHEARSAL_STATUSES)}"
            )
        duration = rehearsal.get("rehearsal_duration_seconds")
        if not isinstance(duration, (int, float)) or duration < 0:
            errors.append(
                "backup_restore_rehearsal.rehearsal_duration_seconds must be a non-negative number"
            )

    remediation = payload.get("vulnerability_remediation")
    if remediation is not None:
        if not isinstance(remediation, Mapping):
            errors.append("vulnerability_remediation must be an object")
        else:
            if not _is_non_empty_string(remediation.get("path")):
                errors.append(
                    "vulnerability_remediation.path must be a non-empty string"
                )
            if not isinstance(remediation.get("exists"), bool):
                errors.append("vulnerability_remediation.exists must be a boolean")
            if remediation.get("status") not in {
                "missing",
                *VULNERABILITY_REMEDIATION_STATUSES,
            }:
                errors.append(
                    "vulnerability_remediation.status must be 'missing' or one of "
                    f"{sorted(VULNERABILITY_REMEDIATION_STATUSES)}"
                )
            for field in ["accepted_finding_count", "unresolved_finding_count"]:
                value = remediation.get(field)
                if not isinstance(value, int) or value < 0:
                    errors.append(
                        f"vulnerability_remediation.{field} must be a non-negative integer"
                    )
            stale_exception_count = remediation.get("stale_exception_count")
            if stale_exception_count is not None and (
                not isinstance(stale_exception_count, int) or stale_exception_count < 0
            ):
                errors.append(
                    "vulnerability_remediation.stale_exception_count must be a non-negative integer"
                )

    exception_report = payload.get("vulnerability_exception_report")
    if exception_report is not None:
        if not isinstance(exception_report, Mapping):
            errors.append("vulnerability_exception_report must be an object")
        else:
            if not _is_non_empty_string(exception_report.get("path")):
                errors.append(
                    "vulnerability_exception_report.path must be a non-empty string"
                )
            if not isinstance(exception_report.get("exists"), bool):
                errors.append("vulnerability_exception_report.exists must be a boolean")
            for field in ["active_exception_count", "expired_exception_count"]:
                value = exception_report.get(field)
                if not isinstance(value, int) or value < 0:
                    errors.append(
                        f"vulnerability_exception_report.{field} must be a non-negative integer"
                    )
            stale_exception_count = exception_report.get("stale_exception_count")
            if stale_exception_count is not None and (
                not isinstance(stale_exception_count, int) or stale_exception_count < 0
            ):
                errors.append(
                    "vulnerability_exception_report.stale_exception_count must be a non-negative integer"
                )
            stale_exception_ids = exception_report.get("stale_exception_ids")
            if stale_exception_ids is not None:
                if not isinstance(stale_exception_ids, list):
                    errors.append(
                        "vulnerability_exception_report.stale_exception_ids must be a list"
                    )
                else:
                    for index, value in enumerate(stale_exception_ids, start=1):
                        if not _is_non_empty_string(value):
                            errors.append(
                                "vulnerability_exception_report.stale_exception_ids"
                                f"[{index}] must be a non-empty string"
                            )
            review_window_days = exception_report.get("review_window_days")
            if review_window_days is not None and (
                not isinstance(review_window_days, int) or review_window_days < 0
            ):
                errors.append(
                    "vulnerability_exception_report.review_window_days must be a non-negative integer"
                )
            review_due_exception_count = exception_report.get(
                "review_due_exception_count"
            )
            if review_due_exception_count is not None and (
                not isinstance(review_due_exception_count, int)
                or review_due_exception_count < 0
            ):
                errors.append(
                    "vulnerability_exception_report.review_due_exception_count must be a non-negative integer"
                )
            next_exception_expiry = exception_report.get("next_exception_expiry")
            if (
                next_exception_expiry is not None
                and next_exception_expiry != ""
                and _parse_iso_datetime(str(next_exception_expiry)) is None
            ):
                errors.append(
                    "vulnerability_exception_report.next_exception_expiry must be null or a valid ISO 8601 datetime string"
                )
            review_status = exception_report.get("review_status")
            if (
                review_status is not None
                and review_status not in VULNERABILITY_EXCEPTION_REVIEW_STATUSES
            ):
                errors.append(
                    "vulnerability_exception_report.review_status must be null or one of "
                    f"{sorted(VULNERABILITY_EXCEPTION_REVIEW_STATUSES)}"
                )

    for field in [
        "missing_vulnerability_reports",
        "blocked_vulnerability_reports",
    ]:
        value = payload.get(field)
        if not isinstance(value, int) or value < 0:
            errors.append(f"{field} must be a non-negative integer")

    for optional_counter in [
        "blocked_vulnerability_execution_reports",
        "blocked_vulnerability_finding_reports",
        "missing_documents",
        "missing_backup_restore_rehearsal_reports",
        "blocked_backup_restore_rehearsal_reports",
        "accepted_vulnerability_findings",
        "unresolved_vulnerability_findings",
    ]:
        if optional_counter not in payload:
            continue
        value = payload.get(optional_counter)
        if not isinstance(value, int) or value < 0:
            errors.append(f"{optional_counter} must be a non-negative integer")

    next_actions = payload.get("next_actions")
    if not isinstance(next_actions, list):
        errors.append("next_actions must be a list")
    else:
        for index, item in enumerate(next_actions, start=1):
            if not _is_non_empty_string(item):
                errors.append(f"next_actions[{index}] must be a non-empty string")

    return errors


def write_security_posture_report(payload: Mapping[str, Any], path: str | Path) -> Path:
    errors = validate_security_posture_report(payload)
    if errors:
        raise ValueError(f"invalid security posture report: {'; '.join(errors)}")

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path
