from __future__ import annotations

import argparse
import json
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
REQUIREMENTS = ROOT / "config" / "coverage_requirements.json"
DEFAULT_OUTPUT = ROOT / "test_env" / "full_coverage_report.json"


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return payload


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def retained_paths(manifest: dict[str, Any]) -> set[str]:
    return {str(artifact.get("path")) for artifact in manifest.get("artifacts", [])}


def check_domain(domain: dict[str, Any], retained: set[str]) -> dict[str, Any]:
    report_path = ROOT / domain["report"]
    checks = {"report_exists": report_path.is_file()}
    observed: dict[str, Any] = {}
    if report_path.is_file():
        payload = load_json(report_path)
        observed = {
            "schema_version": payload.get("schema_version"),
            "status": payload.get("status"),
        }
        if "required_status" in domain:
            checks["status_matches"] = payload.get("status") == domain["required_status"]
        if "required_schema_version" in domain:
            checks["schema_matches"] = payload.get("schema_version") == domain["required_schema_version"]
    else:
        checks["status_matches"] = False
    missing = [path for path in domain.get("required_retention_artifacts", []) if path not in retained]
    checks["retention_artifacts_present"] = not missing
    return {
        "name": domain["name"],
        "status": "covered" if all(checks.values()) else "uncovered",
        "report": domain["report"],
        "observed": observed,
        "checks": checks,
        "missing_retention_artifacts": missing,
    }


def build_report() -> dict[str, Any]:
    requirements = load_json(REQUIREMENTS)
    manifest = load_json(ROOT / requirements["retention"]["manifest"])
    hardware_gap = load_json(ROOT / requirements["hardware_gap_report"])
    retained = retained_paths(manifest)
    domains = [check_domain(domain, retained) for domain in requirements["software_domains"]]
    retention_checks = {
        "manifest_schema_matches": manifest.get("schema_version") == requirements["retention"]["required_schema_version"],
        "artifact_count_sufficient": int(manifest.get("artifact_count", 0)) >= int(requirements["retention"]["min_artifact_count"]),
    }
    software_passed = all(domain["status"] == "covered" for domain in domains) and all(retention_checks.values())
    real_world_status = hardware_gap.get("status", "missing")
    return {
        "schema_version": "biped-full-coverage-report.v1",
        "created_utc": datetime.now(UTC).isoformat(),
        "scope": requirements["scope"],
        "software_coverage_status": "passed" if software_passed else "failed",
        "real_world_coverage_status": real_world_status,
        "overall_status": "passed_with_external_blockers" if software_passed and real_world_status.startswith("blocked") else ("passed" if software_passed else "failed"),
        "software_domains": domains,
        "retention": {
            "manifest": requirements["retention"]["manifest"],
            "artifact_count": manifest.get("artifact_count"),
            "checks": retention_checks,
        },
        "external_domains": [
            {
                "name": name,
                "status": real_world_status,
                "covered": not real_world_status.startswith("blocked"),
                "required_for_real_world_full_coverage": True,
            }
            for name in requirements["external_domains"]
        ],
        "hardware_gap_report": requirements["hardware_gap_report"],
        "decision": {
            "software_and_simulated_stack": "fully_covered" if software_passed else "not_fully_covered",
            "physical_hardware_stack": "not_covered_without_real_hardware" if real_world_status.startswith("blocked") else "covered",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Build biped_robot full software coverage report.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    report = build_report()
    write_json(args.output, report)
    print(json.dumps({"status": report["overall_status"], "output": str(args.output)}, ensure_ascii=False))
    return 0 if report["software_coverage_status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
