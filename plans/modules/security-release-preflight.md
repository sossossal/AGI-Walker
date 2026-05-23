# Module Goal

Keep the security release preflight actionable and release-safe: scanner execution failures, real vulnerability findings, accepted exceptions and stale/expired exception states must be distinguishable in artifacts and CLI output.

# Ownership

- `tools/run_security_release_preflight.py`
- `tools/collect_release_evidence.py`
- `tools/run_python_vulnerability_scan.py`
- `tools/run_container_vulnerability_scan.py`
- `agi_walker/core/api/security_posture_contracts.py`
- `deployment/security/vulnerability_exceptions.input.json`
- `tests/test_security_release_preflight.py`
- `tests/test_security_posture_reports.py`
- `tests/test_vulnerability_scan_runners.py`
- `PROJECT_PLAN.md`

# Contract Checklist

- Public surface this module exposes: `security_release_preflight_report`, `security_posture_report`, vulnerability scan/remediation/exception reports, and CI `security-preflight` stdout.
- Inputs this module accepts: real `pip-audit` output, real `trivy` output, structured vulnerability reports, approved exception input, backup/restore report and security baseline docs.
- Outputs this module produces: machine-readable preflight status, summary, blocked reason metrics, stale/expired/review exception metrics and remediation guidance.
- Compatibility requirements: do not mark security posture passed unless unresolved findings are remediated or covered by active matching exceptions; do not hide scanner execution failures.
- Validation required: targeted security preflight/posture/scan tests plus local preflight reproduction when available.

# Current Task

- [x] Reproduce or inspect the blocked CI preflight.
- [x] Identify whether the blocker is scanner execution, uncovered findings, stale exceptions, expired exceptions or missing evidence.
- [x] Apply the smallest safe fix: update managed exceptions only when findings are still no-fix and intentionally accepted, or update code/tests when classification is wrong.
- [x] Run targeted validation and document residual risk.

# Notes

- 2026-05-23: GitHub CI artifact `security-preflight-artifacts` from run `26324738634` showed one unresolved Python finding: `starlette 0.52.1`, `PYSEC-2026-161` / `GHSA-86qp-5c8j-p5mr`, fixed by `starlette>=1.0.1`. Container findings were covered by active tracked exceptions; stale and expired exception counts were zero.
- 2026-05-23: Local Python vulnerability scanner passed after dependency remediation: `vulnerability_scan_status=passed`, `vulnerability_scan_findings=0`. Full local release preflight remains environment-dependent because Docker is unavailable in this workspace.

# Non-Goals

- Do not suppress vulnerability findings to make CI green.
- Do not convert unresolved findings to accepted risk without a structured exception entry.
- Do not change Docker images, dependency versions or deployment behavior unless the blocker requires real remediation and tests support it.
- Do not make live/container scanning mandatory for regular non-live test suites.

# Validation

```powershell
py -3.12 -m pytest tests\test_security_release_preflight.py tests\test_security_posture_reports.py tests\test_vulnerability_scan_runners.py -q --tb=short
py -3.12 tools\run_security_release_preflight.py --output-root test_env\release_evidence_ci --run-python-vuln-scan --run-container-vuln-scan --container-image-ref deployment-zenoh-router --container-image-ref deployment-web-panel-distributed
```

# Residual Risks

- Real `pip-audit` / `trivy` / Docker behavior depends on external scanner databases and local or CI image availability.
- Accepted no-fix exceptions remain temporary release risk and must be reviewed before expiry.
