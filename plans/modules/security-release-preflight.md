# Module Goal

Keep the security release preflight actionable and release-safe: scanner execution failures, real vulnerability findings, accepted exceptions and stale/expired exception states must be distinguishable in artifacts and CLI output.

# Ownership

- `tools/run_security_release_preflight.py`
- `tools/collect_release_evidence.py`
- `tools/run_python_vulnerability_scan.py`
- `tools/run_container_vulnerability_scan.py`
- `tools/compare_container_vulnerability_baselines.py`
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
- 2026-05-23: Local Python vulnerability scanner passed after dependency remediation: `vulnerability_scan_status=passed`, `vulnerability_scan_findings=0`.
- 2026-05-23: Local Docker/Trivy evidence was rerun against `deployment-zenoh-router` and `deployment-web-panel-distributed`; current scanner data reported `98` container findings across `30` affected components. Rebuilt remediation/posture/preflight artifacts classify them as `accepted_finding_count=98`, `unresolved_finding_count=0`, `stale_exception_count=0`, `security_posture_status=ready`, and skip-collect `security_release_preflight_status=passed`.
- 2026-05-23: Full local preflight with collect was attempted, but the command exceeded a 15-minute local timeout while running the broad `pytest -m "not live"` collect gate after security scan artifacts had already been produced. Security-only skip-collect preflight was used to validate the vulnerability posture from those generated artifacts.
- 2026-05-23: Added `--security-only` to `collect_release_evidence.py` and `run_security_release_preflight.py`. This preserves default full release evidence behavior but lets CI security-preflight collect only SBOM, vulnerability scans, exception review, backup/restore, remediation and security posture evidence.
- 2026-05-26: GitHub CI artifact `security-preflight-artifacts` from scheduled run `26435197977` showed one unresolved Python finding: `fastapi 0.136.3`, `MAL-2026-4750`, with no published fix version. Container findings remained covered by active tracked exceptions; stale and expired exception counts were zero. The 32 `review_due` exceptions were surfaced as non-blocking follow-up evidence. The remediation excludes only `fastapi==0.136.3` across project/deployment install surfaces while preserving the existing preflight contract.
- 2026-05-26: Re-reviewed the active deployment-web-panel-distributed no-fix container exceptions against refreshed security-only preflight evidence. Matching findings remained no-fix, stale and expired exception counts remained zero, and the next exception expiry was renewed to `2026-08-24T00:00:00+01:00`.
- 2026-05-26: Added a reusable container vulnerability baseline comparison tool so candidate base-image raw Trivy reports can be compared against the current baseline before changing Dockerfiles. The tool is advisory evidence only; it does not alter preflight pass/fail semantics.
- 2026-05-31: Removed the PR skip condition from the `security-preflight` workflow job so security-only `pip-audit`/`trivy` evidence runs on PRs as well as manual and scheduled workflow runs.
- 2026-05-31: PR #9 security-preflight run `26715450130` surfaced one new unresolved container finding: `deployment-web-panel-distributed` `libbz2-1.0` `CVE-2026-42250`, with Trivy `FixedVersion=null`. Added a temporary `only_without_fix_version` structured exception scoped to that image, component, version and CVE, expiring with the current web panel container exception batch on `2026-08-24T00:00:00+01:00`.
- 2026-05-31: Tightened `security-preflight` so vulnerability exception review evidence is release-blocking. The preflight now fails closed when the review report is missing/invalid/blocked or when accepted vulnerability exceptions are inside the review-due window and require follow-up.
- 2026-05-31: Updated smoke and stable release rehearsal fixtures for the stricter review gate: smoke now builds `vulnerability_exception_review_report.json`, and approved rehearsal vulnerability exceptions stay outside the review-due window so release evidence closes with `review_candidate_count=0`.

# Non-Goals

- Do not suppress vulnerability findings to make CI green.
- Do not convert unresolved findings to accepted risk without a structured exception entry.
- Do not change Docker images, dependency versions or deployment behavior unless the blocker requires real remediation and tests support it.
- Do not make live/container scanning mandatory for regular non-live test suites.
- Do not allow `review_due` vulnerability exceptions through the formal security release preflight; refresh or re-approve those exceptions first.

# Validation

```powershell
py -3.12 -m pytest tests\test_security_release_preflight.py tests\test_security_posture_reports.py tests\test_vulnerability_scan_runners.py -q --tb=short
py -3.12 tools\run_security_release_preflight.py --output-root test_env\release_evidence_ci --run-python-vuln-scan --run-container-vuln-scan --container-image-ref deployment-zenoh-router --container-image-ref deployment-web-panel-distributed
py -3.12 tools\compare_container_vulnerability_baselines.py --current-raw-report test_env\release_evidence_ci_exception_refresh\security\container_vuln_scan_report_raw\deployment-web-panel-distributed.json --candidate-raw-report <candidate-trivy-raw.json-or-dir> --output test_env\container_baseline_comparison\comparison.json
```

# Residual Risks

- Real `pip-audit` / `trivy` / Docker behavior depends on external scanner databases and local or CI image availability; the current local Docker path is available and scanned successfully.
- Accepted no-fix exceptions remain temporary release risk and must be reviewed before expiry; current generated evidence must show zero review-due, stale or expired exceptions and next expiry at `2026-08-24T00:00:00+01:00`.
- Full release evidence collection still includes broad non-live gates by default; security CI uses the security-only preflight profile to keep vulnerability posture validation independent from that longer release gate.
