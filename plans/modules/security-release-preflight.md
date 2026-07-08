# Module Goal

Keep the security release preflight actionable and release-safe: scanner execution failures, real vulnerability findings, accepted exceptions and stale/expired exception states must be distinguishable in artifacts and CLI output.

# Ownership

- `tools/run_security_release_preflight.py`
- `tools/collect_release_evidence.py`
- `tools/run_python_vulnerability_scan.py`
- `tools/run_container_vulnerability_scan.py`
- `tools/compare_container_vulnerability_baselines.py`
- `tools/build_vulnerability_exception_burndown_report.py`
- `agi_walker/core/api/security_posture_contracts.py`
- `deployment/security/vulnerability_exceptions.input.json`
- `tests/test_security_release_preflight.py`
- `tests/test_security_posture_reports.py`
- `tests/test_vulnerability_scan_runners.py`
- `PROJECT_PLAN.md`

# Contract Checklist

- Public surface this module exposes: `security_release_preflight_report`, `security_posture_report`, vulnerability scan/remediation/exception reports, and CI `security-preflight` stdout.
- Inputs this module accepts: real `pip-audit` output, real `trivy` output, structured vulnerability reports, approved exception input, backup/restore report and security baseline docs.
- Outputs this module produces: machine-readable preflight status, summary, blocked reason metrics, stale/expired/review exception metrics, exception burn-down summaries and remediation guidance.
- Compatibility requirements: do not mark security posture passed unless unresolved findings are remediated or covered by active matching exceptions; do not hide scanner execution failures.
- Validation required: targeted security preflight/posture/scan tests plus local preflight reproduction when available.

# Current Task

- [x] Reproduce or inspect the blocked CI preflight.
- [x] Identify whether the blocker is scanner execution, uncovered findings, stale exceptions, expired exceptions or missing evidence.
- [x] Apply the smallest safe fix: update managed exceptions only when findings are still no-fix and intentionally accepted, or update code/tests when classification is wrong.
- [x] Run targeted validation and document residual risk.
- [x] Add a non-gating vulnerability exception burn-down report so active temporary exceptions can be reviewed by expiry, ticket, component, image and severity without changing preflight pass/fail behavior.
- [x] Preserve the vulnerability exception burn-down report in collected security evidence and surface its status/counts in security preflight metrics without adding a new release blocker.
- [x] Pin the web panel production Dockerfile to a reproducible Debian suite base image and expose a compose override so remote Docker/Trivy evidence can test alternate candidates before promotion.
- [x] Replace `python-jose[cryptography]` with `PyJWT` for HS256 Web Panel tokens so the no-fix transitive `ecdsa` Python vulnerability is removed instead of accepted by exception.
- [ ] Validate `python:3.11-alpine` as the Web Panel default candidate through remote Docker/Trivy evidence before accepting it as a real container finding burn-down.
- [ ] Re-run the Alpine candidate after adding the minimal `libgcc` apk dependency needed by the `eclipse-zenoh` Rust metadata path.
- [ ] Re-run the Alpine candidate after adding temporary `build-base` apk build dependencies for the `eclipse-zenoh` native wheel build path, ensuring build deps are removed from the final image.

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
- 2026-05-31: Tightened container no-fix exception scope. All active deployment-web-panel-distributed no-fix exceptions now carry explicit `vulnerability_ids`/`severities` from current Trivy evidence, and one unmatched `libcap2` exception was removed. Validation now rejects container no-fix exceptions without at least one vulnerability id.
- 2026-06-05: Scheduled main run `26998884678` surfaced one new unresolved container finding: `deployment-web-panel-distributed` `perl-base` `CVE-2026-7010`, with Trivy `FixedVersion=null`. Added it to the existing scoped `perl-base` no-fix exception under the current web panel container review ticket, expiring on `2026-08-24T00:00:00+01:00`.
- 2026-06-10: Scheduled main run `27257251046` surfaced 48 unresolved container findings after scanner database refresh: 45 OpenSSL package findings across `openssl`, `libssl3t64`, and `openssl-provider-legacy`, plus severity drift for existing `perl-base` and `libbz2-1.0` no-fix findings. All matching raw Trivy findings reported `FixedVersion=null`; added scoped OpenSSL no-fix exceptions and refreshed severity lists while preserving the shared `2026-08-24T00:00:00+01:00` expiry.
- 2026-06-16: PR #14 run `27632154360` surfaced 7 unresolved `deployment-web-panel-distributed` container findings after scanner database refresh: `libsqlite3-0` `CVE-2026-11822`/`CVE-2026-11824`, Linux-PAM package `CVE-2026-54411` across `libpam-modules`, `libpam-modules-bin`, `libpam-runtime`, and `libpam0g`, plus `perl-base` `CVE-2026-12087`. Raw Trivy evidence reported `FixedVersion=null` for all 7; updated scoped no-fix exceptions while preserving the shared `2026-08-24T00:00:00+01:00` expiry and fail-closed preflight behavior.
- 2026-06-18: Scheduled main run `27741873038` surfaced 40 unresolved container findings after scanner database refresh. The actionable delta was the util-linux package-family UNKNOWN no-fix CVE group `CVE-2026-53612` through `CVE-2026-53615` across `bsdutils`, `libblkid1`, `liblastlog2-2`, `libmount1`, `libsmartcols1`, `libuuid1`, `login`, `mount`, and `util-linux`, plus Linux-PAM `CVE-2026-54411` severity drift from `UNKNOWN` to `MEDIUM`; raw Trivy evidence reported `FixedVersion=null`. Updated existing scoped no-fix exceptions for those components, preserved the shared `2026-08-24T00:00:00+01:00` expiry, and kept security-preflight fail-closed for future unmatched CVEs or fixed-version drift.
- 2026-07-08: Scheduled main run `28919293675` surfaced 18 unresolved findings after scanner database refresh: Python `ecdsa` `PYSEC-2026-1325`; container `gzip` `CVE-2026-41991`/`CVE-2026-41992`, `libacl1` `CVE-2026-54369`/`CVE-2026-54370`, `libattr1` `CVE-2026-54371`, SQLite `CVE-2026-11822`/`CVE-2026-11824` severity drift to `MEDIUM`, util-linux package-family `CVE-2026-13595` across `bsdutils`, `libblkid1`, `liblastlog2-2`, `libmount1`, `libsmartcols1`, `libuuid1`, `login`, `mount`, and `util-linux`, plus `perl-base` `CVE-2026-7017`. Raw pip-audit/Trivy evidence reported no fix versions for all 18; added scoped supplemental no-fix exceptions, preserved the shared `2026-08-24T00:00:00+01:00` expiry, and replayed the downloaded CI artifact to `security_release_preflight_status=passed`.
- 2026-07-08: PR #16 run `28938765573` reduced the blocker to 9 unresolved `deployment-web-panel-distributed` findings: existing util-linux package-family `CVE-2026-53615` entries across `bsdutils`, `libblkid1`, `liblastlog2-2`, `libmount1`, `libsmartcols1`, `libuuid1`, `login`, `mount`, and `util-linux` drifted to `HIGH` severity while still reporting `FixedVersion=null`. Updated those existing scoped no-fix exception severity lists and kept the fail-closed behavior for future CVE, severity or fix-version drift.
- 2026-07-08: Added `vulnerability_exception_burndown_report` as a read-only residual-risk artifact. It summarizes active, review-due and expired temporary exceptions by scope, ticket, component, image ref and highest severity, emits action items for ongoing no-fix burn-down, is collected into security evidence artifacts, and is surfaced in security preflight metrics without becoming a release gate.
- 2026-07-08: Latest main security artifact showed `deployment-web-panel-distributed` built from floating `python:3.11-slim`, which resolved to Debian 13.5 and carried 165 no-fix container findings covered by temporary exceptions. A remote PR scan proved `python:3.11-slim-bookworm` is not an acceptable default because it increased findings to 186 and left 185 unresolved against current exceptions. The web panel Dockerfile now defaults to the equivalent reproducible `python:3.11-slim-trixie` through `WEB_PANEL_BASE_IMAGE`, and `deployment/docker-compose.yml` exposes `AGI_WALKER_WEB_PANEL_BASE_IMAGE` for controlled candidate scans before any future promotion.
- 2026-07-08: Main security artifact still showed one Python finding: transitive `ecdsa` from `python-jose[cryptography]`, `PYSEC-2026-1325`, with no fix version. Web Panel auth only uses HS256 encode/decode, so the dependency was replaced with `PyJWT` while preserving token semantics and `decode_access_token` returning `None` on invalid tokens.
- 2026-07-08: Remaining production security risk is concentrated in the `deployment-web-panel-distributed` OS package layer. The next candidate switches the Web Panel default base to `python:3.11-alpine` and adds `apk` package-manager support. This is not accepted as remediation until GitHub security-preflight confirms image build success, lower findings and no unresolved/stale exception drift.
- 2026-07-08: PR #20 security-preflight failed before Trivy because the Alpine `eclipse-zenoh` install path downloaded a musl Rust toolchain whose `cargo` needed `libgcc_s.so.1`. The candidate now installs the minimal `libgcc` apk package by default and keeps it overrideable through `AGI_WALKER_WEB_PANEL_APK_PACKAGES`.
- 2026-07-08: PR #20 security-preflight then progressed to native `eclipse-zenoh` compilation and failed because Alpine lacked linker `cc`. The candidate now installs `build-base` as a virtual apk build dependency and removes `.web-panel-build-deps` after pip install so compiler packages are not retained in the final filesystem.

# Non-Goals

- Do not suppress vulnerability findings to make CI green.
- Do not convert unresolved findings to accepted risk without a structured exception entry.
- Do not change Docker images, dependency versions or deployment behavior unless the blocker requires real remediation and tests support it.
- Do not make live/container scanning mandatory for regular non-live test suites.
- Do not allow `review_due` vulnerability exceptions through the formal security release preflight; refresh or re-approve those exceptions first.
- Do not allow broad component-only container no-fix exceptions; every accepted no-fix container finding must be scoped to explicit vulnerability ids.

# Validation

```powershell
py -3.12 -m pytest tests\test_security_release_preflight.py tests\test_security_posture_reports.py tests\test_vulnerability_scan_runners.py -q --tb=short
py -3.12 -m pytest tests\test_vulnerability_exception_burndown_report.py -q
py -3.12 tools\run_security_release_preflight.py --output-root test_env\release_evidence_ci --run-python-vuln-scan --run-container-vuln-scan --container-image-ref deployment-zenoh-router --container-image-ref deployment-web-panel-distributed
py -3.12 tools\compare_container_vulnerability_baselines.py --current-raw-report test_env\release_evidence_ci_exception_refresh\security\container_vuln_scan_report_raw\deployment-web-panel-distributed.json --candidate-raw-report <candidate-trivy-raw.json-or-dir> --output test_env\container_baseline_comparison\comparison.json
```

# Residual Risks

- Real `pip-audit` / `trivy` / Docker behavior depends on external scanner databases and local or CI image availability; the current local Docker path is available and scanned successfully.
- Accepted no-fix exceptions remain temporary release risk and must be reviewed before expiry; current generated evidence must show zero broad component-only, review-due, stale or expired exceptions and next expiry at `2026-08-24T00:00:00+01:00`.
- The web panel base-image pin improves reproducibility but does not itself eliminate accepted no-fix findings; any alternate base-image promotion must first show lower production findings and no new unresolved findings in remote Docker/Trivy evidence.
- Full release evidence collection still includes broad non-live gates by default; security CI uses the security-only preflight profile to keep vulnerability posture validation independent from that longer release gate.
