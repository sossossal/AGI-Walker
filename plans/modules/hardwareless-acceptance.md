# Module Goal

Maximize release confidence when no real robot hardware is available by collecting substitute evidence, probing unavailable external runtimes and producing a fail-explicit hardwareless acceptance report.

# Ownership

- `tools/build_hardwareless_acceptance_report.py`
- `tools/validate_hardwareless_release_gate.py`
- `tests/test_hardwareless_acceptance_report.py`
- `docs/hardware/HARDWARE_INTEGRATION_GUIDE.md`
- `PROJECT_PLAN.md`

# Inputs and Outputs

Inputs:

- Operator confirmation that no hardware is present.
- Existing live Godot mountain readiness artifact when available.
- Optional real hardware live closeout artifact.
- Optional ROS2 bridge live smoke artifact.
- Local Python environment probe for ROS2 runtime modules.
- Existing mock/replay test commands for serial driver, IMC-22 transport, ROS2 fake runtime and Web hardware recovery routes.

Outputs:

- `hardwareless_acceptance_report.v1` JSON under `test_env/hardwareless_acceptance/`.
- Structured substitute evidence command list.
- Structured no-hardware safety scenario matrix with covered test surfaces.
- Structured required external evidence and blockers for real robot hardware, real serial/CAN transport and real ROS2 runtime.
- Machine-readable `release_gate` and `enterprise_acceptance_verdict` fields that keep release use fail-closed even when local hardwareless acceptance is documented.
- Optional validator output artifact for CI/release evidence archival.

# Contract Checklist

- Public surface this module exposes: `py -3.12 tools\build_hardwareless_acceptance_report.py --no-hardware`, with optional strict `--require-external-evidence`.
- Release gate validator: `py -3.12 tools\validate_hardwareless_release_gate.py test_env\hardwareless_acceptance\hardwareless_acceptance_report.json`.
- Inputs this module accepts: optional Godot readiness artifact path, hardware live closeout path, ROS2 bridge live smoke path and output path.
- Outputs this module produces: hardwareless acceptance status, substitute evidence entries, local runtime probes, required external evidence, external blockers, residual risks and next actions.
- Shared contracts touched: hardware integration documentation, root project plan, live Godot readiness artifact contract.
- Backward compatibility requirements: no change to hardware live closeout semantics; hardwareless report must not mark real hardware validation as passed.
- Integration tests required: report schema/status, fail-explicit hardware flag behavior and documentation linkage.

# Non-Goals

- Do not claim real motor, encoder, serial, CAN or customer-site hardware validation.
- Do not require ROS2 Humble, physical devices or vendor sample telemetry.
- Do not modify the existing real hardware live closeout gate.

# Tasks

- [x] Add hardwareless acceptance report builder.
- [x] Add tests for report status, external blockers and docs linkage.
- [x] Document the no-hardware acceptance path and its limits.
- [x] Run targeted mock/replay, ROS2 fake-runtime, live Godot and production compose checks.
- [x] Generate the hardwareless acceptance report artifact.
- [x] Add explicit no-hardware safety scenario coverage for command limits, watchdog fallback, fault-class recovery, serial replay, Web recovery permissions and live Godot mountain evidence.
- [x] Convert irreducible real hardware and ROS2 gaps from duplicated residual risks into resolvable required external evidence gates.
- [x] Add strict release mode that blocks the report when required external evidence is missing.
- [x] Add release-gate verdict fields so ordinary hardwareless reports cannot be mistaken for release-ready evidence.
- [x] Add standalone release-gate validator for CI/release scripts.
- [x] Add validator output artifact support.

# Validation

```powershell
py -3.12 -m py_compile tools\build_hardwareless_acceptance_report.py
py -3.12 -m py_compile tools\validate_hardwareless_release_gate.py
py -3.12 -m pytest tests\test_hardwareless_acceptance_report.py -q
py -3.12 -m pytest tests\test_real_robot_driver.py tests\test_hardware_controller.py tests\test_ros2_bridge_runtime.py tests\test_ros2_workspace.py -q -rs
```

# Completion Criteria

- Hardwareless report is machine-readable and fail-explicit.
- Substitute evidence commands are documented and tested.
- External blockers remain visible until real hardware and ROS2 runtime evidence exist.

# Notes

- 2026-05-21: Added after local live Godot and production compose validation to close the no-hardware acceptance gap without weakening real hardware gates.
- 2026-05-21: Generated `test_env/hardwareless_acceptance/hardwareless_acceptance_report.json` with `status=accepted_with_documented_external_blockers`, `godot_readiness_status=ready`, three external blockers and no report blockers. Targeted substitute tests passed, while real serial and real hardware tests remained skipped as expected.
- 2026-05-21: Refined residual-risk handling by adding `hardwareless_safety_scenarios` and a Zenoh Python runtime probe. The report now distinguishes locally mitigated no-hardware risks from the remaining real hardware and real ROS2 live gaps.
- 2026-05-21: Added optional `--hardware-live-closeout` and `--ros2-bridge-smoke` inputs. Missing real hardware/ROS2 evidence is now represented as `required_external_evidence` plus external blockers, not as unstructured residual risk; supplying `status=ready` hardware closeout and `status=passed` ROS2 smoke removes those blockers.
- 2026-05-21: Added `--require-external-evidence` for release use. In strict mode, missing real hardware closeout or ROS2 bridge live smoke turns the report `blocked` with `required_external_evidence_missing_or_not_ready`.
- 2026-05-21: Added `release_gate.status` and `enterprise_acceptance_verdict`. Ordinary hardwareless reports may remain useful local evidence, but release-gate status stays `blocked` while external evidence is missing.
- 2026-05-21: Added `tools/validate_hardwareless_release_gate.py` so CI/release scripts can fail closed on `release_gate.status != ready` without hand-rolled JSON parsing.
- 2026-05-21: Added `--output` to the validator so failed release gate checks can be archived as structured evidence instead of only appearing in job logs.
- 2026-05-31: Added workflow-contract regression coverage for opt-in external-environment CI jobs, including ROS2 bridge smoke. This keeps ROS2 live validation opt-in while making trigger conditions, enablement env, artifact output path and retention policy auditable.
- 2026-07-08: Added strict industrial live archive binding for customer-site real-device smoke. `build_industrial_live_evidence_archive_report.py --require-customer-site-smoke` now treats a missing or non-passed customer-site smoke report as a blocker, while default archive behavior remains backward compatible.
- 2026-07-08: Hardened next-stage readiness action classification so current hardware diagnostics and browser manual evidence gaps remain external-input actions instead of code-or-config actions.
- 2026-07-08: Updated the next-stage execution plan to use the strict customer-site smoke archive command for industrial signoff, so operator guidance matches the fail-closed tool contract.
- 2026-07-08: Aligned operator checklist and SLA/SLO guidance with the strict industrial signoff sequence: generate customer-site smoke first, then build the industrial archive with `--require-customer-site-smoke`.
- 2026-07-08: Corrected next-stage browser evidence status drift. The plan now matches the canonical blocked `web_browser_validation_evidence_pack.json`: Playwright support evidence is present, but manual report, screenshots, exports, console summary and validation closeout still require real browser evidence.
- 2026-07-08: Clarified `docs/CURRENT_STATUS.md` so release/stable readiness is not confused with next-stage live evidence completion; current next-stage blockers remain external input only.
- 2026-07-08: Added timezone-aware `generated_at` to `next_stage_readiness_report.v1` and documented it in the next-stage execution plan so stale local/CI evidence can be identified during residual-risk closeout.
- 2026-07-08: Added Git provenance metadata to `next_stage_readiness_report.v1` so branch/main and clean/dirty evidence snapshots can be distinguished during residual-risk review.
- 2026-07-08: Added self-validation for `next_stage_readiness_report.v1`; malformed summary counts or blocker/action ordering now appear in `validation_errors` and prevent a ready report from exiting successfully.
- 2026-07-08: Added stdout summary lines for `build_next_stage_readiness_report.py` so operators can distinguish external-input blockers from report validation errors directly in CI/terminal logs.
- 2026-07-08: Added explicit `--expected-status blocked|ready` CLI mode for `build_next_stage_readiness_report.py`. Default behavior remains fail-closed for final readiness, while `--expected-status blocked` is available only for preserving self-validating blocked evidence snapshots.
- 2026-07-08: Added CI `next-stage-readiness` artifact collection. It runs the default ready gate first and falls back to `--expected-status blocked` only when preserving the current external-input blocker snapshot for review.
- 2026-07-08: Remote PR #21 confirmed `next-stage-readiness-artifacts` upload, but the downloaded report showed empty `git.branch` from a detached checkout. Added GitHub Actions env fallback for branch/ref and commit provenance.
- 2026-07-08: Added `tools/build_next_stage_external_evidence_checklist.py` so current external-input readiness blockers can be archived as operator-executable evidence items without weakening the final readiness gate.
- 2026-07-09: Added CI retention for `next_stage_external_evidence_checklist.json` inside `next-stage-readiness-artifacts`; the checklist command is allowed to return blocked while the job still archives the authoritative readiness report.
- 2026-07-09: Enriched `next_stage_external_evidence_checklist.json` items with route-specific `evidence_commands`, `input_templates` and `guide_paths`, so hardware, ROS2, vendor, delivery and browser external evidence can be delegated without weakening the fail-closed readiness report.
- 2026-07-09: Added planned checklist self-validation for handoff command/list shape and repository-relative template/guide paths, so stale operator handoff metadata is visible before external teams act on it.
- 2026-07-09: Added `execution_prerequisites` to the next-stage external evidence checklist so external operators use a supported Python interpreter, preferably Python 3.12, and preserve fail-closed evidence policy instead of substituting placeholders.
- 2026-07-09: Added execution prerequisite self-validation to the next-stage external evidence checklist so malformed Python/evidence policy metadata is reported through `execution_prerequisite_validation_errors` before handoff.
- 2026-07-09: Added full artifact self-validation to the next-stage external evidence checklist so schema, timestamp, collection, summary count and status drift is reported through top-level `validation_errors`.
- 2026-07-09: Added checklist-driven external evidence status reporting. `next_stage_external_evidence_status_report.v1` checks each checklist artifact path against its target status and can archive a blocked intake snapshot without weakening the authoritative readiness gate.
- 2026-07-09: Hardened status report path handling. Checklist `artifact_path` values must remain repository-relative; absolute, parent-directory and empty paths are validation errors and are not read.
- 2026-07-09: Hardened status report item-shape self-validation so malformed evidence item fields cannot be accepted by matching only aggregate counts.
- 2026-07-09: Hardened checklist item-shape self-validation so malformed external evidence handoff items are rejected before status report intake.
- 2026-07-09: Hardened checklist artifact path boundary so external evidence item paths must be repository-relative before status report intake.
