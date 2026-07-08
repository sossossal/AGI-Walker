# Goal

把 AGI-Walker 的 JSON-to-Godot 动态机械节点树能力推进到可验证、可交付、可持续维护的工程闭环：从 JSON/normalized config 到静态 Godot node-tree manifest，再到报告、delivery gate、CI evidence、可选 live Godot smoke 和 Web/session 交付证据，保持同一组契约字段和验收语义。

下一阶段把已完成的静态/运行时还原闭环推进为产品化验收能力：可复用 live Godot 验证 profile、可解释机械行为 evidence、Web 操作闭环、schema 1.5 演进计划，以及面向交付的 evidence bundle。

# Scope

In scope:

- 动态 Godot robot generation 的 schema、manifest、report、gate、CI、fixture、测试和文档。
- 与该能力直接相关的 Godot runtime restoration、Web/session evidence preservation 和 release evidence。
- 项目级计划、模块子计划、跨模块契约和验收标准维护。
- Phase 2: live Godot 验证产品化、运行时机械完整性增强、Web evidence dashboard、机器人 schema 1.5 规划、发布验收包。

Out of scope:

- 不默认要求每个 PR 运行 live Godot；live 验证先进入本地/手动/定时 CI profile。
- 不替换当前 JSON robot config 格式。
- 不做无关重构、格式 churn 或跨领域发布改造。
- 不在 schema 1.5 规划阶段强制迁移旧 fixture 或删除旧 schema 兼容。

# Existing Context

- 根级 `AGENTS.md` 要求后续编程任务默认遵循 `$codex-project-plan`、`$closure-first-engineer`、`$efficient-engineer`、`$enterprise-code-acceptance`。
- `docs/guides/DYNAMIC_GODOT_ROBOT_GENERATION_FUTURE_PLAN.md` 已记录本功能未来路线。
- 当前静态 `godot_node_tree_manifest.v1`、sidecar self-validation、delivery gate strict scan 和 CI static manifest gate 已实现。
- 当前 `static_only`、`godot_load_verified`、`godot_verified` 验收层级和证明命令已文档化。
- 根级 `PROJECT_PLAN.md` 和 `plans/modules/` 是本轮按 `AGENTS.md` 新建的计划入口。

# Architecture Principles

- Contract first: schema、CLI、report、gate、CI 和 docs 使用同一组字段名和含义。
- Static before live: 能静态证明的 topology、path、parameter 和 completeness 先在 manifest/report/gate 中闭合。
- Evidence over confidence: 每个验收结论必须指向命令、测试、artifact 或明确残余风险。
- Backward compatible by default: 严格失败只由显式 flag、CI profile 或专用 closeout 工具触发。
- Minimal change: 每轮只修改实现目标所需的最小文件集合。

# Constraints and Non-Goals

- 模块内临时发现和局部 TODO 不进入本文件，写入对应 `plans/modules/<module>.md`。
- 跨模块契约、范围、验收标准变化必须记录到本文件 `Change Control`。
- live Godot、浏览器、硬件和外部服务验证未运行时必须作为残余风险报告。

# Module Index

| Module | Subplan | Responsibility | Dependencies | Status |
| --- | --- | --- | --- | --- |
| static-godot-node-tree-evidence | `plans/modules/static-godot-node-tree-evidence.md` | Static manifest evidence generation, closeout report, report/gate CI integration | `robot_schema.py`, report builder, gate validator, fixtures | complete |
| runtime-godot-restoration | `plans/modules/runtime-godot-restoration.md` | Runtime Godot node-tree comparison, parameter readback, motion smoke | Static manifest evidence | complete |
| web-session-godot-evidence | `plans/modules/web-session-godot-evidence.md` | Web/session delivery evidence preservation | Runtime restoration, report/gate contracts | complete |
| release-evidence-readiness | `plans/modules/release-evidence-readiness.md` | Dynamic Godot generation readiness summary and evidence level reporting | Static closeout, delivery gates, Web/session evidence | complete |
| live-godot-verification | `plans/modules/live-godot-verification.md` | Productized local/manual/scheduled live Godot verification profile, executable discovery, failure taxonomy, artifact retention, flaky policy | Runtime restoration, CI, report/gate contracts | complete |
| runtime-mechanical-behavior | `plans/modules/runtime-mechanical-behavior.md` | Runtime behavior evidence for joint limits, torque/velocity response, center of mass, contact state, step trace | Live Godot verification, Godot scripts, report builder | complete |
| web-evidence-dashboard | `plans/modules/web-evidence-dashboard.md` | Web UI/API flow for static/load/live evidence display, manifest mismatch, residual risk, one-click load/readiness actions | Web/session evidence, release readiness | complete |
| schema-evolution | `plans/modules/schema-evolution.md` | Robot schema 1.5 optional actuator/sensor/joint-limit/controller/material physics fields and migration compatibility | Static schema, runtime behavior, fixtures | complete |
| release-evidence-bundle | `plans/modules/release-evidence-bundle.md` | Customer-facing evidence bundle with static closeout, gate, readiness, live smoke, Web delivery record, docs index, self-validation | All evidence modules | complete |
| mountain-biped-simulation | `plans/modules/mountain-biped-simulation.md` | Local deterministic mountain-running example for a humanoid biped robot, with terrain/run evidence artifacts | Robot schema 1.5, examples, tests | complete |
| production-compose-smoke | `plans/modules/production-compose-smoke.md` | Supported Docker Compose deployment entrypoint and authenticated Web workflow smoke | Web panel, Celery worker, Redis, deployment docs/tests | complete |
| hardwareless-acceptance | `plans/modules/hardwareless-acceptance.md` | No-hardware acceptance report that preserves substitute evidence and explicit external blockers | Hardware replay/mock tests, ROS2 fake runtime, live Godot readiness, production compose smoke | complete |
| repository-presentation | `plans/modules/repository-presentation.md` | GitHub-facing README, docs index and repository tree guidance | Existing docs, plans, source layout | complete |
| security-release-preflight | `plans/modules/security-release-preflight.md` | Security release preflight scanner execution, vulnerability exception matching and release-blocker classification | Security posture contracts, scanner wrappers, deployment exceptions, CI | complete |

# Interfaces and Contracts

## Cross-Module Contracts

- Static manifest version: `godot_node_tree_manifest.v1`。
- Dynamic Godot generation accepts robot mechanical schema versions `1.1`, `1.2`, `1.3`, `1.4`, and `1.5`; current emitted schema version is `1.5`.
- Path map mismatch kinds: `missing`, `unexpected`, `value_mismatch`, `duplicate`, `root_mismatch`。
- Static evidence artifacts must include report JSON, gate JSON, sidecar directory, validation summary JSON and a closeout JSON.
- Summary counts must be non-negative; mismatch kind-count maps must sum to their aggregate mismatch count.
- Runtime restoration evidence must map back to static fields: `part_id`, `connection_name`, node paths, classes, `origin`, `axis`, and `applied_parameters`。
- Runtime smoke `node_tree_manifest` must expose `static_manifest_version` and `static_manifest_comparison` when static-vs-runtime evidence is available.
- Runtime static-vs-runtime comparison includes part `mass`, `collision_parameters`, and `mesh_parameters` when runtime mappings expose them.
- Web/session Godot delivery records must expose `static_node_tree_manifest_evidence` and the `web_godot_delivery` gate must preserve static manifest validity, completeness, output coverage and path-map mismatch summary counts.
- Release/readiness summaries must expose `dynamic_godot_release_readiness_summary.v1` and report the strongest proven level using the ordered levels `static_only`, `godot_load_verified`, and `godot_verified`.
- Live Godot verification profiles must record Godot executable resolution, profile name, environment mode, failure category, artifact paths, retry/flaky classification and retention policy; structured failure categories include `missing_godot_executable`, `godot_launch_failure`, `godot_tcp_timeout`, and `godot_runtime_failure`.
- Opt-in live/external CI jobs must keep auditable contracts: they may remain manual/scheduled rather than PR-required, but each job must preserve its explicit trigger condition, enablement/environment signals, local command fragments, uploaded artifact name/path and artifact retention policy.
- Dynamic Godot report live smoke invocations auto-select a free localhost TCP port by default, record smoke attempt metadata, update report-level flaky policy attempt/classification fields, and retry once on auto-port TCP startup response failures; explicit `--port <port>` remains the fixed-port override for diagnostics and does not auto-retry.
- After a dynamic Godot report wrapper retry, the final retained smoke JSON artifact and report-level `godot_smoke` evidence must agree on `live_verification.flaky_policy.attempts_recorded`, `max_attempts`, and `classification`; the retained smoke artifact also records wrapper attempt summaries under `live_verification.flaky_policy.wrapper_attempts`.
- Runtime mechanical behavior evidence must use structured fields for joint limits, torque/velocity response, center of mass, contact state and step-by-step motion trace, with tolerances and units documented before enforcement.
- Runtime mechanical behavior evidence includes threshold failure details; joint limit violations expose joint name, relative angle, configured lower/upper limits and margin values so behavior risks are diagnosable rather than only counted.
- Fixed joints are validated by fixed-lock restoration evidence and are excluded from hinge-style angular joint-limit violation checks.
- Web evidence surfaces must preserve the same evidence levels, manifest mismatch counts and residual risk fields used by CLI/readiness artifacts.
- Schema 1.5 must be additive by default; actuator, sensor, joint limit, controller tuning and material/physics preset fields remain optional until migration tests prove compatibility.
- Schema 1.5 optional fields use additive robot JSON keys: connection-level `actuator`, `sensor`/`sensors`, extended `limits.effort`/`limits.velocity`, connection-level `controller`, and part-level `material`/`physics`. Older configs remain valid without these fields.
- Release evidence bundles use `dynamic_godot_release_evidence_bundle.v1` and self-validation uses `dynamic_godot_release_evidence_bundle_validation.v1`; bundle indexes link static closeout, gate, readiness, optional live smoke, optional Web delivery record, validation report and documentation artifacts with unique artifact keys/documentation roles, correct required flags, bundle-root-relative paths, size, SHA-256 and source/bundle modified timestamp metadata. Bundle validation delegates bundled delivery gate shape checks to the shared `delivery_acceptance_gate.v1` contract and checks that index `bundle_root`, timezone-aware `generated_at`, `readiness_status`, `residual_risks`, `validation_status`, the bundled validation report status, validation report error/status self-consistency, validation report bundle root, required artifact/doc role contract lists, artifact/documentation counts and evidence level match the current validation result. When optional live smoke is supplied, bundle validation checks its live verification profile and wrapper retry metadata consistency. When optional Web delivery record evidence is supplied, bundle validation checks its `web_godot_delivery/godot_load` gate metadata and static manifest evidence consistency.
- No-hardware acceptance reports use `hardwareless_acceptance_report.v1`; they may record mock/replay, fake-runtime, live Godot and compose evidence commands, but must keep real robot hardware, real serial/CAN transport and missing ROS2 runtime as external blockers rather than marking hardware validation as passed.
- No-hardware acceptance reports include `hardwareless_safety_scenarios` so command limit clamping, watchdog fallback, fault-class recovery, serial protocol replay, Web recovery permission gates and live Godot mountain mechanical evidence are tracked as covered mitigations before any remaining residual risk is accepted.
- No-hardware acceptance reports expose `required_external_evidence` for real hardware closeout and ROS2 bridge live smoke; missing evidence remains an external blocker, and passed/ready artifacts remove those blockers without changing mock/replay semantics.
- No-hardware acceptance strict mode uses `--require-external-evidence` and must return `blocked` when real hardware closeout or ROS2 bridge live smoke evidence is missing or not ready.
- No-hardware acceptance reports must expose `release_gate.status`; release-gate status remains `blocked` while required external evidence is missing, even if the local hardwareless report status is `accepted_with_documented_external_blockers`.
- No-hardware release readiness is validated by `tools/validate_hardwareless_release_gate.py`, which fails unless `release_gate.status` matches the expected value.
- Security residual-risk tracking uses `vulnerability_exception_burndown_report.v1` as a non-gating artifact that summarizes active temporary exceptions by expiry, ticket, component, image ref and highest severity; security evidence collection preserves it and security preflight may surface its metrics, but it must not change `security-preflight` pass/fail behavior.

# Integration Plan

1. Close static golden evidence into one reusable command and closeout report.
2. Keep CI using the same static evidence path.
3. Extend runtime smoke to compare generated Godot node tree against static manifest.
4. Preserve static/live evidence through Web/session workflows.
5. Add release/readiness summary that states static-only, Godot-load verified, or full motion verified.
6. Productize live Godot verification as opt-in local/manual/scheduled CI profile.
7. Extend `godot_verified` evidence from structure/parameter restoration to explainable runtime mechanical behavior.
8. Add Web operator flow for inspecting evidence levels, mismatches and readiness from generated artifacts.
9. Plan schema 1.5 as additive optional fields with compatibility and migration tests.
10. Package release evidence into a self-validating delivery bundle.

# Tasks

- [x] Create module subplan for static Godot node-tree evidence.
- [x] Add a single golden static evidence command that writes all expected artifacts into one output root.
- [x] Add compact machine-readable closeout report for static evidence readiness.
- [x] Update docs and CI to reference the single command.
- [x] Add tests and run non-live validation.
- [x] Add runtime smoke static manifest comparison contract.
- [x] Add Web/session Godot delivery static manifest evidence contract.
- [x] Add dynamic Godot release/readiness summary contract.
- [x] Add static schema compatibility matrix and reusable negative fixtures.
- [x] Add runtime physical field comparison for mass/collision/mesh readback.
- [x] Preserve Web workflow static manifest sidecar output paths through Godot delivery evidence.
- [x] Keep browser/manual validation separate from the non-live static Godot CI gate.
- [x] Define the release/readiness golden static fixture set as fixed pair, biped, and quadruped.
- [x] Add a manual live smoke checklist with exact artifact paths and expected `godot_verified` fields.
- [x] Document acceptance levels and proof commands for `static_only`, `godot_load_verified`, and `godot_verified`.
- [x] Add productized live Godot verification profile with executable discovery, failure taxonomy, artifact retention and flaky policy.
- [x] Add runtime mechanical behavior evidence for joint limits, torque/velocity response, center of mass, contact state and step trace.
- [x] Add Web evidence dashboard and artifact actions for static/load/live levels, manifest mismatch and readiness summary.
- [x] Plan additive robot schema 1.5 fields with backward compatibility and migration tests.
- [x] Add self-validating release evidence bundle for delivery artifacts and documentation index.
- [x] Add local deterministic mountain humanoid biped simulation example and evidence artifacts.
- [x] Add no-hardware acceptance report and targeted substitute validation evidence.

# Risks and Mitigations

- Risk: New wrapper duplicates existing report/gate logic.
  Mitigation: Wrapper must invoke existing tools and summarize their artifacts instead of reimplementing validation logic.

- Risk: CI and docs drift from the wrapper command.
  Mitigation: Add tests that inspect the workflow and execute the wrapper locally.

- Risk: Static closeout is mistaken for live Godot proof.
  Mitigation: Closeout report must explicitly mark live Godot as not run.

- Risk: Live Godot validation is environment-dependent and flaky.
  Mitigation: Keep it opt-in for local/manual/scheduled profiles, classify failures, retain artifacts and define retry policy before making it release blocking.

- Risk: Mechanical behavior metrics become inconsistent across fixtures or physics settings.
  Mitigation: Define units, tolerances, fixture expectations and trace schema before enforcing pass/fail gates.

- Risk: Schema 1.5 breaks older JSON configs.
  Mitigation: Make new fields optional by default and require compatibility/migration tests for versions 1.1 through 1.5.

# Validation

```powershell
py -3.12 -m py_compile tools\build_static_godot_node_tree_evidence.py tools\build_dynamic_robot_generation_report.py tools\validate_delivery_acceptance_gate.py agi_walker\core\api\workflow_contracts.py
py -3.12 -m pytest tests\test_dynamic_godot_robot_generation.py -q
py -3.12 -m pytest tests\test_workflow_contracts.py -q
py -3.12 -m pytest -m "not live" --collect-only -q
```

Phase 2 validation expands this set with module-specific checks:

- live profile dry-run and, when Godot is available, scheduled/manual live smoke artifact validation.
- runtime mechanical behavior fixture tests and report/gate contract checks.
- Web API/UI tests for evidence display and artifact actions.
- schema 1.5 compatibility and migration tests across supported schema versions.
- release evidence bundle self-validation tests.

# Acceptance Criteria

- A single command produces static report, delivery gate, sidecar manifests, validation summary and closeout JSON.
- Closeout JSON records status, artifact paths, sidecar counts, validation errors, mismatch counts and explicit live-smoke residual risk.
- CI static manifest gate uses or is protected by the same golden evidence command path.
- Tests prove the wrapper command and workflow contract.
- A release/readiness summary can state whether supplied evidence proves static-only, Godot-load verified, or full motion verified.
- Static schema compatibility and negative topology/manifest drift fixtures are reusable from `tests/fixtures`.
- Live Godot verification can be run through a documented reusable profile and produces classified, retained artifacts.
- `godot_verified` can include explainable mechanical behavior evidence, not only structural and parameter restoration.
- Web users can inspect evidence level, manifest mismatches and residual risk, and can trigger Godot load/readiness actions from artifacts.
- Schema 1.5 optional fields are planned and tested without breaking older supported schemas.
- A release evidence bundle can be generated and self-validated for delivery handoff.

# Decision Log

- 2026-05-18: Project plan introduced at repository root because `AGENTS.md` requires root `PROJECT_PLAN.md` for cross-module work.
- 2026-05-18: Static Godot node-tree evidence wrapper delegates to existing report builder and sidecar gate validator instead of duplicating validation logic.
- 2026-05-19: Phase 2 focuses on productized live verification, explainable mechanical behavior, Web evidence operations, additive schema 1.5 planning and delivery evidence bundles.

# Change Control

- 2026-05-18: Approved scope addition: static golden evidence closeout command and machine-readable closeout report before runtime Godot restoration work.
- 2026-05-18: Approved runtime restoration contract addition: smoke reports expose static manifest version and static-vs-runtime comparison summary without making live Godot mandatory.
- 2026-05-18: Approved Web/session delivery contract addition: direct Godot load records preserve generated static node-tree manifest evidence and source-specific gate summary counts.
- 2026-05-18: Approved release/readiness contract addition: summarize existing evidence JSON into the strongest proven dynamic Godot generation level without rerunning Godot.
- 2026-05-18: Approved static schema compatibility contract addition: document accepted robot schema versions and reusable negative fixtures for duplicate ids and root drift.
- 2026-05-18: Approved runtime physical readback contract addition: compare static mass/collision/mesh expectations against Godot runtime mappings when available.
- 2026-05-18: Approved Web artifact evidence contract addition: Godot-loadable workflow artifacts preserve known static manifest sidecar output paths into Web delivery evidence and summary counts.
- 2026-05-18: Approved CI evidence boundary addition: browser/manual validation remains explicit release evidence and is not a dependency of the non-live static Godot node-tree gate.
- 2026-05-19: Approved release/readiness golden fixture contract: static evidence defaults and CI use fixed pair, biped, and quadruped.
- 2026-05-19: Approved manual live smoke evidence contract: live checklist names report, gate, smoke, and readiness artifacts plus required `godot_verified` fields while keeping live smoke optional.
- 2026-05-19: Approved acceptance-level documentation contract: each release/readiness level names the command and required fields that prove it.
- 2026-05-19: Approved Phase 2 scope addition: productize live Godot verification, add explainable runtime mechanical behavior evidence, add Web evidence dashboard/actions, plan additive schema 1.5 fields, and generate a self-validating release evidence bundle.
- 2026-05-19: Approved runtime mechanical behavior gate summary contract addition: dynamic report delivery gates expose additive `mechanical_behavior_*` counts for evidence presence, completeness, residual risks, threshold failures, center-of-mass/contact availability and trace artifact output while keeping the fields nonblocking by default.
- 2026-05-19: Approved Web evidence dashboard contract addition: workflow run responses expose `web_godot_evidence_summary.v1` with static/load/live level state, manifest mismatch counts, residual risks, Godot load actions and a run-scoped readiness summary route.
- 2026-05-19: Approved additive schema 1.5 contract: dynamic Godot generation emits schema `1.5`, keeps `1.1` through `1.4` accepted, and treats actuator, sensor, extended limits, controller tuning and material/physics preset fields as optional validated metadata.
- 2026-05-19: Approved release evidence bundle contract addition: dynamic Godot delivery handoff uses `dynamic_godot_release_evidence_bundle.v1` plus `dynamic_godot_release_evidence_bundle_validation.v1` to package and self-check required evidence artifacts and documentation indexes without rerunning Godot.
- 2026-05-19: Plan status closeout: all Phase 2 dynamic Godot generation modules and root tasks are complete; no further PROJECT_PLAN.md task remains in the current approved scope.
- 2026-05-19: Approved live smoke port allocation contract update: report builder defaults to auto-selected free localhost TCP ports to avoid local/manual CI collisions while preserving explicit fixed-port override.
- 2026-05-19: Approved auto-port startup retry contract: report builder records live smoke attempts and retries once only for automatic-port TCP startup response failures, while fixed-port diagnostics remain single-attempt.
- 2026-05-20: Approved live smoke structured failure contract: direct smoke runs write report artifacts for Godot launch, TCP timeout and runtime failures instead of relying on uncaught exceptions or missing output files.
- 2026-05-20: Approved report-level flaky policy evidence update: dynamic Godot report live smoke evidence mirrors wrapper-level attempt count and retry classification into `godot_smoke.live_verification.flaky_policy`.
- 2026-05-20: Approved retained smoke artifact retry metadata contract: after wrapper-level retries, the final archived smoke JSON is synchronized with report-level `live_verification.flaky_policy` retry fields.
- 2026-05-20: Approved retained smoke attempt evidence contract: wrapper-level retry attempt summaries are preserved in the final archived smoke JSON under `live_verification.flaky_policy.wrapper_attempts`.
- 2026-05-20: Approved release bundle live smoke validation contract: optional bundled live smoke artifacts are self-validated for dynamic Godot live profile metadata and wrapper retry consistency.
- 2026-05-20: Approved release bundle Web delivery validation contract: optional bundled Web delivery records are self-validated for `web_godot_delivery/godot_load` gate metadata and static manifest evidence consistency.
- 2026-05-20: Approved release bundle self-contained path contract: artifact and documentation `bundle_path` values must be non-empty bundle-root-relative paths and may not resolve outside the bundle root.
- 2026-05-20: Approved release bundle index uniqueness contract: artifact `key` values and documentation `role` values must be unique so validation cannot silently overwrite duplicate evidence entries.
- 2026-05-20: Approved release bundle timestamp metadata contract: artifact and documentation entries must record timezone-aware `source_modified_at` and `bundle_modified_at`, and validation checks the bundled file mtime against `bundle_modified_at`.
- 2026-05-20: Approved release bundle gate delegation contract: bundled delivery gates and Web delivery record gates are validated through the shared `delivery_acceptance_gate.v1` contract instead of only checking the manifest version marker.
- 2026-05-20: Approved release bundle self-validation report contract: final bundle indexes must reference an in-bundle `dynamic_godot_release_evidence_bundle_validation.v1` report whose status matches the current validation result.
- 2026-05-20: Approved release bundle validation snapshot contract: bundled validation reports must be internally status/error consistent and their artifact count, documentation count and evidence level must match the current bundle index.
- 2026-05-20: Approved release bundle validation contract snapshot hardening: bundled validation reports must also match the current bundle root and required artifact/documentation role contract lists.
- 2026-05-21: Approved production compose smoke contract update: the opt-in production compose live smoke uses the supported `deployment/docker-compose.yml` entrypoint, starts `redis`, `zenoh-router`, `web-panel`, and `workflow-worker`, runs Alembic migrations before Web startup, and proves authenticated workflow execution through the Web API.
- 2026-05-21: Approved mechanical behavior diagnostics hardening: threshold failures include structured detail, and joint-limit summaries carry violation records with angle, limits and margins.
- 2026-05-21: Approved fixed-joint limit classification update: `joint_type=fixed` reports joint-limit telemetry as not applicable so fixed joints do not create false hinge-limit violations.
- 2026-05-21: Approved no-hardware acceptance contract addition: `hardwareless_acceptance_report.v1` records substitute evidence and external blockers without weakening real hardware live closeout.
- 2026-05-21: Approved no-hardware safety scenario matrix addition: hardwareless acceptance must separate locally covered safety mitigations from irreducible external live hardware/ROS2 blockers.
- 2026-05-21: Approved required external evidence gates for no-hardware acceptance: real hardware closeout and ROS2 bridge live smoke are resolvable artifact inputs rather than duplicated residual-risk text.
- 2026-05-21: Approved strict no-hardware release gate: `--require-external-evidence` converts unresolved external evidence blockers into a blocking report status.
- 2026-05-21: Approved machine-readable release gate verdict for no-hardware reports so local acceptance evidence cannot be misread as release readiness.
- 2026-05-21: Approved standalone no-hardware release gate validator to remove hand-rolled JSON parsing from release scripts.
- 2026-05-21: Approved security preflight scanner normalization fix: Python vulnerability scan runners treat pip-audit "No known vulnerabilities found" text output as a clean structured report, and container scan project roots are resolved before Docker volume mounting.
- 2026-05-22: Approved repository presentation optimization: root README becomes a concise GitHub entry, detailed docs move behind `docs/README.md`, and tracked sample/artifact-like directories are documented rather than deleted in this pass.
- 2026-05-22: Approved repository presentation metadata hardening: GitHub generated-file attributes, repository layout guidance and PR validation prompts may be added without changing runtime, API, CI or Godot contracts.
- 2026-05-22: Approved public collaboration entrypoint hardening: contribution, conduct and security policy docs may be refreshed for GitHub readiness without changing runtime, API, CI or deployment behavior.
- 2026-05-22: Approved issue triage template hardening: GitHub issue templates may collect module scope, contract impact, validation evidence and residual risk without changing runtime, API, CI or deployment behavior.
- 2026-05-22: Approved issue routing hardening: GitHub issue template contact links may route security and documentation questions to existing policy/docs entry points without changing runtime, API, CI or deployment behavior.
- 2026-05-22: Approved support entrypoint hardening: a root support policy may route usage, release, live-environment and security requests to existing docs and issue templates without changing runtime, API, CI or deployment behavior.
- 2026-05-22: Approved CODEOWNERS governance hardening: GitHub review ownership may be declared for source, docs, deployment, Godot, hardware and governance files without changing runtime, API, CI or deployment behavior.
- 2026-05-23: Approved security preflight blocker triage scope: CI security preflight failures must be classified through scanner execution, unresolved findings, stale/expired exceptions and managed exception matching before any release gate behavior is changed.
- 2026-05-23: Approved Starlette vulnerability remediation constraint: Python project and Web Panel deployment dependency contracts explicitly require `starlette>=1.0.1` to resolve `PYSEC-2026-161` while preserving existing FastAPI and scanner gate behavior.
- 2026-05-23: Approved security-only preflight profile: `run_security_release_preflight.py --security-only` and `collect_release_evidence.py --security-only` collect only security posture evidence so CI security scans are not blocked by broad non-live release gates while default full release evidence behavior remains unchanged.
- 2026-05-31: Approved security preflight exception review hardening: `security-preflight` must fail closed when vulnerability exception review evidence is missing, invalid, blocked or has review-due accepted exceptions requiring follow-up.
- 2026-05-31: Approved no-fix exception scope hardening: active container no-fix vulnerability exceptions must include explicit vulnerability IDs so future no-fix CVEs for the same component cannot be accepted by a broad component-only exception.
- 2026-05-20: Approved release bundle index metadata hardening: bundle validation must verify index `bundle_root`, timezone-aware `generated_at`, `readiness_status` and `residual_risks` against the current bundle and readiness summary.
- 2026-05-20: Approved release bundle required-flag contract: artifact and documentation index entries must mark only required artifact keys and required documentation roles as `required=true`.
- 2026-05-20: Approved additive example scope: local mountain humanoid biped simulation may add example config, CLI, docs and tests without changing dynamic Godot release gate contracts or requiring live Godot.
- 2026-05-31: Approved security-preflight CI enforcement update: `security-preflight` runs on PRs as well as manual and scheduled workflows, using the existing security-only profile so vulnerability posture evidence is not skipped on release-fix PRs.
- 2026-05-31: Approved opt-in live/external CI auditability contract: manual/scheduled live and external-environment jobs remain opt-in, but workflow tests now fail if their trigger conditions, enablement signals, artifact names/paths or retention policies drift.
- 2026-06-05: Approved security exception scope update: scheduled main run `26998884678` found `deployment-web-panel-distributed` `perl-base` `CVE-2026-7010` with no Trivy fixed version; the existing temporary no-fix exception is extended only to that explicit CVE while keeping the current `2026-08-24T00:00:00+01:00` expiry and security-preflight fail-closed behavior.
- 2026-06-10: Approved security exception scope update: scheduled main run `27257251046` found no-fix OpenSSL findings for `deployment-web-panel-distributed` components `openssl`, `libssl3t64`, and `openssl-provider-legacy`, and severity drift on existing `perl-base`/`libbz2-1.0` no-fix findings. Exceptions remain explicit CVE-scoped, keep the `2026-08-24T00:00:00+01:00` expiry, and do not change security-preflight fail-closed behavior.
- 2026-07-08: Approved security exception burn-down artifact: active temporary no-fix exceptions may be summarized into `vulnerability_exception_burndown_report.v1` for residual-risk review and remediation planning, preserved in security evidence artifacts and surfaced in preflight metrics, but the artifact remains advisory and does not weaken the existing security-preflight gate.
- 2026-07-08: Approved web panel container base-image pin: `deployment/Dockerfile.web_panel` defaults to the reproducible `python:3.11-slim-trixie` suite tag through `WEB_PANEL_BASE_IMAGE`, and compose exposes `AGI_WALKER_WEB_PANEL_BASE_IMAGE` for controlled candidate comparison. Remote Docker/Trivy security-preflight artifacts remain the required proof before any alternate base-image candidate is promoted as a vulnerability burn-down.
- 2026-07-08: Approved Web Panel JWT dependency remediation: HS256 token encode/decode moves from `python-jose[cryptography]` to `PyJWT`, and the matching Python/container `ecdsa` no-fix exceptions are retired only when security-preflight proves the findings are gone and no stale exceptions remain.
- 2026-07-08: Approved Web Panel Alpine candidate: `deployment/Dockerfile.web_panel` may use `python:3.11-alpine` plus `apk` package-manager support as the default candidate only if remote Docker/Trivy security-preflight proves fewer production findings and no unresolved/stale exception drift.
