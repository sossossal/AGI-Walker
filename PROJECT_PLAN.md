# Goal

把 AGI-Walker 的 JSON-to-Godot 动态机械节点树能力推进到可验证、可交付、可持续维护的工程闭环：从 JSON/normalized config 到静态 Godot node-tree manifest，再到报告、delivery gate、CI evidence、可选 live Godot smoke 和 Web/session 交付证据，保持同一组契约字段和验收语义。

下一阶段把已完成的静态/运行时还原闭环推进为产品化验收能力：可复用 live Godot 验证 profile、可解释机械行为 evidence、Web 操作闭环、schema 1.5 演进计划，以及面向交付的 evidence bundle。

Phase 3 将在现有 Godot/分布式/硬件验收基础上增加机器人控制与通信模拟主线：从 Python/asyncio 的确定性通信和时序模拟开始，同步提供 Godot 脚本执行与日志 artifact 留存，逐步接入 Zenoh、EtherCAT 周期模型、电机/关节物理模型、外部仿真器适配层，最后以 fail-closed 的方式过渡到真实 CAN/EtherCAT/TSN 硬件。

Security remediation track 将把当前临时接受的 no-fix 容器漏洞风险推进到可关闭状态：生产镜像最终不得依赖 active vulnerability exceptions 作为通过条件，而应通过包升级、基线镜像替换、最终镜像减包、运行时边界拆分或非生产范围隔离，把 `accepted_vulnerability_findings` 清零或移出生产发布面。

# Scope

In scope:

- 动态 Godot robot generation 的 schema、manifest、report、gate、CI、fixture、测试和文档。
- 与该能力直接相关的 Godot runtime restoration、Web/session evidence preservation 和 release evidence。
- 项目级计划、模块子计划、跨模块契约和验收标准维护。
- Phase 2: live Godot 验证产品化、运行时机械完整性增强、Web evidence dashboard、机器人 schema 1.5 规划、发布验收包。
- Phase 3: 控制/通信/现场总线模拟的分层契约、确定性时序 evidence、Godot 脚本/日志 evidence、Zenoh 通信模拟、EtherCAT 周期模型、电机/关节模型、仿真器适配规划和真实硬件迁移边界。
- Security remediation track: 容器漏洞完整修复路线、生产镜像依赖减面、候选基线镜像对比、strict zero-exception release profile、exception retirement 和可审计 evidence bundle。

Out of scope:

- 不默认要求每个 PR 运行 live Godot；live 验证先进入本地/手动/定时 CI profile。
- 不替换当前 JSON robot config 格式。
- 不做无关重构、格式 churn 或跨领域发布改造。
- 不在 schema 1.5 规划阶段强制迁移旧 fixture 或删除旧 schema 兼容。
- 不在 Phase 3 第一阶段直接接入真实 CAN/EtherCAT/TSN 硬件或要求 Gazebo/MuJoCo/Isaac Sim 可用。
- 不用永久 exception、scanner suppression、禁用 Trivy、忽略 fixed version 或手工编译不可维护的系统库来宣称安全风险“全部修复”。

# Existing Context

- 根级 `AGENTS.md` 要求后续编程任务默认遵循 `$codex-project-plan`、`$closure-first-engineer`、`$efficient-engineer`、`$enterprise-code-acceptance`。
- `docs/guides/DYNAMIC_GODOT_ROBOT_GENERATION_FUTURE_PLAN.md` 已记录本功能未来路线。
- 当前静态 `godot_node_tree_manifest.v1`、sidecar self-validation、delivery gate strict scan 和 CI static manifest gate 已实现。
- 当前 `static_only`、`godot_load_verified`、`godot_verified` 验收层级和证明命令已文档化。
- 根级 `PROJECT_PLAN.md` 和 `plans/modules/` 是本轮按 `AGENTS.md` 新建的计划入口。
- 当前已有 `agi_walker/core/distributed/`、`agi_walker/core/api/comm/zenoh_interface.py`、`agi_walker/core/api/comm/tcp_zenoh_bridge.py`、控制器模块、硬件文档和 hardwareless acceptance，适合承接控制/通信模拟，但缺少统一时序、消息、现场总线和执行器模型契约。

# Architecture Principles

- Contract first: schema、CLI、report、gate、CI 和 docs 使用同一组字段名和含义。
- Static before live: 能静态证明的 topology、path、parameter 和 completeness 先在 manifest/report/gate 中闭合。
- Evidence over confidence: 每个验收结论必须指向命令、测试、artifact 或明确残余风险。
- Backward compatible by default: 严格失败只由显式 flag、CI profile 或专用 closeout 工具触发。
- Minimal change: 每轮只修改实现目标所需的最小文件集合。
- Simulation before hardware: 控制、通信、现场总线和执行器行为先用可复现本地模拟证明；真实 CAN/EtherCAT/TSN 只能在显式 live profile 与外部 evidence 存在时进入验收。

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
| container-vulnerability-full-remediation | `plans/modules/container-vulnerability-full-remediation.md` | Full-remediation roadmap for production container vulnerability findings, zero-exception acceptance and evidence retention | Security preflight, deployment Dockerfiles, scanner reports, vulnerability exceptions, CI | complete |
| control-communication-simulation | `plans/modules/control-communication-simulation.md` | Python/asyncio deterministic control-loop timing, Godot script/log evidence, local bus contracts, Zenoh/OpenNeuro-like simulation, EtherCAT cycle model, motor/joint model and simulator/hardware migration boundaries | Existing distributed runtime, Godot scripts/runtime smoke, Zenoh comm modules, robot schema 1.5, hardwareless acceptance | complete |
| control-communication-evidence-closeout | `plans/modules/control-communication-evidence-closeout.md` | Reusable closeout artifact for Phase 3 non-live simulation evidence, retained artifact validation and external live hardware blockers | Control/communication simulation artifacts, hardwareless acceptance semantics, release evidence collection | complete |
| control-communication-ci-evidence | `plans/modules/control-communication-ci-evidence.md` | Default non-live CI evidence profile for Phase 3 simulation artifacts and closeout retention | Control/communication simulation CLI, closeout CLI, GitHub Actions | complete |

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
- Dynamic Godot report live smoke invocations auto-select a free localhost TCP port by default, record smoke attempt metadata, update report-level flaky policy attempt/classification fields, and retry once on auto-port TCP startup response failures; explicit `--port <port>` remains the fixed-port override for diagnostics and does not auto-retry.
- After a dynamic Godot report wrapper retry, the final retained smoke JSON artifact and report-level `godot_smoke` evidence must agree on `live_verification.flaky_policy.attempts_recorded`, `max_attempts`, and `classification`; the retained smoke artifact also records wrapper attempt summaries under `live_verification.flaky_policy.wrapper_attempts`.
- Runtime mechanical behavior evidence must use structured fields for joint limits, torque/velocity response, center of mass, contact state and step-by-step motion trace, with tolerances and units documented before enforcement.
- Runtime mechanical behavior evidence includes threshold failure details; joint limit violations expose joint name, relative angle, configured lower/upper limits and margin values so behavior risks are diagnosable rather than only counted.
- Fixed joints are validated by fixed-lock restoration evidence and are excluded from hinge-style angular joint-limit violation checks.
- Web evidence surfaces must preserve the same evidence levels, manifest mismatch counts and residual risk fields used by CLI/readiness artifacts.
- Schema 1.5 must be additive by default; actuator, sensor, joint limit, controller tuning and material/physics preset fields remain optional until migration tests prove compatibility.
- Schema 1.5 optional fields use additive robot JSON keys: connection-level `actuator`, `sensor`/`sensors`, extended `limits.effort`/`limits.velocity`, connection-level `controller`, and part-level `material`/`physics`. Older configs remain valid without these fields.
- Release evidence bundles use `dynamic_godot_release_evidence_bundle.v1` and self-validation uses `dynamic_godot_release_evidence_bundle_validation.v1`; bundle indexes link static closeout, gate, readiness, optional live smoke, optional Web delivery record, validation report and documentation artifacts with unique artifact keys/documentation roles, correct required flags, bundle-root-relative paths, size, SHA-256 and source/bundle modified timestamp metadata. Bundle validation delegates bundled delivery gate shape checks to the shared `delivery_acceptance_gate.v1` contract and checks that index `bundle_root`, timezone-aware `generated_at`, `readiness_status`, `residual_risks`, `validation_status`, the bundled validation report status, validation report error/status self-consistency, validation report bundle root, required artifact/doc role contract lists, artifact/documentation counts and evidence level match the current validation result. When optional live smoke is supplied, bundle validation checks its live verification profile and wrapper retry metadata consistency. When optional Web delivery record evidence is supplied, bundle validation checks its `web_godot_delivery/godot_load` gate metadata and static manifest evidence consistency.
- Dynamic Godot release evidence bundles may optionally include Phase 3 `control_comm_simulation_closeout.v1` as artifact key `control_comm_closeout`; if present, validation requires healthy non-live closeout semantics and keeps the live hardware release gate blocked.
- When the default documentation set is used, release evidence bundles that include `control_comm_closeout` also include optional documentation role `control_comm_workflow` from `docs/guides/CONTROL_COMMUNICATION_SIMULATION.md`.
- Canonical release evidence collection may copy an existing Phase 3 `control_comm_simulation_closeout.v1` via `collect_release_evidence.py --control-comm-closeout-source` into `control_communication/control_comm_simulation_closeout.json`; collection does not generate the simulation and does not upgrade non-live evidence into hardware validation.
- No-hardware acceptance reports use `hardwareless_acceptance_report.v1`; they may record mock/replay, fake-runtime, live Godot and compose evidence commands, but must keep real robot hardware, real serial/CAN transport and missing ROS2 runtime as external blockers rather than marking hardware validation as passed.
- No-hardware acceptance reports include `hardwareless_safety_scenarios` so command limit clamping, watchdog fallback, fault-class recovery, serial protocol replay, Web recovery permission gates and live Godot mountain mechanical evidence are tracked as covered mitigations before any remaining residual risk is accepted.
- No-hardware acceptance reports expose `required_external_evidence` for real hardware closeout and ROS2 bridge live smoke; missing evidence remains an external blocker, and passed/ready artifacts remove those blockers without changing mock/replay semantics.
- No-hardware acceptance strict mode uses `--require-external-evidence` and must return `blocked` when real hardware closeout or ROS2 bridge live smoke evidence is missing or not ready.
- No-hardware acceptance reports must expose `release_gate.status`; release-gate status remains `blocked` while required external evidence is missing, even if the local hardwareless report status is `accepted_with_documented_external_blockers`.
- No-hardware release readiness is validated by `tools/validate_hardwareless_release_gate.py`, which fails unless `release_gate.status` matches the expected value.
- Control/communication simulation uses deterministic local evidence before live transport: `control_comm_simulation_report.v1` records virtual clock source, cycle period, jitter budget, deadline misses, message sequence integrity, transport mode, trace path and residual risks.
- Local asyncio bus, Zenoh transport simulation and future OpenNeuro-like topics must share one canonical message envelope with `schema_version`, `topic`, `sequence`, `timestamp_ns`, `source`, `target`, `payload_type` and typed payload metadata.
- Godot-side control/communication simulation uses the same canonical envelope and produces retained script/log evidence: `godot_control_comm_simulation_log.v1` records script name, Godot executable/profile metadata when available, simulated cycle/message events, output artifact paths, status and residual risks.
- `control_comm_simulation_report.v1` may aggregate Python deterministic traces and Godot script/log traces, but must keep evidence source explicit with `evidence_source` values such as `python_virtual_clock`, `godot_script`, `godot_headless` and future live transport modes.
- EtherCAT simulation remains a cycle/PDO/watchdog/deadline model until real hardware evidence exists; it must not be presented as live EtherCAT validation.
- Motor and joint simulation must expose position, velocity, torque, limits, command saturation, fault state and step trace fields that can map back to robot schema 1.5 actuator/joint-limit/controller metadata.
- Gazebo/MuJoCo/Isaac Sim integrations are adapters over the canonical simulation contracts; core control-loop logic must not depend on a specific simulator runtime.
- Real CAN/EtherCAT/TSN transport stays gated by explicit live profiles, hardware role permissions, operator confirmation and external evidence; no-hardware acceptance remains blocked for real hardware claims.
- Live hardware migration uses `live_hardware_migration_gate.v1`; it must keep CAN, EtherCAT and TSN `status=blocked`, `release_gate.status=blocked` and `simulation_substitute_allowed=false` until explicit external hardware evidence is supplied.
- Control/communication simulation closeout uses `control_comm_simulation_closeout.v1`; it summarizes local non-live simulation evidence and validation errors while preserving live hardware blockers as external blockers, self-validates closeout shape before acceptance, and records retained artifact `size_bytes` plus `sha256` integrity metadata.
- Default CI control/communication evidence uses the `control-communication-simulation-evidence` job to generate `test_env/control_comm_simulation_ci`, build `control_comm_simulation_closeout.json`, and upload `control-communication-simulation-artifacts` without Godot, Zenoh, Docker, external simulators or real hardware.
- Production container vulnerability full-remediation is stronger than the current managed-exception gate: production release evidence must eventually show `unresolved_findings=0`, `stale_exceptions=0`, `expired_exceptions=0`, `review_due=0`, and `accepted_vulnerability_findings=0` for production image refs.
- Strict container vulnerability release acceptance uses `run_security_release_preflight.py --fail-on-accepted-vulnerability-findings`; this profile remains opt-in until candidate images prove zero accepted vulnerability findings.
- Container vulnerability remediation closeout uses `container_vulnerability_remediation_closeout.v1` to combine inventory, reduction-plan and strict-preflight evidence into one blocked/ready verdict.
- Scheduled/manual security CI retains `test_env/container_vulnerability_remediation` artifacts when scanner outputs exist, but default security-preflight pass/fail semantics remain managed-exception compatible unless strict flags are explicitly used.

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
11. Add deterministic Python/asyncio control-loop and message-bus simulation with trace evidence.
12. Add Godot script/headless execution path that replays the same control/message scenario and retains structured logs.
13. Layer Zenoh/OpenNeuro-like transport simulation over the same message envelope.
14. Add EtherCAT cycle/PDO/watchdog model and validate timing/fault evidence without real hardware.
15. Add motor/joint physical response model and connect it to robot schema actuator/joint-limit metadata.
16. Add simulator adapter plan for Gazebo/MuJoCo/Isaac Sim while keeping adapters optional.
17. Define live CAN/EtherCAT/TSN migration gates that require explicit hardware evidence.
18. Add a reusable closeout artifact for Phase 3 control/communication simulation evidence.
19. Retain Phase 3 non-live control/communication simulation evidence in default CI.
20. Allow Phase 3 control/communication closeout evidence to travel with release evidence bundles as an optional validated artifact.
21. Allow canonical release evidence collection to copy existing Phase 3 control/communication closeout evidence.
22. Document and validate the canonical collector-to-bundle handoff path for Phase 3 control/communication closeout evidence.
23. Establish the container vulnerability full-remediation roadmap without weakening the existing security preflight gate.
24. Build current production image vulnerability inventory from security-preflight artifacts and classify findings by fixability, package necessity and production applicability.
25. Evaluate minimal final-image package removals and candidate base images with raw Trivy comparison evidence before changing Dockerfile defaults.
26. Introduce a strict zero-exception release profile only after candidate images pass runtime and security evidence gates.

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
- [x] Add Phase 3 control/communication simulation module plan.
- [x] Define canonical control message envelope and deterministic asyncio timing report.
- [x] Add local asyncio bus timing simulation before external transport.
- [x] Add Godot script/headless control communication simulation that emits retained structured logs.
- [x] Add Zenoh/OpenNeuro-like transport simulation over the canonical envelope.
- [x] Add EtherCAT cycle/PDO/watchdog model as non-live simulation evidence.
- [x] Add motor/joint model evidence and map it to robot schema actuator/joint fields.
- [x] Plan simulator adapters for Gazebo/MuJoCo/Isaac Sim without making them CI requirements.
- [x] Define real CAN/EtherCAT/TSN live migration gates and no-hardware blockers.
- [x] Add control/communication simulation evidence closeout artifact.
- [x] Add default CI profile that generates and uploads Phase 3 non-live control/communication simulation closeout evidence.
- [x] Add optional Phase 3 control/communication closeout artifact support to release evidence bundles.
- [x] Add optional Phase 3 control/communication closeout source support to canonical release evidence collection.
- [x] Document the canonical collector output path as a valid `--control-comm-closeout` bundle input and guard the handoff docs with active-path tests.
- [x] Add a container vulnerability full-remediation plan that defines how temporary no-fix exceptions can be retired rather than renewed indefinitely.

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

- Risk: Communication simulation is mistaken for live hardware validation.
  Mitigation: Reports must state transport mode (`asyncio`, `zenoh_simulated`, `ethercat_model`, `live_hardware`) and keep live hardware gates blocked without external evidence.

- Risk: Control-loop tests become flaky because they use wall-clock timing.
  Mitigation: First implementation must use a deterministic virtual clock and only add real-time smoke as opt-in live evidence.

- Risk: Godot script logs drift from Python simulation contracts.
  Mitigation: Godot logs must use the same canonical envelope fields and be validated as artifacts before being accepted into the aggregate report.

- Risk: Simulator adapters create hard dependencies on Gazebo/MuJoCo/Isaac Sim.
  Mitigation: Keep adapters optional and validate core contracts with local Python tests before simulator-specific smoke.

- Risk: no-fix container findings remain unresolved upstream and cannot be upgraded immediately.
  Mitigation: Treat active exceptions as temporary only; prioritize final-image package removal, safer base image candidates and production-scope reduction before renewing exceptions.

- Risk: base-image replacement fixes scanner findings but breaks runtime behavior.
  Mitigation: Require Docker/Trivy comparison evidence plus compose/Web smoke evidence before changing production image defaults.

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
- Phase 3 control/communication simulation validation:

```powershell
py -3.12 -m py_compile agi_walker\core\api\comm\zenoh_interface.py agi_walker\core\api\comm\tcp_zenoh_bridge.py
py -3.12 -m py_compile agi_walker\core\simulation\control_comm_simulation.py tools\run_control_comm_simulation.py
py -3.12 -m py_compile tools\run_godot_control_comm_replay.py
py -3.12 -m py_compile tools\build_control_comm_simulation_closeout.py
py -3.12 tools\run_control_comm_simulation.py --output-root test_env\control_comm_simulation
py -3.12 tools\build_control_comm_simulation_closeout.py --report test_env\control_comm_simulation\control_comm_simulation_report.json --output test_env\control_comm_simulation\control_comm_simulation_closeout.json
py -3.12 tools\run_control_comm_simulation.py --output-root test_env\control_comm_simulation_ci
py -3.12 tools\build_control_comm_simulation_closeout.py --report test_env\control_comm_simulation_ci\control_comm_simulation_report.json --output test_env\control_comm_simulation_ci\control_comm_simulation_closeout.json
py -3.12 tools\run_godot_control_comm_replay.py --dry-run-discovery --output-root test_env\godot_control_comm_replay
py -3.12 -m pytest tests\test_control_communication_simulation.py -q
py -3.12 -m pytest tests\test_active_path_references.py -q
py -3.12 -m pytest tests\test_distributed_smoke_runner.py tests\test_hardwareless_acceptance_report.py -q
```

Phase 3 implementation includes targeted tests for deterministic virtual clock scheduling, message ordering/drop/jitter simulation, Zenoh transport envelope compatibility, EtherCAT cycle deadline/watchdog behavior, motor/joint response traces, simulator adapter boundaries and live hardware migration gates.
Godot-side Phase 3 validation includes non-live artifact-shape tests for retained Godot simulation logs and an opt-in live/headless Godot replay runner when a Godot executable is available.

Container vulnerability full-remediation validation:

```powershell
py -3.12 tools\run_security_release_preflight.py --security-only --output-root test_env\release_evidence_ci --run-python-vuln-scan --run-container-vuln-scan --container-image-ref deployment-zenoh-router --container-image-ref deployment-web-panel-distributed
py -3.12 tools\compare_container_vulnerability_baselines.py --current-raw-report <current-trivy-raw-json-or-dir> --candidate-raw-report <candidate-trivy-raw-json-or-dir> --output test_env\container_vulnerability_remediation\comparison.json
py -3.12 -m pytest tests\test_security_release_preflight.py tests\test_security_posture_reports.py tests\test_vulnerability_exception_report.py tests\test_vulnerability_remediation_report.py tests\test_vulnerability_scan_runners.py -q --tb=short
py -3.12 -m pytest tests\test_active_path_references.py -q
```

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
- Phase 3 control/communication simulation has a root plan and module plan before implementation.
- Local control-loop timing evidence is deterministic and reproducible without hardware.
- Godot script/headless simulation produces retained structured logs that can be traced back to the same message envelope and report fields.
- Zenoh/OpenNeuro-like and EtherCAT simulations reuse the same canonical envelope and do not weaken live hardware gates.
- Motor/joint model traces map back to robot schema actuator and joint-limit metadata.
- Gazebo/MuJoCo/Isaac Sim adapters remain optional until their runtime-specific evidence exists.
- Control/communication closeout can summarize generated non-live artifacts while preserving live hardware blockers.
- Default CI retains Phase 3 non-live control/communication simulation artifacts and closeout without requiring Godot, real Zenoh, external simulators or hardware.
- Release evidence bundles can carry optional Phase 3 control/communication closeout evidence and reject malformed or hardware-unblocked closeouts.
- Canonical release evidence collection can retain existing Phase 3 control/communication closeout evidence without rerunning simulation or weakening live hardware gates.
- The canonical copied Phase 3 closeout path can be fed directly into dynamic Godot release bundle generation as the optional `control_comm_closeout` artifact.
- Bundled Phase 3 control/communication closeout evidence carries its matching operator/developer guide as optional bundle documentation when default docs are used.
- Container vulnerability full-remediation has an executable plan that defines zero active production vulnerability exceptions as the final acceptance state.
- Production container vulnerability remediation cannot be marked complete while `accepted_vulnerability_findings > 0` for production image refs, even if the current managed-exception preflight passes.
- Candidate production images must prove both vulnerability reduction and runtime compatibility before replacing Dockerfile defaults.

# Decision Log

- 2026-05-18: Project plan introduced at repository root because `AGENTS.md` requires root `PROJECT_PLAN.md` for cross-module work.
- 2026-05-18: Static Godot node-tree evidence wrapper delegates to existing report builder and sidecar gate validator instead of duplicating validation logic.
- 2026-05-19: Phase 2 focuses on productized live verification, explainable mechanical behavior, Web evidence operations, additive schema 1.5 planning and delivery evidence bundles.
- 2026-05-26: Phase 3 planning starts as a new control/communication simulation line, separate from completed dynamic Godot generation work, with deterministic local simulation before live hardware.
- 2026-05-26: Phase 3 includes Godot-side script/log evidence so control/communication simulation can be replayed in Godot and retained as structured artifacts, not only proven by Python traces.
- 2026-05-26: Phase 3 implementation starts with canonical message envelope, deterministic local report generation, Godot log contract preview and retained artifact validation before live transports.
- 2026-05-26: Phase 3 local asyncio bus simulation now records virtual latency, per-cycle jitter, drop, duplicate, delivery order, deadline misses and message integrity before any external transport is used.
- 2026-05-26: Phase 3 Godot replay runner now discovers a Godot executable, runs `control_comm_replay.gd` when available, archives `godot_control_comm_simulation_log.v1` and validates the retained log artifact.
- 2026-05-26: Phase 3 Zenoh/OpenNeuro-like simulation now maps the canonical envelope to deterministic key expressions and retained simulated transport traces without opening a real Zenoh session or claiming OpenNeuro compatibility.
- 2026-05-26: Phase 3 EtherCAT cycle model now records deterministic PDO inputs/outputs, deadline misses, missing frames, watchdog state and fault class as non-live fieldbus evidence.
- 2026-05-26: Phase 3 motor/joint model now records deterministic position, velocity, torque, saturation, limit state and fault state traces mapped to actuator, joint-limit and controller schema concepts.
- 2026-05-26: Phase 3 simulator adapter boundary now defines Gazebo, MuJoCo and Isaac Sim as optional adapters over canonical traces without adding runtime dependencies or CI requirements.
- 2026-05-26: Phase 3 live hardware migration gate now records CAN, EtherCAT and TSN as blocked until external live hardware evidence, hardware role permission and operator confirmation exist.
- 2026-05-26: Phase 3 control/communication simulation plan is closed as complete for the approved non-live scope; remaining real hardware, external simulator and live transport execution are documented residual risks, not open local implementation tasks.
- 2026-05-26: Phase 3 evidence closeout now produces a reusable `control_comm_simulation_closeout.v1` artifact over the generated non-live report and retained artifacts.
- 2026-05-26: Phase 3 closeout artifact now has independent self-validation so malformed closeout JSON cannot be accepted only because the source report was readable.
- 2026-05-26: Phase 3 closeout artifact now records size and SHA-256 metadata for each retained artifact so evidence bundles can detect silent artifact replacement.
- 2026-05-26: Phase 3 non-live control/communication simulation evidence is now retained by default CI through a dedicated artifact job, while live/runtime validations remain opt-in.
- 2026-05-26: Dynamic Godot release evidence bundles can now carry optional Phase 3 control/communication closeout evidence without making it required for older handoffs.
- 2026-05-26: Canonical release evidence collection can now copy existing Phase 3 control/communication closeout evidence into the release evidence tree.
- 2026-05-26: The Phase 3 collector-to-bundle handoff is documented as a direct path from `test_env/release_evidence/control_communication/control_comm_simulation_closeout.json` to bundle `--control-comm-closeout`.
- 2026-05-26: Dynamic Godot release bundles that include Phase 3 control/communication closeout evidence now include the matching optional guide documentation by default.
- 2026-06-10: Container vulnerability full-remediation is tracked as a separate security delivery track because passing with active time-bounded no-fix exceptions is release-risk management, not complete remediation.
- 2026-06-10: The final production security acceptance target is zero accepted vulnerability findings for production image refs; managed exceptions remain a compatibility gate only until package removal, package upgrades or safer base images remove the findings.

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
- 2026-05-20: Approved release bundle index metadata hardening: bundle validation must verify index `bundle_root`, timezone-aware `generated_at`, `readiness_status` and `residual_risks` against the current bundle and readiness summary.
- 2026-05-20: Approved release bundle required-flag contract: artifact and documentation index entries must mark only required artifact keys and required documentation roles as `required=true`.
- 2026-05-20: Approved additive example scope: local mountain humanoid biped simulation may add example config, CLI, docs and tests without changing dynamic Godot release gate contracts or requiring live Godot.
- 2026-05-26: Approved Phase 3 scope addition: add robot control/communication simulation planning from Python/asyncio timing and Zenoh/OpenNeuro-like transport through EtherCAT cycle, motor/joint model, simulator adapters and eventual real CAN/EtherCAT/TSN hardware gates.
- 2026-05-26: Approved Phase 3 contract expansion: control/communication simulation must include Godot script or headless replay logs using the canonical envelope, with artifact retention and report aggregation before it can count as Godot-side simulation evidence.
- 2026-05-26: Approved Phase 3 simulated Zenoh/OpenNeuro evidence contract: topic mappings and traces must use `transport_mode=zenoh_simulated` and `compatibility_claim=simulation_only`; real Zenoh/OpenNeuro validation remains outside non-live evidence.
- 2026-05-26: Approved Phase 3 EtherCAT model evidence contract: cycle/PDO/watchdog traces must use `transport_mode=ethercat_model` and `compatibility_claim=simulation_only`; real EtherCAT master or fieldbus hardware validation remains outside non-live evidence.
- 2026-05-26: Approved Phase 3 motor/joint response evidence contract: response traces must expose command, position, velocity, torque, saturation, limit state, fault state and schema mapping while keeping real motor-driver validation outside non-live evidence.
- 2026-05-26: Approved Phase 3 simulator adapter boundary contract: Gazebo, MuJoCo and Isaac Sim integrations remain adapter-contract artifacts with `runtime_dependency_required=false` and `runtime_status=not_run` until runtime-specific evidence is supplied.
- 2026-05-26: Approved Phase 3 live hardware migration gate contract: `live_hardware_migration_gate.v1` keeps CAN/EtherCAT/TSN and release readiness blocked by default, and local simulation artifacts are never accepted as live hardware substitutes.
- 2026-05-26: Approved Phase 3 closeout status update: the approved local simulation scope is complete, while live Godot, real Zenoh/OpenNeuro compatibility, external simulator runtime validation and real CAN/EtherCAT/TSN validation remain explicit opt-in or external-evidence lanes.
- 2026-05-26: Approved Phase 3 closeout artifact contract: `control_comm_simulation_closeout.v1` may accept local non-live simulation evidence while retaining real hardware and external runtime validation as blockers.
- 2026-05-26: Approved Phase 3 closeout self-validation contract: closeout artifacts must validate their own version, status, evidence level, artifact error counts and live hardware release gate status.
- 2026-05-26: Approved Phase 3 closeout artifact integrity metadata: each present artifact result records `size_bytes` and `sha256`; missing artifacts must keep those fields null.
- 2026-05-26: Approved Phase 3 CI evidence profile contract: default CI may generate and upload non-live control/communication simulation artifacts and closeout, but it must not require Godot, Zenoh, Docker, external simulators or real hardware.
- 2026-05-26: Approved release bundle optional artifact contract: `control_comm_closeout` may be bundled when Phase 3 evidence exists, and validator must reject malformed closeouts or closeouts that claim live hardware release readiness.
- 2026-05-26: Approved release collector optional source contract: `--control-comm-closeout-source` may copy existing Phase 3 closeout evidence into canonical release evidence, but it must not run live transport, hardware, or simulator validation.
- 2026-05-26: Approved collector-to-bundle handoff contract: the canonical copied closeout path may be passed directly to release bundle `--control-comm-closeout`; this remains optional and does not upgrade non-live evidence into live hardware validation.
- 2026-05-26: Approved optional control/communication bundle documentation contract: default bundle docs include role `control_comm_workflow` when `control_comm_closeout` is present; the role is optional and explicit `--doc` inputs remain caller-controlled.
- 2026-06-10: Approved security remediation scope addition: add a container vulnerability full-remediation roadmap whose completion target is production `accepted_vulnerability_findings=0`, not repeated renewal of temporary no-fix exceptions.
