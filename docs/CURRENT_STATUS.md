# AGI-Walker Current Status

更新日期：`2026-04-20`

## 摘要

AGI-Walker 当前可用的主入口已经恢复到可读、可执行、可验证的状态。此次修复的重点是：

- 仓库首页 `README.md` 已恢复为正常 UTF-8 文档。
- MCP `stdio` server 已可通过 `agi-walker-mcp` 和 `python -m agi_walker.mcp.server` 启动。
- `mcp>=1.27.0` 下的初始化兼容问题已修复。
- CLI、Workflow、Web Panel 和 Godot Agent 的入口文档已重新对齐到真实代码接口。
- 阶段一主链稳定化已完成：Workflow v1 契约、真实产物 JSON schema、工作流定义校验和回归测试已经落地。
- 阶段二 Web/Godot 操作闭环已完成：Web workflow artifact manifest、Godot session 状态机、delivery record 和前端展示已复用 v1 契约。
- 阶段三 Distributed Runtime 已完成并完成本机实机验收：Docker smoke report v1、distributed monitor v1、actor discovery 状态、compose log 诊断和 live smoke 证据都已落地。
- 阶段四训练、仿真与硬件边界已完成：`training_run` artifact v1 已落地，mock RL、offline dataset training、sim training 路径会写出可校验训练 manifest，HITL 分类已进入回归测试；IMC-22、真实串口驱动和 ROS2 bridge 的 replay/mock 已接入默认 pytest，Godot headless live smoke 与 ROS2 bridge 的真实 Humble live smoke 都已留存结构化通过报告。
- Nightly / manual 专项回归当前已扩展到 `ros2-bridge-smoke`，并为该 live smoke 增加了 CI job、nightly 跟踪和 artifact 上传入口。
- 阶段五产品化与发布门禁已完成：`capability_matrix` contract v1、`release_manifest` contract v1、发布指南和 release artifact 生成脚本已落地，并通过 smoke 与默认非 live 门禁验收。
- required release evidence 现在优先来自结构化报告，而不是固定摘要；canonical 路径已收口到 `clean_checkout_smoke`、`non_live_gate` 和 targeted release tests，并且都会带上生成时间、命令、报告路径和 commit SHA 进入 release builder。
- `release_policy`、`release_approval` 与 `release_source` 已成为 release manifest 的机器可读字段，`dev / rc / stable / industrial` 的 gate 语义、stable/industrial 签核要求，以及“签核 SHA 必须绑定当前 Git HEAD、版本必须有匹配 Git tag”的规则已经收口到同一套 contract，而不是继续依赖人工解释。
- stable gate 现在还会显式检查 Git worktree 是否 clean，避免在脏工作区上误做 stable promotion。
- 显式 stable 发布演练脚本 `tools/run_release_rehearsal.py` 已落地，并已接入默认 smoke；当前它不再只验证 `release_gate_status=ready`，还会显式 seed rehearsal customer docs / security evidence，并要求 `customer_delivery_surface.status=ready` 与 `industrial_delivery_gate.status=ready`。相对或绝对 `--output-root` 路径现在都能稳定生成闭合 rehearsal 产物。
- `tools/run_clean_checkout_smoke.py` 已在目标 clean checkout 上完成 2 次连续默认 smoke 验证，并证明每次执行后 `git status --short` 仍为空；默认 smoke 的已知 tracked artifact 污染路径已从“推测”升级为“已消除并有报告佐证”。
- 阶段六的阶段 C 客户部署包装已完成：Docker Compose 已被固定为唯一一线客户部署路径，`deployment/compose.env.example`、`deployment/web_panel.env.example`、客户安装指南和部署矩阵都已落地，Compose 端口、卷和持久化目录约定已进入代码、文档和回归测试。
- 阶段六的阶段 E 当前已完成第二批客户交付文档接线：`SUPPORT_MATRIX.md`、`CAPACITY_AND_SCALE.md`、`CUSTOMER_ACCEPTANCE_CHECKLIST.md` 与 `KNOWN_LIMITATIONS.md` 已落地，并已进入 `customer_acceptance_bundle` 的默认 required docs。
- 阶段六的阶段 D 已开始：`sbom_artifact`、`vulnerability_scan_report` 和 `security_posture_report` contract 已落地，4 份安全/运维基线文档已进入仓库，默认 smoke 已新增 SBOM 与 security posture 检查入口。
- 阶段六的阶段 D 当前已继续推进到 canonical evidence / customer acceptance 主链：`backup_restore_rehearsal_report` contract 与 rehearsal runner 已落地，`tools/collect_release_evidence.py` 现会在 `test_env/release_evidence/security/` 下统一收集 SBOM、漏洞报告、恢复演练报告和 `security_posture_report`，客户验收包也会显式暴露安全态势摘要。
- `tools/write_vulnerability_scan_report.py` 与 `tools/collect_release_evidence.py` 当前已支持导入真实 scanner 原始 JSON，并规范化为结构化漏洞报告；当前支持格式为 `pip-audit-json` 和 `trivy-json`。
- 阶段六的阶段 D 当前已新增真实 scanner wrapper：`tools/run_python_vulnerability_scan.py` 与 `tools/run_container_vulnerability_scan.py`。`tools/collect_release_evidence.py` 现在可在未提供结构化报告或 raw JSON 时，通过 `--run-python-vuln-scan` / `--run-container-vuln-scan` 直接执行扫描。
- canonical Python vulnerability scan 当前默认先生成一份与客户部署面一致的 requirements snapshot，只覆盖 `project.dependencies`、`deployment/requirements.web_panel*.txt` 和 `deployment/requirements.distributed_runtime.txt`；`training` / `hardware` optional groups 改为显式 opt-in，不再默认混入 canonical baseline。
- 阶段六的阶段 D 当前已新增正式预检入口 `tools/run_security_release_preflight.py`。该脚本会调用 `tools/collect_release_evidence.py`、读取 `security_posture_report.json`，并生成 `security_release_preflight_report.json`；只有在 security posture ready 时才返回成功。
- `.github/workflows/ci.yml` 当前已新增 `security-preflight` job，专门在 manual/schedule 路径上执行真实 `pip-audit` / `trivy` 安装、镜像构建与 `run_security_release_preflight.py`。
- `tools/check_release_readiness.py` 与 `tools/build_stable_promotion_checklist.py` 当前已直接读取 canonical `security_release_preflight_report.json`；stable promotion 现在会把 security preflight 当作显式 blocking step，而不是只依赖 manifest gate。
- `tools/check_release_readiness.py` 与 `tools/build_stable_promotion_checklist.py` 当前已开始显式暴露 `customer_delivery_surface`；Phase E 的 support matrix、capacity declaration、acceptance checklist 和 known limitations 已不再只停留在 bundle，而是进入 readiness / checklist 的机器可读输出。
- `release_manifest` contract 当前已开始正式携带 `customer_delivery_surface`，因此 Phase E 文档面已经从 bundle/checklist 摘要升级为 release artifact 的机器字段，可直接作为 future industrial gate 的输入。
- `release_manifest` contract 当前已新增 `industrial_delivery_gate`，并把 `deployment_package_status`、`evidence_attested`、`sbom_attached`、`vuln_scan_status`、`backup_restore_verified` 以及 support/capacity/checklist/limitations 挂载状态收口成正式机器字段。
- 正式 `industrial` release channel 当前已落地；它继承 stable gate，并额外要求 `customer_delivery_surface.status=ready` 与 `industrial_delivery_gate.status=ready`，不再只把这两组字段当成桥接摘要。
- `tools/check_release_readiness.py` 与 `tools/build_stable_promotion_checklist.py` 当前已开始同步暴露 `industrial_delivery_gate`；readiness 会直接列出缺失的 industrial requirements，stable checklist 也新增了“确认 industrial delivery gate 已闭合”步骤。
- `tools/check_industrial_release_readiness.py` 与 `tools/build_industrial_promotion_checklist.py` 当前已落地，`industrial` 通道不再继续复用 stable 视角的预览与 checklist；customer delivery / industrial delivery 现在都会以独立工业产物落盘并作为 blocking step 暴露。
- `customer_acceptance_bundle` 当前会按 manifest channel 自动切换 readiness / promotion 报告：`stable` 继续读取 `release_readiness_report.json` / `stable_promotion_checklist.json`，`industrial` 则默认读取 `industrial_release_readiness_report.json` / `industrial_promotion_checklist.json`。
- `customer_delivery_surface.extension_support_surface` 当前已落地，distributed / ROS2 / Godot / Helm-Kubernetes 的支持边界、专项验收要求和非支持范围现在会进入 manifest、industrial gate、readiness、promotion checklist 和 customer acceptance bundle。
- `extension_support_surface` 当前已进一步携带每个扩展 profile 的 `deployment_commands`、`acceptance_checks` 与 `rollback_prerequisites`，专项实施动作不再只停留在支持边界声明。
- `customer_acceptance_bundle`、`industrial_promotion_checklist` 与 `release_rehearsal_report` 当前已开始额外挂出 `extension_execution_plan`，把 actionable profile、推荐部署/验收命令与回滚前提收口成执行面摘要。
- `extension_execution_plan.profiles[*].runbook_entrypoints` 当前已落地，正式 runbook / 安装指南 / distributed、ROS2、Godot 专项指南现在都成为机器可读执行面的固定入口。
- `extension_execution_plan.profiles[*].execution_template` 当前已落地，distributed / ROS2 / Godot / Helm-Kubernetes 扩展面的现场角色分工、升级窗口顺序、值班动作、on-call 交接记录、残余风险交接、exception 到期复核、异常升级闭环证据和回滚证据归档责任现在都已进入机器可读执行面。
- `release_manifest`、`customer_acceptance_bundle`、`industrial_release_readiness_report`、`industrial_promotion_checklist` 与 `release_rehearsal_report` 当前已开始额外挂出 `extension_execution_evidence`，把 `extension_on_call_rehearsal`、`extension_exception_review_schedule`、`extension_escalation_closure` 与 `customer_external_bindings_confirmation` 四类留痕报告压成机器可读交付字段。
- `release_manifest`、`customer_acceptance_bundle`、`industrial_release_readiness_report`、`industrial_promotion_checklist` 与 `release_rehearsal_report` 当前已开始额外挂出 `extension_execution_instance`，把客户实例化的 `engagement_id`、交付窗口、exception 复核到期时间、delivery root、closure archive root 与 profile 级实例化路径一起压成机器字段。
- 同一批产物当前已进一步附带 `extension_execution_schedule`，把 `window_trigger_at`、`signoff_due_at`、`closure_archive_due_at`、`window_trigger_record_path`、`residual_risk_review_record_path` 与 `closure_manifest_path` 收口成客户窗口排程面。
- 同一批产物当前已开始附带 `extension_execution_actuals`，把 `window_trigger_recorded_at`、`signoff_recorded_at`、`residual_risk_reviewed_at`、`closure_archived_at`、`*_recorded_by`、`approval_identity_source_path`、`approval_identity_source_type`、`approval_identity_reference`、`archive_target_binding_type`、`archive_target_binding_reference_base`、`due_trigger_binding_type`、`due_trigger_binding_reference_base`、`due_trigger_checked_at`、`exception_review_due_at`、`closure_archive_due_at` 与 profile 级 `approval_identity_source.json`、`window_trigger.json`、`signoff.json`、`exception_review.json`、`residual_risk_review.json`、`due_trigger_check.json`、`archive_target.json`、`closure_archive/index.json`、`closure_manifest.json` 收口成客户窗口实际执行与后续跟进留痕面。
- 阶段六的阶段 D 当前已完成 canonical security closure：真实 `pip-audit` / `trivy` 产物、`backup_restore_rehearsal_report`、`vulnerability_exception_report`、`vulnerability_exception_review_report`、`vulnerability_remediation_report`、`security_posture_report` 与 `security_release_preflight_report` 都已接入 release evidence；合并后的 main GitHub Actions run `28967203208` 已生成最新 security-preflight 证据：`security_posture_status=ready`、`security_release_preflight_status=passed`、Python findings `0`、container findings `0`、`accepted_finding_count=0`、`unresolved_finding_count=0`、active exceptions `0`、review-due exceptions `0`、expired exceptions `0`。
- `check_release_readiness.py`、`build_stable_promotion_checklist.py` 与 `build_industrial_promotion_checklist.py` 当前在 `vulnerability_exception_review` 场景下，已优先提示重建并复核独立 `vulnerability_exception_review_report.json`，然后再更新 `deployment/security/vulnerability_exceptions.input.json` 并重跑 security release preflight，不再只把用户丢回 aggregate preflight metrics。
- tracked canonical vulnerability exception 输入现已收口到 `deployment/security/vulnerability_exceptions.input.json`；`tools/collect_release_evidence.py` 与 `tools/run_security_release_preflight.py` 默认会先读取这份受管输入并生成结构化 `vulnerability_exception_report.json`，不再依赖 `test_env` 下的临时输入文件。
- `tools/build_vulnerability_exception_report.py`、`vulnerability_exception_report` contract 以及 exception matching 逻辑当前已落地。`tools/build_vulnerability_remediation_report.py` 与 `tools/build_security_posture_report.py` 现在都能消费经过审批、带到期时间的 exception report，并把 accepted / unresolved findings 分开写入结构化产物；dockerized Trivy fallback 的 `/scan/image.tar` 镜像标识问题也已修复，因此镜像级 exception 现在能稳定匹配 `deployment-web-panel-distributed` 的 canonical findings。

## 当前进度确认

- 发布主链已闭合：stable manifest、readiness、promotion checklist、distributed / Godot / ROS2 live evidence 都已完成并可追溯。
- 工业化执行计划已完成阶段 A、B、C，阶段 D 的本轮退出门禁也已闭合；当前焦点已从 security preflight 复绿切换到阶段 E/F 的工业预检链收口与扩展支持边界细化。
- 安全线当前已具备完整 contract：`sbom_artifact`、`vulnerability_scan_report`、`vulnerability_exception_report`、`vulnerability_exception_review_report`、`vulnerability_remediation_report`、`backup_restore_rehearsal_report`、`security_posture_report`、`security_release_preflight_report`。
- Zenoh router 交付镜像 `deployment-zenoh-router` 当前已复绿，`0 findings`。
- `deployment-web-panel-distributed` 的 canonical container findings 已通过 Web Panel Alpine 候选与 build-cache 清理降为 `0 findings`；当前 `accepted_finding_count=0`、`unresolved_finding_count=0`、`matched_exception_count=0`，旧 no-fix exceptions 已从受管输入退休。
- `security_posture_report` 当前为 `ready`，`security_release_preflight_report` 当前为 `passed`；最新 canonical preflight 摘要为 `Security posture is ready.`；最新 rc manifest 与 customer acceptance bundle 当前都已为 `ready`。
- 最新 rc manifest 当前已携带 `industrial_delivery_gate.status=ready`；当前 `deployment_package_status=ready`、`evidence_attested=true`、`backup_restore_verified=true`，并且容器扫描的 residual findings 已通过 `vulnerability_remediation_report.status=ready` / `security_posture_report.status=ready` 被视为工业交付闭合。
- 最新 canonical `extension_execution_actuals.json` 当前也已达到 `status=ready`，并显式携带 `approval_identity_source_type=customer_ticket_registry`、`approval_identity_reference=canonical-release/window-20260416/customer_operator`、`archive_target_binding_type=customer_archive_destination` 与 `due_trigger_binding_type=customer_due_trigger_schedule`，因此客户窗口的审批来源、归档目标和到期触发链已不再只停留在 repo 内路径。
- `python tools/run_release_rehearsal.py --version 2026.04.17-industrial-rehearsal --build-id release-rehearsal-industrial --output-root test_env/release_rehearsal_industrial` 当前已实跑通过；生成的 `release_rehearsal_report.json` 当前为 `status=passed`、`release_gate_status=ready`、`customer_delivery_surface.status=ready`、`industrial_delivery_gate.status=ready`。
- `tools/run_release_rehearsal.py` 当前还会直接写出 `security_release_preflight_report.json`、`release_manifest_industrial.json`、`industrial_release_readiness_report.json`、`industrial_promotion_checklist.json`、`customer_acceptance_bundle_industrial.json` 与独立 `industrial_delivery_rehearsal_report.json`；其中独立 industrial rehearsal report 当前为 `status=ready`、`stage_summary=6/6 passed`。最新本地重建的 `test_env/release_evidence/security/vulnerability_exception_review_report.json` 为 `status=passed` / `review_candidate_count=0`，`test_env/release_evidence/operations/external_mainline_input_checklist_report.json` 为 `status=passed` / `missing_input_count=0`，同时在 `release_rehearsal_report.json` 中结构化记录 `new_environment_install`、`smoke`、`live_evidence`、`upgrade`、`rollback`、`backup_restore` 六个工业交付演练阶段。
- `tools/build_external_mainline_execution_plan.py` 与 `tools/run_external_mainline_execution_plan.py` 当前已落地，用来统一剩余三条外部主线：`customer_external_bindings_closure`、`vulnerability_exception_replacement` 与 `industrial_delivery_live_evidence`。最新本地短路径重建的 `test_env/release_evidence/operations/external_mainline_execution_plan.json` 当前为 `status=ready`，摘要为 `completed=2`、`ready_to_run=1`、`waiting_external_input=0`、`blocked=0`：customer external bindings closure 与 vulnerability exception replacement 已完成，`industrial_delivery_live_evidence` 的 managed inputs 已齐备并进入 `ready_to_run`。managed runner 当前会自动重建 `vulnerability_exception_review_report.json`，并可通过 `--skip-customer-external-bindings-closure` 复用已通过的 closure evidence，避免重复触发 broad release evidence collect。最新 runner 还已支持 `--inputs-file`，仓内模板为 `deployment/external_mainline.inputs.example.json`，并新增了 `tools/build_external_mainline_inputs.py` 用来从已有 customer config / canonical industrial rehearsal baseline 自动刷新 `deployment/external_mainline.inputs.json`；现在 runner 默认就会接管这份 managed inputs file。若需要保留旧的“完全不使用 managed inputs”模式，可显式添加 `--skip-managed-inputs`。默认路径下它仍会忽略 `<confirmed-by>` / `<confirmation-ticket>` 这类 placeholder 确认值，避免误触发 customer closure；同一份 managed inputs 现在也已收口 `industrial_live_evidence.target_environment / access_method / install_entrypoint / upgrade_entrypoint / rollback_entrypoint / backup_restore_entrypoint / closure_archive_root`。P0 agentization 最小验证当前也已完成：external-mainline 的确定性编排已下沉到 `agi_walker/ops/external_mainline.py`，并通过 `agi_walker/core/api/release_ops_contracts.py` 显式暴露内部 request/result contract；现有 CLI 入口、artifact schema 与 stdout 信号保持不变，managed inputs 的 `bootstrapped/refreshed` 状态也已继续透传给 CLI。最新 `test_env/release_evidence/operations/external_mainline_input_checklist_report.json` 已是 `status=passed` / `missing_input_count=0`；真实 industrial live evidence 仍需客户环境执行与现场留痕，才能从 `ready_to_run` 变为 `completed`。最新 contract 还已把 external-mainline plan 的 `source_report_path`、`artifact_paths` 与缺失 summary 统一收成 project-relative surface，不再把 `pytest_tmp` / `runtime_tmp` 这类临时绝对路径泄露进 canonical closeout evidence。
- `tools/check_release_readiness.py` 与 `tools/build_stable_promotion_checklist.py` 当前也已开始直接消费 `external_mainline_execution_plan`：stable readiness stdout 会显式打印 `stable_external_mainline_execution_plan=<status>/<completed>/<ready_to_run>/<waiting_external_input>/<blocked>`、`stable_external_mainline_input_checklist=<status>/<missing>/<waiting>/<ready>/<completed>` 与 `stable_vulnerability_exception_review=<status>/<candidate_count>`，并把 `vulnerability_exception_review` / `external_mainline_execution_plan` / `external_mainline_input_checklist` 一起收成 `release_readiness_report.json` 的顶层字段与 summary；stable promotion checklist 则会把 plan 与 checklist 一起收成顶层字段、summary 和 non-blocking step，不再只在 `next_actions` 里旁路提示 runner。
- P1 agentization 当前又往前推进了一步：`tools/build_stable_promotion_checklist.py` 与 `tools/build_industrial_promotion_checklist.py` 已不再通过 subprocess 调 `check_*_readiness.py`，而是直接复用 `agi_walker/ops/readiness.py` 的确定性 service；现有 checklist schema、stdout 与下游 smoke / rehearsal 消费面保持不变。
- P1 agentization 当前已继续推进到 rehearsal：`tools/run_release_rehearsal.py` 现在是薄封装 CLI，稳定发布演练编排已下沉到 `agi_walker/ops/rehearsal.py`，并通过 `ReleaseRehearsalRequest/Result` 暴露内部 contract；现有 report schema、stdout 与 industrial rehearsal 下游消费面保持不变。
- P1 agentization 当前已继续推进到 worktree blocker：`tools/run_worktree_release_blocker.py` 现在是薄封装 CLI，dirty worktree 收口编排已下沉到 `agi_walker/ops/worktree.py`，并通过 `WorktreeReleaseBlockerRequest/Result` 暴露内部 contract；现有 report schema、stdout 与 readiness / checklist / smoke 消费面保持不变。
- P1 agentization 当前已继续推进到 customer acceptance bundle：`tools/build_customer_acceptance_bundle.py` 现在是薄封装 CLI，客户验收包编排已下沉到 `agi_walker/ops/acceptance.py`，并通过 `CustomerAcceptanceBundleRequest/Result` 暴露内部 contract；现有 bundle schema、stdout 与 rehearsal / smoke / downstream acceptance 消费面保持不变。
- P1 agentization 当前已继续推进到 industrial delivery rehearsal report：`tools/build_industrial_delivery_rehearsal_report.py` 现在是薄封装 CLI，独立 industrial 交付演练报告编排已下沉到 `agi_walker/ops/industrial_delivery.py`，并通过 `IndustrialDeliveryRehearsalReportRequest/Result` 暴露内部 contract；现有 report schema、stdout 与 rehearsal / smoke / acceptance 下游消费面保持不变。
- P1 agentization 当前已继续推进到 promotion checklist：`tools/build_stable_promotion_checklist.py` 与 `tools/build_industrial_promotion_checklist.py` 现在都已把主 orchestration 下沉到 `agi_walker/ops/promotion.py`，并通过 `StablePromotionChecklistRequest/Result` 与 `IndustrialPromotionChecklistRequest/Result` 暴露内部 contract；现有 checklist schema、stdout 与 readiness / smoke 下游消费面保持不变。
- P2 control-plane 当前已继续推进到显式 policy enforcement：统一 `tools/run_release_ops.py` 与 `agi_walker/ops/release_ops.py` 现在除了把 `release_readiness`、`industrial_release_readiness`、`stable_promotion_checklist`、`industrial_promotion_checklist`、`external_mainline_execution`、`release_rehearsal`、`worktree_release_blocker`、`customer_acceptance_bundle` 与 `industrial_delivery_rehearsal_report` 收口到单一 action dispatch，还新增了 `--list-policy-profiles` 与显式 `policy_profile` 校验。当前 control plane 默认使用 `local_safe_refresh` profile，因此 `external_mainline_execution=requires_attestation` 在未显式传入 `--policy-profile requires_attestation` 时会 fail closed；其余已接线 action 继续在默认 profile 下可用。现有 CLI / artifact schema 仍保持不变，这条入口当前已开始具备真正的 policy boundary，而不再只暴露 metadata。
- P3 最小切口当前已开始落地：`release_ops` control plane 现在会把轻量 delivery session 与 JSONL 事件流一起收口到统一 contract。`ReleaseOpRequest` 当前已支持 `session.engagement_id / window_id / change_ticket / channel` 与 `event_stream_file`；对应 `ReleaseOpResult` 会回写标准化 session、`event_stream_path` 与 `event_count`。`tools/run_release_ops.py` 已新增 `--engagement-id`、`--window-id`、`--change-ticket`、`--channel` 与 `--event-stream-file`，并会在 stdout / result JSON 中显式打印 session 上下文和事件流路径。当前事件类型已固定为 `action_started`、`action_completed`、`artifact_written`、`policy_denied` 与 `action_failed`，因此统一 control plane 已开始具备最小可追踪执行面，而不再只返回最终 artifact 摘要。
- P3 当前还已继续把这组 control-plane metadata 接进 `external_mainline_execution_plan` 及其 downstream surface：当 `release_ops` 以 `external_mainline_execution` 运行时，`control_plane_session` 与 `control_plane_event_stream` 会被回写到 plan artifact 本身；rehearsal seed 也会附带 synthetic control-plane session / event metadata，因此 `customer_acceptance_bundle`、`release_rehearsal_report`、`industrial_customer_acceptance_bundle` 与 `industrial_delivery_rehearsal_report` 当前都能继续携带这组 session / event-stream 摘要，而不是只暴露 external-mainline 的 step 计数。
- P3 当前已继续把同一组 control-plane metadata 接进 `external_mainline_input_checklist` 与 `worktree_release_blocker`：`release_ops` 以 `external_mainline_execution` 运行时，会把 `control_plane_session` / `control_plane_event_stream` 一并回写到 `external_mainline_input_checklist_report.json`，并继续流入 `customer_acceptance_bundle`、`industrial_customer_acceptance_bundle` 与 `industrial_delivery_rehearsal_report`；以 `worktree_release_blocker` 运行时，则会把同一组摘要回写到 `worktree_release_blocker_report.json`。`check_release_readiness.py` / `check_industrial_release_readiness.py` 的 checklist preview 现在也会继续保留 checklist / worktree report 上的这组 session / event-stream 摘要，而不是只暴露状态计数。
- P3 当前还已把 control-plane metadata 提升到 readiness / promotion 顶层 artifact：当 `release_ops` 运行 `release_readiness`、`industrial_release_readiness`、`stable_promotion_checklist` 或 `industrial_promotion_checklist` 时，对应 report 文件本身现在也会直接带 `control_plane_session` / `control_plane_event_stream`，而不再只在 `external_mainline_*` 或 `worktree_release_blocker` 这些 nested component 里可见。这样 control-plane 的 delivery-session 上下文已经开始进入 stable / industrial 顶层 gate artifact，而不只是 preview/component 层。
- P3 当前又已把这组 control-plane metadata 继续提升到 rehearsal aggregate surface：`release_rehearsal_report.json` 与 `industrial_delivery_rehearsal_report.json` 现在都会在顶层直接带 `control_plane_session` / `control_plane_event_stream`，来源优先取自 `external_mainline_execution_plan`，回退到 `external_mainline_input_checklist`；`tools/run_release_rehearsal.py` 与 `tools/build_industrial_delivery_rehearsal_report.py` 也会在 stdout 显式打印 `release_rehearsal_control_plane_events=...` / `industrial_delivery_control_plane_events=...`，因此这条 delivery-session 摘要不再只停留在 component 明细里。
- P3 当前又已把同一组 control-plane metadata 提升到 `customer_acceptance_bundle` 顶层：bundle 重建时现在会从 `external_mainline_execution_plan` / `external_mainline_input_checklist` 聚合 `control_plane_session` 与 `control_plane_event_stream`，并把 `control_plane_events=<count>` 直接写进 bundle summary；`tools/build_customer_acceptance_bundle.py` 会显式打印 `customer_acceptance_bundle_control_plane_events=...`，而 smoke artifact 摘要现在也会对 bundle / readiness / rehearsal / industrial delivery 顶层 payload 统一拼出 `control_plane_events=...`。
- P3 当前已开始把 `release_ops` 自身的 normalized result 收成 canonical evidence：当 `tools/run_release_ops.py` 显式传入 `--evidence-report-file` 时，control plane 会额外写出一份标准 `release_evidence_report`，`evidence_name=release_ops_execution`，并把 `action`、`policy_level`、`policy_profile`、`request_type`、`output_path`、`event_count` 以及 `control_plane_session` / `control_plane_event_stream` 一起固化成 action-level wrapper，避免 control-plane 执行结果只存在于临时 `result-file` 或 stdout 中。最新 downstream surface 也已开始直接消费这份 wrapper：`customer_acceptance_bundle` 会把 `release_ops_execution` 挂成 acceptance report 和顶层字段，而 `release_rehearsal_report.json` 与 `industrial_delivery_rehearsal_report.json` 现在都会继续透传 `release_ops_execution.status/event_count`。
- P3 当前已继续把这份 wrapper 接进 canonical evidence collector：`tools/collect_release_evidence.py` 现在支持 `--release-ops-execution-report-source`，会把已有 `release_ops_execution_report.json` 复制进 `test_env/release_evidence/operations/`，这样 control-plane 的 action-level wrapper 不再只能通过 CLI 临时路径消费，而可以进入 canonical release evidence 根目录；collector 仍不会主动执行 `run_release_ops.py`，因此现有 `policy_profile` / `requires_attestation` 边界保持不变。
- P3 当前又已把这份 wrapper 接进 manifest-facing release surface：`build_customer_delivery_surface()` 现在会优先读取 canonical `test_env/release_evidence/operations/release_ops_execution_report.json`，并把 `release_ops_execution.status/event_count` 以及聚合后的 `control_plane_session` / `control_plane_event_stream` 挂进 `release_manifest.customer_delivery_surface`；`tools/build_release_artifact.py` 的 stdout 也新增了 `customer_delivery_release_ops_execution=...`，因此 canonical manifest 已能直接反映 control-plane wrapper，而不必等到 acceptance / rehearsal 面才可见。
- P3 当前又已把同一份 wrapper 继续提升到 `release_manifest.industrial_delivery_gate`：industrial gate 现在会复用 `customer_delivery_surface` 中已归一化的 `release_ops_execution` 与 control-plane 摘要，并把 `release_ops_execution.status/event_count`、`control_plane_session`、`control_plane_event_stream` 一起挂进 gate payload；`tools/build_release_artifact.py` 的 stdout 也新增了 `industrial_delivery_release_ops_execution=...`，因此 manifest 的 customer / industrial 两侧现在都能直接消费 canonical control-plane wrapper。
- P3 当前又已把同一份 wrapper 继续提升到 `release_manifest` 顶层：manifest 现在会直接附带 `release_ops_execution.status/event_count`，并把聚合后的 `control_plane_session` / `control_plane_event_stream` 提升成顶层字段；`tools/build_release_artifact.py` 的 stdout 也新增了 `release_manifest_release_ops_execution=...` 与 `release_manifest_control_plane_events=...`，因此 control-plane wrapper 不再只停留在 customer / industrial 子面。
- P3 当前又已把这组 manifest 顶层 metadata 收成统一 `control_plane_surface`：`release_manifest` 现在会在保留既有 `release_ops_execution` 与聚合 session/event 字段的同时，额外挂出 `control_plane_surface.status/event_count`、嵌套 `release_ops_execution` 以及同一组 `control_plane_session` / `control_plane_event_stream`；`tools/build_release_artifact.py` 也新增了 `release_manifest_control_plane_surface=...`，因此后续 gateway / MCP 不必再从 manifest 多个子面回扫 control-plane 证据。
- P3 当前还已把这组统一 surface 暴露到 MCP：`agi_walker.mcp.server` 新增只读工具 `release_control_plane_surface_get`，优先读取 `release_manifest.control_plane_surface`，缺失时回退到 canonical `test_env/release_evidence/operations/release_ops_execution_report.json`；这样 control-plane canonical surface 现在已经从内部 release artifact 扩展到正式 gateway/MCP 入口，而不需要开放高风险 `release_ops` 执行。
- P3 当前又已把 `release_ops` 的 discovery 面暴露到 MCP：`agi_walker.mcp.server` 新增只读工具 `release_ops_catalog_get`，直接返回 action catalog 与 policy profiles，因此 MCP 客户端现在不仅能读取 canonical control-plane surface，也能先发现当前允许的 release/control-plane 动作及其 `policy_level`，而不需要先调用执行入口探测。
- P3 当前还已把这些只读面再收成单入口 MCP 工具 `release_control_plane_index_get`：它现在会一次返回 canonical `control_plane_surface`、`release_ops` catalog 与 `request_templates`，因此 MCP 客户端现在可以先拿到总览 index，再决定是否读取单独的 surface、catalog 或 request defaults，而不需要自己在多个 tool 之间做第一次聚合。
- P3 当前也已把同一组只读聚合面接到了 Web/Portal：`web_panel.core_api` 新增 `/api/release/control-plane`，并且 `/api/system/status` 现在会附带 `release_control_plane.route/status/actions_count/policy_profiles_count/request_templates_count` 摘要。因此 Portal/HTTP 客户端现在也能直接读取 canonical control-plane 总览，而不必只能通过 MCP。
- P3 当前又已把这组 Web/Portal 只读面细化成独立路由：除了 `/api/release/control-plane` 总览之外，Portal 现在还可直接读取 `/api/release/control-plane/catalog` 与 `/api/release/control-plane/request-templates`；后者支持 `action` 过滤，因此 HTTP 客户端也可以按 action 生成 `request-file` 草稿，而不必每次都先拉全量 index。
- P3 当前又补齐了与 MCP 对称的 Web `surface` 入口：Portal 现在也可直接读取 `/api/release/control-plane/surface`，因此 canonical `control_plane_surface` 在 MCP 与 HTTP 两条只读面上都已有单独路由，不再只能通过总览 index 或 system status 摘要间接消费。
- P3 当前又把剩余未闭合项收成了统一 `release_closeout` surface：MCP 新增只读工具 `release_closeout_get`，Portal 新增 `/api/release/closeout`，两者都会把 `external_mainline_execution_plan`、`vulnerability_exception_review` 与 `worktree_release_blocker` 汇总成单一 payload，并继续附带 component 级 `action_items` / `command`；同时 `/api/system/status.release_control_plane` 现在会附带 `release_closeout_status` 与 component 计数，因此 Portal 首页也能先看到当前是否仍有 external-mainline、安全 residual-risk 或 worktree closeout 待处理，而不需要分别展开三份 artifact。
- P3 当前又把这组 closeout 摘要真正接到了首页 dashboard：`web_panel/static/index.html` 现在会直接消费 `/api/system/status.release_closeout`，显示 closeout 状态、`blocked / waiting / ready / missing` 计数以及 `/api/release/closeout` route，因此 Portal 首页不只知道“有待收口”，还能直接看到剩余 closeout 压力分布。
- P3 当前又把 `release_closeout` summary 继续压到了首页可执行提示：`build_release_closeout_summary()` 现在会额外挂出 `action_items_count` 与 `top_action_items`，而首页 dashboard 会直接显示“下一步组件”和建议命令，因此不打开 `/api/release/closeout` 详情页也能看到当前应该先跑哪条 closeout 动作。
- P3 当前又把 `release_closeout` 从首页摘要扩成了独立 Portal 详情页：`web_panel/static/release-closeout.html` 现在会直接消费 `/api/release/closeout`，展开 external-mainline、安全 residual-risk 与 worktree blocker 三类组件的 `action_items`、`blocking_inputs` 和建议命令；首页的快速操作区也新增了“Release 收口页”入口，因此当前 closeout 已从只读摘要升级为 Portal 内可直接展开的详情面。
- P3 当前又把这条按 action 的控制面继续收成了正式后端 contract：Portal 与 MCP 现在都可读取 `/api/release/control-plane/action?action=...` / `release_control_plane_action_get`，它会把单个 action 的 catalog entry 与 request template 聚合成一份只读 payload；`web_panel/static/release-control-plane.html` 也已切到这条新入口，并会把选中的 action 同步到 URL 查询参数，方便直接分享单 action schema 深链。
- P3 当前又把这组单 action 深链压回到了首页摘要：`/api/system/status.release_control_plane` 现在会额外挂出 `next_action`、默认 policy 和 `/static/release-control-plane.html?action=...` 深链；首页 dashboard 也新增了“优先 action / 深链”入口，因此当前 control-plane 主线已经从后端 contract、MCP、Portal 详情页一路收到了首页可点击跳转。
- P3 当前又把 `release_closeout` 做成了同样的首页深链：`build_release_closeout_summary()` 现在会额外挂出 `next_component` 与 `/static/release-closeout.html?component=...`，而首页 dashboard 也把“下一步组件”改成了可点击入口；`web_panel/static/release-closeout.html` 现在会读取 `component` 查询参数并按组件聚焦显示，因此当前 closeout 主线也已经从 summary 收到了可分享的 Portal 深链。
- P3 当前又把这条 closeout 深链继续收成了正式后端 contract：Portal 与 MCP 现在都可读取 `/api/release/closeout/component?component=...` / `release_closeout_component_get`，它会把单个 closeout component 的 payload 与对应 action item 聚合成一份只读结果；`web_panel/static/release-closeout.html` 也已切到这条新入口，因此当前 closeout 详情页不再依赖前端本地过滤全量 payload。
- P3 当前又把 Portal 深链/JSON 路径本身收成了 canonical payload 字段：`release_closeout.action_items[*]`、`release_closeout_component_get`、`release_ops_request_templates_get` 与 `release_control_plane_action_get` 现在都会直接附带 `component_route` / `component_api_route`、`portal_route` / `action_route` / `request_template_route`；`web_panel/static/release-closeout.html` 与 `web_panel/static/release-control-plane.html` 也已切到消费这些 route 字段，不再在前端本地拼接 URL。
- P3 当前又把 request-template 继续收成了独立 request-file scaffold：Portal 与 MCP 现在都可读取 `/api/release/control-plane/request-file?action=...` / `release_control_plane_request_file_get`，它会返回单 action 的 `request_file_route`、`request_file_name`、`request_file` 和 `request_file_pretty_json`；`web_panel/static/release-control-plane.html` 也已切到这条新入口，并新增“查看 request-file / 下载草稿 / 复制 JSON”按钮，因此当前 control-plane 主线已经从只读 schema 走到了可直接导出 request-file 草稿。
- P3 当前又把这条 request-file scaffold 提升到了首页摘要：`/api/system/status.release_control_plane` 现在会额外挂出 `next_action_request_file_route` 与 `next_action_request_file_name`，而首页 dashboard 也新增了直接可点击的 request-file 草稿入口，因此当前 Portal 首页已经能直接跳到“下一步 action”的 request-file JSON。
- P3 当前又把这组“下一步 action / request-file”信息提升到了 control-plane 总览入口本身：`/api/release/control-plane` 与 `release_control_plane_index_get` 现在都会直接带 `next_action`、`next_action_route`、`next_action_request_route`、`next_action_request_file_route` 与 `next_action_request_file_name`，而 `web_panel/static/release-control-plane.html` 在未选择 action 时也会默认绑定到这组推荐 request-file scaffold，不再依赖前端本地选出默认 action。
- P3 当前又把 request-file 导出收成了 canonical download route：`request_templates`、单 action payload、`/api/release/control-plane` 总览和首页 summary 现在都会继续带 `request_file_download_route` / `next_action_request_file_download_route`；同时 `/api/release/control-plane/request-file?action=...&download=1` 会直接返回带文件名的 JSON 下载响应，因此 Portal 的“下载草稿”按钮也已经切到消费后端 canonical 下载路径，而不再用前端 Blob 本地拼装。
- P3 当前又把“推荐下一步 action”收成了独立 canonical payload：Portal HTTP 新增 `/api/release/control-plane/next`，MCP 新增 `release_control_plane_next_get`；它会把推荐 action 的 `action_definition`、`request_template` 与 `request-file scaffold` 收成单一只读入口，而 `web_panel/static/release-control-plane.html` 在未选择 action 时也已切到优先消费这条 route，不再继续从总览 payload 本地拆推荐入口。
- P3 当前又把 `release_closeout` 做到了与 control-plane `next` 对称：Portal HTTP 新增 `/api/release/closeout/next`，MCP 新增 `release_closeout_next_get`；它会把推荐下一步 component 的 `action_item`、组件 payload 与建议命令收成单一只读入口，而 `web_panel/static/release-closeout.html` 在未选择 component 时也已切到优先消费这条 route，不再继续从 closeout 总览 payload 本地挑第一项。
- P3 当前又把 `closeout/next` 提升回 canonical summary：`release_closeout` payload 与 `/api/system/status.release_closeout` 现在都会直接带 `next_component_next_route=/api/release/closeout/next`，因此首页已可直接暴露“下一步 JSON”快捷入口，而不必继续本地拼装 closeout next 路由。
- P3 当前又把剩余 external closeout 输入收成了独立执行计划：Portal HTTP 新增 `/api/release/closeout/plan`，MCP 新增 `release_closeout_plan_get`，Portal 新增 `/static/release-closeout-plan.html`。这条 payload 会把 customer bindings、安全 replacement evidence、industrial live evidence、canonical rebuild 和 acceptance review 收成 5 个 stage，并直接附带输入文件、建议命令和完成标准，因此“剩余外部输入计划”现在已经从文字说明升级成可消费 contract。
- P3 当前又把这份 staged closeout plan 收成了单阶段与下一阶段入口：Portal HTTP 新增 `/api/release/closeout/plan/stage` 与 `/api/release/closeout/plan/next`，MCP 新增 `release_closeout_plan_stage_get` 与 `release_closeout_plan_next_get`；`web_panel/static/release-closeout-plan.html` 现在会在未指定 `?stage=` 时优先消费推荐下一阶段，在带 `?stage=` 时直接读取单阶段 payload，而不再只靠前端本地过滤全量 plan。
- P3 当前又把这组 Portal 只读摘要接到了首页 dashboard：`web_panel/static/index.html` 现在会直接消费 `/api/system/status.release_control_plane`，显示 control-plane 状态、动作/模板计数以及 policy profiles / route，因此不打开单独 Portal 子页也能先确认当前只读 control-plane 面是否已就绪。
- P3 当前又把统一 `release_next` 做成了独立 Portal 详情页：`/api/release/next` 现在会显式带 `portal_route=/static/release-next.html`，而 `web_panel/static/release-next.html` 会同时展开 unified primary next、`control_plane_next` 与 `release_closeout_next`；首页 dashboard 里的 `Release Next` 深链也已优先跳到这页，而不是继续把用户直接丢到 JSON。
- P3 当前又把 `release_next` 收成了和 control-plane / closeout 对称的单主推荐入口：HTTP 新增 `/api/release/next/primary`，MCP 新增 `release_next_primary_get`；它会把统一 `release_next` 里的主推荐项单独收成 canonical payload，而 `web_panel/static/release-next.html` 也已改成直接消费这条 route 渲染 primary 区块，不再继续从 `/api/release/next` 本地拆主推荐项。
- P3 当前又把这条主推荐入口继续收成了 canonical follow-up contract：`release_next_primary` 现在会统一返回 `primary_follow_up_kind / label / route / text / download_route`，把 `request-file`、`建议命令` 与 `下一步 JSON` 三类 follow-up 收口成一套字段；`web_panel/static/release-next.html` 也已切到直接消费这组字段渲染“下一步执行 / 下载草稿”，而不是继续在前端本地分支判断。
- P3 当前又把这组 follow-up 字段提升成了独立只读入口：HTTP 新增 `/api/release/next/follow-up`，MCP 新增 `release_next_follow_up_get`；它会把统一主推荐项的执行/导出动作单独收成 canonical payload，而 `web_panel/static/release-next.html` 也已改成直接消费这条 route 渲染“下一步执行 / Follow-up JSON / 下载草稿”，不再只依赖 `release_next_primary` 内嵌字段。
- P3 当前又把 `release_next` 的 canonical follow-up 深链接回了首页 dashboard：`web_panel/static/index.html` 现在会直接消费 `/api/system/status.release_next.primary_follow_up_route` 与 `primary_follow_up_payload_route`，新增“下一步执行 / Follow-up JSON”两个快捷入口，因此不打开 `release-next.html` 也能直接跳到当前主推荐项的执行动作或 follow-up payload。
- P3 当前又把 `release_next` 的主推荐详情也收回了首页 dashboard：`web_panel/static/index.html` 现在会继续消费 `/api/system/status.release_next.primary_portal_route` 与 `primary_api_route`，新增“主推荐详情 / 主推荐 JSON”两个快捷入口，因此首页已能完整覆盖 unified release-next 的总览、主推荐详情和 follow-up 深链，而不必先进入 `release-next.html`。
- P3 当前又把 `release_next` 的 canonical 主推荐 payload route 显式化成了 `primary_payload_route=/api/release/next/primary`，并让首页“主推荐 JSON”改为优先消费这条字段，而不再借用具体 action/component 的 `primary_api_route`；这样 unified release-next 的总览、主推荐详情和主推荐 payload 现在都有各自稳定的 canonical 深链。
- P3 当前又把 `release_next` 的 `primary_follow_up_download_route` 接回了首页 dashboard：`web_panel/static/index.html` 现在会直接暴露“下一步草稿”快捷入口，因此当统一主推荐项的 follow-up 是 request-file 路径时，首页也能直接下载草稿，而不必先进入 `release-next.html`。
- P3 当前又把 `release_next` 真正收成了单入口 canonical payload：`build_release_next_payload()` 现在会直接内嵌 `primary_payload` 与 `follow_up_payload`，而 `web_panel/static/release-next.html` 也已改成只消费 `/api/release/next`，不再额外再拉 `/api/release/next/primary` 与 `/api/release/next/follow-up` 做前端本地拼装。
- P3 当前又把 unified `release_next request-file` 做成了独立只读入口：HTTP 新增 `/api/release/next/request-file`，MCP 新增 `release_next_request_file_get`；`build_release_next_payload()` 现在也会继续内嵌 `request_file_payload`，而 `web_panel/static/release-next.html` 在当前主推荐项为 control-plane request-file 时会优先走这条统一导出路由，而不再直接跳回 control-plane request-file route。
- P3 当前又把首页 `release_next` 摘要切到了 unified request-file contract：`build_release_next_summary()` 现在会稳定暴露 `request_file_status / request_file_payload_route / request_file_download_route`，而 `web_panel/static/index.html` 在当前主推荐项为 control-plane request-file 时也会优先跳 `/api/release/next/request-file`，不再继续回退到 `primary_follow_up_*` 深链。
- P3 当前又把 unified `release_next request-file` 做成了显式 Portal 入口：首页 dashboard 现在新增独立 `request-file JSON` 快捷入口，而 `web_panel/static/release-next.html` 也新增了显式 `request-file JSON` 按钮；两者都直接消费 unified `request_file_route / request_file_name`，不再把 request-file 只隐含在 follow-up 区块里。
- P3 当前还把 `release_ops` 的请求面显式暴露到了 MCP：`release_ops_request_templates_get` 现在会返回每个 action 的只读 request template defaults、`required_fields`、`optional_fields` 与 `default_policy_profile`。因此 MCP 客户端现在不只知道有哪些 action，还能直接生成 `request-file` 草稿，而不需要自己猜 JSON 结构或触发执行入口。

## 当前可用入口

- CLI：`python -m agi_walker.cli`
- MCP：`agi-walker-mcp`
- MCP 模块入口：`python -m agi_walker.mcp.server`
- Web Panel：`python -m web_panel.server`
- Smoke：`python tests/run_smoke_tests.py`
- Release Artifact：`python tools/build_release_artifact.py --version ... --channel ... --build-id ... --release-summary ...`
- Customer Acceptance Bundle：`python tools/build_customer_acceptance_bundle.py --manifest ...`
- Release Evidence：`python tools/collect_release_evidence.py`
- SBOM Artifact：`python tools/build_sbom_artifact.py --output test_env/security/sbom.json`
- Vulnerability Scan Report：`python tools/write_vulnerability_scan_report.py --scan-name ... --target ... --status passed --scanner ... --summary ... --command ... --output test_env/security/...`
- Vulnerability Exception Input：受管源文件位于 `deployment/security/vulnerability_exceptions.input.json`
- Backup Restore Rehearsal：`python tools/run_backup_restore_rehearsal.py --output-root test_env/security/backup_restore_rehearsal --report-file test_env/security/backup_restore_rehearsal_report.json`
- Security Posture Report：`python tools/build_security_posture_report.py --output test_env/security/security_posture_report.json`
- Security Release Preflight：`python tools/run_security_release_preflight.py --output-root test_env/release_evidence`
- Vulnerability Remediation Report：`python tools/build_vulnerability_remediation_report.py --python-vuln-report test_env/release_evidence/security/python_vuln_scan_report.json --container-vuln-report test_env/release_evidence/security/container_vuln_scan_report.json --output test_env/release_evidence/security/vulnerability_remediation_report.json`
- Pytest Evidence Writer：`python tools/write_pytest_evidence_report.py --name ... --output ... -- tests/...`
- Release Readiness：`python tools/check_release_readiness.py`
- Industrial Release Readiness：`python tools/check_industrial_release_readiness.py`
- Worktree Release Blocker Runner：`python tools/run_worktree_release_blocker.py`
- Worktree Cleanup Report：`python tools/build_worktree_cleanup_report.py`
- Tracked Artifact Review Report：`python tools/build_tracked_artifact_review_report.py`
- Clean Checkout Smoke：`python tools/run_clean_checkout_smoke.py --output-root test_env/clean_checkout_smoke_real`
- Stable Promotion Checklist：`python tools/build_stable_promotion_checklist.py`
- Industrial Promotion Checklist：`python tools/build_industrial_promotion_checklist.py`
- Extension Execution Actuals：`python tools/build_extension_execution_actuals.py --output test_env/release_evidence/operations/extension_execution_actuals.json --schedule-artifact test_env/release_evidence/operations/extension_execution_schedule.json`
- External Mainline Execution Plan：`python tools/build_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json`
- External Mainline Input Checklist：`python tools/build_external_mainline_input_checklist.py --output test_env/release_evidence/operations/external_mainline_input_checklist_report.json`
- External Mainline Inputs Builder：`python tools/build_external_mainline_inputs.py --output deployment/external_mainline.inputs.json`
- External Mainline Runner：`python tools/run_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json`
- External Mainline Inputs Template：`deployment/external_mainline.inputs.example.json`
- 已有 stable manifest 反哺预检：`python tools/check_release_readiness.py --approval-manifest test_env/release/release_manifest_stable.json`
- 已有 stable manifest 反哺 checklist：`python tools/build_stable_promotion_checklist.py --approval-manifest test_env/release/release_manifest_stable.json`
- 已有 industrial manifest 反哺预检：`python tools/check_industrial_release_readiness.py --approval-manifest test_env/release/release_manifest_industrial.json`
- 已有 industrial manifest 反哺 checklist：`python tools/build_industrial_promotion_checklist.py --approval-manifest test_env/release/release_manifest_industrial.json`

## 已验证项

以下命令在当前工作区已执行并通过：

- `python -m agi_walker.cli skills list`
- `python -m agi_walker.cli workflows list`
- `python -m agi_walker.cli workflows run robot_creation_pipeline --mock --output-root test_env/doc_check_workflow --resume`
- `python -m pytest tests/test_mcp_tools.py tests/test_mcp_server.py -q`
- `python tests/run_smoke_tests.py --output-root test_env/smoke_phase5_acceptance`
- `python tools/build_release_artifact.py --version 2026.04.12 --channel stable --build-id build-20260413-stable --approval-status approved --approved-by "AGI-Walker Bot" --approved-at 2026-04-13T09:01:43.0889172+01:00 --approval-notes "stable signoff" --output test_env/release/release_manifest_stable.json`
- `python tools/check_release_readiness.py --approval-manifest test_env/release/release_manifest_stable.json --output-root test_env/release_readiness_ready`
- `python tools/build_worktree_cleanup_report.py`
- `python tools/build_tracked_artifact_review_report.py`
- `python tools/run_clean_checkout_smoke.py --output-root test_env/clean_checkout_smoke_real`
- `python tools/build_stable_promotion_checklist.py --approval-manifest test_env/release/release_manifest_stable.json --output-root test_env/stable_promotion_ready`
- `python tools/run_security_release_preflight.py --output-root test_env/release_rehearsal_industrial/test_env/release_evidence --skip-collect --security-posture-report test_env/release_rehearsal_industrial/test_env/release_evidence/security/security_posture_report.json --report-file test_env/release_rehearsal_industrial/test_env/release_evidence/security_release_preflight_report.json`
- `python tools/check_industrial_release_readiness.py --project-root test_env/release_rehearsal_industrial --source-root test_env/release_rehearsal_industrial/git_source_1a48b7a2 --current-version 2026.04.17-industrial-rehearsal --approval-manifest test_env/release_rehearsal_industrial/release_manifest_industrial.json --output-root test_env/release_rehearsal_industrial/test_env/industrial_release_readiness_ready`
- `python tools/build_industrial_promotion_checklist.py --project-root test_env/release_rehearsal_industrial --source-root test_env/release_rehearsal_industrial/git_source_1a48b7a2 --current-version 2026.04.17-industrial-rehearsal --approval-manifest test_env/release_rehearsal_industrial/release_manifest_industrial.json --output-root test_env/release_rehearsal_industrial/test_env/industrial_promotion_ready`
- `python tools/build_customer_acceptance_bundle.py --manifest release_manifest_industrial.json --project-root test_env/release_rehearsal_industrial --output test_env/release_rehearsal_industrial/test_env/release/customer_acceptance_bundle_industrial.json`
- `tools/run_worktree_release_blocker.py` 当前已落地，作为 `clean_worktree` 收口的 primary runner：它会先生成 `worktree_cleanup_report.json`，在存在 tracked runtime/generated 候选时继续生成 `tracked_artifact_review_report.json`，最后把这两步收成统一的 `worktree_release_blocker_report.json`。`check_release_readiness.py`、`build_stable_promotion_checklist.py` 与 `build_industrial_promotion_checklist.py` 现在都会优先暴露这条 runner，并把 `worktree_release_blocker` 收成顶层机器字段、summary 和 stdout；当前对应 stdout 分别为 `stable_worktree_release_blocker=...` 与 `industrial_worktree_release_blocker=...`。底层 `tools/build_worktree_cleanup_report.py` 仍继续补出 `tracked_review_candidate_count`、`tracked_review_candidate_paths`、`tracked_review_report_path` 与 `tracked_review_command`，供进一步审查 tracked artifact 时继续消费。
- `python tools/run_release_rehearsal.py --version 2026.04.12-rehearsal --build-id release-rehearsal`
- `python tests/run_distributed_smoke.py --build --stop-after --report-file test_env/distributed_smoke/distributed_smoke_report.json`
- `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv`
- `docker run --rm -v "<repo>:/workspace" -w /workspace ros:humble-ros-base bash -lc 'source /opt/ros/humble/setup.bash && python3 -m pip install --no-cache-dir --upgrade "pytest>=7.4,<8" pytest-asyncio pytest-cov pytest-mock anyio numpy pyyaml pydantic && export PYTHONPATH=/workspace:$PYTHONPATH && export AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1 && python3 -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv'`
- `python -m pytest tests\test_workflow_contracts.py tests\test_workflow_orchestrator.py -q`
- `python -m pytest tests\test_web_godot_session_bridge.py tests\test_web_panel_integration_routes.py -q`
- `python -m pytest tests\test_distributed_smoke_runner.py tests\test_web_panel_aux_apis.py::test_core_panel_routes_smoke tests\test_web_panel_aux_apis.py::test_distributed_monitor_prunes_stale_actors tests\test_active_path_references.py::test_distributed_runtime_uses_current_package_entrypoints -q`
- `python -m pytest -m "not live" -q`
- `python -m pytest tests\ -m "not integration and not hardware" -v --tb=short --cov=agi_walker --cov-report=xml --cov-report=term-missing`

已验证结果：

- Skills 列表可正常读取，当前检出包含 `robot-modeling`、`parameter-optimizer`、`urdf-generator`、`model-distiller`。
- Workflows 列表可正常读取，当前至少包含 `robot_creation_pipeline` 和 `simulation_ready_robot`。
- `robot_creation_pipeline` 的 mock 执行可完成，输出可写入 `test_env/doc_check_workflow`。
- MCP 相关测试当前为 `16 passed`。
- canonical stable manifest 位于 `test_env/release/release_manifest_stable.json`，当前 `release_gate_status=ready`，其中 distributed、Godot headless 和 ROS2 bridge live evidence 都已为 `passed`，`blocked_evidence=0`、`opt_in_evidence=0`，`distributed_runtime` domain 已提升为 `ready`。
- canonical stable readiness 报告位于 `test_env/release_readiness_ready/release_readiness_report.json`，当前 stable preview 为 `ready`，不再是早期 dirty worktree 阶段的 `blocked`；同一路径的 stable readiness codepath 现在也会把 `vulnerability_exception_review.status`、`external_mainline_execution_plan.status` 与 `worktree_release_blocker.status` 收成 report 顶层字段，并在 summary 中直接写出 `exception_review=...` / `external_mainline=...` / `worktree=...`。
- required release evidence 现在已有结构化落盘路径：`test_env/release_evidence/clean_checkout_smoke_report.json`、`test_env/release_evidence/non_live_gate_report.json`、`test_env/release_evidence/release_contracts_and_capability_matrix_report.json`；builder 会优先读取这些最新报告，而不是继续依赖固定摘要。
- canonical `non_live_gate_report.json` 当前已存在且为 `passed`；targeted release / capability matrix 报告为 `38 passed`；canonical `clean_checkout_smoke_report.json` 也已在 `test_env/release_evidence/` 下通过收集并写回。
- 当前开发态 checkout 上的 `smoke_runner_report.json` 仍可作为本地信号存在，但它不再属于 canonical required release evidence；正式发布应以 `clean_checkout_smoke_report.json` 为准。
- `python tools/collect_release_evidence.py --output-root test_env/release_evidence` 当前已可统一收集 `clean_checkout_smoke`、`non_live_gate` 和 targeted release tests，并在本次实跑中得到 `release_evidence_status=passed`。
- `python tools/run_clean_checkout_smoke.py --output-root test_env/clean_checkout_smoke_real` 当前已生成 `test_env/clean_checkout_smoke_real/clean_checkout_smoke_report.json`，状态为 `passed`，并记录了 2 次连续默认 smoke 在 clean checkout 上都保持空 `git status --short`。
- 阶段 D 当前固定安全产物路径为：`test_env/security/sbom.json`、`test_env/security/python_vuln_scan_report.json`、`test_env/security/container_vuln_scan_report.json`、`test_env/security/vulnerability_exception_report.json`、`test_env/security/backup_restore_rehearsal_report.json`、`test_env/security/security_posture_report.json`。
- 阶段 D 当前 remediation 产物路径为：`test_env/release_evidence/security/vulnerability_remediation_report.json`。
- 阶段 D 当前受管 exception 输入路径为：`deployment/security/vulnerability_exceptions.input.json`；collector / preflight 会默认从该路径生成 canonical `vulnerability_exception_report.json`。
- canonical release evidence 当前还会在 `test_env/release_evidence/security/` 下收集：`sbom.json`、`python_vuln_scan_report.json`、`container_vuln_scan_report.json`、`vulnerability_exception_report.json`、`backup_restore_rehearsal_report.json`、`security_posture_report.json`。
- 阶段 D 当前固定基线文档为：`docs/guides/SECURITY_BASELINE.md`、`docs/guides/AUDIT_TRAIL_POLICY.md`、`docs/guides/BACKUP_RESTORE_RUNBOOK.md`、`docs/guides/INCIDENT_RESPONSE_MATRIX.md`。
- `python tests/run_smoke_tests.py --output-root test_env/smoke_phase_d_rehearsal_chain` 当前已通过，SBOM、Python/container 漏洞扫描报告、`backup_restore_rehearsal_report` 和 `security_posture_status=ready` 都已进入默认 smoke 主链；隔离 smoke 根下的客户验收包也已实测得到 `customer_acceptance_bundle_security_posture=ready`。最新 smoke artifact 摘要还会在 `release_readiness_report.json`、`industrial_release_readiness_report.json`、`stable_promotion_checklist.json`、`industrial_promotion_checklist.json`、`worktree_release_blocker_report.json`、`customer_acceptance_bundle.json`、`customer_acceptance_bundle_industrial.json`、`release_rehearsal_report.json` 与 `industrial_delivery_rehearsal_report.json` 路径后直接拼出 `exception_review=...`、`external_mainline=...` 与 `worktree=...`，不需要再手工翻 JSON。
- `python tools/collect_release_evidence.py --output-root test_env/release_evidence` 当前已通过，并生成 canonical security evidence；最新合并后 main artifact `test_env/gh_run_28967203208_security_artifacts/security/vulnerability_exception_report.json` 由受管输入自动生成，`vulnerability_exception_active=0`，`vulnerability_exception_review_report.json` 当前为 `status=passed` / `review_candidate_count=0`，而 `security_posture_report.json` 当前为 `security_posture_status=ready`。
- 合并后 main GitHub Actions run `28967203208` 当前已生成 canonical `security_release_preflight_report.json`，状态为 `passed`，并显式证明 Python findings `0`、container findings `0`、`accepted_finding_count=0`、`unresolved_finding_count=0`、active exceptions `0`、review-due exceptions `0`、expired exceptions `0`。
- 当前 canonical 安全扫描实测结果为：Python 依赖报告当前已 `passed` 且 `finding_count=0`；Zenoh router 自管交付镜像 `deployment-zenoh-router` 当前已扫到 `0 findings`；`deployment-web-panel-distributed` 的 canonical container 报告当前已扫到 `0 findings`，最新 `vulnerability_remediation_report.json` 已为 `ready`，其中 `accepted_finding_count=0`、`unresolved_finding_count=0`、`matched_exception_count=0`。
- `PRODUCTION_DEPLOYMENT_RUNBOOK.md` 现已收口到真实存在的 Docker Compose 路径，不再把 Helm / Kubernetes / `docker-compose.prod.yml` 当成当前有效入口。
- release artifact 现在会写出 `release_policy`，其中 `dev` 允许未闭合的 `opt_in` evidence / `diagnostic_ready` domain，`rc` 和 `stable` 则要求这些项先收口后才能达到 `ready`。
- `stable` 通道还会显式要求 `release_approval.status=approved`，并校验 `approved_by`、`approved_at`、`commit_sha`；如果当前仓库 Git HEAD 可解析，`release_source` 会被写入 manifest，stable gate 还会校验签核 SHA 与当前 HEAD 一致，并要求当前 HEAD 存在与 `version` 或 `v{version}` 匹配的 Git tag。
- Workflow 契约和 orchestrator 回归测试当前为 `16 passed`。
- `tests/test_workflow_orchestrator.py` 当前为 `12 passed`；此前会回写 `.agi_data/workflows/artifacts/*_smoke_real/` 的两条 real smoke 测试现在已改为显式使用临时 `output_root`，重复执行后不再改动那 6 个 tracked artifact。
- Web/Godot session 与 workflow 路由回归测试当前为 `36 passed`。
- Distributed smoke runner、distributed status 和容器入口路径回归测试当前为 `7 passed`。
- 最新串行 `python -m pytest -m "not live" -q` 验证结果为 `994 passed, 3 skipped, 3 deselected, 1 warning`。
- `python tools/build_release_artifact.py --version 2026.04.15-rc-evidence --channel rc --build-id build-20260415-security-preflight --output test_env/release/release_manifest_rc_evidence.json` 当前已基于最新 `test_env/release_evidence/*.json` 报告生成 `release_gate_status=ready` 的 rc manifest。
- `python tools/build_customer_acceptance_bundle.py --manifest test_env/release/release_manifest_rc_evidence.json --output test_env/release/customer_acceptance_bundle_rc_evidence.json` 当前这条 codepath 已把 `vulnerability_exception_review`、`customer_external_bindings_closure` 与 `external_mainline_execution_plan` 纳入 acceptance reports；在最新 schema 下重建时，bundle 还会把 `vulnerability_exception_review` 与 `external_mainline_execution_plan` 提升成顶层字段与 summary / CLI stdout 信号，并和 `security_posture_report`、`vulnerability_exception_report`、`customer_external_bindings_confirmation` 一起对外暴露这份 residual-risk review / external bindings closure / external-mainline surface。
- `python tools/build_extension_execution_instance.py --output test_env/release_evidence/operations/extension_execution_instance.json --vulnerability-exception-report test_env/release_evidence/security/vulnerability_exception_report.json` 当前已生成 canonical `extension_execution_instance.status=ready`、`ready_profiles=3/3`；最新受管 vulnerability exception 输入已无 active exceptions。
- `python tools/build_extension_execution_schedule.py --output test_env/release_evidence/operations/extension_execution_schedule.json --instance-artifact test_env/release_evidence/operations/extension_execution_instance.json` 当前已生成 canonical `extension_execution_schedule.status=ready`、`ready_profiles=3/3`、`window_trigger_at` / `closure_archive_due_at` 已对齐客户窗口。
- `python tools/build_extension_execution_actuals.py --output test_env/release_evidence/operations/extension_execution_actuals.json --schedule-artifact test_env/release_evidence/operations/extension_execution_schedule.json --window-trigger-recorded-by delivery_lead --signoff-recorded-by customer_operator --residual-risk-reviewed-by delivery_lead --closure-archived-by rollback_owner --approval-identity-source-type customer_ticket_registry --approval-identity-reference canonical-release/window-20260416/customer_operator --archive-target-binding-type customer_archive_destination --archive-target-binding-reference-base archive://canonical-release/window-20260416 --due-trigger-binding-type customer_due_trigger_schedule --due-trigger-binding-reference-base schedule://canonical-release/window-20260416 --due-trigger-checked-at 2026-04-16T13:20:08+01:00` 当前已生成 canonical `extension_execution_actuals.status=ready`、`ready_profiles=3/3`、`approval_identity_source_type=customer_ticket_registry`、`approval_identity_reference=canonical-release/window-20260416/customer_operator`、`archive_target_binding_type=customer_archive_destination`、`due_trigger_binding_type=customer_due_trigger_schedule`、`due_trigger_checked_at=2026-04-16T13:20:08+01:00`，并已实际写出 `approval_identity_source.json`、profile 级 `due_trigger_check.json` / `archive_target.json`、以及 `window_trigger.json` / `signoff.json` / `exception_review.json` / `residual_risk_review.json` / `closure_archive/index.json` / `closure_manifest.json`。
- `python tools/build_customer_acceptance_bundle.py --manifest test_env/release/release_manifest_rc_evidence.json --output test_env/release/customer_acceptance_bundle_rc_evidence.json` 当前可把最新 manifest、canonical evidence 和一线交付文档汇总成客户验收包。
- `python tools/build_release_artifact.py --version 2026.04.16-rc-actuals --channel rc --build-id build-20260416-actuals --release-summary "Customer window actuals external bindings closed." --output test_env/release/release_manifest_rc_actuals.json` 当前已生成 `release_gate_status=ready`、`customer_delivery_status=ready`、`industrial_delivery_status=ready`、`extension_execution_actuals.status=ready`、`extension_execution_actuals.approval_identity_reference=canonical-release/window-20260416/customer_operator`、`extension_execution_actuals.archive_target_binding_type=customer_archive_destination`、`extension_execution_actuals.due_trigger_binding_type=customer_due_trigger_schedule` 的 rc manifest。
- `python tools/build_customer_acceptance_bundle.py --manifest test_env/release/release_manifest_rc_actuals.json --output test_env/release/customer_acceptance_bundle_rc_actuals.json` 当前这条 codepath 也已把 `vulnerability_exception_review` 与 `external_mainline_execution_plan` 接进 acceptance reports；在最新 schema 下重建时，bundle 会继续附带 `extension_execution_actuals.status=ready`、独立的 residual-risk review evidence，以及剩余外部主线计划面，并把这两项直接提升为顶层 bundle 字段。
- `test_env/release_rehearsal_industrial/test_env/industrial_release_readiness_ready/industrial_release_readiness_report.json` 当前已生成，实际状态为 `industrial_release_gate=ready`、`security_preflight=passed`、`customer_delivery=ready`、`industrial_delivery=ready`，并已附带顶层 `worktree_release_blocker.status=ready`；最新本地重建的 external-mainline evidence 为 `external_mainline_execution_plan.status=ready`、`completed=2`、`ready_to_run=1`、`waiting_external_input=0`、`blocked=0`，`external_mainline_input_checklist.status=passed`、`missing_input_count=0`，剩余项为真实 `industrial_delivery_live_evidence` 现场执行与归档留痕。
- `test_env/release_rehearsal_industrial/test_env/industrial_promotion_ready/industrial_promotion_checklist.json` 当前已生成，实际状态为 `industrial_release_gate=ready`、`blocking_steps=0`、`ready_to_promote=true`；最新本地重建的 external-mainline summary 已收敛为 `external_mainline=ready/2/1/0/0` 与 `external_mainline_input_checklist=passed/0/0/1/2`，并保留 non-blocking `external_mainline_execution_plan` / `external_mainline_input_checklist` step。
- `test_env/release_rehearsal_industrial/test_env/release/customer_acceptance_bundle_industrial.json` 当前已重新生成，最新流程下会把 `industrial_release_readiness`、`industrial_promotion_checklist`、`industrial_delivery_rehearsal_report`、`vulnerability_exception_review`、`customer_external_bindings_closure`、`customer_external_bindings_confirmation`、`external_mainline_execution_plan`、`external_mainline_input_checklist` 与 `release_ops_execution` 一起带进 acceptance reports；对应 canonical `release_rehearsal_report.json` 内联的 `industrial_customer_acceptance_bundle` 摘要当前为 `reports_present=18/18`，而最新 external-mainline artifact 显式携带 `vulnerability_exception_replacement.status=completed`、`external_mainline_input_checklist.status=passed` 与 `external_mainline_input_checklist.metrics.missing_input_count=0`。
- `python tools/run_release_rehearsal.py --version 2026.04.17-industrial-rehearsal --build-id release-rehearsal-industrial --output-root test_env/release_rehearsal_industrial` 当前已实跑通过，`release_rehearsal_report.json` 实际状态为 `status=passed`、`release_gate_status=ready`、`extension_execution_actuals.status=ready`，并附带 `industrial_delivery_artifact_paths=5/5`；runner 当前会默认复制 `deployment/customer_delivery.external_bindings.rehearsal.json` 到演练工作区。
- `test_env/release_rehearsal_industrial/release_manifest_industrial.json`、`industrial_release_readiness_report.json`、`industrial_promotion_checklist.json`、`customer_acceptance_bundle_industrial.json` 与 `industrial_delivery_rehearsal_report.json` 当前都已附带 `extension_execution_actuals.status=ready`、`ready_profiles=3/3`、`approval_identity_source_type=customer_ticket_registry`、`approval_identity_reference=release-rehearsal-industrial/window-release-rehearsal/customer_operator`、`archive_target_binding_type=customer_sharepoint_archive`、`due_trigger_binding_type=customer_service_now_schedule`、`due_trigger_checked_at=2026-04-17T16:03:49.052924+00:00` 与 follow-up artifact 路径；同一批产物现在还会显式带出 `external_bindings_status=ready`、`external_bindings_ready_count=3/3`、`external_bindings_confirmed_count=3/3`、`external_bindings_follow_up_required=false`，并记录受管演练配置 `deployment/customer_delivery.external_bindings.rehearsal.json` 与对应 synthetic confirmation metadata；仓内默认 `deployment/customer_delivery.external_bindings.json` 仍保持 placeholder，用于后续客户实例化时替换为真实客户系统绑定；其中 industrial manifest 为 `release_gate_status=ready`，promotion checklist 为 `blocking_steps=0`，独立 industrial rehearsal report 为 `status=ready`。
- `test_env/release_rehearsal_industrial/release_manifest_industrial.json`、`industrial_release_readiness_report.json`、`industrial_promotion_checklist.json` 与 `customer_acceptance_bundle_industrial.json` 当前都已带出 `extension_support_surface.status=ready`、`declared_profiles=4/4`，并开始显式携带 profile 级别的实施动作、专项验收命令和回滚前提。
- `test_env/release_rehearsal_industrial/test_env/industrial_promotion_ready/industrial_promotion_checklist.json`、`customer_acceptance_bundle_industrial.json`、`release_rehearsal_report.json` 与 `industrial_delivery_rehearsal_report.json` 当前都已开始附带 `extension_execution_plan.status=ready`，其中 `actionable_profiles=3`；同一批产物现已同时附带 `extension_execution_evidence.status=ready`、`ready_reports=4/4`、`extension_execution_instance.status=ready`、`ready_profiles=3/3`，以及 `extension_execution_schedule.status=ready`、`ready_profiles=3/3`，且 `extension_execution_evidence` 当前会把 `customer_external_bindings_confirmation` 一并计入机器可读 evidence surface，而 acceptance reports 当前会额外挂出 `customer_external_bindings_closure.status=passed`、`external_mainline_execution_plan.status=ready`、`external_mainline_input_checklist.status=passed` 与 `release_ops_execution.status=passed`。`release_rehearsal_report.json` 当前也已把 `customer_external_bindings_closure`、`external_mainline_execution_plan` 与 `release_ops_execution` 提升成顶层机器字段；与此同时，`industrial_customer_acceptance_bundle` 摘要现在还会内联 `vulnerability_exception_review`、`external_mainline_execution_plan`、`external_mainline_input_checklist` 与 `release_ops_execution`，独立 `industrial_delivery_rehearsal_report.json` 在顶层字段缺失时会优先消费这层内联摘要或 bundle 顶层字段，只在最后才回退到 `acceptance_reports`。当前最新 external-mainline 重建结果显式带出 `external_mainline=ready/2/1/0/0`、`external_mainline_input_checklist=passed/0/0/1/2` 与 `external_bindings_closure=passed`，并保持 industrial rehearsal `stage_summary.total=6`、`stage_summary.passed=6`、`stage_summary.failed=0`。promotion checklist 现已继续保留非阻塞 `extension_external_bindings` 步骤，并新增 non-blocking `external_mainline_execution_plan` / `external_mainline_input_checklist` step；剩余外部主线已缩小为真实客户工业环境执行与现场留痕。
- `python tools/build_customer_external_bindings_config.py --output deployment/customer_delivery.external_bindings.customer.json --instance-artifact test_env/release_evidence/operations/extension_execution_instance.json` 与 `python tools/confirm_customer_external_bindings.py --config deployment/customer_delivery.external_bindings.customer.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <confirmed-by> --confirmation-ticket <confirmation-ticket>` 当前都已落地，可先从客户窗口实例生成 customer-specific external bindings config，再显式确认哪些 section 已切到真实客户系统。生成文件当前会显式写出 `binding_state=draft` 与 integration notes，因此不会被误判为真实客户系统已闭合；而现在即便字段值已经像真实客户系统，只要没有进入 `binding_state=confirmed` 并带上 `confirmed_by`、`confirmed_at` 与 `confirmation_ticket` 这组最小审计字段，actuals 仍会把它识别成 `external_bindings_unconfirmed_sections`，而不是错误地算作 ready。`build_extension_execution_actuals.py` 生成的 actuals 现在会直接暴露 `external_bindings_confirmed_sections`、`external_bindings_confirmed_by`、`external_bindings_confirmation_tickets` 与 `external_bindings_last_confirmed_at`，因此 `customer_acceptance_bundle`、`check_release_readiness.py`、`stable_promotion_checklist` 与 `industrial_promotion_checklist` 现在都能沿着 generate -> confirm -> rebuild 这条链直接读取确认摘要，而不需要回头解析整份 customer config。
- `tools/run_customer_external_bindings_closure.py` 当前已落地，external bindings 的真实客户闭环不再需要手工串 `generate -> confirm -> rebuild actuals -> build confirmation report -> collect_release_evidence` 多条命令。runner 会在 customer-specific config 缺失时自动先生成 draft config；如果同一条命令没有提供任何 `--set section.field=value` 或 `--overrides-file` 覆盖，它会在写出 draft config 后阻塞退出，避免把默认占位值直接 confirm 成 ready。`tools/confirm_customer_external_bindings.py` 现在也已支持 `--overrides-file`，可直接消费受管 JSON 映射；仓内模板为 `deployment/customer_delivery.external_bindings.overrides.example.json`。`check_release_readiness.py`、`stable_promotion_checklist`、`industrial_promotion_checklist` 与 `customer_acceptance_bundle` 现已开始把这条 runner 作为 primary closure command 暴露出来。
- `tools/run_customer_external_bindings_closure.py` 当前还会无论成功或阻塞都写出独立的 `test_env/release_evidence/operations/customer_external_bindings_closure_report.json`，把 `failed_steps`、`selected_sections`、config/actuals/confirmation evidence 是否已落盘，以及本次是否执行 `collect_release_evidence.py` 固化成结构化 `release_evidence_report`，不再只依赖 runner stdout。`check_release_readiness.py`、`stable_promotion_checklist`、`industrial_promotion_checklist` 与 `customer_acceptance_bundle` 现在都会直接读取最近一次 closure report，把 `failed_steps` 与 report 路径带进 follow-up / checklist / acceptance 摘要。
- `tools/build_customer_external_bindings_confirmation_report.py` 当前已落地，并会把 `extension_execution_actuals` 内的 external bindings 确认状态固化成独立 `customer_external_bindings_confirmation_report.json`；`tools/collect_release_evidence.py` 现在会在 actuals 之后自动生成这份报告，`customer_acceptance_bundle` 与 canonical industrial rehearsal 也已开始把它作为 acceptance report 挂载。
- `tools/collect_release_evidence.py` 当前已开始优先读取 `deployment/customer_delivery.external_bindings.customer.json`；只有当 customer-specific 配置不存在时，才回落到 repo 内默认 placeholder `deployment/customer_delivery.external_bindings.json`。
- `PRODUCTION_DEPLOYMENT_RUNBOOK.md`、`docs/guides/CUSTOMER_INSTALLATION_GUIDE.md`、`docs/guides/DISTRIBUTED_GUIDE.md`、`docs/ros2/ROS2_QUICK_START.md` 与 `docs/guides/GODOT_TESTING_GUIDE.md` 当前已接入 `runbook_entrypoints` / `execution_template` 语义，不再只是静态旁路文档。
- 默认非 integration/hardware 覆盖率门禁当前为 `704 passed, 28 deselected`，并生成 `coverage.xml`。
- clean checkout smoke 当前为 `passed`，`runs=2`，并在 `test_env/clean_checkout_smoke_real/clean_checkout_smoke_report.json` 中保留了逐次 stdout/stderr 与 worktree clean 结果。

## 当前保存计划

当前保存的执行顺序如下：

1. 保持 `deployment/security/vulnerability_exceptions.input.json` 作为 canonical 输入；当前旧 no-fix exceptions 已退休，后续若 scanner DB 刷新重新出现 findings，应通过真实修复或新的显式审批重新进入 preflight。
2. `industrial_delivery_gate` 当前已经进入 `release_manifest`、`release_readiness_report.json`、`stable_promotion_checklist.json` 与 `tools/run_release_rehearsal.py`，并且正式 `industrial` release channel 也已开始把它作为 blocking gate。
3. stable readiness / checklist 的 `clean_worktree` 阻塞项已在当前分支闭合；最新 `test_env/release_evidence/operations/worktree_release_blocker_report.json` 为 `status=ready`、`clean_worktree=true`、`total_paths=0`、`tracked_review_status=not_required`。`customer_delivery_surface` 与 `industrial_delivery_gate` 也已闭合。
4. `industrial` channel 的独立 readiness / promotion flow 当前已机器化，smoke 与 customer acceptance bundle 也已切到工业通道自己的 readiness/checklist 文件名。
5. distributed / ROS2 / Godot 扩展面的客户支持边界、实施动作、专项验收命令和回滚前提当前都已进入 `extension_support_surface`，并进一步汇总到 `extension_execution_plan`；正式 runbook / 安装指南 / 扩展专项指南现在已通过 `runbook_entrypoints` 接入这组执行面摘要。
6. `extension_execution_evidence` 当前已落地，`extension_on_call_rehearsal`、`extension_exception_review_schedule`、`extension_escalation_closure` 与 `customer_external_bindings_confirmation` 四类留痕报告已经进入 manifest / bundle / readiness / checklist / rehearsal 的机器字段。
7. `extension_execution_instance` 当前已落地，客户实例化的交付窗口、exception 复核到期时间与 closure archive 目标已经进入 manifest / bundle / industrial readiness / industrial promotion / rehearsal 的机器字段。
8. `extension_execution_schedule` 当前已落地，客户窗口触发、signoff 截点、residual risk review 记录与 closure archive manifest 已进入 manifest / bundle / industrial readiness / industrial promotion / rehearsal 的机器字段。
9. `extension_execution_actuals` 当前已落地，客户窗口实际触发、实际 signoff、residual risk review 实际留痕、approval identity source、due trigger check、archive target、exception review follow-up 与 closure archive index 已进入 manifest / bundle / industrial readiness / industrial promotion / rehearsal 的机器字段。
10. canonical industrial rehearsal 已不再停留在 repo 内默认 placeholder JSON，而是默认使用带 synthetic confirmation metadata 的 `deployment/customer_delivery.external_bindings.rehearsal.json` 生成 `external_bindings_status=ready`；真实客户交付窗口现在也可以先生成 `deployment/customer_delivery.external_bindings.customer.json`，但这份文件默认仍是 `binding_state=draft`。当前正式收口动作已经落到 `tools/confirm_customer_external_bindings.py`；下一步是把其中 `approval_identity`、`archive_target` 与 `due_trigger` 三段替换成真实客户系统元数据，并通过 confirm 工具把对应 section 切到 `confirmed`，而不是继续停留在通用生成值。
11. `tools/build_external_mainline_execution_plan.py` / `tools/run_external_mainline_execution_plan.py` 当前已把剩余三条外部主线收口到统一计划面；最新本地重建的 plan 当前显式标记 `customer_external_bindings_closure` 与 `vulnerability_exception_replacement` 为 `completed`，`industrial_delivery_live_evidence` 为 `ready_to_run`。最新 `--inputs-file`、`deployment/external_mainline.inputs.example.json` 与 `tools/build_external_mainline_inputs.py` 已经把这条主线的常规入参收口成统一受管 JSON；下一步不再是补缺字段，而是在真实客户环境执行现场 install / upgrade / rollback / backup-restore 并归档 live evidence。
12. 持续复跑 canonical release evidence / security preflight，确保 residual risk 的变化会被结构化证据、industrial delivery gate 与发布门禁及时反映。

Phase D 的 security preflight 复绿、Phase E 的客户交付文档集接线、Phase F 的 rehearsal bridge、industrial channel gate、独立 industrial readiness / promotion flow，以及 `extension_support_surface` / `extension_execution_plan` / `runbook_entrypoints` / `execution_template` / `extension_execution_evidence` / `extension_execution_instance` / `extension_execution_schedule` / `extension_execution_actuals` 的执行面 contract 都已经完成；当前不再扩新功能面，下一阶段围绕客户特定窗口实例化、closure 实际归档和 residual risk 到期执行展开。

## 阶段一契约状态

阶段一新增的稳定契约入口位于：

- [agi_walker/core/api/workflow_contracts.py](../agi_walker/core/api/workflow_contracts.py)

当前已固定并测试的契约包括：

- `workflow_step` artifact
- `workflow_result`
- workflow definition
- RobotConfig
- PartSpec
- mass optimization result
- URDF/SDF/MJCF export result

真实 workflow step artifact 当前包含 `schema_version=1.0`、`artifact_type=workflow_step`、executor/action/status/mode、resolved inputs、executor output、attempt count、duration 和创建时间。后续 Web、MCP、distributed runtime 应复用该契约，不应重新定义 ad hoc 字段。

完整补全路线见：

- [功能补全计划](FEATURE_COMPLETION_PLAN.md)
- [发布指南](guides/RELEASE_GUIDE.md)

## Web / Godot 状态

Web Panel 的 FastAPI 入口位于 [web_panel/server.py](../web_panel/server.py)，当前公开的主要能力包括：

- 根页面 `/`
- 主控制台 `/static/index.html`
- Workflow 控制台 `/static/workflows.html`
- Nightly 面板 `/static/nightly.html`
- Distributed 面板 `/static/distributed.html`
- Godot 设计页 `/static/design.html`
- Godot 控制页 `/static/godot-control.html`
- 发布面矩阵接口 `/api/capabilities/matrix`

Godot 集成当前同时支持两种 backend：

- `legacy`
- `godot-agent`

切换环境变量：

```text
AGI_WALKER_GODOT_AGENT_BACKEND=legacy|godot-agent
```

阶段二后，Web workflow 与 Godot delivery 使用以下稳定元数据：

- workflow/run 响应包含 `workflow_contract_version`、`workflow_result_schema_version` 和 `workflow_result_artifact_type`。
- artifact manifest 包含 `schema_version`、`contract.valid`、`contract.errors`、`contract.payload_type` 和 `step_artifact_contract`。
- Godot session status payload 使用 `schema_version=1.0`，状态值限定为 `disconnected`、`launching`、`connected`、`schema_ready`、`running`、`failed`。
- Godot delivery record 会保存 artifact contract、`session_state`、`session_status`、`schema_available` 和 `transport_status_url`。

## 推荐的配置入口

Web workflow 相关参数优先从以下 env 文件读取：

- `deployment/web_panel.env`
- `deployment/compose.env`
- `deployment/web_panel.env.example`
- 或通过 `AGI_WALKER_WEB_ENV_FILE` 显式指定

当前需要重点关注的环境变量：

- `AGI_WALKER_COMPOSE_WEB_ENV_FILE`
- `AGI_WALKER_RUNTIME_ROOT`
- `AGI_WALKER_WEB_PORT`
- `AGI_WALKER_WEB_DISTRIBUTED_PORT`
- `AGI_WALKER_DATABASE_URL`
- `AGI_WALKER_WEB_OUTPUT_ROOT`
- `AGI_WALKER_WEB_ARCHIVE_ROOT`
- `AGI_WALKER_SECRET_KEY`
- `AGI_WALKER_WEB_RUNS_PAGE_SIZE`
- `AGI_WALKER_WEB_RUNS_MAX_PAGE_SIZE`
- `AGI_WALKER_WEB_ARCHIVE_MAX_RUNS`
- `AGI_WALKER_WEB_ARCHIVE_MAX_AGE_DAYS`
- `AGI_WALKER_ZENOH_ENDPOINT`
- `AGI_WALKER_DISTRIBUTED_ACTOR_TTL_SECONDS`
- `AGI_WALKER_GITHUB_REPO`
- `AGI_WALKER_GITHUB_TOKEN`
- `AGI_WALKER_GITHUB_WORKFLOW_FILE`
- `AGI_WALKER_NIGHTLY_STATUS_CACHE_SECONDS`

Godot headless smoke 相关环境变量：

- `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE`
- `AGI_WALKER_GODOT_HEADLESS_SCENE`
- `AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR`
- `AGI_WALKER_GODOT_HEADLESS_CONNECT_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_SCHEMA_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_STEP_COUNT`

ROS2 bridge live smoke 相关环境变量：

- `AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE`
- `AGI_WALKER_ROS2_BRIDGE_SMOKE_ARTIFACT_DIR`

Nightly 当前跟踪的专项 job：

- `smoke`
- `distributed-smoke`
- `godot-headless-smoke`
- `ros2-bridge-smoke`

发布面 capability matrix 当前覆盖：

- `cli`
- `web_panel`
- `mcp`
- `distributed_runtime`
- `godot_integration`

发布门禁当前通过以下产物固定：

- `release_manifest` artifact
- `capability_matrix` artifact
- `docs/guides/RELEASE_GUIDE.md`
- `tools/build_release_artifact.py`
- `docs/guides/DEPLOYMENT_MATRIX.md`
- `docs/guides/CUSTOMER_INSTALLATION_GUIDE.md`

## Distributed Runtime 状态

当前 Docker compose 分布式入口使用模块路径：

- learner：`python -u -m agi_walker.core.distributed.run_learner`
- sidecar：`python -u -m agi_walker.core.distributed.sidecar`

分布式监控和烟测当前使用以下稳定元数据：

- `/api/distributed/status` 返回 `schema_version=1.0`、`actors`、`actor_ids`、`actors_count` 和 `monitor`。
- `distributed_monitor` capability 返回 `schema_version=1.0`、Zenoh endpoint、订阅 topic `ag/*/obs`、actor TTL、active actor 数量和最近裁剪状态。
- `tests/run_distributed_smoke.py --report-file ...` 会输出 `schema_version=1.0` 的 smoke report，包含 compose build/up、Web status、monitor、sidecar start、actor discovery 和 learner action loop 的逐项结果。
- smoke runner 默认发布 Zenoh 宿主端口 `17447/18000`，避免与本地 Web/API 服务常用 `8000` 端口冲突。
- smoke runner 默认向 sidecar 注入 `AGI_WALKER_FORCE_OFFLOAD=1`，用于 Docker smoke 的启动闭环；否则 sidecar 初始 `cloud_available=False` 会导致 learner 收不到第一帧 observation。
- CI `distributed-smoke` job 只在 `workflow_dispatch` 或 `schedule` 路径运行，并上传 `distributed-smoke-artifacts`。
- smoke 失败诊断会识别 Docker buildx 权限、Docker Desktop Linux engine 不可用、旧 `/app/distributed/*.py` 入口、Python 模块导入失败、Zenoh 连接和 Godot TCP 连接问题。

本机实机 smoke 已在 `2026-04-14` 重新执行并通过。最新通过报告位于 `test_env/distributed_smoke/distributed_smoke_report.json`，状态为 `passed`，7/7 checks 通过，已覆盖 `compose_build`、`compose_up`、`web_panel_status`、`distributed_monitor`、`sidecar_start`、`actor_discovery` 和 `learner_action_loop`。当前 release artifact 也已成功吸收该证据，并将 `distributed_runtime` 从 `diagnostic_ready` 提升为 `ready`。复跑命令保持不变：

```powershell
python tests\run_distributed_smoke.py --build --stop-after --report-file test_env\distributed_smoke\distributed_smoke_report.json
```

Godot headless live smoke 也已在 `2026-04-12` 本机通过。当前通过报告位于 `test_env/godot_headless_smoke/headless_smoke_report.json`，状态为 `passed`，已覆盖 Godot executable 发现、headless 场景拉起、TCP 建连、schema 获取、`load_robot` 和 step loop。复跑命令：

```powershell
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
python -m pytest tests\test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv
```

ROS2 bridge live smoke 也已在 `2026-04-12` 的 `ros:humble-ros-base` 容器化 ROS2 Humble runtime 中通过。当前通过报告位于 `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json`，状态为 `passed`，已覆盖 ROS2 runtime 预检、`start_sim` / `stop_sim` service、JointState 映射、`/cmd_vel -> update_params` 和 TCP bridge 命令闭环。复跑时应在具备 ROS2 Humble Python runtime 的主机或容器中执行：

```powershell
$env:AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE='1'
python -m pytest tests\test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv
```

## Training / Hardware 边界状态

阶段四新增的稳定训练契约入口位于：

- [agi_walker/core/api/training_contracts.py](../agi_walker/core/api/training_contracts.py)

当前 `training_run` artifact 使用 `schema_version=1.0`，字段包括：

- `run_id`
- `run_type`
- `stage`
- `status`
- `algorithm`
- `environment`
- `inputs`
- `metrics`
- `artifacts`
- `hardware_required`
- `hardware_enabled`
- `started_at`
- `finished_at`
- `duration_seconds`

训练类型当前限定为：

- `mock_training`
- `offline_dataset_training`
- `sim_training`
- `hardware_in_the_loop`

`EvolutionManager.stage_rl_training()` 当前会在 mock RL 阶段写出 `training_run_manifest.json`。`OfflineRLTrainer.train_offline()` 当前会在 `save_dir` 写出 `training_run_manifest.json`，并记录数据摘要、训练步数、模型目录和硬件边界。`RLOptimizer.train()` 当前也会写出同名 manifest，并按以下规则分类：

- `DummyEnv` -> `mock_training`
- 普通仿真环境 -> `sim_training`
- 显式 `hardware_required=True` -> `hardware_in_the_loop`

这一步先解决训练产物“无统一元数据”的问题，并把 sim/mock/hardware 的训练标签从约定提升为受测试约束的行为。

硬件控制器边界本轮也完成了一次收口：

- `IMC22Controller` 支持 `bus=` 注入，默认单元测试不需要真实 `python-can` 总线。
- `IMC22Controller.from_replay()` 可从回放文件构造 controller。
- `ReplayCANBus` 和 `tests/fixtures/imc22_status_replay.json` 可回放 IMC-22 状态帧。
- `HardwareEnvironment(controller=...)` 可直接复用 replay controller 跑 `reset()` / `step()`。
- `RealRobotDriver` 支持 `transport=` 注入和 `RealRobotDriver.from_replay()`。
- `tests/fixtures/real_robot_driver_replay.json` 可回放串口状态帧。
- `tests/test_real_robot_driver.py` 已覆盖 mock driver、replay driver 和 mock SysID 数据采集。
- ROS2 bridge 支持 replay payload 校验 helper。
- `tests/fixtures/ros2_bridge_replay.json` 可回放 bridge 输入数据。
- `tests/test_ros2_bridge_runtime.py` 已在 fake ROS2 runtime 下覆盖 JointState/RobotState 映射、`/cmd_vel` 参数转换和 start/stop service 基线。
- `tests/test_ros2_bridge_smoke.py` 已补成真实 ROS2 Python runtime 下的 opt-in live smoke，并会输出 `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json`。
- CI 已新增 `ros2-bridge-smoke` manual/schedule job，并上传 `ros2-bridge-smoke-artifacts`。
- `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json` 当前已为 `passed`，通过证据来自容器化 `ros:humble-ros-base` 运行时，而不是当前宿主 Python 环境。

这意味着默认 pytest 已经能验证 IMC-22 协议编码、节点发现、基础环境闭环，串口驱动的数据包打包和状态回放，以及 ROS2 bridge 的核心消息映射，而不会访问真实 CAN、真实串口或真实 ROS 2 环境；同时目标 ROS2 Humble 环境现在也有单独的 opt-in smoke 入口。

当前已验证：

- `python -m pytest tests\test_training_contracts.py tests\test_verify_mocked.py tests\test_docs_utf8.py tests\test_offline_rl.py tests\test_rl_optimizer_training_contract.py -q`：`12 passed`
- `python -m pytest tests\test_hardware_controller.py -q`：`16 passed, 1 skipped`
- `python -m pytest tests\test_real_robot_driver.py -q`：`5 passed, 1 skipped`
- `python -m pytest tests\test_ros2_bridge_runtime.py tests\test_ros2_workspace.py -q`：`5 passed`
- `python -m pytest tests\test_ros2_bridge_smoke.py tests\test_ros2_bridge_runtime.py tests\test_ros2_workspace.py -q`：`5 passed, 1 skipped`
- `docker run --rm -v "<repo>:/workspace" -w /workspace ros:humble-ros-base bash -lc 'source /opt/ros/humble/setup.bash && ... && AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1 python3 -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv'`：`1 passed`
- `python -m pytest tests\test_training_contracts.py tests\test_verify_mocked.py tests\test_offline_rl.py tests\test_rl_optimizer_training_contract.py tests\test_hardware_controller.py tests\test_real_robot_driver.py tests\test_ros2_bridge_runtime.py tests\test_ros2_workspace.py tests\test_docs_utf8.py -q`：`38 passed, 2 skipped`
- `python -m pytest -m "not live" -q`：`854 passed, 4 skipped, 3 deselected`

## 当前风险与边界

本次修复聚焦于“主入口文档 + MCP 入口 + 核心用户路径”。以下内容仍然可能需要后续清理：

- `docs/archive_and_reports/` 下的历史归档文档
- 部分未纳入本轮修复的旧文档页面
- 依赖真实 Godot 可执行文件的 headless smoke 路径
- 依赖 Docker Desktop/Linux engine 的 distributed smoke 路径
- 当前开发态 checkout 上如果 Git worktree 不 clean，`smoke_runner` 结构化报告仍会对 stable 相关检查给出 `blocked`；本轮已重新生成 `test_env/release_evidence/operations/worktree_release_blocker_report.json`，当前分支为 `status=ready`、`clean_worktree=true`。
- Phase D 当前已通过 Web Panel Alpine 候选把 `deployment-web-panel-distributed` 的 container findings 降到 `0`，旧 no-fix exceptions 已退休；后续风险转为 scanner DB / upstream package refresh 后可能重新出现 findings。

当前 canonical stable release gate 状态为 `ready`。当前仍保留的信息性边界包括：

- MCP 当前只声明 `stdio` 作为产品化传输面
- live Godot / ROS2 验证刻意不纳入默认 `not live` 门禁
- Godot headless / ROS2 bridge live smoke 仍要求显式环境准备；当前 ROS2 通过证据来自容器化 Humble runtime，而非宿主 Python

需要额外区分的是：canonical stable manifest 仍为 `ready`，当前开发态 worktree 上重新计算的 worktree release blocker 也已为 `ready`。此前的 `clean_worktree` 当前显式阻塞项已闭合，后续如果工作区重新变脏，stable readiness / promotion checklist 会再次按真实工作区状态 fail closed。

需要单独跟踪的 residual risk 已从“阻塞发布”切换为“持续监控”：`deployment-web-panel-distributed` 当前 remote Trivy evidence 为 `0 findings`，不再依赖 no-fix exceptions；若后续 scanner DB 刷新重新发现漏洞，`security-preflight` 会重新 fail closed。

这意味着：

- 入口文档已经可以作为当前操作手册使用。
- 历史归档文档不应继续作为一线使用说明。

## 推荐阅读顺序

1. [README.md](../README.md)
2. [MCP 集成说明](mcp.md)
3. [CLI 指南](guides/CLI_GUIDE.md)
4. [Web Panel 指南](guides/WEB_PANEL_GUIDE.md)
5. [发布指南](guides/RELEASE_GUIDE.md)
6. [迁移指南](MIGRATION_GUIDE.md)
