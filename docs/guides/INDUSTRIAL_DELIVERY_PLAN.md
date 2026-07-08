# AGI-Walker 严格工业化交付执行计划

更新日期：`2026-04-16`

## 目标

本计划的目标，不是把仓库继续做成“工程上可发布”，而是把它推进到“客户可安装、可验收、可运维、可升级、可审计”的严格工业化交付状态。

当前基线：

- stable 发布门禁已经是 `ready`，见 `test_env/release/release_manifest_stable.json`
- stable readiness 已闭合，见 `test_env/release_readiness_ready/release_readiness_report.json`
- stable promotion checklist 已闭合，见 `test_env/stable_promotion_ready/stable_promotion_checklist.json`

当前主要差距：

1. required release evidence 已完成结构化自动采集，canonical release manifest 现已可基于最新 `clean_checkout_smoke`、`non_live_gate` 和 targeted reports 重建；当前已新增 `customer_acceptance_bundle` 作为阶段 B 的结构化出口，后续重点转为把它接入客户验收包和持续发布流程。
2. workflow orchestrator 的两条 real smoke 测试此前会回写 tracked workflow artifact；该根因已修复，且 `tools/run_clean_checkout_smoke.py` 已在目标 clean checkout 上完成 2 次连续默认 smoke 无副作用复验。当前剩余工作是把这条 proof 稳定纳入客户验收入口，而不是继续停留在一次性报告。
3. 生产 runbook 已收口到真实 Compose 路径，但状态页、计划页和客户文档仍需持续同步，避免再次漂移。
4. 客户交付面已完成 support matrix、capacity declaration、acceptance checklist 和 known limitations 的正式接线，并已进入 release manifest / stable readiness / stable checklist / 独立 industrial readiness / industrial checklist 输出。当前剩余工作转为扩展面支持边界细化，而不是继续补主链接线。
5. 安全与合规基线的当前 checkpoint 已经复绿：SBOM、漏洞扫描、异常审批、备份恢复、security posture 和 security preflight 都已闭合。当前剩余工作已从“让 preflight 通过”切换为“在 exception 到期前持续消化 residual risk，并把客户支持矩阵与验收包做成正式交付面”。

当前执行状态：

- 阶段 A：完成
- 阶段 B：完成
- 阶段 C：完成
- 已完成：P0-1 `release evidence` 结构化采集与 attestation
- 已完成：P0-2 默认 smoke 在 clean checkout 上的双跑无副作用复验
- 已完成：P0-3 生产 runbook 当前入口收口
- 已完成：阶段 C 的 Compose 客户部署包装、部署矩阵、安装指南、卷/端口/备份目录约定
- 当前焦点：独立 industrial readiness / promotion flow 已完成，smoke 与 customer acceptance bundle 也已切到工业通道自己的 readiness/checklist 产物。最新 canonical security chain 已达到 `security_posture_status=ready`、`security_release_preflight_status=passed`，rc manifest 与 customer acceptance bundle 也都为 `ready`。`extension_execution_actuals` 现已把客户窗口实际触发、signoff、residual risk review、approval identity source、archive target binding 和 due trigger binding 压成机器字段；默认 canonical 值当前为 `customer_ticket_registry`、`customer_archive_destination` 和 `customer_due_trigger_schedule`。接下来要继续把这组 actuals 推进到客户真实审批系统、真实 archive target 和真实调度触发适配，并在 exception 到期前持续压缩 residual risk

严格工业化交付完成的定义：

- 客户可按文档独立部署，不依赖仓库维护者口头解释。
- 发布证据可追溯、可复算、可审计。
- 默认验收链路无副作用，不污染仓库和业务数据。
- 生产部署、升级、回滚、告警、备份和灾难恢复有明确执行面。
- 安全、合规和支持边界被正式声明，而不是隐含在实现细节里。

## 执行原则

1. 先修证据链，再修包装层。
2. 先做无副作用验收，再做客户交付包装。
3. 所有“工业化完成”都必须有机器可读产物，不接受纯文档口头完成。
4. 每一阶段必须有退出门禁，未闭合前不进入下一阶段。

## 阶段 A：证据链与文档口径收口

目标：把“当前说法”和“当前真实状态”完全对齐。

执行项：

1. 把 `release_manifest` 中的 `non_live_gate`、targeted tests、smoke 结果改成从最新报告自动采集，而不是硬编码摘要。
2. 为 release builder 增加 `test evidence attestation`，要求每条 evidence 都带上：
   - 生成时间
   - 来源命令
   - 报告路径
   - 当前 commit SHA
3. 修正文档口径漂移：
   - `docs/CURRENT_STATUS.md`
   - `docs/FEATURE_COMPLETION_PLAN.md`
   - `PRODUCTION_DEPLOYMENT_RUNBOOK.md`
   - `README.md`
4. 对生产文档建立“当前有效入口”检查，防止 Helm、Kubernetes、env file、nightly 路径再次写成不存在或过期入口。

交付物：

- `release evidence` 自动采集逻辑
- 文档一致性检查
- 更新后的当前状态、发布指南、生产 runbook

退出门禁：

- release manifest 中不再出现与最新测试结果不一致的摘要
- `CURRENT_STATUS.md` 与 stable manifest 状态一致
- `PRODUCTION_DEPLOYMENT_RUNBOOK.md` 中不再保留 `[待填充]`、不存在路径或伪造环境前提

## 阶段 B：无副作用验收链路

目标：让默认 smoke、readiness、promotion、rehearsal 都成为可重复执行的客户验收工具，而不是开发者专用脚本。

执行项：

1. 修复 `tests/run_smoke_tests.py` 会污染 tracked workflow artifact 的问题。
2. 把 workflow smoke 全部改到隔离目录，例如：
   - `test_env/...`
   - 临时工作区
   - 或 builder 管理的 artifact sandbox
3. 对所有 smoke / rehearsal / readiness 工具增加：
   - `--output-root`
   - `--project-root`
   - `--read-only-source` 或等价隔离模式
4. 把“仓库必须 clean”纳入 smoke 自检，禁止验收脚本默认改写 tracked 文件。
5. 为 smoke 产物建立 retention 和目录规范，避免客户环境长期堆积运行垃圾。

交付物：

- side-effect-free smoke runner
- side-effect-free workflow smoke
- 验收产物目录规范

退出门禁：

- 连续执行两次默认 smoke，`git status --short` 仍为空
- cleanup report 不再把默认 smoke 产物识别为 tracked runtime artifact

## 阶段 C：客户部署包装

目标：把“能跑源码”升级为“可安装、可部署、可升级”。

执行项：

1. 确认唯一的一线部署路径：
   - Docker Compose
   - Kubernetes / Helm
   - 或两者都支持，但必须声明主次
2. 如果继续声明 Helm，则补齐真实 Helm chart：
   - `helm/agi-walker`
   - values 分层
   - lint / template / install 验证
3. 如果暂不支持 Helm，则从一线文档中去除 Helm 作为当前有效路径，避免误导客户。
4. 补齐客户部署矩阵：
   - 单机评估版
   - Docker 生产版
   - ROS2 / Godot 扩展节点
5. 固化 `.env`、密钥、端口、卷、日志和备份目录约定。
6. 产出正式安装文档：
   - 安装前检查
   - 首次启动
   - 健康检查
   - 升级
   - 回滚
   - 卸载

交付物：

- 真实可执行的部署包
- 可执行的部署文档
- 支持矩阵

退出门禁：

- 新环境按文档可独立拉起最小可用系统
- 升级与回滚可在演练环境复现
- 部署路径不再引用不存在的 Helm chart 或占位配置

## 阶段 D：安全、合规与运维基线

目标：补齐工业交付最低安全与 SRE 能力。

### 当前 checkpoint

- Python dependency scan：已复绿，当前 canonical 状态为 `passed`，`finding_count=0`。
- Zenoh router 交付镜像：已复绿，当前 canonical 状态为 `0 findings`。
- Web panel distributed 镜像：当前 canonical container 扫描结果已通过 Web Panel Alpine 候选与 build-cache 清理降为 `0 findings`；当前 `accepted_finding_count=0`、`unresolved_finding_count=0`、`matched_exception_count=0`、`stale_exception_count=0`。
- tracked canonical exception 输入已收口到 `deployment/security/vulnerability_exceptions.input.json`；`tools/collect_release_evidence.py` 与 `tools/run_security_release_preflight.py` 默认会从该路径生成 structured `vulnerability_exception_report.json`。
- dockerized Trivy fallback 的 `/scan/image.tar` 镜像标识问题已修复，因此镜像级 exception 现在能稳定匹配 `deployment-web-panel-distributed` 的 canonical findings。
- `security_posture_report` 当前为 `ready`，`security_release_preflight_report` 当前为 `passed`；最新合并后 main preflight 当前已显式写出 active exceptions `0`、review-due exceptions `0`、expired exceptions `0` 与 `vulnerability_exception_review_report_status=passed`；当前 readiness / stable / industrial promotion surface 也已优先指向独立 `vulnerability_exception_review_report.json`，不再只暴露 aggregate preflight metrics。最新 rc manifest 与 customer acceptance bundle 当前都已为 `ready`。

### 当前保存计划

1. 保持 `deployment/security/vulnerability_exceptions.input.json` 作为受管输入；当前受管 no-fix exceptions 为空，后续若 scanner DB 刷新重新出现 findings，应由 `security-preflight` fail closed 后再通过真实修复或新的显式审批处理。
2. `industrial_delivery_gate` 当前已进入 `release_manifest`、`release_readiness_report.json`、`stable_promotion_checklist.json` 以及独立的 `industrial_release_readiness_report.json` / `industrial_promotion_checklist.json`，并已正式收口 `deployment_package_status`、`evidence_attested`、`sbom_attached`、`vuln_scan_status`、`backup_restore_verified` 与 Phase E 文档挂载状态。
3. 最新 canonical rc manifest 上，`industrial_delivery_gate.status=ready`；容器扫描 residual findings 现已通过 `vulnerability_remediation_report.status=ready` / `security_posture_report.status=ready` 收口，不再单纯卡在原始 `container_vuln_scan_report.status=blocked`。
4. `tools/run_release_rehearsal.py` 当前已实跑通过，并会显式 seed customer delivery / security / remediation 产物，要求 `industrial_delivery_gate.status=ready`；相对或绝对 `--output-root` 路径都已验证可闭环。最新 runner 还会直接生成 `security_release_preflight_report.json`、`release_manifest_industrial.json`、`industrial_release_readiness_report.json`、`industrial_promotion_checklist.json`、`customer_acceptance_bundle_industrial.json` 与独立 `industrial_delivery_rehearsal_report.json`，并把 `new_environment_install`、`smoke`、`live_evidence`、`upgrade`、`rollback`、`backup_restore` 六个阶段写入结构化 rehearsal report。当前 `industrial_release_readiness_report` 与独立 `industrial_delivery_rehearsal_report` 也都会显式带出 `vulnerability_exception_review.status` / `review_candidate_count`，使 residual-risk review 不再只停留在 security preflight metrics。canonical industrial rehearsal 现在默认复制携带 synthetic confirmation metadata 的 `deployment/customer_delivery.external_bindings.rehearsal.json`，使演练产物中的 `extension_execution_actuals.external_bindings_status=ready`，同时保留 `deployment/customer_delivery.external_bindings.json` 作为客户实例化时的 placeholder 输入。
5. `tools/build_customer_external_bindings_config.py` 与 `tools/confirm_customer_external_bindings.py` 当前都已落地，可先从 `extension_execution_instance.json` 直接生成 customer-specific external bindings config，再显式确认哪些 section 已切到真实客户系统。生成文件默认会显式写出 `binding_state=draft`，避免把“已生成客户配置”误判成“真实客户系统绑定已完成”；而 `binding_state=confirmed` 现在还要求最少附带 `confirmed_by`、`confirmed_at` 与 `confirmation_ticket`。`tools/collect_release_evidence.py` 现在会优先读取 `deployment/customer_delivery.external_bindings.customer.json`，而 `check_release_readiness.py` / `customer_acceptance_bundle` / `stable_promotion_checklist` / `industrial_promotion_checklist` 在 external bindings 未闭合时会优先暴露新的 `tools/run_customer_external_bindings_closure.py` runner 作为 primary closure command；这条 runner 会在 customer-specific config 缺失时自动先生成 draft config，并在没有 `--set section.field=value` 或 `--overrides-file` 覆盖时阻塞退出，避免把默认占位值直接 confirm 成 ready。最新 runner 还会无论成功或阻塞都写出独立 `customer_external_bindings_closure_report.json`，把 `failed_steps`、`selected_sections` 与 config/actuals/confirmation evidence 是否已落盘固化成结构化 evidence；`check_release_readiness.py`、`stable_promotion_checklist`、`industrial_promotion_checklist` 与 `customer_acceptance_bundle` 现在都会直接回显最近一次 closure report 的 `failed_steps` 与 report 路径，便于后续 readiness / checklist / 客户交付面继续消费。`tools/confirm_customer_external_bindings.py` 现也支持 `--overrides-file`，仓内提供 `deployment/customer_delivery.external_bindings.overrides.example.json` 作为受管模板，便于把真实客户审批/归档/到期触发元数据整理成可审计 JSON 后再收口。生成后的 actuals 还会直接带出 `external_bindings_confirmed_sections`、`external_bindings_confirmed_by`、`external_bindings_confirmation_tickets` 与 `external_bindings_last_confirmed_at`，供后续 industrial gate 和客户验收面直接消费。当前还新增了独立 `customer_external_bindings_confirmation_report.json`，用于把这组确认状态固定成可挂到 acceptance reports 的结构化 evidence，而不是继续只依赖 actuals 嵌套字段。
6. `tools/build_external_mainline_execution_plan.py` 与 `tools/run_external_mainline_execution_plan.py` 当前已把剩余三条外部主线收口到统一计划面。builder 只读取已有 `customer_external_bindings_closure_report.json`、`vulnerability_exception_review_report.json` 与 canonical `industrial_delivery_rehearsal_report.json`，生成结构化 `external_mainline_execution_plan.json`；runner 则会在同一路径下先自动重建 `vulnerability_exception_review_report.json`，并可复用已通过的 customer closure evidence。当前 canonical runner 短路径已显式标记 `customer_external_bindings_closure`、`vulnerability_exception_replacement` 为 `completed`，`industrial_delivery_live_evidence` 为 `ready_to_run`，说明 managed industrial live inputs 已齐备，剩余动作是执行真实客户环境 install / upgrade / rollback / backup-restore 留痕并归档。最新 runner 仍支持 `--inputs-file`，可直接消费统一模板 `deployment/external_mainline.inputs.example.json`，把 customer overrides、confirmation metadata 和 optional industrial rehearsal refresh 参数集中到一份受管 JSON 中；同时新增的 `tools/build_external_mainline_inputs.py` 可以从已有 customer config / industrial rehearsal baseline 自动刷新 `deployment/external_mainline.inputs.json` 草稿，减少客户窗口准备期的重复手工编辑。现在默认 codepath 已会直接接管这份 managed inputs file；若只需刷新 plan/checklist 而不重新执行已闭合的客户 closure，可使用 `--skip-customer-external-bindings-closure`。如果想保留旧的“完全不使用 managed inputs”模式，则可显式传 `--skip-managed-inputs`。最新 runner 还会自动写出 `external_mainline_input_checklist_report.json`，把 customer / vulnerability / industrial 三条外部主线仍缺的真实输入压成结构化 checklist；industrial acceptance bundle、`industrial_release_readiness_report.json.summary`、smoke artifact 摘要和独立 industrial rehearsal report 现在都会直接挂出 / 透出计划面与 checklist 面，便于客户窗口继续消费剩余外部依赖步骤。
7. 正式 `industrial` release channel 当前已落地；它继承 stable gate，并额外要求 `customer_delivery_surface.status=ready` 与 `industrial_delivery_gate.status=ready`。
8. `tools/check_industrial_release_readiness.py` 与 `tools/build_industrial_promotion_checklist.py` 当前已落地，smoke 与 customer acceptance bundle 也会切到工业通道自己的 readiness / checklist 产物；最新 industrial readiness summary 会显式带出 `exception_review=<status>/<candidate_count>`、`external_mainline=<status>/<completed>/<ready_to_run>/<waiting_external_input>/<blocked>` 与 `external_mainline_input_checklist=<status>/<missing>/<waiting>/<ready>/<completed>`，并在 stdout 中直接打印 `industrial_external_mainline_input_checklist=...`。industrial promotion checklist 现在也会把同一份 external-mainline plan 与 checklist 一并收成顶层字段和 non-blocking step，industrial bundle 还会默认挂入 `industrial_delivery_rehearsal_report`。同一条链上，`release_rehearsal_report.json.industrial_customer_acceptance_bundle` 现已内联 `vulnerability_exception_review` / `external_mainline_execution_plan` / `external_mainline_input_checklist` 摘要，独立 industrial rehearsal report 会优先消费这层摘要或 bundle 顶层字段，只在缺失时才回退到 `acceptance_reports`；最新 `industrial_delivery_rehearsal_report.json.summary` 也会显式带出 `external_mainline_input_checklist=...`，避免再回 acceptance bundle 里翻剩余输入缺口。
9. `customer_delivery_surface.extension_support_surface` 当前已落地；distributed / ROS2 / Godot / Helm-Kubernetes 的支持边界、专项验收要求和非支持范围都已进入 manifest、industrial gate 和 customer acceptance bundle。
10. `extension_support_surface.profiles[*]` 当前已携带 `deployment_commands`、`acceptance_checks` 与 `rollback_prerequisites`，扩展实施动作已进入机器字段。
11. `customer_acceptance_bundle`、`industrial_promotion_checklist` 与 `release_rehearsal_report` 当前已开始携带 `extension_execution_plan`，扩展实施动作已经进入执行面摘要。
12. `extension_execution_plan.profiles[*].runbook_entrypoints` 当前已把正式 runbook、客户安装指南和 distributed / ROS2 / Godot 专项指南接进机器可读执行面。
13. `extension_execution_plan.profiles[*].execution_template` 当前已把现场角色分工、升级窗口步骤、值班动作、on-call 交接记录、残余风险交接、exception 到期复核、异常升级闭环证据和回滚证据归档责任接进机器可读执行面。
14. `release_manifest`、`customer_acceptance_bundle`、`industrial_release_readiness_report`、`industrial_promotion_checklist` 与 `release_rehearsal_report` 当前已开始携带 `extension_execution_evidence`，`extension_on_call_rehearsal`、`extension_exception_review_schedule` 与 `extension_escalation_closure` 三类留痕报告已经进入机器字段。
15. `release_manifest`、`customer_acceptance_bundle`、`industrial_release_readiness_report`、`industrial_promotion_checklist` 与 `release_rehearsal_report` 当前已开始携带 `extension_execution_instance`，客户实例化的交付窗口、exception 复核到期时间、delivery root、closure archive root 与 profile 级路径已经进入机器字段。
16. 同一批产物当前已开始携带 `extension_execution_schedule`，客户窗口触发时间、signoff 截点、residual risk review 记录和 closure archive manifest 已进入机器字段。
17. 同一批产物当前已开始携带 `extension_execution_actuals`，客户窗口实际触发时间、实际 signoff、residual risk review 实际留痕、approval identity source、archive target binding、due trigger binding 和 closure manifest 已进入机器字段。
18. 持续复跑 canonical release evidence / security preflight，确保 residual risk 的变更会被结构化产物、industrial delivery gate 和发布门禁及时反映。

执行项：

1. 建立 SBOM 产物并纳入 release artifact。
2. 建立依赖和镜像漏洞扫描门禁：
   - Python 依赖
   - Docker 镜像
3. 建立 secrets 基线：
   - `.env` 分类
   - 生产 secrets 注入方式
   - 明文密钥禁入库检查
4. 建立审计与留痕：
   - release approver
   - deployment actor
   - config change trace
   - workflow run audit
5. 建立备份和恢复最小闭环：
   - 备份对象
   - 备份频率
   - 恢复时间目标
   - 恢复演练报告
6. 建立告警和值班边界：
   - P1/P2/P3 事件级别
   - 首次响应 SLA
   - 升级路径

交付物：

- SBOM
- vuln scan report
- secret policy
- audit trail policy
- backup / restore runbook
- incident severity matrix

退出门禁：

- 每个 release artifact 都能关联 SBOM 和漏洞扫描结果
- 至少完成一次备份恢复演练并留存报告
- 生产配置不依赖手工口头密钥分发

## 阶段 E：客户交付包与支持体系

目标：把项目从“技术团队可接手”推进到“客户团队可使用”。

执行项：

1. 建立客户版本文档集：
   - 安装指南
   - 运维手册
   - 故障排查
   - 常见限制
   - 升级说明
2. 建立支持矩阵：
   - 支持的 OS
   - Python 版本
   - Docker 版本
   - ROS2 版本
   - Godot 版本
   - 浏览器范围
3. 建立容量与规模声明：
   - 单机
   - 小规模团队
   - 分布式实验环境
4. 建立客户验收清单：
   - 功能验收
   - 部署验收
   - 安全验收
   - 性能验收
   - 回滚验收
5. 建立已知限制和不支持项文档，避免“暗支持”。

交付物：

- 客户交付文档包
- 版本支持矩阵
- 客户验收 checklist

退出门禁：

- 非仓库开发者可按文档完成安装和基础验收
- 客户支持边界有正式文档，不再依赖 issue / chat 历史

## 阶段 F：工业化验收发布

目标：把 strict industrial delivery 变成一个新的 release channel 门禁，而不是一次性项目。

执行项：

1. 为 `release_manifest` 增加工业化交付字段：
   - `deployment_package_status`
   - `evidence_attested`
   - `sbom_attached`
   - `vuln_scan_status`
   - `backup_restore_verified`
   - `support_matrix_attached`
   - 当前状态：已完成，字段现已收口到 `industrial_delivery_gate`
2. 为 `stable` 之上新增 `industrial` 或等价交付级别 gate。
   - 当前状态：已完成，`build_release_artifact.py --channel industrial` 已成为正式门禁入口
3. 把工业化门禁接入：
   - readiness
   - promotion checklist
   - rehearsal
   - 当前状态：stable readiness / checklist / rehearsal 已消费 `industrial_delivery_gate`，并且独立 `industrial_release_readiness` / `industrial_promotion_checklist` 也已落地
4. 产出一次完整“工业交付演练”：
   - 新环境安装
   - smoke
   - live evidence
   - 升级
   - 回滚
   - 备份恢复

交付物：

- 工业化 release gate
- 工业化交付演练报告
- 客户验收版 release manifest

退出门禁：

- 新 gate 状态达到 `ready`
- 工业化演练报告完整归档
- 客户交付 checklist 全部闭合

## 推荐顺序与周期

建议周期：`10-14 周`

推荐顺序：

1. 阶段 A：`1-2 周`
2. 阶段 B：`1-2 周`
3. 阶段 C：`2-3 周`
4. 阶段 D：`3-4 周`
5. 阶段 E：`2-3 周`
6. 阶段 F：`1 周`

不建议并行的部分：

- 阶段 A 未闭合前，不要开始对外宣称工业化交付。
- 阶段 B 未闭合前，不要把 smoke 当客户验收工具。
- 阶段 C 未闭合前，不要把生产 runbook 当正式交付文档。

## 当前优先级

P0：

1. 把 `execution_template` 继续细化到真实值班演练 evidence、exception 到期排程、升级 closure artifact 收集以及外部 binding reference。
   - 当前状态：已完成到 default binding reference 层，canonical 留痕报告现已由 `tools/build_extension_execution_evidence.py` 生成，并进入 manifest / bundle / readiness / checklist / rehearsal；客户实例化窗口已由 `tools/build_extension_execution_instance.py` 落盘，窗口触发 / signoff / closure archive 目标已由 `tools/build_extension_execution_schedule.py` 落盘，客户审批来源 / 归档目标 / 到期触发则已由 `tools/build_extension_execution_actuals.py` 以 `customer_ticket_registry`、`customer_archive_destination` 和 `customer_due_trigger_schedule` 形式写入机器字段。
2. 持续保持 canonical release manifest、release evidence 与 security preflight 的闭环复算，并同步当前状态文档口径。
3. 持续复跑 canonical security evidence；当前 no-fix exceptions 已退休，后续若 scanner DB 刷新重新出现 findings，应由 security-preflight fail closed 后再处理，避免 residual risk 重新变成长尾常态。

P1：

1. 把 Phase E 文档继续接入 industrial channel 的后续 promotion/release surface。
2. 为 industrial channel 继续保留容量声明和支持边界的机器可读字段。
3. 在独立 industrial readiness / promotion flow 稳定后，推进完整工业交付演练。

P2：

1. 增加工业化交付 gate。
2. 产出完整工业交付演练报告。
3. 将客户交付 checklist 与 evidence attestation 绑定到 release manifest。

## 完成判定

只有以下条件同时满足，才能认为 AGI-Walker 达到“严格工业化交付完成度”：

1. 最新 release evidence 全部自动采集且可复算。
2. 默认验收链路无副作用。
3. 客户可按文档独立部署、升级、回滚和排障。
4. 安全、合规、审计、备份恢复有正式产物。
5. release gate 中存在独立的工业化交付门禁，并通过一次完整交付演练。
