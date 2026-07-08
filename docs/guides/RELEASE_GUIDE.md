# AGI-Walker Release Guide

更新日期：`2026-04-16`

本页是当前有效的发布门禁说明。历史 release checklist 和 Git release 备忘仍保留在 `docs/archive_and_reports/`，但不应再作为一线发布手册使用。

## 发布产物

阶段五当前固定的发布产物是 `release_manifest`：

- `schema_version=1.0`
- `artifact_type=release_manifest`
- 包含 `build_id`
- 包含 `version`
- 包含 `channel`
- 包含 `release_policy`
- 包含 `release_approval`
- 包含 `release_source`
- 包含 `release_summary`
- 包含 `contract_versions`
- 包含 `capability_matrix`
- 包含 `test_evidence`
- 包含 `known_limitations`
- 包含 `customer_delivery_surface`
- 包含 `industrial_delivery_gate`
- 包含 `release_ops_execution`
- 包含 `control_plane_surface`
- 包含 `extension_execution_evidence`
- 包含 `extension_execution_instance`
- 包含 `extension_execution_schedule`
- 包含 `extension_execution_actuals`

阶段六当前新增的客户验收产物是 `customer_acceptance_bundle`：

- `schema_version=1.0`
- `artifact_type=customer_acceptance_bundle`
- 包含 `bundle_status`
- 包含 `release_manifest`
- 包含 `known_limitations`
- 包含 `extension_execution_plan`
- 包含 `extension_execution_evidence`
- 包含 `extension_execution_instance`
- 包含 `extension_execution_schedule`
- 包含 `extension_execution_actuals`
- 包含 `required_evidence`
- 包含 `optional_evidence`
- 包含 `acceptance_documents`
- 包含 `acceptance_reports`
- 包含 `recommended_commands`

当前生成命令：

```bash
python tools/build_release_artifact.py --version 2026.04.12-rc6 --channel rc --build-id build-20260412-006
```

默认输出：

```text
test_env/release/release_manifest.json
```

`build_release_artifact.py` 现在会自动尝试读取以下默认证据文件：

- `test_env/release_evidence/clean_checkout_smoke_report.json`
- `test_env/release_evidence/non_live_gate_report.json`
- `test_env/release_evidence/release_contracts_and_capability_matrix_report.json`
- `test_env/distributed_smoke/distributed_smoke_report.json`
- `test_env/godot_headless_smoke/headless_smoke_report.json`
- `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json`

如果这些文件存在，builder 会自动把对应 evidence 状态聚合进 `release_manifest`。如果你要从别的目录读取证据，可以显式传入：

```bash
python tools/build_release_artifact.py --version 2026.04.12-rc1 --channel rc --build-id build-20260412-001 --release-summary "阶段五发布门禁闭环。" --project-root D:/tmp/release_evidence_root --distributed-report D:/tmp/release_evidence_root/test_env/distributed_smoke/distributed_smoke_report.json
```

最新 manifest-facing surface 还会继续读取 canonical `test_env/release_evidence/operations/release_ops_execution_report.json`。如果该 wrapper 存在，`release_manifest.customer_delivery_surface` 会直接带出 `release_ops_execution.status` / `event_count`，并继续聚合 `control_plane_session` / `control_plane_event_stream`；`release_manifest.industrial_delivery_gate` 也会复用这组已归一化字段，把同一份 `release_ops_execution` / control-plane 摘要继续提升到 industrial gate；而 `release_manifest` 顶层现在也会直接附带 `release_ops_execution` 与聚合后的 control-plane 摘要，并把它们再收口成统一 `control_plane_surface`。`tools/build_release_artifact.py` 的 stdout 现在会同时显式打印 `customer_delivery_release_ops_execution=...`、`industrial_delivery_release_ops_execution=...`、`release_manifest_release_ops_execution=...`、`release_manifest_control_plane_events=...` 与 `release_manifest_control_plane_surface=...`，因此 control-plane wrapper 不再只能等到 acceptance / rehearsal 阶段才可见。

当前建议先运行一轮统一 evidence 采集，再生成 manifest：

```bash
python tools/run_clean_checkout_smoke.py --output-root test_env/clean_checkout_smoke_real
python tools/collect_release_evidence.py --output-root test_env/release_evidence --vulnerability-exception-input-source deployment/security/vulnerability_exceptions.input.json
python tools/run_security_release_preflight.py --security-only --output-root test_env/release_evidence --run-python-vuln-scan --run-container-vuln-scan --container-image-ref deployment-zenoh-router --container-image-ref deployment-web-panel-distributed --vulnerability-exception-input-source deployment/security/vulnerability_exceptions.input.json
python tools/build_extension_execution_evidence.py --output-root test_env/release_evidence/operations --vulnerability-exception-report test_env/release_evidence/security/vulnerability_exception_report.json
python tools/build_extension_execution_instance.py --output test_env/release_evidence/operations/extension_execution_instance.json --vulnerability-exception-report test_env/release_evidence/security/vulnerability_exception_report.json
python tools/build_extension_execution_schedule.py --output test_env/release_evidence/operations/extension_execution_schedule.json --instance-artifact test_env/release_evidence/operations/extension_execution_instance.json
python tools/build_extension_execution_actuals.py --output test_env/release_evidence/operations/extension_execution_actuals.json --schedule-artifact test_env/release_evidence/operations/extension_execution_schedule.json --external-bindings-config deployment/customer_delivery.external_bindings.json
python tools/build_release_artifact.py --version 2026.04.15-rc-evidence --channel rc --build-id build-20260415-security-preflight --release-summary "Phase D security preflight closed with canonical no-fix exceptions for deployment-web-panel-distributed." --output test_env/release/release_manifest_rc_evidence.json
python tools/build_customer_acceptance_bundle.py --manifest test_env/release/release_manifest_rc_evidence.json --output test_env/release/customer_acceptance_bundle_rc_evidence.json
```

`tools/collect_release_evidence.py` 现在还会默认生成 `extension_execution_evidence`、`extension_execution_instance`、`extension_execution_schedule`、`extension_execution_actuals`、独立的 `customer_external_bindings_confirmation_report`，以及 `vulnerability_exception_review_report`。后者会把 review-due / expired exception 列表、follow-up 是否必需、以及最早到期时间固化成结构化 release evidence，供 `run_security_release_preflight.py` 和 `customer_acceptance_bundle` 直接消费。如果仓库中存在受管输入 `deployment/customer_delivery.external_bindings.json`，它会自动把这份配置接到 actuals 生成链，并在 actuals 之后追加 external bindings confirmation report。最新 collector 还支持 `--release-ops-execution-report-source`，用于把已有 `release_ops_execution_report.json` 复制进 canonical `test_env/release_evidence/operations/`；它只负责收集已有 wrapper，不会绕过 `release_ops` 的 `policy_profile` 边界主动执行高风险 action。

如果本次客户窗口已经绑定到外部审批系统、归档目标或到期触发调度，建议复制 `deployment/customer_delivery.external_bindings.example.json` 为客户实例文件，并在生成 actuals 时追加 `--external-bindings-config <path>`，这样 `extension_execution_actuals`、`approval_identity_source.json`、`archive_target.json` 和 `due_trigger_check.json` 会显式记录本次使用的绑定配置来源。当前仓内默认 `deployment/customer_delivery.external_bindings.json` 仍是占位输入，因此生成的 actuals 会带 `external_bindings_status=placeholder`；只有把 `approval_identity.reference/source_path`、`archive_target.binding_reference_base` 和 `due_trigger.binding_reference_base` 换成真实客户系统映射，并附带 `binding_state=confirmed`、`confirmed_by`、`confirmed_at` 与 `confirmation_ticket` 这组确认留痕后，这组字段才会收口到 `external_bindings_status=ready`。为了让 canonical industrial rehearsal 自带一份闭环的 ready 配置，`tools/run_release_rehearsal.py` 现在默认复制并使用 `deployment/customer_delivery.external_bindings.rehearsal.json`；这份文件已经带有演练用的 synthetic confirmation metadata，只用于演练，不替代真实客户窗口配置。真实客户窗口现在也可以先从 `extension_execution_instance.json` 生成一份 customer-specific 配置，再回填真实系统信息：

```bash
python tools/build_customer_external_bindings_config.py --output deployment/customer_delivery.external_bindings.customer.json --instance-artifact test_env/release_evidence/operations/extension_execution_instance.json
python tools/confirm_customer_external_bindings.py --config deployment/customer_delivery.external_bindings.customer.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <confirmed-by> --confirmation-ticket <confirmation-ticket>
python tools/build_extension_execution_actuals.py --output test_env/release_evidence/operations/extension_execution_actuals.json --schedule-artifact test_env/release_evidence/operations/extension_execution_schedule.json --external-bindings-config deployment/customer_delivery.external_bindings.customer.json
python tools/build_customer_external_bindings_confirmation_report.py --output test_env/release_evidence/operations/customer_external_bindings_confirmation_report.json --actuals-artifact test_env/release_evidence/operations/extension_execution_actuals.json
```

如果你不想再手工串这 4 步，当前也可以直接使用受管 runner：

```bash
python tools/run_customer_external_bindings_closure.py --config deployment/customer_delivery.external_bindings.customer.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <confirmed-by> --confirmation-ticket <confirmation-ticket>
```

这条命令会在 customer config 缺失时先生成 draft config，然后继续执行 confirm、重建 `extension_execution_actuals`、生成 `customer_external_bindings_confirmation_report.json`，最后刷新 `collect_release_evidence.py`。如果 config 是首次生成而你还没有补齐真实客户字段，runner 不会直接把默认占位值 confirm 成 ready；它会先写出 draft config 并阻塞退出，要求你通过编辑文件、补充 `--set section.field=value`，或者提供 `--overrides-file <path>` 后再重跑。
最新 runner 还会无论成功或阻塞都写出 `test_env/release_evidence/operations/customer_external_bindings_closure_report.json`，以结构化 `release_evidence_report` 形式固化本次 closure chain 的 `failed_steps`、`selected_sections`、config/actuals/confirmation report 是否已落盘，以及是否执行了 `collect_release_evidence.py`。`check_release_readiness.py`、`build_stable_promotion_checklist.py` 与 `build_industrial_promotion_checklist.py` 现在会直接读取最近一次 closure report，把 `failed_steps` 与 report 路径回显到 external bindings follow-up / checklist summary 中，避免操作面继续只依赖 runner stdout；`customer_acceptance_bundle` 也会把这份 closure report 作为 optional acceptance report 一并挂出。

如果现场更适合把真实客户映射整理成一份受管 JSON，而不是在命令行里写很多 `--set`，可以先复制仓内模板 `deployment/customer_delivery.external_bindings.overrides.example.json`，然后执行：

```bash
python tools/run_customer_external_bindings_closure.py --config deployment/customer_delivery.external_bindings.customer.json --overrides-file deployment/customer_delivery.external_bindings.customer.overrides.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <confirmed-by> --confirmation-ticket <confirmation-ticket>
```

`--overrides-file` 会把 JSON 中的 `section -> field -> value` 覆盖应用到 config 后再 confirm；如果同一条命令还带了 `--set`，则 `--set` 优先级更高。

如果你想先按最小待填项梳理真实客户字段，而不是直接看长篇发布指南，可直接使用：

- `docs/guides/CUSTOMER_EXTERNAL_BINDINGS_CHECKLIST.md`

如果你不想再分别判断 external bindings、vulnerability exception replacement 和真实 industrial 环境留痕三条外部主线，仓内现在还有统一计划面：

```bash
python tools/build_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json
python tools/run_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json
```

第一条命令只读取现有 closure / review / rehearsal baseline，生成结构化 `external_mainline_execution_plan.json`；第二条命令则会先自动刷新可安全执行的部分，再回写同一份 plan。当前 runner 默认会重建 `vulnerability_exception_review_report.json`；如果你同时提供 customer confirmation 参数，它还会继续执行 `run_customer_external_bindings_closure.py`。在没有真实客户 overrides、`confirmed_by` / `confirmation_ticket` 或真实 industrial 环境入口时，runner 会把对应 step 明确标成 `waiting_external_input`，而不是隐式失败；如果默认的 `deployment/external_mainline.inputs.json` 缺失，它现在也会先自动调用 `tools/build_external_mainline_inputs.py` bootstrap 这份 managed inputs file，再在后续执行前自动 refresh 这份默认草稿。因此默认 codepath 已不再需要手工写出 `--inputs-file`；如果你明确想保留旧的“完全不使用 managed inputs”模式，可以额外传 `--skip-managed-inputs`。

当前还新增了一条只做输入缺口盘点的结构化入口：

```bash
python tools/build_external_mainline_input_checklist.py --output test_env/release_evidence/operations/external_mainline_input_checklist_report.json
```

这条命令会读取 `deployment/external_mainline.inputs.json` 和当前 external-mainline plan，把 customer bindings、vulnerability exception replacement、industrial live evidence 仍缺的真实输入压成一份 `release_evidence_report`。`run_external_mainline_execution_plan.py` 现在也会在每次执行后自动写出同一份 checklist，因此你既可以单独查看缺口，也可以直接依赖 runner 的自动产物。

如果你不想再把这些参数散落在命令行里，仓内现在还提供一份统一输入模板 `deployment/external_mainline.inputs.example.json`。复制并填充后，可以直接运行：

```bash
python tools/run_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json
```

当前 `inputs-file` 已支持：

- `customer_external_bindings.config / overrides_file / sections / confirmed_by / confirmation_ticket / skip_collect_release_evidence`
- `vulnerability_exception_review.report_output`
- `industrial_rehearsal.refresh / version / build_id / output_root / report_path`
- `industrial_live_evidence.target_environment / access_method / install_entrypoint / upgrade_entrypoint / rollback_entrypoint / backup_restore_entrypoint / closure_archive_root / evidence_output_root`

这意味着后续 external-mainline 的常规执行可以被收口成“一份 JSON + 一条 runner 命令”，而不需要反复手工拼接参数。runner 还会忽略 `<confirmed-by>` / `<confirmation-ticket>` 这类 placeholder 确认值，因此自动 bootstrap / refresh 出来的默认草稿不会误触发 customer closure；同样地，`industrial_live_evidence` 段里的 placeholder 也会让 `industrial_delivery_live_evidence` step 继续停留在 `waiting_external_input`。只有当真实环境标识、访问方式、install/upgrade/rollback/backup-restore 入口和 archive 根目录都补齐后，这个 step 才会在 plan 中提升为 `ready_to_run`。最新 runner stdout 还会显式打印 `external_mainline_execution_plan_industrial_live_evidence_inputs_ready=<true|false>`，便于快速判断这组现场输入是否已经补齐。

如果你连这份 JSON 也不想手工起草，当前还可以先自动生成/刷新仓内草稿：

```bash
python tools/build_external_mainline_inputs.py --output deployment/external_mainline.inputs.json
```

这个 builder 会尽量复用已有 `deployment/customer_delivery.external_bindings.customer.json`、`customer_external_bindings_closure_report.json` 和 canonical `industrial_delivery_rehearsal_report.json` 回填默认值，并保留已经写进 `deployment/external_mainline.inputs.json` 的真实字段，因此更适合在客户窗口准备期反复刷新草稿，而不是每次都从 example 手工复制。

`tools/build_customer_external_bindings_config.py` 当前默认会把各 section 的 `binding_state` 生成为 `draft`，并附带 integration notes，因此这份 customer-specific 配置在补齐真实客户审批系统、archive destination 和 due-trigger 元数据之前，仍会让 actuals 保持 `external_bindings_status=placeholder`。补齐真实元数据后，使用 `tools/confirm_customer_external_bindings.py` 把相关 section 收口到 `binding_state=confirmed`；`confirmed` 现在不再只是状态字面值，还要求最少携带 `confirmed_by`、`confirmed_at` 和 `confirmation_ticket`。只有完成这一步之后，这组字段才会真正收口到 `external_bindings_status=ready`。

`extension_execution_actuals` 现在会直接暴露 `external_bindings_confirmed_sections`、`external_bindings_confirmed_by`、`external_bindings_confirmation_tickets` 和 `external_bindings_last_confirmed_at`，因此 readiness / bundle / rehearsal 下游不需要再回头解析整份 customer config，就能看出哪些 section 已被客户窗口正式确认。除此之外，`tools/build_customer_external_bindings_confirmation_report.py` 还会把这组状态固化成独立的 `customer_external_bindings_confirmation_report.json`，供 `customer_acceptance_bundle` 和后续客户交付面直接挂载，而不是继续只依赖 nested actuals 字段。

如果 `deployment/customer_delivery.external_bindings.customer.json` 已经存在，`tools/collect_release_evidence.py` 现在会优先使用它；只有没有 customer-specific 配置时，才回落到默认 placeholder `deployment/customer_delivery.external_bindings.json`。

其中 `tools/collect_release_evidence.py` 现在还会生成并收集：

- `test_env/release_evidence/security/sbom.json`
- `test_env/release_evidence/security/backup_restore_rehearsal_report.json`
- `test_env/release_evidence/security/security_posture_report.json`

默认情况下，`tools/collect_release_evidence.py` 与 `tools/run_security_release_preflight.py` 都会先读取受管输入 `deployment/security/vulnerability_exceptions.input.json`，并自动生成 canonical `vulnerability_exception_report.json`；只有在需要单独重建 report 或导入外部审批结果时，才显式传入 `--vulnerability-exception-report-source`。

如果你已经在外部 scanner 中生成了结构化漏洞报告，可以追加：

```bash
python tools/collect_release_evidence.py --python-vuln-report-source D:/path/python_vuln_scan_report.json --container-vuln-report-source D:/path/container_vuln_scan_report.json --vulnerability-exception-input-source D:/path/vulnerability_exceptions.input.json
```

如果外部系统已经直接产出结构化 `vulnerability_exception_report.json`，也可以改用 `--vulnerability-exception-report-source D:/path/vulnerability_exception_report.json`。

如果你只有 scanner 原始 JSON，而不是已经规范化的 contract，也可以直接交给 collector：

```bash
python tools/collect_release_evidence.py --python-vuln-raw-report D:/scanner/pip_audit.json --python-vuln-raw-format pip-audit-json --python-vuln-command "pip-audit --format json" --container-vuln-raw-report D:/scanner/trivy.json --container-vuln-raw-format trivy-json --container-vuln-command "trivy image --format json deployment-zenoh-router"
```

如果目标环境已安装 scanner，并且你希望 collector 直接执行扫描而不是只导入外部 JSON，可使用：

```bash
python tools/collect_release_evidence.py --run-python-vuln-scan --run-container-vuln-scan --container-image-ref deployment-zenoh-router
```

也可以单独运行两条 scanner wrapper：

```bash
python tools/run_python_vulnerability_scan.py --output test_env/security/python_vuln_scan_report.json
python tools/run_container_vulnerability_scan.py --image-ref deployment-zenoh-router --output test_env/security/container_vuln_scan_report.json
```

默认的 Python scanner 现在会先生成一份 aligned requirements snapshot，只覆盖当前客户交付面：

- `project.dependencies`
- `deployment/requirements.web_panel*.txt`
- `deployment/requirements.distributed_runtime.txt`

它不会默认把 `dev`、`training`、`hardware` optional groups 混进 canonical release evidence。

如果要单独评估训练或硬件扩展面，显式追加：

```bash
python tools/run_python_vulnerability_scan.py --include-optional-group training --include-optional-group hardware --output test_env/security/python_vuln_scan_report.json
```

如果你希望把这条扫描链作为正式发布前检查，而不是手工拼 `collect_release_evidence.py` 参数，可直接运行：

```bash
python tools/run_security_release_preflight.py --security-only --output-root test_env/release_evidence --run-python-vuln-scan --run-container-vuln-scan --container-image-ref deployment-zenoh-router --container-image-ref deployment-web-panel-distributed --vulnerability-exception-input-source deployment/security/vulnerability_exceptions.input.json
```

该命令会：

- 调用 `tools/collect_release_evidence.py`
- 生成或刷新 canonical security evidence
- 读取 `security_posture_report.json`
- 写出 `security_release_preflight_report.json`
- 仅在 `security_posture_status=ready` 时返回成功

如果未提供真实结构化漏洞报告，collector 仍会成功写出 canonical evidence，但 `security_posture_report` 会如实保持 `blocked`。
如果剩余 findings 需要按 no-fix residual risk 进入发布审查，应通过 `vulnerability_exception_report` 显式记录审批信息、到期时间和匹配范围，而不是直接篡改原始 scanner 输出。

建议在目标发布检出上执行，最好保持 Git worktree clean。否则 smoke 中的 stable 相关检查会按当前真实工作区状态报告 `blocked`，这是预期行为，不应再用硬编码摘要掩盖。

如果 `--release-summary` 未显式传入，builder 会尝试从 changelog 中提取：

- 第一行 `# ...` 作为 `changelog.title`
- `发布摘要：...` 或 `Release Summary: ...` 作为 `release_summary`

这意味着一线发布流程只需要先更新 `RELEASE_NOTES.md`，再执行 builder；只有需要临时覆盖时才传 `--release-summary` 或 `--changelog-title`。

## 发布前检查

1. 确认 `README.md`、`docs/CURRENT_STATUS.md` 和本页仍代表当前真实入口。
2. 运行 targeted tests 覆盖本次改动区域。
3. 运行默认 smoke：

```bash
python tests/run_smoke_tests.py
```

4. 对客户验收或工业化交付，运行 clean-checkout smoke 证明默认 smoke 无副作用：

```bash
python tools/run_clean_checkout_smoke.py --output-root test_env/clean_checkout_smoke_real
```

5. 采集结构化 release evidence：

```bash
python tools/collect_release_evidence.py --output-root test_env/release_evidence --vulnerability-exception-input-source deployment/security/vulnerability_exceptions.input.json
```

6. 在目标发布检出上运行正式 security release preflight：

```bash
python tools/run_security_release_preflight.py --security-only --output-root test_env/release_evidence --run-python-vuln-scan --run-container-vuln-scan --container-image-ref deployment-zenoh-router --container-image-ref deployment-web-panel-distributed --vulnerability-exception-input-source deployment/security/vulnerability_exceptions.input.json
```

7. 生成扩展执行留痕报告：

```bash
python tools/build_extension_execution_evidence.py --output-root test_env/release_evidence/operations --vulnerability-exception-report test_env/release_evidence/security/vulnerability_exception_report.json
```

默认会写出：

- `test_env/release_evidence/operations/extension_on_call_rehearsal_report.json`
- `test_env/release_evidence/operations/extension_exception_review_schedule_report.json`
- `test_env/release_evidence/operations/extension_escalation_closure_report.json`
- `test_env/release_evidence/operations/customer_external_bindings_confirmation_report.json`

8. 生成客户实例化扩展执行面：

```bash
python tools/build_extension_execution_instance.py --output test_env/release_evidence/operations/extension_execution_instance.json --vulnerability-exception-report test_env/release_evidence/security/vulnerability_exception_report.json
```

默认会写出：

- `test_env/release_evidence/operations/extension_execution_instance.json`

9. 生成客户窗口排程与 closure archive 面：

```bash
python tools/build_extension_execution_schedule.py --output test_env/release_evidence/operations/extension_execution_schedule.json --instance-artifact test_env/release_evidence/operations/extension_execution_instance.json
```

默认会写出：

- `test_env/release_evidence/operations/extension_execution_schedule.json`

10. 生成客户窗口执行留痕与 closure manifest：

```bash
python tools/build_extension_execution_actuals.py --output test_env/release_evidence/operations/extension_execution_actuals.json --schedule-artifact test_env/release_evidence/operations/extension_execution_schedule.json
```

默认会写出：

- `test_env/release_evidence/operations/extension_execution_actuals.json`
- `test_env/release_delivery/<engagement>/distributed_profile/window_trigger.json`
- `test_env/release_delivery/<engagement>/distributed_profile/signoff.json`
- `test_env/release_delivery/<engagement>/distributed_profile/residual_risk_review.json`
- `test_env/release_delivery/<engagement>/distributed_profile/closure_manifest.json`

11. 运行默认非 live 门禁：

```bash
python -m pytest -m "not live" -q
```

12. 如本次发布涉及 live 环境声明，再单独收集对应证据：

```bash
python tests/run_distributed_smoke.py --build --stop-after --report-file test_env/distributed_smoke/distributed_smoke_report.json
AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv
AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1 python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv
```

13. 生成客户验收包：

```bash
python tools/build_customer_acceptance_bundle.py --manifest test_env/release/release_manifest.json
```

默认输出：

```text
test_env/release/customer_acceptance_bundle.json
```

12. 阶段 D 安全基线自检：

```bash
python tools/build_sbom_artifact.py --output test_env/security/sbom.json
python tools/write_vulnerability_scan_report.py --scan-name python_dependencies --target pyproject.toml --status passed --scanner manual-review --summary "Python dependency review completed." --command "manual placeholder" --output test_env/security/python_vuln_scan_report.json
python tools/write_vulnerability_scan_report.py --scan-name container_images --target deployment/docker-compose.yml --status passed --scanner manual-review --summary "Container image review completed." --command "manual placeholder" --output test_env/security/container_vuln_scan_report.json
python tools/run_python_vulnerability_scan.py --output test_env/security/python_vuln_scan_report.json
python tools/run_container_vulnerability_scan.py --image-ref deployment-zenoh-router --output test_env/security/container_vuln_scan_report.json
python tools/build_vulnerability_exception_report.py --input deployment/security/vulnerability_exceptions.input.json --output test_env/security/vulnerability_exception_report.json
python tools/build_vulnerability_remediation_report.py --python-vuln-report test_env/security/python_vuln_scan_report.json --container-vuln-report test_env/security/container_vuln_scan_report.json --vulnerability-exception-report test_env/security/vulnerability_exception_report.json --output test_env/security/vulnerability_remediation_report.json
python tools/run_security_release_preflight.py --skip-collect --security-posture-report test_env/security/security_posture_report.json --report-file test_env/security/security_release_preflight_report.json
python tools/run_backup_restore_rehearsal.py --output-root test_env/security/backup_restore_rehearsal --report-file test_env/security/backup_restore_rehearsal_report.json
python tools/build_security_posture_report.py --vulnerability-remediation-report test_env/security/vulnerability_remediation_report.json --vulnerability-exception-report test_env/security/vulnerability_exception_report.json --output test_env/security/security_posture_report.json
```

## capability matrix

发布面矩阵当前通过两条稳定入口公开：

- Web：`GET /api/capabilities/matrix`
- MCP：`capability_matrix_get`

当前矩阵覆盖的发布面：

- `cli`
- `web_panel`
- `mcp`
- `distributed_runtime`
- `godot_integration`

`distributed_runtime` 默认基线仍从 `diagnostic_ready` 起步；一旦 `test_env/distributed_smoke/distributed_smoke_report.json` 存在且状态为 `passed`，release builder 会自动把该 domain 提升为 `ready`。

## test evidence 口径

`release_manifest` 中的 `test_evidence` 现在使用这三类状态：

- `passed`
- `blocked`
- `opt_in`

当前默认语义：

- `passed`：默认 smoke、默认非 live 门禁、release/capability targeted tests 已通过。
- `blocked`：当前环境存在明确外部阻塞，或真实 live smoke 报告显示失败。
- `opt_in`：测试需要显式环境准备，不进入默认非 live 门禁；如果结构化 live report 已存在且状态为 `passed`，builder 会自动把该证据提升为 `passed`。

required evidence 现在建议通过结构化 report 提供，而不是继续依赖硬编码摘要。当前固定路径为：

- `test_env/release_evidence/clean_checkout_smoke_report.json`
- `test_env/release_evidence/non_live_gate_report.json`
- `test_env/release_evidence/release_contracts_and_capability_matrix_report.json`

其中：

- `tests/run_smoke_tests.py` 已支持 `--report-file`
- `tools/run_clean_checkout_smoke.py` 会在目标 clean checkout 上执行 2 次连续 smoke，并写出 `clean_checkout_smoke_report.json`
- `tools/write_pytest_evidence_report.py` 可把任意 pytest 命令写成结构化 evidence
- `tools/collect_release_evidence.py` 会一次性收集 smoke、non-live gate 和 targeted release tests

当前建议把 `smoke_runner_report.json` 视为开发态本地信号，而不是 canonical required release evidence。正式发布和客户验收应以 `clean_checkout_smoke_report.json` 为准。

## customer acceptance bundle

`customer_acceptance_bundle` 用于把当前对客户有意义的交付入口收口成一份结构化产物，而不是让客户或交付团队自己拼接多个 JSON 和文档路径。

当前 builder：

```bash
python tools/build_customer_acceptance_bundle.py --manifest test_env/release/release_manifest_stable.json
```

默认会汇总：

- release manifest 路径和 gate 状态
- release manifest 当前 `known_limitations`
- required / optional evidence 当前状态
- 当前 bundle codepath 已把 `vulnerability_exception_review`、`customer_external_bindings_closure`、`external_mainline_execution_plan`、`external_mainline_input_checklist` 与 `release_ops_execution` 纳入 canonical acceptance reports；在当前 schema 下重建时，这组 acceptance reports 会同时包含 `security_posture_report`、`vulnerability_exception_report`、`vulnerability_exception_review`、`customer_external_bindings_closure`、`customer_external_bindings_confirmation`、`external_mainline_execution_plan`、`external_mainline_input_checklist` 与 `release_ops_execution`。最新 bundle 顶层现在也会直接透出 `vulnerability_exception_review.status`、`external_mainline_execution_plan.status`、`external_mainline_input_checklist.status`、`release_ops_execution.status`，以及聚合后的 `control_plane_session` / `control_plane_event_stream`；CLI stdout 会显式打印 `customer_acceptance_bundle_vulnerability_exception_review=...`、`customer_acceptance_bundle_external_mainline_execution_plan=...`、`customer_acceptance_bundle_external_mainline_input_checklist=...`、`customer_acceptance_bundle_release_ops_execution=...` 与 `customer_acceptance_bundle_control_plane_events=...`。同时，`release_rehearsal_report.json` 内联的 `industrial_customer_acceptance_bundle` 摘要现在也会一并携带 checklist 与 `release_ops_execution` 摘要，只在缺失时才回退到 `acceptance_reports`。`release_ops` 以 `external_mainline_execution` 运行时，还会把统一 control-plane 的 `control_plane_session` / `control_plane_event_stream` 回写到 `external_mainline_execution_plan.json` 与 `external_mainline_input_checklist_report.json`；若显式传入 `--evidence-report-file`，control plane 还会额外写出一份标准 `release_evidence_report`，其中 `evidence_name=release_ops_execution`，并固化 `action`、`policy_level`、`policy_profile`、`request_type`、`output_path`、`event_count` 与同一组 delivery-session 摘要。现在这份 `release_ops_execution` wrapper 也会继续进入 customer acceptance / rehearsal / industrial delivery downstream surface，而不再只存在于 CLI 临时输出。`release_readiness_report.json` / `industrial_release_readiness_report.json` 现在也会把 `external_mainline_input_checklist` 提升成顶层字段，并在 stdout 里显式打印 `stable_external_mainline_input_checklist=...` / `industrial_external_mainline_input_checklist=...`；`stable_promotion_checklist.json` 与 `industrial_promotion_checklist.json` 则会新增同名 non-blocking step。`release_readiness_report.json.summary` / `stable_promotion_checklist.json.summary` 的 stable surface、`industrial_release_readiness_report.json.summary`、`industrial_promotion_checklist.json.summary`，以及 smoke artifact 摘要中的 readiness / checklist / bundle / rehearsal / `worktree_release_blocker_report.json` 路径后缀，也都会直接拼出 `exception_review=...`、`external_mainline=...`、`external_mainline_input_checklist=...`、`worktree=...` 与 `control_plane_events=...`，便于快速判断剩余外部主线、当前 worktree 阻塞以及本次 control-plane 现场执行是否已稳定落盘。
- 当前要求保留的一线文档：
  - `README.md`
  - `docs/CURRENT_STATUS.md`
  - `docs/guides/RELEASE_GUIDE.md`
  - `docs/guides/DEPLOYMENT_MATRIX.md`
  - `docs/guides/CUSTOMER_INSTALLATION_GUIDE.md`
  - `docs/guides/SUPPORT_MATRIX.md`
  - `docs/guides/CAPACITY_AND_SCALE.md`
  - `docs/guides/CUSTOMER_ACCEPTANCE_CHECKLIST.md`
  - `docs/guides/KNOWN_LIMITATIONS.md`
  - `PRODUCTION_DEPLOYMENT_RUNBOOK.md`
  - `docs/guides/SECURITY_BASELINE.md`
  - `docs/guides/AUDIT_TRAIL_POLICY.md`
  - `docs/guides/BACKUP_RESTORE_RUNBOOK.md`
  - `docs/guides/INCIDENT_RESPONSE_MATRIX.md`
- 默认 acceptance reports：
  - `test_env/release_readiness_ready/release_readiness_report.json`
- canonical security reports：
  - `test_env/release_evidence/security/security_posture_report.json`
  - `test_env/release_evidence/security/sbom.json`
  - `test_env/release_evidence/security/python_vuln_scan_report.json`
  - `test_env/release_evidence/security/container_vuln_scan_report.json`
  - `test_env/release_evidence/security/vulnerability_exception_report.json`
  - `test_env/release_evidence/security/backup_restore_rehearsal_report.json`

如果要显式指定 readiness 或 checklist，可追加：

```bash
python tools/build_customer_acceptance_bundle.py --manifest test_env/release/release_manifest_stable.json --readiness-report test_env/release_readiness_ready/release_readiness_report.json --promotion-checklist test_env/stable_promotion_ready/stable_promotion_checklist.json
```

如果 `--manifest` 指向 `industrial` 通道，builder 会默认切到 `industrial_release_readiness_report.json`、`industrial_promotion_checklist.json` 和 `industrial_delivery_rehearsal_report.json`。如需显式覆盖，可运行：

```bash
python tools/build_customer_acceptance_bundle.py --manifest test_env/release/release_manifest_industrial.json --readiness-report test_env/industrial_release_readiness_ready/industrial_release_readiness_report.json --promotion-checklist test_env/industrial_promotion_ready/industrial_promotion_checklist.json --industrial-delivery-rehearsal-report industrial_delivery_rehearsal_report.json
```

如果你要在隔离 smoke / rehearsal 输出根下生成 bundle，而不是读取 canonical `test_env/release_evidence/security/`，可显式覆盖安全报告路径：

```bash
python tools/build_customer_acceptance_bundle.py --manifest test_env/release/release_manifest_stable.json --security-posture-report test_env/smoke_phase_d_rehearsal_chain/security/security_posture_report.json --sbom-artifact test_env/smoke_phase_d_rehearsal_chain/security/sbom.json --python-vuln-report test_env/smoke_phase_d_rehearsal_chain/security/python_vuln_scan_report.json --container-vuln-report test_env/smoke_phase_d_rehearsal_chain/security/container_vuln_scan_report.json --backup-restore-report test_env/smoke_phase_d_rehearsal_chain/security/backup_restore_rehearsal_report.json
```

builder 当前还会额外打印：

- `customer_acceptance_bundle_reports_present=...`
- `customer_acceptance_bundle_security_posture=ready|blocked|missing`
- `customer_acceptance_bundle_external_mainline_input_checklist=passed|blocked/<missing>/<waiting>/<ready>/<completed>`

生成后的 bundle 还会直接携带 `extension_support_surface`，把 distributed / ROS2 / Godot / Helm-Kubernetes 的支持边界与非支持项以机器可读形式带给客户交付面。
每个 profile 现在还会显式携带 `deployment_commands`、`acceptance_checks` 和 `rollback_prerequisites`，用于把扩展实施动作、专项验收命令和回滚前提一起交给客户实施面。
bundle 还会额外挂出 `extension_execution_plan`，把 actionable profiles、专项验收要求和推荐 deployment/acceptance 命令收口成执行面摘要，并把这些命令并入 `recommended_commands`。
bundle 还会额外挂出 `extension_execution_evidence`，把 `extension_on_call_rehearsal`、`extension_exception_review_schedule`、`extension_escalation_closure` 和 `customer_external_bindings_confirmation` 四类留痕报告直接带进客户验收面；同时，最新的 `customer_external_bindings_closure_report.json` 也会作为 acceptance report 暴露最近一次 external bindings closure 尝试及其 `failed_steps`。

在 industrial rehearsal 主链中，`release_rehearsal_report.json` 现在也会把 `customer_external_bindings_closure` 与 `release_ops_execution` 一起提升成顶层机器字段，而独立 `industrial_delivery_rehearsal_report.json.summary` 会显式写出 `external_bindings_closure=passed|blocked`、`external_mainline_input_checklist=passed|blocked/<missing>/<waiting>/<ready>/<completed>` 与 `release_ops_execution=passed|blocked/<event_count>`，这样工业交付摘要不需要再回头展开 acceptance bundle 才能判断 closure 链、剩余外部输入缺口以及 control-plane wrapper 是否闭合。最新 contract 还会把 control-plane 的 `control_plane_session` / `control_plane_event_stream` 聚合到 `release_rehearsal_report.json` 与 `industrial_delivery_rehearsal_report.json` 顶层，来源优先取 `external_mainline_execution_plan`，回退到 `external_mainline_input_checklist`；同时 `release_ops_execution` wrapper 也会继续透传同一组 delivery-session 摘要。
bundle 还会额外挂出 `extension_execution_instance`，把本次客户交付的 `engagement_id`、`window_id`、`exception_review_due_at`、`delivery_root`、`closure_archive_root` 和 profile 级实例化路径直接带进验收面。
bundle 还会额外挂出 `extension_execution_schedule`，把 `window_trigger_at`、`signoff_due_at`、`closure_archive_due_at`、`window_trigger_record_path`、`residual_risk_review_record_path` 和 `closure_manifest_path` 直接带进客户窗口执行面。
bundle 还会额外挂出 `extension_execution_actuals`，把 `window_trigger_recorded_at`、`signoff_recorded_at`、`residual_risk_reviewed_at`、`closure_archived_at`、`window_trigger_recorded_by`、`signoff_recorded_by`、`residual_risk_reviewed_by`、`closure_archived_by`、`approval_identity_source_path`、`approval_identity_source_type`、`approval_identity_reference`、`archive_target_binding_type`、`archive_target_binding_reference_base`、`due_trigger_binding_type`、`due_trigger_binding_reference_base`、`due_trigger_checked_at`、`exception_review_due_at`、`closure_archive_due_at` 以及 `approval_identity_source.json`、`window_trigger.json`、`signoff.json`、`exception_review.json`、`residual_risk_review.json`、`due_trigger_check.json`、`archive_target.json`、`closure_archive/index.json`、`closure_manifest.json` 直接带进现场执行与后续跟进留痕面。
industrial bundle 当前还会默认挂入 `industrial_delivery_rehearsal_report`，把完整工业交付演练的六个阶段结果和 stage summary 一起带进客户验收面。
独立 `industrial_delivery_rehearsal_report` 现在还会把 `vulnerability_exception_review` 提升成一等 component；如果 `release_rehearsal_report.json` 没有显式写出这段，builder 也会回退读取 `security_release_preflight.metrics` 中的 `vulnerability_exception_review_report_status` / `vulnerability_exception_review_candidate_count`。
每个 actionable profile 现在还会附带 `runbook_entrypoints`，把正式执行文档入口固定到 `PRODUCTION_DEPLOYMENT_RUNBOOK.md`、客户安装指南以及 distributed / ROS2 / Godot 专项指南。
每个 profile 现在还会附带 `execution_template`，把 `operator_roles`、`upgrade_window_steps`、`handoff_owner_role`、`handoff_checkpoints`、`watch_owner_role`、`watch_actions`、`on_call_handoff_owner_role`、`on_call_handoff_records`、`residual_risk_owner_role`、`residual_risk_handoff_steps`、`exception_review_owner_role`、`exception_review_steps`、`incident_escalation_owner_role`、`incident_escalation_steps`、`escalation_closure_owner_role`、`escalation_closure_steps`、`signoff_checkpoints`、`rollback_owner_role`、`rollback_steps`、`rollback_evidence_owner_role` 和 `rollback_evidence_archive_steps` 一起压成客户现场执行模板。

## security posture baseline

阶段 D 当前已经固定 4 份一线安全/运维文档：

- `docs/guides/SECURITY_BASELINE.md`
- `docs/guides/AUDIT_TRAIL_POLICY.md`
- `docs/guides/BACKUP_RESTORE_RUNBOOK.md`
- `docs/guides/INCIDENT_RESPONSE_MATRIX.md`

当前安全 contract / tool 入口：

- `python tools/build_sbom_artifact.py`
- `python tools/write_vulnerability_scan_report.py`
- `python tools/run_python_vulnerability_scan.py`
- `python tools/run_container_vulnerability_scan.py`
- `python tools/build_vulnerability_exception_report.py`
- `python tools/build_vulnerability_remediation_report.py`
- `python tools/run_security_release_preflight.py`
- `python tools/build_security_posture_report.py`

当前默认输出：

- `test_env/security/sbom.json`
- `test_env/security/python_vuln_scan_report.json`
- `test_env/security/container_vuln_scan_report.json`
- `test_env/security/vulnerability_exception_report.json`
- `test_env/security/vulnerability_remediation_report.json`
- `test_env/security/backup_restore_rehearsal_report.json`
- `test_env/security/security_posture_report.json`
- `test_env/security/security_release_preflight_report.json`

canonical release evidence 采集后，对应路径为：

- `test_env/release_evidence/security/sbom.json`
- `test_env/release_evidence/security/python_vuln_scan_report.json`
- `test_env/release_evidence/security/container_vuln_scan_report.json`
- `test_env/release_evidence/security/vulnerability_exception_report.json`
- `test_env/release_evidence/security/vulnerability_remediation_report.json`
- `test_env/release_evidence/security/backup_restore_rehearsal_report.json`
- `test_env/release_evidence/security/security_posture_report.json`
- `test_env/release_evidence/security_release_preflight_report.json`

当前 `security_posture_report` 会校验：

- SBOM 是否存在且合法
- Python 依赖漏洞扫描报告是否存在
- 容器镜像漏洞扫描报告是否存在
- `vulnerability_remediation_report` 是否存在并能解释剩余 findings
- `vulnerability_exception_report` 是否在使用 exception 时提供有效审批和过期时间
- backup / restore rehearsal report 是否存在且状态为 `passed`
- 4 份 security baseline 文档是否存在

只有这些项全部齐全，并且剩余 findings 要么已修复、要么被有效 exception 覆盖时，`security_posture_status` 才会变成 `ready`。

当前 canonical 安全证据已经从“缺少 scanner 报告”推进到“报告存在、可解释且已通过”。最新基线是：

- `python_vuln_scan_report.json`: `passed`，`finding_count=0`
- `zenoh-router` 当前交付镜像：`deployment-zenoh-router`，上游基础镜像默认来自 `eclipse/zenoh:1.9.0`
- `container_vuln_scan_report.json`: 当前 canonical 以 `deployment-zenoh-router` 和 `deployment-web-panel-distributed` 为准；PR #20 / GitHub Actions run `28962518720` 证明两者当前均为 `0 findings`
- `vulnerability_remediation_report.json`: `ready`，`accepted_finding_count=0`，`unresolved_finding_count=0`，`matched_exception_count=0`
- `security_posture_report.json`: `ready`
- `security_release_preflight_report.json`: `passed`

因此，阶段 D 当前已从“让 preflight 过线”切换为“围绕 `deployment-web-panel-distributed` 的真实容器漏洞修复顺序，逐步用真实上游修复替换当前 `31` 条 no-fix exceptions”。

## release channel policy

`release_manifest` 现在会附带机器可读的 `release_policy`，用于表达 `dev / rc / stable / industrial` 的不同门禁语义。

- `dev`
  - 允许 `opt_in` evidence 仍未闭合
  - 允许 `diagnostic_ready` domain 仍未闭合
  - 只要 required evidence 全部通过，就可以得到 `release_gate_status=ready`
- `rc`
  - 不允许 `opt_in` evidence 未闭合
  - 不允许 `diagnostic_ready` domain 未闭合
  - optional live 证据和 distributed diagnostic state 都必须收口后才会得到 `ready`
- `stable`
  - 不允许 `opt_in` evidence 未闭合
  - 不允许 `diagnostic_ready` domain 未闭合
  - 额外要求 `release_approval.status=approved`
  - 额外要求 `release_source` 成功解析当前 Git HEAD
  - 额外要求当前 Git worktree 为 clean
  - 额外要求当前 HEAD 至少存在一个与 `version` 或 `v{version}` 匹配的 Git tag
  - `approved_by`、`approved_at`、`commit_sha` 缺一不可
  - `release_approval.commit_sha` 必须与 `release_source.commit_sha` 对齐
- `industrial`
  - 继承全部 `stable` 要求
  - 额外要求 `customer_delivery_surface.status=ready`
  - 额外要求 `industrial_delivery_gate.status=ready`
  - 适合正式工业交付包，而不是仅工程意义上的 stable 发布

`known_limitations` 仍然保留，但只作为信息字段；它们不会自动把 gate 从 `ready` 降级为 `ready_with_limitations`。

## stable signoff

稳定通道现在要求显式签核。builder 会把签核记录写入 `release_approval`，字段包括：

- `status`
- `required`
- `approved_by`
- `approved_at`
- `commit_sha`
- `notes`

同时，builder 会把当前仓库 HEAD 写入 `release_source`：

- `resolved_from_git`
- `commit_sha`
- `short_commit_sha`
- `git_tag`
- `matched_version_tag`
- `worktree_clean`
- `worktree_status_summary`
- `version_tag_matches`

如果 `--approval-status approved` 且未显式传 `--commit-sha`，builder 会自动把当前 HEAD commit 写入 `release_approval.commit_sha`。如果显式传入的 `--commit-sha` 与当前 HEAD 不一致，stable gate 会保持 `blocked`。如果当前 worktree 不是 clean，stable gate 也会保持 `blocked`。如果当前 HEAD 没有匹配 `version` 或 `v{version}` 的 Git tag，stable gate 同样会保持 `blocked`。

稳定通道示例：

```bash
python tools/build_release_artifact.py --version 2026.04.12 --channel stable --build-id build-20260412-stable --approval-status approved --approved-by release-manager --approved-at 2026-04-12T12:30:00+00:00 --approval-notes "stable signoff"
```

如果你要显式演练这条路径，而不是直接在当前仓库打 tag，可以运行：

```bash
python tools/run_release_rehearsal.py --version 2026.04.12-rehearsal --build-id release-rehearsal
```

该脚本会：

- 创建临时 Git repo
- 提交当前 rehearsal seed
- 创建与 `version` 匹配的 tag
- 写入 required release evidence、通过态 distributed / Godot / ROS2 证据，以及一套 rehearsal customer delivery / security / remediation 产物
- 调用 builder 生成 stable manifest
- 生成 `security_release_preflight_report.json`
- 生成 `release_manifest_industrial.json`
- 生成 `industrial_release_readiness_report.json`
- 生成 `industrial_promotion_checklist.json`
- 生成 `customer_acceptance_bundle_industrial.json`
- 生成 `industrial_delivery_rehearsal_report.json`
- 默认复制 `deployment/customer_delivery.external_bindings.rehearsal.json` 到 rehearsal 工作区，并让 `extension_execution_actuals.external_bindings_status=ready`
- 要求 `release_gate_status=ready`
- 要求 `customer_delivery_surface.status=ready`
- 要求 `industrial_delivery_gate.status=ready`
- 报告会额外挂出 `security_release_preflight`
- 报告会额外挂出 `vulnerability_exception_review`
- 报告会额外挂出 `industrial_manifest`
- 报告会额外挂出 `industrial_release_readiness`
- 报告会额外挂出 `industrial_promotion_checklist`
- 报告会额外挂出 `industrial_customer_acceptance_bundle`
- 报告会额外挂出 `extension_execution_plan`
- 报告会额外挂出 `extension_execution_evidence`
- 报告会额外挂出 `extension_execution_instance`
- 报告会额外挂出 `extension_execution_schedule`
- 报告会额外挂出 `extension_execution_actuals`
- 报告会额外挂出 `delivery_rehearsal_stages`，显式覆盖 `new_environment_install`、`smoke`、`live_evidence`、`upgrade`、`rollback`、`backup_restore`
- 支持相对或绝对 `--output-root`；脚本会先规范化路径，再生成 security / remediation / backup-restore 产物
- 输出 `release_rehearsal_report.json`

如果你已经有 `release_rehearsal_report.json`，但想单独重建工业交付演练报告，可运行：

```bash
python tools/build_industrial_delivery_rehearsal_report.py --rehearsal-report test_env/release_rehearsal/release_rehearsal_report.json
```

最新 CLI stdout 还会显式打印 `industrial_delivery_vulnerability_exception_review=<status>/<candidate_count>`、`industrial_delivery_external_mainline_input_checklist=<status>/<missing>/<waiting>/<ready>/<completed>`、`industrial_delivery_release_ops_execution=<status>/<event_count>` 与 `industrial_delivery_control_plane_events=<count>`；`tools/run_release_rehearsal.py` 也会同步打印 `release_rehearsal_control_plane_events=<count>`。`tests/run_smoke_tests.py` 的 artifact summary 在发现 `industrial_release_readiness_report.json` 或 `industrial_delivery_rehearsal_report.json` 时，也会把同一组 `exception_review` / `external_mainline` / `external_mainline_input_checklist` / `worktree` / `control_plane_events` 摘要直接拼到路径后面。

如果你要先看“当前仓库离 stable 还差什么”，而不是直接演练通过态，可以运行：

```bash
python tools/check_release_readiness.py
```

如果 stable manifest 已经生成，而且你想让 readiness 直接复用其中的签核元数据，而不是重复传 `--approval-status/--approved-by/...`，可以运行：

```bash
python tools/check_release_readiness.py --approval-manifest test_env/release/release_manifest_stable.json
```

该脚本会：

- 预览当前 `rc` manifest
- 预览目标 `stable` manifest
- 读取 canonical `security_release_preflight_report.json`
- 读取 preview manifest 自带的 `customer_delivery_surface`
- 读取 `customer_delivery_surface.extension_support_surface`
- 读取 preview manifest 自带的 `industrial_delivery_gate`
- 汇总 `customer_delivery_surface`，显式给出 support matrix、capacity declaration、acceptance checklist 和 known limitations 的挂载状态
- 汇总 `extension_support_surface`，显式给出 distributed / ROS2 / Godot / Helm-Kubernetes 的当前支持边界、专项验收要求、实施动作和回滚前提
- 汇总 `extension_execution_plan.runbook_entrypoints`，显式给出正式 runbook / 安装手册 / 扩展专项指南入口
- 汇总 `extension_execution_plan.profiles[*].execution_template`
- 汇总 `extension_execution_evidence`
- 汇总 `extension_execution_instance`
- 汇总 `extension_execution_schedule`
- 汇总 `extension_execution_actuals`
- 显式给出 `extension_on_call_rehearsal` / `extension_exception_review_schedule` / `extension_escalation_closure`
- 显式给出 `handoff_owner_role` / `handoff_checkpoints`
- 显式给出 `watch_owner_role` / `watch_actions`
- 显式给出 `on_call_handoff_owner_role` / `on_call_handoff_records`
- 显式给出 `residual_risk_owner_role` / `residual_risk_handoff_steps`
- 显式给出 `exception_review_owner_role` / `exception_review_steps`
- 显式给出 `incident_escalation_owner_role` / `incident_escalation_steps`
- 显式给出 `escalation_closure_owner_role` / `escalation_closure_steps`
- 显式给出 `signoff_checkpoints`
- 显式给出 `rollback_evidence_owner_role` / `rollback_evidence_archive_steps`
- 显式给出角色分工、升级窗口顺序、值班动作、on-call 交接、残余风险交接、exception 复核、异常升级闭环、签收点和回滚证据归档责任
- 汇总 `industrial_delivery_gate`，显式给出 deployment package、required evidence attestation、SBOM、漏洞扫描和 backup/restore rehearsal 的闭合状态
- 写出 `release_readiness_report.json`
- 明确列出下一步命令，例如补签核、补 tag、生成 worktree cleanup report、重跑 live evidence、补齐 security release preflight、补齐客户交付文档集或补齐 industrial delivery gate

如需显式指定 security preflight 报告路径，可追加：

```bash
python tools/check_release_readiness.py --security-preflight-report test_env/release_evidence/security_release_preflight_report.json
```

如果你要直接预览 `industrial` 通道，而不是继续从 stable 视角折算，可以运行：

```bash
python tools/check_industrial_release_readiness.py
```

如果 industrial manifest 已经生成，而且你想让该预检直接复用其中的签核元数据，可以运行：

```bash
python tools/check_industrial_release_readiness.py --approval-manifest test_env/release/release_manifest_industrial.json
```

该脚本会：

- 预览目标 `industrial` manifest
- 读取 canonical `security_release_preflight_report.json`
- 读取 industrial preview manifest 自带的 `customer_delivery_surface`
- 读取 industrial preview manifest 自带的 `industrial_delivery_gate`
- 写出 `industrial_release_readiness_report.json`
- 明确列出下一步命令，例如补 industrial 签核、补 tag、生成 worktree cleanup report、补齐 security release preflight、补齐客户交付文档集或补齐 industrial delivery gate

如需显式指定 security preflight 报告路径，可追加：

```bash
python tools/check_industrial_release_readiness.py --security-preflight-report test_env/release_evidence/security_release_preflight_report.json
```

如果当前 stable 阻塞项包含 dirty worktree，建议先运行：

优先入口现在是：

```bash
python tools/run_worktree_release_blocker.py
```

该 runner 会：

- 写出统一的 `worktree_release_blocker_report.json`
- 先刷新 `worktree_cleanup_report.json`
- 在存在 tracked runtime / generated 候选时继续刷新 `tracked_artifact_review_report.json`
- 把 `clean_worktree` 当前是否仍阻塞 stable / industrial promotion 固定成结构化摘要

底层如需单独查看 cleanup 细分分类，仍可直接运行：

```bash
python tools/build_worktree_cleanup_report.py
```

该脚本会：

- 写出 `worktree_cleanup_report.json`
- 非破坏性分类当前 Git worktree 改动
- 区分运行时产物、生成物候选、源码/文档人工审查项和未知分类
- 给出下一步清理计划，而不是直接删除文件
- 额外挂出 `tracked_review_candidate_count`、`tracked_review_candidate_paths` 和 `tracked_review_command`，便于判断是否需要继续做 tracked artifact review

如果 cleanup report 里仍有 tracked 的运行时产物或生成物候选，建议继续运行：

```bash
python tools/build_tracked_artifact_review_report.py
```

该脚本会：

- 聚焦 tracked 的 runtime / generated 候选
- 汇总每个文件的 diff 行数和 diff 预览
- 给出“回退或重建基线”和“审查是否继续跟踪”的建议动作
- 避免直接对 tracked 文件做破坏性清理

如果你要把这些阻塞项直接转换成当前 HEAD 的结构化执行清单，可以运行：

```bash
python tools/build_stable_promotion_checklist.py
```

如果 stable manifest 已经存在，并且你希望 checklist 直接识别“当前 HEAD 已有 ready manifest”，可以运行：

```bash
python tools/build_stable_promotion_checklist.py --approval-manifest test_env/release/release_manifest_stable.json
```

该脚本会：

- 读取或刷新当前 `release_readiness_report.json`
- 读取 stable preview manifest 自带的 `customer_delivery_surface`
- 读取 stable preview manifest 自带的 `industrial_delivery_gate`
- 写出 `stable_promotion_checklist.json`
- 固定列出 evidence、security release preflight、diagnostic domain、stable approval、Git HEAD 绑定、clean worktree、版本 tag 和最终 builder step
- 同步带出 `customer_delivery_surface`，并新增“确认 Phase E 客户交付文档集齐备”步骤
- 同步带出 `industrial_delivery_gate`，并新增“确认 industrial delivery gate 已闭合”步骤
- 区分 `blocking` 前置项与“已可执行但尚未执行”的最终 stable builder
- 在 dirty worktree 场景下，`clean_worktree` step 会直接给出 `run_worktree_release_blocker.py` 命令
- 在 security preflight 缺失或 `blocked` 时，`security_release_preflight` step 会保持阻塞，即使底层 stable manifest gate 已经是 `ready`
- 当前 `customer_delivery_surface` 与 `industrial_delivery_gate` step 都先作为 stable checklist 的 Phase E/F 桥接项暴露，不直接阻塞 stable gate；正式 `industrial` release channel 现在已经会消费这组字段并把它们视为 blocking gate
- 在传入 `--approval-manifest` 时，直接复用已有 stable manifest 的签核元数据，并在 manifest 已 ready 且匹配当前 HEAD 时把最终 builder step 标记为已完成

最新 `check_release_readiness.py`、`build_stable_promotion_checklist.py` 与 `build_industrial_promotion_checklist.py` 还会把这条 runner 的结果固定成 `worktree_release_blocker` 顶层字段，并通过 `stable_worktree_release_blocker=...` / `industrial_worktree_release_blocker=...` stdout 显式打印。`release_ops` 以 `worktree_release_blocker` action 运行时，也会把统一 control-plane 的 `control_plane_session` / `control_plane_event_stream` 回写到 `worktree_release_blocker_report.json`，这样 worktree closeout runner 现在和 external-mainline 一样具备可追踪 delivery-session 摘要。`tests/run_smoke_tests.py` 现在也会在 `release_readiness_report.json`、两份 promotion checklist 和 `worktree_release_blocker_report.json` 的 artifact 摘要后追加 `worktree=<status>/<total_paths>/<tracked_review_candidate_count>`，因此 dirty worktree 是否仍阻塞 release 已经不需要再手工翻报告体。

当前 `industrial_delivery_gate` 的 `vuln_scan_status` 不再只看原始 `vulnerability_scan_report.status`。如果原始容器扫描仍为 `blocked`，但 `vulnerability_remediation_report` 已 `ready` 且 `unresolved_finding_count=0`，或者 `security_posture_report` 已 `ready` 且不存在未解决漏洞，则 industrial gate 会把该项视为已闭合。

如需显式指定 security preflight 报告路径，可追加：

```bash
python tools/build_stable_promotion_checklist.py --security-preflight-report test_env/release_evidence/security_release_preflight_report.json
```

如果你要把当前 HEAD 的 industrial 阻塞项转成真正阻塞的工业发布清单，可以运行：

```bash
python tools/build_industrial_promotion_checklist.py
```

如果 industrial manifest 已经存在，并且你希望 checklist 直接识别“当前 HEAD 已有 ready industrial manifest”，可以运行：

```bash
python tools/build_industrial_promotion_checklist.py --approval-manifest test_env/release/release_manifest_industrial.json
```

该脚本会：

- 读取或刷新当前 `industrial_release_readiness_report.json`
- 读取 industrial preview manifest 自带的 `customer_delivery_surface`
- 读取 `customer_delivery_surface.extension_support_surface`
- 读取 industrial preview manifest 自带的 `industrial_delivery_gate`
- 生成 `extension_execution_plan`
- 读取或重建 `extension_execution_evidence`
- 读取或重建 `extension_execution_instance`
- 读取或重建 `extension_execution_schedule`
- 读取或重建 `extension_execution_actuals`
- 写出 `industrial_promotion_checklist.json`
- 固定列出 evidence、security release preflight、customer delivery、industrial delivery、diagnostic domain、industrial approval、Git HEAD 绑定、clean worktree、版本 tag 和最终 builder step
- 保留 `extension_support_surface.profiles[*].deployment_commands`、`acceptance_checks` 与 `rollback_prerequisites`，便于工业发布清单直接引用扩展实施动作
- 保留 `extension_execution_plan.profiles[*].runbook_entrypoints`，便于工业发布清单直接定位正式执行文档
- 保留 `extension_execution_plan.profiles[*].execution_template`，便于工业发布清单直接引用现场角色分工、升级窗口步骤、值班/交接记录、风险复核、升级闭环和回滚证据归档责任
- 具体包括 `handoff_owner_role`、`handoff_checkpoints`、`watch_owner_role`、`watch_actions`、`on_call_handoff_owner_role`、`on_call_handoff_records`、`residual_risk_owner_role`、`residual_risk_handoff_steps`、`exception_review_owner_role`、`exception_review_steps`、`incident_escalation_owner_role`、`incident_escalation_steps`、`escalation_closure_owner_role`、`escalation_closure_steps`、`signoff_checkpoints`、`rollback_evidence_owner_role` 和 `rollback_evidence_archive_steps`
- 将 `extension_execution_plan` 一并落盘，供工业 promotion 链路直接引用 actionable profile 的实施与验收命令
- 将 `extension_execution_evidence` 一并落盘，供工业 promotion 链路直接引用值班演练、exception 复核排程、升级 closure 留痕和 customer external bindings 确认留痕
- 将 `extension_execution_instance` 一并落盘，供工业 promotion 链路直接引用客户交付窗口、到期排程和 closure archive 目标
- 将 `extension_execution_schedule` 一并落盘，供工业 promotion 链路直接引用窗口触发时间、signoff 截点、residual risk review 记录和 closure archive manifest
- 将 `extension_execution_actuals` 一并落盘，供 stable/industrial promotion 与 rehearsal 直接引用 approval identity source、window trigger、signoff、residual risk review、exception review follow-up、due trigger check、archive target、closure archive index 和 closure manifest 留痕
- 把“确认 Phase E 客户交付文档集齐备”和“确认 industrial delivery gate 已闭合”都视为正式 blocking step
- 在传入 `--approval-manifest` 时，直接复用已有 industrial manifest 的签核元数据，并在 manifest 已 ready 且匹配当前 HEAD 时把最终 builder step 标记为已完成

如需显式指定 security preflight 报告路径，可追加：

```bash
python tools/build_industrial_promotion_checklist.py --security-preflight-report test_env/release_evidence/security_release_preflight_report.json
```

## 验收标准

阶段五完成的标准不是“所有 live 环境都永远绿色”，而是：

- 发布产物有正式契约
- capability matrix 有正式契约
- release checklist 能关联 contract version、test evidence 和 known limitations
- 当前一线文档不再依赖 archive 里的 release 模板
- 默认 smoke 和默认非 live 门禁都覆盖到发布门禁主链

## 当前结果

当前仓库已具备：

- `capability_matrix` contract v1
- `release_manifest` contract v1
- `tools/build_release_artifact.py`
- Web `/api/capabilities/matrix`
- MCP `capability_matrix_get`
- smoke 中的 capability matrix probe

当前最新 release 口径是：

- `distributed_runtime_live=passed`
- `godot_headless_live=passed`
- `ros2_bridge_live=passed`
- `blocked_evidence=0`
- `blocked_optional_evidence=0`
- `opt_in_evidence=0`
- `release_gate_status=ready`
- `clean_checkout_smoke.status=passed`
- `clean_checkout_smoke.runs=2`

仓库仍会保留 `known_limitations` 作为信息字段，例如 MCP 仅声明 `stdio` 传输面，以及 live Godot / ROS2 验证不并入默认 `not live` 门禁；但这些已接受的产品边界不再自动把 release gate 降级为 `ready_with_limitations`。只有 `diagnostic_ready` domain、`blocked` evidence 或仍未闭合的 `opt_in` evidence 会让 gate 保持在非 `ready` 状态。

如果需要生成新的 release artifact，只需要替换 `version`、`channel`、`build_id` 和 `release_summary`，然后归档生成的 `release_manifest.json`。stable 通道额外需要确认签核记录绑定的是当前 Git HEAD，并且当前 HEAD 已打上匹配发布版本的 tag。
