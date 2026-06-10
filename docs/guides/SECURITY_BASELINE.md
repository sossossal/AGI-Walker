# AGI-Walker Security Baseline

更新日期：`2026-04-15`

本页定义当前仓库已经落实的最小安全基线。它只覆盖当前一线交付路径，也就是 Docker Compose + Web Panel + release artifact，不声明尚未产品化的外部安全平台。

## 适用范围

- Docker Compose 部署入口：`deployment/docker-compose.yml`
- 主机侧配置：`deployment/compose.env`
- 容器侧应用配置：`deployment/web_panel.env`
- Release 证据与交付产物：`test_env/release/`、`test_env/release_evidence/`

## secrets 基线

生产环境必须显式提供以下敏感配置：

- `AGI_WALKER_SECRET_KEY`

当前代码会在以下任一条件成立时拒绝使用默认 secret 启动：

- `AGI_WALKER_ENV=prod|production`
- `AGI_WALKER_REQUIRE_EXPLICIT_SECRET=1`

对应实现位于 `web_panel/auth.py`。当前要求：

1. `deployment/web_panel.env.example` 只能保留占位值，不允许写入真实密钥。
2. 真实 `deployment/web_panel.env` 不入库。
3. 真实 `AGI_WALKER_SECRET_KEY` 通过客户环境的 secrets 管理方式注入，不通过聊天、邮件或 issue 明文分发。
4. 发布前必须确认 `AGI_WALKER_SECRET_KEY` 不等于 `change-me-before-production` 或代码默认值。

## 配置分层

当前支持的配置分层如下：

- `deployment/compose.env.example`
  - 主机端口
  - 持久化目录根
  - `AGI_WALKER_COMPOSE_WEB_ENV_FILE`
  - 容器内数据库、workflow runs、archive 路径
- `deployment/web_panel.env.example`
  - Web Panel 认证与分页配置
  - `AGI_WALKER_SECRET_KEY`
  - `AGI_WALKER_ZENOH_ENDPOINT`

严禁把以下内容混写进源码、文档示例或 tracked runtime 文件：

- 真实 JWT secret
- 客户 token
- 数据库口令
- 外部 GitHub token

## SBOM 与漏洞扫描

阶段 D 当前使用以下受管输入与结构化安全产物：

- `deployment/security/vulnerability_exceptions.input.json`
- `test_env/security/sbom.json`
- `test_env/security/python_vuln_scan_report.json`
- `test_env/security/container_vuln_scan_report.json`
- `test_env/security/vulnerability_exception_report.json`
- `test_env/security/vulnerability_remediation_report.json`
- `test_env/security/backup_restore_rehearsal_report.json`
- `test_env/security/security_posture_report.json`

当前生成入口：

```bash
python tools/build_sbom_artifact.py --output test_env/security/sbom.json
python tools/write_vulnerability_scan_report.py --scan-name python_dependencies --target pyproject.toml --status passed --scanner manual-review --summary "Python dependency review completed." --command "manual placeholder" --output test_env/security/python_vuln_scan_report.json
python tools/write_vulnerability_scan_report.py --scan-name container_images --target deployment/docker-compose.yml --status passed --scanner manual-review --summary "Container image review completed." --command "manual placeholder" --output test_env/security/container_vuln_scan_report.json
python tools/run_python_vulnerability_scan.py --output test_env/security/python_vuln_scan_report.json
python tools/run_container_vulnerability_scan.py --image-ref deployment-zenoh-router --output test_env/security/container_vuln_scan_report.json
python tools/build_vulnerability_exception_report.py --input deployment/security/vulnerability_exceptions.input.json --output test_env/security/vulnerability_exception_report.json
python tools/build_vulnerability_remediation_report.py --python-vuln-report test_env/security/python_vuln_scan_report.json --container-vuln-report test_env/security/container_vuln_scan_report.json --vulnerability-exception-report test_env/security/vulnerability_exception_report.json --output test_env/security/vulnerability_remediation_report.json
python tools/run_backup_restore_rehearsal.py --output-root test_env/security/backup_restore_rehearsal --report-file test_env/security/backup_restore_rehearsal_report.json
python tools/build_security_posture_report.py --vulnerability-remediation-report test_env/security/vulnerability_remediation_report.json --vulnerability-exception-report test_env/security/vulnerability_exception_report.json --output test_env/security/security_posture_report.json
python tools/run_security_release_preflight.py --skip-collect --security-posture-report test_env/security/security_posture_report.json --report-file test_env/security/security_release_preflight_report.json
```

默认情况下，`tools/collect_release_evidence.py` 与 `tools/run_security_release_preflight.py` 会先读取 `deployment/security/vulnerability_exceptions.input.json`，并自动生成 canonical `vulnerability_exception_report.json`；单独调用 `tools/build_vulnerability_exception_report.py` 主要用于显式审查或离线编译 exception report。

默认的 Python vulnerability scan 会先生成一份与当前客户交付面一致的 requirements snapshot，只包含：

- `pyproject.toml` 的 `project.dependencies`
- `deployment/requirements.web_panel*.txt`
- `deployment/requirements.distributed_runtime.txt`

它不会默认把 `dev`、`training`、`hardware` optional groups 混进 canonical baseline。

如果你需要对训练或硬件扩展面做附加扫描，显式传入：

```bash
python tools/run_python_vulnerability_scan.py --include-optional-group training --include-optional-group hardware --output test_env/security/python_vuln_scan_report.json
```

如果本机 `pip-audit` 在 Windows 上因临时目录 ACL 或沙箱环境失败，可显式覆盖临时目录：

```bash
set AGI_WALKER_PIP_AUDIT_TMPDIR=D:\scan_tmp\agi_walker_pip_audit
python tools/run_python_vulnerability_scan.py --output test_env/security/python_vuln_scan_report.json
```

如果客户环境已经在外部 scanner 中生成原始 JSON，可直接规范化为 contract，而不是手工拼 summary：

```bash
python tools/write_vulnerability_scan_report.py --scan-name python_dependencies --target pyproject.toml --raw-report D:/scanner/pip_audit.json --raw-format pip-audit-json --command "pip-audit --format json" --output test_env/security/python_vuln_scan_report.json
python tools/write_vulnerability_scan_report.py --scan-name container_images --target deployment/docker-compose.yml --raw-report D:/scanner/trivy.json --raw-format trivy-json --command "trivy image --format json deployment-zenoh-router" --output test_env/security/container_vuln_scan_report.json
```

当前 raw 格式支持面：

- `pip-audit-json`
- `trivy-json`

如果 release preflight 所在环境已安装 scanner，而不是只提供外部 JSON，可直接执行：

```bash
python tools/collect_release_evidence.py --run-python-vuln-scan --run-container-vuln-scan --container-image-ref deployment-zenoh-router --vulnerability-exception-input-source deployment/security/vulnerability_exceptions.input.json
```

collector 仍保持固定优先级：

1. 已存在的结构化漏洞报告
2. 外部 scanner 原始 JSON
3. 本地实际 scanner 执行

如果你已经有经过审批的 exception 输入或 report，也应一并接入 canonical evidence：

```bash
python tools/collect_release_evidence.py --vulnerability-exception-input-source deployment/security/vulnerability_exceptions.input.json
```

如果外部系统已经直接产出结构化 `vulnerability_exception_report.json`，也可以改用 `--vulnerability-exception-report-source D:/path/vulnerability_exception_report.json`。

当前 Compose 默认会基于 `AGI_WALKER_ZENOH_BASE_IMAGE` 构建一份自带 Alpine 安全升级的 `zenoh-router` 交付镜像。默认上游基础镜像为 `eclipse/zenoh:1.9.0`，最终交付镜像名为 `AGI_WALKER_ZENOH_IMAGE=deployment-zenoh-router`。release preflight 与 CI 应扫描这个当前交付镜像，而不是继续手写旧的上游 tag。

如果你需要一条会对 `security_posture_report` 做最终成败判定的正式预检命令，应运行：

```bash
python tools/run_security_release_preflight.py --security-only --output-root test_env/release_evidence --run-python-vuln-scan --run-container-vuln-scan --container-image-ref deployment-zenoh-router --container-image-ref deployment-web-panel-distributed --vulnerability-exception-input-source deployment/security/vulnerability_exceptions.input.json
```

该命令当前是阶段 D 的正式 preflight 入口，CI 也应复用它，而不是各自拼 scanner 命令。

默认 preflight 允许被有效、未过期 exception 覆盖的 no-fix findings 作为 tracked residual risk 通过。完整生产修复不能停在这个状态；容器 remediation 线的最终目标是生产镜像 `accepted_vulnerability_findings=0`。当需要生成可审计的修复闭环 evidence 时，按以下顺序保留产物：

```bash
python tools/build_container_vulnerability_inventory.py --raw-report test_env/release_evidence/security/container_vuln_scan_report_raw --production-image-ref deployment-zenoh-router --production-image-ref deployment-web-panel-distributed --output test_env/container_vulnerability_remediation/inventory.json
python tools/build_container_vulnerability_reduction_plan.py --inventory test_env/container_vulnerability_remediation/inventory.json --dockerfile deployment-web-panel-distributed=deployment/Dockerfile.web_panel --dockerfile deployment-zenoh-router=deployment/Dockerfile.zenoh_router --output test_env/container_vulnerability_remediation/reduction_plan.json
python tools/run_security_release_preflight.py --skip-collect --fail-on-accepted-vulnerability-findings --security-posture-report test_env/release_evidence/security/security_posture_report.json --vulnerability-exception-review-report test_env/release_evidence/security/vulnerability_exception_review_report.json --report-file test_env/container_vulnerability_remediation/strict_security_preflight_report.json
python tools/build_container_vulnerability_remediation_closeout.py --inventory test_env/container_vulnerability_remediation/inventory.json --reduction-plan test_env/container_vulnerability_remediation/reduction_plan.json --strict-preflight-report test_env/container_vulnerability_remediation/strict_security_preflight_report.json --output test_env/container_vulnerability_remediation/closeout.json
```

`container_vulnerability_remediation_closeout.v1` 只有在生产 findings、active accepted findings 和 strict preflight blocker 都清零后才能进入 ready。scheduled/manual security CI 会在 raw scanner evidence 存在时上传 `test_env/container_vulnerability_remediation`，但默认 security preflight 的 pass/fail 语义仍保持向后兼容。

`deployment/Dockerfile.web_panel` 会在 Python 依赖安装完成后从最终镜像移除 `apt`、`bash` 和 `libbz2-1.0`，对应 reduction plan 中的 package-manager/diagnostic removal candidates 以及 `CVE-2026-42250` 的 final-image reduction path。移除这些包后，Dockerfile 会执行一次 Web server/worker import smoke；仍必须通过 Docker/Trivy 重新生成 inventory、strict preflight 和 closeout artifact，只有远端或本地 Docker 证据确认 matching findings 消失后，才能退休对应 exception。

如果 scanner 已经执行完，并且你要把修复顺序固定成结构化报告，而不是只看原始 JSON，可直接生成 remediation report：

```bash
python tools/build_vulnerability_exception_report.py --input deployment/security/vulnerability_exceptions.input.json --output test_env/release_evidence/security/vulnerability_exception_report.json
python tools/build_vulnerability_remediation_report.py --python-vuln-report test_env/release_evidence/security/python_vuln_scan_report.json --container-vuln-report test_env/release_evidence/security/container_vuln_scan_report.json --vulnerability-exception-report test_env/release_evidence/security/vulnerability_exception_report.json --output test_env/release_evidence/security/vulnerability_remediation_report.json
```

当前 canonical 口径下，security posture 与 preflight 已从“缺少漏洞报告”推进到“报告完整、risk 已解释，但生产发布仍 blocked”。当前基线为：

- Python 依赖：当前已 `passed`，`finding_count=0`
- 容器镜像：当前 canonical findings 以 `deployment-zenoh-router` 和 `deployment-web-panel-distributed` 为准；最新远端 security-preflight artifact 显示 production findings 集中在 `deployment-web-panel-distributed`
- `vulnerability_remediation_report`: 当前仍为 `blocked`，已确认 `accepted_finding_count=106`、`unresolved_finding_count=1`；unresolved 项为 `libbz2-1.0` / `CVE-2026-42250`，本仓库已增加 final-image removal candidate，等待远端 Docker/Trivy 证明确认
  - 若某条 `only_without_fix_version=true` 的 active exception 对应 findings 开始携带 fix version，report 现在会额外挂出 `stale_exception_count` / `stale_exceptions`，明确标记哪些 no-fix exceptions 已失效，需要从审批输入里移除或替换
- `security_posture_status=blocked`
- `security_release_preflight_status=blocked`；默认 managed-exception profile 仍可用于跟踪有效例外，但完整生产修复以 strict zero-exception closeout 为准

dockerized Trivy fallback 的 `/scan/image.tar` 镜像标识问题也已修复，因此镜像级 exceptions 现在会按真实交付镜像名匹配 canonical findings，而不是误匹配到归档路径。

当前 contract 语义：

- `sbom_artifact`
- `vulnerability_scan_report`
- `vulnerability_exception_report`
  - 当前会额外挂出 `review_window_days`、`review_due_exception_count`、`next_exception_expiry` 与 `review_status`，用于标记 active exception 是否已进入正式复核窗口，而不是只区分 `active/expired`
- `vulnerability_remediation_report`
  - 当前会在 `vulnerability_exception_report` 段补充 `stale_exception_count`、`stale_exception_ids` 与 `stale_exceptions`，用于标记“原先只允许 no-fix 的 exception 现在已经看到 fix version”
- `security_posture_report`
  - 当前会在 `vulnerability_exception_report` 摘要段继续透传 review-window 指标和 `stale_exception_count` / `stale_exception_ids`；`tools/run_security_release_preflight.py` 也会把它们写进结构化 evidence metrics，便于持续追踪 `2026-05-15` 前的 residual risk 消化状态

只有在 SBOM、两份漏洞扫描报告、恢复演练报告和本页列出的基线文档都齐全时，`security_posture_report.posture_status` 才会进入 ready 判定。若剩余 findings 已被有效 `vulnerability_exception_report` 覆盖，security posture 可以带着显式 residual risk 进入 `ready`；否则仍保持 `blocked`。

## 生产运行目录

当前 Compose 会把以下主机目录映射到容器内：

- `<runtime-root>/db`
- `<runtime-root>/workflow_runs`
- `<runtime-root>/workflow_archive`
- `<runtime-root>/backups`

容器内对应路径：

- `/var/lib/agi_walker/db`
- `/var/lib/agi_walker/workflow_runs`
- `/var/lib/agi_walker/workflow_archive`
- `/var/lib/agi_walker/backups`

安全要求：

1. 只对运维用户开放主机侧 runtime 根目录写权限。
2. 备份目录和数据库目录不得与源码目录混放。
3. 客户交付环境应定期审查 `workflow_runs` 和 `workflow_archive` 中是否包含敏感输入或客户数据。

## 发布门禁关联

当前 release / customer acceptance 主链与安全基线的关联字段包括：

- `release_approval`
- `release_source`
- `test_evidence`
- `customer_acceptance_bundle`

当前阶段 D 目标不是引入新的审批系统，而是要求这些字段、文档和安全产物能形成一条可审计链路。

## 当前不声明的能力

以下能力当前不作为已交付能力声明：

- 托管 secrets manager 集成
- 自动化依赖漏洞扫描平台
- 第三方 SIEM / 审计平台对接
- Kubernetes secret / vault 模板

这些项后续可以接入，但在当前工业交付包中只能作为扩展项，不能当作默认已支持能力。
