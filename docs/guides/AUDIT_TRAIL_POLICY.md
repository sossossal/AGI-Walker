# AGI-Walker Audit Trail Policy

更新日期：`2026-04-13`

本页定义当前工业化交付范围内的最小审计留痕要求。目标不是一次性覆盖全部企业合规场景，而是确保 release、部署、配置变更和 workflow 运行都有可追溯记录。

## 审计对象

当前必须可追溯的对象：

- release 生成与签核
- stable promotion 决策
- 部署 actor 与部署时间
- 配置文件变更
- workflow 运行记录
- backup / restore 演练记录

## release 审计

当前 release 主链已经固定以下审计字段：

- `release_manifest.release_approval`
- `release_manifest.release_source`
- `release_manifest.test_evidence`
- `release_manifest.build_id`
- `release_manifest.version`
- `release_manifest.channel`

最小要求：

1. stable 发布必须保留 `approved_by`、`approved_at`、`commit_sha`。
2. `release_approval.commit_sha` 必须和 `release_source.commit_sha` 对齐。
3. 当前 HEAD 必须存在与版本匹配的 Git tag。
4. canonical release evidence 必须保留命令、时间、报告路径和 commit SHA。

## 部署审计

当前 Compose 交付至少要记录：

- 谁执行了部署
- 使用了哪份 `deployment/compose.env`
- 使用了哪份 `deployment/web_panel.env`
- 部署时对应的 `release_manifest`
- 部署时间
- 部署后的健康检查结果

推荐最小做法：

1. 归档部署时使用的 release manifest 路径。
2. 记录部署命令、执行用户和时间。
3. 记录 `docker compose ps` 与健康检查结果。
4. 将记录存放在客户运行时目录或交付报告目录中，不写回源码树。

## 配置变更审计

以下配置变更必须可说明来源：

- `AGI_WALKER_SECRET_KEY`
- `AGI_WALKER_DATABASE_URL`
- `AGI_WALKER_WEB_OUTPUT_ROOT`
- `AGI_WALKER_WEB_ARCHIVE_ROOT`
- `AGI_WALKER_ZENOH_ENDPOINT`
- 端口、runtime root、保留策略等 Compose 覆盖项

当前策略：

1. 示例文件保留在仓库中。
2. 真实客户环境文件不入库。
3. 变更由客户侧配置仓库、变更单或部署记录承担审计，不在本仓库中存放真实 secret。

## workflow 运行审计

当前最小可审计对象：

- workflow result
- workflow step artifact
- workflow_runs
- workflow_archive
- workflow archive

运行审计要求：

1. workflow runs 写入 `AGI_WALKER_WEB_OUTPUT_ROOT`。
2. archive 写入 `AGI_WALKER_WEB_ARCHIVE_ROOT`。
3. 需要保留 run id、workflow 名称、状态、时间、artifact 路径。
4. 默认 smoke 和演练产物必须写入 `test_env/` 或隔离目录，不能污染 tracked runtime artifact。

## 备份与恢复审计

每次备份或恢复演练至少记录：

- actor
- 时间
- 目标目录
- 覆盖的数据范围
- 成功/失败状态
- 关联报告路径

## 当前不声明的能力

当前不把以下内容当作已产品化能力：

- 集中式审计平台
- 不可篡改日志存储
- 企业级审批流系统对接
- 自动化 CMDB 变更同步

如果客户需要这些能力，应在实施层单独扩展，而不是把当前仓库误写成已经具备。
