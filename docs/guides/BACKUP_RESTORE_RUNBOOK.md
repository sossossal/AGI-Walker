# AGI-Walker Backup And Restore Runbook

更新日期：`2026-04-13`

本页描述当前 Docker Compose 交付路径下的最小备份与恢复闭环。它只覆盖当前已正式支持的持久化目录，不声明数据库集群、对象存储或外部备份平台。

## 备份对象

当前必须纳入备份范围的目录：

- `<runtime-root>/db`
- `<runtime-root>/workflow_runs`
- `<runtime-root>/workflow_archive`
- `<runtime-root>/backups`

当前默认 `runtime-root` 由 `deployment/compose.env` 中的 `AGI_WALKER_RUNTIME_ROOT` 控制，示例值为 `./runtime`。

建议一并归档以下配置文件副本：

- `deployment/compose.env`
- `deployment/web_panel.env`
- 当前使用的 `release_manifest.json`

## 备份频率

当前建议的最小频率：

- `db`：每日
- `workflow_runs`：每日
- `workflow_archive`：每日
- `deployment/*.env`：每次变更后立即归档

如果客户环境频繁执行 workflow 或保留大量产物，应缩短周期并对 archive 做独立保留策略。

## 备份步骤

1. 确认当前 Compose 工程根目录和 `AGI_WALKER_RUNTIME_ROOT`。
2. 记录部署对应的 `release_manifest` 路径。
3. 将 runtime 目录整体复制到日期化备份目录。
4. 复制 `deployment/compose.env` 和 `deployment/web_panel.env` 的当前版本。
5. 记录备份完成时间、操作者和目标路径。

最小示例：

```bash
mkdir -p runtime/backups/2026-04-13
cp -R runtime/db runtime/backups/2026-04-13/
cp -R runtime/workflow_runs runtime/backups/2026-04-13/
cp -R runtime/workflow_archive runtime/backups/2026-04-13/
cp deployment/compose.env runtime/backups/2026-04-13/compose.env
cp deployment/web_panel.env runtime/backups/2026-04-13/web_panel.env
```

Windows 环境可使用等价目录复制方案，但要求目标目录与源码树分离。

## 恢复目标

当前最小恢复目标：

- Web Panel 可重新启动
- 数据库可读
- workflow runs / archive 可重新浏览
- 当前部署配置可重建

当前建议目标值：

- RPO：`24 小时`
- RTO：`4 小时`

这是当前最小工业交付基线，不代表高可用承诺。

## 恢复步骤

1. 停止当前 Compose 服务。
2. 校验目标备份目录完整性。
3. 恢复 `db`、`workflow_runs`、`workflow_archive`。
4. 恢复 `deployment/compose.env` 与 `deployment/web_panel.env`。
5. 使用当前唯一一线入口重新拉起服务：

```bash
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml up -d --build zenoh-router web-panel
```

6. 执行健康检查：

```bash
python tests/run_smoke_tests.py --output-root test_env/smoke_backup_restore_validation
```

7. 记录恢复结果、耗时和异常。

## 恢复演练报告

每次恢复演练至少保留：

- 演练时间
- actor
- 备份来源路径
- 恢复目标路径
- 关联 release manifest
- 健康检查结果
- 是否达到 RTO / RPO

当前阶段 D 的退出门禁之一，是至少有一次恢复演练报告可归档，而不是只有 runbook 文本。
