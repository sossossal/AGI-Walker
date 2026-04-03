# AGI-Walker V1 状态宣言

**版本**: v1.0.0-RC  
**更新时间**: 2026-04-02

本文件标志着 AGI-Walker V1 计划的正式达成。经过 Week 1-8 的收口工作，项目已从“实验性原型”演进为“具备核心闭环的机器人建模与任务系统”。

## V1 核心达成指标 (100% 完成)

以下主线功能已通过全量 Smoke Test 和回归验证，被视为 V1 稳定入口：

- **Skills & Workflow 闭环**:
  - ✅ 3 个核心 Skills (robot-modeling, parameter-optimizer, urdf-generator) 已收口。
  - ✅ `robot_creation_pipeline` 官方 Workflow 已稳定，支持 real/mock 双模执行。
  - ✅ CLI 指令 (`skills list/validate`, `workflows run/status`) 具备生产级鲁棒性。

- **Web Workflow Console**:
  - ✅ `workflows.html` 已正式产品化，支持实时日志 (SSE)、运行详情、参数覆盖和取消/重试。
  - ✅ 产物保留策略 (Retention Strategy) 已就绪，默认保留 200 条或 30 天运行记录。

- **Godot 官方链路**:
  - ✅ `session_bridge` 已确立为默认传输路径，支持自动同步 Workflow 产物。
  - ✅ Headless Smoke 烟雾测试已覆盖全量 lifecycle (launch/status/schema/load/step/stop)。
  - ✅ `2026-04-02` 在 Windows 本地环境下重新实测通过 `tests/run_smoke_tests.py` 与 `tests/test_godot_headless_smoke.py`。
  - ✅ 默认场景 `demo_generated_biped.tscn` 与 runner 场景 `run_rl_server.tscn` 均已通过 headless lifecycle 验收。

- **Nightly 运维闭环**:
  - ✅ `Nightly 运维页` (/static/nightly.html) 已上线，实时追踪 GitHub Actions 专项回归。
  - ✅ 具备标准化诊断产物与 Issue 模板，支持快速复现与故障排查。

## 当前项目定位

截至 2026-04-02，AGI-Walker v1.0.0 是：

**一个面向机器人建模、参数优化与 Godot 集成验证的标准化工作流平台。**

它为开发者提供了从“参数化设计”到“物理仿真验证”的最小可信路径。

## 最近验收快照

`2026-04-02` 的补充验收结果：

- `python tests/run_smoke_tests.py --output-root test_env/smoke_runs/v1_plan_check_after_fix`：通过
- `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 python -m pytest tests/test_godot_headless_smoke.py -q -m integration --tb=short -vv`：通过
- `AGI_WALKER_GODOT_HEADLESS_SCENE=run_rl_server.tscn` 后复跑同一条 headless smoke：通过

这意味着此前 V1 验收口径中唯一需要继续证明的 `godot-headless-smoke` 已完成闭环验证。

## 2026-04-03 额外检验

- `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1` 叠加 `AGI_WALKER_GODOT_HEADLESS_SCENE=run_rl_server.tscn`：headless smoke 再次通过，确认 runner 场景完整 lifecycle 成功，生成的 `headless_smoke_report.json` 中 `failure_stage` 为 `success`。
- `docker compose -f docker-compose.prod.yml up --build`：Web API、worker、Redis、PostgreSQL、Prometheus 及 Grafana 镜像都能部署并进入健康状态；Grafana 仍无法绑 3000 端口因为 Windows 在 `netsh interface ipv4 show excludedportrange protocol=tcp` 中保留 2970-3069，所以只能基于其他端口（4001+）启动，待系统管理员解除排除后再用默认端口重跑。
- `python tests/run_distributed_smoke.py --build`：在授予 Docker 配置读写权限后构建与 distributed smoke 均通过，`distributed_monitor.monitor_active`、Godot actor、nightly status 都显示 ready，summary 显示 `PASS`。

`2026-04-03` 的这些检验进一步巩固了 V1 “production smoke + godot headless + distributed path” 的可用性。

## 历史文档说明

仓库中 `docs/archive_and_reports/` 与 `docs/architecture/` 下的早期文档 (2026-01 以前) 仅作为演进参考。V1 的权威口径请以本文件及根目录 `README.md` 为准。

## 推荐阅读顺序

1. [README.md](../README.md)
2. [CLI 使用指南](guides/CLI_GUIDE.md)
3. [Web 面板指南](guides/WEB_PANEL_GUIDE.md)
4. [部署手册](../PRODUCTION_DEPLOYMENT_RUNBOOK.md)
5. [Smoke 测试集](../tests/run_smoke_tests.py)
