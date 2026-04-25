# AGI-Walker Support Matrix

更新日期：`2026-04-15`

本页给出当前客户交付面的正式支持边界。它只覆盖当前一线交付路径，不把实验性入口、历史模板或未进入验收主链的环境算作默认支持面。

容量与规模声明见 `CAPACITY_AND_SCALE.md`；若客户目标超出本页支持矩阵，应先回到容量声明和验收清单补做专项确认。

当前 `release_manifest.customer_delivery_surface.extension_support_surface` 会把本页与 `KNOWN_LIMITATIONS.md`、`CUSTOMER_ACCEPTANCE_CHECKLIST.md` 的扩展边界压成机器可读字段，供 industrial gate、readiness 和 customer acceptance bundle 直接消费。
现在每个扩展 profile 还会附带 `deployment_commands`、`acceptance_checks` 和 `rollback_prerequisites`，避免支持边界只停留在声明层。

## 当前支持矩阵

| 维度 | 当前支持状态 | 当前基线 | 说明 |
| --- | --- | --- | --- |
| 部署路径 | 支持 | Docker Compose | 当前唯一的一线客户部署入口；见 `deployment/docker-compose.yml`。 |
| Helm / Kubernetes | 不支持 | 无 | 仓库中的历史模板不构成当前客户交付承诺。 |
| 主机 OS | 条件支持 | Windows PowerShell / Ubuntu 22.04 | Windows 是当前本地操作与交付基线；Ubuntu 22.04 已进入 CI 与 ROS2 Humble 验证面。 |
| Python 运行时 | 支持 | `>=3.10` | `pyproject.toml` 当前要求 `>=3.10`；自动化矩阵覆盖 `3.10` 与 `3.11`。 |
| Python 3.12 | 条件支持 | 工程验证 | 当前工程环境可运行，但尚未进入正式 CI 版本矩阵。 |
| Docker | 支持 | Docker Engine + `docker compose` v2 | 当前安装、升级、回滚和 customer acceptance 都基于 Compose v2 命令。 |
| `docker-compose` v1 | 不支持 | 无 | 文档与脚本不再维护 v1 命令兼容。 |
| ROS2 | 条件支持 | ROS2 Humble | 仅在需要 bridge 扩展节点时进入客户范围；当前 live smoke 证据来自 Humble runtime。 |
| Godot | 条件支持 | 外部 Godot TCP 目标或 headless 环境 | 不是基础部署前提；distributed / headless 场景都要求显式环境准备。 |
| 浏览器 | 条件支持 | 最新桌面 Chromium 内核浏览器 | 当前客户默认基线是桌面 Chrome / Edge；Firefox、Safari 和移动浏览器未进入正式验收矩阵。 |
| 数据库 | 支持 | SQLite | 默认控制面数据库是 SQLite，适合单机部署，不声明 HA 集群能力。 |

## 扩展边界

| 扩展面 | 当前状态 | 默认纳入基础交付 | 专项验收要求 | 当前非支持范围 |
| --- | --- | --- | --- | --- |
| distributed profile | 条件支持 | 否 | 需要 `distributed_runtime_live` structured evidence，并在 Compose 主部署之外显式启用 distributed profile | 不承诺零配置默认启用，也不把 distributed 当作基础控制面替代入口 |
| ROS2 bridge extension | 条件支持 | 否 | 需要 `ros2_bridge_live` structured evidence，且目标运行时为 ROS2 Humble | 其他 ROS2 发行版未进入正式客户矩阵 |
| Godot extension | 条件支持 | 否 | 需要 `godot_headless_live` structured evidence 或等价专项说明，并提前准备外部 Godot TCP / headless 环境 | 不承诺零配置 Godot 环境交付 |
| Helm / Kubernetes delivery | 不支持 | 否 | 无当前专项验收路径 | 历史模板不构成当前工业交付承诺 |

## 机器可读扩展动作

- `distributed_profile`：默认部署命令为 `docker compose ... --profile distributed up -d --build zenoh-router learner sidecar-1 web-panel-distributed`；默认专项验收命令为 `python tests/run_distributed_smoke.py --build --stop-after --report-file test_env/distributed_smoke/distributed_smoke_report.json` 与 `GET /api/distributed/status`；回滚前提是保留 distributed profile 的 env 快照，并先备份 `<runtime-root>/db`、`workflow_runs`、`workflow_archive`。
- `ros2_bridge_extension`：默认实施动作是 `hardware/ros2_ws` 下的 `colcon build` 与 `ros2 launch agi_walker_ros2 agi_walker.launch.py`；专项验收命令是 `ros2 service call /start_simulation ...` 与 `AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1 python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv`；回滚前提是保留 ROS2 Humble 环境、launch/params 配置和当前 topic/service 连通性记录。
- `godot_extension`：默认实施动作是显式配置 `GODOT_EXECUTABLE`、`AGI_WALKER_GODOT_HEADLESS_SCENE` 与 artifact 目录；专项验收命令是 `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv`；回滚前提是保留上一个已验收的 scene、可执行文件路径和 backend 选择。
- `kubernetes_delivery`：当前没有受支持的部署命令；专项验收应直接回退到 Docker Compose 基线；如果客户仍要求 Kubernetes，必须在 release signoff 前停止 promotion，而不是把历史模板视为默认支持。

## 验证口径

- 非 live 主链：`python -m pytest -m "not live" -q`
- clean checkout smoke：`python tools/run_clean_checkout_smoke.py --output-root test_env/clean_checkout_smoke_real`
- release evidence：`python tools/collect_release_evidence.py --output-root test_env/release_evidence`
- security preflight：`python tools/run_security_release_preflight.py --output-root test_env/release_evidence`
- distributed / Godot / ROS2 live 证据：保持 opt-in，只在目标环境显式收集

## 使用建议

- 只做客户控制面部署：使用 Docker Compose + Web Panel。
- 需要 distributed runtime：在 Compose 主部署基础上显式启用 distributed profile。
- 需要 ROS2 / Godot：把它们视为扩展节点或扩展环境，不要把它们当成基础控制面的替代入口。
- 需要超出本页边界的环境组合：应先走专项验收，不要默认视为已支持。
