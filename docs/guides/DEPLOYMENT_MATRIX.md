# AGI-Walker Deployment Matrix

更新日期：`2026-04-13`

## 当前主路径

当前唯一的一线客户部署路径是 Docker Compose：

- Compose 文件：[deployment/docker-compose.yml](../../deployment/docker-compose.yml)
- Compose 级变量模板：[deployment/compose.env.example](../../deployment/compose.env.example)
- Web 应用变量模板：[deployment/web_panel.env.example](../../deployment/web_panel.env.example)

以下路径当前不作为客户一线交付承诺：

- `helm/agi-walker`
- `docker-compose.prod.yml`
- Kubernetes / Helm / Istio 模板

## 部署矩阵

| 场景 | 入口 | 组件 | 状态 | 说明 |
| --- | --- | --- | --- | --- |
| 单机评估版 | `docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml up -d --build zenoh-router web-panel` | `zenoh-router`, `web-panel` | 当前支持 | 用于 Web 控制面、workflow、auth、capability matrix。 |
| 单机 distributed 扩展版 | `docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml --profile distributed up -d --build zenoh-router learner sidecar-1 web-panel-distributed` | `zenoh-router`, `learner`, `sidecar-1`, `web-panel-distributed` | 当前支持 | 需要额外准备 Godot TCP 目标。 |
| ROS2 / Godot 扩展节点 | 基于上面的 Compose 主部署，外接 ROS2 bridge 或 Godot | 外部 ROS2 / Godot 进程 | 条件支持 | 主控制面仍是 Compose，自身不是独立部署栈。 |
| 源码开发模式 | `python -m web_panel.server` | 本地 Python 进程 | 开发者专用 | 不是客户默认部署面。 |
| Helm / Kubernetes | 无当前有效入口 | 无 | 当前不支持 | 仓库内保留的历史模板不能视为产品化交付路径。 |

## 目录约定

默认持久化目录由 `AGI_WALKER_RUNTIME_ROOT` 决定，约定如下：

- `<runtime-root>/db`
- `<runtime-root>/workflow_runs`
- `<runtime-root>/workflow_archive`
- `<runtime-root>/backups`

日志约定：

- 应用日志默认走 `docker compose logs`
- 如需留存到宿主机，建议导出到 `<runtime-root>/logs/compose/`

## 选择建议

- 只需要 Web 工作流和客户验收：用单机评估版。
- 需要 actor discovery、learner、sidecar 闭环：用 distributed 扩展版。
- 需要 ROS2 或 Godot：在 Compose 主部署之上外挂扩展节点，不要把 ROS2/Godot 误当成替代控制面。
