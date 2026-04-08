# Distributed Guide

更新日期：`2026-04-08`

本页说明 AGI-Walker 当前仓库里的 distributed 路径。它主要面向需要验证 Zenoh + learner + sidecar + Web 监控联动的场景，不属于默认本地快速上手流程。

## 1. 当前组件

distributed smoke 当前围绕这些服务工作：

- `zenoh-router`
- `learner`
- `web-panel-distributed`
- `mock-godot`
- `sidecar-1`

它们的编排入口在：

- `deployment/docker-compose.yml`
- `tests/run_distributed_smoke.py`

## 2. 代码角色

### Learner

文件：

- `agi_walker/core/distributed/run_learner.py`

职责：

- 订阅 `ag/*/obs`
- 计算动作
- 回写 `ag/<actor>/act`

### Sidecar

文件：

- `agi_walker/core/distributed/sidecar.py`

职责：

- 连接 Godot TCP
- 把 observation 发到 Zenoh
- 接收 cloud action
- 做本地/云端 offloading 决策

### Web Distributed Monitor

文件：

- `web_panel/distributed_monitor.py`

职责：

- 订阅 actor 观测数据
- 聚合活跃 actor 状态
- 向 Web 广播 `distributed_update`

## 3. 环境变量

当前分布式路径常见变量：

- `AGI_WALKER_ZENOH_ENDPOINT`
- `AGI_WALKER_DISTRIBUTED_ACTOR_TTL_SECONDS`
- `ZENOH_ROUTER`
- `AGI_WALKER_SIDECAR_ACTOR_ID`
- `AGI_WALKER_GODOT_HOST`

需要注意：

- monitor 侧默认 endpoint 是 `tcp/127.0.0.1:7447`
- actor TTL 默认是 `30` 秒

## 4. 推荐启动方式

优先使用现成 smoke runner：

```bash
python tests/run_distributed_smoke.py --build --stop-after
```

这个脚本会负责：

- 拉起 compose 服务
- 等待 Web distributed monitor ready
- 检查 actor 是否出现在 `/api/distributed/status`
- 检查 mock-godot 是否收到 step action

不要先手工散着启动多个服务，排错成本会更高。

## 5. Web 验证点

分布式路径最关键的两个 HTTP 入口：

- `/api/system/status`
- `/api/distributed/status`

你需要重点关注：

- `distributed_monitor.monitor_active`
- `actors`
- `endpoint`
- `last_error`

## 6. 本地开发建议

如果你只是做一般功能开发，不需要默认打开 distributed 模式。建议顺序：

1. 先跑 CLI / workflow
2. 再跑普通 Web Panel
3. 确认主线稳定后，再进入 distributed smoke

## 7. 常见失败点

### Zenoh 没连上

检查：

- router 是否启动
- endpoint 是否一致
- 容器网络是否可达

### Actor 不出现

检查：

- `sidecar-1` 是否启动
- `AGI_WALKER_SIDECAR_ACTOR_ID` 是否与预期一致
- mock-godot 是否可用

### Web 监控未激活

检查：

- `web-panel-distributed` 日志
- `/api/system/status`
- `DistributedMonitor.last_error`

## 8. 何时使用 distributed

当前更适合这些场景：

- 端到端部署冒烟
- Zenoh 观察链路验证
- learner / sidecar 回路验证
- 夜间或 CI 集成检查

不适合：

- 纯文档修复
- 单机 workflow 调试
- skills 元数据开发

## 结论

AGI-Walker 的 distributed 路径已经有清晰的烟测入口，但它仍是重型集成能力。优先把它当作专项验证链，而不是日常开发默认模式。
