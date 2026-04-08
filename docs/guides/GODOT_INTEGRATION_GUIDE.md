# Godot Integration Guide

更新日期：`2026-04-08`

本页说明 AGI-Walker 当前仓库中的 Godot 集成方式。当前应把 Godot 理解为多条桥接路径的集合，而不是单一接口。

## 1. 三条主要路径

### Legacy Controller API

由 `web_panel/godot_legacy_api.py` 暴露：

- `POST /api/godot/connect`
- `POST /api/godot/disconnect`
- `GET /api/godot/status`
- `POST /api/godot/load-robot`
- `POST /api/godot/start`
- `POST /api/godot/stop`
- `POST /api/godot/update-params`

适合：

- 老的命令式连接流程
- 手工加载机器人配置

### Session Bridge

由 `web_panel/godot_session_bridge.py` 提供：

- `GET /api/simulation/status`
- `POST /api/godot/{session_id}/launch`
- `POST /api/godot/{session_id}/stop`
- `POST /api/godot/{session_id}/control`
- `WS /ws/{session_id}`

适合：

- 会话隔离
- telemetry / control loop
- workflow artifacts 后续联动

### Godot Agent Backend

由 `web_panel/agent_api.py` 与 `agi_walker/integrations/godot_agent/` 提供：

- `GET /api/godot-agent/status`
- `GET /api/godot-agent/templates`
- `POST /api/godot-agent/plan`
- `GET /api/godot-agent/doctor`
- `GET /api/godot-agent/history`
- `POST /api/godot-agent/launch`

兼容别名：

- `GET /api/godot_skills/list`
- `POST /api/godot_skills/apply`

## 2. 当前默认资源

仓库内默认 Godot 项目路径是：

- `godot_project/`

Godot Agent 默认后端规则：

- `legacy`：使用仓库内 `godot_studio_agent`
- `godot-agent`：默认寻找仓库同级目录下的外部 `godot-agent`

## 3. 关键环境变量

```text
AGI_WALKER_GODOT_AGENT_BACKEND=legacy|godot-agent
AGI_WALKER_GODOT_AGENT_DIR=...
AGI_WALKER_GODOT_PROJECT_PATH=...
AGI_WALKER_GODOT_AGENT_HISTORY_FILE=...
GODOT_EXECUTABLE=...
```

## 4. 最小使用方式

推荐先启动 Web Panel：

```bash
python -m web_panel.server
```

然后再决定走哪条 Godot 路径：

- 只想测试旧接口：走 legacy API
- 想做会话化控制：走 session bridge
- 想做模板、计划和历史：走 Godot Agent backend

## 5. 与 workflow 的结合

Web workflow 控制面支持把产物同步到 Godot：

- `POST /api/workflows/runs/{run_id}/artifacts/{artifact_index}/godot-load`
- `POST /api/workflows/runs/{run_id}/godot-sync`

这让 workflow 产物可以直接进入后续场景验证，不必手工中转文件。

## 6. 真实运行前提

要让完整 Godot 链路工作，通常需要同时满足：

- 可用的 Godot 可执行文件
- 有效的 `godot_project/`
- 可建立的 TCP 端口
- 对应场景或模板资源存在

如果缺少这些前提，优先使用 fake backend 或 HTTP 层测试，不要直接假设场景可运行。

## 7. 推荐排错顺序

1. 先看 `/api/godot-agent/status`
2. 再看 `/api/simulation/status`
3. 再测试 legacy `/api/godot/status`
4. 最后再跑真实 headless smoke

## 8. 常见误区

- 不要把 `godot-agent` 外部仓库当作默认必备
- 不要默认所有 GUI 工具都能直接驱动 Godot
- 不要把 headless smoke 当作普通单测

## 结论

AGI-Walker 当前的 Godot 集成是“多路径桥接”。对多数场景，推荐顺序是：先 Web，再 session bridge，再 workflow sync，最后才是完整 headless 或外部 modern backend。
