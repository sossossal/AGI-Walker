# Testing Guide

更新日期：`2026-04-08`

本页说明 AGI-Walker 当前仓库里应该怎样分层跑测试。重点是优先跑高信号、低成本的验证，再进入重型集成路径。

## 1. 最小高信号测试

先跑这三类：

```bash
python -m pytest tests/test_docs_utf8.py -q
python -m pytest tests/test_mcp_tools.py tests/test_mcp_server.py -q
python tests/run_smoke_tests.py
```

这已经能覆盖：

- 主入口文档是否回退到乱码
- MCP server / tools 是否可用
- CLI、workflow、Web import、Godot fake backend 主线是否通

## 2. 常规 pytest 用法

### 运行全部测试

```bash
python -m pytest
```

### 运行单个文件

```bash
python -m pytest tests/test_cli.py -q
```

### 运行多个高相关测试

```bash
python -m pytest tests/test_workflow_orchestrator.py tests/test_integration_workflows.py -q
```

## 3. 按主题挑测试

### 文档与入口

```bash
python -m pytest tests/test_docs_utf8.py tests/test_skill_docs_utf8.py -q
```

### MCP

```bash
python -m pytest tests/test_mcp_tools.py tests/test_mcp_server.py -q
```

### Skills

```bash
python -m pytest tests/test_skills_loader.py tests/test_skill_validation.py -q
```

### Workflow

```bash
python -m pytest tests/test_workflow_orchestrator.py tests/test_integration_workflows.py -q
```

### Web

```bash
python -m pytest tests/test_web_panel_integration_routes.py tests/test_web_panel_auth_api.py -q
```

### Godot

```bash
python -m pytest tests/test_web_godot_session_bridge.py tests/test_godot_agent_factory.py -q
```

## 4. Smoke 测试

主 smoke runner：

```bash
python tests/run_smoke_tests.py
```

它是当前最值得保留在日常开发循环里的专项验证脚本。
当显式设置相应环境变量时，它也会尝试运行 live smoke，例如 Godot headless 和 ROS2 bridge。
默认 non-live smoke 现在还会覆盖：

- Godot instruction-set smoke
- ROS2 instruction-set smoke
- simulated circuit replay smoke

## 5. 重型集成测试

### 真实 Godot headless smoke

显式 opt-in：

```bash
set AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1
python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live"
```

前提：

- Godot 可执行文件可用
- 场景资源存在

### Distributed smoke

```bash
python tests/run_distributed_smoke.py --build --stop-after --report-file test_env/distributed_smoke/distributed_smoke_report.json
```

前提：

- Docker
- Compose
- Zenoh 相关依赖

说明：

- 该 smoke 是重型集成测试，应作为独立 CI job 或手动/nightly 验收运行。
- 失败时优先查看 `test_env/distributed_smoke/distributed_smoke_report.json`，其中包含逐项 check、服务状态和日志 hint。

### 真实 ROS2 bridge smoke

显式 opt-in：

```bash
set AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1
python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live"
```

前提：

- ROS 2 Humble Python 运行时
- `rclpy`、`sensor_msgs`、`geometry_msgs`、`std_srvs`、`tf2_ros`

说明：

- 该 smoke 使用仓库内的 mock Godot TCP server，不要求真实 Godot。
- 结果会写入 `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json`。

## 6. 什么时候不该跑全量

下面这些改动通常不需要直接跑全量测试：

- 单纯修文档
- 只改 README
- 只改 MCP 文案
- 只改一个小范围 skill metadata

此时优先跑定向测试即可。

## 7. 失败时怎么缩小范围

建议顺序：

1. 先看最近改动涉及哪条主线
2. 只跑那条主线的专项测试
3. 再决定是否升级到 smoke
4. 最后才考虑完整集成测试

## 8. 提交前建议

常见提交前组合：

### 文档 / README / guide

```bash
python -m pytest tests/test_docs_utf8.py -q
```

### MCP / Web / workflow

```bash
python -m pytest tests/test_mcp_tools.py tests/test_mcp_server.py -q
python tests/run_smoke_tests.py
```

### Skills / workflow

```bash
python -m pytest tests/test_skills_loader.py tests/test_workflow_orchestrator.py -q
```

## 结论

AGI-Walker 当前测试策略的关键不是“永远跑全量”，而是按主线分层验证。先守住 docs、MCP 和 smoke，再进入 Godot 或 distributed 的重型集成测试。
