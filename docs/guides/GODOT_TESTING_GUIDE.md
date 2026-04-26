# Godot Testing Guide

更新日期：`2026-04-15`

本页说明如何测试 AGI-Walker 当前的 Godot 相关能力。重点是把“便宜测试”和“重型集成测试”分开。

当前本页已被 `extension_execution_plan.profiles[godot_extension].runbook_entrypoints` 正式引用，并与同一 profile 的 `execution_template` 保持一致。若 Godot 扩展的专项验收命令或环境前提变化，应同步更新本页和 machine-readable execution plan。

## 1. 先跑便宜测试

推荐先从不依赖真实 Godot 可执行文件的测试开始。

### Web / Agent / Session Bridge

```bash
python -m pytest tests/test_web_godot_session_bridge.py -q
python -m pytest tests/test_godot_agent_factory.py tests/test_godot_agent_adapter.py -q
python -m pytest tests/test_web_godot_integration.py tests/test_web_panel_integration_routes.py -q
```

### 结构化指令集 / 模拟电路 smoke

```bash
python tests/run_smoke_tests.py
```

默认 smoke 现在还会覆盖：

- Godot session bridge `instruction_set`
- Godot session bridge `simulated_circuit`
- simulated circuit replay 非 live 闭环

### Smoke Runner 中的 fake backend

```bash
python tests/run_smoke_tests.py
```

这个 smoke runner 已经包含：

- fake Godot Agent backend 检查
- modern backend 预检
- Web Panel import 检查

如果你要把 Godot instruction-set / ROS2 instruction-set / simulated circuit replay 作为一套完整演示链来跑，直接使用：

- `docs/guides/INSTRUCTION_CONTROL_DEMO_RUNBOOK.md`

## 2. 真实 headless smoke 是 opt-in

真实 Godot headless smoke 只会在显式开启时运行。

最低要求：

- `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1`
- 可用的 Godot 可执行文件
- 真实存在的场景

常用变量：

```text
AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1
GODOT_EXECUTABLE=...
AGI_WALKER_GODOT_HEADLESS_SCENE=demo_generated_biped.tscn
AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR=test_env/godot_headless_smoke
```

运行方式：

```bash
python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live"
```

## 3. 真实 headless smoke 会验证什么

当前 `tests/test_godot_headless_smoke.py` 会检查：

- 找到 Godot 可执行文件
- 拉起 headless 场景
- 建立 TCP 连接
- 获取 schema
- 发送机器人载荷
- 拉一段 step loop
- 写出结构化报告

这不是单纯的 import test，而是完整生命周期验证。

## 4. 报告与工件

headless smoke 会写诊断报告到：

- 默认 `test_env/godot_headless_smoke/`

可通过环境变量重定向：

- `AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR`

## 5. Modern backend 测试

如果要测试 modern `godot-agent` backend，需要满足至少其一：

- 设置 `AGI_WALKER_GODOT_AGENT_DIR`
- 仓库同级目录存在 `godot-agent`

切换方式：

```text
AGI_WALKER_GODOT_AGENT_BACKEND=godot-agent
```

如果没有外部仓库，就不要把这条路径当成默认失败。

## 6. Legacy API 测试

如果你改了旧 TCP 控制路径，优先补这些方向：

- legacy `/api/godot/*` 路由
- `godot_controller`
- Web 层状态接口

推荐搭配：

```bash
python -m pytest tests/test_godot_client.py tests/test_godot_client_unit.py -q
python -m pytest tests/test_web_panel_aux_apis.py tests/test_web_panel_integration_routes.py -q
```

## 7. 什么时候不该跑 Godot 测试

这些场景通常不需要：

- 只改 README 或 docs
- 只改 skills metadata
- 只改 MCP 文档或初始化逻辑

此时用 docs / MCP / workflow 相关测试就够了。

## 8. 推荐测试顺序

1. `python tests/run_smoke_tests.py`
2. `pytest` 跑 Godot 相关轻量测试
3. 检查 modern backend 预检
4. 最后才启用真实 headless smoke

## 结论

Godot 测试应分层执行。便宜测试负责守住接口与桥接层，真实 headless smoke 只在你明确需要验证完整生命周期时再开启。
