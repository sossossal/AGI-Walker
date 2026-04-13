# Simulation Guide

更新日期：`2026-04-12`

本页描述 AGI-Walker 当前仓库里的仿真路径。当前应把“仿真”理解为多条并存的通路，而不是单个统一引擎。

## 1. 当前几条仿真路径

### Godot Robot Environment

文件：

- `agi_walker/core/api/godot_robot_env/gym_env.py`

作用：

- 提供 `gymnasium.Env` 兼容接口
- 通过 TCP 与 Godot 通信
- 支持 reset / step / reward / termination
- 支持随机化物理参数

### Web + Godot Session Bridge

文件：

- `web_panel/godot_session_bridge.py`

作用：

- 管理 Godot 进程和会话
- 提供会话级 telemetry / control 通道
- 与 Web API 和 workflow artifact 同步结合

### Legacy Godot API

文件：

- `web_panel/godot_legacy_api.py`

作用：

- 提供旧的 connect / load / start / stop / update-params 风格控制接口

## 2. 推荐使用顺序

对当前仓库，建议顺序如下：

1. 先跑 CLI / workflow
2. 再启动 Web Panel
3. 需要会话化控制时使用 session bridge
4. 需要 RL 训练时才接入 `GodotRobotEnv`

## 3. `GodotRobotEnv` 能做什么

当前 `GodotRobotEnv` 提供：

- `reset()` 时发送 `robot_config`、`physics_config` 和随机化参数
- `step(action)` 时发送动作并接收观测
- 内置 reward 计算
- 内置 termination / truncation 判断
- 占位的 `rgb_camera` 和 `elevation_map` 观测槽位

默认通信目标：

- host `127.0.0.1`
- port `9999`

## 4. `GodotRobotEnv` 的限制

需要明确几点：

- 它依赖真实 Godot TCP 服务
- 多模态观测中的图像与高程图默认是占位值
- 示例不等于已完整接通所有视觉传感器

因此它更适合作为仿真接口骨架，而不是“开箱即用的通用训练环境”。

## 5. Web 仿真相关入口

当前常用 Web 路由：

- `GET /api/simulation/status`
- `POST /api/godot/{session_id}/launch`
- `POST /api/godot/{session_id}/stop`
- `POST /api/godot/{session_id}/control`
- `WS /ws/{session_id}`

另外 workflow run 也可以把产物同步到 Godot：

- `POST /api/workflows/runs/{run_id}/artifacts/{artifact_index}/godot-load`
- `POST /api/workflows/runs/{run_id}/godot-sync`

## 6. 真实 Godot Headless Smoke

真实 headless smoke 是显式 opt-in：

```bash
python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live"
```

前提：

- `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1`
- 可用的 Godot 可执行文件
- 对应场景存在

它验证的是完整生命周期，而不是 import。

## 7. RL 训练中的仿真定位

在当前仓库里，仿真和 RL 的衔接主要通过：

- `GodotRobotEnv`
- `RLOptimizer`
- ONNX 导出和部署

当前 `RLOptimizer.train()` 已开始复用 `training_run` artifact v1，并会在保存目录写出 `training_run_manifest.json`。默认分类规则是：

- 环境类名为 `DummyEnv` 时标记为 `mock_training`
- 普通仿真环境标记为 `sim_training`
- 显式传入 `hardware_required=True` 时标记为 `hardware_in_the_loop`

这意味着当前仓库里，“仿真训练”和“硬件在环训练”的边界已经开始从代码层固定，而不是只靠文档说明。

但很多高阶训练示例仍依赖额外环境和第三方库，不应默认写成随仓库立即可跑。

## 8. 推荐验证命令

```bash
python tests/run_smoke_tests.py
python -m pytest tests/test_web_godot_session_bridge.py -q
python -m pytest tests/test_rl_optimizer_training_contract.py -q
python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live"
```

## 结论

AGI-Walker 当前的仿真主线是 Godot 通信、Web session bridge 和可选的 Gym 训练接口。训练侧已经开始产出统一 manifest，但 live Godot/headless 验证仍保持显式 opt-in。多数场景先用 Web + workflow 验证，再进入 sim training 或 headless smoke，会更稳妥。
