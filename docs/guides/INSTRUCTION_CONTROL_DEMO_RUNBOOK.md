# Instruction Control Demo Runbook

更新日期：`2026-04-25`

本页给出一条最短的演示路径，用于展示当前已经产品化的：

- Godot `instruction_set`
- ROS2 `instruction_set`
- simulated circuit replay

默认入口使用：

```bash
python tools/run_instruction_control_validation.py --output-root test_env/instruction_control_validation
```

## 1. 适用范围

这条 runbook 适合：

- 本地演示
- 功能验收
- 交付前非 live 验证

它不替代真实现场环境，但可以快速证明以下链路已经打通：

- Web/API → Godot session bridge
- ROS2 bridge → instruction runtime
- simulated circuit → replay feedback

## 2. 演示前前置条件

最小前提：

- 当前仓库依赖已安装
- `python tests/run_smoke_tests.py` 可运行
- 非 live 测试环境可用

建议先确认：

```bash
python -m pytest tests/test_instruction_control_contracts.py tests/test_ros2_bridge_runtime.py tests/test_web_godot_session_bridge.py -q
```

## 3. 一键演示命令

执行：

```bash
python tools/run_instruction_control_validation.py --output-root test_env/instruction_control_validation
```

成功后会生成：

- `test_env/instruction_control_validation/smoke_report.json`
- `test_env/instruction_control_validation/instruction_control_validation_report.json`
- `test_env/instruction_control_validation/godot_instruction_smoke/godot_instruction_smoke_report.json`
- `test_env/instruction_control_validation/ros2_instruction_smoke/ros2_instruction_smoke_report.json`
- `test_env/instruction_control_validation/simulated_circuit_smoke/simulated_circuit_replay_smoke_report.json`

## 4. 演示时最该看什么

优先看总摘要：

- `instruction_control_validation_report.json`

关键字段：

- `status`
- `smoke_report.status`
- `specialized_reports.godot_instruction_smoke.status`
- `specialized_reports.ros2_instruction_smoke.status`
- `specialized_reports.simulated_circuit_replay_smoke.status`

判定标准：

- 总体 `status=passed`
- 三条专项 smoke 全部 `passed`

## 5. 三条专项分别说明什么

### Godot instruction smoke

证明：

- Godot instruction API 可收包
- instruction payload 已进入 runtime
- simulated circuit config 可同时下发

对应报告：

- `godot_instruction_smoke_report.json`

### ROS2 instruction smoke

证明：

- ROS2 bridge 可接收 instruction-set payload
- runtime payload 可生成
- circuit config 与兼容参数可同时透传

对应报告：

- `ros2_instruction_smoke_report.json`

### Simulated circuit replay smoke

证明：

- command batch 已投影到 IMC-22 replay payload
- replay feedback 已回流到结构化状态

对应报告：

- `simulated_circuit_replay_smoke_report.json`

## 6. 失败时怎么定位

先看：

- `instruction_control_validation_report.json`

再按专项报告定位：

- Godot 侧失败：看 `godot_instruction_smoke_report.json`
- ROS2 侧失败：看 `ros2_instruction_smoke_report.json`
- replay 侧失败：看 `simulated_circuit_replay_smoke_report.json`

如果总 smoke 失败，再看：

- `smoke_report.json`

## 7. 如需进入 live 验证

本页只覆盖非 live 演示入口。若要继续进入 live：

- Godot headless：见 `docs/guides/GODOT_TESTING_GUIDE.md`
- ROS2 本机环境：见 `docs/guides/ROS2_LOCAL_ENV_CHECKLIST_20260425.md`
- Windows ROS2 引导：见 `docs/guides/ROS2_WINDOWS_BOOTSTRAP_20260425.md`

## 8. 推荐对外表述

如果这条 runbook 执行通过，可以对内或对客户实施侧表述为：

- instruction-set 控制面已完成非 live 验证
- ROS2 指令集桥接已完成非 live 验证
- simulated circuit replay 闭环已完成非 live 验证
- 可继续进入专项 live 验证或现场交付验证
