# 下一阶段开发执行计划

更新日期：`2026-04-26`

本页用于承接当前项目从“主线已闭环”进入“产品面深化”的下一阶段工作。

当前已经具备：

- 本地最小部署
- distributed 本地链路
- Godot headless smoke
- ROS2 bridge smoke
- Godot / ROS2 / simulated circuit instruction-control 主干
- 演示与非 live 验收入口

因此下一阶段的重点不再是补主线，而是补产品使用面、真实硬件面、ROS2 生态标准化和交付深度。

## 1. 总体优先级

建议按以下顺序推进：

1. Web 控制台深化
2. 真实硬件闭环
3. ROS2 标准化与生态接入
4. 交付与运维产品化

原因：

- 当前后端 contract 与 smoke 主线已经稳定
- 最短价值路径是先把能力做成更好用的操作面
- 最大真实性缺口在真实硬件闭环
- ROS2 和交付面适合在前两者稳定后继续扩展

## 2. 路线 A：Web 控制台深化

### 目标

把当前可用的 instruction-control 能力，从 API / WebSocket 能用，推进到真正可操作的工程控制台。

### 当前已具备

- `POST /api/godot/instruction-set`
- `POST /api/godot/simulated-circuit`
- session 级 instruction/circuit 路由
- WebSocket 指令集消息类型

### 还需实现

- instruction-set 可视化编排面板
- simulated circuit 参数面板
- runtime telemetry 状态面板
- replay 结果与错误态展示
- session 历史与最近一次执行记录

### 建议落点

- `web_panel/server.py`
- `web_panel/workflows_api.py`
- `web_panel/static/`
- `web_panel/core_api.py`

### 完成标准

- 用户可在 Web 面提交 instruction-set
- 可查看 runtime telemetry 与 simulated circuit feedback
- 可直接从控制台重放最近一次 payload

## 3. 路线 B：真实硬件闭环

### 目标

把当前 simulated circuit replay 闭环推进到真实硬件 adapter / device 层。

### 当前薄弱点

- 目前 IMC-22 还是 replay/feedback 模拟
- 未覆盖真实总线错误、时序抖动、硬件失联、限幅/故障恢复

### 还需实现

- 真实 CAN / serial / device adapter
- 设备发现与健康检查
- 硬件错误码与恢复策略
- 命令限幅与 watchdog
- 真实 telemetry 映射到当前 runtime contract

### 建议落点

- `agi_walker/core/api/godot_robot_env/hardware_controller.py`
- `agi_walker/core/api/comm/`
- `hardware/`

### 完成标准

- 同一份 instruction-set/circuit contract 可驱动真实 adapter
- runtime telemetry 可同时覆盖 simulated 与 real hardware

## 4. 路线 C：ROS2 标准化与生态接入

### 目标

把当前 ROS2 bridge 从“桥接可用”推进到“标准接口层可复用”。

### 当前已具备

- instruction-set topic
- simulated circuit topic
- runtime telemetry publisher
- replay / apply-default service

### 还需实现

- 标准化 message / service 定义
- launch profile 分层
- replay fixture 与 bag 级测试
- 多节点协同 smoke
- 更深层的 behavior / navigation / perception 接口挂接

### 建议落点

- `hardware/ros2_ws/src/agi_walker_ros2/`
- `hardware/ros2_ws/launch/`
- `docs/ros2/`

### 完成标准

- ROS2 控制面不再主要依赖 JSON string payload
- 标准 launch 可直接起控制/回放/观测链

## 5. 路线 D：交付与运维产品化

### 目标

把现有 smoke / runbook / signoff 继续推进成 operator-facing 的交付体系。

### 当前已具备

- release/signoff/readiness 主链
- instruction-control validation runner
- 演示 runbook

### 还需实现

- operator runbook
- 故障树与恢复流程
- 版本兼容矩阵
- 监控与告警基线
- SLA / SLO 视图
- 现场交付 checklist 自动化

### 建议落点

- `docs/guides/`
- `tools/`
- `agi_walker/ops/`

### 完成标准

- 非研发角色也能按文档完成演示、验收和基础排障

## 6. 推荐的下一步迭代节奏

### Iteration 1

- Web instruction console 最小可用版
- runtime telemetry 面板
- replay 最近一次 payload

### Iteration 2

- 真实硬件 adapter 抽象层
- 真实 telemetry 对齐 runtime contract
- circuit error handling

### Iteration 3

- ROS2 标准 message / service 定义
- launch profile 与 replay smoke

### Iteration 4

- operator-facing 交付与运维 runbook
- 交付 checklist 自动化

## 7. 当前最需要注意的风险

- 如果直接跳到真实硬件，而不先稳定 Web/操作面，调试成本会明显上升
- 如果继续只扩 contract 而不补产品 UI，项目会长期停留在“工程能力强、使用面偏硬”的状态
- 如果 ROS2 长期保持 JSON string bridge 形态，后续生态复用成本会偏高

## 8. 建议结论

当前最优先的方向是：

- 先做 `Web 控制台深化`

之后立即衔接：

- `真实硬件闭环`

ROS2 与交付面继续跟进，但不建议先于上述两条主线投入更大成本。
