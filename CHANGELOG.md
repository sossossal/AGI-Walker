# 变更日志

本项目的所有重要变更都将记录在此文件中。

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，
版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

---

## [4.1.0] - 2026-01-21 (OpenNeuro Integration)

### 🚀 重大更新
*   **OpenNeuro 通信框架集成**: 完整集成 Zenoh + ROS 2 生态
*   **硬件部署就绪**: ESP32 固件模板和完整部署文档
*   **ROS 2 深度集成**: 标准 ROS 2 包、URDF 模型、Launch 文件

### ✨ 新增功能

#### Phase 1: Zenoh 基础集成
*   **Zenoh 接口层** (`python_api/zenoh_interface.py`)
    *   统一 Pub/Sub API
    *   自动 JSON 序列化
    *   支持 peer/client 模式
*   **TCP-Zenoh 桥接器** (`python_api/tcp_zenoh_bridge.py`)
    *   保持 Godot TCP 向后兼容
    *   双向转发 (TCP ↔ Zenoh)
*   **ROS 2 节点** (`python_api/ros2_robot_node.py`)
    *   发布 `/robot/joint_states`
    *   订阅 `/robot/joint_commands`

#### Phase 2: ROS 2 深度集成
*   **ROS 2 包结构** (`ros2_ws/src/agi_walker_ros2/`)
    *   标准 `package.xml` 和 `setup.py`
    *   Launch 文件 (`robot.launch.py`)
    *   URDF 机器人描述 (四足机器人)
    *   RViz 配置文件
*   **机器人状态发布器**
    *   TF 树发布
    *   关节状态可视化

#### Phase 3: 硬件部署
*   **ESP32 固件** (`firmware/esp32_neuron/`)
    *   Zenoh-Pico 通信
    *   PWM 舵机控制
    *   传感器数据采集
    *   PlatformIO 项目配置
*   **硬件部署文档** (`docs/HARDWARE_DEPLOYMENT.md`)
    *   完整 BOM 清单
    *   接线图和拓扑图
    *   烧录和调试指南

### 📚 文档更新
*   新增 `docs/OPENNEURO_INTEGRATION.md` - Zenoh/ROS 2 集成指南
*   新增 `docs/HARDWARE_DEPLOYMENT.md` - 硬件部署完整指南
*   更新 `README.md` - 添加 OpenNeuro 特性说明
*   更新 `requirements.txt` - 添加 Zenoh 依赖

### 🔧 优化
*   保持完全向后兼容 (现有 TCP 接口不受影响)
*   模块化设计,可选择性启用 Zenoh/ROS 2
*   完整的示例代码 (`examples/zenoh_ros2_demo.py`)

---

## [4.0.0] - 2026-01-20 (Self-Evolving Era)

### 🚀 重大更新
*   **Sim2Real 闭环**: 实现了完整的仿真到现实落地工具链
*   **模块化构建器**: 允许用户通过 JSON 配置组装基于真实零件的机器人
*   **程序化环境 (PCG)**: 新增 Godot 侧的 `TerrainGenerator`
*   **远程仪表盘**: 基于 TCP 流传输的实时 Python GUI

### ✨ 新增功能
*   **Parts Library**: `python_api/parts_library.json` 包含 10+ 种真实硬件规格
*   **Godot Integration**: `procedural_terrain.gd` 和 `camera_streamer.gd` 脚本
*   **Documentation**: 全面更新 `SIMULATION_GUIDE.md` 至 v4.0 标准
*   **Examples**: 新增 `custom_parts_demo.py` 和 `dashboard_demo.py`

---

## [3.0.0] - 2026-01-18 (AI Integration)
*   集成 IMC-22 神经形态芯片仿真
*   初步实现 RL 训练循环

## [2.0.0] - 2026-01-15 (Parametric Control)
*   参数化物理控制系统
*   基础 Godot-Python 通信协议

## [1.0.0] - 2026-01-10 (Prototype)
*   项目初始化
*   基本的盒子机器人 demo
