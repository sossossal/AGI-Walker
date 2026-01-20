# 变更日志

本项目的所有重要变更都将记录在此文件中。

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，
版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

---

## [4.0.0] - 2026-01-20 (Self-Evolving Era)

### 🚀 重大更新
*   **Sim2Real 闭环**: 实现了完整的仿真到现实落地工具链，包括 `Sim2RealAnalyzer` 和 `PhysicsCalibrator`。
*   **模块化构建器 (Modular Builder)**: 允许用户通过 JSON 配置组装基于真实零件（Unitree/Tesla）的机器人。
*   **程序化环境 (PCG)**: 新增 Godot 侧的 `TerrainGenerator`，基于 Perlin 噪声动态生成训练地形。
*   **远程仪表盘 (Remote Dashboard)**: 基于 TCP 流传输的实时 Python GUI，可远程查看 Godot 仿真画面。

### ✨ 新增功能
*   **Parts Library**: `python_api/parts_library.json` 包含 10+ 种真实硬件规格。
*   **Godot Integration**: `procedural_terrain.gd` 和 `camera_streamer.gd` 脚本。
*   **Documentation**: 全面更新 `SIMULATION_GUIDE.md` 至 v4.0 标准，包含 GUI 和 Builder 指南。
*   **Examples**: 新增 `custom_parts_demo.py` 和 `dashboard_demo.py`。
*   **Analysis**: `Sim2RealAnalyzer` 支持功率谱密度 (PSD) 对比分析。

### 🔧 优化
*   重构 `README.md`，反映项目从原型到生产就绪的状态。
*   优化 `gym_env.py`，支持动态地形种子 (`terrain_seed`) 的重置。
*   统一了机器人模型基类 `base_robot.py`，支持从零件列表初始化。

---

## [3.0.0] - 2026-01-18 (AI Integration)
*   集成 IMC-22 神经形态芯片仿真。
*   初步实现 RL 训练循环。

## [2.0.0] - 2026-01-15 (Parametric Control)
*   参数化物理控制系统。
*   基础 Godot-Python 通信协议。

## [1.0.0] - 2026-01-10 (Prototype)
*   项目初始化。
*   基本的盒子机器人 demo。

---

## [0.9.0-beta] - 2026-01-16 (Legacy)
*(保留用于历史参考)*
...
