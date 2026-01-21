# AGI-Walker: 自进化通用机器人 AI 平台

<div align="center">

![AGI-Walker](https://img.shields.io/badge/AGI--Walker-v4.0-blue)
![Python](https://img.shields.io/badge/Python-3.10+-green)
![RL](https://img.shields.io/badge/RL-Stable--Baselines3-orange)
![License](https://img.shields.io/badge/License-MIT-yellow)
![Status](https://img.shields.io/badge/Status-Production%20Ready-brightgreen)

**从仿真到现实：具备自动化进化能力的通用机器人大脑**

[English](README_EN.md) | 简体中文

</div>

---

## 📖 项目简介

AGI-Walker 是一个先进的**具身智能 (Embodied AI)** 开发平台，旨在构建通用、鲁棒且具备自我进化能力的机器人控制系统。

与传统仿真平台不同，AGI-Walker 引入了**自动化进化循环 (Evolution Loop)**，能够无人值守地完成“数据收集 -> 自动标记 -> 模型微调”的闭环，让机器人越用越聪明。支持从简单的 PID 控制到复杂的 Vision-Language-Action (VLA) 多模态策略。

### 🌟 核心突破 (v4.0)

*   **🔄 自动化进化**: 集成 `EvolutionManager`，实现全流程自动化迭代。
*   **🌍 Sim2Real 闭环**: 独有的数据差异分析器 (`GapAnalyzer`) 和在线参数校准，大幅缩小虚实差距。
*   **👁️ 多模态感知**: 集成 SigLIP 视觉编码器和局部高程图构建，支持复杂地形盲走。
*   **🧩 模块化架构**: 统一控制双足、四足和轮式机器人，一套大脑，多种形态。
*   **📡 OpenNeuro 通信** (NEW): 集成 Zenoh + ROS 2，支持分布式机器人和硬件部署。

---

## ✨ 核心功能模块

### 1. 自动化进化循环 (Evolution Loop)
无人值守的自我学习引擎，将模型迭代周期缩短至小时级。
*   **RL Explorer**: 基于 Stable-Baselines3 (PPO/SAC) 进行探索。
*   **Auto Labeler**: 利用 LLM (Ollama) 自动评估轨迹质量，生成语义标签。
*   **PEFT Trainer**: 参数高效微调 (LoRA/Prefix)，低成本适配大模型。

### 2. Sim2Real 深度落地
致力于解决 Reality Gap 问题，确保策略在真实硬件上可用。
*   **动力学随机化**: 随机化质量、摩擦、延迟和电机强度，训练强鲁棒性策略。
*   **Sim2Real Analyzer**: 实时对比“理论功率”与“实际反馈”，量化系统差异。
*   **Physics Calibrator**: 基于差异报告自动修正仿真参数 (Online System ID)。

### 3. 多模态感知 (Multimodal Perception)
赋予机器人环境理解能力。
*   **Vision Processor**: 集成 SigLIP/CLIP，提取语义特征，支持 VLA 模型。
*   **Terrain Mapper**: 构建“以机器人为中心”的滚动网格高程图 (Rolling Grid)，实现地形感知。

### 4. 工程化基础设施
*   **Cloud Sim**: 支持 AWS RoboMaker 和本地并行仿真。
*   **CI/CD**: 完整的 GitHub Actions 测试管道。
*   **Multi-Robot**: 统一配置接口，支持 Biped/Quadruped/Wheeled 机器人。

---

## 🚀 快速开始

### 环境要求
*   Python 3.10+
*   PyTorch 2.0+
*   Stable-Baselines3, Transformers, PEFT
*   Godot 4.x (用于仿真)

### 安装

```bash
git clone https://github.com/sossossal/AGI-Walker.git
cd AGI-Walker
pip install -r requirements.txt
```

### 1. 启动自动化进化循环
这是 AGI-Walker 的核心功能，将自动进行 RL 训练、数据生成和模型微调。

```bash
cd python_controller
python evolution_manager.py
```

### 2. 运行 Sim2Real 差异分析
对比理论指令与实际反馈，分析系统差异。

```bash
python python_controller/sim2real_analyzer.py
```

### 3. 运行强化学习训练
手动启动 RL 训练。

```bash
python python_controller/rl_optimizer.py --algorithm PPO --timesteps 100000
```

---

## 🏗️ 项目结构

```
AGI-Walker/
├── python_controller/          # 核心控制逻辑
│   ├── evolution_manager.py    # 进化循环主控 ⭐
│   ├── rl_optimizer.py         # 强化学习优化器
│   ├── sim2real_analyzer.py    # Sim2Real 差异分析 ⭐
│   ├── vision_processor.py     # 视觉感知模块
│   └── terrain_mapper.py       # 地形建图模块
├── python_api/
│   └── godot_robot_env/        # Gym 环境接口
├── training/                   # 训练工具
│   ├── auto_labeler.py         # 自动标注器
│   ├── peft_trainer.py         # 微调训练器
│   └── dataset_cleaner.py      # 数据清洗
├── robot_models/               # 机器人配置 (Biped/Quad/Wheeled)
└── ...
```

---

## 📚 文档资源

*   [**项目全功能总览**](PROJECT_FULL_SUMMARY.md): 详细的功能清单和架构说明。
*   [**仿真环境使用指南**](SIMULATION_GUIDE.md): 🔌 Godot与Python仿真双模式详解。
*   [**模块化构建指南**](docs/MODULAR_ROBOT_BUILDER.md): 🧩 像组装乐高一样组装机器人 (Unitree/Tesla 零件库)。
*   [**OpenNeuro 集成指南**](docs/OPENNEURO_INTEGRATION.md): 📡 Zenoh + ROS 2 通信框架集成 (NEW)。
*   [**实施指南 (Walkthrough)**](walkthrough.md): 详细的使用教程和图表。
*   [**Sim2Real 落地报告**](FINAL_TEST_REPORT.md): 关于 Sim2Real 闭环的验证报告。

### 🕒 版本历史
*   [**CHANGELOG**](CHANGELOG.md): 详细版本变更记录
*   [v3.0 Snapshot (AI Integration)](archive/v3.0_snapshot.md)
*   [v2.0 Snapshot (Parametric Control)](archive/v2.0_snapshot.md)
*   [v1.0 Snapshot (Prototype)](archive/v1.0_snapshot.md)

---

## 🤝 贡献
欢迎提交 Issue 和 PR！请阅读 [CONTRIBUTING.md](CONTRIBUTING.md) 了解详情。

## 📄 许可证
MIT License
