# Godot 机器人模拟套件

**工业级机器人仿真开发工具** - 从零件选型到策略训练的完整解决方案

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Godot](https://img.shields.io/badge/Godot-4.2%2B-blue)](https://godotengine.org/)
[![Python](https://img.shields.io/badge/Python-3.8%2B-green)](https://www.python.org/)

---

## 📖 项目简介

本项目提供一个**完整的机器人仿真开发工具链**，包括：

- 🔧 **真实零件库** - 基于制造商规格的数字化零件
- 🎮 **精确物理模拟** - 速度-扭矩曲线、摩擦、热效应
- 🤖 **标准化训练接口** - OpenAI Gym/Gymnasium 兼容
- 🌍 **动态环境系统** - 可调重力、材质、倾斜等
- 📊 **完整文档** - 从快速开始到进阶使用

---

## ✨ 核心特性

| 特性 | 说明 | 状态 |
|------|------|------|
| **零件库系统** | JSON 格式，真实硬件规格 | ✅ 完成 |
| **物理引擎增强** | C++ 实现的高精度电机模型 | 🔄 代码完成 |
| **Python API** | Gymnasium 环境，支持 RL 训练 | ✅ 完成 |
| **环境控制** | 4预设 + 7参数 + 8材质 | ✅ 完成 |
| **域随机化** | 提高 Sim-to-Real 迁移 | ✅ 完成 |
| **硬件控制器** | IMC-22 芯片，支持真实部署 | ✅ 文档完成 |

---

## 🚀 快速开始

### 1. 环境要求

- **Godot**: 4.2+ 
- **Python**: 3.8+
- **依赖**: gymnasium, numpy, stable-baselines3

### 2. 安装

```bash
# 克隆项目
git clone https://github.com/your-repo/godot-robot-sim.git
cd godot-robot-sim

# 安装 Python 依赖
cd python_api
pip install -r requirements.txt
```

### 3. 测试零件库

```bash
python examples/demo_parts.py
```

### 4. 在 Godot 中测试

1. 启动 Godot 4.2+
2. 导入 `godot_project/project.godot`
3. 启用插件：项目设置 → 插件 → Robot Simulation Toolkit
4. 运行测试场景

详细说明见 [快速开始指南](QUICK_START.md)

---

## 📊 项目结构

```
AGI-Walker/
├── parts_library/          # 零件数据库
│   ├── schema/            # JSON Schema 验证
│   ├── motors/            # 电机数据
│   └── sensors/           # 传感器数据
│
├── godot_project/         # Godot 项目
│   ├── addons/            # 插件
│   └── scripts/           # 脚本
│       └── environment/   # 环境系统
│
├── gdextension_src/       # C++ 源码
│   └── src/               # 物理模型
│
├── python_api/            # Python API
│   ├── godot_robot_env/   # Gym 环境
│   └── examples/          # 示例脚本
│
└── 文档/
    ├── QUICK_START.md     # 快速开始
    ├── ADVANCED_USAGE.md  # 进阶使用
    └── ...
```

---

## 🎓 文档

| 文档 | 说明 |
|------|------|
| [快速开始](QUICK_START.md) | 5分钟上手指南 |
| [零件库指南](PARTS_LIBRARY_GUIDE.md) | 如何使用和扩展零件库 |
| [环境系统指南](PHYSICS_ENVIRONMENT_GUIDE.md) | 环境参数和材质系统 |
| [进阶使用](ADVANCED_USAGE.md) | 自定义机器人、域随机化 |
| [参数转换](PARAMETER_CONVERSION_GUIDE.md) | 物理参数说明 |
| [硬件规格](HARDWARE_SPEC.md) | IMC-22 控制器技术规格 |

---

## 💡 使用示例

### Python - 加载零件

```python
from godot_robot_env import PartsDatabase

db = PartsDatabase()
motor = db.get_part("dynamixel_xl430_w250")
print(f"扭矩: {motor['specifications']['stall_torque']} N·m")
```

### Python - 创建 Gym 环境

```python
from godot_robot_env import GodotRobotEnv
from stable_baselines3 import PPO

env = GodotRobotEnv()
model = PPO("MultiInputPolicy", env, verbose=1)
model.learn(total_timesteps=100000)
```

### GDScript - 环境控制

```gdscript
# 切换到月球环境
$EnvironmentController.load_preset("moon")

# 切换地面材质
$GroundMaterialLibrary.apply_material($Ground, "ice")
```

---

## 🎯 应用场景

1. **Sim-to-Real 机器人开发**
   - 在仿真中快速迭代设计
   - 训练控制策略
   - 迁移到真实硬件

2. **强化学习研究**
   - 标准化实验环境
   - 域随机化训练
   - 多环境并行

3. **教育和原型验证**
   - 教学演示
   - 概念验证
   - 硬件选型分析

---

## 📈 项目状态

**当前版本**: 0.8.0-beta  
**完成度**: 80%

**已完成**:
- ✅ 零件库基础设施
- ✅ Python API 和训练接口
- ✅ 环境控制系统
- 🔄 GDExtension C++ 插件（代码完成，待编译）

**计划中**:
- ⏳ 编译 C++ 插件
- ⏳ 集成测试
- ⏳ 演示视频

---

## 🤝 贡献

欢迎贡献！您可以：

1. 添加新零件数据
2. 改进物理模型
3. 创建示例机器人
4. 完善文档
5. 报告 Bug

---

## 📄 许可证

MIT License - 详见 LICENSE

---

## 🙏 致谢

- **Godot Engine** - 优秀的开源游戏引擎
- **OpenAI Gym** - 标准化的 RL 接口
- **Stable-Baselines3** - 高质量的 RL 实现
- 所有零件制造商的开放数据手册

---

**Happy Coding!** 🚀

---

**最后更新**: 2026-01-14  
**维护者**: AGI-Walker Team
