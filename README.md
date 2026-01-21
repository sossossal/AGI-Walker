# AGI-Walker: 自进化通用机器人平台

<div align="center">

![AGI-Walker Logo](https://via.placeholder.com/800x200/667eea/ffffff?text=AGI-Walker)

**从仿真到现实的完整机器人 AI 开发平台**

[![GitHub Stars](https://img.shields.io/github/stars/sossossal/AGI-Walker?style=social)](https://github.com/sossossal/AGI-Walker)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/)
[![Tests](https://img.shields.io/badge/tests-passing-brightgreen.svg)](tests/)
[![Documentation](https://img.shields.io/badge/docs-complete-success.svg)](docs/)

[English](README_EN.md) | [中文](README.md) | [文档](docs/) | [演示](examples/) | [Discord](https://discord.gg/agi-walker)

</div>

---

## 🌟 为什么选择 AGI-Walker?

### ⚡ 业界最快的通信性能
- **10μs 延迟** - 比 Isaac Sim 快 100x, 比 Gazebo 快 200x
- msgpack 序列化 - 4.7x 性能提升
- Zenoh 实时通信 - 零拷贝优化

### 🎯 10 个即用型 RL 任务
- **Locomotion**: 楼梯攀爬、崎岖地形、斜坡行走
- **Manipulation**: 物体抓取、开门、堆叠积木
- **Navigation**: 避障导航、动态环境
- **Multi-Agent**: 协作搬运、编队行走

### 🌍 最完整的 Sim2Real 工具链
- 数据差异分析器 - 自动识别虚实差距
- 物理参数校准 - 在线优化仿真参数
- 任务编辑器 - 可视化对比虚拟与现实

### 🚀 一键启动,开箱即用
```bash
# 一键安装
./install.sh  # Linux/Mac
install.bat   # Windows

# 启动 Web 控制面板
python web_panel/server.py

# 访问 http://localhost:8000
```

---

## 📊 与主流平台对比

| 平台 | 通信延迟 | 任务数 | Sim2Real | Web GUI | 开源 |
|------|---------|--------|---------|---------|------|
| **AGI-Walker** | **10μs** | **10** | **⭐⭐⭐⭐⭐** | **✅** | **✅** |
| Isaac Sim | 1000μs | 20+ | ⭐⭐⭐ | ✅ | ❌ |
| MuJoCo | N/A | 3 | ⭐⭐ | ❌ | ✅ |
| PyBullet | 500μs | 5 | ⭐⭐ | ❌ | ✅ |

---

## 🎬 快速演示

### 楼梯攀爬任务
```python
import gymnasium as gym
from stable_baselines3 import PPO

# 加载预训练模型
model = PPO.load("models/stair_climbing_ppo.zip")

# 创建环境
env = gym.make('StairClimbing-v0')

# 运行
obs, _ = env.reset()
for _ in range(1000):
    action, _ = model.predict(obs)
    obs, reward, done, truncated, info = env.step(action)
    if done or truncated:
        print(f"✅ 成功爬上 {info['steps_climbed']}/5 级楼梯!")
        break
```

### Web 控制面板
```bash
python web_panel/server.py
# 访问 http://localhost:8000
# - 创建训练任务
# - 实时监控进度
# - 查看性能统计
```

---

## 🏗️ 核心功能

### 1. 标准任务库 (10 个)
- ✅ 完整的 Gymnasium 接口
- ✅ 难度分级 (⭐-⭐⭐⭐⭐⭐)
- ✅ 预训练模型
- ✅ 性能 Baseline

[查看所有任务 →](examples/tasks/README.md)

### 2. 高精度物理仿真
- **MuJoCo 后端**: 学术标准,10x 精度提升
- **Godot 可视化**: 实时 3D 渲染
- **程序化地形**: 动态生成,防止过拟合

### 3. 完整的 Sim2Real 工具链
- **数据分析**: 自动识别虚实差距
- **参数校准**: 在线优化物理参数
- **任务编辑器**: 可视化对比工具

[Sim2Real 指南 →](docs/SIM2REAL_GUIDE.md)

### 4. 模块化零件系统
- **14 个真实零件**: Unitree, Tesla 等
- **BOM 自动计算**: 成本估算 ¥2000-¥5000
- **一键组装**: 像乐高一样组装机器人

[零件库文档 →](docs/MODULAR_ROBOT_BUILDER.md)

### 5. Web 控制面板
- **任务管理**: 创建、监控、评估
- **实时更新**: WebSocket 通信
- **性能可视化**: 训练曲线、统计数据

[Web 面板指南 →](docs/WEB_PANEL_GUIDE.md)

### 6. 硬件部署支持
- **ESP32 固件**: Zenoh-Pico 通信
- **ROS 2 集成**: 标准机器人生态
- **完整文档**: BOM、接线图、烧录指南

[硬件部署 →](docs/HARDWARE_DEPLOYMENT.md)

---

## 📦 安装

### 方法 1: 一键安装 (推荐)
```bash
# Linux/Mac
chmod +x install.sh
./install.sh

# Windows
install.bat
```

### 方法 2: Docker
```bash
docker build -t agi-walker .
docker run -p 8000:8000 -p 9090:9090 agi-walker
```

### 方法 3: 手动安装
```bash
git clone https://github.com/sossossal/AGI-Walker.git
cd AGI-Walker
pip install -r requirements.txt
```

---

## 🚀 快速开始

### 1. 运行第一个任务
```bash
python examples/tasks/stair_climbing/env.py
```

### 2. 训练 RL 模型
```bash
pip install stable-baselines3
python examples/tasks/stair_climbing/train.py --timesteps 100000
```

### 3. 启动 Web 控制面板
```bash
python web_panel/server.py
# 访问 http://localhost:8000
```

---

## 📚 文档

- [**快速开始**](docs/QUICKSTART.md) - 5 分钟上手
- [**API 文档**](docs/api/) - 完整 API 参考
- [**任务库**](examples/tasks/README.md) - 10 个标准任务
- [**开发者指南**](docs/guides/developer_guide.md) - 贡献代码
- [**硬件部署**](docs/HARDWARE_DEPLOYMENT.md) - 真实机器人
- [**性能基准**](tests/benchmark_performance.py) - 性能测试

---

## 🎯 预训练模型

| 任务 | 算法 | 成功率 | 下载 |
|------|------|--------|------|
| 楼梯攀爬 | PPO | 85% | [下载](https://github.com/sossossal/AGI-Walker/releases) |
| 物体抓取 | SAC | 72% | 训练中 |
| 避障导航 | PPO | 90% | 训练中 |

[模型库 →](docs/MODEL_ZOO.md)

---

## 🤝 贡献

我们欢迎各种形式的贡献!

- 🐛 [报告 Bug](https://github.com/sossossal/AGI-Walker/issues)
- 💡 [提出新功能](https://github.com/sossossal/AGI-Walker/discussions)
- 📝 [改进文档](docs/)
- 🎯 [贡献任务](examples/tasks/)
- 🤖 [分享模型](docs/MODEL_ZOO.md)

[贡献指南 →](CONTRIBUTING.md)

---

## 📈 性能指标

| 指标 | AGI-Walker | 目标 | 状态 |
|------|-----------|------|------|
| 通信延迟 | 10μs | <2000μs | ✅ 超额 200x |
| 序列化速度 | 14μs | <100μs | ✅ |
| 测试覆盖率 | 60% | >80% | 🟡 |
| 文档完整度 | 100% | 100% | ✅ |

[性能基准测试 →](tests/benchmark_performance.py)

---

## 🌐 社区

- **Discord**: [加入讨论](https://discord.gg/agi-walker)
- **GitHub Discussions**: [技术交流](https://github.com/sossossal/AGI-Walker/discussions)
- **知乎专栏**: [技术博客](https://zhuanlan.zhihu.com/agi-walker)
- **Bilibili**: [视频教程](https://space.bilibili.com/agi-walker)

---

## 📄 许可证

本项目采用 [MIT 许可证](LICENSE)。

---

## 🙏 致谢

- [Eclipse Zenoh](https://zenoh.io/) - 高性能通信
- [MuJoCo](https://mujoco.org/) - 物理仿真
- [Godot Engine](https://godotengine.org/) - 3D 可视化
- [Stable-Baselines3](https://stable-baselines3.readthedocs.io/) - RL 算法
- [ROS 2](https://ros.org/) - 机器人生态

---

<div align="center">

**⭐ 如果觉得有用,请给个 Star!**

Made with ❤️ by AGI-Walker Team

</div>
