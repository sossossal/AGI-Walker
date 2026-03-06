# AGI-Walker 入门教程

完整的从零到一的机器人训练指南。

## 📖 教程内容

本教程包含 5 个部分，预计 2-3 小时完成：

1. **环境准备** (30分钟)
2. **运行第一个示例** (20分钟)
3. **创建自己的机器人** (60分钟)
4. **可视化训练过程** (15分钟)
5. **进阶实验** (30分钟)

## 🎯 学习目标

完成本教程后，你将能够：
- ✅ 搭建 AGI-Walker 开发环境
- ✅ 创建自定义机器人配置
- ✅ 训练强化学习模型
- ✅ 评估和测试策略
- ✅ 可视化训练过程

## 📚 查看完整教程

**主教程**: [GETTING_STARTED.md](file:///C:/Users/荣耀/.gemini/antigravity/brain/87a9b052-dd49-43f6-b9ac-8d8f9c86d6b8/GETTING_STARTED.md)

## 快速开始

```bash
# 1. 克隆项目
git clone https://github.com/sossossal/AGI-Walker.git
cd AGI-Walker

# 2. 创建虚拟环境
python -m venv walker_env
walker_env\Scripts\activate  # Windows
# source walker_env/bin/activate  # Linux/Mac

# 3. 安装依赖
cd AGI-Walker
pip install -r requirements.txt

# 4. 运行示例
python examples/quick_start_balance.py
```

## 项目结构

```
AGI-Walker/
├── python_api/          # Python API 和工具
├── examples/            # 示例脚本
├── configs/             # 机器人配置
├── godot_project/       # Godot 仿真场景
├── docs/                # 文档
└── requirements.txt     # 依赖列表
```

## 需要的前置知识

**必需**:
- 基础 Python 编程
- 命令行操作

**推荐**:
- 机器学习基础
- 强化学习概念（可以边学边用）

## 学习路径

```
入门 → 基础训练 → 自定义机器人 → 高级算法 → 真实部署
 ↓        ↓           ↓            ↓          ↓
本教程   examples/   配置文件    离线RL/GAIL  芯片设计
```

## 帮助和支持

- 📘 文档: `docs/`
- 💬 讨论: GitHub Discussions
- 🐛 问题: GitHub Issues
- 📧 联系: sownly@gmail.com
