# 贡献指南 (Contributing Guide)

感谢您对 AGI-Walker 的关注! 我们欢迎各种形式的贡献。

---

## 🤝 贡献方式

### 1. 报告 Bug
在 [GitHub Issues](https://github.com/sossossal/AGI-Walker/issues) 提交 Bug 报告时,请包含:
- 问题描述
- 复现步骤
- 预期行为 vs 实际行为
- 环境信息 (OS, Python 版本, 依赖版本)
- 错误日志

**模板**:
```markdown
**描述**: 简要描述问题

**复现步骤**:
1. 运行 `python ...`
2. 观察到 ...

**环境**:
- OS: Windows 11
- Python: 3.12
- AGI-Walker: v4.1.0

**日志**:
```
错误日志粘贴在这里
```
```

### 2. 提出新功能
在 [GitHub Discussions](https://github.com/sossossal/AGI-Walker/discussions) 讨论新功能:
- 功能描述
- 使用场景
- 预期收益
- 实现难度估计

### 3. 贡献代码
遵循以下流程:

#### 3.1 Fork 仓库
```bash
# 1. Fork 到你的 GitHub 账号
# 2. Clone 到本地
git clone https://github.com/YOUR_USERNAME/AGI-Walker.git
cd AGI-Walker

# 3. 添加上游仓库
git remote add upstream https://github.com/sossossal/AGI-Walker.git
```

#### 3.2 创建分支
```bash
git checkout -b feature/your-feature-name
# 或
git checkout -b fix/your-bug-fix
```

#### 3.3 编写代码
遵循我们的代码规范 (见下文)

#### 3.4 测试
```bash
# 运行测试
python tests/test_integration.py

# 检查代码风格
flake8 agi_walker/ web_panel/ tests/
black --check agi_walker/ web_panel/ tests/
```

#### 3.5 提交
```bash
git add .
git commit -m "feat: add awesome feature"
# 或
git commit -m "fix: resolve issue #123"
```

**Commit 消息规范**:
- `feat:` 新功能
- `fix:` Bug 修复
- `docs:` 文档更新
- `test:` 测试相关
- `refactor:` 代码重构
- `perf:` 性能优化

#### 3.6 推送并创建 PR
```bash
git push origin feature/your-feature-name
```

然后在 GitHub 上创建 Pull Request。

---

## 📝 代码规范

### Python 代码风格
遵循 [PEP 8](https://pep8.org/):
- 使用 4 空格缩进
- 行长度 ≤ 100 字符
- 使用 `black` 格式化代码
- 使用 `flake8` 检查代码

**示例**:
```python
def calculate_reward(
    position: np.ndarray,
    velocity: np.ndarray,
    target: np.ndarray
) -> float:
    """
    计算奖励函数
    
    Args:
        position: 当前位置
        velocity: 当前速度
        target: 目标位置
    
    Returns:
        奖励值
    """
    distance = np.linalg.norm(position - target)
    reward = -distance + 0.1 * np.linalg.norm(velocity)
    return reward
```

### 类型注解
所有公共 API 必须有类型注解:
```python
from typing import List, Dict, Optional

def process_data(
    data: List[float],
    config: Optional[Dict[str, Any]] = None
) -> np.ndarray:
    ...
```

### 文档字符串
使用 Google 风格:
```python
def train_model(env, algorithm="PPO", timesteps=1000000):
    """
    训练强化学习模型
    
    Args:
        env: Gymnasium 环境
        algorithm: 算法名称 (PPO/SAC/TD3)
        timesteps: 训练步数
    
    Returns:
        训练好的模型
    
    Raises:
        ValueError: 如果算法不支持
    
    Example:
        >>> env = gym.make('StairClimbing-v0')
        >>> model = train_model(env, algorithm="PPO")
    """
    ...
```

---

## 🧪 测试要求

### 单元测试
为新功能添加测试:
```python
# tests/test_your_feature.py
def test_your_function():
    result = your_function(input_data)
    assert result == expected_output
```

### 集成测试
确保不破坏现有功能:
```bash
python tests/test_integration.py
```

---

## 📚 文档贡献

### 更新文档
文档位于 `docs/` 目录:
- 使用 Markdown 格式
- 添加代码示例
- 包含截图 (如适用)

### API 文档
使用 Sphinx 生成:
```bash
cd docs
make html
```

---

## 🎯 贡献新任务

我们特别欢迎新的 RL 任务! 请遵循以下结构:

```
examples/tasks/your_task/
├── env.py          # Gymnasium 环境
├── train.py        # 训练脚本
├── evaluate.py     # 评估脚本
├── README.md       # 任务说明
└── configs/
    └── default.yaml
```

**任务要求**:
- 清晰的任务目标
- 合理的奖励函数
- 性能 Baseline
- 详细的文档

---

## 🏆 贡献者名单

感谢所有贡献者! 您的名字将出现在:
- [CONTRIBUTORS.md](CONTRIBUTORS.md)
- 项目 README
- Release Notes

---

## 📧 联系方式

- **GitHub Issues**: Bug 报告和功能请求
- **GitHub Discussions**: 技术讨论
- **Discord**: [加入我们](https://discord.gg/agi-walker) (即将开放)
- **Email**: team@agi-walker.org

---

## 📜 行为准则

请遵守我们的 [行为准则](CODE_OF_CONDUCT.md):
- 尊重他人
- 建设性反馈
- 包容多样性
- 专注技术

---

**感谢您的贡献,让 AGI-Walker 变得更好! 🚀**
