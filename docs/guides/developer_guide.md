# 开发者指南

本指南帮助开发者快速上手 AGI-Walker 的开发。

## 环境设置

### 1. 克隆仓库
```bash
git clone https://github.com/sossossal/AGI-Walker.git
cd AGI-Walker
```

### 2. 安装依赖
```bash
# 使用一键安装脚本
./install.sh  # Linux/Mac
# 或
install.bat   # Windows

# 或手动安装
python -m venv venv
source venv/bin/activate  # Linux/Mac
# venv\Scripts\activate.bat  # Windows
pip install -r requirements.txt
```

### 3. 运行测试
```bash
python tests/test_integration.py
python tests/test_extended.py
```

## 代码规范

### Python 风格
遵循 PEP 8:
- 使用 4 空格缩进
- 行长度 ≤ 100 字符
- 使用 `black` 格式化

### 类型注解
所有公共 API 必须有类型注解:
```python
from typing import List, Dict, Optional

def process_data(
    data: List[float],
    config: Optional[Dict[str, Any]] = None
) -> np.ndarray:
    """处理数据"""
    ...
```

### 文档字符串
使用 Google 风格:
```python
def train_model(env, algorithm="PPO"):
    """
    训练强化学习模型
    
    Args:
        env: Gymnasium 环境
        algorithm: 算法名称
    
    Returns:
        训练好的模型
    
    Example:
        >>> model = train_model(env, "PPO")
    """
    ...
```

## 项目结构

```
AGI-Walker/
├── python_api/          # Python 接口
│   ├── zenoh_interface.py
│   ├── task_editor.py
│   └── mujoco_backend.py
├── python_controller/   # 控制器
│   ├── evolution_manager.py
│   └── rl_optimizer.py
├── examples/            # 示例代码
│   └── tasks/          # 任务环境
├── tests/              # 测试
├── docs/               # 文档
└── godot_project/      # Godot 仿真
```

## 添加新任务

### 1. 创建环境文件
```python
# examples/tasks/my_task/env.py
import gymnasium as gym

class MyTaskEnv(gym.Env):
    def __init__(self):
        super().__init__()
        # 定义观测和动作空间
        ...
    
    def reset(self, seed=None, options=None):
        ...
    
    def step(self, action):
        ...
```

### 2. 注册环境
```python
gym.register(
    id='MyTask-v0',
    entry_point='examples.tasks.my_task.env:MyTaskEnv'
)
```

### 3. 添加测试
```python
def test_my_task():
    env = gym.make('MyTask-v0')
    obs, info = env.reset()
    assert obs.shape == env.observation_space.shape
```

## 提交代码

### 1. 创建分支
```bash
git checkout -b feature/my-feature
```

### 2. 编写代码和测试
```bash
# 运行测试
python tests/test_extended.py

# 格式化代码
black python_api/ python_controller/
```

### 3. 提交
```bash
git add .
git commit -m "feat: add my feature"
git push origin feature/my-feature
```

### 4. 创建 Pull Request
在 GitHub 上创建 PR,等待审核。

## 调试技巧

### 1. 使用 Python 调试器
```python
import pdb; pdb.set_trace()
```

### 2. Zenoh 调试
```bash
# 启用详细日志
export RUST_LOG=debug
python your_script.py
```

### 3. 查看 Godot 日志
```bash
# 在 Godot 编辑器中查看输出面板
```

## 常见问题

### Q: Zenoh 连接失败?
A: 检查防火墙设置,确保端口 7447 开放。

### Q: MuJoCo 安装失败?
A: 确保有 C++ 编译器 (Linux: gcc, Windows: MSVC)。

### Q: 测试失败?
A: 运行 `pip install -r requirements.txt` 确保依赖完整。

## 资源链接

- [GitHub 仓库](https://github.com/sossossal/AGI-Walker)
- [贡献指南](../CONTRIBUTING.md)
- [技术债务计划](TECH_DEBT_PLAN.md)
