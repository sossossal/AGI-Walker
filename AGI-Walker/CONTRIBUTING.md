# Contributing to AGI-Walker

感谢您对 AGI-Walker 项目的关注！我们欢迎所有形式的贡献。

## 🤝 如何贡献

### 报告 Bug

在提交 Bug 之前，请确保：
1. 搜索现有的 [Issues](https://github.com/agi-walker/agi-walker-sim/issues)，避免重复
2. 使用 Bug 报告模板
3. 提供详细的复现步骤

### 提交功能建议

1. 打开新的 Issue
2. 使用 "Feature Request" 标签
3. 清楚描述功能的用途和价值

### 提交代码

1. **Fork 项目**
   ```bash
   git clone https://github.com/your-username/agi-walker-sim.git
   cd agi-walker-sim
   ```

2. **创建功能分支**
   ```bash
   git checkout -b feature/your-feature-name
   ```

3. **进行修改并提交**
   ```bash
   git add .
   git commit -m "Add: 简短描述你的修改"
   ```

4. **推送到你的 Fork**
   ```bash
   git push origin feature/your-feature-name
   ```

5. **创建 Pull Request**
   - 填写 PR 模板
   - 链接相关 Issue
   - 等待代码审查

## 📝 代码规范

### Python

- 遵循 [PEP 8](https://www.python.org/dev/peps/pep-0008/)
- 使用类型提示
- 添加文档字符串

```python
def example_function(param: str) -> int:
    """
    函数简短描述
    
    Args:
        param: 参数说明
        
    Returns:
        返回值说明
    """
    return len(param)
```

### GDScript

- 使用 4 空格缩进
- 变量命名使用 snake_case
- 添加注释说明复杂逻辑

### C/C++

- 遵循 [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- 使用 clang-format 格式化

## 🧪 测试

所有新功能应包含测试：

```bash
# Python 测试
cd python_api
pytest tests/

# Godot 测试
# 在 Godot 编辑器中运行测试场景
```

## 📚 文档

如果你的修改影响用户使用：
- 更新相关的 .md 文档
- 添加代码注释
- 在 PR 中说明文档变更

## 🎯 优先级任务

查看 [Issues](https://github.com/agi-walker/agi-walker-sim/issues) 中标记为 "good first issue" 的任务，适合新贡献者。

## 💬 交流

- Discord: [加入我们](https://discord.gg/agi-walker)
- GitHub Discussions: [讨论区](https://github.com/agi-walker/agi-walker-sim/discussions)

## 📜 行为准则

请阅读并遵守我们的 [行为准则](CODE_OF_CONDUCT.md)。

---

再次感谢您的贡献！🎉
