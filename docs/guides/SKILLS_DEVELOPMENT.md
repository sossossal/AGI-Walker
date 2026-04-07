# Skills 开发指�?

本指南将帮助你为AGI-Walker开发新的Skills�?

---

## 快速开�?

### 1. 创建Skill目录

```bash
mkdir -p agi_walker/skills/my-skill
cd agi_walker/skills/my-skill
```

### 2. 创建SKILL.md

创建 `SKILL.md` 文件,包含YAML frontmatter和Markdown内容:

```markdown
---
name: my-skill
description: "简短描述你的skill功能和适用场景"
metadata:
  agi_walker:
    emoji: "🔧"
    category: "工具"
    requires:
      python_modules: ["numpy"]
---

# My Skill

详细的使用文�?..

## 快速开�?

...

## API参�?

...
```

### 3. 实现核心功能

创建 `__init__.py`:

```python
"""
My Skill - 功能描述
"""

def my_function():
    """主要功能"""
    pass
```

### 4. 测试

```bash
python -m agi_walker.cli skills info my-skill
```

---

## SKILL.md 规范

### Frontmatter 字段

#### 必需字段

```yaml
name: skill-name          # Skill名称 (kebab-case)
description: "描述..."    # 简短描�?(一句话)
```

#### 可选字�?

```yaml
metadata:
  agi_walker:
    emoji: "🔧"            # Emoji图标
    category: "分类"       # 分类名称
    requires:
      python_modules:      # Python依赖
        - numpy
        - scipy
      bins:                # 命令行工具依�?
        - ffmpeg
      system_packages:     # 系统包依�?
        - libopencv-dev
```

### Markdown 内容结构

推荐结构:

```markdown
# Skill名称

简�?(1-2�?

## 快速开�?

最简单的使用示例 (代码�?

## 功能特�?

- 特�?
- 特�?

## API参�?

详细API说明 (或链接到 references/api.md)

## 示例

完整使用示例

## 常见问题

Q&A

## 下一�?

相关Skill推荐
```

---

## 目录结构

### 最小结�?

```
my-skill/
├── SKILL.md           # 必需: Skill文档
└── __init__.py        # 必需: 核心实现
```

### 完整结构

```
my-skill/
├── SKILL.md                  # Skill文档
├── __init__.py              # 核心实现
├── scripts/                 # 可�? 命令行工�?
�?  ├── tool1.py
�?  └── tool2.py
├── references/              # 可�? 参考文�?
�?  ├── api.md
�?  └── algorithms.md
├── assets/                  # 可�? 资源文件
�?  ├── templates/
�?  └── examples/
└── tests/                   # 可�? 测试
    └── test_my_skill.py
```

---

## 代码规范

### 导出接口

�?`__init__.py` 中明确导�?

```python
"""
My Skill - 功能描述
"""

from .core import MyClass, my_function

__all__ = ['MyClass', 'my_function']
```

### 便捷函数

提供简单的顶层函数:

```python
def do_something(input_file: str, output_file: str) -> Result:
    """简洁的API
    
    Args:
        input_file: 输入文件
        output_file: 输出文件
        
    Returns:
        Result对象
        
    Example:
        >>> result = do_something("in.json", "out.json")
        >>> print(result.success)
        True
    """
    # 实现...
```

### 类型注解

使用类型注解:

```python
from typing import Dict, List, Optional
from pathlib import Path

def process_data(
    data: Dict[str, Any],
    options: Optional[List[str]] = None
) -> Path:
    ...
```

---

## 命令行工�?

### 创建CLI脚本

�?`scripts/` 目录创建:

```python
#!/usr/bin/env python
"""
My Tool - 描述
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from agi_walker.skills.my_skill import do_something


def main():
    parser = argparse.ArgumentParser(description="...")
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    
    args = parser.parse_args()
    
    result = do_something(args.input, args.output)
    
    if result.success:
        print(f"�?成功")
        return 0
    else:
        print(f"�?失败: {result.error}")
        return 1


if __name__ == '__main__':
    sys.exit(main())
```

---

## 参考文�?

### API文档模板

�?`references/api.md`:

```markdown
# My Skill API 参�?

## 函数

### do_something

\`\`\`python
do_something(input_file: str, output_file: str) -> Result
\`\`\`

描述...

**参数:**
- `input_file` - 说明
- `output_file` - 说明

**返回:**
- `Result` - 说明

**示例:**
\`\`\`python
result = do_something("in.json", "out.json")
\`\`\`

## �?

### MyClass

...
```

---

## 最佳实�?

### 1. 渐进式披�?

遵循Moltbot的渐进式披露原则:

- **Level 1 (Metadata)**: 简短的name和description
- **Level 2 (SKILL.md)**: 快速开始和核心功能
- **Level 3 (References)**: 详细API和算法文�?

### 2. 示例优先

提供可运行的示例:

```python
# �?不好
def process(data): ...

# �?�?
def process(data):
    """
    Example:
        >>> from agi_walker.skills.my_skill import process
        >>> result = process({"key": "value"})
        >>> print(result)
        {'processed': True}
    """
    ...
```

### 3. 错误处理

提供清晰的错误信�?

```python
if not input_path.exists():
    raise FileNotFoundError(
        f"输入文件不存�? {input_path}\n"
        f"请检查路径是否正�?
    )
```

### 4. 文档字符�?

使用Google风格docstring:

```python
def my_function(param1: str, param2: int = 0) -> bool:
    """简短描�?
    
    详细说明 (可�?
    
    Args:
        param1: 参数1说明
        param2: 参数2说明, 默认0
        
    Returns:
        成功返回True
        
    Raises:
        ValueError: 如果param1为空
        
    Example:
        >>> my_function("test")
        True
    """
    ...
```

---

## 分类指南

### 建议的分�?

- **建模** - 机器人建模相�?
- **优化** - 参数优化相关
- **转换** - 格式转换相关
- **仿真** - 仿真执行相关
- **数据生成** - AI数据生成
- **分析** - 数据分析工具
- **工具** - 通用工具
- **其他** - 其他类型

### 选择分类

根据Skill的主要用途选择:
- 如果Skill有多个用�?选择最主要�?
- 尽量使用已有分类
- 必要时可以创建新分类

---

## 依赖管理

### 声明依赖

在SKILL.md frontmatter中声�?

```yaml
metadata:
  agi_walker:
    requires:
      python_modules:
        - numpy>=1.24.0
        - scipy>=1.10.0
      bins:
        - ffmpeg
```

### 检查依�?

在代码中检�?

```python
def __init__(self):
    try:
        import numpy as np
    except ImportError:
        raise ImportError(
            "This skill requires numpy.\n"
            "Install: pip install numpy>=1.24.0"
        )
```

---

## 测试

### 单元测试

创建 `tests/test_my_skill.py`:

```python
import pytest
from agi_walker.skills.my_skill import do_something


def test_basic_functionality():
    """测试基本功能"""
    result = do_something("test.json", "out.json")
    assert result.success


def test_error_handling():
    """测试错误处理"""
    with pytest.raises(FileNotFoundError):
        do_something("nonexistent.json", "out.json")
```

### 运行测试

```bash
pytest tests/test_my_skill.py -v
```

---

## 发布清单

提交新Skill前检�?

- [ ] SKILL.md包含完整frontmatter
- [ ] 有快速开始示�?
- [ ] 代码有类型注�?
- [ ] 主要函数有docstring
- [ ] 有使用示�?
- [ ] 声明了所有依�?
- [ ] 通过 `agi_walker skills validate`
- [ ] 在CLI和GUI中测试过

---

## 示例

参考现有Skills:

- `robot-modeling` - 完整的流式API示例
- `parameter-optimizer` - 科学计算集成示例
- `urdf-generator` - 格式转换示例

---

## 获取帮助

- 查看 `.agent/AGENTS.md` - 项目规范
- 阅读现有Skills的源�?
- 使用 `agi_walker skills info <name> -d` 查看文档

---

**记住**: Skills应该简洁、实用、文档齐全。优先提供可用的工具,而非复杂的抽象�?
