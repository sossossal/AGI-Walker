# AGI-Walker 测试指南

## 运行测试

### 运行所有测试
```bash
pytest
```

### 运行特定测试文件
```bash
pytest tests/test_parts_database.py
```

### 运行特定测试函数
```bash
pytest tests/test_parts_database.py::TestPartsDatabase::test_get_part
```

### 显示详细输出
```bash
pytest -v
```

### 显示打印输出
```bash
pytest -s
```

## 覆盖率报告

### 生成覆盖率报告
```bash
pytest --cov=python_api --cov-report=html
```

### 查看 HTML 报告
```bash
# Linux/macOS
open htmlcov/index.html

# Windows
start htmlcov/index.html
```

### 覆盖率要求
- 目标覆盖率: 60%+
- 核心模块: 80%+
- 工具函数: 50%+

## 测试标记

### 跳过慢速测试
```bash
pytest -m "not slow"
```

### 仅运行集成测试
```bash
pytest -m integration
```

### 跳过硬件测试
```bash
pytest -m "not hardware"
```

## 持续集成

GitHub Actions 会在每次 push 和 PR 时自动运行测试。

查看测试状态：
- GitHub Actions 页面
- README 中的徽章

## 添加新测试

1. 在 `tests/` 目录创建 `test_*.py` 文件
2. 创建测试类 `Test*`
3. 编写测试函数 `test_*()`
4. 使用 fixtures 共享设置
5. 使用 marks 标记特殊测试

### 示例

```python
import pytest

class TestExample:
    @pytest.fixture
    def setup_data(self):
        return {"key": "value"}
    
    def test_something(self, setup_data):
        assert setup_data["key"] == "value"
    
    @pytest.mark.slow
    def test_slow_operation(self):
        # 慢速测试
        pass
```

## 调试测试

### 使用 pdb
```bash
pytest --pdb
```

### 在失败时进入调试器
```bash
pytest --pdb --maxfail=1
```

### 显示最后 N 个失败
```bash
pytest --lf  # last-failed
pytest --ff  # failed-first
```

## 性能测试

使用 pytest-benchmark:
```python
def test_performance(benchmark):
    result = benchmark(function_to_test)
    assert result is not None
```

## 最佳实践

1. **测试命名**: 清晰描述测试目的
2. **独立性**: 测试之间相互独立
3. **可重复**: 结果确定性
4. **快速**: 单元测试应快速执行
5. **覆盖边界**: 测试边界条件
6. **Mock 外部依赖**: 使用 mock 隔离
