import os
import sys
from pathlib import Path
import pytest

# 1. 确保项目根目录在 sys.path 中，且具有最高优先级
project_root = str(Path(__file__).resolve().parent.parent)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# 2. 全局环境检测与防御
def pytest_sessionstart(session):
    """
    在测试收集开始前运行。
    """
    print(f"\n[CI-SelfCheck] Project Root: {project_root}")
    print(f"[CI-SelfCheck] Platform: {sys.platform}")
    
    # 检测可能导致段错误的底层库（但不直接加载它们）
    try:
        import importlib.util
        for lib in ['numpy', 'gymnasium', 'zenoh']:
            spec = importlib.util.find_spec(lib)
            status = "Found" if spec else "Not Found"
            print(f"[CI-SelfCheck] Dependency {lib}: {status}")
    except Exception as e:
        print(f"[CI-SelfCheck] Pre-check warning: {e}")

# 3. 强制忽略非测试脚本的收集
def pytest_ignore_collect(path, config):
    """
    根据路径忽略特定的测试文件。
    彻底杜绝 verify_*, validate_*, benchmark_* 等脚本在扫描时产生副作用。
    """
    base_name = os.path.basename(str(path))
    
    # 排除所有不以 test_ 开头的 Python 脚本（除了 conftest.py）
    if base_name.endswith(".py") and not base_name.startswith("test_") and base_name != "conftest.py":
        return True
        
    return False

# 4. 全局捕获导入时的致命错误 (试验性)
# 注意：这无法捕获真正的段错误，但可以捕获 OSError
@pytest.hookimpl(tryfirst=True)
def pytest_collect_file(file_path, parent):
    try:
        # 这里仅作记录，不执行实际导入
        pass
    except Exception as e:
        print(f"[CI-Critical] Failed to scan {file_path}: {e}")
