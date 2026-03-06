import os
import sys
from pathlib import Path
import pytest

# 1. 确保项目根目录在 sys.path 中，且具有最高优先级
# 使用绝对路径解析，防止符号链接或工作目录不一致导致的问题
project_root = str(Path(__file__).resolve().parent.parent)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# 2. 全局环境检测与防御
def pytest_sessionstart(session):
    """
    在测试收集开始前运行。
    用于检测 CI 环境中是否存在可能导致段错误的缺失依赖。
    """
    print(f"\n[CI-SelfCheck] Project Root: {project_root}")
    print(f"[CI-SelfCheck] Python Executable: {sys.executable}")
    
    # 检测关键库是否可用，但不加载它们，仅探测
    try:
        import importlib.util
        for lib in ['numpy', 'gymnasium']:
            spec = importlib.util.find_spec(lib)
            status = "Found" if spec else "Not Found"
            print(f"[CI-SelfCheck] Library {lib}: {status}")
    except Exception as e:
        print(f"[CI-SelfCheck] Warning during self-check: {e}")

# 3. 拦截特定文件的收集（可选，作为最后手段）
# 如果已知某个文件在特定平台上会导致崩溃，可以在这里排除它
def pytest_ignore_collect(path, config):
    """
    根据路径忽略特定的测试文件。
    """
    # 如果在 Linux 上运行且检测到某些硬件相关文件，可以忽略
    # if sys.platform == 'linux' and 'hardware' in str(path):
    #     return True
    return False
