import logging
logger = logging.getLogger(__name__)
import os
import sys
import tempfile
from pathlib import Path
import pytest

# 1. 确保项目根目录在 sys.path 中，且具有最高优先级
project_root = str(Path(__file__).resolve().parent.parent)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# 1.1 将临时目录固定到仓库内，避免 Windows 用户目录权限问题
_temp_root = Path(project_root) / "test_env" / "tmp"
_temp_root.mkdir(parents=True, exist_ok=True)
os.environ["TMP"] = str(_temp_root)
os.environ["TEMP"] = str(_temp_root)
os.environ["TMPDIR"] = str(_temp_root)
tempfile.tempdir = str(_temp_root)

# 2. 全局环境检测与防御
def pytest_sessionstart(session):
    """
    在测试收集开始前运行。
    """
    logger.info(f"\n[CI-SelfCheck] Project Root: {project_root}")
    logger.info(f"[CI-SelfCheck] Platform: {sys.platform}")
    
    # 检测可能导致段错误的底层库（但不直接加载它们）
    try:
        import importlib.util
        for lib in ['numpy', 'gymnasium', 'zenoh']:
            spec = importlib.util.find_spec(lib)
            status = "Found" if spec else "Not Found"
            logger.info(f"[CI-SelfCheck] Dependency {lib}: {status}")
    except Exception as e:
        logger.info(f"[CI-SelfCheck] Pre-check warning: {e}")

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
        logger.info(f"[CI-Critical] Failed to scan {file_path}: {e}")
