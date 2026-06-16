import logging

import os
import shutil
import sys
import tempfile
import uuid
from pathlib import Path
import pytest

from tests.tcp_json_mock_server import JsonLineGodotServer

# 1. 确保项目根目录在 sys.path 中，且具有最高优先级

logger = logging.getLogger(__name__)
project_root = str(Path(__file__).resolve().parent.parent)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# 1.1 将临时目录固定到仓库内的 runtime 根，避免复用已损坏 ACL 的历史目录
_temp_root = Path(project_root) / "test_env" / "runtime_tmp"
_temp_root.mkdir(parents=True, exist_ok=True)
os.environ["TMP"] = str(_temp_root)
os.environ["TEMP"] = str(_temp_root)
os.environ["TMPDIR"] = str(_temp_root)


def _resolve_temp_root(dir_arg: str | os.PathLike | None = None) -> Path:
    if dir_arg is None:
        root = _temp_root
    else:
        root = Path(dir_arg)
        if not root.is_absolute():
            root = (Path.cwd() / root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    return root


def _safe_mkdtemp(suffix: str = "", prefix: str = "tmp", dir: str | os.PathLike | None = None):
    root = _resolve_temp_root(dir)
    while True:
        candidate = root / f"{prefix}{uuid.uuid4().hex[:8]}{suffix}"
        try:
            candidate.mkdir(parents=True, exist_ok=False)
            return str(candidate)
        except FileExistsError:
            continue


class _SafeTemporaryDirectory:
    def __init__(self, suffix: str = "", prefix: str = "tmp", dir: str | os.PathLike | None = None):
        self.name = _safe_mkdtemp(suffix=suffix, prefix=prefix, dir=dir)

    def __enter__(self):
        return self.name

    def __exit__(self, exc_type, exc, tb):
        self.cleanup()

    def cleanup(self):
        shutil.rmtree(self.name, ignore_errors=True)


tempfile.mkdtemp = _safe_mkdtemp
tempfile.TemporaryDirectory = _SafeTemporaryDirectory
tempfile.tempdir = str(_temp_root)


@pytest.fixture
def tmp_path():
    path = Path(_safe_mkdtemp(prefix="pytest_tmp_"))
    try:
        yield path
    finally:
        shutil.rmtree(path, ignore_errors=True)


@pytest.fixture
def jsonline_godot_server():
    server = JsonLineGodotServer()
    server.start()
    try:
        yield server
    finally:
        server.stop()


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

        for lib in ["numpy", "gymnasium", "zenoh"]:
            spec = importlib.util.find_spec(lib)
            status = "Found" if spec else "Not Found"
            logger.info(f"[CI-SelfCheck] Dependency {lib}: {status}")
    except Exception as e:
        logger.info(f"[CI-SelfCheck] Pre-check warning: {e}")


# 3. 强制忽略非测试脚本的收集
def pytest_ignore_collect(collection_path, config):
    """
    根据路径忽略特定的测试文件。
    彻底杜绝 verify_*, validate_*, benchmark_* 等脚本在扫描时产生副作用。
    """
    base_name = os.path.basename(str(collection_path))

    # 排除所有不以 test_ 开头的 Python 脚本（除了 conftest.py）
    if (
        base_name.endswith(".py")
        and not base_name.startswith("test_")
        and base_name != "conftest.py"
    ):
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
