"""
测试公共配置。

确保直接运行 `pytest` 时可以从仓库根目录导入 `agi_walker` 包。
"""

from pathlib import Path
import sys


ROOT_DIR = Path(__file__).resolve().parents[1]
root_dir_str = str(ROOT_DIR)

if root_dir_str not in sys.path:
    sys.path.insert(0, root_dir_str)
