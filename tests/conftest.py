import os
import sys
from pathlib import Path

# 确保项目根目录在 sys.path 中
project_root = str(Path(__file__).parent.parent.absolute())
if project_root not in sys.path:
    sys.path.insert(0, project_root)
