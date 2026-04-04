#!/usr/bin/env python
"""自动修复skills_cli.py - 把logger.info()改为print()"""

import re
from pathlib import Path

filepath = Path(r"d:\新建文件夹\AGI-Walker\agi_walker\cli\skills_cli.py")

content = filepath.read_text(encoding='utf-8')

# 替换所有logger.info()为print()
# 但要保持缩进和参数结构不变
content = re.sub(r'logger\.info\((.*?)\)', r'print(\1)', content, flags=re.DOTALL)

# 也修复logger.error()
content = re.sub(r'logger\.error\((.*?)\)', r'print(\1, file=sys.stderr)', content, flags=re.DOTALL)

# 确保有sys.stderr的导入
if 'import sys' not in content:
    # 在第一个import后添加
    content = re.sub(r'(import logging.*?\n)', r'\1import sys\n', content)

filepath.write_text(content, encoding='utf-8')
print(f"✅ 修复完成: {filepath}")
print(f"   - 所有logger.info()改为print()")
print(f"   - 所有logger.error()改为print(..., file=sys.stderr)")
