# Sphinx 配置文件
import os
import sys
sys.path.insert(0, os.path.abspath('..'))

# 项目信息
project = 'AGI-Walker'
copyright = '2026, AGI-Walker Team'
author = 'AGI-Walker Team'
release = '4.2.0'

# 扩展
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.intersphinx',
    'myst_parser',
]

# 模板
templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# HTML 输出
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# Napoleon 设置 (Google 风格 docstring)
napoleon_google_docstring = True
napoleon_numpy_docstring = False

# Autodoc 设置
autodoc_default_options = {
    'members': True,
    'undoc-members': True,
    'show-inheritance': True,
}

# Intersphinx
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
}
