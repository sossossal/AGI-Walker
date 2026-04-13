# Sphinx 闁板秶鐤嗛弬鍥︽
import os
import sys

sys.path.insert(0, os.path.abspath(".."))

# 妞ゅ湱娲版穱鈩冧紖
project = "AGI-Walker"
copyright = "2026, AGI-Walker Team"
author = "AGI-Walker Team"
release = "4.2.0"

# 閹碘晛鐫?
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "sphinx.ext.intersphinx",
    "myst_parser",
]

# 濡剝婢?
templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

# HTML 鏉堟挸鍤?
html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]

# Napoleon 鐠佸墽鐤?(Google 妞嬪孩鐗?docstring)
napoleon_google_docstring = True
napoleon_numpy_docstring = False

# Autodoc 鐠佸墽鐤?
autodoc_default_options = {
    "members": True,
    "undoc-members": True,
    "show-inheritance": True,
}

# Intersphinx
intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "numpy": ("https://numpy.org/doc/stable/", None),
}
