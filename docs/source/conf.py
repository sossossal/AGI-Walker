# Configuration file for the Sphinx documentation builder.

import os
import sys
sys.path.insert(0, os.path.abspath('../../'))

project = 'AGI-Walker'
copyright = '2026, Antigravity Team'
author = 'Antigravity Team'
release = '1.0.0'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
]

templates_path = ['_templates']
exclude_patterns = []

html_theme = 'sphinx_rtd_theme' # Revert to alabaster if not installed
html_static_path = ['_static']

autodoc_mock_imports = ["godot", "gymnasium", "numpy", "scipy", "requests", "fastapi", "uvicorn", "pydantic"]
