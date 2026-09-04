"""Sphinx configuration for the FrankaTwin docs (PyData theme, Markdown via MyST)."""

import os
import sys
from datetime import date

sys.path.insert(0, os.path.abspath("../python"))  # autodoc imports frankatwin from the checkout

project = "FrankaTwin"
author = "Tao Sun, Patrick Yin"
copyright = f"{date.today().year}, {author}"
release = "0.2.0"

extensions = [
    "myst_parser",
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "sphinx.ext.intersphinx",
    "sphinx_copybutton",
    "sphinxcontrib.mermaid",
]

source_suffix = {".md": "markdown", ".rst": "restructuredtext"}
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

# --- MyST ---------------------------------------------------------------
myst_enable_extensions = ["colon_fence", "deflist", "tasklist", "attrs_inline"]
myst_heading_anchors = 4                      # GitHub-style #slug anchors, as used by the cross-links
myst_fence_as_directive = ["mermaid"]         # ```mermaid blocks render via sphinxcontrib-mermaid
suppress_warnings = ["myst.xref_missing"]     # links into the repo (examples/*.py, LICENSE, …) are fine on GitHub

# --- autodoc ------------------------------------------------------------
autodoc_member_order = "bysource"
autodoc_typehints = "description"
autodoc_mock_imports = []                     # numpy / pyyaml / pyzmq are installed with the package
intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "numpy": ("https://numpy.org/doc/stable/", None),
}
napoleon_google_docstring = True
napoleon_numpy_docstring = True

# --- HTML ---------------------------------------------------------------
html_theme = "pydata_sphinx_theme"
html_title = "FrankaTwin"
html_static_path = ["_static"]
html_css_files = ["custom.css"]
html_last_updated_fmt = "%Y-%m-%d"
html_theme_options = {
    "github_url": "https://github.com/tsrobcvai/frankatwin",
    "navbar_start": ["navbar-logo"],
    "navbar_end": ["theme-switcher", "navbar-icon-links"],
    "show_toc_level": 2,
    "navigation_with_keys": True,
    "footer_start": ["copyright"],
    "footer_end": ["last-updated"],
    "icon_links": [
        {
            "name": "GitHub",
            "url": "https://github.com/tsrobcvai/frankatwin",
            "icon": "fa-brands fa-github",
        }
    ],
}
html_context = {
    "github_user": "tsrobcvai",
    "github_repo": "frankatwin",
    "github_version": "v0.2",
    "doc_path": "docs",
}
copybutton_prompt_text = r"\$ |>>> "
copybutton_prompt_is_regexp = True
