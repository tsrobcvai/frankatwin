"""Sphinx configuration for the FrankaTwin docs (sphinx-book-theme, Markdown via MyST)."""

import os
import sys
from datetime import date

sys.path.insert(0, os.path.abspath("../python"))  # autodoc imports frankatwin from the checkout

project = "FrankaTwin"
author = "Tao Sun, Patrick Yin, Harry He"
copyright = f"{date.today().year}, {author}"
version = "0.2"
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
# sphinx-book-theme: the whole table of contents lives in the left sidebar
# (captions as section headers, pages expandable); no top navigation bar.
html_theme = "sphinx_book_theme"
html_title = "FrankaTwin"
html_baseurl = "https://tsrobcvai.github.io/frankatwin/"   # canonical URLs for the published site
html_logo = "images/favicon.png"
html_favicon = "images/favicon.png"
html_static_path = ["_static"]
html_css_files = ["custom.css"]
html_last_updated_fmt = "%Y-%m-%d"
html_theme_options = {
    "repository_url": "https://github.com/tsrobcvai/frankatwin",
    "repository_branch": "v0.2",
    "path_to_docs": "docs",
    "use_repository_button": True,
    "use_edit_page_button": False,
    "use_issues_button": False,
    "use_download_button": False,
    "use_fullscreen_button": False,
    "home_page_in_toc": True,
    "show_navbar_depth": 1,        # sections collapsed to page level; expand on click
    "max_navbar_depth": 3,
    "show_toc_level": 2,           # right-hand "On this page"
    "navigation_with_keys": True,
    "logo": {"text": "FrankaTwin v0.2"},
}
copybutton_prompt_text = r"\$ |>>> "
copybutton_prompt_is_regexp = True
