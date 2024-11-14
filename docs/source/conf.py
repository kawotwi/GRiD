# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'GRiD'
copyright = '2024, Kwamena Awotwi, Zachary Pestrikov, Danelle Tuchman, Abhinav Sharma, Brian Plancher'
author = 'Kwamena Awotwi, Zachary Pestrikov, Danelle Tuchman, Abhinav Sharma, Brian Plancher'
release = '1.0.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

import sys
import os
sys.path.insert(0, os.path.abspath('/Users/kwam/POST_GRAD/GRiD')) # change as needed

extensions = [
	'sphinx.ext.autodoc',
	'sphinx.ext.autosummary',
	'sphinx.ext.napoleon',
	'sphinx.ext.viewcode',
    'sphinx.ext.autosectionlabel',
    'sphinx.ext.intersphinx',
]

#myst parser
myst_enable_extensions = ["colon_fence", "dollarmath"]
myst_heading_anchors = 4

# Configure autodoc
autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'special-members': '__init__',
    'undoc-members': True,
    'exclude-members': '__weakref__'
}

# confugre autosummary
autosummary_generate = True

# Configure Napoleon for Google-style docstrings
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = False
napoleon_include_private_with_doc = False
napoleon_include_special_with_doc = True
napoleon_use_admonition_for_examples = False
napoleon_use_admonition_for_notes = False
napoleon_use_admonition_for_references = False
napoleon_use_ivar = False
napoleon_use_param = True
napoleon_use_rtype = True
napoleon_type_aliases = None

templates_path = ['_templates']
exclude_patterns = []

# Enable numref feature
numfig = True

# LaTeX configuration for math
latex_elements = {
    'preamble': r'''
    \usepackage{amsmath}
    \usepackage{amsfonts}
    \usepackage{amssymb}
    ''',
}


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'pydata_sphinx_theme'
html_favicon = '_static/favicon/favicon.ico'
# html_theme = 'furo'
html_theme_options = {
    'navigation_depth': 4,
    "github_url": "https://github.com/kawotwi/GRiD", # Link to github
    "use_edit_page_button": True, # Enables edit button
        "logo": {
        "image_light": "_static/a2r_lab.jpg",
        "image_dark": "_static/a2r_lab.jpg",
    },
    "collapse_navigation": True,
    "header_links_before_dropdown": 6,
    # Add light/dark mode and documentation version switcher:
    "navbar_end": [
        "search-button",
        "theme-switcher",
        "navbar-icon-links"
    ],
    "navbar_persistent": [],
    "show_version_warning_banner": True,
}
html_static_path = ['_static']
html_css_files = ['custom.css']
html_logo = "_static/a2r_lab.jpg"

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_context = {
    "display_github": True,
    "github_user": "kawotwi",
    "github_repo": "GRiD",
    "github_version": "main",
    "conf_py_path": "/source/",
    "doc_path": "docs/source"
}



intersphinx_mapping = {'gymnasium': ('https://gymnasium.farama.org/', None)}
