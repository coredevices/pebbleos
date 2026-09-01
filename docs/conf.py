# SPDX-FileCopyrightText: 2025 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "PebbleOS"
copyright = "2025, The PebbleOS Contributors"
author = "The PebbleOS Contributors"
release = "0.1.0"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "myst_parser",
    "sphinx_design",
]

exclude_patterns = ["_build", "_apidoc", "Thumbs.db", ".DS_Store"]

nitpicky = True

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_book_theme"
html_static_path = ["_static"]

# Doxygen output built by the RTD pre_build job (see .readthedocs.yaml),
# published verbatim under /apidoc/
if os.path.isdir(os.path.join(os.path.dirname(__file__), "_apidoc")):
    html_extra_path = ["_apidoc"]

html_logo = "_static/images/logo.svg"
html_css_files = ["css/custom.css"]
html_theme_options = {
    "logo": {
        "text": "PebbleOS",
    },
    "repository_url": "https://github.com/coredevices/pebbleos",
    "repository_branch": "main",
    "path_to_docs": "docs",
    "use_repository_button": True,
    "use_edit_page_button": True,
    "use_issues_button": True,
}

# -- Options for the linkcheck builder ----------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-the-linkcheck-builder

# Sites that are alive but block automated requests
linkcheck_ignore = [
    r"https?://reveng\.sourceforge\.net/.*",
    r"https://https://www\.itu\.int/rec/T-REC-X\.25-199610-I/en",
]

# -- Options for myst_parser extension -----------------------------------------
# https://myst-parser.readthedocs.io/en/latest/configuration.html

myst_enable_extensions = [
    "colon_fence",
    "fieldlist",
]
