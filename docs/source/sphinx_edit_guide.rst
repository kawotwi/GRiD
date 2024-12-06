========================================
Editing Sphinx Documentation
========================================

Welcome to the guide on how to edit and deploy the Sphinx documentation for our project. This document will provide instructions on how to modify, build, and deploy the documentation locally and on the web.

Follow the instructions below for tips on how to edit, build, deploy, and run the code locally. The docs folder currently has the following structure:

.. code:: shell

    docs
    ├── source/
    │   ├── _static/
    │   │   ├──custom.css
    |   |   └── etc..
    │   ├── _templates/
    │   ├── user_guide/
    |   |   ├── concepts
    |   |   |   ├── algorithms
    |   |   |   |   ├── aba.rst
    |   |   |   |   └── etc..
    |   |   |   └── index.rst
    |   |   ├── getting_started
    |   |   |   ├── docker_setup.rst
    |   |   |   ├── installation.rst
    |   |   |   ├── library_overview.rst
    |   |   |   └── etc..
    |   |   ├── tutorials
    |   |   |   ├── codegen.rst
    |   |   |   ├── python_algorithms.rst
    |   |   |   ├── urdf_parser.rst
    |   |   |   └── etc..
    │   ├── conf.py
    │   └── index.rst
    └── Makefile & etc...


.. note::
    When making changes to any of the above files, note the existence of a ``toctree`` at the bottom of some files.
    ``index.rst`` files typically point to other files. In order for your new file to be a part of documentation, ensure that it is included in the toctree in its folder or the previous folder.
    Also note that you can write files in the format of a README.md or .rst format. Both are quick to learn and use Latex.


Table of Contents
------------------
1. `Editing the Documentation`
2. `Building the Documentation Locally`
3. `Deploying the Documentation`
4. `Editing Code and Running Locally`
5. `Deploying the Code`

Editing the Documentation
=========================

To begin editing the documentation, follow these steps:

1. **Clone the repository**:
   If you haven't already cloned the repository, use the following command to clone the project:

.. code::bash
   git clone https://github.com/A2R-Lab/GRiD.git


2. **Locate the Documentation Files**:
    The Sphinx documentation files are located in the docs/ directory (or a similar name depending on the project setup). 
    Inside this directory, you will find index.rst (the main entry point) and other .rst files that contain the content.

3. **Edit .rst Files**: 
    To edit the documentation, open any .rst file using a text editor (such as VSCode, Atom, or Sublime Text).
    The primary file is index.rst, but other files are organized in subdirectories.

4. **Preview Changes Locally**:
    After making changes you can build locally to preview the changes. See Makefile for instructions for deployment (can use ``make all``)

Building Documentaiton Locally
==============================

1. **Install Dependencies**:
    Before building the documentation, ensure you have the necessary dependencies installed. 
    You can install the dependencies using the following command:

.. code:: bash

    pip install sphinx
    pip install furo # html theme
    pip install sphinx-rtd-theme #read the docs theme

.. note::

    If there are missing themes, easily pip install them as well as extensions.


.. code:: bash

    python3 -m venv venvsource venv/bin/activate
    pip3 install -r requirements.txt

2. **Build the Documentation**:
    sphinx-build -b html docs/source docs/build

3. **View Locally**:
    After building the documentation, you can view it locally by opening the index.html file in your browser.

.. code:: bash

    # without using npm
    sphinx-build -b html . ../build # this updates files and builds 
    python -m http.server 8000 #

    # using npm
    npm install -g http-server
    http-server ./build # run to view server in docs directory


To be completed...