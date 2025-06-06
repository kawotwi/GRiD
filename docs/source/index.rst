.. sphinx_grid documentation master file, created by
   sphinx-quickstart on Tue Oct 29 12:04:57 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to GRiD Documentation
=============================

This is a template landing page for GRiD documentation. The structure of the site needs work, but is currently in development and open to feedback.

Add your content using ``reStructuredText`` syntax. See the
`reStructuredText <https://www.sphinx-doc.org/en/master/usage/restructuredtext/index.html>`_
documentation for details. It is also possible to convert README.md files and syntax used in README files to reStructuredText using `pandoc <https://pandoc.org/>`_.

.. code-block::
   :caption:  Bash commands to convert README.md to reStructuredText

       # For most systems
       brew install pandoc          # macOS (Homebrew)
       sudo apt install pandoc      # debian/ubuntu
       pandoc README.md -f markdown -t rst -o README.rst

``-f markdown`` specifies the input format and ``-t rst`` specifies the output format. ``-o README.rst`` specifies the output file.


.. toctree::
   :maxdepth: 3
   :hidden:

   user_guide/landing_page
   api_reference/index
   contribution_guidelines
   sphinx_edit_guide
   faq
   todo_list

Citation 
--------

If you use GRiD in your research, please cite using our bibtex citation: 

.. code-block:: text

   @inproceedings{plancher2022grid,
     title={GRiD: GPU-Accelerated Rigid Body Dynamics with Analytical Gradients}, 
     author={Brian Plancher and Sabrina M. Neuman and Radhika Ghosal and Scott Kuindersma and Vijay Janapa Reddi},
     booktitle={IEEE International Conference on Robotics and Automation (ICRA)}, 
     year={2022}, 
     month={May}
   }
   
