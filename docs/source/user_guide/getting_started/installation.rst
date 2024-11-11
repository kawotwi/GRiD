Installation
============

It is recommended when installing GRiD to use git clone on the main library `GRiD <https://github.com/robot-acceleration/GRiD>`.
In the case that the linked submodules :doc:`URDFParser <tutorials/urdf_parser>`, :doc:`GRiDCodeGenerator <tutorials/codegen>`, :doc:`RBDReference <tutorials/python_algorithms>` and are empty folders,
one can individually popoulate each submodule using its git link as needed. 

Run the following script to install GRiD and its related submodules.
`URDFParser <https://github.com/robot-acceleration/URDFParser>`__,
`GRiDCodeGenerator <https://github.com/robot-acceleration/GRiDCodeGenerator>`__,
and
`RBDReference <https://github.com/robot-acceleration/RBDReference>`__
.. code:: shell

    # In the root of your desired project directory
    git clone https://github.com/robot-acceleration/GRiD.git
    cd RBDReference
    git clone https://github.com/robot-acceleration/RBDReference.git
    cd ..
    cd URDFParser
    git clone https://github.com/robot-acceleration/URDFParser.git
    cd ..
    cd GRiDCodeGenerator
    git clone https://github.com/robot-acceleration/GRiDCodeGenerator.git

.. note::
    
    Alternatively, can directly download the zips from these links: `URDFParser <https://github.com/robot-acceleration/URDFParser>`__, `GRiDCodeGenerator <https://github.com/robot-acceleration/GRiDCodeGenerator>`__, and `RBDReference <https://github.com/robot-acceleration/RBDReference>`__.
    Note that directory setup in this manner requires adjustment of python ``import`` statements such that ``from URDFParser import URDFParser`` becomes ``from URDFParser.URDFParser import URDFParser``. Thus, each import statement from submodules will require an additional call for correct directory linking. 


It is also recommended to create a virtual environment for each external dependency for ease of access. Run the following script to list and update the requirements tab if other dependencies are needed during the installation process.

.. code:: shell

    # Run in virtual enviornment
    pip3 list # list all pip modules
    pip freeze > requirements.txt

Install Python Dependencies
~~~~~~~~~~~~~~~~~~~~~~~~~~~

In order to support the wrapped packages there are 4 required external
packages ``beautifulsoup4, lxml, numpy, sympy`` which can be
automatically installed by running:

.. code:: shell

   pip3 install -r requirements.txt

Install CUDA Dependencies
~~~~~~~~~~~~~~~~~~~~~~~~~

::

   sudo apt-get update
   sudo apt-get -y install xorg xorg-dev linux-headers-$(uname -r) apt-transport-https

Download and Install CUDA
~~~~~~~~~~~~~~~~~~~~~~~~~

Note: for Ubuntu 20.04 see https://developer.nvidia.com/cuda-downloads
for other distros

::

   wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
   sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
   sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub
   sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
   sudo apt-get update
   sudo apt-get -y install cuda

Add the following to ``~/.bashrc``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

   export PATH="/usr/local/cuda/bin:$PATH"
   export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"
   export PATH="opt/nvidia/nsight-compute/:$PATH"


.. note::

    This is an example of how to do a "note". Good luck with the rest of the setup! 

.. warning::

    Example of a warning.

.. tip:: 

    Here is a tip!

.. caution:: 

    And proceed with caution!

