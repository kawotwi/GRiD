Library Overview
=================

Each submodule of GRiD is an essential component to getting the most out of GRiD as a whole. Here we will discuss how each module relates to each other. 

Table of Contents
==================
I. `RBDReference`
II. `URDFParser`
III. `GRiDCodeGenerator`

I. RBDReference 
----------------

RBDReference is composed of one essential file ``RBDReference.py`` which hosts a class of functions responsible for the easy-to-read rigid body dynamics algorithms in Python.

It currently supports the following algorithmic functions which can be viewed from the function glossary:

* ``apply_external_forces``
* ``rnea``
* ``rnea_grad``
* ``minv``
* ``aba``
* ``crba``
* ``forward_dynamics_grad``

Each of these functions and more included within the file call upon getters from URDFParser which initializes a convenient ``robotObj``. 
Here is a list of relevant and helpful getters which can also be viewed from the function glossary for ``URDFParser``:

* ``self.robot.get_num_bodies()``
* ``self.robot.get_parent_id()``
* ``self.robot.get_joint_index_q()``
* ``self.robot.get_Xmat_Func_by_id()()`` 
* ``self.robot.get_Imat_by_id()``
* ``self.robot.get_subtree_by_id()``
* ``self.robot.get_num_vel()``
* ``self.robot.get_S_by_id()``

This is just a short list, please look to the function glossary for ``URDFParser`` for more detailed usage instructions and guidelines.

II. URDFParser 
---------------


III. GRiDCodeGenerator
-----------------------

