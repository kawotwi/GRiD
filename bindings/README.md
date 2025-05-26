PyGrid: Python Bindings for CUDA Grid Dynamics

This package provides Python bindings for the CUDA-based Grid Dynamics library using pybind11.
Requirements

    C++11 compatible compiler
    CUDA Toolkit (compatible with compute capability 86)
    CMake >= 3.10
    Python >= 3.6
    pybind11

If no ''grid.cuh'' file included, run the following which will generate a grid.cuh file and move it into the include file:

python3 init_include.py urdfs/URDF_FILENAME 

Installation
Option 1: Install using pip

bash

pip install .

Option 2: Manual build

bash

mkdir build
cd build
cmake ..
make
cd ..

Usage

Here's a basic example showing how to use the PyGrid library:

python

import numpy as np
import gridCuda

# Create a grid instance with default gravity (9.81)
grid = gridCuda.GRidDataFloat()  # or GRidDataDouble for double precision

# Set joint positions, velocities, and control inputs
q = np.random.normal(0, 1, gridCuda.NUM_JOINTS).astype(np.float32)
qd = np.random.normal(0, 1, gridCuda.NUM_JOINTS).astype(np.float32)
u = np.random.normal(0, 1, gridCuda.NUM_JOINTS).astype(np.float32)

grid.load_joint_info(q, qd, u)

# Calculate inverse dynamics
c = grid.inverse_dynamics()


See example_pygrid.py for a more detailed example.
API Reference
Classes

    GridDataFloat: Single-precision (float) implementation
    GridDataDouble: Double-precision (double) implementation

Methods

    load_joint_info(q, qd, u): Set joint positions, velocities, and control inputs
    inverse_dynamics(): Calculate inverse dynamics

Constants

    NUM_JOINTS: Number of joints in the robot model


