"""
setup.py - Build script for GRiD PyTorch extension
"""
import os
import sys
import glob

# Platform-specific CUDA architecture list
if os.name == 'nt':  # Windows
    # Use semicolon separator for Windows
    os.environ['TORCH_CUDA_ARCH_LIST'] = '7.5;8.0;8.6;8.9'
else:  # Linux/Unix  
    # Use space separator for Linux
    os.environ['TORCH_CUDA_ARCH_LIST'] = '7.5 8.0 8.6 8.9'

# CMake setup
from setuptools import setup, find_packages

try:
    import torch
    from torch.utils.cpp_extension import BuildExtension, CUDAExtension
    
    # Monkey patch to disable CUDA version checking
    def _check_cuda_version(compiler_name, compiler_version):
        pass
    
    # Apply the monkey patch
    import torch.utils.cpp_extension
    torch.utils.cpp_extension._check_cuda_version = _check_cuda_version
    
    TORCH_AVAILABLE = True
except ImportError:
    print("PyTorch not available during setup. Please install PyTorch first.")
    TORCH_AVAILABLE = False

# Get the directory containing this setup.py
root_dir = os.path.dirname(os.path.abspath(__file__))

# Paths to GRiD source files
grid_include_path = os.path.join(root_dir, 'include')  # location of grid.cuh
grid_cuda_path = os.path.join(root_dir, 'src')         # location of CUDA source files

# Find all CUDA source files
cuda_sources = glob.glob(os.path.join(grid_cuda_path, '*.cu'))
# Make sure your C++ source file is included with full path
cpp_source = os.path.join(grid_cuda_path, 'grid_torch_ops.cpp')
if os.path.exists(cpp_source):
    cuda_sources.append(cpp_source)
else:
    print(f"Warning: Could not find {cpp_source}")

print(f"Found CUDA sources: {cuda_sources}")

# Setup extension modules if torch is available
ext_modules = []
cmdclass = {}

if TORCH_AVAILABLE:
    # CUDA compilation flags
    cuda_archs = []
    if torch.cuda.is_available():
        for i in range(torch.cuda.device_count()):
            major, minor = torch.cuda.get_device_capability(i)
            cuda_archs.append(f"{major}{minor}")
    else:
        cuda_archs = ['75']  # fallback

    nvcc_flags = []
    for arch in cuda_archs:
        nvcc_flags += ['-gencode', f'arch=compute_{arch},code=sm_{arch}']

    # Platform-specific compilation flags
    if os.name == 'nt':  # Windows
        cpp_flags = ['/O2', '/std:c++14']
        cuda_libraries = ['cudart', 'cublas', 'cusolver']
    else:  # Linux/Unix
        cpp_flags = ['-O3', '-std=c++14']
        cuda_libraries = ['cudart', 'cublas', 'cusolver']

    ext_modules = [
        CUDAExtension(
            name='grid_torch_cpp',
            sources=cuda_sources,
            include_dirs=[grid_include_path, grid_cuda_path],
            extra_compile_args={
                'cxx': cpp_flags,
                'nvcc': nvcc_flags,
            },
            libraries=cuda_libraries,
        ),
    ]
    cmdclass = {'build_ext': BuildExtension}

setup(
    name='grid_torch',
    version='0.1.0',
    author='Your Name',
    packages=find_packages(),
    ext_modules=ext_modules,
    cmdclass=cmdclass,
    install_requires=['torch', 'numpy'],
    python_requires='>=3.7',
)