"""
compile_jit.py - JIT compilation for quick testing without setup.py
"""

import torch
from torch.utils.cpp_extension import load
import os
import glob

def compile_grid_torch(grid_path: str, verbose: bool = True):
    """
    Compile GRiD PyTorch extension using JIT compilation
    
    Args:
        grid_path: Path to the GRiD source directory
        verbose: Whether to show compilation output
    
    Returns:
        Compiled module
    """
    
    # Find source files
    cpp_sources = ['grid_torch_ops.cpp']
    cuda_sources = glob.glob(os.path.join(grid_path, 'src', '*.cu'))
    
    # Include directories
    include_dirs = [
        grid_path,
        os.path.join(grid_path, 'include'),
        os.path.join(grid_path, 'src'),
    ]
    
    # Compilation flags
    extra_cflags = ['-O3', '-std=c++14']
    extra_cuda_cflags = [
        '-O3',
        '-gencode=arch=compute_70,code=sm_70',
        '-gencode=arch=compute_75,code=sm_75',
        '-gencode=arch=compute_80,code=sm_80',
        '-gencode=arch=compute_86,code=sm_86',
    ]
    
    # Compile
    module = load(
        name='grid_torch_cpp',
        sources=cpp_sources + cuda_sources,
        extra_include_paths=include_dirs,
        extra_cflags=extra_cflags,
        extra_cuda_cflags=extra_cuda_cflags,
        extra_ldflags=['-lcudart', '-lcublas', '-lcusolver'],
        verbose=verbose,
        with_cuda=True,
    )
    
    return module

if __name__ == "__main__":
    # Example usage
    import argparse
    
    parser = argparse.ArgumentParser(description='Compile GRiD PyTorch extension')
    parser.add_argument('--grid-path', type=str, required=True,
                        help='Path to GRiD source directory')
    parser.add_argument('--verbose', action='store_true',
                        help='Show compilation output')
    
    args = parser.parse_args()
    
    print(f"Compiling GRiD PyTorch extension from {args.grid_path}...")
    module = compile_grid_torch(args.grid_path, args.verbose)
    print("Compilation successful!")
    
    # Test the module
    print(f"Number of joints: {module.NUM_JOINTS}")
    print(f"Number of end effectors: {module.NUM_EES}")