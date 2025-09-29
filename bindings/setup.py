"""
setup.py - Build script for GRiD PyTorch CUDA extension
"""

from setuptools import setup, find_packages
import torch
import os
import glob
import warnings

try:
    from torch.utils.cpp_extension import BuildExtension, CUDAExtension
    HAS_CUDA_EXTENSION = True
except ImportError:
    HAS_CUDA_EXTENSION = False
    print("Warning: torch.utils.cpp_extension not available, building without CUDA extensions")

# Platform detection
def get_platform_flags():
    """Get platform-specific compiler flags"""
    if os.name == 'nt':  # Windows
        return {
            'cxx': ['/O2', '/std:c++17'],
            'nvcc': ['-O3', '--std=c++17']
        }
    else:  # Linux/Unix
        return {
            'cxx': ['-O3', '-std=c++17'],
            'nvcc': ['-O3', '--std=c++17']
        }

# Get CUDA compute capabilities for the current GPU
def get_compute_capabilities():
    """Get compute capabilities of available GPUs"""
    # Default capabilities for common GPUs
    default_caps = ['70', '75', '80', '86', '89', '90']
    
    try:
        # Try to detect actual GPU capabilities
        if torch.cuda.is_available():
            capability = torch.cuda.get_device_capability(0)
            cap_str = str(capability[0]) + str(capability[1])
            if cap_str not in default_caps:
                default_caps.append(cap_str)
            print(f"Detected GPU compute capability: {capability[0]}.{capability[1]}")
            return default_caps
    except Exception as e:
        print(f"Could not detect GPU capabilities: {e}")
    
    print(f"Using default compute capabilities: {default_caps}")
    return default_caps

# Handle CUDA version mismatch
def check_cuda_compatibility():
    """Check and handle CUDA version compatibility"""
    try:
        import subprocess
        result = subprocess.run(['nvcc', '--version'], capture_output=True, text=True)
        if result.returncode == 0:
            output = result.stdout
            # Extract CUDA version
            for line in output.split('\n'):
                if 'release' in line:
                    cuda_version = line.split('release')[1].split(',')[0].strip()
                    torch_cuda = torch.version.cuda
                    if cuda_version != torch_cuda:
                        warnings.warn(
                            f"CUDA version mismatch: System has {cuda_version}, "
                            f"PyTorch compiled with {torch_cuda}. "
                            f"This may cause compilation issues.",
                            UserWarning
                        )
                    break
    except Exception:
        pass  # Ignore CUDA detection failures

# Check CUDA compatibility
check_cuda_compatibility()

# Get the directory containing this setup.py
root_dir = os.path.dirname(os.path.abspath(__file__))

# Paths to GRiD source files
grid_include_path = os.path.join(root_dir, 'include')
grid_src_path = os.path.join(root_dir, 'src')

# Find all source files
cuda_sources = glob.glob(os.path.join(grid_src_path, '*.cu'))
cpp_sources = glob.glob(os.path.join(grid_src_path, '*.cpp'))

# Combine all sources
sources = cuda_sources + cpp_sources

if not sources:
    raise RuntimeError(f"No source files found in {grid_src_path}")

print(f"Found {len(sources)} source files:")
for src in sources:
    print(f"  - {os.path.basename(src)}")

# Get platform-specific flags
platform_flags = get_platform_flags()

# Get compute capabilities
compute_caps = get_compute_capabilities()

# Base CUDA compilation flags
nvcc_flags = platform_flags['nvcc'] + [
    '--use_fast_math',
    '--expt-relaxed-constexpr',
]

# Add gencode flags for each compute capability
for cap in compute_caps:
    nvcc_flags.extend([
        '-gencode', f'arch=compute_{cap},code=sm_{cap}'
    ])

# Add PTX for forward compatibility
if compute_caps:
    latest_cap = max(compute_caps)
    nvcc_flags.extend(['-gencode', f'arch=compute_{latest_cap},code=compute_{latest_cap}'])

# C++ compilation flags
cpp_flags = platform_flags['cxx']

# PyTorch-specific includes
torch_includes = torch.utils.cpp_extension.include_paths()

print(f"Using C++ flags: {cpp_flags}")
print(f"Using NVCC flags: {nvcc_flags}")

# Prepare extension modules
ext_modules = []
cmdclass = {}

if HAS_CUDA_EXTENSION and torch.cuda.is_available():
    try:
        ext_modules = [
            CUDAExtension(
                'grid_torch_cpp',
                sources=sources,
                include_dirs=[
                    grid_include_path,
                    grid_src_path,
                ] + torch_includes,
                extra_compile_args={
                    'cxx': cpp_flags,
                    'nvcc': nvcc_flags,
                },
                libraries=['cudart', 'cublas', 'cusolver'],
            ),
        ]
        cmdclass = {
            'build_ext': BuildExtension.with_options(no_python_abi_suffix=True)
        }
        print("Building with CUDA extensions")
    except Exception as e:
        print(f"Warning: Failed to setup CUDA extension: {e}")
        print("Building without CUDA extensions")
        ext_modules = []
        cmdclass = {}
else:
    print("Building without CUDA extensions (CUDA not available or extensions disabled)")

setup(
    name='grid_torch',
    version='0.1.0',
    author='Your Name',
    packages=find_packages(),
    py_modules=['grid_torch'],
    ext_modules=ext_modules,
    cmdclass=cmdclass,
    install_requires=[
        'torch>=1.9.0',
        'numpy>=1.19.0',
    ],
    python_requires='>=3.7',
)