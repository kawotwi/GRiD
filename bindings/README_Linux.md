# GRiD PyTorch Bindings - Linux Installation Guide

This guide covers installation and usage of GRiD PyTorch bindings on Linux systems.

## Prerequisites

### System Requirements
- Linux distribution (Ubuntu 18.04+ recommended)
- Python 3.7 or higher
- CUDA 11.8+ (for GPU acceleration)
- GCC 7+ or Clang 6+
- CMake 3.12+

### Required System Packages

On Ubuntu/Debian:
```bash
sudo apt-get update
sudo apt-get install build-essential cmake ninja-build python3-dev
```

On CentOS/RHEL:
```bash
sudo yum install gcc gcc-c++ cmake ninja-build python3-devel
# or on newer versions:
sudo dnf install gcc gcc-c++ cmake ninja-build python3-devel
```

### CUDA Installation (Optional but Recommended)

1. Install CUDA Toolkit from NVIDIA's official website
2. Set up environment variables:
```bash
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
```

3. Add these to your `~/.bashrc` file for persistence.

## Installation

### Method 1: Automatic Setup (Recommended)
```bash
chmod +x setup_linux.sh
./setup_linux.sh
```

### Method 2: Manual Installation

1. **Install PyTorch with CUDA support:**
```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

2. **Set environment variables:**
```bash
export TORCH_CUDA_ARCH_LIST="7.5 8.0 8.6 8.9"
export CUDA_HOME="/usr/local/cuda"
```

3. **Install the package:**
```bash
pip install -e .
```

### Method 3: Development Setup

For development with hot reloading:
```bash
# Install in development mode
pip install -e .

# Test compatibility
python3 test_linux_compatibility.py
```

## Usage

### Basic Example
```python
import torch
import grid_torch

# Create GRiD instance (will compile CUDA extensions on first run)
grid = grid_torch.GRiDTorch('iiwa', device='cuda')

# Use with PyTorch tensors
q = torch.randn(7, device='cuda', requires_grad=True)
qd = torch.randn(7, device='cuda', requires_grad=True) 
u = torch.randn(7, device='cuda', requires_grad=True)

# Compute inverse dynamics with automatic differentiation
tau = grid.inverse_dynamics(q, qd, u)
tau.backward(torch.ones_like(tau))

print(f"Joint torques: {tau}")
print(f"Gradients w.r.t. q: {q.grad}")
```

### Performance Optimization

1. **CUDA Architecture Optimization:**
   The package automatically detects your GPU and compiles for the appropriate CUDA architecture. For manual control:
   ```bash
   export TORCH_CUDA_ARCH_LIST="8.6"  # For RTX 30xx series
   ```

2. **Compilation Caching:**
   PyTorch caches compiled extensions. Clear cache if needed:
   ```bash
   rm -rf ~/.cache/torch_extensions/
   ```

## Troubleshooting

### Common Issues

1. **CUDA Version Mismatch:**
   ```
   Error: CUDA version mismatch
   ```
   Solution: Ensure PyTorch CUDA version matches system CUDA:
   ```bash
   python3 -c "import torch; print(torch.version.cuda)"
   nvcc --version
   ```

2. **Missing CUDA Libraries:**
   ```
   Error: cannot find -lcudart
   ```
   Solution: Install CUDA development libraries:
   ```bash
   sudo apt-get install cuda-toolkit-11-8
   ```

3. **GCC Version Issues:**
   ```
   Error: unsupported compiler version
   ```
   Solution: CUDA requires GCC ≤ 11. Install compatible version:
   ```bash
   sudo apt-get install gcc-9 g++-9
   export CC=gcc-9
   export CXX=g++-9
   ```

4. **Permission Errors:**
   ```
   Error: Permission denied when compiling
   ```
   Solution: Ensure write permissions to temp directories:
   ```bash
   chmod 755 ~/.cache/torch_extensions/
   ```

### Debugging Compilation

Enable verbose compilation output:
```python
import os
os.environ['TORCH_EXTENSIONS_DIR'] = '/tmp/torch_extensions'

import grid_torch
# Compilation details will be printed
```

### Performance Verification

Test GPU performance vs CPU fallback:
```python
import torch
import time
import grid_torch

# Time GPU version
grid_gpu = grid_torch.GRiDTorch('iiwa', device='cuda')
q = torch.randn(7, device='cuda')
qd = torch.randn(7, device='cuda')
u = torch.randn(7, device='cuda')

start = time.time()
for _ in range(1000):
    tau = grid_gpu.inverse_dynamics(q, qd, u)
gpu_time = time.time() - start

print(f"GPU time: {gpu_time:.4f}s")
```

## Platform-Specific Notes

### Ubuntu 20.04+
- Default GCC version is compatible
- CUDA packages available via apt

### CentOS 7/8
- May need to enable EPEL repository
- Use `yum` instead of `apt-get`

### Arch Linux
- Use `pacman -S` for package installation
- CUDA available in AUR

### Docker Usage
```dockerfile
FROM nvidia/cuda:11.8-devel-ubuntu20.04

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential cmake ninja-build python3-dev python3-pip

# Install PyTorch
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Copy and install GRiD
COPY . /app/grid-torch
WORKDIR /app/grid-torch
RUN pip install -e .

# Set environment variables
ENV CUDA_HOME=/usr/local/cuda
ENV TORCH_CUDA_ARCH_LIST="7.5 8.0 8.6 8.9"
```

## Contributing

When developing on Linux:
1. Use the development setup method
2. Check compilation with verbose output enabled
3. Test on multiple GPU architectures if available

## Support

For Linux-specific issues:
1. Verify CUDA installation and environment variables
2. Enable verbose compilation to diagnose build issues
3. Check PyTorch CUDA compatibility
