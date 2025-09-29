#!/bin/bash
# setup_linux.sh - Linux setup script for GRiD PyTorch bindings

set -e  # Exit on error

echo "GRiD PyTorch Linux Setup"
echo "======================="

# Check if running on Linux
if [[ "$OSTYPE" != "linux-gnu"* ]]; then
    echo "This script is for Linux systems only."
    exit 1
fi

# Check Python version
python_version=$(python3 --version 2>&1 | awk '{print $2}' | cut -d. -f1,2)
echo "Python version: $python_version"

if [[ $(echo "$python_version >= 3.7" | bc -l) -eq 0 ]]; then
    echo "Error: Python 3.7+ required"
    exit 1
fi

# Check for CUDA
if command -v nvcc &> /dev/null; then
    cuda_version=$(nvcc --version | grep "release" | awk '{print $6}' | cut -d, -f1)
    echo "CUDA version: $cuda_version"
else
    echo "Warning: CUDA not found. GPU acceleration will not be available."
fi

# Check for required system packages
echo "Checking system dependencies..."

required_packages=("build-essential" "cmake" "ninja-build")
missing_packages=()

for package in "${required_packages[@]}"; do
    if ! dpkg -l | grep -q "^ii.*$package "; then
        missing_packages+=("$package")
    fi
done

if [ ${#missing_packages[@]} -ne 0 ]; then
    echo "Missing packages: ${missing_packages[*]}"
    echo "Install with: sudo apt-get install ${missing_packages[*]}"
    read -p "Install missing packages now? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo apt-get update
        sudo apt-get install "${missing_packages[@]}"
    else
        echo "Please install missing packages and run this script again."
        exit 1
    fi
fi

# Install Python dependencies
echo "Installing Python dependencies..."
python3 -m pip install --upgrade pip
python3 -m pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
python3 -m pip install numpy

# Set up environment variables
echo "Setting up environment variables..."
export TORCH_CUDA_ARCH_LIST="7.5 8.0 8.6 8.9"
export CUDA_HOME="/usr/local/cuda"

# Add to bashrc if not already present
if ! grep -q "TORCH_CUDA_ARCH_LIST" ~/.bashrc; then
    echo 'export TORCH_CUDA_ARCH_LIST="7.5 8.0 8.6 8.9"' >> ~/.bashrc
fi

if ! grep -q "CUDA_HOME" ~/.bashrc; then
    echo 'export CUDA_HOME="/usr/local/cuda"' >> ~/.bashrc
fi

# Test the setup
echo "Testing setup..."
python3 test_linux_compatibility.py

# Install the package
echo "Installing GRiD PyTorch bindings..."
python3 -m pip install -e .

echo ""
echo "Setup complete! You can now use GRiD PyTorch bindings on Linux."
echo "Note: You may need to restart your shell or run 'source ~/.bashrc' to load environment variables."