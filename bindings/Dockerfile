# Multi-stage Dockerfile for GRiD PyTorch CUDA bindings
# Supports multiple GPU architectures and PyTorch versions

ARG CUDA_VERSION=11.8.0
ARG CUDNN_VERSION=8
ARG UBUNTU_VERSION=22.04
ARG PYTHON_VERSION=3.10

# Stage 1: Base CUDA + Python
FROM nvidia/cuda:${CUDA_VERSION}-cudnn${CUDNN_VERSION}-devel-ubuntu${UBUNTU_VERSION} AS cuda-base

ENV DEBIAN_FRONTEND=noninteractive
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}
ENV TORCH_CUDA_ARCH_LIST="7.0;7.5;8.0;8.6;8.9;9.0"

RUN apt-get update && apt-get install -y \
    build-essential cmake ninja-build git wget curl \
    libopenblas-dev liblapack-dev libgtest-dev \
    python${PYTHON_VERSION} python${PYTHON_VERSION}-dev python${PYTHON_VERSION}-venv \
    python3-pip python3-setuptools python3-wheel \
    && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/python python /usr/bin/python${PYTHON_VERSION} 1 && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python${PYTHON_VERSION} 1

RUN python -m pip install --upgrade pip setuptools wheel

# Stage 2: PyTorch
FROM cuda-base AS pytorch-base

RUN pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

RUN pip install numpy scipy matplotlib jupyter ipython pandas scikit-learn tqdm pyyaml tensorboard

# Stage 3: Build GRiD
FROM pytorch-base AS grid-build

WORKDIR /workspace/grid

# Copy repo files
COPY include/ /workspace/grid/include/
COPY src/ /workspace/grid/src/
COPY tests/ /workspace/grid/tests/
COPY generateGRiD.py /workspace/grid/
COPY CMakeLists.txt /workspace/grid/

# Generate grid.cuh from URDF (user mounts URDF at runtime)
# Example: docker run -v $(pwd)/robot.urdf:/workspace/grid/robot.urdf grid
RUN python generateGRiD.py /workspace/grid/robot.urdf || true

# Configure & build with CMake
RUN cmake -B build -S . -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_ARCHITECTURES="70;75;80;86;89;90" \
    && cmake --build build -j$(nproc)

# Stage 4: Runtime
FROM pytorch-base AS runtime

WORKDIR /workspace/grid

# Copy built shared library + python sources
COPY --from=grid-build /workspace/grid/build/*.so /workspace/grid/
COPY --from=grid-build /workspace/grid/src/*.py /workspace/grid/src/
COPY --from=grid-build /workspace/grid/include /workspace/grid/include

ENV PYTHONPATH=/workspace/grid/src:$PYTHONPATH

# Add a small self-test
RUN cat > /workspace/grid/test_installation.py <<EOF
import torch
import sys
print("PyTorch version:", torch.__version__)
print("CUDA available:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("CUDA device:", torch.cuda.get_device_name(0))
try:
    import grid_torch
    print("GRiD bindings imported successfully!")
except ImportError as e:
    print("Failed to import GRiD:", e)
EOF

ENTRYPOINT ["python", "/workspace/grid/test_installation.py"]
