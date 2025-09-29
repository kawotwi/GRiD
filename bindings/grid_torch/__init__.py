"""
grid_torch.py - PyTorch wrapper for GRiD dynamics with automatic differentiation
"""

import os
import torch
import torch.autograd as autograd
from torch.utils.cpp_extension import load
import numpy as np
from typing import Optional, Tuple


class InverseDynamics(autograd.Function):
    """
    Custom autograd function for inverse dynamics computation.
    tau = M(q) * qdd + C(q, qd) + G(q) - u
    """
    
    @staticmethod
    def forward(ctx, q, qd, u):
        """
        Forward pass: compute inverse dynamics
        Args:
            q: Joint positions [NUM_JOINTS]
            qd: Joint velocities [NUM_JOINTS]
            u: Joint torques/controls [NUM_JOINTS]
        Returns:
            tau: Required torques [NUM_JOINTS]
        """
        ctx.save_for_backward(q, qd, u)
        
        # Import the C++ module (loaded elsewhere)
        import grid_torch_cpp
        
        # Call CUDA kernel
        tau = grid_torch_cpp.inverse_dynamics(q, qd, u)
        return tau
    
    @staticmethod
    def backward(ctx, grad_output):
        """
        Backward pass: compute gradients w.r.t inputs
        """
        q, qd, u = ctx.saved_tensors
        
        import grid_torch_cpp
        
        # Get gradients from CUDA
        dc_dq, dc_dqd = grid_torch_cpp.inverse_dynamics_gradient(q, qd, u)
        
        # Chain rule: grad_input = grad_output^T @ jacobian
        grad_q = grad_qd = grad_u = None
        
        if ctx.needs_input_grad[0]:  # gradient w.r.t q
            grad_q = torch.matmul(grad_output.unsqueeze(0), dc_dq).squeeze(0)
        
        if ctx.needs_input_grad[1]:  # gradient w.r.t qd
            grad_qd = torch.matmul(grad_output.unsqueeze(0), dc_dqd).squeeze(0)
        
        if ctx.needs_input_grad[2]:  # gradient w.r.t u
            # tau is linear in u with coefficient -1
            grad_u = -grad_output
        
        return grad_q, grad_qd, grad_u


class ForwardDynamics(autograd.Function):
    """
    Custom autograd function for forward dynamics computation.
    qdd = M(q)^{-1} * (u - C(q, qd) - G(q))
    """
    
    @staticmethod
    def forward(ctx, q, qd, u):
        """
        Forward pass: compute forward dynamics (accelerations)
        Args:
            q: Joint positions [NUM_JOINTS]
            qd: Joint velocities [NUM_JOINTS]
            u: Joint torques/controls [NUM_JOINTS]
        Returns:
            qdd: Joint accelerations [NUM_JOINTS]
        """
        ctx.save_for_backward(q, qd, u)
        
        import grid_torch_cpp
        
        # Call CUDA kernel
        qdd = grid_torch_cpp.forward_dynamics(q, qd, u)
        return qdd
    
    @staticmethod
    def backward(ctx, grad_output):
        """
        Backward pass: compute gradients w.r.t inputs
        """
        q, qd, u = ctx.saved_tensors
        
        import grid_torch_cpp
        
        # Get gradients from CUDA
        df_dq, df_dqd, df_du = grid_torch_cpp.forward_dynamics_gradient(q, qd, u)
        
        grad_q = grad_qd = grad_u = None
        
        if ctx.needs_input_grad[0]:  # gradient w.r.t q
            grad_q = torch.matmul(grad_output.unsqueeze(0), df_dq).squeeze(0)
        
        if ctx.needs_input_grad[1]:  # gradient w.r.t qd
            grad_qd = torch.matmul(grad_output.unsqueeze(0), df_dqd).squeeze(0)
        
        if ctx.needs_input_grad[2]:  # gradient w.r.t u
            grad_u = torch.matmul(grad_output.unsqueeze(0), df_du).squeeze(0)
        
        return grad_q, grad_qd, grad_u


class EndEffectorPositions(autograd.Function):
    """
    Custom autograd function for end effector forward kinematics
    """
    
    @staticmethod
    def forward(ctx, q, qd, u):
        """
        Forward pass: compute end effector positions
        Args:
            q: Joint positions [NUM_JOINTS]
            qd: Joint velocities [NUM_JOINTS] (not used but kept for consistency)
            u: Joint torques [NUM_JOINTS] (not used but kept for consistency)
        Returns:
            ee_pos: End effector positions [6 * NUM_EES] (x,y,z,roll,pitch,yaw for each EE)
        """
        ctx.save_for_backward(q, qd, u)
        
        import grid_torch_cpp
        
        # Call CUDA kernel
        ee_pos = grid_torch_cpp.end_effector_positions(q, qd, u)
        return ee_pos
    
    @staticmethod
    def backward(ctx, grad_output):
        """
        Backward pass: compute gradients w.r.t joint positions
        """
        q, qd, u = ctx.saved_tensors
        
        import grid_torch_cpp
        
        # Get Jacobian from CUDA
        jacobian = grid_torch_cpp.end_effector_gradients(q, qd, u)
        
        grad_q = grad_qd = grad_u = None
        
        if ctx.needs_input_grad[0]:  # gradient w.r.t q
            # Reshape for proper matrix multiplication
            # jacobian is [6, NUM_EES * NUM_JOINTS]
            # We need to extract the relevant parts for each joint
            grad_q = torch.matmul(grad_output.view(1, -1), jacobian.T).squeeze(0)
            
            # Extract only the q-related gradients
            num_joints = grid_torch_cpp.NUM_JOINTS
            grad_q = grad_q[:num_joints]
        
        # qd and u don't affect end effector positions directly
        if ctx.needs_input_grad[1]:
            grad_qd = torch.zeros_like(qd)
        
        if ctx.needs_input_grad[2]:
            grad_u = torch.zeros_like(u)
        
        return grad_q, grad_qd, grad_u


class MassMatrixInverse(autograd.Function):
    """
    Custom autograd function for mass matrix inverse computation
    """
    
    @staticmethod
    def forward(ctx, q, qd, u):
        """
        Forward pass: compute mass matrix inverse
        Args:
            q: Joint positions [NUM_JOINTS]
            qd: Joint velocities [NUM_JOINTS] (not used)
            u: Joint torques [NUM_JOINTS] (not used)
        Returns:
            Minv: Mass matrix inverse [NUM_JOINTS, NUM_JOINTS]
        """
        ctx.save_for_backward(q, qd, u)
        
        import grid_torch_cpp
        
        # Call CUDA kernel
        Minv = grid_torch_cpp.minv(q, qd, u)
        return Minv
    
    @staticmethod
    def backward(ctx, grad_output):
        """
        Backward pass: compute gradients w.r.t joint positions
        Using the formula: d(M^{-1})/dq = -M^{-1} @ dM/dq @ M^{-1}
        """
        q, qd, u = ctx.saved_tensors
        
        # For now, we'll return zero gradients as computing dM/dq is complex
        # This can be extended with finite differences or analytical derivatives
        grad_q = grad_qd = grad_u = None
        
        if ctx.needs_input_grad[0]:
            grad_q = torch.zeros_like(q)
        
        if ctx.needs_input_grad[1]:
            grad_qd = torch.zeros_like(qd)
        
        if ctx.needs_input_grad[2]:
            grad_u = torch.zeros_like(u)
        
        return grad_q, grad_qd, grad_u


class GRiDTorch:
    """
    High-level Python interface for GRiD dynamics with PyTorch
    """
    
    def __init__(self, urdf_path: Optional[str] = None, gravity: float = 9.81, 
                 dtype: torch.dtype = torch.float32, device: str = 'cuda'):
        """
        Initialize GRiD for a specific robot
        
        Args:
            urdf_path: Path to URDF file (if needed for preprocessing)
            gravity: Gravitational acceleration
            dtype: Data type (torch.float32 or torch.float64)
            device: Device to use ('cuda' or specific GPU)
        """
        self.gravity = gravity
        self.dtype = dtype
        self.device = device
        
        # Load the C++ extension
        self._load_extension()
        
        # Initialize the appropriate GRiD instance if CUDA extension is available
        if self.cpp_module is not None:
            if dtype == torch.float32:
                self.cpp_module.init_grid_float(gravity)
            elif dtype == torch.float64:
                self.cpp_module.init_grid_double(gravity)
            else:
                raise ValueError(f"Unsupported dtype: {dtype}")
            
            self.num_joints = self.cpp_module.NUM_JOINTS
            self.num_ees = self.cpp_module.NUM_EES
        else:
            # Fallback values when CUDA extension is not available
            print("Warning: CUDA extension not available, using fallback configuration")
            # These are default values for common robots (can be overridden)
            self.num_joints = 7  # Common for manipulators like IIWA
            self.num_ees = 1     # Single end effector
        
        # Store URDF path for reference
        self.urdf_path = urdf_path
    
    def _load_extension(self):
        """Load the C++ extension"""
        # Try to import pre-built module first
        try:
            import grid_torch_cpp
            self.cpp_module = grid_torch_cpp
        except ImportError:
            # If not found, compile on the fly
            print("Compiling GRiD PyTorch extension...")
            
            # Get the correct paths to source files
            current_dir = os.path.dirname(os.path.abspath(__file__))
            parent_dir = os.path.dirname(current_dir)
            src_dir = os.path.join(parent_dir, 'src')
            include_dir = os.path.join(parent_dir, 'include')
            
            # Source files with correct paths (both are .cu files)
            cuda_ops_file = os.path.join(src_dir, 'grid_torch_ops.cu')
            python_bindings_file = os.path.join(src_dir, 'python_bindings.cu')
            
            if not os.path.exists(cuda_ops_file):
                raise FileNotFoundError(f"Could not find {cuda_ops_file}")
            if not os.path.exists(python_bindings_file):
                raise FileNotFoundError(f"Could not find {python_bindings_file}")
            
            sources = [cuda_ops_file, python_bindings_file]
            
            # Platform-specific compilation flags
            if os.name == 'nt':  # Windows
                extra_cflags = ['/O2', '/std:c++17']
                extra_cuda_cflags = ['-O3', '--std=c++17']
            else:  # Linux/Unix
                extra_cflags = ['-O3', '-std=c++17']
                extra_cuda_cflags = ['-O3', '--std=c++17']
            
            # Add CUDA architecture flags for better GPU compatibility
            if torch.cuda.is_available():
                for i in range(torch.cuda.device_count()):
                    major, minor = torch.cuda.get_device_capability(i)
                    arch = f'{major}{minor}'
                    extra_cuda_cflags.extend([
                        f'-gencode=arch=compute_{arch},code=sm_{arch}',
                        f'-gencode=arch=compute_{arch},code=compute_{arch}'
                    ])
            else:
                # Fallback architectures
                extra_cuda_cflags.extend([
                    '-gencode=arch=compute_75,code=sm_75',
                    '-gencode=arch=compute_80,code=sm_80',
                    '-gencode=arch=compute_86,code=sm_86'
                ])
            
            try:
                self.cpp_module = load(
                    name='grid_torch_cpp',
                    sources=sources,
                    extra_include_paths=[include_dir, src_dir],
                    extra_cuda_cflags=extra_cuda_cflags,
                    extra_cflags=extra_cflags,
                    verbose=True
                )
            except Exception as e:
                print(f"Warning: Failed to compile CUDA extension: {e}")
                print("Falling back to CPU-only mode or using precompiled version")
                self.cpp_module = None
    
    def inverse_dynamics(self, q: torch.Tensor, qd: torch.Tensor, 
                        u: torch.Tensor) -> torch.Tensor:
        """
        Compute inverse dynamics with automatic differentiation
        
        Args:
            q: Joint positions [batch_size, NUM_JOINTS] or [NUM_JOINTS]
            qd: Joint velocities [batch_size, NUM_JOINTS] or [NUM_JOINTS]
            u: Joint torques [batch_size, NUM_JOINTS] or [NUM_JOINTS]
        
        Returns:
            tau: Required torques [batch_size, NUM_JOINTS] or [NUM_JOINTS]
        """
        # Handle batched input
        batched = q.dim() > 1
        if batched:
            batch_size = q.shape[0]
            results = []
            for i in range(batch_size):
                tau = InverseDynamics.apply(q[i], qd[i], u[i])
                results.append(tau)
            return torch.stack(results)
        else:
            return InverseDynamics.apply(q, qd, u)
    
    def forward_dynamics(self, q: torch.Tensor, qd: torch.Tensor, 
                        u: torch.Tensor) -> torch.Tensor:
        """
        Compute forward dynamics with automatic differentiation
        
        Args:
            q: Joint positions [batch_size, NUM_JOINTS] or [NUM_JOINTS]
            qd: Joint velocities [batch_size, NUM_JOINTS] or [NUM_JOINTS]
            u: Joint torques [batch_size, NUM_JOINTS] or [NUM_JOINTS]
        
        Returns:
            qdd: Joint accelerations [batch_size, NUM_JOINTS] or [NUM_JOINTS]
        """
        batched = q.dim() > 1
        if batched:
            batch_size = q.shape[0]
            results = []
            for i in range(batch_size):
                qdd = ForwardDynamics.apply(q[i], qd[i], u[i])
                results.append(qdd)
            return torch.stack(results)
        else:
            return ForwardDynamics.apply(q, qd, u)
    
    def end_effector_positions(self, q: torch.Tensor, 
                               qd: Optional[torch.Tensor] = None,
                               u: Optional[torch.Tensor] = None) -> torch.Tensor:
        """
        Compute end effector positions with automatic differentiation
        
        Args:
            q: Joint positions [batch_size, NUM_JOINTS] or [NUM_JOINTS]
            qd: Joint velocities (optional, zeros if not provided)
            u: Joint torques (optional, zeros if not provided)
        
        Returns:
            ee_pos: End effector positions [batch_size, 6*NUM_EES] or [6*NUM_EES]
        """
        # Create zero tensors if not provided
        if qd is None:
            qd = torch.zeros_like(q)
        if u is None:
            u = torch.zeros_like(q)
        
        batched = q.dim() > 1
        if batched:
            batch_size = q.shape[0]
            results = []
            for i in range(batch_size):
                ee_pos = EndEffectorPositions.apply(q[i], qd[i], u[i])
                results.append(ee_pos)
            return torch.stack(results)
        else:
            return EndEffectorPositions.apply(q, qd, u)
    
    def mass_matrix_inverse(self, q: torch.Tensor) -> torch.Tensor:
        """
        Compute mass matrix inverse
        
        Args:
            q: Joint positions [batch_size, NUM_JOINTS] or [NUM_JOINTS]
        
        Returns:
            Minv: Mass matrix inverse [batch_size, NUM_JOINTS, NUM_JOINTS] or [NUM_JOINTS, NUM_JOINTS]
        """
        qd = torch.zeros_like(q)
        u = torch.zeros_like(q)
        
        batched = q.dim() > 1
        if batched:
            batch_size = q.shape[0]
            results = []
            for i in range(batch_size):
                Minv = MassMatrixInverse.apply(q[i], qd, u)
                results.append(Minv)
            return torch.stack(results)
        else:
            return MassMatrixInverse.apply(q, qd, u)
    
    def simulate_step(self, q: torch.Tensor, qd: torch.Tensor, 
                     u: torch.Tensor, dt: float) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Simulate one time step using forward dynamics
        
        Args:
            q: Current joint positions
            qd: Current joint velocities
            u: Applied torques
            dt: Time step
        
        Returns:
            q_next: Next joint positions
            qd_next: Next joint velocities
        """
        # Compute accelerations
        qdd = self.forward_dynamics(q, qd, u)
        
        # Semi-implicit Euler integration
        qd_next = qd + qdd * dt
        q_next = q + qd_next * dt
        
        return q_next, qd_next