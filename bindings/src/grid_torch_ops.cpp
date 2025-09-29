// grid_torch_ops.cpp
#include <torch/extension.h>
#include <cuda_runtime.h>
#include "grid.cuh"
#include <vector>

// Helper to check CUDA tensors
#define CHECK_CUDA(x) TORCH_CHECK(x.device().is_cuda(), #x " must be a CUDA tensor")
#define CHECK_CONTIGUOUS(x) TORCH_CHECK(x.is_contiguous(), #x " must be contiguous")
#define CHECK_INPUT(x) CHECK_CUDA(x); CHECK_CONTIGUOUS(x)

template <typename scalar_t>
class TorchGRiD {
private:
    scalar_t gravity;
    dim3 dimms;
    grid::gridData<scalar_t>* grid_data;
    cudaStream_t* streams;
    grid::robotModel<scalar_t>* d_robot_model;

public:
    TorchGRiD(scalar_t g = static_cast<scalar_t>(9.81)) {
        gravity = g;
        dimms = dim3(grid::SUGGESTED_THREADS, 1, 1);
        streams = grid::init_grid<scalar_t>();
        d_robot_model = grid::init_robotModel<scalar_t>();
        grid_data = grid::init_gridData<scalar_t, 1>();
    }

    ~TorchGRiD() {
        grid::close_grid<scalar_t>(streams, d_robot_model, grid_data);
    }

    void load_state_from_torch(torch::Tensor q, torch::Tensor qd, torch::Tensor u) {
        CHECK_INPUT(q);
        CHECK_INPUT(qd);
        CHECK_INPUT(u);
        
        TORCH_CHECK(q.size(0) == grid::NUM_JOINTS, "q must have size NUM_JOINTS");
        TORCH_CHECK(qd.size(0) == grid::NUM_JOINTS, "qd must have size NUM_JOINTS");
        TORCH_CHECK(u.size(0) == grid::NUM_JOINTS, "u must have size NUM_JOINTS");

        scalar_t* q_ptr = q.data_ptr<scalar_t>();
        scalar_t* qd_ptr = qd.data_ptr<scalar_t>();
        scalar_t* u_ptr = u.data_ptr<scalar_t>();

        // Copy directly to device memory
        cudaMemcpy(grid_data->d_q_qd_u, q_ptr, grid::NUM_JOINTS * sizeof(scalar_t), cudaMemcpyDeviceToDevice);
        cudaMemcpy(grid_data->d_q_qd_u + grid::NUM_JOINTS, qd_ptr, grid::NUM_JOINTS * sizeof(scalar_t), cudaMemcpyDeviceToDevice);
        cudaMemcpy(grid_data->d_q_qd_u + 2 * grid::NUM_JOINTS, u_ptr, grid::NUM_JOINTS * sizeof(scalar_t), cudaMemcpyDeviceToDevice);
        
        cudaDeviceSynchronize();
    }

    torch::Tensor inverse_dynamics_torch(torch::Tensor q, torch::Tensor qd, torch::Tensor u) {
        load_state_from_torch(q, qd, u);
        
        grid::inverse_dynamics<scalar_t, false, false>(
            grid_data, d_robot_model, gravity, 1,
            dim3(1, 1, 1), dimms, streams
        );
        
        // Create output tensor on GPU
        auto options = torch::TensorOptions()
            .dtype(q.dtype())
            .device(q.device());
        
        torch::Tensor output = torch::empty({grid::NUM_JOINTS}, options);
        
        // Copy result directly to tensor
        cudaMemcpy(output.data_ptr<scalar_t>(), grid_data->d_c, 
                   grid::NUM_JOINTS * sizeof(scalar_t), cudaMemcpyDeviceToDevice);
        
        return output;
    }

    std::vector<torch::Tensor> inverse_dynamics_gradient_torch(torch::Tensor q, torch::Tensor qd, torch::Tensor u) {
        load_state_from_torch(q, qd, u);
        
        grid::inverse_dynamics_gradient<scalar_t, true, false>(
            grid_data, d_robot_model, gravity, 1, 
            dim3(1, 1, 1), dimms, streams
        );
        
        auto options = torch::TensorOptions()
            .dtype(q.dtype())
            .device(q.device());
        
        torch::Tensor dc_dq = torch::empty({grid::NUM_JOINTS, grid::NUM_JOINTS}, options);
        torch::Tensor dc_dqd = torch::empty({grid::NUM_JOINTS, grid::NUM_JOINTS}, options);
        
        // Copy gradients to tensors
        cudaMemcpy(dc_dq.data_ptr<scalar_t>(), grid_data->d_dc_du, 
                   grid::NUM_JOINTS * grid::NUM_JOINTS * sizeof(scalar_t), 
                   cudaMemcpyDeviceToDevice);
        
        cudaMemcpy(dc_dqd.data_ptr<scalar_t>(), 
                   grid_data->d_dc_du + grid::NUM_JOINTS * grid::NUM_JOINTS, 
                   grid::NUM_JOINTS * grid::NUM_JOINTS * sizeof(scalar_t), 
                   cudaMemcpyDeviceToDevice);
        
        return {dc_dq, dc_dqd};
    }

    torch::Tensor forward_dynamics_torch(torch::Tensor q, torch::Tensor qd, torch::Tensor u) {
        load_state_from_torch(q, qd, u);
        
        grid::forward_dynamics<scalar_t>(
            grid_data, d_robot_model, gravity, 1, 
            dim3(1, 1, 1), dimms, streams
        );
        
        auto options = torch::TensorOptions()
            .dtype(q.dtype())
            .device(q.device());
        
        torch::Tensor output = torch::empty({grid::NUM_JOINTS}, options);
        
        cudaMemcpy(output.data_ptr<scalar_t>(), grid_data->d_qdd, 
                   grid::NUM_JOINTS * sizeof(scalar_t), cudaMemcpyDeviceToDevice);
        
        return output;
    }

    std::vector<torch::Tensor> forward_dynamics_gradient_torch(torch::Tensor q, torch::Tensor qd, torch::Tensor u) {
        load_state_from_torch(q, qd, u);
        
        grid::forward_dynamics_gradient<scalar_t, false>(
            grid_data, d_robot_model, gravity, 1, 
            dim3(1, 1, 1), dimms, streams
        );
        
        auto options = torch::TensorOptions()
            .dtype(q.dtype())
            .device(q.device());
        
        torch::Tensor df_dq = torch::empty({grid::NUM_JOINTS, grid::NUM_JOINTS}, options);
        torch::Tensor df_dqd = torch::empty({grid::NUM_JOINTS, grid::NUM_JOINTS}, options);
        torch::Tensor df_du = torch::empty({grid::NUM_JOINTS, grid::NUM_JOINTS}, options);
        
        // Copy gradients
        cudaMemcpy(df_dq.data_ptr<scalar_t>(), grid_data->d_df_du, 
                   grid::NUM_JOINTS * grid::NUM_JOINTS * sizeof(scalar_t), 
                   cudaMemcpyDeviceToDevice);
        
        cudaMemcpy(df_dqd.data_ptr<scalar_t>(), 
                   grid_data->d_df_du + grid::NUM_JOINTS * grid::NUM_JOINTS, 
                   grid::NUM_JOINTS * grid::NUM_JOINTS * sizeof(scalar_t), 
                   cudaMemcpyDeviceToDevice);
        
        cudaMemcpy(df_du.data_ptr<scalar_t>(), 
                   grid_data->d_df_du + 2 * grid::NUM_JOINTS * grid::NUM_JOINTS, 
                   grid::NUM_JOINTS * grid::NUM_JOINTS * sizeof(scalar_t), 
                   cudaMemcpyDeviceToDevice);
        
        return {df_dq, df_dqd, df_du};
    }

    torch::Tensor minv_torch(torch::Tensor q, torch::Tensor qd, torch::Tensor u) {
        load_state_from_torch(q, qd, u);
        
        grid::direct_minv<scalar_t, false>(
            grid_data, d_robot_model, 1, 
            dim3(1, 1, 1), dimms, streams
        );
        
        auto options = torch::TensorOptions()
            .dtype(q.dtype())
            .device(q.device());
        
        torch::Tensor output = torch::empty({grid::NUM_JOINTS, grid::NUM_JOINTS}, options);
        
        cudaMemcpy(output.data_ptr<scalar_t>(), grid_data->d_Minv, 
                   grid::NUM_JOINTS * grid::NUM_JOINTS * sizeof(scalar_t), 
                   cudaMemcpyDeviceToDevice);
        
        return output;
    }

    torch::Tensor end_effector_positions_torch(torch::Tensor q, torch::Tensor qd, torch::Tensor u) {
        load_state_from_torch(q, qd, u);
        
        grid::end_effector_positions<scalar_t, false>(
            grid_data, d_robot_model, 1, 
            dim3(1, 1, 1), dimms, streams
        );
        
        auto options = torch::TensorOptions()
            .dtype(q.dtype())
            .device(q.device());
        
        torch::Tensor output = torch::empty({6 * grid::NUM_EES}, options);
        
        cudaMemcpy(output.data_ptr<scalar_t>(), grid_data->d_eePos, 
                   6 * grid::NUM_EES * sizeof(scalar_t), 
                   cudaMemcpyDeviceToDevice);
        
        return output;
    }

    torch::Tensor end_effector_gradients_torch(torch::Tensor q, torch::Tensor qd, torch::Tensor u) {
        load_state_from_torch(q, qd, u);
        
        grid::end_effector_positions_gradient<scalar_t, false>(
            grid_data, d_robot_model, 1, 
            dim3(1, 1, 1), dimms, streams
        );
        
        auto options = torch::TensorOptions()
            .dtype(q.dtype())
            .device(q.device());
        
        torch::Tensor output = torch::empty({6, grid::NUM_EES * grid::NUM_JOINTS}, options);
        
        cudaMemcpy(output.data_ptr<scalar_t>(), grid_data->d_deePos, 
                   6 * grid::NUM_EES * grid::NUM_JOINTS * sizeof(scalar_t), 
                   cudaMemcpyDeviceToDevice);
        
        return output;
    }
};

// Global instance management
std::unique_ptr<TorchGRiD<float>> grid_float_instance;
std::unique_ptr<TorchGRiD<double>> grid_double_instance;

// Initialize the grid with a specific robot
void init_grid_float(float gravity) {
    grid_float_instance = std::make_unique<TorchGRiD<float>>(gravity);
}

void init_grid_double(double gravity) {
    grid_double_instance = std::make_unique<TorchGRiD<double>>(gravity);
}

// Wrapper functions that dispatch based on tensor dtype
torch::Tensor inverse_dynamics(torch::Tensor q, torch::Tensor qd, torch::Tensor u) {
    if (q.dtype() == torch::kFloat32) {
        TORCH_CHECK(grid_float_instance, "GRiD float instance not initialized");
        return grid_float_instance->inverse_dynamics_torch(q, qd, u);
    } else if (q.dtype() == torch::kFloat64) {
        TORCH_CHECK(grid_double_instance, "GRiD double instance not initialized");
        return grid_double_instance->inverse_dynamics_torch(q, qd, u);
    } else {
        TORCH_CHECK(false, "Unsupported dtype. Only float32 and float64 are supported.");
    }
}

std::vector<torch::Tensor> inverse_dynamics_gradient(torch::Tensor q, torch::Tensor qd, torch::Tensor u) {
    if (q.dtype() == torch::kFloat32) {
        TORCH_CHECK(grid_float_instance, "GRiD float instance not initialized");
        return grid_float_instance->inverse_dynamics_gradient_torch(q, qd, u);
    } else {
        TORCH_CHECK(grid_double_instance, "GRiD double instance not initialized");
        return grid_double_instance->inverse_dynamics_gradient_torch(q, qd, u);
    }
}

torch::Tensor forward_dynamics(torch::Tensor q, torch::Tensor qd, torch::Tensor u) {
    if (q.dtype() == torch::kFloat32) {
        TORCH_CHECK(grid_float_instance, "GRiD float instance not initialized");
        return grid_float_instance->forward_dynamics_torch(q, qd, u);
    } else {
        TORCH_CHECK(grid_double_instance, "GRiD double instance not initialized");
        return grid_double_instance->forward_dynamics_torch(q, qd, u);
    }
}

std::vector<torch::Tensor> forward_dynamics_gradient(torch::Tensor q, torch::Tensor qd, torch::Tensor u) {
    if (q.dtype() == torch::kFloat32) {
        TORCH_CHECK(grid_float_instance, "GRiD float instance not initialized");
        return grid_float_instance->forward_dynamics_gradient_torch(q, qd, u);
    } else {
        TORCH_CHECK(grid_double_instance, "GRiD double instance not initialized");
        return grid_double_instance->forward_dynamics_gradient_torch(q, qd, u);
    }
}

torch::Tensor minv(torch::Tensor q, torch::Tensor qd, torch::Tensor u) {
    if (q.dtype() == torch::kFloat32) {
        TORCH_CHECK(grid_float_instance, "GRiD float instance not initialized");
        return grid_float_instance->minv_torch(q, qd, u);
    } else {
        TORCH_CHECK(grid_double_instance, "GRiD double instance not initialized");
        return grid_double_instance->minv_torch(q, qd, u);
    }
}

torch::Tensor end_effector_positions(torch::Tensor q, torch::Tensor qd, torch::Tensor u) {
    if (q.dtype() == torch::kFloat32) {
        TORCH_CHECK(grid_float_instance, "GRiD float instance not initialized");
        return grid_float_instance->end_effector_positions_torch(q, qd, u);
    } else {
        TORCH_CHECK(grid_double_instance, "GRiD double instance not initialized");
        return grid_double_instance->end_effector_positions_torch(q, qd, u);
    }
}

torch::Tensor end_effector_gradients(torch::Tensor q, torch::Tensor qd, torch::Tensor u) {
    if (q.dtype() == torch::kFloat32) {
        TORCH_CHECK(grid_float_instance, "GRiD float instance not initialized");
        return grid_float_instance->end_effector_gradients_torch(q, qd, u);
    } else {
        TORCH_CHECK(grid_double_instance, "GRiD double instance not initialized");
        return grid_double_instance->end_effector_gradients_torch(q, qd, u);
    }
}

PYBIND11_MODULE(TORCH_EXTENSION_NAME, m) {
    m.def("init_grid_float", &init_grid_float, "Initialize GRiD with float precision");
    m.def("init_grid_double", &init_grid_double, "Initialize GRiD with double precision");
    m.def("inverse_dynamics", &inverse_dynamics, "Compute inverse dynamics");
    m.def("inverse_dynamics_gradient", &inverse_dynamics_gradient, "Compute inverse dynamics gradient");
    m.def("forward_dynamics", &forward_dynamics, "Compute forward dynamics");
    m.def("forward_dynamics_gradient", &forward_dynamics_gradient, "Compute forward dynamics gradient");
    m.def("minv", &minv, "Compute mass matrix inverse");
    m.def("end_effector_positions", &end_effector_positions, "Compute end effector positions");
    m.def("end_effector_gradients", &end_effector_gradients, "Compute end effector position gradients");
    
    m.attr("NUM_JOINTS") = grid::NUM_JOINTS;
    m.attr("NUM_EES") = grid::NUM_EES;
}