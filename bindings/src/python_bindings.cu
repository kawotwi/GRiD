#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include "grid.cuh" // Path to the grid.cuh header
#include <cuda_runtime.h>

namespace py = pybind11;

// Helper: convert numpy array to raw ptr
template <typename T>
T* get_data_ptr(py::array_t<T, py::array::c_style> array) {
    return static_cast<T*>(array.request().ptr);
}

// Python wrapper class for GRiD
template <typename T>
class PyGRidData {
private:
    T gravity;
    dim3 dimms;
    grid::gridData<T>* grid_data;
    cudaStream_t* streams;
    grid::robotModel<T>* d_robot_model;

public:
    PyGRidData(T g = static_cast<T>(9.81)) {
        gravity = g;
        dimms = dim3(grid::SUGGESTED_THREADS, 1, 1);
        streams = grid::init_grid<T>();
        d_robot_model = grid::init_robotModel<T>();
        grid_data = grid::init_gridData<T, 1>();
    }

    ~PyGRidData() {
        grid::close_grid<T>(streams, d_robot_model, grid_data);
    }

    void load_joint_info(py::array_t<T, py::array::c_style> q,
                          py::array_t<T, py::array::c_style> qd,
                          py::array_t<T, py::array::c_style> u) {
        if (q.size() != grid::NUM_JOINTS || qd.size() != grid::NUM_JOINTS || u.size() != grid::NUM_JOINTS) {
            throw std::runtime_error("Input arrays must be of size NUM_JOINTS");
        }
        
        T* q_ptr = get_data_ptr(q);
        T* qd_ptr = get_data_ptr(qd);
        T* u_ptr = get_data_ptr(u);
        
        for (int i = 0; i < grid::NUM_JOINTS; i++) {
            grid_data->h_q_qd_u[i] = q_ptr[i];
            grid_data->h_q_qd_u[i + grid::NUM_JOINTS] = qd_ptr[i];
            grid_data->h_q_qd_u[i + 2 * grid::NUM_JOINTS] = u_ptr[i];
        }
        
        gpuErrchk(cudaMemcpy(grid_data->d_q_qd_u, grid_data->h_q_qd_u,
                             3 * grid::NUM_JOINTS * sizeof(T), cudaMemcpyHostToDevice));
        gpuErrchk(cudaDeviceSynchronize());
    }

    py::array_t<T> inverse_dynamics() {
        grid::inverse_dynamics<T, false, false>(
            grid_data, d_robot_model, gravity, 1,
            dim3(1, 1, 1), dimms, streams
        );
        
        auto result = py::array_t<T>(grid::NUM_JOINTS);
        py::buffer_info buf = result.request();
        T* ptr = static_cast<T*>(buf.ptr);
        std::memcpy(ptr, grid_data->h_c, grid::NUM_JOINTS * sizeof(T));
        return result;
    }
};

// Pybind11 module
PYBIND11_MODULE(gridCuda, m) {
    m.doc() = "Python bindings for CUDA GRiD dynamics";
    
    py::class_<PyGRidData<float>>(m, "GRidDataFloat")
        .def(py::init<float>(), py::arg("gravity") = 9.81f)
        .def("load_joint_info", &PyGRidData<float>::load_joint_info,
             "Load joint positions, velocities, torques",
             py::arg("q"), py::arg("qd"), py::arg("u"))
        .def("inverse_dynamics", &PyGRidData<float>::inverse_dynamics,
             "Compute inverse dynamics");
    
    // double compatability was breaking code.
    // py::class_<PyGRidData<double>>(m, "GRidDataDouble")
    //     .def(py::init<double>(), py::arg("gravity") = 9.81)
    //     .def("load_joint_info", &PyGRidData<double>::load_joint_info,
    //          "Load joint positions, velocities, torques",
    //          py::arg("q"), py::arg("qd"), py::arg("u"))
    //     .def("inverse_dynamics", &PyGRidData<double>::inverse_dynamics,
    //          "Compute inverse dynamics");
    
    m.attr("NUM_JOINTS") = grid::NUM_JOINTS;
}