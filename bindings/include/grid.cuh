/**
 * This instance of grid.cuh is optimized for the urdf: hyq
 *
 * Notes:
 *   Interface is:
 *       __host__   robotModel<T> *d_robotModel = init_robotModel<T>()
 *       __host__   cudaStream_t streams = init_grid<T>()
 *       __host__   gridData<T> *hd_ata = init_gridData<T,NUM_TIMESTEPS>();    __host__   close_grid<T>(cudaStream_t *streams, robotModel<T> *d_robotModel, gridData<T> *hd_data)
 *   
 *       __device__ inverse_dynamics_inner<T>(T *s_c,  T *s_vaf, const T *s_q, const T *s_qd, const T *s_qdd, T *s_XImats, int *s_topology_helpers, T *s_temp, const T gravity)
 *       __device__ inverse_dynamics_inner<T>(T *s_c,  T *s_vaf, const T *s_q, const T *s_qd, T *s_XImats, int *s_topology_helpers, T *s_temp, const T gravity)
 *       __device__ inverse_dynamics_device<T>(T *s_c, const T *s_q, const T *s_qd, const robotModel<T> *d_robotModel, const T gravity)
 *       __device__ inverse_dynamics_device<T>(T *s_c, const T *s_q, const T *s_qd, const T *s_qdd, const robotModel<T> *d_robotModel, const T gravity)
 *       __global__ inverse_dynamics_kernel<T>(T *d_c, const T *d_q_qd, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS)
 *       __global__ inverse_dynamics_kernel<T>(T *d_c, const T *d_q_qd, const T *d_qdd, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS)
 *       __host__   inverse_dynamics<T,USE_QDD_FLAG=false,USE_COMPRESSED_MEM=false>(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps, const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams)
 *   
 *       __device__ inverse_dynamics_inner_vaf<T>(T *s_vaf, const T *s_q, const T *s_qd, const T *s_qdd, T *s_XImats, int *s_topology_helpers, T *s_temp, const T gravity)
 *       __device__ inverse_dynamics_inner_vaf<T>(T *s_vaf, const T *s_q, const T *s_qd, T *s_XImats, int *s_topology_helpers, T *s_temp, const T gravity)
 *       __device__ inverse_dynamics_vaf_device<T>(T *s_vaf, const T *s_q, const T *s_qd, const robotModel<T> *d_robotModel, const T gravity)
 *       __device__ inverse_dynamics_vaf_device<T>(T *s_vaf, const T *s_q, const T *s_qd, const T *s_qdd, const robotModel<T> *d_robotModel, const T gravity)
 *   
 *       __device__ direct_minv_inner<T>(T *s_Minv, const T *s_q, T *s_XImats, int *s_topology_helpers, T *s_temp)
 *       __device__ direct_minv_device<T>(T *s_Minv, const T *s_q, const robotModel<T> *d_robotModel)
 *       __global__ direct_minv_Kernel<T>(T *d_Minv, const T *d_q, const robotModel<T> *d_robotModel, const int NUM_TIMESTEPS)
 *       __host__   direct_minv<T,USE_COMPRESSED_MEM=false>(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const int num_timesteps, const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams)
 *   
 *       __device__ forward_dynamics_inner<T>(T *s_qdd, const T *s_q, const T *s_qd, const T *s_u, T *s_XImats, int *s_topology_helpers, T *s_temp, const T gravity)
 *       __device__ forward_dynamics_device<T>(T *s_qdd, const T *s_q, const T *s_qd, const T *s_u, const robotModel<T> *d_robotModel, const T gravity)
 *       __global__ forward_dynamics_kernel<T>(T *d_qdd, const T *d_q_qd_u, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS)
 *       __host__   forward_dynamics<T>(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps, const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams)
 *   
 *       __device__ inverse_dynamics_gradient_inner<T>(T *s_dc_du, const T *s_q, const T *s_qd, const T *s_vaf, T *s_XImats, int *s_topology_helpers, T *s_temp, const T gravity)
 *       __device__ inverse_dynamics_gradient_device<T>(T *s_dc_du, const T *s_q, const T *s_qd, const T *robotModel<T> *d_robotModel, const T gravity)
 *       __device__ inverse_dynamics_gradient_device<T>(T *s_dc_du, const T *s_q, const T *s_qd, const T *s_qdd, const robotModel<T> *d_robotModel, const T gravity)
 *       __global__ inverse_dynamics_gradient_kernel<T>(T *d_dc_du, const T *d_q_qd, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS)
 *       __global__ inverse_dynamics_gradient_kernel<T>(T *d_dc_du, const T *d_q_qd, const T *d_qdd, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS)
 *       __host__   inverse_dynamics_gradient<T,USE_QDD_FLAG=false,USE_COMPRESSED_MEM=false>(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps, const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams)
 *   
 *       __device__ forward_dynamics_gradient_device<T>(T *s_df_du, const T *s_q, const T *s_qd, const T *s_u, const robotModel<T> *d_robotModel, const T gravity)
 *       __device__ forward_dynamics_gradient_device<T>(T *s_df_du, const T *s_q, const T *s_qd, const T *s_qdd, const T *s_Minv, const robotModel<T> *d_robotModel, const T gravity)
 *       __global__ forward_dynamics_gradient_kernel<T>(T *d_df_du, const T *d_q_qd_u, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS)
 *       __global__ forward_dynamics_gradient_kernel<T>(T *d_df_du, const T *d_q_qd, const T *d_qdd, const T *d_Minv, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS)
 *       __host__   forward_dynamics_gradient<T,USE_QDD_MINV_FLAG=false>(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps, const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams)
 *   
 *       __device__ end_effector_positions_inner<T>(T *s_eePos, const T *s_q, const T *s_Xhom, int *s_topology_helpers, T *s_temp)
 *       __device__ end_effector_positions_device<T>(T *s_eePos, const T *s_q, const robotModel<T> *d_robotModel)
 *       __global__ end_effector_positions_kernel<T>(T *d_eePos, const T *d_q, const robotModel<T> *d_robotModel, const int NUM_TIMESTEPS)
 *       __host__   end_effector_positions<T,USE_COMPRESSED_MEM=false>(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const int num_timesteps, const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams)
 *   
 *       __device__ end_effector_positions_gradient_inner<T>(T *s_deePos, const T *s_q, const T *s_Xhom, const T *s_dXhom, int *s_topology_helpers, T *s_temp)
 *       __device__ end_effector_positions_gradient_device<T>(T *s_deePos, const T *s_q, const robotModel<T> *d_robotModel)
 *       __global__ end_effector_positions_gradient_kernel<T>(T *d_deePos, const T *d_q, const robotModel<T> *d_robotModel, const int NUM_TIMESTEPS)
 *       __host__   end_effector_positions_gradient<T,USE_COMPRESSED_MEM=false>(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const int num_timesteps, const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams)
 *   
 *   Suggested Type T is float
 *   
 *   Additional helper functions and ALGORITHM_inner functions which take in __shared__ memory temp variables exist -- see function descriptions in the file
 *   
 *   By default device and kernels need to be launched with dynamic shared mem of size <FUNC_CODE>_DYNAMIC_SHARED_MEM_COUNT where <FUNC_CODE> = [ID, MINV, FD, ID_DU, FD_DU]
 *
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cuda_runtime.h>
// single kernel timing helper code
#define time_delta_us_timespec(start,end) (1e6*static_cast<double>(end.tv_sec - start.tv_sec)+1e-3*static_cast<double>(end.tv_nsec - start.tv_nsec))

/**
 * Check for runtime errors using the CUDA API
 *
 * Notes:
 *   Adapted from https://stackoverflow.com/questions/14038589/what-is-the-canonical-way-to-check-for-errors-using-the-cuda-runtime-api
 *
 */
__host__
void gpuAssert(cudaError_t code, const char *file, const int line, bool abort=true){
    if (code != cudaSuccess){
        fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort){cudaDeviceReset(); exit(code);}
    }
}
#define gpuErrchk(err) {gpuAssert(err, __FILE__, __LINE__);}

template <typename T, int M, int N>
__host__ __device__
void printMat(T *A, int lda){
    for(int i=0; i<M; i++){
        for(int j=0; j<N; j++){printf("%.4f ",A[i + lda*j]);}
        printf("\n");
    }
}

template <typename T, int M, int N>
__host__ __device__
void printMat(const T *A, int lda){
    for(int i=0; i<M; i++){
        for(int j=0; j<N; j++){printf("%.4f ",A[i + lda*j]);}
        printf("\n");
    }
}

/**
 * All functions are kept in this namespace
 *
 */
namespace grid {
    const int NUM_JOINTS = 12;
    const int NUM_VEL = 12;
    const int NUM_EES = 4;
    const int ID_DYNAMIC_SHARED_MEM_COUNT = 1320;
    const int MINV_DYNAMIC_SHARED_MEM_COUNT = 2916;
    const int FD_DYNAMIC_SHARED_MEM_COUNT = 3360;
    const int ID_DU_DYNAMIC_SHARED_MEM_COUNT = 3048;
    const int FD_DU_DYNAMIC_SHARED_MEM_COUNT = 3048;
    const int ABA_DYNAMIC_SHARED_MEM_COUNT = 2928;
    const int CRBA_SHARED_MEM_COUNT = 2928;
    const int ID_DU_MAX_SHARED_MEM_COUNT = 3588;
    const int FD_DU_MAX_SHARED_MEM_COUNT = 4032;
    const int EE_POS_DYNAMIC_SHARED_MEM_COUNT = 320;
    const int DEE_POS_DYNAMIC_SHARED_MEM_COUNT = 3456;
    const int SUGGESTED_THREADS = 288;
    // Define custom structs
    template <typename T>
    struct robotModel {
        T *d_XImats;
        int *d_topology_helpers;
    };
    template <typename T>
    struct gridData {
        // GPU INPUTS
        T *d_q_qd_u;
        T *d_q_qd;
        T *d_q;
        // CPU INPUTS
        T *h_q_qd_u;
        T *h_q_qd;
        T *h_q;
        // GPU OUTPUTS
        T *d_c;
        T *d_Minv;
        T *d_qdd;
        T *d_M;
        T *d_dc_du;
        T *d_df_du;
        T *d_eePos;
        T *d_deePos;
        // CPU OUTPUTS
        T *h_c;
        T *h_Minv;
        T *h_qdd;
        T *h_M;
        T *h_dc_du;
        T *h_df_du;
        T *h_eePos;
        T *h_deePos;
    };
    /**
     * Compute the dot product between two vectors
     *
     * Notes:
     *   Assumes computed by a single thread
     *
     * @param vec1 is the first vector of length N with stride S1
     * @param vec2 is the second vector of length N with stride S2
     * @return the resulting final value
     */
    template <typename T, int N, int S1, int S2>
    __device__
    T dot_prod(const T *vec1, const T *vec2) {
        T result = 0;
        for(int i = 0; i < N; i++) {
            result += vec1[i*S1] * vec2[i*S2];
        }
        return result;
    }

    /**
     * Compute the dot product between two vectors
     *
     * Notes:
     *   Assumes computed by a single thread
     *
     * @param vec1 is the first vector of length N with stride S1
     * @param vec2 is the second vector of length N with stride S2
     * @return the resulting final value
     */
    template <typename T, int N, int S1, int S2>
    __device__
    T dot_prod(T *vec1, const T *vec2) {
        T result = 0;
        for(int i = 0; i < N; i++) {
            result += vec1[i*S1] * vec2[i*S2];
        }
        return result;
    }

    /**
     * Compute the dot product between two vectors
     *
     * Notes:
     *   Assumes computed by a single thread
     *
     * @param vec1 is the first vector of length N with stride S1
     * @param vec2 is the second vector of length N with stride S2
     * @return the resulting final value
     */
    template <typename T, int N, int S1, int S2>
    __device__
    T dot_prod(const T *vec1, T *vec2) {
        T result = 0;
        for(int i = 0; i < N; i++) {
            result += vec1[i*S1] * vec2[i*S2];
        }
        return result;
    }

    /**
     * Compute the dot product between two vectors
     *
     * Notes:
     *   Assumes computed by a single thread
     *
     * @param vec1 is the first vector of length N with stride S1
     * @param vec2 is the second vector of length N with stride S2
     * @return the resulting final value
     */
    template <typename T, int N, int S1, int S2>
    __device__
    T dot_prod(T *vec1, T *vec2) {
        T result = 0;
        for(int i = 0; i < N; i++) {
            result += vec1[i*S1] * vec2[i*S2];
        }
        return result;
    }

    /**
     * Generates the motion vector cross product matrix column 0
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     */
    template <typename T>
    __device__
    void mx0(T *s_vecX, const T *s_vec) {
        s_vecX[0] = static_cast<T>(0);
        s_vecX[1] = s_vec[2];
        s_vecX[2] = -s_vec[1];
        s_vecX[3] = static_cast<T>(0);
        s_vecX[4] = s_vec[5];
        s_vecX[5] = -s_vec[4];
    }

    /**
     * Adds the motion vector cross product matrix column 0
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     */
    template <typename T>
    __device__
    void mx0_peq(T *s_vecX, const T *s_vec) {
        s_vecX[1] += s_vec[2];
        s_vecX[2] += -s_vec[1];
        s_vecX[4] += s_vec[5];
        s_vecX[5] += -s_vec[4];
    }

    /**
     * Generates the motion vector cross product matrix column 0
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     * @param alpha is the scaling factor
     */
    template <typename T>
    __device__
    void mx0_scaled(T *s_vecX, const T *s_vec, const T alpha) {
        s_vecX[0] = static_cast<T>(0);
        s_vecX[1] = s_vec[2]*alpha;
        s_vecX[2] = -s_vec[1]*alpha;
        s_vecX[3] = static_cast<T>(0);
        s_vecX[4] = s_vec[5]*alpha;
        s_vecX[5] = -s_vec[4]*alpha;
    }

    /**
     * Adds the motion vector cross product matrix column 0
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     * @param alpha is the scaling factor
     */
    template <typename T>
    __device__
    void mx0_peq_scaled(T *s_vecX, const T *s_vec, const T alpha) {
        s_vecX[1] += s_vec[2]*alpha;
        s_vecX[2] += -s_vec[1]*alpha;
        s_vecX[4] += s_vec[5]*alpha;
        s_vecX[5] += -s_vec[4]*alpha;
    }

    /**
     * Generates the motion vector cross product matrix column 1
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     */
    template <typename T>
    __device__
    void mx1(T *s_vecX, const T *s_vec) {
        s_vecX[0] = -s_vec[2];
        s_vecX[1] = static_cast<T>(0);
        s_vecX[2] = s_vec[0];
        s_vecX[3] = -s_vec[5];
        s_vecX[4] = static_cast<T>(0);
        s_vecX[5] = s_vec[3];
    }

    /**
     * Adds the motion vector cross product matrix column 1
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     */
    template <typename T>
    __device__
    void mx1_peq(T *s_vecX, const T *s_vec) {
        s_vecX[0] += -s_vec[2];
        s_vecX[2] += s_vec[0];
        s_vecX[3] += -s_vec[5];
        s_vecX[5] += s_vec[3];
    }

    /**
     * Generates the motion vector cross product matrix column 1
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     * @param alpha is the scaling factor
     */
    template <typename T>
    __device__
    void mx1_scaled(T *s_vecX, const T *s_vec, const T alpha) {
        s_vecX[0] = -s_vec[2]*alpha;
        s_vecX[1] = static_cast<T>(0);
        s_vecX[2] = s_vec[0]*alpha;
        s_vecX[3] = -s_vec[5]*alpha;
        s_vecX[4] = static_cast<T>(0);
        s_vecX[5] = s_vec[3]*alpha;
    }

    /**
     * Adds the motion vector cross product matrix column 1
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     * @param alpha is the scaling factor
     */
    template <typename T>
    __device__
    void mx1_peq_scaled(T *s_vecX, const T *s_vec, const T alpha) {
        s_vecX[0] += -s_vec[2]*alpha;
        s_vecX[2] += s_vec[0]*alpha;
        s_vecX[3] += -s_vec[5]*alpha;
        s_vecX[5] += s_vec[3]*alpha;
    }

    /**
     * Generates the motion vector cross product matrix column 2
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     */
    template <typename T>
    __device__
    void mx2(T *s_vecX, const T *s_vec) {
        s_vecX[0] = s_vec[1];
        s_vecX[1] = -s_vec[0];
        s_vecX[2] = static_cast<T>(0);
        s_vecX[3] = s_vec[4];
        s_vecX[4] = -s_vec[3];
        s_vecX[5] = static_cast<T>(0);
    }

    /**
     * Adds the motion vector cross product matrix column 2
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     */
    template <typename T>
    __device__
    void mx2_peq(T *s_vecX, const T *s_vec) {
        s_vecX[0] += s_vec[1];
        s_vecX[1] += -s_vec[0];
        s_vecX[3] += s_vec[4];
        s_vecX[4] += -s_vec[3];
    }

    /**
     * Generates the motion vector cross product matrix column 2
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     * @param alpha is the scaling factor
     */
    template <typename T>
    __device__
    void mx2_scaled(T *s_vecX, const T *s_vec, const T alpha) {
        s_vecX[0] = s_vec[1]*alpha;
        s_vecX[1] = -s_vec[0]*alpha;
        s_vecX[2] = static_cast<T>(0);
        s_vecX[3] = s_vec[4]*alpha;
        s_vecX[4] = -s_vec[3]*alpha;
        s_vecX[5] = static_cast<T>(0);
    }

    /**
     * Adds the motion vector cross product matrix column 2
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     * @param alpha is the scaling factor
     */
    template <typename T>
    __device__
    void mx2_peq_scaled(T *s_vecX, const T *s_vec, const T alpha) {
        s_vecX[0] += s_vec[1]*alpha;
        s_vecX[1] += -s_vec[0]*alpha;
        s_vecX[3] += s_vec[4]*alpha;
        s_vecX[4] += -s_vec[3]*alpha;
    }

    /**
     * Generates the motion vector cross product matrix column 3
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     */
    template <typename T>
    __device__
    void mx3(T *s_vecX, const T *s_vec) {
        s_vecX[0] = static_cast<T>(0);
        s_vecX[1] = static_cast<T>(0);
        s_vecX[2] = static_cast<T>(0);
        s_vecX[3] = static_cast<T>(0);
        s_vecX[4] = s_vec[2];
        s_vecX[5] = -s_vec[1];
    }

    /**
     * Adds the motion vector cross product matrix column 3
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     */
    template <typename T>
    __device__
    void mx3_peq(T *s_vecX, const T *s_vec) {
        s_vecX[4] += s_vec[2];
        s_vecX[5] += -s_vec[1];
    }

    /**
     * Generates the motion vector cross product matrix column 3
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     * @param alpha is the scaling factor
     */
    template <typename T>
    __device__
    void mx3_scaled(T *s_vecX, const T *s_vec, const T alpha) {
        s_vecX[0] = static_cast<T>(0);
        s_vecX[1] = static_cast<T>(0);
        s_vecX[2] = static_cast<T>(0);
        s_vecX[3] = static_cast<T>(0);
        s_vecX[4] = s_vec[2]*alpha;
        s_vecX[5] = -s_vec[1]*alpha;
    }

    /**
     * Adds the motion vector cross product matrix column 3
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     * @param alpha is the scaling factor
     */
    template <typename T>
    __device__
    void mx3_peq_scaled(T *s_vecX, const T *s_vec, const T alpha) {
        s_vecX[4] += s_vec[2]*alpha;
        s_vecX[5] += -s_vec[1]*alpha;
    }

    /**
     * Generates the motion vector cross product matrix column 4
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     */
    template <typename T>
    __device__
    void mx4(T *s_vecX, const T *s_vec) {
        s_vecX[0] = static_cast<T>(0);
        s_vecX[1] = static_cast<T>(0);
        s_vecX[2] = static_cast<T>(0);
        s_vecX[3] = -s_vec[2];
        s_vecX[4] = static_cast<T>(0);
        s_vecX[5] = s_vec[0];
    }

    /**
     * Adds the motion vector cross product matrix column 4
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     */
    template <typename T>
    __device__
    void mx4_peq(T *s_vecX, const T *s_vec) {
        s_vecX[3] += -s_vec[2];
        s_vecX[5] += s_vec[0];
    }

    /**
     * Generates the motion vector cross product matrix column 4
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     * @param alpha is the scaling factor
     */
    template <typename T>
    __device__
    void mx4_scaled(T *s_vecX, const T *s_vec, const T alpha) {
        s_vecX[0] = static_cast<T>(0);
        s_vecX[1] = static_cast<T>(0);
        s_vecX[2] = static_cast<T>(0);
        s_vecX[3] = -s_vec[2]*alpha;
        s_vecX[4] = static_cast<T>(0);
        s_vecX[5] = s_vec[0]*alpha;
    }

    /**
     * Adds the motion vector cross product matrix column 4
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     * @param alpha is the scaling factor
     */
    template <typename T>
    __device__
    void mx4_peq_scaled(T *s_vecX, const T *s_vec, const T alpha) {
        s_vecX[3] += -s_vec[2]*alpha;
        s_vecX[5] += s_vec[0]*alpha;
    }

    /**
     * Generates the motion vector cross product matrix column 5
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     */
    template <typename T>
    __device__
    void mx5(T *s_vecX, const T *s_vec) {
        s_vecX[0] = static_cast<T>(0);
        s_vecX[1] = static_cast<T>(0);
        s_vecX[2] = static_cast<T>(0);
        s_vecX[3] = s_vec[1];
        s_vecX[4] = -s_vec[0];
        s_vecX[5] = static_cast<T>(0);
    }

    /**
     * Adds the motion vector cross product matrix column 5
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     */
    template <typename T>
    __device__
    void mx5_peq(T *s_vecX, const T *s_vec) {
        s_vecX[3] += s_vec[1];
        s_vecX[4] += -s_vec[0];
    }

    /**
     * Generates the motion vector cross product matrix column 5
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     * @param alpha is the scaling factor
     */
    template <typename T>
    __device__
    void mx5_scaled(T *s_vecX, const T *s_vec, const T alpha) {
        s_vecX[0] = static_cast<T>(0);
        s_vecX[1] = static_cast<T>(0);
        s_vecX[2] = static_cast<T>(0);
        s_vecX[3] = s_vec[1]*alpha;
        s_vecX[4] = -s_vec[0]*alpha;
        s_vecX[5] = static_cast<T>(0);
    }

    /**
     * Adds the motion vector cross product matrix column 5
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     * @param alpha is the scaling factor
     */
    template <typename T>
    __device__
    void mx5_peq_scaled(T *s_vecX, const T *s_vec, const T alpha) {
        s_vecX[3] += s_vec[1]*alpha;
        s_vecX[4] += -s_vec[0]*alpha;
    }

    /**
     * Generates the motion vector cross product matrix for a runtime selected column
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     */
    template <typename T>
    __device__
    void mxX(T *s_vecX, const T *s_vec, const int S_ind) {
        switch(S_ind){
            case 0: mx0<T>(s_vecX, s_vec); break;
            case 1: mx1<T>(s_vecX, s_vec); break;
            case 2: mx2<T>(s_vecX, s_vec); break;
            case 3: mx3<T>(s_vecX, s_vec); break;
            case 4: mx4<T>(s_vecX, s_vec); break;
            case 5: mx5<T>(s_vecX, s_vec); break;
        }
    }

    /**
     * Generates the motion vector cross product matrix for a runtime selected column
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     */
    template <typename T>
    __device__
    void mxX_peq(T *s_vecX, const T *s_vec, const int S_ind) {
        switch(S_ind){
            case 0: mx0_peq<T>(s_vecX, s_vec); break;
            case 1: mx1_peq<T>(s_vecX, s_vec); break;
            case 2: mx2_peq<T>(s_vecX, s_vec); break;
            case 3: mx3_peq<T>(s_vecX, s_vec); break;
            case 4: mx4_peq<T>(s_vecX, s_vec); break;
            case 5: mx5_peq<T>(s_vecX, s_vec); break;
        }
    }

    /**
     * Generates the motion vector cross product matrix for a runtime selected column
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     * @param alpha is the scaling factor
     */
    template <typename T>
    __device__
    void mxX_scaled(T *s_vecX, const T *s_vec, const T alpha, const int S_ind) {
        switch(S_ind){
            case 0: mx0_scaled<T>(s_vecX, s_vec, alpha); break;
            case 1: mx1_scaled<T>(s_vecX, s_vec, alpha); break;
            case 2: mx2_scaled<T>(s_vecX, s_vec, alpha); break;
            case 3: mx3_scaled<T>(s_vecX, s_vec, alpha); break;
            case 4: mx4_scaled<T>(s_vecX, s_vec, alpha); break;
            case 5: mx5_scaled<T>(s_vecX, s_vec, alpha); break;
        }
    }

    /**
     * Generates the motion vector cross product matrix for a runtime selected column
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_vecX is the destination vector
     * @param s_vec is the source vector
     * @param alpha is the scaling factor
     */
    template <typename T>
    __device__
    void mxX_peq_scaled(T *s_vecX, const T *s_vec, const T alpha, const int S_ind) {
        switch(S_ind){
            case 0: mx0_peq_scaled<T>(s_vecX, s_vec, alpha); break;
            case 1: mx1_peq_scaled<T>(s_vecX, s_vec, alpha); break;
            case 2: mx2_peq_scaled<T>(s_vecX, s_vec, alpha); break;
            case 3: mx3_peq_scaled<T>(s_vecX, s_vec, alpha); break;
            case 4: mx4_peq_scaled<T>(s_vecX, s_vec, alpha); break;
            case 5: mx5_peq_scaled<T>(s_vecX, s_vec, alpha); break;
        }
    }

    /**
     * Generates the motion vector cross product matrix
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_matX is the destination matrix
     * @param s_vecX is the source vector
     */
    template <typename T>
    __device__
    void fx(T *s_matX, const T *s_vecX) {
        s_matX[6*0 + 0] = static_cast<T>(0);
        s_matX[6*0 + 1] = s_vecX[2];
        s_matX[6*0 + 2] = -s_vecX[1];
        s_matX[6*0 + 3] = static_cast<T>(0);
        s_matX[6*0 + 4] = static_cast<T>(0);
        s_matX[6*0 + 5] = static_cast<T>(0);
        s_matX[6*1 + 0] = -s_vecX[2];
        s_matX[6*1 + 1] = static_cast<T>(0);
        s_matX[6*1 + 2] = s_vecX[0];
        s_matX[6*1 + 3] = static_cast<T>(0);
        s_matX[6*1 + 4] = static_cast<T>(0);
        s_matX[6*1 + 5] = static_cast<T>(0);
        s_matX[6*2 + 0] = s_vecX[1];
        s_matX[6*2 + 1] = -s_vecX[0];
        s_matX[6*2 + 2] = static_cast<T>(0);
        s_matX[6*2 + 3] = static_cast<T>(0);
        s_matX[6*2 + 4] = static_cast<T>(0);
        s_matX[6*2 + 5] = static_cast<T>(0);
        s_matX[6*3 + 0] = static_cast<T>(0);
        s_matX[6*3 + 1] = s_vecX[5];
        s_matX[6*3 + 2] = -s_vecX[4];
        s_matX[6*3 + 3] = static_cast<T>(0);
        s_matX[6*3 + 4] = s_vecX[2];
        s_matX[6*3 + 5] = -s_vecX[1];
        s_matX[6*4 + 0] = -s_vecX[5];
        s_matX[6*4 + 1] = static_cast<T>(0);
        s_matX[6*4 + 2] = s_vecX[3];
        s_matX[6*4 + 3] = -s_vecX[2];
        s_matX[6*4 + 4] = static_cast<T>(0);
        s_matX[6*4 + 5] = s_vecX[0];
        s_matX[6*5 + 0] = s_vecX[4];
        s_matX[6*5 + 1] = -s_vecX[3];
        s_matX[6*5 + 2] = static_cast<T>(0);
        s_matX[6*5 + 3] = s_vecX[1];
        s_matX[6*5 + 4] = -s_vecX[0];
        s_matX[6*5 + 5] = static_cast<T>(0);
    }

    /**
     * Generates the motion vector cross product matrix for a pre-zeroed destination
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *   Assumes destination is zeroed
     *
     * @param s_matX is the destination matrix
     * @param s_vecX is the source vector
     */
    template <typename T>
    __device__
    void fx_zeroed(T *s_matX, const T *s_vecX) {
        s_matX[6*0 + 1] = s_vecX[2];
        s_matX[6*0 + 2] = -s_vecX[1];
        s_matX[6*1 + 0] = -s_vecX[2];
        s_matX[6*1 + 2] = s_vecX[0];
        s_matX[6*2 + 0] = s_vecX[1];
        s_matX[6*2 + 1] = -s_vecX[0];
        s_matX[6*3 + 1] = s_vecX[5];
        s_matX[6*3 + 2] = -s_vecX[4];
        s_matX[6*3 + 4] = s_vecX[2];
        s_matX[6*3 + 5] = -s_vecX[1];
        s_matX[6*4 + 0] = -s_vecX[5];
        s_matX[6*4 + 2] = s_vecX[3];
        s_matX[6*4 + 3] = -s_vecX[2];
        s_matX[6*4 + 5] = s_vecX[0];
        s_matX[6*5 + 0] = s_vecX[4];
        s_matX[6*5 + 1] = -s_vecX[3];
        s_matX[6*5 + 3] = s_vecX[1];
        s_matX[6*5 + 4] = -s_vecX[0];
    }

    /**
     * Generates the motion vector cross product matrix and multiples by the input vector
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_result is the result vector
     * @param s_fxVec is the fx vector
     * @param s_timesVec is the multipled vector
     */
    template <typename T>
    __device__
    void fx_times_v(T *s_result, const T *s_fxVec, const T *s_timesVec) {
        s_result[0] = -s_fxVec[2] * s_timesVec[1] + s_fxVec[1] * s_timesVec[2] - s_fxVec[5] * s_timesVec[4] + s_fxVec[4] * s_timesVec[5];
        s_result[1] =  s_fxVec[2] * s_timesVec[0] - s_fxVec[0] * s_timesVec[2] + s_fxVec[5] * s_timesVec[3] - s_fxVec[3] * s_timesVec[5];
        s_result[2] = -s_fxVec[1] * s_timesVec[0] + s_fxVec[0] * s_timesVec[1] - s_fxVec[4] * s_timesVec[3] + s_fxVec[3] * s_timesVec[4];
        s_result[3] =                                                          - s_fxVec[2] * s_timesVec[4] + s_fxVec[1] * s_timesVec[5];
        s_result[4] =                                                            s_fxVec[2] * s_timesVec[3] - s_fxVec[0] * s_timesVec[5];
        s_result[5] =                                                          - s_fxVec[1] * s_timesVec[3] + s_fxVec[0] * s_timesVec[4];
    }

    /**
     * Adds the motion vector cross product matrix multiplied by the input vector
     *
     * Notes:
     *   Assumes only one thread is running each function call
     *
     * @param s_result is the result vector
     * @param s_fxVec is the fx vector
     * @param s_timesVec is the multipled vector
     */
    template <typename T>
    __device__
    void fx_times_v_peq(T *s_result, const T *s_fxVec, const T *s_timesVec) {
        s_result[0] += -s_fxVec[2] * s_timesVec[1] + s_fxVec[1] * s_timesVec[2] - s_fxVec[5] * s_timesVec[4] + s_fxVec[4] * s_timesVec[5];
        s_result[1] +=  s_fxVec[2] * s_timesVec[0] - s_fxVec[0] * s_timesVec[2] + s_fxVec[5] * s_timesVec[3] - s_fxVec[3] * s_timesVec[5];
        s_result[2] += -s_fxVec[1] * s_timesVec[0] + s_fxVec[0] * s_timesVec[1] - s_fxVec[4] * s_timesVec[3] + s_fxVec[3] * s_timesVec[4];
        s_result[3] +=                                                          - s_fxVec[2] * s_timesVec[4] + s_fxVec[1] * s_timesVec[5];
        s_result[4] +=                                                            s_fxVec[2] * s_timesVec[3] - s_fxVec[0] * s_timesVec[5];
        s_result[5] +=                                                          - s_fxVec[1] * s_timesVec[3] + s_fxVec[0] * s_timesVec[4];
    }
    template <typename T>
    __device__
    void vcross(T *dest, T *v){
        dest[0] = static_cast<T>(0);
        dest[1] = v[2];
        dest[2] = -1*v[1];
        dest[3] = static_cast<T>(0);
        dest[4] = v[5];
        dest[5] = -1*v[4];
        dest[6] = -1*v[2];
        dest[7] = static_cast<T>(0);
        dest[8] = v[0];
        dest[9] = -1*v[5];
        dest[10] = static_cast<T>(0);
        dest[11] = v[3];
        dest[12] = v[1];
        dest[13] = -1*v[0];
        dest[14] = static_cast<T>(0);
        dest[15] = v[4];
        dest[16] = -1*v[3];
        dest[17] = static_cast<T>(0);
        dest[18] = static_cast<T>(0);
        dest[19] = static_cast<T>(0);
        dest[20] = static_cast<T>(0);
        dest[21] = static_cast<T>(0);
        dest[22] = v[2];
        dest[23] = -1*v[1];
        dest[24] = static_cast<T>(0);
        dest[25] = static_cast<T>(0);
        dest[26] = static_cast<T>(0);
        dest[27] = -1*v[2];
        dest[28] = static_cast<T>(0);
        dest[29] = v[0];
        dest[30] = static_cast<T>(0);
        dest[31] = static_cast<T>(0);
        dest[32] = static_cast<T>(0);
        dest[33] = v[1];
        dest[34] = -1*v[0];
        dest[35] = static_cast<T>(0);
    }
    /**
     * Compute the inverse of a matrix
     *
     * Notes:
     *   Uses gaussian elimination
     *
     * @param dimA is number of rows in A
     * @param A is a pointer to the original invertible matrix. It is turned into an identity matrix
     * @param Ainv is a pointer to an identity matrix that will be transformed into the inverse of A
     * @param s_temp is a pointer to temporary memory of size 4*dimA
     */
    template <typename T>
    __device__
    void invert_matrix(uint32_t dimA, T *A, T *Ainv, T *s_temp) {
        for (unsigned pivRC = 0; pivRC < dimA; pivRC++) {
            unsigned pivColOffset = pivRC*dimA;
            T pvInv = static_cast<T>(1)/A[pivRC + pivColOffset];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < dimA; ind += blockDim.x*blockDim.y){
                s_temp[ind] = static_cast<T>(A[pivRC * dimA + ind]);
                s_temp[ind+dimA] = static_cast<T>(Ainv[pivRC * dimA + ind]);
                s_temp[ind+dimA*2] = static_cast<T>(A[pivRC + dimA * ind]);
                s_temp[ind+dimA*3] = static_cast<T>(Ainv[pivRC + dimA * ind]);
            }
            __syncthreads();
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < dimA*dimA; ind += blockDim.x*blockDim.y){
                unsigned row = ind % dimA, col = ind / dimA;
                if (row == pivRC) {
                    A[row * dimA + col] *= pvInv;
                    Ainv[row * dimA + col] *= pvInv;
                }
                else {
                    T multiplier = s_temp[row+dimA*2] / s_temp[pivRC];
                    A[row * dimA + col] -= multiplier * s_temp[col];
                    Ainv[row * dimA + col] -= multiplier * s_temp[col+dimA];
                }
            }
            __syncthreads();
        }
    }

    /**
     * Initializes the topology_helpers in GPU memory
     *
     * @return A pointer to the topology_helpers memory in the GPU
     */
    template <typename T>
    __host__
    int *init_topology_helpers() {
        int h_topology_helpers[] = {-1,0,1,-1,3,4,-1,6,7,-1,9,10, // parent_inds
                                    0,1,2,0,1,2,0,1,2,0,1,2, // num_ancestors
                                    3,2,1,3,2,1,3,2,1,3,2,1, // num_subtree
                                    0,0,1,3,3,4,6,6,7,9,9,10,12, // running_sum_num_ancestors
                                    0,3,5,6,9,11,12,15,17,18,21,23}; // running_sum_num_subtree
        int *d_topology_helpers; gpuErrchk(cudaMalloc((void**)&d_topology_helpers,61*sizeof(int)));
        gpuErrchk(cudaMemcpy(d_topology_helpers,h_topology_helpers,61*sizeof(int),cudaMemcpyHostToDevice));
        return d_topology_helpers;
    }

    /**
     * Initializes the Xmats and Imats in GPU memory
     *
     * Notes:
     *   Memory order is X[0...N], I[0...N], Xhom[0...N]
     *
     * @return A pointer to the XI memory in the GPU
     */
    template <typename T>
    __host__
    T* init_XImats() {
        T *h_XImats = (T *)malloc(1248*sizeof(T));
        // X[0]
        h_XImats[0] = static_cast<T>(0);
        h_XImats[1] = static_cast<T>(0);
        h_XImats[2] = static_cast<T>(-1.00000000000000);
        h_XImats[3] = static_cast<T>(0);
        h_XImats[4] = static_cast<T>(0);
        h_XImats[5] = static_cast<T>(0);
        h_XImats[6] = static_cast<T>(0);
        h_XImats[7] = static_cast<T>(0);
        h_XImats[8] = static_cast<T>(0);
        h_XImats[9] = static_cast<T>(0);
        h_XImats[10] = static_cast<T>(0);
        h_XImats[11] = static_cast<T>(0);
        h_XImats[12] = static_cast<T>(0);
        h_XImats[13] = static_cast<T>(0);
        h_XImats[14] = static_cast<T>(0);
        h_XImats[15] = static_cast<T>(0);
        h_XImats[16] = static_cast<T>(0);
        h_XImats[17] = static_cast<T>(0.207000000000000);
        h_XImats[18] = static_cast<T>(0);
        h_XImats[19] = static_cast<T>(0);
        h_XImats[20] = static_cast<T>(0);
        h_XImats[21] = static_cast<T>(0);
        h_XImats[22] = static_cast<T>(0);
        h_XImats[23] = static_cast<T>(-1.00000000000000);
        h_XImats[24] = static_cast<T>(0);
        h_XImats[25] = static_cast<T>(0);
        h_XImats[26] = static_cast<T>(0);
        h_XImats[27] = static_cast<T>(0);
        h_XImats[28] = static_cast<T>(0);
        h_XImats[29] = static_cast<T>(0);
        h_XImats[30] = static_cast<T>(0);
        h_XImats[31] = static_cast<T>(0);
        h_XImats[32] = static_cast<T>(0);
        h_XImats[33] = static_cast<T>(0);
        h_XImats[34] = static_cast<T>(0);
        h_XImats[35] = static_cast<T>(0);
        // X[1]
        h_XImats[36] = static_cast<T>(0);
        h_XImats[37] = static_cast<T>(0);
        h_XImats[38] = static_cast<T>(0);
        h_XImats[39] = static_cast<T>(0);
        h_XImats[40] = static_cast<T>(0);
        h_XImats[41] = static_cast<T>(0);
        h_XImats[42] = static_cast<T>(0);
        h_XImats[43] = static_cast<T>(0);
        h_XImats[44] = static_cast<T>(-1.00000000000000);
        h_XImats[45] = static_cast<T>(0);
        h_XImats[46] = static_cast<T>(0);
        h_XImats[47] = static_cast<T>(0);
        h_XImats[48] = static_cast<T>(0);
        h_XImats[49] = static_cast<T>(0);
        h_XImats[50] = static_cast<T>(0);
        h_XImats[51] = static_cast<T>(0);
        h_XImats[52] = static_cast<T>(0);
        h_XImats[53] = static_cast<T>(-0.0800000000000000);
        h_XImats[54] = static_cast<T>(0);
        h_XImats[55] = static_cast<T>(0);
        h_XImats[56] = static_cast<T>(0);
        h_XImats[57] = static_cast<T>(0);
        h_XImats[58] = static_cast<T>(0);
        h_XImats[59] = static_cast<T>(0);
        h_XImats[60] = static_cast<T>(0);
        h_XImats[61] = static_cast<T>(0);
        h_XImats[62] = static_cast<T>(0);
        h_XImats[63] = static_cast<T>(0);
        h_XImats[64] = static_cast<T>(0);
        h_XImats[65] = static_cast<T>(-1.00000000000000);
        h_XImats[66] = static_cast<T>(0);
        h_XImats[67] = static_cast<T>(0);
        h_XImats[68] = static_cast<T>(0);
        h_XImats[69] = static_cast<T>(0);
        h_XImats[70] = static_cast<T>(0);
        h_XImats[71] = static_cast<T>(0);
        // X[2]
        h_XImats[72] = static_cast<T>(0);
        h_XImats[73] = static_cast<T>(0);
        h_XImats[74] = static_cast<T>(0);
        h_XImats[75] = static_cast<T>(0);
        h_XImats[76] = static_cast<T>(0);
        h_XImats[77] = static_cast<T>(0);
        h_XImats[78] = static_cast<T>(0);
        h_XImats[79] = static_cast<T>(0);
        h_XImats[80] = static_cast<T>(0);
        h_XImats[81] = static_cast<T>(0);
        h_XImats[82] = static_cast<T>(0);
        h_XImats[83] = static_cast<T>(-0.350000000000000);
        h_XImats[84] = static_cast<T>(0);
        h_XImats[85] = static_cast<T>(0);
        h_XImats[86] = static_cast<T>(1.00000000000000);
        h_XImats[87] = static_cast<T>(0);
        h_XImats[88] = static_cast<T>(0);
        h_XImats[89] = static_cast<T>(0);
        h_XImats[90] = static_cast<T>(0);
        h_XImats[91] = static_cast<T>(0);
        h_XImats[92] = static_cast<T>(0);
        h_XImats[93] = static_cast<T>(0);
        h_XImats[94] = static_cast<T>(0);
        h_XImats[95] = static_cast<T>(0);
        h_XImats[96] = static_cast<T>(0);
        h_XImats[97] = static_cast<T>(0);
        h_XImats[98] = static_cast<T>(0);
        h_XImats[99] = static_cast<T>(0);
        h_XImats[100] = static_cast<T>(0);
        h_XImats[101] = static_cast<T>(0);
        h_XImats[102] = static_cast<T>(0);
        h_XImats[103] = static_cast<T>(0);
        h_XImats[104] = static_cast<T>(0);
        h_XImats[105] = static_cast<T>(0);
        h_XImats[106] = static_cast<T>(0);
        h_XImats[107] = static_cast<T>(1.00000000000000);
        // X[3]
        h_XImats[108] = static_cast<T>(0);
        h_XImats[109] = static_cast<T>(0);
        h_XImats[110] = static_cast<T>(-1.00000000000000);
        h_XImats[111] = static_cast<T>(0);
        h_XImats[112] = static_cast<T>(0);
        h_XImats[113] = static_cast<T>(0);
        h_XImats[114] = static_cast<T>(0);
        h_XImats[115] = static_cast<T>(0);
        h_XImats[116] = static_cast<T>(0);
        h_XImats[117] = static_cast<T>(0);
        h_XImats[118] = static_cast<T>(0);
        h_XImats[119] = static_cast<T>(0);
        h_XImats[120] = static_cast<T>(0);
        h_XImats[121] = static_cast<T>(0);
        h_XImats[122] = static_cast<T>(0);
        h_XImats[123] = static_cast<T>(0);
        h_XImats[124] = static_cast<T>(0);
        h_XImats[125] = static_cast<T>(0.207000000000000);
        h_XImats[126] = static_cast<T>(0);
        h_XImats[127] = static_cast<T>(0);
        h_XImats[128] = static_cast<T>(0);
        h_XImats[129] = static_cast<T>(0);
        h_XImats[130] = static_cast<T>(0);
        h_XImats[131] = static_cast<T>(-1.00000000000000);
        h_XImats[132] = static_cast<T>(0);
        h_XImats[133] = static_cast<T>(0);
        h_XImats[134] = static_cast<T>(0);
        h_XImats[135] = static_cast<T>(0);
        h_XImats[136] = static_cast<T>(0);
        h_XImats[137] = static_cast<T>(0);
        h_XImats[138] = static_cast<T>(0);
        h_XImats[139] = static_cast<T>(0);
        h_XImats[140] = static_cast<T>(0);
        h_XImats[141] = static_cast<T>(0);
        h_XImats[142] = static_cast<T>(0);
        h_XImats[143] = static_cast<T>(0);
        // X[4]
        h_XImats[144] = static_cast<T>(0);
        h_XImats[145] = static_cast<T>(0);
        h_XImats[146] = static_cast<T>(0);
        h_XImats[147] = static_cast<T>(0);
        h_XImats[148] = static_cast<T>(0);
        h_XImats[149] = static_cast<T>(0);
        h_XImats[150] = static_cast<T>(0);
        h_XImats[151] = static_cast<T>(0);
        h_XImats[152] = static_cast<T>(-1.00000000000000);
        h_XImats[153] = static_cast<T>(0);
        h_XImats[154] = static_cast<T>(0);
        h_XImats[155] = static_cast<T>(0);
        h_XImats[156] = static_cast<T>(0);
        h_XImats[157] = static_cast<T>(0);
        h_XImats[158] = static_cast<T>(0);
        h_XImats[159] = static_cast<T>(0);
        h_XImats[160] = static_cast<T>(0);
        h_XImats[161] = static_cast<T>(-0.0800000000000000);
        h_XImats[162] = static_cast<T>(0);
        h_XImats[163] = static_cast<T>(0);
        h_XImats[164] = static_cast<T>(0);
        h_XImats[165] = static_cast<T>(0);
        h_XImats[166] = static_cast<T>(0);
        h_XImats[167] = static_cast<T>(0);
        h_XImats[168] = static_cast<T>(0);
        h_XImats[169] = static_cast<T>(0);
        h_XImats[170] = static_cast<T>(0);
        h_XImats[171] = static_cast<T>(0);
        h_XImats[172] = static_cast<T>(0);
        h_XImats[173] = static_cast<T>(-1.00000000000000);
        h_XImats[174] = static_cast<T>(0);
        h_XImats[175] = static_cast<T>(0);
        h_XImats[176] = static_cast<T>(0);
        h_XImats[177] = static_cast<T>(0);
        h_XImats[178] = static_cast<T>(0);
        h_XImats[179] = static_cast<T>(0);
        // X[5]
        h_XImats[180] = static_cast<T>(0);
        h_XImats[181] = static_cast<T>(0);
        h_XImats[182] = static_cast<T>(0);
        h_XImats[183] = static_cast<T>(0);
        h_XImats[184] = static_cast<T>(0);
        h_XImats[185] = static_cast<T>(0);
        h_XImats[186] = static_cast<T>(0);
        h_XImats[187] = static_cast<T>(0);
        h_XImats[188] = static_cast<T>(0);
        h_XImats[189] = static_cast<T>(0);
        h_XImats[190] = static_cast<T>(0);
        h_XImats[191] = static_cast<T>(-0.350000000000000);
        h_XImats[192] = static_cast<T>(0);
        h_XImats[193] = static_cast<T>(0);
        h_XImats[194] = static_cast<T>(1.00000000000000);
        h_XImats[195] = static_cast<T>(0);
        h_XImats[196] = static_cast<T>(0);
        h_XImats[197] = static_cast<T>(0);
        h_XImats[198] = static_cast<T>(0);
        h_XImats[199] = static_cast<T>(0);
        h_XImats[200] = static_cast<T>(0);
        h_XImats[201] = static_cast<T>(0);
        h_XImats[202] = static_cast<T>(0);
        h_XImats[203] = static_cast<T>(0);
        h_XImats[204] = static_cast<T>(0);
        h_XImats[205] = static_cast<T>(0);
        h_XImats[206] = static_cast<T>(0);
        h_XImats[207] = static_cast<T>(0);
        h_XImats[208] = static_cast<T>(0);
        h_XImats[209] = static_cast<T>(0);
        h_XImats[210] = static_cast<T>(0);
        h_XImats[211] = static_cast<T>(0);
        h_XImats[212] = static_cast<T>(0);
        h_XImats[213] = static_cast<T>(0);
        h_XImats[214] = static_cast<T>(0);
        h_XImats[215] = static_cast<T>(1.00000000000000);
        // X[6]
        h_XImats[216] = static_cast<T>(0);
        h_XImats[217] = static_cast<T>(0);
        h_XImats[218] = static_cast<T>(1.00000000000000);
        h_XImats[219] = static_cast<T>(0);
        h_XImats[220] = static_cast<T>(0);
        h_XImats[221] = static_cast<T>(0);
        h_XImats[222] = static_cast<T>(0);
        h_XImats[223] = static_cast<T>(0);
        h_XImats[224] = static_cast<T>(0);
        h_XImats[225] = static_cast<T>(0);
        h_XImats[226] = static_cast<T>(0);
        h_XImats[227] = static_cast<T>(0);
        h_XImats[228] = static_cast<T>(0);
        h_XImats[229] = static_cast<T>(0);
        h_XImats[230] = static_cast<T>(0);
        h_XImats[231] = static_cast<T>(0);
        h_XImats[232] = static_cast<T>(0);
        h_XImats[233] = static_cast<T>(0.207000000000000);
        h_XImats[234] = static_cast<T>(0);
        h_XImats[235] = static_cast<T>(0);
        h_XImats[236] = static_cast<T>(0);
        h_XImats[237] = static_cast<T>(0);
        h_XImats[238] = static_cast<T>(0);
        h_XImats[239] = static_cast<T>(1.00000000000000);
        h_XImats[240] = static_cast<T>(0);
        h_XImats[241] = static_cast<T>(0);
        h_XImats[242] = static_cast<T>(0);
        h_XImats[243] = static_cast<T>(0);
        h_XImats[244] = static_cast<T>(0);
        h_XImats[245] = static_cast<T>(0);
        h_XImats[246] = static_cast<T>(0);
        h_XImats[247] = static_cast<T>(0);
        h_XImats[248] = static_cast<T>(0);
        h_XImats[249] = static_cast<T>(0);
        h_XImats[250] = static_cast<T>(0);
        h_XImats[251] = static_cast<T>(0);
        // X[7]
        h_XImats[252] = static_cast<T>(0);
        h_XImats[253] = static_cast<T>(0);
        h_XImats[254] = static_cast<T>(0);
        h_XImats[255] = static_cast<T>(0);
        h_XImats[256] = static_cast<T>(0);
        h_XImats[257] = static_cast<T>(0);
        h_XImats[258] = static_cast<T>(0);
        h_XImats[259] = static_cast<T>(0);
        h_XImats[260] = static_cast<T>(1.00000000000000);
        h_XImats[261] = static_cast<T>(0);
        h_XImats[262] = static_cast<T>(0);
        h_XImats[263] = static_cast<T>(0);
        h_XImats[264] = static_cast<T>(0);
        h_XImats[265] = static_cast<T>(0);
        h_XImats[266] = static_cast<T>(0);
        h_XImats[267] = static_cast<T>(0);
        h_XImats[268] = static_cast<T>(0);
        h_XImats[269] = static_cast<T>(0.0800000000000000);
        h_XImats[270] = static_cast<T>(0);
        h_XImats[271] = static_cast<T>(0);
        h_XImats[272] = static_cast<T>(0);
        h_XImats[273] = static_cast<T>(0);
        h_XImats[274] = static_cast<T>(0);
        h_XImats[275] = static_cast<T>(0);
        h_XImats[276] = static_cast<T>(0);
        h_XImats[277] = static_cast<T>(0);
        h_XImats[278] = static_cast<T>(0);
        h_XImats[279] = static_cast<T>(0);
        h_XImats[280] = static_cast<T>(0);
        h_XImats[281] = static_cast<T>(1.00000000000000);
        h_XImats[282] = static_cast<T>(0);
        h_XImats[283] = static_cast<T>(0);
        h_XImats[284] = static_cast<T>(0);
        h_XImats[285] = static_cast<T>(0);
        h_XImats[286] = static_cast<T>(0);
        h_XImats[287] = static_cast<T>(0);
        // X[8]
        h_XImats[288] = static_cast<T>(0);
        h_XImats[289] = static_cast<T>(0);
        h_XImats[290] = static_cast<T>(0);
        h_XImats[291] = static_cast<T>(0);
        h_XImats[292] = static_cast<T>(0);
        h_XImats[293] = static_cast<T>(0);
        h_XImats[294] = static_cast<T>(0);
        h_XImats[295] = static_cast<T>(0);
        h_XImats[296] = static_cast<T>(0);
        h_XImats[297] = static_cast<T>(0);
        h_XImats[298] = static_cast<T>(0);
        h_XImats[299] = static_cast<T>(-0.350000000000000);
        h_XImats[300] = static_cast<T>(0);
        h_XImats[301] = static_cast<T>(0);
        h_XImats[302] = static_cast<T>(1.00000000000000);
        h_XImats[303] = static_cast<T>(0);
        h_XImats[304] = static_cast<T>(0);
        h_XImats[305] = static_cast<T>(0);
        h_XImats[306] = static_cast<T>(0);
        h_XImats[307] = static_cast<T>(0);
        h_XImats[308] = static_cast<T>(0);
        h_XImats[309] = static_cast<T>(0);
        h_XImats[310] = static_cast<T>(0);
        h_XImats[311] = static_cast<T>(0);
        h_XImats[312] = static_cast<T>(0);
        h_XImats[313] = static_cast<T>(0);
        h_XImats[314] = static_cast<T>(0);
        h_XImats[315] = static_cast<T>(0);
        h_XImats[316] = static_cast<T>(0);
        h_XImats[317] = static_cast<T>(0);
        h_XImats[318] = static_cast<T>(0);
        h_XImats[319] = static_cast<T>(0);
        h_XImats[320] = static_cast<T>(0);
        h_XImats[321] = static_cast<T>(0);
        h_XImats[322] = static_cast<T>(0);
        h_XImats[323] = static_cast<T>(1.00000000000000);
        // X[9]
        h_XImats[324] = static_cast<T>(0);
        h_XImats[325] = static_cast<T>(0);
        h_XImats[326] = static_cast<T>(1.00000000000000);
        h_XImats[327] = static_cast<T>(0);
        h_XImats[328] = static_cast<T>(0);
        h_XImats[329] = static_cast<T>(0);
        h_XImats[330] = static_cast<T>(0);
        h_XImats[331] = static_cast<T>(0);
        h_XImats[332] = static_cast<T>(0);
        h_XImats[333] = static_cast<T>(0);
        h_XImats[334] = static_cast<T>(0);
        h_XImats[335] = static_cast<T>(0);
        h_XImats[336] = static_cast<T>(0);
        h_XImats[337] = static_cast<T>(0);
        h_XImats[338] = static_cast<T>(0);
        h_XImats[339] = static_cast<T>(0);
        h_XImats[340] = static_cast<T>(0);
        h_XImats[341] = static_cast<T>(0.207000000000000);
        h_XImats[342] = static_cast<T>(0);
        h_XImats[343] = static_cast<T>(0);
        h_XImats[344] = static_cast<T>(0);
        h_XImats[345] = static_cast<T>(0);
        h_XImats[346] = static_cast<T>(0);
        h_XImats[347] = static_cast<T>(1.00000000000000);
        h_XImats[348] = static_cast<T>(0);
        h_XImats[349] = static_cast<T>(0);
        h_XImats[350] = static_cast<T>(0);
        h_XImats[351] = static_cast<T>(0);
        h_XImats[352] = static_cast<T>(0);
        h_XImats[353] = static_cast<T>(0);
        h_XImats[354] = static_cast<T>(0);
        h_XImats[355] = static_cast<T>(0);
        h_XImats[356] = static_cast<T>(0);
        h_XImats[357] = static_cast<T>(0);
        h_XImats[358] = static_cast<T>(0);
        h_XImats[359] = static_cast<T>(0);
        // X[10]
        h_XImats[360] = static_cast<T>(0);
        h_XImats[361] = static_cast<T>(0);
        h_XImats[362] = static_cast<T>(0);
        h_XImats[363] = static_cast<T>(0);
        h_XImats[364] = static_cast<T>(0);
        h_XImats[365] = static_cast<T>(0);
        h_XImats[366] = static_cast<T>(0);
        h_XImats[367] = static_cast<T>(0);
        h_XImats[368] = static_cast<T>(1.00000000000000);
        h_XImats[369] = static_cast<T>(0);
        h_XImats[370] = static_cast<T>(0);
        h_XImats[371] = static_cast<T>(0);
        h_XImats[372] = static_cast<T>(0);
        h_XImats[373] = static_cast<T>(0);
        h_XImats[374] = static_cast<T>(0);
        h_XImats[375] = static_cast<T>(0);
        h_XImats[376] = static_cast<T>(0);
        h_XImats[377] = static_cast<T>(0.0800000000000000);
        h_XImats[378] = static_cast<T>(0);
        h_XImats[379] = static_cast<T>(0);
        h_XImats[380] = static_cast<T>(0);
        h_XImats[381] = static_cast<T>(0);
        h_XImats[382] = static_cast<T>(0);
        h_XImats[383] = static_cast<T>(0);
        h_XImats[384] = static_cast<T>(0);
        h_XImats[385] = static_cast<T>(0);
        h_XImats[386] = static_cast<T>(0);
        h_XImats[387] = static_cast<T>(0);
        h_XImats[388] = static_cast<T>(0);
        h_XImats[389] = static_cast<T>(1.00000000000000);
        h_XImats[390] = static_cast<T>(0);
        h_XImats[391] = static_cast<T>(0);
        h_XImats[392] = static_cast<T>(0);
        h_XImats[393] = static_cast<T>(0);
        h_XImats[394] = static_cast<T>(0);
        h_XImats[395] = static_cast<T>(0);
        // X[11]
        h_XImats[396] = static_cast<T>(0);
        h_XImats[397] = static_cast<T>(0);
        h_XImats[398] = static_cast<T>(0);
        h_XImats[399] = static_cast<T>(0);
        h_XImats[400] = static_cast<T>(0);
        h_XImats[401] = static_cast<T>(0);
        h_XImats[402] = static_cast<T>(0);
        h_XImats[403] = static_cast<T>(0);
        h_XImats[404] = static_cast<T>(0);
        h_XImats[405] = static_cast<T>(0);
        h_XImats[406] = static_cast<T>(0);
        h_XImats[407] = static_cast<T>(-0.350000000000000);
        h_XImats[408] = static_cast<T>(0);
        h_XImats[409] = static_cast<T>(0);
        h_XImats[410] = static_cast<T>(1.00000000000000);
        h_XImats[411] = static_cast<T>(0);
        h_XImats[412] = static_cast<T>(0);
        h_XImats[413] = static_cast<T>(0);
        h_XImats[414] = static_cast<T>(0);
        h_XImats[415] = static_cast<T>(0);
        h_XImats[416] = static_cast<T>(0);
        h_XImats[417] = static_cast<T>(0);
        h_XImats[418] = static_cast<T>(0);
        h_XImats[419] = static_cast<T>(0);
        h_XImats[420] = static_cast<T>(0);
        h_XImats[421] = static_cast<T>(0);
        h_XImats[422] = static_cast<T>(0);
        h_XImats[423] = static_cast<T>(0);
        h_XImats[424] = static_cast<T>(0);
        h_XImats[425] = static_cast<T>(0);
        h_XImats[426] = static_cast<T>(0);
        h_XImats[427] = static_cast<T>(0);
        h_XImats[428] = static_cast<T>(0);
        h_XImats[429] = static_cast<T>(0);
        h_XImats[430] = static_cast<T>(0);
        h_XImats[431] = static_cast<T>(1.00000000000000);
        // I[0]
        h_XImats[432] = static_cast<T>(0.134701016973);
        h_XImats[433] = static_cast<T>(-4e-05);
        h_XImats[434] = static_cast<T>(-0.022737817929000002);
        h_XImats[435] = static_cast<T>(0.0);
        h_XImats[436] = static_cast<T>(-0.49607829999999997);
        h_XImats[437] = static_cast<T>(0.0);
        h_XImats[438] = static_cast<T>(-4e-05);
        h_XImats[439] = static_cast<T>(0.14417575549);
        h_XImats[440] = static_cast<T>(-5e-05);
        h_XImats[441] = static_cast<T>(0.49607829999999997);
        h_XImats[442] = static_cast<T>(0.0);
        h_XImats[443] = static_cast<T>(-0.12490590000000001);
        h_XImats[444] = static_cast<T>(-0.022737817929);
        h_XImats[445] = static_cast<T>(-5e-05);
        h_XImats[446] = static_cast<T>(0.011034738517);
        h_XImats[447] = static_cast<T>(0.0);
        h_XImats[448] = static_cast<T>(0.12490590000000001);
        h_XImats[449] = static_cast<T>(0.0);
        h_XImats[450] = static_cast<T>(0.0);
        h_XImats[451] = static_cast<T>(0.49607829999999997);
        h_XImats[452] = static_cast<T>(0.0);
        h_XImats[453] = static_cast<T>(2.93);
        h_XImats[454] = static_cast<T>(0.0);
        h_XImats[455] = static_cast<T>(0.0);
        h_XImats[456] = static_cast<T>(-0.49607829999999997);
        h_XImats[457] = static_cast<T>(0.0);
        h_XImats[458] = static_cast<T>(0.12490590000000001);
        h_XImats[459] = static_cast<T>(0.0);
        h_XImats[460] = static_cast<T>(2.93);
        h_XImats[461] = static_cast<T>(0.0);
        h_XImats[462] = static_cast<T>(0.0);
        h_XImats[463] = static_cast<T>(-0.12490590000000001);
        h_XImats[464] = static_cast<T>(0.0);
        h_XImats[465] = static_cast<T>(0.0);
        h_XImats[466] = static_cast<T>(0.0);
        h_XImats[467] = static_cast<T>(2.93);
        // I[1]
        h_XImats[468] = static_cast<T>(0.005497746874999999);
        h_XImats[469] = static_cast<T>(0.00741836815);
        h_XImats[470] = static_cast<T>(0.0001);
        h_XImats[471] = static_cast<T>(0.0);
        h_XImats[472] = static_cast<T>(0.0);
        h_XImats[473] = static_cast<T>(-0.06924749999999999);
        h_XImats[474] = static_cast<T>(0.007418368149999998);
        h_XImats[475] = static_cast<T>(0.08713208056880001);
        h_XImats[476] = static_cast<T>(2e-05);
        h_XImats[477] = static_cast<T>(0.0);
        h_XImats[478] = static_cast<T>(0.0);
        h_XImats[479] = static_cast<T>(-0.39765212);
        h_XImats[480] = static_cast<T>(0.0001);
        h_XImats[481] = static_cast<T>(2e-05);
        h_XImats[482] = static_cast<T>(0.0898698274438);
        h_XImats[483] = static_cast<T>(0.06924749999999999);
        h_XImats[484] = static_cast<T>(0.39765212);
        h_XImats[485] = static_cast<T>(0.0);
        h_XImats[486] = static_cast<T>(0.0);
        h_XImats[487] = static_cast<T>(0.0);
        h_XImats[488] = static_cast<T>(0.06924749999999999);
        h_XImats[489] = static_cast<T>(2.638);
        h_XImats[490] = static_cast<T>(0.0);
        h_XImats[491] = static_cast<T>(0.0);
        h_XImats[492] = static_cast<T>(0.0);
        h_XImats[493] = static_cast<T>(0.0);
        h_XImats[494] = static_cast<T>(0.39765212);
        h_XImats[495] = static_cast<T>(0.0);
        h_XImats[496] = static_cast<T>(2.638);
        h_XImats[497] = static_cast<T>(0.0);
        h_XImats[498] = static_cast<T>(-0.06924749999999999);
        h_XImats[499] = static_cast<T>(-0.39765212);
        h_XImats[500] = static_cast<T>(0.0);
        h_XImats[501] = static_cast<T>(0.0);
        h_XImats[502] = static_cast<T>(0.0);
        h_XImats[503] = static_cast<T>(2.638);
        // I[2]
        h_XImats[504] = static_cast<T>(0.0004700102196);
        h_XImats[505] = static_cast<T>(5.5580904e-05);
        h_XImats[506] = static_cast<T>(1.0477400000000013e-06);
        h_XImats[507] = static_cast<T>(0.0);
        h_XImats[508] = static_cast<T>(8.81e-05);
        h_XImats[509] = static_cast<T>(3.5239999999999995e-05);
        h_XImats[510] = static_cast<T>(5.5580904e-05);
        h_XImats[511] = static_cast<T>(0.026413874770000005);
        h_XImats[512] = static_cast<T>(3.524e-09);
        h_XImats[513] = static_cast<T>(-8.81e-05);
        h_XImats[514] = static_cast<T>(0.0);
        h_XImats[515] = static_cast<T>(-0.11047740000000002);
        h_XImats[516] = static_cast<T>(1.0477399999999996e-06);
        h_XImats[517] = static_cast<T>(3.524e-09);
        h_XImats[518] = static_cast<T>(0.026183867369600003);
        h_XImats[519] = static_cast<T>(-3.5239999999999995e-05);
        h_XImats[520] = static_cast<T>(0.11047740000000002);
        h_XImats[521] = static_cast<T>(0.0);
        h_XImats[522] = static_cast<T>(0.0);
        h_XImats[523] = static_cast<T>(-8.81e-05);
        h_XImats[524] = static_cast<T>(-3.5239999999999995e-05);
        h_XImats[525] = static_cast<T>(0.881);
        h_XImats[526] = static_cast<T>(0.0);
        h_XImats[527] = static_cast<T>(0.0);
        h_XImats[528] = static_cast<T>(8.81e-05);
        h_XImats[529] = static_cast<T>(0.0);
        h_XImats[530] = static_cast<T>(0.11047740000000002);
        h_XImats[531] = static_cast<T>(0.0);
        h_XImats[532] = static_cast<T>(0.881);
        h_XImats[533] = static_cast<T>(0.0);
        h_XImats[534] = static_cast<T>(3.5239999999999995e-05);
        h_XImats[535] = static_cast<T>(-0.11047740000000002);
        h_XImats[536] = static_cast<T>(0.0);
        h_XImats[537] = static_cast<T>(0.0);
        h_XImats[538] = static_cast<T>(0.0);
        h_XImats[539] = static_cast<T>(0.881);
        // I[3]
        h_XImats[540] = static_cast<T>(0.134701016973);
        h_XImats[541] = static_cast<T>(4e-05);
        h_XImats[542] = static_cast<T>(0.022737817929000002);
        h_XImats[543] = static_cast<T>(0.0);
        h_XImats[544] = static_cast<T>(0.49607829999999997);
        h_XImats[545] = static_cast<T>(0.0);
        h_XImats[546] = static_cast<T>(4e-05);
        h_XImats[547] = static_cast<T>(0.14417575549);
        h_XImats[548] = static_cast<T>(-5e-05);
        h_XImats[549] = static_cast<T>(-0.49607829999999997);
        h_XImats[550] = static_cast<T>(0.0);
        h_XImats[551] = static_cast<T>(-0.12490590000000001);
        h_XImats[552] = static_cast<T>(0.022737817929);
        h_XImats[553] = static_cast<T>(-5e-05);
        h_XImats[554] = static_cast<T>(0.011034738517);
        h_XImats[555] = static_cast<T>(0.0);
        h_XImats[556] = static_cast<T>(0.12490590000000001);
        h_XImats[557] = static_cast<T>(0.0);
        h_XImats[558] = static_cast<T>(0.0);
        h_XImats[559] = static_cast<T>(-0.49607829999999997);
        h_XImats[560] = static_cast<T>(0.0);
        h_XImats[561] = static_cast<T>(2.93);
        h_XImats[562] = static_cast<T>(0.0);
        h_XImats[563] = static_cast<T>(0.0);
        h_XImats[564] = static_cast<T>(0.49607829999999997);
        h_XImats[565] = static_cast<T>(0.0);
        h_XImats[566] = static_cast<T>(0.12490590000000001);
        h_XImats[567] = static_cast<T>(0.0);
        h_XImats[568] = static_cast<T>(2.93);
        h_XImats[569] = static_cast<T>(0.0);
        h_XImats[570] = static_cast<T>(0.0);
        h_XImats[571] = static_cast<T>(-0.12490590000000001);
        h_XImats[572] = static_cast<T>(0.0);
        h_XImats[573] = static_cast<T>(0.0);
        h_XImats[574] = static_cast<T>(0.0);
        h_XImats[575] = static_cast<T>(2.93);
        // I[4]
        h_XImats[576] = static_cast<T>(0.005497746874999999);
        h_XImats[577] = static_cast<T>(-0.00741836815);
        h_XImats[578] = static_cast<T>(-0.0001);
        h_XImats[579] = static_cast<T>(0.0);
        h_XImats[580] = static_cast<T>(0.0);
        h_XImats[581] = static_cast<T>(0.06924749999999999);
        h_XImats[582] = static_cast<T>(-0.007418368149999998);
        h_XImats[583] = static_cast<T>(0.08713208056880001);
        h_XImats[584] = static_cast<T>(2e-05);
        h_XImats[585] = static_cast<T>(0.0);
        h_XImats[586] = static_cast<T>(0.0);
        h_XImats[587] = static_cast<T>(-0.39765212);
        h_XImats[588] = static_cast<T>(-0.0001);
        h_XImats[589] = static_cast<T>(2e-05);
        h_XImats[590] = static_cast<T>(0.0898698274438);
        h_XImats[591] = static_cast<T>(-0.06924749999999999);
        h_XImats[592] = static_cast<T>(0.39765212);
        h_XImats[593] = static_cast<T>(0.0);
        h_XImats[594] = static_cast<T>(0.0);
        h_XImats[595] = static_cast<T>(0.0);
        h_XImats[596] = static_cast<T>(-0.06924749999999999);
        h_XImats[597] = static_cast<T>(2.638);
        h_XImats[598] = static_cast<T>(0.0);
        h_XImats[599] = static_cast<T>(0.0);
        h_XImats[600] = static_cast<T>(0.0);
        h_XImats[601] = static_cast<T>(0.0);
        h_XImats[602] = static_cast<T>(0.39765212);
        h_XImats[603] = static_cast<T>(0.0);
        h_XImats[604] = static_cast<T>(2.638);
        h_XImats[605] = static_cast<T>(0.0);
        h_XImats[606] = static_cast<T>(0.06924749999999999);
        h_XImats[607] = static_cast<T>(-0.39765212);
        h_XImats[608] = static_cast<T>(0.0);
        h_XImats[609] = static_cast<T>(0.0);
        h_XImats[610] = static_cast<T>(0.0);
        h_XImats[611] = static_cast<T>(2.638);
        // I[5]
        h_XImats[612] = static_cast<T>(0.0004700102196);
        h_XImats[613] = static_cast<T>(-5.5580904e-05);
        h_XImats[614] = static_cast<T>(-1.0477400000000013e-06);
        h_XImats[615] = static_cast<T>(0.0);
        h_XImats[616] = static_cast<T>(-8.81e-05);
        h_XImats[617] = static_cast<T>(-3.5239999999999995e-05);
        h_XImats[618] = static_cast<T>(-5.5580904e-05);
        h_XImats[619] = static_cast<T>(0.026413874770000005);
        h_XImats[620] = static_cast<T>(3.524e-09);
        h_XImats[621] = static_cast<T>(8.81e-05);
        h_XImats[622] = static_cast<T>(0.0);
        h_XImats[623] = static_cast<T>(-0.11047740000000002);
        h_XImats[624] = static_cast<T>(-1.0477399999999996e-06);
        h_XImats[625] = static_cast<T>(3.524e-09);
        h_XImats[626] = static_cast<T>(0.026183867369600003);
        h_XImats[627] = static_cast<T>(3.5239999999999995e-05);
        h_XImats[628] = static_cast<T>(0.11047740000000002);
        h_XImats[629] = static_cast<T>(0.0);
        h_XImats[630] = static_cast<T>(0.0);
        h_XImats[631] = static_cast<T>(8.81e-05);
        h_XImats[632] = static_cast<T>(3.5239999999999995e-05);
        h_XImats[633] = static_cast<T>(0.881);
        h_XImats[634] = static_cast<T>(0.0);
        h_XImats[635] = static_cast<T>(0.0);
        h_XImats[636] = static_cast<T>(-8.81e-05);
        h_XImats[637] = static_cast<T>(0.0);
        h_XImats[638] = static_cast<T>(0.11047740000000002);
        h_XImats[639] = static_cast<T>(0.0);
        h_XImats[640] = static_cast<T>(0.881);
        h_XImats[641] = static_cast<T>(0.0);
        h_XImats[642] = static_cast<T>(-3.5239999999999995e-05);
        h_XImats[643] = static_cast<T>(-0.11047740000000002);
        h_XImats[644] = static_cast<T>(0.0);
        h_XImats[645] = static_cast<T>(0.0);
        h_XImats[646] = static_cast<T>(0.0);
        h_XImats[647] = static_cast<T>(0.881);
        // I[6]
        h_XImats[648] = static_cast<T>(0.134701016973);
        h_XImats[649] = static_cast<T>(4e-05);
        h_XImats[650] = static_cast<T>(0.022737817929000002);
        h_XImats[651] = static_cast<T>(0.0);
        h_XImats[652] = static_cast<T>(0.49607829999999997);
        h_XImats[653] = static_cast<T>(0.0);
        h_XImats[654] = static_cast<T>(4e-05);
        h_XImats[655] = static_cast<T>(0.14417575549);
        h_XImats[656] = static_cast<T>(-5e-05);
        h_XImats[657] = static_cast<T>(-0.49607829999999997);
        h_XImats[658] = static_cast<T>(0.0);
        h_XImats[659] = static_cast<T>(-0.12490590000000001);
        h_XImats[660] = static_cast<T>(0.022737817929);
        h_XImats[661] = static_cast<T>(-5e-05);
        h_XImats[662] = static_cast<T>(0.011034738517);
        h_XImats[663] = static_cast<T>(0.0);
        h_XImats[664] = static_cast<T>(0.12490590000000001);
        h_XImats[665] = static_cast<T>(0.0);
        h_XImats[666] = static_cast<T>(0.0);
        h_XImats[667] = static_cast<T>(-0.49607829999999997);
        h_XImats[668] = static_cast<T>(0.0);
        h_XImats[669] = static_cast<T>(2.93);
        h_XImats[670] = static_cast<T>(0.0);
        h_XImats[671] = static_cast<T>(0.0);
        h_XImats[672] = static_cast<T>(0.49607829999999997);
        h_XImats[673] = static_cast<T>(0.0);
        h_XImats[674] = static_cast<T>(0.12490590000000001);
        h_XImats[675] = static_cast<T>(0.0);
        h_XImats[676] = static_cast<T>(2.93);
        h_XImats[677] = static_cast<T>(0.0);
        h_XImats[678] = static_cast<T>(0.0);
        h_XImats[679] = static_cast<T>(-0.12490590000000001);
        h_XImats[680] = static_cast<T>(0.0);
        h_XImats[681] = static_cast<T>(0.0);
        h_XImats[682] = static_cast<T>(0.0);
        h_XImats[683] = static_cast<T>(2.93);
        // I[7]
        h_XImats[684] = static_cast<T>(0.005497746874999999);
        h_XImats[685] = static_cast<T>(0.00741836815);
        h_XImats[686] = static_cast<T>(0.0001);
        h_XImats[687] = static_cast<T>(0.0);
        h_XImats[688] = static_cast<T>(0.0);
        h_XImats[689] = static_cast<T>(-0.06924749999999999);
        h_XImats[690] = static_cast<T>(0.007418368149999998);
        h_XImats[691] = static_cast<T>(0.08713208056880001);
        h_XImats[692] = static_cast<T>(2e-05);
        h_XImats[693] = static_cast<T>(0.0);
        h_XImats[694] = static_cast<T>(0.0);
        h_XImats[695] = static_cast<T>(-0.39765212);
        h_XImats[696] = static_cast<T>(0.0001);
        h_XImats[697] = static_cast<T>(2e-05);
        h_XImats[698] = static_cast<T>(0.0898698274438);
        h_XImats[699] = static_cast<T>(0.06924749999999999);
        h_XImats[700] = static_cast<T>(0.39765212);
        h_XImats[701] = static_cast<T>(0.0);
        h_XImats[702] = static_cast<T>(0.0);
        h_XImats[703] = static_cast<T>(0.0);
        h_XImats[704] = static_cast<T>(0.06924749999999999);
        h_XImats[705] = static_cast<T>(2.638);
        h_XImats[706] = static_cast<T>(0.0);
        h_XImats[707] = static_cast<T>(0.0);
        h_XImats[708] = static_cast<T>(0.0);
        h_XImats[709] = static_cast<T>(0.0);
        h_XImats[710] = static_cast<T>(0.39765212);
        h_XImats[711] = static_cast<T>(0.0);
        h_XImats[712] = static_cast<T>(2.638);
        h_XImats[713] = static_cast<T>(0.0);
        h_XImats[714] = static_cast<T>(-0.06924749999999999);
        h_XImats[715] = static_cast<T>(-0.39765212);
        h_XImats[716] = static_cast<T>(0.0);
        h_XImats[717] = static_cast<T>(0.0);
        h_XImats[718] = static_cast<T>(0.0);
        h_XImats[719] = static_cast<T>(2.638);
        // I[8]
        h_XImats[720] = static_cast<T>(0.0004700102196);
        h_XImats[721] = static_cast<T>(5.5580904e-05);
        h_XImats[722] = static_cast<T>(1.0477400000000013e-06);
        h_XImats[723] = static_cast<T>(0.0);
        h_XImats[724] = static_cast<T>(8.81e-05);
        h_XImats[725] = static_cast<T>(3.5239999999999995e-05);
        h_XImats[726] = static_cast<T>(5.5580904e-05);
        h_XImats[727] = static_cast<T>(0.026413874770000005);
        h_XImats[728] = static_cast<T>(3.524e-09);
        h_XImats[729] = static_cast<T>(-8.81e-05);
        h_XImats[730] = static_cast<T>(0.0);
        h_XImats[731] = static_cast<T>(-0.11047740000000002);
        h_XImats[732] = static_cast<T>(1.0477399999999996e-06);
        h_XImats[733] = static_cast<T>(3.524e-09);
        h_XImats[734] = static_cast<T>(0.026183867369600003);
        h_XImats[735] = static_cast<T>(-3.5239999999999995e-05);
        h_XImats[736] = static_cast<T>(0.11047740000000002);
        h_XImats[737] = static_cast<T>(0.0);
        h_XImats[738] = static_cast<T>(0.0);
        h_XImats[739] = static_cast<T>(-8.81e-05);
        h_XImats[740] = static_cast<T>(-3.5239999999999995e-05);
        h_XImats[741] = static_cast<T>(0.881);
        h_XImats[742] = static_cast<T>(0.0);
        h_XImats[743] = static_cast<T>(0.0);
        h_XImats[744] = static_cast<T>(8.81e-05);
        h_XImats[745] = static_cast<T>(0.0);
        h_XImats[746] = static_cast<T>(0.11047740000000002);
        h_XImats[747] = static_cast<T>(0.0);
        h_XImats[748] = static_cast<T>(0.881);
        h_XImats[749] = static_cast<T>(0.0);
        h_XImats[750] = static_cast<T>(3.5239999999999995e-05);
        h_XImats[751] = static_cast<T>(-0.11047740000000002);
        h_XImats[752] = static_cast<T>(0.0);
        h_XImats[753] = static_cast<T>(0.0);
        h_XImats[754] = static_cast<T>(0.0);
        h_XImats[755] = static_cast<T>(0.881);
        // I[9]
        h_XImats[756] = static_cast<T>(0.134701016973);
        h_XImats[757] = static_cast<T>(-4e-05);
        h_XImats[758] = static_cast<T>(-0.022737817929000002);
        h_XImats[759] = static_cast<T>(0.0);
        h_XImats[760] = static_cast<T>(-0.49607829999999997);
        h_XImats[761] = static_cast<T>(0.0);
        h_XImats[762] = static_cast<T>(-4e-05);
        h_XImats[763] = static_cast<T>(0.14417575549);
        h_XImats[764] = static_cast<T>(-5e-05);
        h_XImats[765] = static_cast<T>(0.49607829999999997);
        h_XImats[766] = static_cast<T>(0.0);
        h_XImats[767] = static_cast<T>(-0.12490590000000001);
        h_XImats[768] = static_cast<T>(-0.022737817929);
        h_XImats[769] = static_cast<T>(-5e-05);
        h_XImats[770] = static_cast<T>(0.011034738517);
        h_XImats[771] = static_cast<T>(0.0);
        h_XImats[772] = static_cast<T>(0.12490590000000001);
        h_XImats[773] = static_cast<T>(0.0);
        h_XImats[774] = static_cast<T>(0.0);
        h_XImats[775] = static_cast<T>(0.49607829999999997);
        h_XImats[776] = static_cast<T>(0.0);
        h_XImats[777] = static_cast<T>(2.93);
        h_XImats[778] = static_cast<T>(0.0);
        h_XImats[779] = static_cast<T>(0.0);
        h_XImats[780] = static_cast<T>(-0.49607829999999997);
        h_XImats[781] = static_cast<T>(0.0);
        h_XImats[782] = static_cast<T>(0.12490590000000001);
        h_XImats[783] = static_cast<T>(0.0);
        h_XImats[784] = static_cast<T>(2.93);
        h_XImats[785] = static_cast<T>(0.0);
        h_XImats[786] = static_cast<T>(0.0);
        h_XImats[787] = static_cast<T>(-0.12490590000000001);
        h_XImats[788] = static_cast<T>(0.0);
        h_XImats[789] = static_cast<T>(0.0);
        h_XImats[790] = static_cast<T>(0.0);
        h_XImats[791] = static_cast<T>(2.93);
        // I[10]
        h_XImats[792] = static_cast<T>(0.005497746874999999);
        h_XImats[793] = static_cast<T>(-0.00741836815);
        h_XImats[794] = static_cast<T>(-0.0001);
        h_XImats[795] = static_cast<T>(0.0);
        h_XImats[796] = static_cast<T>(0.0);
        h_XImats[797] = static_cast<T>(0.06924749999999999);
        h_XImats[798] = static_cast<T>(-0.007418368149999998);
        h_XImats[799] = static_cast<T>(0.08713208056880001);
        h_XImats[800] = static_cast<T>(2e-05);
        h_XImats[801] = static_cast<T>(0.0);
        h_XImats[802] = static_cast<T>(0.0);
        h_XImats[803] = static_cast<T>(-0.39765212);
        h_XImats[804] = static_cast<T>(-0.0001);
        h_XImats[805] = static_cast<T>(2e-05);
        h_XImats[806] = static_cast<T>(0.0898698274438);
        h_XImats[807] = static_cast<T>(-0.06924749999999999);
        h_XImats[808] = static_cast<T>(0.39765212);
        h_XImats[809] = static_cast<T>(0.0);
        h_XImats[810] = static_cast<T>(0.0);
        h_XImats[811] = static_cast<T>(0.0);
        h_XImats[812] = static_cast<T>(-0.06924749999999999);
        h_XImats[813] = static_cast<T>(2.638);
        h_XImats[814] = static_cast<T>(0.0);
        h_XImats[815] = static_cast<T>(0.0);
        h_XImats[816] = static_cast<T>(0.0);
        h_XImats[817] = static_cast<T>(0.0);
        h_XImats[818] = static_cast<T>(0.39765212);
        h_XImats[819] = static_cast<T>(0.0);
        h_XImats[820] = static_cast<T>(2.638);
        h_XImats[821] = static_cast<T>(0.0);
        h_XImats[822] = static_cast<T>(0.06924749999999999);
        h_XImats[823] = static_cast<T>(-0.39765212);
        h_XImats[824] = static_cast<T>(0.0);
        h_XImats[825] = static_cast<T>(0.0);
        h_XImats[826] = static_cast<T>(0.0);
        h_XImats[827] = static_cast<T>(2.638);
        // I[11]
        h_XImats[828] = static_cast<T>(0.0004700102196);
        h_XImats[829] = static_cast<T>(-5.5580904e-05);
        h_XImats[830] = static_cast<T>(-1.0477400000000013e-06);
        h_XImats[831] = static_cast<T>(0.0);
        h_XImats[832] = static_cast<T>(-8.81e-05);
        h_XImats[833] = static_cast<T>(-3.5239999999999995e-05);
        h_XImats[834] = static_cast<T>(-5.5580904e-05);
        h_XImats[835] = static_cast<T>(0.026413874770000005);
        h_XImats[836] = static_cast<T>(3.524e-09);
        h_XImats[837] = static_cast<T>(8.81e-05);
        h_XImats[838] = static_cast<T>(0.0);
        h_XImats[839] = static_cast<T>(-0.11047740000000002);
        h_XImats[840] = static_cast<T>(-1.0477399999999996e-06);
        h_XImats[841] = static_cast<T>(3.524e-09);
        h_XImats[842] = static_cast<T>(0.026183867369600003);
        h_XImats[843] = static_cast<T>(3.5239999999999995e-05);
        h_XImats[844] = static_cast<T>(0.11047740000000002);
        h_XImats[845] = static_cast<T>(0.0);
        h_XImats[846] = static_cast<T>(0.0);
        h_XImats[847] = static_cast<T>(8.81e-05);
        h_XImats[848] = static_cast<T>(3.5239999999999995e-05);
        h_XImats[849] = static_cast<T>(0.881);
        h_XImats[850] = static_cast<T>(0.0);
        h_XImats[851] = static_cast<T>(0.0);
        h_XImats[852] = static_cast<T>(-8.81e-05);
        h_XImats[853] = static_cast<T>(0.0);
        h_XImats[854] = static_cast<T>(0.11047740000000002);
        h_XImats[855] = static_cast<T>(0.0);
        h_XImats[856] = static_cast<T>(0.881);
        h_XImats[857] = static_cast<T>(0.0);
        h_XImats[858] = static_cast<T>(-3.5239999999999995e-05);
        h_XImats[859] = static_cast<T>(-0.11047740000000002);
        h_XImats[860] = static_cast<T>(0.0);
        h_XImats[861] = static_cast<T>(0.0);
        h_XImats[862] = static_cast<T>(0.0);
        h_XImats[863] = static_cast<T>(0.881);
        // Xhom[0]
        h_XImats[864] = static_cast<T>(0);
        h_XImats[865] = static_cast<T>(0);
        h_XImats[866] = static_cast<T>(0);
        h_XImats[867] = static_cast<T>(0);
        h_XImats[868] = static_cast<T>(0);
        h_XImats[869] = static_cast<T>(0);
        h_XImats[870] = static_cast<T>(0);
        h_XImats[871] = static_cast<T>(0);
        h_XImats[872] = static_cast<T>(-1.00000000000000);
        h_XImats[873] = static_cast<T>(0);
        h_XImats[874] = static_cast<T>(0);
        h_XImats[875] = static_cast<T>(0);
        h_XImats[876] = static_cast<T>(0.373500000000000);
        h_XImats[877] = static_cast<T>(0.207000000000000);
        h_XImats[878] = static_cast<T>(0);
        h_XImats[879] = static_cast<T>(1.00000000000000);
        // Xhom[1]
        h_XImats[880] = static_cast<T>(0);
        h_XImats[881] = static_cast<T>(0);
        h_XImats[882] = static_cast<T>(0);
        h_XImats[883] = static_cast<T>(0);
        h_XImats[884] = static_cast<T>(0);
        h_XImats[885] = static_cast<T>(0);
        h_XImats[886] = static_cast<T>(0);
        h_XImats[887] = static_cast<T>(0);
        h_XImats[888] = static_cast<T>(0);
        h_XImats[889] = static_cast<T>(-1.00000000000000);
        h_XImats[890] = static_cast<T>(0);
        h_XImats[891] = static_cast<T>(0);
        h_XImats[892] = static_cast<T>(0.0800000000000000);
        h_XImats[893] = static_cast<T>(0);
        h_XImats[894] = static_cast<T>(0);
        h_XImats[895] = static_cast<T>(1.00000000000000);
        // Xhom[2]
        h_XImats[896] = static_cast<T>(0);
        h_XImats[897] = static_cast<T>(0);
        h_XImats[898] = static_cast<T>(0);
        h_XImats[899] = static_cast<T>(0);
        h_XImats[900] = static_cast<T>(0);
        h_XImats[901] = static_cast<T>(0);
        h_XImats[902] = static_cast<T>(0);
        h_XImats[903] = static_cast<T>(0);
        h_XImats[904] = static_cast<T>(0);
        h_XImats[905] = static_cast<T>(0);
        h_XImats[906] = static_cast<T>(1.00000000000000);
        h_XImats[907] = static_cast<T>(0);
        h_XImats[908] = static_cast<T>(0.350000000000000);
        h_XImats[909] = static_cast<T>(0);
        h_XImats[910] = static_cast<T>(0);
        h_XImats[911] = static_cast<T>(1.00000000000000);
        // Xhom[3]
        h_XImats[912] = static_cast<T>(0);
        h_XImats[913] = static_cast<T>(0);
        h_XImats[914] = static_cast<T>(0);
        h_XImats[915] = static_cast<T>(0);
        h_XImats[916] = static_cast<T>(0);
        h_XImats[917] = static_cast<T>(0);
        h_XImats[918] = static_cast<T>(0);
        h_XImats[919] = static_cast<T>(0);
        h_XImats[920] = static_cast<T>(-1.00000000000000);
        h_XImats[921] = static_cast<T>(0);
        h_XImats[922] = static_cast<T>(0);
        h_XImats[923] = static_cast<T>(0);
        h_XImats[924] = static_cast<T>(-0.373500000000000);
        h_XImats[925] = static_cast<T>(0.207000000000000);
        h_XImats[926] = static_cast<T>(0);
        h_XImats[927] = static_cast<T>(1.00000000000000);
        // Xhom[4]
        h_XImats[928] = static_cast<T>(0);
        h_XImats[929] = static_cast<T>(0);
        h_XImats[930] = static_cast<T>(0);
        h_XImats[931] = static_cast<T>(0);
        h_XImats[932] = static_cast<T>(0);
        h_XImats[933] = static_cast<T>(0);
        h_XImats[934] = static_cast<T>(0);
        h_XImats[935] = static_cast<T>(0);
        h_XImats[936] = static_cast<T>(0);
        h_XImats[937] = static_cast<T>(-1.00000000000000);
        h_XImats[938] = static_cast<T>(0);
        h_XImats[939] = static_cast<T>(0);
        h_XImats[940] = static_cast<T>(0.0800000000000000);
        h_XImats[941] = static_cast<T>(0);
        h_XImats[942] = static_cast<T>(0);
        h_XImats[943] = static_cast<T>(1.00000000000000);
        // Xhom[5]
        h_XImats[944] = static_cast<T>(0);
        h_XImats[945] = static_cast<T>(0);
        h_XImats[946] = static_cast<T>(0);
        h_XImats[947] = static_cast<T>(0);
        h_XImats[948] = static_cast<T>(0);
        h_XImats[949] = static_cast<T>(0);
        h_XImats[950] = static_cast<T>(0);
        h_XImats[951] = static_cast<T>(0);
        h_XImats[952] = static_cast<T>(0);
        h_XImats[953] = static_cast<T>(0);
        h_XImats[954] = static_cast<T>(1.00000000000000);
        h_XImats[955] = static_cast<T>(0);
        h_XImats[956] = static_cast<T>(0.350000000000000);
        h_XImats[957] = static_cast<T>(0);
        h_XImats[958] = static_cast<T>(0);
        h_XImats[959] = static_cast<T>(1.00000000000000);
        // Xhom[6]
        h_XImats[960] = static_cast<T>(0);
        h_XImats[961] = static_cast<T>(0);
        h_XImats[962] = static_cast<T>(0);
        h_XImats[963] = static_cast<T>(0);
        h_XImats[964] = static_cast<T>(0);
        h_XImats[965] = static_cast<T>(0);
        h_XImats[966] = static_cast<T>(0);
        h_XImats[967] = static_cast<T>(0);
        h_XImats[968] = static_cast<T>(1.00000000000000);
        h_XImats[969] = static_cast<T>(0);
        h_XImats[970] = static_cast<T>(0);
        h_XImats[971] = static_cast<T>(0);
        h_XImats[972] = static_cast<T>(0.373500000000000);
        h_XImats[973] = static_cast<T>(-0.207000000000000);
        h_XImats[974] = static_cast<T>(0);
        h_XImats[975] = static_cast<T>(1.00000000000000);
        // Xhom[7]
        h_XImats[976] = static_cast<T>(0);
        h_XImats[977] = static_cast<T>(0);
        h_XImats[978] = static_cast<T>(0);
        h_XImats[979] = static_cast<T>(0);
        h_XImats[980] = static_cast<T>(0);
        h_XImats[981] = static_cast<T>(0);
        h_XImats[982] = static_cast<T>(0);
        h_XImats[983] = static_cast<T>(0);
        h_XImats[984] = static_cast<T>(0);
        h_XImats[985] = static_cast<T>(1.00000000000000);
        h_XImats[986] = static_cast<T>(0);
        h_XImats[987] = static_cast<T>(0);
        h_XImats[988] = static_cast<T>(0.0800000000000000);
        h_XImats[989] = static_cast<T>(0);
        h_XImats[990] = static_cast<T>(0);
        h_XImats[991] = static_cast<T>(1.00000000000000);
        // Xhom[8]
        h_XImats[992] = static_cast<T>(0);
        h_XImats[993] = static_cast<T>(0);
        h_XImats[994] = static_cast<T>(0);
        h_XImats[995] = static_cast<T>(0);
        h_XImats[996] = static_cast<T>(0);
        h_XImats[997] = static_cast<T>(0);
        h_XImats[998] = static_cast<T>(0);
        h_XImats[999] = static_cast<T>(0);
        h_XImats[1000] = static_cast<T>(0);
        h_XImats[1001] = static_cast<T>(0);
        h_XImats[1002] = static_cast<T>(1.00000000000000);
        h_XImats[1003] = static_cast<T>(0);
        h_XImats[1004] = static_cast<T>(0.350000000000000);
        h_XImats[1005] = static_cast<T>(0);
        h_XImats[1006] = static_cast<T>(0);
        h_XImats[1007] = static_cast<T>(1.00000000000000);
        // Xhom[9]
        h_XImats[1008] = static_cast<T>(0);
        h_XImats[1009] = static_cast<T>(0);
        h_XImats[1010] = static_cast<T>(0);
        h_XImats[1011] = static_cast<T>(0);
        h_XImats[1012] = static_cast<T>(0);
        h_XImats[1013] = static_cast<T>(0);
        h_XImats[1014] = static_cast<T>(0);
        h_XImats[1015] = static_cast<T>(0);
        h_XImats[1016] = static_cast<T>(1.00000000000000);
        h_XImats[1017] = static_cast<T>(0);
        h_XImats[1018] = static_cast<T>(0);
        h_XImats[1019] = static_cast<T>(0);
        h_XImats[1020] = static_cast<T>(-0.373500000000000);
        h_XImats[1021] = static_cast<T>(-0.207000000000000);
        h_XImats[1022] = static_cast<T>(0);
        h_XImats[1023] = static_cast<T>(1.00000000000000);
        // Xhom[10]
        h_XImats[1024] = static_cast<T>(0);
        h_XImats[1025] = static_cast<T>(0);
        h_XImats[1026] = static_cast<T>(0);
        h_XImats[1027] = static_cast<T>(0);
        h_XImats[1028] = static_cast<T>(0);
        h_XImats[1029] = static_cast<T>(0);
        h_XImats[1030] = static_cast<T>(0);
        h_XImats[1031] = static_cast<T>(0);
        h_XImats[1032] = static_cast<T>(0);
        h_XImats[1033] = static_cast<T>(1.00000000000000);
        h_XImats[1034] = static_cast<T>(0);
        h_XImats[1035] = static_cast<T>(0);
        h_XImats[1036] = static_cast<T>(0.0800000000000000);
        h_XImats[1037] = static_cast<T>(0);
        h_XImats[1038] = static_cast<T>(0);
        h_XImats[1039] = static_cast<T>(1.00000000000000);
        // Xhom[11]
        h_XImats[1040] = static_cast<T>(0);
        h_XImats[1041] = static_cast<T>(0);
        h_XImats[1042] = static_cast<T>(0);
        h_XImats[1043] = static_cast<T>(0);
        h_XImats[1044] = static_cast<T>(0);
        h_XImats[1045] = static_cast<T>(0);
        h_XImats[1046] = static_cast<T>(0);
        h_XImats[1047] = static_cast<T>(0);
        h_XImats[1048] = static_cast<T>(0);
        h_XImats[1049] = static_cast<T>(0);
        h_XImats[1050] = static_cast<T>(1.00000000000000);
        h_XImats[1051] = static_cast<T>(0);
        h_XImats[1052] = static_cast<T>(0.350000000000000);
        h_XImats[1053] = static_cast<T>(0);
        h_XImats[1054] = static_cast<T>(0);
        h_XImats[1055] = static_cast<T>(1.00000000000000);
        // dXhom[0]
        h_XImats[1056] = static_cast<T>(0);
        h_XImats[1057] = static_cast<T>(0);
        h_XImats[1058] = static_cast<T>(0);
        h_XImats[1059] = static_cast<T>(0);
        h_XImats[1060] = static_cast<T>(0);
        h_XImats[1061] = static_cast<T>(0);
        h_XImats[1062] = static_cast<T>(0);
        h_XImats[1063] = static_cast<T>(0);
        h_XImats[1064] = static_cast<T>(0);
        h_XImats[1065] = static_cast<T>(0);
        h_XImats[1066] = static_cast<T>(0);
        h_XImats[1067] = static_cast<T>(0);
        h_XImats[1068] = static_cast<T>(0);
        h_XImats[1069] = static_cast<T>(0);
        h_XImats[1070] = static_cast<T>(0);
        h_XImats[1071] = static_cast<T>(0);
        // dXhom[1]
        h_XImats[1072] = static_cast<T>(0);
        h_XImats[1073] = static_cast<T>(0);
        h_XImats[1074] = static_cast<T>(0);
        h_XImats[1075] = static_cast<T>(0);
        h_XImats[1076] = static_cast<T>(0);
        h_XImats[1077] = static_cast<T>(0);
        h_XImats[1078] = static_cast<T>(0);
        h_XImats[1079] = static_cast<T>(0);
        h_XImats[1080] = static_cast<T>(0);
        h_XImats[1081] = static_cast<T>(0);
        h_XImats[1082] = static_cast<T>(0);
        h_XImats[1083] = static_cast<T>(0);
        h_XImats[1084] = static_cast<T>(0);
        h_XImats[1085] = static_cast<T>(0);
        h_XImats[1086] = static_cast<T>(0);
        h_XImats[1087] = static_cast<T>(0);
        // dXhom[2]
        h_XImats[1088] = static_cast<T>(0);
        h_XImats[1089] = static_cast<T>(0);
        h_XImats[1090] = static_cast<T>(0);
        h_XImats[1091] = static_cast<T>(0);
        h_XImats[1092] = static_cast<T>(0);
        h_XImats[1093] = static_cast<T>(0);
        h_XImats[1094] = static_cast<T>(0);
        h_XImats[1095] = static_cast<T>(0);
        h_XImats[1096] = static_cast<T>(0);
        h_XImats[1097] = static_cast<T>(0);
        h_XImats[1098] = static_cast<T>(0);
        h_XImats[1099] = static_cast<T>(0);
        h_XImats[1100] = static_cast<T>(0);
        h_XImats[1101] = static_cast<T>(0);
        h_XImats[1102] = static_cast<T>(0);
        h_XImats[1103] = static_cast<T>(0);
        // dXhom[3]
        h_XImats[1104] = static_cast<T>(0);
        h_XImats[1105] = static_cast<T>(0);
        h_XImats[1106] = static_cast<T>(0);
        h_XImats[1107] = static_cast<T>(0);
        h_XImats[1108] = static_cast<T>(0);
        h_XImats[1109] = static_cast<T>(0);
        h_XImats[1110] = static_cast<T>(0);
        h_XImats[1111] = static_cast<T>(0);
        h_XImats[1112] = static_cast<T>(0);
        h_XImats[1113] = static_cast<T>(0);
        h_XImats[1114] = static_cast<T>(0);
        h_XImats[1115] = static_cast<T>(0);
        h_XImats[1116] = static_cast<T>(0);
        h_XImats[1117] = static_cast<T>(0);
        h_XImats[1118] = static_cast<T>(0);
        h_XImats[1119] = static_cast<T>(0);
        // dXhom[4]
        h_XImats[1120] = static_cast<T>(0);
        h_XImats[1121] = static_cast<T>(0);
        h_XImats[1122] = static_cast<T>(0);
        h_XImats[1123] = static_cast<T>(0);
        h_XImats[1124] = static_cast<T>(0);
        h_XImats[1125] = static_cast<T>(0);
        h_XImats[1126] = static_cast<T>(0);
        h_XImats[1127] = static_cast<T>(0);
        h_XImats[1128] = static_cast<T>(0);
        h_XImats[1129] = static_cast<T>(0);
        h_XImats[1130] = static_cast<T>(0);
        h_XImats[1131] = static_cast<T>(0);
        h_XImats[1132] = static_cast<T>(0);
        h_XImats[1133] = static_cast<T>(0);
        h_XImats[1134] = static_cast<T>(0);
        h_XImats[1135] = static_cast<T>(0);
        // dXhom[5]
        h_XImats[1136] = static_cast<T>(0);
        h_XImats[1137] = static_cast<T>(0);
        h_XImats[1138] = static_cast<T>(0);
        h_XImats[1139] = static_cast<T>(0);
        h_XImats[1140] = static_cast<T>(0);
        h_XImats[1141] = static_cast<T>(0);
        h_XImats[1142] = static_cast<T>(0);
        h_XImats[1143] = static_cast<T>(0);
        h_XImats[1144] = static_cast<T>(0);
        h_XImats[1145] = static_cast<T>(0);
        h_XImats[1146] = static_cast<T>(0);
        h_XImats[1147] = static_cast<T>(0);
        h_XImats[1148] = static_cast<T>(0);
        h_XImats[1149] = static_cast<T>(0);
        h_XImats[1150] = static_cast<T>(0);
        h_XImats[1151] = static_cast<T>(0);
        // dXhom[6]
        h_XImats[1152] = static_cast<T>(0);
        h_XImats[1153] = static_cast<T>(0);
        h_XImats[1154] = static_cast<T>(0);
        h_XImats[1155] = static_cast<T>(0);
        h_XImats[1156] = static_cast<T>(0);
        h_XImats[1157] = static_cast<T>(0);
        h_XImats[1158] = static_cast<T>(0);
        h_XImats[1159] = static_cast<T>(0);
        h_XImats[1160] = static_cast<T>(0);
        h_XImats[1161] = static_cast<T>(0);
        h_XImats[1162] = static_cast<T>(0);
        h_XImats[1163] = static_cast<T>(0);
        h_XImats[1164] = static_cast<T>(0);
        h_XImats[1165] = static_cast<T>(0);
        h_XImats[1166] = static_cast<T>(0);
        h_XImats[1167] = static_cast<T>(0);
        // dXhom[7]
        h_XImats[1168] = static_cast<T>(0);
        h_XImats[1169] = static_cast<T>(0);
        h_XImats[1170] = static_cast<T>(0);
        h_XImats[1171] = static_cast<T>(0);
        h_XImats[1172] = static_cast<T>(0);
        h_XImats[1173] = static_cast<T>(0);
        h_XImats[1174] = static_cast<T>(0);
        h_XImats[1175] = static_cast<T>(0);
        h_XImats[1176] = static_cast<T>(0);
        h_XImats[1177] = static_cast<T>(0);
        h_XImats[1178] = static_cast<T>(0);
        h_XImats[1179] = static_cast<T>(0);
        h_XImats[1180] = static_cast<T>(0);
        h_XImats[1181] = static_cast<T>(0);
        h_XImats[1182] = static_cast<T>(0);
        h_XImats[1183] = static_cast<T>(0);
        // dXhom[8]
        h_XImats[1184] = static_cast<T>(0);
        h_XImats[1185] = static_cast<T>(0);
        h_XImats[1186] = static_cast<T>(0);
        h_XImats[1187] = static_cast<T>(0);
        h_XImats[1188] = static_cast<T>(0);
        h_XImats[1189] = static_cast<T>(0);
        h_XImats[1190] = static_cast<T>(0);
        h_XImats[1191] = static_cast<T>(0);
        h_XImats[1192] = static_cast<T>(0);
        h_XImats[1193] = static_cast<T>(0);
        h_XImats[1194] = static_cast<T>(0);
        h_XImats[1195] = static_cast<T>(0);
        h_XImats[1196] = static_cast<T>(0);
        h_XImats[1197] = static_cast<T>(0);
        h_XImats[1198] = static_cast<T>(0);
        h_XImats[1199] = static_cast<T>(0);
        // dXhom[9]
        h_XImats[1200] = static_cast<T>(0);
        h_XImats[1201] = static_cast<T>(0);
        h_XImats[1202] = static_cast<T>(0);
        h_XImats[1203] = static_cast<T>(0);
        h_XImats[1204] = static_cast<T>(0);
        h_XImats[1205] = static_cast<T>(0);
        h_XImats[1206] = static_cast<T>(0);
        h_XImats[1207] = static_cast<T>(0);
        h_XImats[1208] = static_cast<T>(0);
        h_XImats[1209] = static_cast<T>(0);
        h_XImats[1210] = static_cast<T>(0);
        h_XImats[1211] = static_cast<T>(0);
        h_XImats[1212] = static_cast<T>(0);
        h_XImats[1213] = static_cast<T>(0);
        h_XImats[1214] = static_cast<T>(0);
        h_XImats[1215] = static_cast<T>(0);
        // dXhom[10]
        h_XImats[1216] = static_cast<T>(0);
        h_XImats[1217] = static_cast<T>(0);
        h_XImats[1218] = static_cast<T>(0);
        h_XImats[1219] = static_cast<T>(0);
        h_XImats[1220] = static_cast<T>(0);
        h_XImats[1221] = static_cast<T>(0);
        h_XImats[1222] = static_cast<T>(0);
        h_XImats[1223] = static_cast<T>(0);
        h_XImats[1224] = static_cast<T>(0);
        h_XImats[1225] = static_cast<T>(0);
        h_XImats[1226] = static_cast<T>(0);
        h_XImats[1227] = static_cast<T>(0);
        h_XImats[1228] = static_cast<T>(0);
        h_XImats[1229] = static_cast<T>(0);
        h_XImats[1230] = static_cast<T>(0);
        h_XImats[1231] = static_cast<T>(0);
        // dXhom[11]
        h_XImats[1232] = static_cast<T>(0);
        h_XImats[1233] = static_cast<T>(0);
        h_XImats[1234] = static_cast<T>(0);
        h_XImats[1235] = static_cast<T>(0);
        h_XImats[1236] = static_cast<T>(0);
        h_XImats[1237] = static_cast<T>(0);
        h_XImats[1238] = static_cast<T>(0);
        h_XImats[1239] = static_cast<T>(0);
        h_XImats[1240] = static_cast<T>(0);
        h_XImats[1241] = static_cast<T>(0);
        h_XImats[1242] = static_cast<T>(0);
        h_XImats[1243] = static_cast<T>(0);
        h_XImats[1244] = static_cast<T>(0);
        h_XImats[1245] = static_cast<T>(0);
        h_XImats[1246] = static_cast<T>(0);
        h_XImats[1247] = static_cast<T>(0);
        T *d_XImats; gpuErrchk(cudaMalloc((void**)&d_XImats,1248*sizeof(T)));
        gpuErrchk(cudaMemcpy(d_XImats,h_XImats,1248*sizeof(T),cudaMemcpyHostToDevice));
        free(h_XImats);
        return d_XImats;
    }

    /**
     * Initializes the robotModel helpers in GPU memory
     *
     * @return A pointer to the robotModel struct
     */
    template <typename T>
    __host__
    robotModel<T>* init_robotModel() {
        robotModel<T> h_robotModel;
        h_robotModel.d_XImats = init_XImats<T>();
        h_robotModel.d_topology_helpers = init_topology_helpers<T>();
        robotModel<T> *d_robotModel; gpuErrchk(cudaMalloc((void**)&d_robotModel,sizeof(robotModel<T>)));
        gpuErrchk(cudaMemcpy(d_robotModel,&h_robotModel,sizeof(robotModel<T>),cudaMemcpyHostToDevice));
        return d_robotModel;
    }

    /**
     * Allocated device and host memory for all computations
     *
     * @return A pointer to the gridData struct of pointers
     */
    template <typename T, int NUM_TIMESTEPS>
    __host__
    gridData<T> *init_gridData(){
        gridData<T> *hd_data = (gridData<T> *)malloc(sizeof(gridData<T>));// first the input variables on the GPU
        gpuErrchk(cudaMalloc((void**)&hd_data->d_q_qd_u, 3*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_q_qd, 2*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_q, NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        // and the CPU
        hd_data->h_q_qd_u = (T *)malloc(3*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_q_qd = (T *)malloc(2*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_q = (T *)malloc(NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        // then the GPU outputs
        gpuErrchk(cudaMalloc((void**)&hd_data->d_c, NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_Minv, NUM_JOINTS*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_qdd, NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_M, NUM_JOINTS*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_dc_du, NUM_JOINTS*2*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_df_du, NUM_JOINTS*2*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_eePos, 6*NUM_EES*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_deePos, 6*NUM_EES*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        // and the CPU
        hd_data->h_c = (T *)malloc(NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_Minv = (T *)malloc(NUM_JOINTS*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_M = (T *)malloc(NUM_JOINTS*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_qdd = (T *)malloc(NUM_JOINTS*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_dc_du = (T *)malloc(NUM_JOINTS*2*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_df_du = (T *)malloc(NUM_JOINTS*2*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_eePos = (T *)malloc(6*NUM_EES*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_deePos = (T *)malloc(6*NUM_EES*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        return hd_data;
    }

    /**
     * Allocated device and host memory for all computations
     *
     * @param Max number of timesteps in the trajectory
     * @return A pointer to the gridData struct of pointers
     */
    template <typename T>
    __host__
    gridData<T> *init_gridData(int NUM_TIMESTEPS){
        gridData<T> *hd_data = (gridData<T> *)malloc(sizeof(gridData<T>));// first the input variables on the GPU
        gpuErrchk(cudaMalloc((void**)&hd_data->d_q_qd_u, 3*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_q_qd, 2*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_q, NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        // and the CPU
        hd_data->h_q_qd_u = (T *)malloc(3*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_q_qd = (T *)malloc(2*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_q = (T *)malloc(NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        // then the GPU outputs
        gpuErrchk(cudaMalloc((void**)&hd_data->d_c, NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_Minv, NUM_JOINTS*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_qdd, NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_M, NUM_JOINTS*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_dc_du, NUM_JOINTS*2*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_df_du, NUM_JOINTS*2*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_eePos, 6*NUM_EES*NUM_TIMESTEPS*sizeof(T)));
        gpuErrchk(cudaMalloc((void**)&hd_data->d_deePos, 6*NUM_EES*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T)));
        // and the CPU
        hd_data->h_c = (T *)malloc(NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_Minv = (T *)malloc(NUM_JOINTS*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_M = (T *)malloc(NUM_JOINTS*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_qdd = (T *)malloc(NUM_JOINTS*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_dc_du = (T *)malloc(NUM_JOINTS*2*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_df_du = (T *)malloc(NUM_JOINTS*2*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_eePos = (T *)malloc(6*NUM_EES*NUM_TIMESTEPS*sizeof(T));
        hd_data->h_deePos = (T *)malloc(6*NUM_EES*NUM_JOINTS*NUM_TIMESTEPS*sizeof(T));
        return hd_data;
    }

    /**
     * Updates the Xmats in (shared) GPU memory acording to the configuration
     *
     * @param s_XImats is the (shared) memory destination location for the XImats
     * @param s_q is the (shared) memory location of the current configuration
     * @param s_topology_helpers is the (shared) memory destination location for the topology_helpers
     * @param d_robotModel is the pointer to the initialized model specific helpers (XImats, mxfuncs, topology_helpers, etc.)
     * @param s_temp is temporary (shared) memory used to compute sin and cos if needed of size: 24
     */
    template <typename T>
    __device__
    void load_update_XImats_helpers(T *s_XImats, const T *s_q, int *s_topology_helpers, const robotModel<T> *d_robotModel, T *s_temp) {
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 864; ind += blockDim.x*blockDim.y){
            s_XImats[ind] = d_robotModel->d_XImats[ind];
        }
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 61; ind += blockDim.x*blockDim.y){
            s_topology_helpers[ind] = d_robotModel->d_topology_helpers[ind];
        }
        for(int k = threadIdx.x + threadIdx.y*blockDim.x; k < 12; k += blockDim.x*blockDim.y){
            s_temp[k] = static_cast<T>(sin(s_q[k]));
            s_temp[k+12] = static_cast<T>(cos(s_q[k]));
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            // X[0]
            s_XImats[3] = static_cast<T>(-0.207*s_temp[12]);
            s_XImats[4] = static_cast<T>(0.207*s_temp[0]);
            s_XImats[6] = static_cast<T>(-s_temp[0]);
            s_XImats[7] = static_cast<T>(-s_temp[12]);
            s_XImats[9] = static_cast<T>(0.3735*s_temp[12]);
            s_XImats[10] = static_cast<T>(-0.3735*s_temp[0]);
            s_XImats[12] = static_cast<T>(-s_temp[12]);
            s_XImats[13] = static_cast<T>(s_temp[0]);
            s_XImats[15] = static_cast<T>(-0.3735*s_temp[0]);
            s_XImats[16] = static_cast<T>(-0.3735*s_temp[12]);
            // X[1]
            s_XImats[36] = static_cast<T>(s_temp[13]);
            s_XImats[37] = static_cast<T>(-s_temp[1]);
            s_XImats[45] = static_cast<T>(-0.08*s_temp[1]);
            s_XImats[46] = static_cast<T>(-0.08*s_temp[13]);
            s_XImats[48] = static_cast<T>(s_temp[1]);
            s_XImats[49] = static_cast<T>(s_temp[13]);
            // X[2]
            s_XImats[72] = static_cast<T>(s_temp[14]);
            s_XImats[73] = static_cast<T>(-s_temp[2]);
            s_XImats[78] = static_cast<T>(s_temp[2]);
            s_XImats[79] = static_cast<T>(s_temp[14]);
            s_XImats[87] = static_cast<T>(0.35*s_temp[2]);
            s_XImats[88] = static_cast<T>(0.35*s_temp[14]);
            // X[3]
            s_XImats[111] = static_cast<T>(-0.207*s_temp[15]);
            s_XImats[112] = static_cast<T>(0.207*s_temp[3]);
            s_XImats[114] = static_cast<T>(-s_temp[3]);
            s_XImats[115] = static_cast<T>(-s_temp[15]);
            s_XImats[117] = static_cast<T>(-0.3735*s_temp[15]);
            s_XImats[118] = static_cast<T>(0.3735*s_temp[3]);
            s_XImats[120] = static_cast<T>(-s_temp[15]);
            s_XImats[121] = static_cast<T>(s_temp[3]);
            s_XImats[123] = static_cast<T>(0.3735*s_temp[3]);
            s_XImats[124] = static_cast<T>(0.3735*s_temp[15]);
            // X[4]
            s_XImats[144] = static_cast<T>(s_temp[16]);
            s_XImats[145] = static_cast<T>(-s_temp[4]);
            s_XImats[153] = static_cast<T>(-0.08*s_temp[4]);
            s_XImats[154] = static_cast<T>(-0.08*s_temp[16]);
            s_XImats[156] = static_cast<T>(s_temp[4]);
            s_XImats[157] = static_cast<T>(s_temp[16]);
            // X[5]
            s_XImats[180] = static_cast<T>(s_temp[17]);
            s_XImats[181] = static_cast<T>(-s_temp[5]);
            s_XImats[186] = static_cast<T>(s_temp[5]);
            s_XImats[187] = static_cast<T>(s_temp[17]);
            s_XImats[195] = static_cast<T>(0.35*s_temp[5]);
            s_XImats[196] = static_cast<T>(0.35*s_temp[17]);
            // X[6]
            s_XImats[219] = static_cast<T>(0.207*s_temp[18]);
            s_XImats[220] = static_cast<T>(-0.207*s_temp[6]);
            s_XImats[222] = static_cast<T>(s_temp[6]);
            s_XImats[223] = static_cast<T>(s_temp[18]);
            s_XImats[225] = static_cast<T>(0.3735*s_temp[18]);
            s_XImats[226] = static_cast<T>(-0.3735*s_temp[6]);
            s_XImats[228] = static_cast<T>(-s_temp[18]);
            s_XImats[229] = static_cast<T>(s_temp[6]);
            s_XImats[231] = static_cast<T>(0.3735*s_temp[6]);
            s_XImats[232] = static_cast<T>(0.3735*s_temp[18]);
            // X[7]
            s_XImats[252] = static_cast<T>(s_temp[19]);
            s_XImats[253] = static_cast<T>(-s_temp[7]);
            s_XImats[261] = static_cast<T>(0.08*s_temp[7]);
            s_XImats[262] = static_cast<T>(0.08*s_temp[19]);
            s_XImats[264] = static_cast<T>(-s_temp[7]);
            s_XImats[265] = static_cast<T>(-s_temp[19]);
            // X[8]
            s_XImats[288] = static_cast<T>(s_temp[20]);
            s_XImats[289] = static_cast<T>(-s_temp[8]);
            s_XImats[294] = static_cast<T>(s_temp[8]);
            s_XImats[295] = static_cast<T>(s_temp[20]);
            s_XImats[303] = static_cast<T>(0.35*s_temp[8]);
            s_XImats[304] = static_cast<T>(0.35*s_temp[20]);
            // X[9]
            s_XImats[327] = static_cast<T>(0.207*s_temp[21]);
            s_XImats[328] = static_cast<T>(-0.207*s_temp[9]);
            s_XImats[330] = static_cast<T>(s_temp[9]);
            s_XImats[331] = static_cast<T>(s_temp[21]);
            s_XImats[333] = static_cast<T>(-0.3735*s_temp[21]);
            s_XImats[334] = static_cast<T>(0.3735*s_temp[9]);
            s_XImats[336] = static_cast<T>(-s_temp[21]);
            s_XImats[337] = static_cast<T>(s_temp[9]);
            s_XImats[339] = static_cast<T>(-0.3735*s_temp[9]);
            s_XImats[340] = static_cast<T>(-0.3735*s_temp[21]);
            // X[10]
            s_XImats[360] = static_cast<T>(s_temp[22]);
            s_XImats[361] = static_cast<T>(-s_temp[10]);
            s_XImats[369] = static_cast<T>(0.08*s_temp[10]);
            s_XImats[370] = static_cast<T>(0.08*s_temp[22]);
            s_XImats[372] = static_cast<T>(-s_temp[10]);
            s_XImats[373] = static_cast<T>(-s_temp[22]);
            // X[11]
            s_XImats[396] = static_cast<T>(s_temp[23]);
            s_XImats[397] = static_cast<T>(-s_temp[11]);
            s_XImats[402] = static_cast<T>(s_temp[11]);
            s_XImats[403] = static_cast<T>(s_temp[23]);
            s_XImats[411] = static_cast<T>(0.35*s_temp[11]);
            s_XImats[412] = static_cast<T>(0.35*s_temp[23]);
        }
        __syncthreads();
        for(int kcr = threadIdx.x + threadIdx.y*blockDim.x; kcr < 108; kcr += blockDim.x*blockDim.y){
            int k = kcr / 9; int cr = kcr % 9; int c = cr / 3; int r = cr % 3;
            int srcInd = k*36 + c*6 + r; int dstInd = srcInd + 21; // 3 more rows and cols
            s_XImats[dstInd] = s_XImats[srcInd];
        }
        __syncthreads();
    }

    /**
     * Updates the (d)XmatsHom in (shared) GPU memory acording to the configuration
     *
     * @param s_XmatsHom is the (shared) memory destination location for the XmatsHom
     * @param s_q is the (shared) memory location of the current configuration
     * @param s_topology_helpers is the (shared) memory destination location for the topology_helpers
     * @param d_robotModel is the pointer to the initialized model specific helpers (XImats, mxfuncs, topology_helpers, etc.)
     * @param s_temp is temporary (shared) memory used to compute sin and cos if needed of size: 24
     */
    template <typename T>
    __device__
    void load_update_XmatsHom_helpers(T *s_XmatsHom, int *s_topology_helpers, const T *s_q, const robotModel<T> *d_robotModel, T *s_temp) {
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 192; ind += blockDim.x*blockDim.y){
            s_XmatsHom[ind] = d_robotModel->d_XImats[ind+864];
        }
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 61; ind += blockDim.x*blockDim.y){
            s_topology_helpers[ind] = d_robotModel->d_topology_helpers[ind];
        }
        for(int k = threadIdx.x + threadIdx.y*blockDim.x; k < 12; k += blockDim.x*blockDim.y){
            s_temp[k] = static_cast<T>(sin(s_q[k]));
            s_temp[k+12] = static_cast<T>(cos(s_q[k]));
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            // X_hom[0]
            s_XmatsHom[1] = static_cast<T>(-s_temp[0]);
            s_XmatsHom[2] = static_cast<T>(-s_temp[12]);
            s_XmatsHom[5] = static_cast<T>(-s_temp[12]);
            s_XmatsHom[6] = static_cast<T>(s_temp[0]);
            // X_hom[1]
            s_XmatsHom[16] = static_cast<T>(s_temp[13]);
            s_XmatsHom[18] = static_cast<T>(s_temp[1]);
            s_XmatsHom[20] = static_cast<T>(-s_temp[1]);
            s_XmatsHom[22] = static_cast<T>(s_temp[13]);
            // X_hom[2]
            s_XmatsHom[32] = static_cast<T>(s_temp[14]);
            s_XmatsHom[33] = static_cast<T>(s_temp[2]);
            s_XmatsHom[36] = static_cast<T>(-s_temp[2]);
            s_XmatsHom[37] = static_cast<T>(s_temp[14]);
            // X_hom[3]
            s_XmatsHom[49] = static_cast<T>(-s_temp[3]);
            s_XmatsHom[50] = static_cast<T>(-s_temp[15]);
            s_XmatsHom[53] = static_cast<T>(-s_temp[15]);
            s_XmatsHom[54] = static_cast<T>(s_temp[3]);
            // X_hom[4]
            s_XmatsHom[64] = static_cast<T>(s_temp[16]);
            s_XmatsHom[66] = static_cast<T>(s_temp[4]);
            s_XmatsHom[68] = static_cast<T>(-s_temp[4]);
            s_XmatsHom[70] = static_cast<T>(s_temp[16]);
            // X_hom[5]
            s_XmatsHom[80] = static_cast<T>(s_temp[17]);
            s_XmatsHom[81] = static_cast<T>(s_temp[5]);
            s_XmatsHom[84] = static_cast<T>(-s_temp[5]);
            s_XmatsHom[85] = static_cast<T>(s_temp[17]);
            // X_hom[6]
            s_XmatsHom[97] = static_cast<T>(s_temp[6]);
            s_XmatsHom[98] = static_cast<T>(-s_temp[18]);
            s_XmatsHom[101] = static_cast<T>(s_temp[18]);
            s_XmatsHom[102] = static_cast<T>(s_temp[6]);
            // X_hom[7]
            s_XmatsHom[112] = static_cast<T>(s_temp[19]);
            s_XmatsHom[114] = static_cast<T>(-s_temp[7]);
            s_XmatsHom[116] = static_cast<T>(-s_temp[7]);
            s_XmatsHom[118] = static_cast<T>(-s_temp[19]);
            // X_hom[8]
            s_XmatsHom[128] = static_cast<T>(s_temp[20]);
            s_XmatsHom[129] = static_cast<T>(s_temp[8]);
            s_XmatsHom[132] = static_cast<T>(-s_temp[8]);
            s_XmatsHom[133] = static_cast<T>(s_temp[20]);
            // X_hom[9]
            s_XmatsHom[145] = static_cast<T>(s_temp[9]);
            s_XmatsHom[146] = static_cast<T>(-s_temp[21]);
            s_XmatsHom[149] = static_cast<T>(s_temp[21]);
            s_XmatsHom[150] = static_cast<T>(s_temp[9]);
            // X_hom[10]
            s_XmatsHom[160] = static_cast<T>(s_temp[22]);
            s_XmatsHom[162] = static_cast<T>(-s_temp[10]);
            s_XmatsHom[164] = static_cast<T>(-s_temp[10]);
            s_XmatsHom[166] = static_cast<T>(-s_temp[22]);
            // X_hom[11]
            s_XmatsHom[176] = static_cast<T>(s_temp[23]);
            s_XmatsHom[177] = static_cast<T>(s_temp[11]);
            s_XmatsHom[180] = static_cast<T>(-s_temp[11]);
            s_XmatsHom[181] = static_cast<T>(s_temp[23]);
        }
        __syncthreads();
    }

    /**
     * Updates the (d)XmatsHom in (shared) GPU memory acording to the configuration
     *
     * @param s_XmatsHom is the (shared) memory destination location for the XmatsHom
     * @param s_dXmatsHom is the (shared) memory destination location for the dXmatsHom
     * @param s_q is the (shared) memory location of the current configuration
     * @param s_topology_helpers is the (shared) memory destination location for the topology_helpers
     * @param d_robotModel is the pointer to the initialized model specific helpers (XImats, mxfuncs, topology_helpers, etc.)
     * @param s_temp is temporary (shared) memory used to compute sin and cos if needed of size: 24
     */
    template <typename T>
    __device__
    void load_update_XmatsHom_helpers(T *s_XmatsHom, T *s_dXmatsHom, int *s_topology_helpers, const T *s_q, const robotModel<T> *d_robotModel, T *s_temp) {
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 192; ind += blockDim.x*blockDim.y){
            s_XmatsHom[ind] = d_robotModel->d_XImats[ind+864];
            s_dXmatsHom[ind] = d_robotModel->d_XImats[ind+1056];
        }
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 61; ind += blockDim.x*blockDim.y){
            s_topology_helpers[ind] = d_robotModel->d_topology_helpers[ind];
        }
        for(int k = threadIdx.x + threadIdx.y*blockDim.x; k < 12; k += blockDim.x*blockDim.y){
            s_temp[k] = static_cast<T>(sin(s_q[k]));
            s_temp[k+12] = static_cast<T>(cos(s_q[k]));
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            // X_hom[0]
            s_XmatsHom[1] = static_cast<T>(-s_temp[0]);
            s_XmatsHom[2] = static_cast<T>(-s_temp[12]);
            s_XmatsHom[5] = static_cast<T>(-s_temp[12]);
            s_XmatsHom[6] = static_cast<T>(s_temp[0]);
            // X_hom[1]
            s_XmatsHom[16] = static_cast<T>(s_temp[13]);
            s_XmatsHom[18] = static_cast<T>(s_temp[1]);
            s_XmatsHom[20] = static_cast<T>(-s_temp[1]);
            s_XmatsHom[22] = static_cast<T>(s_temp[13]);
            // X_hom[2]
            s_XmatsHom[32] = static_cast<T>(s_temp[14]);
            s_XmatsHom[33] = static_cast<T>(s_temp[2]);
            s_XmatsHom[36] = static_cast<T>(-s_temp[2]);
            s_XmatsHom[37] = static_cast<T>(s_temp[14]);
            // X_hom[3]
            s_XmatsHom[49] = static_cast<T>(-s_temp[3]);
            s_XmatsHom[50] = static_cast<T>(-s_temp[15]);
            s_XmatsHom[53] = static_cast<T>(-s_temp[15]);
            s_XmatsHom[54] = static_cast<T>(s_temp[3]);
            // X_hom[4]
            s_XmatsHom[64] = static_cast<T>(s_temp[16]);
            s_XmatsHom[66] = static_cast<T>(s_temp[4]);
            s_XmatsHom[68] = static_cast<T>(-s_temp[4]);
            s_XmatsHom[70] = static_cast<T>(s_temp[16]);
            // X_hom[5]
            s_XmatsHom[80] = static_cast<T>(s_temp[17]);
            s_XmatsHom[81] = static_cast<T>(s_temp[5]);
            s_XmatsHom[84] = static_cast<T>(-s_temp[5]);
            s_XmatsHom[85] = static_cast<T>(s_temp[17]);
            // X_hom[6]
            s_XmatsHom[97] = static_cast<T>(s_temp[6]);
            s_XmatsHom[98] = static_cast<T>(-s_temp[18]);
            s_XmatsHom[101] = static_cast<T>(s_temp[18]);
            s_XmatsHom[102] = static_cast<T>(s_temp[6]);
            // X_hom[7]
            s_XmatsHom[112] = static_cast<T>(s_temp[19]);
            s_XmatsHom[114] = static_cast<T>(-s_temp[7]);
            s_XmatsHom[116] = static_cast<T>(-s_temp[7]);
            s_XmatsHom[118] = static_cast<T>(-s_temp[19]);
            // X_hom[8]
            s_XmatsHom[128] = static_cast<T>(s_temp[20]);
            s_XmatsHom[129] = static_cast<T>(s_temp[8]);
            s_XmatsHom[132] = static_cast<T>(-s_temp[8]);
            s_XmatsHom[133] = static_cast<T>(s_temp[20]);
            // X_hom[9]
            s_XmatsHom[145] = static_cast<T>(s_temp[9]);
            s_XmatsHom[146] = static_cast<T>(-s_temp[21]);
            s_XmatsHom[149] = static_cast<T>(s_temp[21]);
            s_XmatsHom[150] = static_cast<T>(s_temp[9]);
            // X_hom[10]
            s_XmatsHom[160] = static_cast<T>(s_temp[22]);
            s_XmatsHom[162] = static_cast<T>(-s_temp[10]);
            s_XmatsHom[164] = static_cast<T>(-s_temp[10]);
            s_XmatsHom[166] = static_cast<T>(-s_temp[22]);
            // X_hom[11]
            s_XmatsHom[176] = static_cast<T>(s_temp[23]);
            s_XmatsHom[177] = static_cast<T>(s_temp[11]);
            s_XmatsHom[180] = static_cast<T>(-s_temp[11]);
            s_XmatsHom[181] = static_cast<T>(s_temp[23]);
            // dX_hom[0]
            s_dXmatsHom[1] = static_cast<T>(-s_temp[12]);
            s_dXmatsHom[2] = static_cast<T>(s_temp[0]);
            s_dXmatsHom[5] = static_cast<T>(s_temp[0]);
            s_dXmatsHom[6] = static_cast<T>(s_temp[12]);
            // dX_hom[1]
            s_dXmatsHom[16] = static_cast<T>(-s_temp[1]);
            s_dXmatsHom[18] = static_cast<T>(s_temp[13]);
            s_dXmatsHom[20] = static_cast<T>(-s_temp[13]);
            s_dXmatsHom[22] = static_cast<T>(-s_temp[1]);
            // dX_hom[2]
            s_dXmatsHom[32] = static_cast<T>(-s_temp[2]);
            s_dXmatsHom[33] = static_cast<T>(s_temp[14]);
            s_dXmatsHom[36] = static_cast<T>(-s_temp[14]);
            s_dXmatsHom[37] = static_cast<T>(-s_temp[2]);
            // dX_hom[3]
            s_dXmatsHom[49] = static_cast<T>(-s_temp[15]);
            s_dXmatsHom[50] = static_cast<T>(s_temp[3]);
            s_dXmatsHom[53] = static_cast<T>(s_temp[3]);
            s_dXmatsHom[54] = static_cast<T>(s_temp[15]);
            // dX_hom[4]
            s_dXmatsHom[64] = static_cast<T>(-s_temp[4]);
            s_dXmatsHom[66] = static_cast<T>(s_temp[16]);
            s_dXmatsHom[68] = static_cast<T>(-s_temp[16]);
            s_dXmatsHom[70] = static_cast<T>(-s_temp[4]);
            // dX_hom[5]
            s_dXmatsHom[80] = static_cast<T>(-s_temp[5]);
            s_dXmatsHom[81] = static_cast<T>(s_temp[17]);
            s_dXmatsHom[84] = static_cast<T>(-s_temp[17]);
            s_dXmatsHom[85] = static_cast<T>(-s_temp[5]);
            // dX_hom[6]
            s_dXmatsHom[97] = static_cast<T>(s_temp[18]);
            s_dXmatsHom[98] = static_cast<T>(s_temp[6]);
            s_dXmatsHom[101] = static_cast<T>(-s_temp[6]);
            s_dXmatsHom[102] = static_cast<T>(s_temp[18]);
            // dX_hom[7]
            s_dXmatsHom[112] = static_cast<T>(-s_temp[7]);
            s_dXmatsHom[114] = static_cast<T>(-s_temp[19]);
            s_dXmatsHom[116] = static_cast<T>(-s_temp[19]);
            s_dXmatsHom[118] = static_cast<T>(s_temp[7]);
            // dX_hom[8]
            s_dXmatsHom[128] = static_cast<T>(-s_temp[8]);
            s_dXmatsHom[129] = static_cast<T>(s_temp[20]);
            s_dXmatsHom[132] = static_cast<T>(-s_temp[20]);
            s_dXmatsHom[133] = static_cast<T>(-s_temp[8]);
            // dX_hom[9]
            s_dXmatsHom[145] = static_cast<T>(s_temp[21]);
            s_dXmatsHom[146] = static_cast<T>(s_temp[9]);
            s_dXmatsHom[149] = static_cast<T>(-s_temp[9]);
            s_dXmatsHom[150] = static_cast<T>(s_temp[21]);
            // dX_hom[10]
            s_dXmatsHom[160] = static_cast<T>(-s_temp[10]);
            s_dXmatsHom[162] = static_cast<T>(-s_temp[22]);
            s_dXmatsHom[164] = static_cast<T>(-s_temp[22]);
            s_dXmatsHom[166] = static_cast<T>(s_temp[10]);
            // dX_hom[11]
            s_dXmatsHom[176] = static_cast<T>(-s_temp[11]);
            s_dXmatsHom[177] = static_cast<T>(s_temp[23]);
            s_dXmatsHom[180] = static_cast<T>(-s_temp[23]);
            s_dXmatsHom[181] = static_cast<T>(-s_temp[11]);
        }
        __syncthreads();
    }

    /**
     * Computes the End Effector Position
     *
     * Notes:
     *   Assumes the Xhom matricies have already been updated for the given q
     *
     * @param s_eePos is a pointer to shared memory of size 6*NUM_EE where NUM_EE = 4
     * @param s_q is the vector of joint positions
     * @param s_Xhom is the pointer to the homogenous transformation matricies 
     * @param s_topology_helpers is the (shared) memory destination location for the topology_helpers
     * @param s_temp is a pointer to helper shared memory of size 128
     */
    template <typename T>
    __device__
    void end_effector_positions_inner(T *s_eePos, const T *s_q, const T *s_Xhom, int *s_topology_helpers, T *s_temp) {
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("q\n"); printMat<T,1,12>(s_q,1);
            for (int i = 0; i < 12; i++){printf("X[%d]\n",i); printMat<T,4,4>(&s_Xhom[16*i],4);}
        }
        __syncthreads();
        //
        // For each branch in parallel chain up the transform
        // Keep chaining until reaching the root (starting from the leaves)
        //
        // First set to leaf transform
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 64; ind += blockDim.x*blockDim.y){
            int rc = ind % 16;
            // non-branching pointer selector
            int eeInd = (ind < 16) * 2 + (ind < 32 && ind >= 16) * 5 + (ind < 48 && ind >= 32) * 8 + (ind >= 48) * 11;
            s_temp[ind] = s_Xhom[16*eeInd + rc];
        }
        __syncthreads();
        // Update with parent transform until you reach the base [level 1/2]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 64; ind += blockDim.x*blockDim.y){
            int row = ind % 4; int col = (ind / 4) % 4; int eeOffset = ind - (ind % 16);
            // non-branching pointer selector
            int parent_jid = (ind < 16) * 1 + (ind < 32 && ind >= 16) * 4 + (ind < 48 && ind >= 32) * 7 + (ind >= 48) * 10;
            s_temp[ind + 64] = dot_prod<T,4,4,1>(&s_Xhom[16*parent_jid + row], &s_temp[0 + eeOffset + 4*col]);
        }
        __syncthreads();
        // Update with parent transform until you reach the base [level 2/2]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 64; ind += blockDim.x*blockDim.y){
            int row = ind % 4; int col = (ind / 4) % 4; int eeOffset = ind - (ind % 16);
            // non-branching pointer selector
            int parent_jid = (ind < 16) * 0 + (ind < 32 && ind >= 16) * 3 + (ind < 48 && ind >= 32) * 6 + (ind >= 48) * 9;
            s_temp[ind + 0] = dot_prod<T,4,4,1>(&s_Xhom[16*parent_jid + row], &s_temp[64 + eeOffset + 4*col]);
        }
        __syncthreads();
        //
        // Now extract the eePos from the Tansforms
        // TODO: ADD OFFSETS
        //
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
            // xyz is easy
            int xyzInd = ind % 3; int eeInd = ind / 3; T *s_Xmat_hom = &s_temp[0 + 16*eeInd];
            s_eePos[6*eeInd + xyzInd] = s_Xmat_hom[12 + xyzInd];
            // roll pitch yaw is a bit more difficult
            if(xyzInd > 0){continue;}
            s_eePos[6*eeInd + 3] = atan2(s_Xmat_hom[6],s_Xmat_hom[10]);
            s_eePos[6*eeInd + 4] = -atan2(s_Xmat_hom[2],sqrt(s_Xmat_hom[6]*s_Xmat_hom[6] + s_Xmat_hom[10]*s_Xmat_hom[10]));
            s_eePos[6*eeInd + 5] = atan2(s_Xmat_hom[1],s_Xmat_hom[0]);
        }
        __syncthreads();
    }

    /**
     * Computes the End Effector Position
     *
     * @param s_eePos is a pointer to shared memory of size 6*NUM_EE where NUM_EE = 4
     * @param s_q is the vector of joint positions
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     */
    template <typename T>
    __device__
    void end_effector_positions_device(T *s_eePos, const T *s_q, const robotModel<T> *d_robotModel) {
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XHomTemp[]; T *s_XmatsHom = s_XHomTemp; T *s_temp = &s_XHomTemp[192];
        load_update_XmatsHom_helpers<T>(s_XmatsHom, s_topology_helpers, s_q, d_robotModel, s_temp);
        end_effector_positions_inner<T>(s_eePos, s_q, s_XmatsHom, s_topology_helpers, s_temp);
    }

    /**
     * Compute the End Effector Position
     *
     * @param d_eePos is the vector of end effector positions
     * @param d_q is the vector of joint positions
     * @param stride_q is the stide between each q
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void end_effector_positions_kernel_single_timing(T *d_eePos, const T *d_q, const int stride_q, const robotModel<T> *d_robotModel, const int NUM_TIMESTEPS) {
        __shared__ T s_q[12];
        __shared__ T s_eePos[24];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XHomTemp[]; T *s_XmatsHom = s_XHomTemp; T *s_temp = &s_XHomTemp[192];
        // load to shared mem
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
            s_q[ind] = d_q[ind];
        }
        __syncthreads();
        // compute with NUM_TIMESTEPS as NUM_REPS for timing
        for (int rep = 0; rep < NUM_TIMESTEPS; rep++){
            load_update_XmatsHom_helpers<T>(s_XmatsHom, s_topology_helpers, s_q, d_robotModel, s_temp);
            end_effector_positions_inner<T>(s_eePos, s_q, s_XmatsHom, s_topology_helpers, s_temp);
        }
        // save down to global
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            d_eePos[ind] = s_eePos[ind];
        }
        __syncthreads();
    }

    /**
     * Compute the End Effector Position
     *
     * @param d_eePos is the vector of end effector positions
     * @param d_q is the vector of joint positions
     * @param stride_q is the stide between each q
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void end_effector_positions_kernel(T *d_eePos, const T *d_q, const int stride_q, const robotModel<T> *d_robotModel, const int NUM_TIMESTEPS) {
        __shared__ T s_q[12];
        __shared__ T s_eePos[24];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XHomTemp[]; T *s_XmatsHom = s_XHomTemp; T *s_temp = &s_XHomTemp[192];
        for(int k = blockIdx.x + blockIdx.y*gridDim.x; k < NUM_TIMESTEPS; k += gridDim.x*gridDim.y){
            // load to shared mem
            const T *d_q_k = &d_q[k*stride_q];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
                s_q[ind] = d_q_k[ind];
            }
            __syncthreads();
            // compute
            load_update_XmatsHom_helpers<T>(s_XmatsHom, s_topology_helpers, s_q, d_robotModel, s_temp);
            end_effector_positions_inner<T>(s_eePos, s_q, s_XmatsHom, s_topology_helpers, s_temp);
            __syncthreads();
            // save down to global
            T *d_eePos_k = &d_eePos[k*24];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
                d_eePos_k[ind] = s_eePos[ind];
            }
            __syncthreads();
        }
    }

    /**
     * Compute the End Effector Positions
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_COMPRESSED_MEM = false>
    __host__
    void end_effector_positions(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const int num_timesteps,
                                const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        // start code with memory transfer
        int stride_q;
        if (USE_COMPRESSED_MEM) {stride_q = NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q,hd_data->h_q,stride_q*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        else {stride_q = 3*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        gpuErrchk(cudaDeviceSynchronize());
        // then call the kernel
        if (USE_COMPRESSED_MEM) {end_effector_positions_kernel<T><<<block_dimms,thread_dimms,EE_POS_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_eePos,hd_data->d_q,stride_q,d_robotModel,num_timesteps);}
        else                    {end_effector_positions_kernel<T><<<block_dimms,thread_dimms,EE_POS_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_eePos,hd_data->d_q_qd_u,stride_q,d_robotModel,num_timesteps);}
        gpuErrchk(cudaDeviceSynchronize());
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_eePos,hd_data->d_eePos,6*NUM_EES*num_timesteps*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Compute the End Effector Positions
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_COMPRESSED_MEM = false>
    __host__
    void end_effector_positions_single_timing(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const int num_timesteps,
                                              const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        // start code with memory transfer
        int stride_q;
        if (USE_COMPRESSED_MEM) {stride_q = NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q,hd_data->h_q,stride_q*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        else {stride_q = 3*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        gpuErrchk(cudaDeviceSynchronize());
        // then call the kernel
        struct timespec start, end; clock_gettime(CLOCK_MONOTONIC,&start);
        if (USE_COMPRESSED_MEM) {end_effector_positions_kernel_single_timing<T><<<block_dimms,thread_dimms,EE_POS_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_eePos,hd_data->d_q,stride_q,d_robotModel,num_timesteps);}
        else                    {end_effector_positions_kernel_single_timing<T><<<block_dimms,thread_dimms,EE_POS_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_eePos,hd_data->d_q_qd_u,stride_q,d_robotModel,num_timesteps);}
        gpuErrchk(cudaDeviceSynchronize());
        clock_gettime(CLOCK_MONOTONIC,&end);
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_eePos,hd_data->d_eePos,6*NUM_EES*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
        printf("Single Call EEPOS %fus\n",time_delta_us_timespec(start,end)/static_cast<double>(num_timesteps));
    }

    /**
     * Compute the End Effector Positions
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_COMPRESSED_MEM = false>
    __host__
    void end_effector_positions_compute_only(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const int num_timesteps,
                                             const dim3 block_dimms, const dim3 thread_dimms) {
        int stride_q = USE_COMPRESSED_MEM ? NUM_JOINTS: 3*NUM_JOINTS;
        // then call the kernel
        if (USE_COMPRESSED_MEM) {end_effector_positions_kernel<T><<<block_dimms,thread_dimms,EE_POS_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_eePos,hd_data->d_q,stride_q,d_robotModel,num_timesteps);}
        else                    {end_effector_positions_kernel<T><<<block_dimms,thread_dimms,EE_POS_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_eePos,hd_data->d_q_qd_u,stride_q,d_robotModel,num_timesteps);}
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Computes the Gradient of the End Effector Position with respect to joint position
     *
     * Notes:
     *   Assumes the Xhom and dXhom matricies have already been updated for the given q
     *
     * @param s_deePos is a pointer to shared memory of size 6*NUM_JOINTS*NUM_EE where NUM_JOINTS = 12 and NUM_EE = 4
     * @param s_q is the vector of joint positions
     * @param s_Xhom is the pointer to the homogenous transformation matricies 
     * @param s_dXhom is the pointer to the gradient of the homogenous transformation matricies 
     * @param s_topology_helpers is the (shared) memory destination location for the topology_helpers
     * @param s_temp is a pointer to helper shared memory of size 3072
     */
    template <typename T>
    __device__
    void end_effector_positions_gradient_inner(T *s_deePos, const T *s_q, const T *s_Xhom, const T *s_dXhom, int *s_topology_helpers, T *s_temp) {
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("q\n"); printMat<T,1,12>(s_q,1);
            for (int i = 0; i < 12; i++){printf("X[%d]\n",i); printMat<T,4,4>(&s_Xhom[16*i],4);}
            for (int i = 0; i < 12; i++){printf("dX[%d]\n",i); printMat<T,4,4>(&s_dXhom[16*i],4);}
        }
        __syncthreads();
        //
        // For each branch/gradient in parallel chain up the transform
        // Keep chaining until reaching the root (starting from the leaves)
        //
        T *s_eeTemp = &s_temp[0]; T *s_deeTemp = &s_temp[1536];
        // First set the leaf transforms for eePos and deePos
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 768; ind += blockDim.x*blockDim.y){
            int rc = ind % 16; int djid = (ind / 16) % 12;
            // branch to get pointer locations
            int eeInd; bool inChain;
                 if (ind < 192){ eeInd = 2; inChain = ((djid == 0) || (djid == 1) || (djid == 2)); }
            else if (ind < 384){ eeInd = 5; inChain = ((djid == 3) || (djid == 4) || (djid == 5)); }
            else if (ind < 576){ eeInd = 8; inChain = ((djid == 6) || (djid == 7) || (djid == 8)); }
            else              { eeInd = 11; inChain = ((djid == 9) || (djid == 10) || (djid == 11)); }
            s_eeTemp[ind] = s_Xhom[16*eeInd + rc];
            s_deeTemp[ind] = inChain * ((djid == eeInd) ? s_dXhom[16*eeInd + rc] : s_Xhom[16*eeInd + rc]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            for (int i = 0; i < 48; i++){printf("X_chain level[%d] with dj_ee_id [%d]\n",0,i); printMat<T,4,4>(&s_eeTemp[16*i],4);}
            for (int i = 0; i < 48; i++){printf("dX_chain level[%d] with dj_ee_id [%d]\n",0,i); printMat<T,4,4>(&s_deeTemp[16*i],4);}
        }
        __syncthreads();
        // Update with parent transform until you reach the base [level 1/2]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 768; ind += blockDim.x*blockDim.y){
            int rc = ind % 16; int djid = (ind / 16) % 12;
            int row = rc % 4; int colInd = ind - row;
            // non-branching pointer selector
            int parent_jid = (ind < 192) * 1 + (ind < 384 && ind >= 192) * 4 + (ind < 576 && ind >= 384) * 7 + (ind >= 576) * 10;
            s_eeTemp[ind + 768] = dot_prod<T,4,4,1>(&s_Xhom[16*parent_jid + row], &s_eeTemp[0 + colInd]);
            const T *s_Xhom_dXhom = ((djid == parent_jid) ? s_dXhom : s_Xhom);
            s_deeTemp[ind + 768] = dot_prod<T,4,4,1>(&s_Xhom_dXhom[16*parent_jid + row], &s_deeTemp[0 + colInd]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            for (int i = 0; i < 48; i++){printf("X_chain[%d]\n",i); printMat<T,4,4>(&s_eeTemp[16*i + 768],4);}
            for (int i = 0; i < 48; i++){printf("dX_chain[%d]\n",i); printMat<T,4,4>(&s_deeTemp[16*i + 768],4);}
        }
        __syncthreads();
        // Update with parent transform until you reach the base [level 2/2]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 768; ind += blockDim.x*blockDim.y){
            int rc = ind % 16; int djid = (ind / 16) % 12;
            int row = rc % 4; int colInd = ind - row;
            // non-branching pointer selector
            int parent_jid = (ind < 192) * 0 + (ind < 384 && ind >= 192) * 3 + (ind < 576 && ind >= 384) * 6 + (ind >= 576) * 9;
            s_eeTemp[ind + 0] = dot_prod<T,4,4,1>(&s_Xhom[16*parent_jid + row], &s_eeTemp[768 + colInd]);
            const T *s_Xhom_dXhom = ((djid == parent_jid) ? s_dXhom : s_Xhom);
            s_deeTemp[ind + 0] = dot_prod<T,4,4,1>(&s_Xhom_dXhom[16*parent_jid + row], &s_deeTemp[768 + colInd]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            for (int i = 0; i < 48; i++){printf("X_chain[%d]\n",i); printMat<T,4,4>(&s_eeTemp[16*i + 0],4);}
            for (int i = 0; i < 48; i++){printf("dX_chain[%d]\n",i); printMat<T,4,4>(&s_deeTemp[16*i + 0],4);}
        }
        __syncthreads();
        //
        // Now extract the eePos from the Tansforms
        // TODO: ADD OFFSETS
        //
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
            int outputInd = ind % 6; int deeInd = ind / 6;
            T *s_Xmat_hom = &s_eeTemp[0 + 16*deeInd]; T *s_dXmat_hom = &s_deeTemp[0 + 16*deeInd];
            // xyz is easy
            if (outputInd < 3){s_deePos[6*deeInd + outputInd] = s_dXmat_hom[12 + outputInd];}
            // roll pitch yaw is a bit more difficult
            // note: d/dz of arctan2(y(z),x(z)) = [-x'(z)y(z)+x(z)y'(z)]/[(x(z)^2 + y(z)^2)]
            // Also note that d/dz of sqrt(f(z)) = f'(z)/2sqrt(f(z))
            else {
                // simpler to recompute
                T sqrtTerm = sqrt(s_Xmat_hom[10]*s_Xmat_hom[10] + s_Xmat_hom[6]*s_Xmat_hom[6]);
                T dsqrtTerm = (s_Xmat_hom[10]*s_dXmat_hom[10] + s_Xmat_hom[6]*s_dXmat_hom[6])/sqrtTerm;
                // branch to get pointer locations
                T y; T x; T y_prime; T x_prime;
                     if (outputInd == 3){ y = s_Xmat_hom[6]; x = s_Xmat_hom[10]; y_prime = s_dXmat_hom[6]; x_prime = s_dXmat_hom[10]; }
                else if (outputInd == 4){ y = -s_Xmat_hom[2]; x = sqrtTerm; y_prime = -s_dXmat_hom[2]; x_prime = dsqrtTerm; }
                else              { y = s_Xmat_hom[1]; x = s_Xmat_hom[0]; y_prime = s_dXmat_hom[1]; x_prime = s_dXmat_hom[0]; }
                s_deePos[6*deeInd + outputInd] = (-x_prime*y + x*y_prime)/(x*x + y*y);
            }
        }
        __syncthreads();
    }

    /**
     * Computes the Gradient of the End Effector Position with respect to joint position
     *
     * @param s_deePos is a pointer to shared memory of size 6*NUM_JOINTS*NUM_EE where NUM_JOINTS = 12 and NUM_EE = 4
     * @param s_q is the vector of joint positions
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     */
    template <typename T>
    __device__
    void end_effector_positions_gradient_device(T *s_deePos, const T *s_q, const robotModel<T> *d_robotModel) {
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XHomTemp[]; T *s_XmatsHom = s_XHomTemp; T *s_dXmatsHom = &s_XHomTemp[192]; T *s_temp = &s_dXmatsHom[192];
        load_update_XmatsHom_helpers<T>(s_XmatsHom, s_dXmatsHom, s_topology_helpers, s_q, d_robotModel, s_temp);
        end_effector_positions_gradient_inner<T>(s_deePos, s_q, s_XmatsHom, s_dXmatsHom, s_topology_helpers, s_temp);
    }

    /**
     * Computes the Gradient of the End Effector Position with respect to joint position
     *
     * @param d_deePos is the vector of end effector positions gradients
     * @param d_q is the vector of joint positions
     * @param stride_q is the stide between each q
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void end_effector_positions_gradient_kernel_single_timing(T *d_deePos, const T *d_q, const int stride_q, const robotModel<T> *d_robotModel, const int NUM_TIMESTEPS) {
        __shared__ T s_q[12];
        __shared__ T s_deePos[288];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XHomTemp[]; T *s_XmatsHom = s_XHomTemp; T *s_dXmatsHom = &s_XHomTemp[192]; T *s_temp = &s_dXmatsHom[192];
        // load to shared mem
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
            s_q[ind] = d_q[ind];
        }
        __syncthreads();
        // compute with NUM_TIMESTEPS as NUM_REPS for timing
        for (int rep = 0; rep < NUM_TIMESTEPS; rep++){
            load_update_XmatsHom_helpers<T>(s_XmatsHom, s_dXmatsHom, s_topology_helpers, s_q, d_robotModel, s_temp);
            end_effector_positions_gradient_inner<T>(s_deePos, s_q, s_XmatsHom, s_dXmatsHom, s_topology_helpers, s_temp);
        }
        // save down to global
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
            d_deePos[ind] = s_deePos[ind];
        }
        __syncthreads();
    }

    /**
     * Computes the Gradient of the End Effector Position with respect to joint position
     *
     * @param d_deePos is the vector of end effector positions gradients
     * @param d_q is the vector of joint positions
     * @param stride_q is the stide between each q
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void end_effector_positions_gradient_kernel(T *d_deePos, const T *d_q, const int stride_q, const robotModel<T> *d_robotModel, const int NUM_TIMESTEPS) {
        __shared__ T s_q[12];
        __shared__ T s_deePos[288];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XHomTemp[]; T *s_XmatsHom = s_XHomTemp; T *s_dXmatsHom = &s_XHomTemp[192]; T *s_temp = &s_dXmatsHom[192];
        for(int k = blockIdx.x + blockIdx.y*gridDim.x; k < NUM_TIMESTEPS; k += gridDim.x*gridDim.y){
            // load to shared mem
            const T *d_q_k = &d_q[k*stride_q];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
                s_q[ind] = d_q_k[ind];
            }
            __syncthreads();
            // compute
            load_update_XmatsHom_helpers<T>(s_XmatsHom, s_dXmatsHom, s_topology_helpers, s_q, d_robotModel, s_temp);
            end_effector_positions_gradient_inner<T>(s_deePos, s_q, s_XmatsHom, s_dXmatsHom, s_topology_helpers, s_temp);
            __syncthreads();
            // save down to global
            T *d_deePos_k = &d_deePos[k*288];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
                d_deePos_k[ind] = s_deePos[ind];
            }
            __syncthreads();
        }
    }

    /**
     * Computes the Gradient of the End Effector Position with respect to joint position
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_COMPRESSED_MEM = false>
    __host__
    void end_effector_positions_gradient(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const int num_timesteps,
                                const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        // start code with memory transfer
        int stride_q;
        if (USE_COMPRESSED_MEM) {stride_q = NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q,hd_data->h_q,stride_q*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        else {stride_q = 3*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        gpuErrchk(cudaDeviceSynchronize());
        // then call the kernel
        if (USE_COMPRESSED_MEM) {end_effector_positions_gradient_kernel<T><<<block_dimms,thread_dimms,DEE_POS_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_deePos,hd_data->d_q,stride_q,d_robotModel,num_timesteps);}
        else                    {end_effector_positions_gradient_kernel<T><<<block_dimms,thread_dimms,DEE_POS_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_deePos,hd_data->d_q_qd_u,stride_q,d_robotModel,num_timesteps);}
        gpuErrchk(cudaDeviceSynchronize());
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_deePos,hd_data->d_deePos,6*NUM_EES*NUM_JOINTS*num_timesteps*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Computes the Gradient of the End Effector Position with respect to joint position
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_COMPRESSED_MEM = false>
    __host__
    void end_effector_positions_gradient_single_timing(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const int num_timesteps,
                                              const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        // start code with memory transfer
        int stride_q;
        if (USE_COMPRESSED_MEM) {stride_q = NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q,hd_data->h_q,stride_q*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        else {stride_q = 3*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        gpuErrchk(cudaDeviceSynchronize());
        // then call the kernel
        struct timespec start, end; clock_gettime(CLOCK_MONOTONIC,&start);
        if (USE_COMPRESSED_MEM) {end_effector_positions_gradient_kernel_single_timing<T><<<block_dimms,thread_dimms,DEE_POS_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_deePos,hd_data->d_q,stride_q,d_robotModel,num_timesteps);}
        else                    {end_effector_positions_gradient_kernel_single_timing<T><<<block_dimms,thread_dimms,DEE_POS_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_deePos,hd_data->d_q_qd_u,stride_q,d_robotModel,num_timesteps);}
        gpuErrchk(cudaDeviceSynchronize());
        clock_gettime(CLOCK_MONOTONIC,&end);
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_deePos,hd_data->d_deePos,6*NUM_EES*NUM_JOINTS*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
        printf("Single Call DEEPOS %fus\n",time_delta_us_timespec(start,end)/static_cast<double>(num_timesteps));
    }

    /**
     * Computes the Gradient of the End Effector Position with respect to joint position
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_COMPRESSED_MEM = false>
    __host__
    void end_effector_positions_gradient_compute_only(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const int num_timesteps,
                                             const dim3 block_dimms, const dim3 thread_dimms) {
        int stride_q = USE_COMPRESSED_MEM ? NUM_JOINTS: 3*NUM_JOINTS;
        // then call the kernel
        if (USE_COMPRESSED_MEM) {end_effector_positions_gradient_kernel<T><<<block_dimms,thread_dimms,DEE_POS_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_deePos,hd_data->d_q,stride_q,d_robotModel,num_timesteps);}
        else                    {end_effector_positions_gradient_kernel<T><<<block_dimms,thread_dimms,DEE_POS_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_deePos,hd_data->d_q_qd_u,stride_q,d_robotModel,num_timesteps);}
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * Notes:
     *   Assumes the XI matricies have already been updated for the given q
     *
     * @param s_c is the vector of output torques
     * @param s_vaf is a pointer to shared memory of size 3*6*NUM_JOINTS = 216
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param s_qdd is (optional vector of joint accelerations
     * @param s_XI is the pointer to the transformation and inertia matricies 
     * @param s_XImats is the (shared) memory holding the updated XI matricies for the given s_q
     * @param s_topology_helpers is the (shared) memory destination location for the topology_helpers
     * @param s_temp is a pointer to helper shared memory of size 6*NUM_JOINTS = 72
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void inverse_dynamics_inner(T *s_c,  T *s_vaf, const T *s_q, const T *s_qd, const T *s_qdd, T *s_XImats, int *s_topology_helpers, T *s_temp, const T gravity) {
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("q\n"); printMat<T,1,12>(s_q,1);
            printf("qd\n"); printMat<T,1,12>(s_qd,1);
            printf("qdd\n"); printMat<T,1,6>(s_qdd,1);
            for (int i = 0; i < 12; i++){printf("X[%d]\n",i); printMat<T,6,6>(&s_XImats[36*i],6);}
            for (int i = 0; i < 12; i++){printf("I[%d]\n",i); printMat<T,6,6>(&s_XImats[36*(i+12)],6);}
        }
        __syncthreads();
        //
        // Forward Pass
        //
        // s_v, s_a where parent is base
        //     joints are: lf_haa_joint, lh_haa_joint, rf_haa_joint, rh_haa_joint
        //     links are: lf_hipassembly, lh_hipassembly, rf_hipassembly, rh_hipassembly
        // s_v[k] = S[k]*qd[k] and s_a[k] = X[k]*gravityS[k]*qdd[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 0 + (ind < 12 && ind >= 6) * 3 + (ind < 18 && ind >= 12) * 6 + (ind >= 18) * 9;
            int jid6 = 6*jid;
            s_vaf[jid6 + row] = static_cast<T>(0);
            s_vaf[72 + jid6 + row] = s_XImats[6*jid6 + 30 + row]*gravity;
            if (row == 2){s_vaf[jid6 + 2] += s_qd[jid]; s_vaf[72 + jid6 + 2] += s_qdd[jid];}
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_v[0]\n"); printMat<T,1,6>(&s_vaf[6*0],1);
            printf("s_a[0]\n"); printMat<T,1,6>(&s_vaf[72 + 6*0],1);
            printf("s_v[3]\n"); printMat<T,1,6>(&s_vaf[6*3],1);
            printf("s_a[3]\n"); printMat<T,1,6>(&s_vaf[72 + 6*3],1);
            printf("s_v[6]\n"); printMat<T,1,6>(&s_vaf[6*6],1);
            printf("s_a[6]\n"); printMat<T,1,6>(&s_vaf[72 + 6*6],1);
            printf("s_v[9]\n"); printMat<T,1,6>(&s_vaf[6*9],1);
            printf("s_a[9]\n"); printMat<T,1,6>(&s_vaf[72 + 6*9],1);
        }
        __syncthreads();
        // s_v and s_a where bfs_level is 1
        //     joints are: lf_hfe_joint, lh_hfe_joint, rf_hfe_joint, rh_hfe_joint
        //     links are: lf_upperleg, lh_upperleg, rf_upperleg, rh_upperleg
        // s_v[k] = X[k]*v[parent_k] + S[k]*qd[k] and s_a[k] = X[k]*a[parent_k] + S[k]*qdd[k] + mxS[k](v[k])*qd[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 48; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int comp_mod = comp % 4; int vFlag = comp == comp_mod;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 1 + (comp_mod == 1) * 4 + (comp_mod == 2) * 7 + (comp_mod == 3) * 10;
            int vaOffset = !vFlag * 72; int jid6 = 6 * jid;
            T qd_qdd_val = (row == 2) * (vFlag * s_qd[jid] + !vFlag * s_qdd[jid]);
            // compute based on the branch and use bool multiply for no branch
            s_vaf[vaOffset + jid6 + row] = dot_prod<T,6,6,1>(&s_XImats[6*jid6 + row], &s_vaf[vaOffset + 6*s_topology_helpers[jid]]) + qd_qdd_val;
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_v[1] = X*s_v[0] + S*qd[1]\n"); printMat<T,1,6>(&s_vaf[6*1],1);
            printf("s_a[1] = X*s_a[0] + S*qdd[1]\n"); printMat<T,1,6>(&s_vaf[72 + 6*1],1);
            printf("s_v[4] = X*s_v[3] + S*qd[4]\n"); printMat<T,1,6>(&s_vaf[6*4],1);
            printf("s_a[4] = X*s_a[3] + S*qdd[4]\n"); printMat<T,1,6>(&s_vaf[72 + 6*4],1);
            printf("s_v[7] = X*s_v[6] + S*qd[7]\n"); printMat<T,1,6>(&s_vaf[6*7],1);
            printf("s_a[7] = X*s_a[6] + S*qdd[7]\n"); printMat<T,1,6>(&s_vaf[72 + 6*7],1);
            printf("s_v[10] = X*s_v[9] + S*qd[10]\n"); printMat<T,1,6>(&s_vaf[6*10],1);
            printf("s_a[10] = X*s_a[9] + S*qdd[10]\n"); printMat<T,1,6>(&s_vaf[72 + 6*10],1);
        }
        __syncthreads();
        // sync before a += MxS(v)*qd[S] 
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 4; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind == 0) * 1 + (ind == 1) * 4 + (ind == 2) * 7 + (ind == 3) * 10;
            mx2_peq_scaled<T>(&s_vaf[72 + 6*jid], &s_vaf[6*jid], s_qd[jid]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_a[1] += MxS(s_v[1])\n"); printMat<T,1,6>(&s_vaf[72 + 6*1],1);
            printf("s_a[4] += MxS(s_v[4])\n"); printMat<T,1,6>(&s_vaf[72 + 6*4],1);
            printf("s_a[7] += MxS(s_v[7])\n"); printMat<T,1,6>(&s_vaf[72 + 6*7],1);
            printf("s_a[10] += MxS(s_v[10])\n"); printMat<T,1,6>(&s_vaf[72 + 6*10],1);
        }
        __syncthreads();
        // s_v and s_a where bfs_level is 2
        //     joints are: lf_kfe_joint, lh_kfe_joint, rf_kfe_joint, rh_kfe_joint
        //     links are: lf_lowerleg, lh_lowerleg, rf_lowerleg, rh_lowerleg
        // s_v[k] = X[k]*v[parent_k] + S[k]*qd[k] and s_a[k] = X[k]*a[parent_k] + S[k]*qdd[k] + mxS[k](v[k])*qd[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 48; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int comp_mod = comp % 4; int vFlag = comp == comp_mod;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 2 + (comp_mod == 1) * 5 + (comp_mod == 2) * 8 + (comp_mod == 3) * 11;
            int vaOffset = !vFlag * 72; int jid6 = 6 * jid;
            T qd_qdd_val = (row == 2) * (vFlag * s_qd[jid] + !vFlag * s_qdd[jid]);
            // compute based on the branch and use bool multiply for no branch
            s_vaf[vaOffset + jid6 + row] = dot_prod<T,6,6,1>(&s_XImats[6*jid6 + row], &s_vaf[vaOffset + 6*s_topology_helpers[jid]]) + qd_qdd_val;
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_v[2] = X*s_v[1] + S*qd[2]\n"); printMat<T,1,6>(&s_vaf[6*2],1);
            printf("s_a[2] = X*s_a[1] + S*qdd[2]\n"); printMat<T,1,6>(&s_vaf[72 + 6*2],1);
            printf("s_v[5] = X*s_v[4] + S*qd[5]\n"); printMat<T,1,6>(&s_vaf[6*5],1);
            printf("s_a[5] = X*s_a[4] + S*qdd[5]\n"); printMat<T,1,6>(&s_vaf[72 + 6*5],1);
            printf("s_v[8] = X*s_v[7] + S*qd[8]\n"); printMat<T,1,6>(&s_vaf[6*8],1);
            printf("s_a[8] = X*s_a[7] + S*qdd[8]\n"); printMat<T,1,6>(&s_vaf[72 + 6*8],1);
            printf("s_v[11] = X*s_v[10] + S*qd[11]\n"); printMat<T,1,6>(&s_vaf[6*11],1);
            printf("s_a[11] = X*s_a[10] + S*qdd[11]\n"); printMat<T,1,6>(&s_vaf[72 + 6*11],1);
        }
        __syncthreads();
        // sync before a += MxS(v)*qd[S] 
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 4; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind == 0) * 2 + (ind == 1) * 5 + (ind == 2) * 8 + (ind == 3) * 11;
            mx2_peq_scaled<T>(&s_vaf[72 + 6*jid], &s_vaf[6*jid], s_qd[jid]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_a[2] += MxS(s_v[2])\n"); printMat<T,1,6>(&s_vaf[72 + 6*2],1);
            printf("s_a[5] += MxS(s_v[5])\n"); printMat<T,1,6>(&s_vaf[72 + 6*5],1);
            printf("s_a[8] += MxS(s_v[8])\n"); printMat<T,1,6>(&s_vaf[72 + 6*8],1);
            printf("s_a[11] += MxS(s_v[11])\n"); printMat<T,1,6>(&s_vaf[72 + 6*11],1);
        }
        __syncthreads();
        //
        // s_f in parallel given all v, a
        //
        // s_f[k] = I[k]*a[k] + fx(v[k])*I[k]*v[k]
        // start with s_f[k] = I[k]*a[k] and temp = *I[k]*v[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int jid = comp % 12;
            bool IaFlag = comp == jid; int jid6 = 6*jid; int vaOffset = IaFlag * 72 + jid6;
            T *dst = IaFlag ? &s_vaf[144] : s_temp;
            // compute based on the branch and save Iv to temp to prep for fx(v)*Iv and then sync
            dst[jid6 + row] = dot_prod<T,6,6,1>(&s_XImats[432 + 6*jid6 + row], &s_vaf[vaOffset]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_f[0] = I*s_a[0])\n"); printMat<T,1,6>(&s_vaf[144 + 6*0],1);
            printf("I*s_v[0])\n"); printMat<T,1,6>(&s_temp[6*0],1);
            printf("s_f[1] = I*s_a[1])\n"); printMat<T,1,6>(&s_vaf[144 + 6*1],1);
            printf("I*s_v[1])\n"); printMat<T,1,6>(&s_temp[6*1],1);
            printf("s_f[2] = I*s_a[2])\n"); printMat<T,1,6>(&s_vaf[144 + 6*2],1);
            printf("I*s_v[2])\n"); printMat<T,1,6>(&s_temp[6*2],1);
            printf("s_f[3] = I*s_a[3])\n"); printMat<T,1,6>(&s_vaf[144 + 6*3],1);
            printf("I*s_v[3])\n"); printMat<T,1,6>(&s_temp[6*3],1);
            printf("s_f[4] = I*s_a[4])\n"); printMat<T,1,6>(&s_vaf[144 + 6*4],1);
            printf("I*s_v[4])\n"); printMat<T,1,6>(&s_temp[6*4],1);
            printf("s_f[5] = I*s_a[5])\n"); printMat<T,1,6>(&s_vaf[144 + 6*5],1);
            printf("I*s_v[5])\n"); printMat<T,1,6>(&s_temp[6*5],1);
            printf("s_f[6] = I*s_a[6])\n"); printMat<T,1,6>(&s_vaf[144 + 6*6],1);
            printf("I*s_v[6])\n"); printMat<T,1,6>(&s_temp[6*6],1);
            printf("s_f[7] = I*s_a[7])\n"); printMat<T,1,6>(&s_vaf[144 + 6*7],1);
            printf("I*s_v[7])\n"); printMat<T,1,6>(&s_temp[6*7],1);
            printf("s_f[8] = I*s_a[8])\n"); printMat<T,1,6>(&s_vaf[144 + 6*8],1);
            printf("I*s_v[8])\n"); printMat<T,1,6>(&s_temp[6*8],1);
            printf("s_f[9] = I*s_a[9])\n"); printMat<T,1,6>(&s_vaf[144 + 6*9],1);
            printf("I*s_v[9])\n"); printMat<T,1,6>(&s_temp[6*9],1);
            printf("s_f[10] = I*s_a[10])\n"); printMat<T,1,6>(&s_vaf[144 + 6*10],1);
            printf("I*s_v[10])\n"); printMat<T,1,6>(&s_temp[6*10],1);
            printf("s_f[11] = I*s_a[11])\n"); printMat<T,1,6>(&s_vaf[144 + 6*11],1);
            printf("I*s_v[11])\n"); printMat<T,1,6>(&s_temp[6*11],1);
        }
        __syncthreads();
        // finish with s_f[k] += fx(v[k])*Iv[k]
        for(int jid = threadIdx.x + threadIdx.y*blockDim.x; jid < 12; jid += blockDim.x*blockDim.y){
            int jid6 = 6*jid;
            fx_times_v_peq<T>(&s_vaf[144 + jid6], &s_vaf[jid6], &s_temp[jid6]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_f[0] += fx(v[0])*I*v[0])\n"); printMat<T,1,6>(&s_vaf[144 + 6*0],1);
            printf("s_f[1] += fx(v[1])*I*v[1])\n"); printMat<T,1,6>(&s_vaf[144 + 6*1],1);
            printf("s_f[2] += fx(v[2])*I*v[2])\n"); printMat<T,1,6>(&s_vaf[144 + 6*2],1);
            printf("s_f[3] += fx(v[3])*I*v[3])\n"); printMat<T,1,6>(&s_vaf[144 + 6*3],1);
            printf("s_f[4] += fx(v[4])*I*v[4])\n"); printMat<T,1,6>(&s_vaf[144 + 6*4],1);
            printf("s_f[5] += fx(v[5])*I*v[5])\n"); printMat<T,1,6>(&s_vaf[144 + 6*5],1);
            printf("s_f[6] += fx(v[6])*I*v[6])\n"); printMat<T,1,6>(&s_vaf[144 + 6*6],1);
            printf("s_f[7] += fx(v[7])*I*v[7])\n"); printMat<T,1,6>(&s_vaf[144 + 6*7],1);
            printf("s_f[8] += fx(v[8])*I*v[8])\n"); printMat<T,1,6>(&s_vaf[144 + 6*8],1);
            printf("s_f[9] += fx(v[9])*I*v[9])\n"); printMat<T,1,6>(&s_vaf[144 + 6*9],1);
            printf("s_f[10] += fx(v[10])*I*v[10])\n"); printMat<T,1,6>(&s_vaf[144 + 6*10],1);
            printf("s_f[11] += fx(v[11])*I*v[11])\n"); printMat<T,1,6>(&s_vaf[144 + 6*11],1);
            printf("s_f forward pass\n"); printMat<T,6,12>(&s_vaf[144],6);
        }
        __syncthreads();
        //
        // Backward Pass
        //
        // s_f update where bfs_level is 2
        //     joints are: lf_kfe_joint, lh_kfe_joint, rf_kfe_joint, rh_kfe_joint
        //     links are: lf_lowerleg, lh_lowerleg, rf_lowerleg, rh_lowerleg
        // s_f[parent_k] += X[k]^T*f[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 2 + (ind < 12 && ind >= 6) * 5 + (ind < 18 && ind >= 12) * 8 + (ind >= 18) * 11;
            T val = dot_prod<T,6,1,1>(&s_XImats[36*jid + 6*row], &s_vaf[144 + 6*jid]);
            int dstOffset = 144 + 6*s_topology_helpers[jid] + row;
            s_vaf[dstOffset] += val;
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_f[1] += X^T*s_f[2]\n"); printMat<T,1,6>(&s_vaf[144 + 6*1],1);
            printf("s_f[4] += X^T*s_f[5]\n"); printMat<T,1,6>(&s_vaf[144 + 6*4],1);
            printf("s_f[7] += X^T*s_f[8]\n"); printMat<T,1,6>(&s_vaf[144 + 6*7],1);
            printf("s_f[10] += X^T*s_f[11]\n"); printMat<T,1,6>(&s_vaf[144 + 6*10],1);
        }
        __syncthreads();
        // s_f update where bfs_level is 1
        //     joints are: lf_hfe_joint, lh_hfe_joint, rf_hfe_joint, rh_hfe_joint
        //     links are: lf_upperleg, lh_upperleg, rf_upperleg, rh_upperleg
        // s_f[parent_k] += X[k]^T*f[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 1 + (ind < 12 && ind >= 6) * 4 + (ind < 18 && ind >= 12) * 7 + (ind >= 18) * 10;
            T val = dot_prod<T,6,1,1>(&s_XImats[36*jid + 6*row], &s_vaf[144 + 6*jid]);
            int dstOffset = 144 + 6*s_topology_helpers[jid] + row;
            s_vaf[dstOffset] += val;
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_f[0] += X^T*s_f[1]\n"); printMat<T,1,6>(&s_vaf[144 + 6*0],1);
            printf("s_f[3] += X^T*s_f[4]\n"); printMat<T,1,6>(&s_vaf[144 + 6*3],1);
            printf("s_f[6] += X^T*s_f[7]\n"); printMat<T,1,6>(&s_vaf[144 + 6*6],1);
            printf("s_f[9] += X^T*s_f[10]\n"); printMat<T,1,6>(&s_vaf[144 + 6*9],1);
        }
        __syncthreads();
        //
        // s_c extracted in parallel (S*f)
        //
        for(int dof_id = threadIdx.x + threadIdx.y*blockDim.x; dof_id < 12; dof_id += blockDim.x*blockDim.y){
            s_c[dof_id] = s_vaf[144 + 6*dof_id + 2];
        }
        __syncthreads();
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * Notes:
     *   Assumes the XI matricies have already been updated for the given q
     *   optimized for qdd = 0
     *
     * @param s_c is the vector of output torques
     * @param s_vaf is a pointer to shared memory of size 3*6*NUM_JOINTS = 216
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param s_XI is the pointer to the transformation and inertia matricies 
     * @param s_XImats is the (shared) memory holding the updated XI matricies for the given s_q
     * @param s_topology_helpers is the (shared) memory destination location for the topology_helpers
     * @param s_temp is a pointer to helper shared memory of size 6*NUM_JOINTS = 72
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void inverse_dynamics_inner(T *s_c,  T *s_vaf, const T *s_q, const T *s_qd, T *s_XImats, int *s_topology_helpers, T *s_temp, const T gravity) {
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("q\n"); printMat<T,1,12>(s_q,1);
            printf("qd\n"); printMat<T,1,12>(s_qd,1);
            for (int i = 0; i < 12; i++){printf("X[%d]\n",i); printMat<T,6,6>(&s_XImats[36*i],6);}
            for (int i = 0; i < 12; i++){printf("I[%d]\n",i); printMat<T,6,6>(&s_XImats[36*(i+12)],6);}
        }
        __syncthreads();
        //
        // Forward Pass
        //
        // s_v, s_a where parent is base
        //     joints are: lf_haa_joint, lh_haa_joint, rf_haa_joint, rh_haa_joint
        //     links are: lf_hipassembly, lh_hipassembly, rf_hipassembly, rh_hipassembly
        // s_v[k] = S[k]*qd[k] and s_a[k] = X[k]*gravity
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 0 + (ind < 12 && ind >= 6) * 3 + (ind < 18 && ind >= 12) * 6 + (ind >= 18) * 9;
            int jid6 = 6*jid;
            s_vaf[jid6 + row] = static_cast<T>(0);
            s_vaf[72 + jid6 + row] = s_XImats[6*jid6 + 30 + row]*gravity;
            if (row == 2){s_vaf[jid6 + 2] += s_qd[jid];}
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_v[0]\n"); printMat<T,1,6>(&s_vaf[6*0],1);
            printf("s_a[0]\n"); printMat<T,1,6>(&s_vaf[72 + 6*0],1);
            printf("s_v[3]\n"); printMat<T,1,6>(&s_vaf[6*3],1);
            printf("s_a[3]\n"); printMat<T,1,6>(&s_vaf[72 + 6*3],1);
            printf("s_v[6]\n"); printMat<T,1,6>(&s_vaf[6*6],1);
            printf("s_a[6]\n"); printMat<T,1,6>(&s_vaf[72 + 6*6],1);
            printf("s_v[9]\n"); printMat<T,1,6>(&s_vaf[6*9],1);
            printf("s_a[9]\n"); printMat<T,1,6>(&s_vaf[72 + 6*9],1);
        }
        __syncthreads();
        // s_v and s_a where bfs_level is 1
        //     joints are: lf_hfe_joint, lh_hfe_joint, rf_hfe_joint, rh_hfe_joint
        //     links are: lf_upperleg, lh_upperleg, rf_upperleg, rh_upperleg
        // s_v[k] = X[k]*v[parent_k] + S[k]*qd[k] and s_a[k] = X[k]*a[parent_k] + mxS[k](v[k])*qd[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 48; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int comp_mod = comp % 4; int vFlag = comp == comp_mod;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 1 + (comp_mod == 1) * 4 + (comp_mod == 2) * 7 + (comp_mod == 3) * 10;
            int vaOffset = !vFlag * 72; int jid6 = 6 * jid;
            T qd_qdd_val = (row == 2) * (vFlag * s_qd[jid]);
            // compute based on the branch and use bool multiply for no branch
            s_vaf[vaOffset + jid6 + row] = dot_prod<T,6,6,1>(&s_XImats[6*jid6 + row], &s_vaf[vaOffset + 6*s_topology_helpers[jid]]) + qd_qdd_val;
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_v[1] = X*s_v[0] + S*qd[1]\n"); printMat<T,1,6>(&s_vaf[6*1],1);
            printf("s_a[1] = X*s_a[0]\n"); printMat<T,1,6>(&s_vaf[72 + 6*1],1);
            printf("s_v[4] = X*s_v[3] + S*qd[4]\n"); printMat<T,1,6>(&s_vaf[6*4],1);
            printf("s_a[4] = X*s_a[3]\n"); printMat<T,1,6>(&s_vaf[72 + 6*4],1);
            printf("s_v[7] = X*s_v[6] + S*qd[7]\n"); printMat<T,1,6>(&s_vaf[6*7],1);
            printf("s_a[7] = X*s_a[6]\n"); printMat<T,1,6>(&s_vaf[72 + 6*7],1);
            printf("s_v[10] = X*s_v[9] + S*qd[10]\n"); printMat<T,1,6>(&s_vaf[6*10],1);
            printf("s_a[10] = X*s_a[9]\n"); printMat<T,1,6>(&s_vaf[72 + 6*10],1);
        }
        __syncthreads();
        // sync before a += MxS(v)*qd[S] 
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 4; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind == 0) * 1 + (ind == 1) * 4 + (ind == 2) * 7 + (ind == 3) * 10;
            mx2_peq_scaled<T>(&s_vaf[72 + 6*jid], &s_vaf[6*jid], s_qd[jid]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_a[1] += MxS(s_v[1])\n"); printMat<T,1,6>(&s_vaf[72 + 6*1],1);
            printf("s_a[4] += MxS(s_v[4])\n"); printMat<T,1,6>(&s_vaf[72 + 6*4],1);
            printf("s_a[7] += MxS(s_v[7])\n"); printMat<T,1,6>(&s_vaf[72 + 6*7],1);
            printf("s_a[10] += MxS(s_v[10])\n"); printMat<T,1,6>(&s_vaf[72 + 6*10],1);
        }
        __syncthreads();
        // s_v and s_a where bfs_level is 2
        //     joints are: lf_kfe_joint, lh_kfe_joint, rf_kfe_joint, rh_kfe_joint
        //     links are: lf_lowerleg, lh_lowerleg, rf_lowerleg, rh_lowerleg
        // s_v[k] = X[k]*v[parent_k] + S[k]*qd[k] and s_a[k] = X[k]*a[parent_k] + mxS[k](v[k])*qd[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 48; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int comp_mod = comp % 4; int vFlag = comp == comp_mod;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 2 + (comp_mod == 1) * 5 + (comp_mod == 2) * 8 + (comp_mod == 3) * 11;
            int vaOffset = !vFlag * 72; int jid6 = 6 * jid;
            T qd_qdd_val = (row == 2) * (vFlag * s_qd[jid]);
            // compute based on the branch and use bool multiply for no branch
            s_vaf[vaOffset + jid6 + row] = dot_prod<T,6,6,1>(&s_XImats[6*jid6 + row], &s_vaf[vaOffset + 6*s_topology_helpers[jid]]) + qd_qdd_val;
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_v[2] = X*s_v[1] + S*qd[2]\n"); printMat<T,1,6>(&s_vaf[6*2],1);
            printf("s_a[2] = X*s_a[1]\n"); printMat<T,1,6>(&s_vaf[72 + 6*2],1);
            printf("s_v[5] = X*s_v[4] + S*qd[5]\n"); printMat<T,1,6>(&s_vaf[6*5],1);
            printf("s_a[5] = X*s_a[4]\n"); printMat<T,1,6>(&s_vaf[72 + 6*5],1);
            printf("s_v[8] = X*s_v[7] + S*qd[8]\n"); printMat<T,1,6>(&s_vaf[6*8],1);
            printf("s_a[8] = X*s_a[7]\n"); printMat<T,1,6>(&s_vaf[72 + 6*8],1);
            printf("s_v[11] = X*s_v[10] + S*qd[11]\n"); printMat<T,1,6>(&s_vaf[6*11],1);
            printf("s_a[11] = X*s_a[10]\n"); printMat<T,1,6>(&s_vaf[72 + 6*11],1);
        }
        __syncthreads();
        // sync before a += MxS(v)*qd[S] 
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 4; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind == 0) * 2 + (ind == 1) * 5 + (ind == 2) * 8 + (ind == 3) * 11;
            mx2_peq_scaled<T>(&s_vaf[72 + 6*jid], &s_vaf[6*jid], s_qd[jid]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_a[2] += MxS(s_v[2])\n"); printMat<T,1,6>(&s_vaf[72 + 6*2],1);
            printf("s_a[5] += MxS(s_v[5])\n"); printMat<T,1,6>(&s_vaf[72 + 6*5],1);
            printf("s_a[8] += MxS(s_v[8])\n"); printMat<T,1,6>(&s_vaf[72 + 6*8],1);
            printf("s_a[11] += MxS(s_v[11])\n"); printMat<T,1,6>(&s_vaf[72 + 6*11],1);
        }
        __syncthreads();
        //
        // s_f in parallel given all v, a
        //
        // s_f[k] = I[k]*a[k] + fx(v[k])*I[k]*v[k]
        // start with s_f[k] = I[k]*a[k] and temp = *I[k]*v[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int jid = comp % 12;
            bool IaFlag = comp == jid; int jid6 = 6*jid; int vaOffset = IaFlag * 72 + jid6;
            T *dst = IaFlag ? &s_vaf[144] : s_temp;
            // compute based on the branch and save Iv to temp to prep for fx(v)*Iv and then sync
            dst[jid6 + row] = dot_prod<T,6,6,1>(&s_XImats[432 + 6*jid6 + row], &s_vaf[vaOffset]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_f[0] = I*s_a[0])\n"); printMat<T,1,6>(&s_vaf[144 + 6*0],1);
            printf("I*s_v[0])\n"); printMat<T,1,6>(&s_temp[6*0],1);
            printf("s_f[1] = I*s_a[1])\n"); printMat<T,1,6>(&s_vaf[144 + 6*1],1);
            printf("I*s_v[1])\n"); printMat<T,1,6>(&s_temp[6*1],1);
            printf("s_f[2] = I*s_a[2])\n"); printMat<T,1,6>(&s_vaf[144 + 6*2],1);
            printf("I*s_v[2])\n"); printMat<T,1,6>(&s_temp[6*2],1);
            printf("s_f[3] = I*s_a[3])\n"); printMat<T,1,6>(&s_vaf[144 + 6*3],1);
            printf("I*s_v[3])\n"); printMat<T,1,6>(&s_temp[6*3],1);
            printf("s_f[4] = I*s_a[4])\n"); printMat<T,1,6>(&s_vaf[144 + 6*4],1);
            printf("I*s_v[4])\n"); printMat<T,1,6>(&s_temp[6*4],1);
            printf("s_f[5] = I*s_a[5])\n"); printMat<T,1,6>(&s_vaf[144 + 6*5],1);
            printf("I*s_v[5])\n"); printMat<T,1,6>(&s_temp[6*5],1);
            printf("s_f[6] = I*s_a[6])\n"); printMat<T,1,6>(&s_vaf[144 + 6*6],1);
            printf("I*s_v[6])\n"); printMat<T,1,6>(&s_temp[6*6],1);
            printf("s_f[7] = I*s_a[7])\n"); printMat<T,1,6>(&s_vaf[144 + 6*7],1);
            printf("I*s_v[7])\n"); printMat<T,1,6>(&s_temp[6*7],1);
            printf("s_f[8] = I*s_a[8])\n"); printMat<T,1,6>(&s_vaf[144 + 6*8],1);
            printf("I*s_v[8])\n"); printMat<T,1,6>(&s_temp[6*8],1);
            printf("s_f[9] = I*s_a[9])\n"); printMat<T,1,6>(&s_vaf[144 + 6*9],1);
            printf("I*s_v[9])\n"); printMat<T,1,6>(&s_temp[6*9],1);
            printf("s_f[10] = I*s_a[10])\n"); printMat<T,1,6>(&s_vaf[144 + 6*10],1);
            printf("I*s_v[10])\n"); printMat<T,1,6>(&s_temp[6*10],1);
            printf("s_f[11] = I*s_a[11])\n"); printMat<T,1,6>(&s_vaf[144 + 6*11],1);
            printf("I*s_v[11])\n"); printMat<T,1,6>(&s_temp[6*11],1);
        }
        __syncthreads();
        // finish with s_f[k] += fx(v[k])*Iv[k]
        for(int jid = threadIdx.x + threadIdx.y*blockDim.x; jid < 12; jid += blockDim.x*blockDim.y){
            int jid6 = 6*jid;
            fx_times_v_peq<T>(&s_vaf[144 + jid6], &s_vaf[jid6], &s_temp[jid6]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_f[0] += fx(v[0])*I*v[0])\n"); printMat<T,1,6>(&s_vaf[144 + 6*0],1);
            printf("s_f[1] += fx(v[1])*I*v[1])\n"); printMat<T,1,6>(&s_vaf[144 + 6*1],1);
            printf("s_f[2] += fx(v[2])*I*v[2])\n"); printMat<T,1,6>(&s_vaf[144 + 6*2],1);
            printf("s_f[3] += fx(v[3])*I*v[3])\n"); printMat<T,1,6>(&s_vaf[144 + 6*3],1);
            printf("s_f[4] += fx(v[4])*I*v[4])\n"); printMat<T,1,6>(&s_vaf[144 + 6*4],1);
            printf("s_f[5] += fx(v[5])*I*v[5])\n"); printMat<T,1,6>(&s_vaf[144 + 6*5],1);
            printf("s_f[6] += fx(v[6])*I*v[6])\n"); printMat<T,1,6>(&s_vaf[144 + 6*6],1);
            printf("s_f[7] += fx(v[7])*I*v[7])\n"); printMat<T,1,6>(&s_vaf[144 + 6*7],1);
            printf("s_f[8] += fx(v[8])*I*v[8])\n"); printMat<T,1,6>(&s_vaf[144 + 6*8],1);
            printf("s_f[9] += fx(v[9])*I*v[9])\n"); printMat<T,1,6>(&s_vaf[144 + 6*9],1);
            printf("s_f[10] += fx(v[10])*I*v[10])\n"); printMat<T,1,6>(&s_vaf[144 + 6*10],1);
            printf("s_f[11] += fx(v[11])*I*v[11])\n"); printMat<T,1,6>(&s_vaf[144 + 6*11],1);
            printf("s_f forward pass\n"); printMat<T,6,12>(&s_vaf[144],6);
        }
        __syncthreads();
        //
        // Backward Pass
        //
        // s_f update where bfs_level is 2
        //     joints are: lf_kfe_joint, lh_kfe_joint, rf_kfe_joint, rh_kfe_joint
        //     links are: lf_lowerleg, lh_lowerleg, rf_lowerleg, rh_lowerleg
        // s_f[parent_k] += X[k]^T*f[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 2 + (ind < 12 && ind >= 6) * 5 + (ind < 18 && ind >= 12) * 8 + (ind >= 18) * 11;
            T val = dot_prod<T,6,1,1>(&s_XImats[36*jid + 6*row], &s_vaf[144 + 6*jid]);
            int dstOffset = 144 + 6*s_topology_helpers[jid] + row;
            s_vaf[dstOffset] += val;
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_f[1] += X^T*s_f[2]\n"); printMat<T,1,6>(&s_vaf[144 + 6*1],1);
            printf("s_f[4] += X^T*s_f[5]\n"); printMat<T,1,6>(&s_vaf[144 + 6*4],1);
            printf("s_f[7] += X^T*s_f[8]\n"); printMat<T,1,6>(&s_vaf[144 + 6*7],1);
            printf("s_f[10] += X^T*s_f[11]\n"); printMat<T,1,6>(&s_vaf[144 + 6*10],1);
        }
        __syncthreads();
        // s_f update where bfs_level is 1
        //     joints are: lf_hfe_joint, lh_hfe_joint, rf_hfe_joint, rh_hfe_joint
        //     links are: lf_upperleg, lh_upperleg, rf_upperleg, rh_upperleg
        // s_f[parent_k] += X[k]^T*f[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 1 + (ind < 12 && ind >= 6) * 4 + (ind < 18 && ind >= 12) * 7 + (ind >= 18) * 10;
            T val = dot_prod<T,6,1,1>(&s_XImats[36*jid + 6*row], &s_vaf[144 + 6*jid]);
            int dstOffset = 144 + 6*s_topology_helpers[jid] + row;
            s_vaf[dstOffset] += val;
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_f[0] += X^T*s_f[1]\n"); printMat<T,1,6>(&s_vaf[144 + 6*0],1);
            printf("s_f[3] += X^T*s_f[4]\n"); printMat<T,1,6>(&s_vaf[144 + 6*3],1);
            printf("s_f[6] += X^T*s_f[7]\n"); printMat<T,1,6>(&s_vaf[144 + 6*6],1);
            printf("s_f[9] += X^T*s_f[10]\n"); printMat<T,1,6>(&s_vaf[144 + 6*9],1);
        }
        __syncthreads();
        //
        // s_c extracted in parallel (S*f)
        //
        for(int dof_id = threadIdx.x + threadIdx.y*blockDim.x; dof_id < 12; dof_id += blockDim.x*blockDim.y){
            s_c[dof_id] = s_vaf[144 + 6*dof_id + 2];
        }
        __syncthreads();
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * Notes:
     *   Assumes the XI matricies have already been updated for the given q
     *   used to compute vaf as helper values
     *
     * @param s_vaf is a pointer to shared memory of size 3*6*NUM_JOINTS = 216
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param s_qdd is (optional vector of joint accelerations
     * @param s_XI is the pointer to the transformation and inertia matricies 
     * @param s_XImats is the (shared) memory holding the updated XI matricies for the given s_q
     * @param s_topology_helpers is the (shared) memory destination location for the topology_helpers
     * @param s_temp is a pointer to helper shared memory of size 6*NUM_JOINTS = 72
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void inverse_dynamics_inner_vaf(T *s_vaf, const T *s_q, const T *s_qd, const T *s_qdd, T *s_XImats, int *s_topology_helpers, T *s_temp, const T gravity) {
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("q\n"); printMat<T,1,12>(s_q,1);
            printf("qd\n"); printMat<T,1,12>(s_qd,1);
            printf("qdd\n"); printMat<T,1,6>(s_qdd,1);
            for (int i = 0; i < 12; i++){printf("X[%d]\n",i); printMat<T,6,6>(&s_XImats[36*i],6);}
            for (int i = 0; i < 12; i++){printf("I[%d]\n",i); printMat<T,6,6>(&s_XImats[36*(i+12)],6);}
        }
        __syncthreads();
        //
        // Forward Pass
        //
        // s_v, s_a where parent is base
        //     joints are: lf_haa_joint, lh_haa_joint, rf_haa_joint, rh_haa_joint
        //     links are: lf_hipassembly, lh_hipassembly, rf_hipassembly, rh_hipassembly
        // s_v[k] = S[k]*qd[k] and s_a[k] = X[k]*gravityS[k]*qdd[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 0 + (ind < 12 && ind >= 6) * 3 + (ind < 18 && ind >= 12) * 6 + (ind >= 18) * 9;
            int jid6 = 6*jid;
            s_vaf[jid6 + row] = static_cast<T>(0);
            s_vaf[72 + jid6 + row] = s_XImats[6*jid6 + 30 + row]*gravity;
            if (row == 2){s_vaf[jid6 + 2] += s_qd[jid]; s_vaf[72 + jid6 + 2] += s_qdd[jid];}
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_v[0]\n"); printMat<T,1,6>(&s_vaf[6*0],1);
            printf("s_a[0]\n"); printMat<T,1,6>(&s_vaf[72 + 6*0],1);
            printf("s_v[3]\n"); printMat<T,1,6>(&s_vaf[6*3],1);
            printf("s_a[3]\n"); printMat<T,1,6>(&s_vaf[72 + 6*3],1);
            printf("s_v[6]\n"); printMat<T,1,6>(&s_vaf[6*6],1);
            printf("s_a[6]\n"); printMat<T,1,6>(&s_vaf[72 + 6*6],1);
            printf("s_v[9]\n"); printMat<T,1,6>(&s_vaf[6*9],1);
            printf("s_a[9]\n"); printMat<T,1,6>(&s_vaf[72 + 6*9],1);
        }
        __syncthreads();
        // s_v and s_a where bfs_level is 1
        //     joints are: lf_hfe_joint, lh_hfe_joint, rf_hfe_joint, rh_hfe_joint
        //     links are: lf_upperleg, lh_upperleg, rf_upperleg, rh_upperleg
        // s_v[k] = X[k]*v[parent_k] + S[k]*qd[k] and s_a[k] = X[k]*a[parent_k] + S[k]*qdd[k] + mxS[k](v[k])*qd[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 48; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int comp_mod = comp % 4; int vFlag = comp == comp_mod;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 1 + (comp_mod == 1) * 4 + (comp_mod == 2) * 7 + (comp_mod == 3) * 10;
            int vaOffset = !vFlag * 72; int jid6 = 6 * jid;
            T qd_qdd_val = (row == 2) * (vFlag * s_qd[jid] + !vFlag * s_qdd[jid]);
            // compute based on the branch and use bool multiply for no branch
            s_vaf[vaOffset + jid6 + row] = dot_prod<T,6,6,1>(&s_XImats[6*jid6 + row], &s_vaf[vaOffset + 6*s_topology_helpers[jid]]) + qd_qdd_val;
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_v[1] = X*s_v[0] + S*qd[1]\n"); printMat<T,1,6>(&s_vaf[6*1],1);
            printf("s_a[1] = X*s_a[0] + S*qdd[1]\n"); printMat<T,1,6>(&s_vaf[72 + 6*1],1);
            printf("s_v[4] = X*s_v[3] + S*qd[4]\n"); printMat<T,1,6>(&s_vaf[6*4],1);
            printf("s_a[4] = X*s_a[3] + S*qdd[4]\n"); printMat<T,1,6>(&s_vaf[72 + 6*4],1);
            printf("s_v[7] = X*s_v[6] + S*qd[7]\n"); printMat<T,1,6>(&s_vaf[6*7],1);
            printf("s_a[7] = X*s_a[6] + S*qdd[7]\n"); printMat<T,1,6>(&s_vaf[72 + 6*7],1);
            printf("s_v[10] = X*s_v[9] + S*qd[10]\n"); printMat<T,1,6>(&s_vaf[6*10],1);
            printf("s_a[10] = X*s_a[9] + S*qdd[10]\n"); printMat<T,1,6>(&s_vaf[72 + 6*10],1);
        }
        __syncthreads();
        // sync before a += MxS(v)*qd[S] 
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 4; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind == 0) * 1 + (ind == 1) * 4 + (ind == 2) * 7 + (ind == 3) * 10;
            mx2_peq_scaled<T>(&s_vaf[72 + 6*jid], &s_vaf[6*jid], s_qd[jid]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_a[1] += MxS(s_v[1])\n"); printMat<T,1,6>(&s_vaf[72 + 6*1],1);
            printf("s_a[4] += MxS(s_v[4])\n"); printMat<T,1,6>(&s_vaf[72 + 6*4],1);
            printf("s_a[7] += MxS(s_v[7])\n"); printMat<T,1,6>(&s_vaf[72 + 6*7],1);
            printf("s_a[10] += MxS(s_v[10])\n"); printMat<T,1,6>(&s_vaf[72 + 6*10],1);
        }
        __syncthreads();
        // s_v and s_a where bfs_level is 2
        //     joints are: lf_kfe_joint, lh_kfe_joint, rf_kfe_joint, rh_kfe_joint
        //     links are: lf_lowerleg, lh_lowerleg, rf_lowerleg, rh_lowerleg
        // s_v[k] = X[k]*v[parent_k] + S[k]*qd[k] and s_a[k] = X[k]*a[parent_k] + S[k]*qdd[k] + mxS[k](v[k])*qd[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 48; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int comp_mod = comp % 4; int vFlag = comp == comp_mod;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 2 + (comp_mod == 1) * 5 + (comp_mod == 2) * 8 + (comp_mod == 3) * 11;
            int vaOffset = !vFlag * 72; int jid6 = 6 * jid;
            T qd_qdd_val = (row == 2) * (vFlag * s_qd[jid] + !vFlag * s_qdd[jid]);
            // compute based on the branch and use bool multiply for no branch
            s_vaf[vaOffset + jid6 + row] = dot_prod<T,6,6,1>(&s_XImats[6*jid6 + row], &s_vaf[vaOffset + 6*s_topology_helpers[jid]]) + qd_qdd_val;
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_v[2] = X*s_v[1] + S*qd[2]\n"); printMat<T,1,6>(&s_vaf[6*2],1);
            printf("s_a[2] = X*s_a[1] + S*qdd[2]\n"); printMat<T,1,6>(&s_vaf[72 + 6*2],1);
            printf("s_v[5] = X*s_v[4] + S*qd[5]\n"); printMat<T,1,6>(&s_vaf[6*5],1);
            printf("s_a[5] = X*s_a[4] + S*qdd[5]\n"); printMat<T,1,6>(&s_vaf[72 + 6*5],1);
            printf("s_v[8] = X*s_v[7] + S*qd[8]\n"); printMat<T,1,6>(&s_vaf[6*8],1);
            printf("s_a[8] = X*s_a[7] + S*qdd[8]\n"); printMat<T,1,6>(&s_vaf[72 + 6*8],1);
            printf("s_v[11] = X*s_v[10] + S*qd[11]\n"); printMat<T,1,6>(&s_vaf[6*11],1);
            printf("s_a[11] = X*s_a[10] + S*qdd[11]\n"); printMat<T,1,6>(&s_vaf[72 + 6*11],1);
        }
        __syncthreads();
        // sync before a += MxS(v)*qd[S] 
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 4; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind == 0) * 2 + (ind == 1) * 5 + (ind == 2) * 8 + (ind == 3) * 11;
            mx2_peq_scaled<T>(&s_vaf[72 + 6*jid], &s_vaf[6*jid], s_qd[jid]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_a[2] += MxS(s_v[2])\n"); printMat<T,1,6>(&s_vaf[72 + 6*2],1);
            printf("s_a[5] += MxS(s_v[5])\n"); printMat<T,1,6>(&s_vaf[72 + 6*5],1);
            printf("s_a[8] += MxS(s_v[8])\n"); printMat<T,1,6>(&s_vaf[72 + 6*8],1);
            printf("s_a[11] += MxS(s_v[11])\n"); printMat<T,1,6>(&s_vaf[72 + 6*11],1);
        }
        __syncthreads();
        //
        // s_f in parallel given all v, a
        //
        // s_f[k] = I[k]*a[k] + fx(v[k])*I[k]*v[k]
        // start with s_f[k] = I[k]*a[k] and temp = *I[k]*v[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int jid = comp % 12;
            bool IaFlag = comp == jid; int jid6 = 6*jid; int vaOffset = IaFlag * 72 + jid6;
            T *dst = IaFlag ? &s_vaf[144] : s_temp;
            // compute based on the branch and save Iv to temp to prep for fx(v)*Iv and then sync
            dst[jid6 + row] = dot_prod<T,6,6,1>(&s_XImats[432 + 6*jid6 + row], &s_vaf[vaOffset]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_f[0] = I*s_a[0])\n"); printMat<T,1,6>(&s_vaf[144 + 6*0],1);
            printf("I*s_v[0])\n"); printMat<T,1,6>(&s_temp[6*0],1);
            printf("s_f[1] = I*s_a[1])\n"); printMat<T,1,6>(&s_vaf[144 + 6*1],1);
            printf("I*s_v[1])\n"); printMat<T,1,6>(&s_temp[6*1],1);
            printf("s_f[2] = I*s_a[2])\n"); printMat<T,1,6>(&s_vaf[144 + 6*2],1);
            printf("I*s_v[2])\n"); printMat<T,1,6>(&s_temp[6*2],1);
            printf("s_f[3] = I*s_a[3])\n"); printMat<T,1,6>(&s_vaf[144 + 6*3],1);
            printf("I*s_v[3])\n"); printMat<T,1,6>(&s_temp[6*3],1);
            printf("s_f[4] = I*s_a[4])\n"); printMat<T,1,6>(&s_vaf[144 + 6*4],1);
            printf("I*s_v[4])\n"); printMat<T,1,6>(&s_temp[6*4],1);
            printf("s_f[5] = I*s_a[5])\n"); printMat<T,1,6>(&s_vaf[144 + 6*5],1);
            printf("I*s_v[5])\n"); printMat<T,1,6>(&s_temp[6*5],1);
            printf("s_f[6] = I*s_a[6])\n"); printMat<T,1,6>(&s_vaf[144 + 6*6],1);
            printf("I*s_v[6])\n"); printMat<T,1,6>(&s_temp[6*6],1);
            printf("s_f[7] = I*s_a[7])\n"); printMat<T,1,6>(&s_vaf[144 + 6*7],1);
            printf("I*s_v[7])\n"); printMat<T,1,6>(&s_temp[6*7],1);
            printf("s_f[8] = I*s_a[8])\n"); printMat<T,1,6>(&s_vaf[144 + 6*8],1);
            printf("I*s_v[8])\n"); printMat<T,1,6>(&s_temp[6*8],1);
            printf("s_f[9] = I*s_a[9])\n"); printMat<T,1,6>(&s_vaf[144 + 6*9],1);
            printf("I*s_v[9])\n"); printMat<T,1,6>(&s_temp[6*9],1);
            printf("s_f[10] = I*s_a[10])\n"); printMat<T,1,6>(&s_vaf[144 + 6*10],1);
            printf("I*s_v[10])\n"); printMat<T,1,6>(&s_temp[6*10],1);
            printf("s_f[11] = I*s_a[11])\n"); printMat<T,1,6>(&s_vaf[144 + 6*11],1);
            printf("I*s_v[11])\n"); printMat<T,1,6>(&s_temp[6*11],1);
        }
        __syncthreads();
        // finish with s_f[k] += fx(v[k])*Iv[k]
        for(int jid = threadIdx.x + threadIdx.y*blockDim.x; jid < 12; jid += blockDim.x*blockDim.y){
            int jid6 = 6*jid;
            fx_times_v_peq<T>(&s_vaf[144 + jid6], &s_vaf[jid6], &s_temp[jid6]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_f[0] += fx(v[0])*I*v[0])\n"); printMat<T,1,6>(&s_vaf[144 + 6*0],1);
            printf("s_f[1] += fx(v[1])*I*v[1])\n"); printMat<T,1,6>(&s_vaf[144 + 6*1],1);
            printf("s_f[2] += fx(v[2])*I*v[2])\n"); printMat<T,1,6>(&s_vaf[144 + 6*2],1);
            printf("s_f[3] += fx(v[3])*I*v[3])\n"); printMat<T,1,6>(&s_vaf[144 + 6*3],1);
            printf("s_f[4] += fx(v[4])*I*v[4])\n"); printMat<T,1,6>(&s_vaf[144 + 6*4],1);
            printf("s_f[5] += fx(v[5])*I*v[5])\n"); printMat<T,1,6>(&s_vaf[144 + 6*5],1);
            printf("s_f[6] += fx(v[6])*I*v[6])\n"); printMat<T,1,6>(&s_vaf[144 + 6*6],1);
            printf("s_f[7] += fx(v[7])*I*v[7])\n"); printMat<T,1,6>(&s_vaf[144 + 6*7],1);
            printf("s_f[8] += fx(v[8])*I*v[8])\n"); printMat<T,1,6>(&s_vaf[144 + 6*8],1);
            printf("s_f[9] += fx(v[9])*I*v[9])\n"); printMat<T,1,6>(&s_vaf[144 + 6*9],1);
            printf("s_f[10] += fx(v[10])*I*v[10])\n"); printMat<T,1,6>(&s_vaf[144 + 6*10],1);
            printf("s_f[11] += fx(v[11])*I*v[11])\n"); printMat<T,1,6>(&s_vaf[144 + 6*11],1);
            printf("s_f forward pass\n"); printMat<T,6,12>(&s_vaf[144],6);
        }
        __syncthreads();
        //
        // Backward Pass
        //
        // s_f update where bfs_level is 2
        //     joints are: lf_kfe_joint, lh_kfe_joint, rf_kfe_joint, rh_kfe_joint
        //     links are: lf_lowerleg, lh_lowerleg, rf_lowerleg, rh_lowerleg
        // s_f[parent_k] += X[k]^T*f[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 2 + (ind < 12 && ind >= 6) * 5 + (ind < 18 && ind >= 12) * 8 + (ind >= 18) * 11;
            T val = dot_prod<T,6,1,1>(&s_XImats[36*jid + 6*row], &s_vaf[144 + 6*jid]);
            int dstOffset = 144 + 6*s_topology_helpers[jid] + row;
            s_vaf[dstOffset] += val;
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_f[1] += X^T*s_f[2]\n"); printMat<T,1,6>(&s_vaf[144 + 6*1],1);
            printf("s_f[4] += X^T*s_f[5]\n"); printMat<T,1,6>(&s_vaf[144 + 6*4],1);
            printf("s_f[7] += X^T*s_f[8]\n"); printMat<T,1,6>(&s_vaf[144 + 6*7],1);
            printf("s_f[10] += X^T*s_f[11]\n"); printMat<T,1,6>(&s_vaf[144 + 6*10],1);
        }
        __syncthreads();
        // s_f update where bfs_level is 1
        //     joints are: lf_hfe_joint, lh_hfe_joint, rf_hfe_joint, rh_hfe_joint
        //     links are: lf_upperleg, lh_upperleg, rf_upperleg, rh_upperleg
        // s_f[parent_k] += X[k]^T*f[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 1 + (ind < 12 && ind >= 6) * 4 + (ind < 18 && ind >= 12) * 7 + (ind >= 18) * 10;
            T val = dot_prod<T,6,1,1>(&s_XImats[36*jid + 6*row], &s_vaf[144 + 6*jid]);
            int dstOffset = 144 + 6*s_topology_helpers[jid] + row;
            s_vaf[dstOffset] += val;
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_f[0] += X^T*s_f[1]\n"); printMat<T,1,6>(&s_vaf[144 + 6*0],1);
            printf("s_f[3] += X^T*s_f[4]\n"); printMat<T,1,6>(&s_vaf[144 + 6*3],1);
            printf("s_f[6] += X^T*s_f[7]\n"); printMat<T,1,6>(&s_vaf[144 + 6*6],1);
            printf("s_f[9] += X^T*s_f[10]\n"); printMat<T,1,6>(&s_vaf[144 + 6*9],1);
        }
        __syncthreads();
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * Notes:
     *   Assumes the XI matricies have already been updated for the given q
     *   used to compute vaf as helper values
     *   optimized for qdd = 0
     *
     * @param s_vaf is a pointer to shared memory of size 3*6*NUM_JOINTS = 216
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param s_XI is the pointer to the transformation and inertia matricies 
     * @param s_XImats is the (shared) memory holding the updated XI matricies for the given s_q
     * @param s_topology_helpers is the (shared) memory destination location for the topology_helpers
     * @param s_temp is a pointer to helper shared memory of size 6*NUM_JOINTS = 72
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void inverse_dynamics_inner_vaf(T *s_vaf, const T *s_q, const T *s_qd, T *s_XImats, int *s_topology_helpers, T *s_temp, const T gravity) {
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("q\n"); printMat<T,1,12>(s_q,1);
            printf("qd\n"); printMat<T,1,12>(s_qd,1);
            for (int i = 0; i < 12; i++){printf("X[%d]\n",i); printMat<T,6,6>(&s_XImats[36*i],6);}
            for (int i = 0; i < 12; i++){printf("I[%d]\n",i); printMat<T,6,6>(&s_XImats[36*(i+12)],6);}
        }
        __syncthreads();
        //
        // Forward Pass
        //
        // s_v, s_a where parent is base
        //     joints are: lf_haa_joint, lh_haa_joint, rf_haa_joint, rh_haa_joint
        //     links are: lf_hipassembly, lh_hipassembly, rf_hipassembly, rh_hipassembly
        // s_v[k] = S[k]*qd[k] and s_a[k] = X[k]*gravity
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 0 + (ind < 12 && ind >= 6) * 3 + (ind < 18 && ind >= 12) * 6 + (ind >= 18) * 9;
            int jid6 = 6*jid;
            s_vaf[jid6 + row] = static_cast<T>(0);
            s_vaf[72 + jid6 + row] = s_XImats[6*jid6 + 30 + row]*gravity;
            if (row == 2){s_vaf[jid6 + 2] += s_qd[jid];}
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_v[0]\n"); printMat<T,1,6>(&s_vaf[6*0],1);
            printf("s_a[0]\n"); printMat<T,1,6>(&s_vaf[72 + 6*0],1);
            printf("s_v[3]\n"); printMat<T,1,6>(&s_vaf[6*3],1);
            printf("s_a[3]\n"); printMat<T,1,6>(&s_vaf[72 + 6*3],1);
            printf("s_v[6]\n"); printMat<T,1,6>(&s_vaf[6*6],1);
            printf("s_a[6]\n"); printMat<T,1,6>(&s_vaf[72 + 6*6],1);
            printf("s_v[9]\n"); printMat<T,1,6>(&s_vaf[6*9],1);
            printf("s_a[9]\n"); printMat<T,1,6>(&s_vaf[72 + 6*9],1);
        }
        __syncthreads();
        // s_v and s_a where bfs_level is 1
        //     joints are: lf_hfe_joint, lh_hfe_joint, rf_hfe_joint, rh_hfe_joint
        //     links are: lf_upperleg, lh_upperleg, rf_upperleg, rh_upperleg
        // s_v[k] = X[k]*v[parent_k] + S[k]*qd[k] and s_a[k] = X[k]*a[parent_k] + mxS[k](v[k])*qd[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 48; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int comp_mod = comp % 4; int vFlag = comp == comp_mod;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 1 + (comp_mod == 1) * 4 + (comp_mod == 2) * 7 + (comp_mod == 3) * 10;
            int vaOffset = !vFlag * 72; int jid6 = 6 * jid;
            T qd_qdd_val = (row == 2) * (vFlag * s_qd[jid]);
            // compute based on the branch and use bool multiply for no branch
            s_vaf[vaOffset + jid6 + row] = dot_prod<T,6,6,1>(&s_XImats[6*jid6 + row], &s_vaf[vaOffset + 6*s_topology_helpers[jid]]) + qd_qdd_val;
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_v[1] = X*s_v[0] + S*qd[1]\n"); printMat<T,1,6>(&s_vaf[6*1],1);
            printf("s_a[1] = X*s_a[0]\n"); printMat<T,1,6>(&s_vaf[72 + 6*1],1);
            printf("s_v[4] = X*s_v[3] + S*qd[4]\n"); printMat<T,1,6>(&s_vaf[6*4],1);
            printf("s_a[4] = X*s_a[3]\n"); printMat<T,1,6>(&s_vaf[72 + 6*4],1);
            printf("s_v[7] = X*s_v[6] + S*qd[7]\n"); printMat<T,1,6>(&s_vaf[6*7],1);
            printf("s_a[7] = X*s_a[6]\n"); printMat<T,1,6>(&s_vaf[72 + 6*7],1);
            printf("s_v[10] = X*s_v[9] + S*qd[10]\n"); printMat<T,1,6>(&s_vaf[6*10],1);
            printf("s_a[10] = X*s_a[9]\n"); printMat<T,1,6>(&s_vaf[72 + 6*10],1);
        }
        __syncthreads();
        // sync before a += MxS(v)*qd[S] 
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 4; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind == 0) * 1 + (ind == 1) * 4 + (ind == 2) * 7 + (ind == 3) * 10;
            mx2_peq_scaled<T>(&s_vaf[72 + 6*jid], &s_vaf[6*jid], s_qd[jid]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_a[1] += MxS(s_v[1])\n"); printMat<T,1,6>(&s_vaf[72 + 6*1],1);
            printf("s_a[4] += MxS(s_v[4])\n"); printMat<T,1,6>(&s_vaf[72 + 6*4],1);
            printf("s_a[7] += MxS(s_v[7])\n"); printMat<T,1,6>(&s_vaf[72 + 6*7],1);
            printf("s_a[10] += MxS(s_v[10])\n"); printMat<T,1,6>(&s_vaf[72 + 6*10],1);
        }
        __syncthreads();
        // s_v and s_a where bfs_level is 2
        //     joints are: lf_kfe_joint, lh_kfe_joint, rf_kfe_joint, rh_kfe_joint
        //     links are: lf_lowerleg, lh_lowerleg, rf_lowerleg, rh_lowerleg
        // s_v[k] = X[k]*v[parent_k] + S[k]*qd[k] and s_a[k] = X[k]*a[parent_k] + mxS[k](v[k])*qd[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 48; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int comp_mod = comp % 4; int vFlag = comp == comp_mod;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 2 + (comp_mod == 1) * 5 + (comp_mod == 2) * 8 + (comp_mod == 3) * 11;
            int vaOffset = !vFlag * 72; int jid6 = 6 * jid;
            T qd_qdd_val = (row == 2) * (vFlag * s_qd[jid]);
            // compute based on the branch and use bool multiply for no branch
            s_vaf[vaOffset + jid6 + row] = dot_prod<T,6,6,1>(&s_XImats[6*jid6 + row], &s_vaf[vaOffset + 6*s_topology_helpers[jid]]) + qd_qdd_val;
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_v[2] = X*s_v[1] + S*qd[2]\n"); printMat<T,1,6>(&s_vaf[6*2],1);
            printf("s_a[2] = X*s_a[1]\n"); printMat<T,1,6>(&s_vaf[72 + 6*2],1);
            printf("s_v[5] = X*s_v[4] + S*qd[5]\n"); printMat<T,1,6>(&s_vaf[6*5],1);
            printf("s_a[5] = X*s_a[4]\n"); printMat<T,1,6>(&s_vaf[72 + 6*5],1);
            printf("s_v[8] = X*s_v[7] + S*qd[8]\n"); printMat<T,1,6>(&s_vaf[6*8],1);
            printf("s_a[8] = X*s_a[7]\n"); printMat<T,1,6>(&s_vaf[72 + 6*8],1);
            printf("s_v[11] = X*s_v[10] + S*qd[11]\n"); printMat<T,1,6>(&s_vaf[6*11],1);
            printf("s_a[11] = X*s_a[10]\n"); printMat<T,1,6>(&s_vaf[72 + 6*11],1);
        }
        __syncthreads();
        // sync before a += MxS(v)*qd[S] 
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 4; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind == 0) * 2 + (ind == 1) * 5 + (ind == 2) * 8 + (ind == 3) * 11;
            mx2_peq_scaled<T>(&s_vaf[72 + 6*jid], &s_vaf[6*jid], s_qd[jid]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_a[2] += MxS(s_v[2])\n"); printMat<T,1,6>(&s_vaf[72 + 6*2],1);
            printf("s_a[5] += MxS(s_v[5])\n"); printMat<T,1,6>(&s_vaf[72 + 6*5],1);
            printf("s_a[8] += MxS(s_v[8])\n"); printMat<T,1,6>(&s_vaf[72 + 6*8],1);
            printf("s_a[11] += MxS(s_v[11])\n"); printMat<T,1,6>(&s_vaf[72 + 6*11],1);
        }
        __syncthreads();
        //
        // s_f in parallel given all v, a
        //
        // s_f[k] = I[k]*a[k] + fx(v[k])*I[k]*v[k]
        // start with s_f[k] = I[k]*a[k] and temp = *I[k]*v[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int jid = comp % 12;
            bool IaFlag = comp == jid; int jid6 = 6*jid; int vaOffset = IaFlag * 72 + jid6;
            T *dst = IaFlag ? &s_vaf[144] : s_temp;
            // compute based on the branch and save Iv to temp to prep for fx(v)*Iv and then sync
            dst[jid6 + row] = dot_prod<T,6,6,1>(&s_XImats[432 + 6*jid6 + row], &s_vaf[vaOffset]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_f[0] = I*s_a[0])\n"); printMat<T,1,6>(&s_vaf[144 + 6*0],1);
            printf("I*s_v[0])\n"); printMat<T,1,6>(&s_temp[6*0],1);
            printf("s_f[1] = I*s_a[1])\n"); printMat<T,1,6>(&s_vaf[144 + 6*1],1);
            printf("I*s_v[1])\n"); printMat<T,1,6>(&s_temp[6*1],1);
            printf("s_f[2] = I*s_a[2])\n"); printMat<T,1,6>(&s_vaf[144 + 6*2],1);
            printf("I*s_v[2])\n"); printMat<T,1,6>(&s_temp[6*2],1);
            printf("s_f[3] = I*s_a[3])\n"); printMat<T,1,6>(&s_vaf[144 + 6*3],1);
            printf("I*s_v[3])\n"); printMat<T,1,6>(&s_temp[6*3],1);
            printf("s_f[4] = I*s_a[4])\n"); printMat<T,1,6>(&s_vaf[144 + 6*4],1);
            printf("I*s_v[4])\n"); printMat<T,1,6>(&s_temp[6*4],1);
            printf("s_f[5] = I*s_a[5])\n"); printMat<T,1,6>(&s_vaf[144 + 6*5],1);
            printf("I*s_v[5])\n"); printMat<T,1,6>(&s_temp[6*5],1);
            printf("s_f[6] = I*s_a[6])\n"); printMat<T,1,6>(&s_vaf[144 + 6*6],1);
            printf("I*s_v[6])\n"); printMat<T,1,6>(&s_temp[6*6],1);
            printf("s_f[7] = I*s_a[7])\n"); printMat<T,1,6>(&s_vaf[144 + 6*7],1);
            printf("I*s_v[7])\n"); printMat<T,1,6>(&s_temp[6*7],1);
            printf("s_f[8] = I*s_a[8])\n"); printMat<T,1,6>(&s_vaf[144 + 6*8],1);
            printf("I*s_v[8])\n"); printMat<T,1,6>(&s_temp[6*8],1);
            printf("s_f[9] = I*s_a[9])\n"); printMat<T,1,6>(&s_vaf[144 + 6*9],1);
            printf("I*s_v[9])\n"); printMat<T,1,6>(&s_temp[6*9],1);
            printf("s_f[10] = I*s_a[10])\n"); printMat<T,1,6>(&s_vaf[144 + 6*10],1);
            printf("I*s_v[10])\n"); printMat<T,1,6>(&s_temp[6*10],1);
            printf("s_f[11] = I*s_a[11])\n"); printMat<T,1,6>(&s_vaf[144 + 6*11],1);
            printf("I*s_v[11])\n"); printMat<T,1,6>(&s_temp[6*11],1);
        }
        __syncthreads();
        // finish with s_f[k] += fx(v[k])*Iv[k]
        for(int jid = threadIdx.x + threadIdx.y*blockDim.x; jid < 12; jid += blockDim.x*blockDim.y){
            int jid6 = 6*jid;
            fx_times_v_peq<T>(&s_vaf[144 + jid6], &s_vaf[jid6], &s_temp[jid6]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_f[0] += fx(v[0])*I*v[0])\n"); printMat<T,1,6>(&s_vaf[144 + 6*0],1);
            printf("s_f[1] += fx(v[1])*I*v[1])\n"); printMat<T,1,6>(&s_vaf[144 + 6*1],1);
            printf("s_f[2] += fx(v[2])*I*v[2])\n"); printMat<T,1,6>(&s_vaf[144 + 6*2],1);
            printf("s_f[3] += fx(v[3])*I*v[3])\n"); printMat<T,1,6>(&s_vaf[144 + 6*3],1);
            printf("s_f[4] += fx(v[4])*I*v[4])\n"); printMat<T,1,6>(&s_vaf[144 + 6*4],1);
            printf("s_f[5] += fx(v[5])*I*v[5])\n"); printMat<T,1,6>(&s_vaf[144 + 6*5],1);
            printf("s_f[6] += fx(v[6])*I*v[6])\n"); printMat<T,1,6>(&s_vaf[144 + 6*6],1);
            printf("s_f[7] += fx(v[7])*I*v[7])\n"); printMat<T,1,6>(&s_vaf[144 + 6*7],1);
            printf("s_f[8] += fx(v[8])*I*v[8])\n"); printMat<T,1,6>(&s_vaf[144 + 6*8],1);
            printf("s_f[9] += fx(v[9])*I*v[9])\n"); printMat<T,1,6>(&s_vaf[144 + 6*9],1);
            printf("s_f[10] += fx(v[10])*I*v[10])\n"); printMat<T,1,6>(&s_vaf[144 + 6*10],1);
            printf("s_f[11] += fx(v[11])*I*v[11])\n"); printMat<T,1,6>(&s_vaf[144 + 6*11],1);
            printf("s_f forward pass\n"); printMat<T,6,12>(&s_vaf[144],6);
        }
        __syncthreads();
        //
        // Backward Pass
        //
        // s_f update where bfs_level is 2
        //     joints are: lf_kfe_joint, lh_kfe_joint, rf_kfe_joint, rh_kfe_joint
        //     links are: lf_lowerleg, lh_lowerleg, rf_lowerleg, rh_lowerleg
        // s_f[parent_k] += X[k]^T*f[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 2 + (ind < 12 && ind >= 6) * 5 + (ind < 18 && ind >= 12) * 8 + (ind >= 18) * 11;
            T val = dot_prod<T,6,1,1>(&s_XImats[36*jid + 6*row], &s_vaf[144 + 6*jid]);
            int dstOffset = 144 + 6*s_topology_helpers[jid] + row;
            s_vaf[dstOffset] += val;
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_f[1] += X^T*s_f[2]\n"); printMat<T,1,6>(&s_vaf[144 + 6*1],1);
            printf("s_f[4] += X^T*s_f[5]\n"); printMat<T,1,6>(&s_vaf[144 + 6*4],1);
            printf("s_f[7] += X^T*s_f[8]\n"); printMat<T,1,6>(&s_vaf[144 + 6*7],1);
            printf("s_f[10] += X^T*s_f[11]\n"); printMat<T,1,6>(&s_vaf[144 + 6*10],1);
        }
        __syncthreads();
        // s_f update where bfs_level is 1
        //     joints are: lf_hfe_joint, lh_hfe_joint, rf_hfe_joint, rh_hfe_joint
        //     links are: lf_upperleg, lh_upperleg, rf_upperleg, rh_upperleg
        // s_f[parent_k] += X[k]^T*f[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 1 + (ind < 12 && ind >= 6) * 4 + (ind < 18 && ind >= 12) * 7 + (ind >= 18) * 10;
            T val = dot_prod<T,6,1,1>(&s_XImats[36*jid + 6*row], &s_vaf[144 + 6*jid]);
            int dstOffset = 144 + 6*s_topology_helpers[jid] + row;
            s_vaf[dstOffset] += val;
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_f[0] += X^T*s_f[1]\n"); printMat<T,1,6>(&s_vaf[144 + 6*0],1);
            printf("s_f[3] += X^T*s_f[4]\n"); printMat<T,1,6>(&s_vaf[144 + 6*3],1);
            printf("s_f[6] += X^T*s_f[7]\n"); printMat<T,1,6>(&s_vaf[144 + 6*6],1);
            printf("s_f[9] += X^T*s_f[10]\n"); printMat<T,1,6>(&s_vaf[144 + 6*9],1);
        }
        __syncthreads();
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * @param s_c is the vector of output torques
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param s_qdd is the vector of joint accelerations
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void inverse_dynamics_device(T *s_c,  const T *s_q, const T *s_qd, const T *s_qdd, const robotModel<T> *d_robotModel, const T gravity) {
        __shared__ T s_vaf[216];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
        inverse_dynamics_inner<T>(s_c, s_vaf, s_q, s_qd, s_qdd, s_XImats, s_topology_helpers, s_temp, gravity);
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * Notes:
     *   optimized for qdd = 0
     *
     * @param s_c is the vector of output torques
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void inverse_dynamics_device(T *s_c,  const T *s_q, const T *s_qd, const robotModel<T> *d_robotModel, const T gravity) {
        __shared__ T s_vaf[216];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
        inverse_dynamics_inner<T>(s_c, s_vaf, s_q, s_qd, s_XImats, s_topology_helpers, s_temp, gravity);
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * Notes:
     *   used to compute vaf as helper values
     *
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param s_qdd is the vector of joint accelerations
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void inverse_dynamics_vaf_device(T *s_vaf, const T *s_q, const T *s_qd, const T *s_qdd, const robotModel<T> *d_robotModel, const T gravity) {
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
        inverse_dynamics_inner_vaf<T>(s_vaf, s_q, s_qd, s_qdd, s_XImats, s_topology_helpers, s_temp, gravity);
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * Notes:
     *   used to compute vaf as helper values
     *   optimized for qdd = 0
     *
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void inverse_dynamics_vaf_device(T *s_vaf, const T *s_q, const T *s_qd, const robotModel<T> *d_robotModel, const T gravity) {
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
        inverse_dynamics_inner_vaf<T>(s_vaf, s_q, s_qd, s_XImats, s_topology_helpers, s_temp, gravity);
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * @param d_c is the vector of output torques
     * @param d_q_dq is the vector of joint positions and velocities
     * @param d_qdd is the vector of joint accelerations
     * @param stride_q_qd is the stide between each q, qd
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void inverse_dynamics_kernel_single_timing(T *d_c, const T *d_q_qd, const int stride_q_qd, const T *d_qdd, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_q_qd[2*12]; T *s_q = s_q_qd; T *s_qd = &s_q_qd[12];
        __shared__ T s_qdd[12]; 
        __shared__ T s_c[12];
        __shared__ T s_vaf[216];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        // load to shared mem
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            s_q_qd[ind] = d_q_qd[ind];
        }
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
            s_qdd[ind] = d_qdd[ind];
        }
        __syncthreads();
        // compute with NUM_TIMESTEPS as NUM_REPS for timing
        for (int rep = 0; rep < NUM_TIMESTEPS; rep++){
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            inverse_dynamics_inner<T>(s_c, s_vaf, s_q, s_qd, s_qdd, s_XImats, s_topology_helpers, s_temp, gravity);
        }
        // save down to global
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
            d_c[ind] = s_c[ind];
        }
        __syncthreads();
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * @param d_c is the vector of output torques
     * @param d_q_dq is the vector of joint positions and velocities
     * @param d_qdd is the vector of joint accelerations
     * @param stride_q_qd is the stide between each q, qd
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void inverse_dynamics_kernel(T *d_c, const T *d_q_qd, const int stride_q_qd, const T *d_qdd, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_q_qd[2*12]; T *s_q = s_q_qd; T *s_qd = &s_q_qd[12];
        __shared__ T s_qdd[12]; 
        __shared__ T s_c[12];
        __shared__ T s_vaf[216];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        for(int k = blockIdx.x + blockIdx.y*gridDim.x; k < NUM_TIMESTEPS; k += gridDim.x*gridDim.y){
            // load to shared mem
            const T *d_q_qd_k = &d_q_qd[k*stride_q_qd];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
                s_q_qd[ind] = d_q_qd_k[ind];
            }
            const T *d_qdd_k = &d_qdd[k*12];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
                s_qdd[ind] = d_qdd_k[ind];
            }
            __syncthreads();
            // compute
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            inverse_dynamics_inner<T>(s_c, s_vaf, s_q, s_qd, s_qdd, s_XImats, s_topology_helpers, s_temp, gravity);
            __syncthreads();
            // save down to global
            T *d_c_k = &d_c[k*12];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
                d_c_k[ind] = s_c[ind];
            }
            __syncthreads();
        }
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * Notes:
     *   optimized for qdd = 0
     *
     * @param d_c is the vector of output torques
     * @param d_q_dq is the vector of joint positions and velocities
     * @param stride_q_qd is the stide between each q, qd
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void inverse_dynamics_kernel_single_timing(T *d_c, const T *d_q_qd, const int stride_q_qd, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_q_qd[2*12]; T *s_q = s_q_qd; T *s_qd = &s_q_qd[12];
        __shared__ T s_c[12];
        __shared__ T s_vaf[216];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        // load to shared mem
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            s_q_qd[ind] = d_q_qd[ind];
        }
        __syncthreads();
        // compute with NUM_TIMESTEPS as NUM_REPS for timing
        for (int rep = 0; rep < NUM_TIMESTEPS; rep++){
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            inverse_dynamics_inner<T>(s_c, s_vaf, s_q, s_qd, s_XImats, s_topology_helpers, s_temp, gravity);
        }
        // save down to global
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
            d_c[ind] = s_c[ind];
        }
        __syncthreads();
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * Notes:
     *   optimized for qdd = 0
     *
     * @param d_c is the vector of output torques
     * @param d_q_dq is the vector of joint positions and velocities
     * @param stride_q_qd is the stide between each q, qd
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void inverse_dynamics_kernel(T *d_c, const T *d_q_qd, const int stride_q_qd, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_q_qd[2*12]; T *s_q = s_q_qd; T *s_qd = &s_q_qd[12];
        __shared__ T s_c[12];
        __shared__ T s_vaf[216];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        for(int k = blockIdx.x + blockIdx.y*gridDim.x; k < NUM_TIMESTEPS; k += gridDim.x*gridDim.y){
            // load to shared mem
            const T *d_q_qd_k = &d_q_qd[k*stride_q_qd];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
                s_q_qd[ind] = d_q_qd_k[ind];
            }
            __syncthreads();
            // compute
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            inverse_dynamics_inner<T>(s_c, s_vaf, s_q, s_qd, s_XImats, s_topology_helpers, s_temp, gravity);
            __syncthreads();
            // save down to global
            T *d_c_k = &d_c[k*12];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
                d_c_k[ind] = s_c[ind];
            }
            __syncthreads();
        }
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_QDD_FLAG = false, bool USE_COMPRESSED_MEM = false>
    __host__
    void inverse_dynamics(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                          const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        // start code with memory transfer
        int stride_q_qd;
        if (USE_COMPRESSED_MEM) {stride_q_qd = 2*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd,hd_data->h_q_qd,stride_q_qd*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        else {stride_q_qd = 3*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q_qd*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        if (USE_QDD_FLAG) {gpuErrchk(cudaMemcpyAsync(hd_data->d_qdd,hd_data->h_qdd,NUM_JOINTS*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[1]));}
        gpuErrchk(cudaDeviceSynchronize());
        // then call the kernel
        if (USE_QDD_FLAG) {
            if (USE_COMPRESSED_MEM) {inverse_dynamics_kernel<T><<<block_dimms,thread_dimms,ID_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_c,hd_data->d_q_qd,stride_q_qd,hd_data->d_qdd, d_robotModel,gravity,num_timesteps);}
            else                    {inverse_dynamics_kernel<T><<<block_dimms,thread_dimms,ID_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_c,hd_data->d_q_qd_u,stride_q_qd,hd_data->d_qdd, d_robotModel,gravity,num_timesteps);}
        }
        else {
            if (USE_COMPRESSED_MEM) {inverse_dynamics_kernel<T><<<block_dimms,thread_dimms,ID_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_c,hd_data->d_q_qd,stride_q_qd,d_robotModel,gravity,num_timesteps);}
            else                    {inverse_dynamics_kernel<T><<<block_dimms,thread_dimms,ID_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_c,hd_data->d_q_qd_u,stride_q_qd,d_robotModel,gravity,num_timesteps);}
        }
        gpuErrchk(cudaDeviceSynchronize());
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_c,hd_data->d_c,NUM_JOINTS*num_timesteps*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_QDD_FLAG = false, bool USE_COMPRESSED_MEM = false>
    __host__
    void inverse_dynamics_single_timing(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                                        const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        // start code with memory transfer
        int stride_q_qd;
        if (USE_COMPRESSED_MEM) {stride_q_qd = 2*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd,hd_data->h_q_qd,stride_q_qd*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        else {stride_q_qd = 3*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q_qd*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        if (USE_QDD_FLAG) {gpuErrchk(cudaMemcpyAsync(hd_data->d_qdd,hd_data->h_qdd,NUM_JOINTS*sizeof(T),cudaMemcpyHostToDevice,streams[1]));}
        gpuErrchk(cudaDeviceSynchronize());
        // then call the kernel
        struct timespec start, end; clock_gettime(CLOCK_MONOTONIC,&start);
        if (USE_QDD_FLAG) {
            if (USE_COMPRESSED_MEM) {inverse_dynamics_kernel_single_timing<T><<<block_dimms,thread_dimms,ID_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_c,hd_data->d_q_qd,stride_q_qd,hd_data->d_qdd, d_robotModel,gravity,num_timesteps);}
            else                    {inverse_dynamics_kernel_single_timing<T><<<block_dimms,thread_dimms,ID_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_c,hd_data->d_q_qd_u,stride_q_qd,hd_data->d_qdd, d_robotModel,gravity,num_timesteps);}
        }
        else {
            if (USE_COMPRESSED_MEM) {inverse_dynamics_kernel_single_timing<T><<<block_dimms,thread_dimms,ID_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_c,hd_data->d_q_qd,stride_q_qd,d_robotModel,gravity,num_timesteps);}
            else                    {inverse_dynamics_kernel_single_timing<T><<<block_dimms,thread_dimms,ID_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_c,hd_data->d_q_qd_u,stride_q_qd,d_robotModel,gravity,num_timesteps);}
        }
        gpuErrchk(cudaDeviceSynchronize());
        clock_gettime(CLOCK_MONOTONIC,&end);
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_c,hd_data->d_c,NUM_JOINTS*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
        printf("Single Call ID %fus\n",time_delta_us_timespec(start,end)/static_cast<double>(num_timesteps));
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_QDD_FLAG = false, bool USE_COMPRESSED_MEM = false>
    __host__
    void inverse_dynamics_compute_only(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                                       const dim3 block_dimms, const dim3 thread_dimms) {
        int stride_q_qd = USE_COMPRESSED_MEM ? 2*NUM_JOINTS: 3*NUM_JOINTS;
        // then call the kernel
        if (USE_QDD_FLAG) {
            if (USE_COMPRESSED_MEM) {inverse_dynamics_kernel<T><<<block_dimms,thread_dimms,ID_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_c,hd_data->d_q_qd,stride_q_qd,hd_data->d_qdd, d_robotModel,gravity,num_timesteps);}
            else                    {inverse_dynamics_kernel<T><<<block_dimms,thread_dimms,ID_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_c,hd_data->d_q_qd_u,stride_q_qd,hd_data->d_qdd, d_robotModel,gravity,num_timesteps);}
        }
        else {
            if (USE_COMPRESSED_MEM) {inverse_dynamics_kernel<T><<<block_dimms,thread_dimms,ID_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_c,hd_data->d_q_qd,stride_q_qd,d_robotModel,gravity,num_timesteps);}
            else                    {inverse_dynamics_kernel<T><<<block_dimms,thread_dimms,ID_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_c,hd_data->d_q_qd_u,stride_q_qd,d_robotModel,gravity,num_timesteps);}
        }
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Compute the inverse of the mass matrix
     *
     * Notes:
     *   Assumes the XI matricies have already been updated for the given q
     *   Outputs a SYMMETRIC_UPPER triangular matrix for Minv
     *
     * @param s_Minv is a pointer to memory for the final result
     * @param s_q is the vector of joint positions
     * @param s_XImats is the (shared) memory holding the updated XI matricies for the given s_q
     * @param s_topology_helpers is the (shared) memory destination location for the topology_helpers
     * @param s_temp is a pointer to helper shared memory of size 1668
     */
    template <typename T>
    __device__
    void direct_minv_inner(T *s_Minv, const T *s_q, T *s_XImats, int *s_topology_helpers, T *s_temp) {
        // T *s_F = &s_temp[0]; T *s_IA = &s_temp[864]; T *s_U = &s_temp[1296]; T *s_Dinv = &s_temp[1368]; T *s_Ia = &s_temp[1380]; T *s_IaTemp = &s_temp[1524];
        // Initialize IA = I
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 432; ind += blockDim.x*blockDim.y){
            s_temp[864 + ind] = s_XImats[432 + ind];
        }
        // Zero Minv and F
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 1008; ind += blockDim.x*blockDim.y){
            if(ind < 864){s_temp[0 + ind] = static_cast<T>(0);}
            else{s_Minv[ind - 864] = static_cast<T>(0);}
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("q\n"); printMat<T,1,12>(s_q,1);
            for (int i = 0; i < 12; i++){printf("X[%d]\n",i); printMat<T,6,6>(&s_XImats[36*i],6);}
            for (int i = 0; i < 12; i++){printf("IA_init = I[%d]\n",i); printMat<T,6,6>(&s_temp[864 + 36*i],6);}
        }
        __syncthreads();
        //
        // Backward Pass
        //
        // backward pass updates where bfs_level is 2
        //     joints are: lf_kfe_joint, lh_kfe_joint, rf_kfe_joint, rh_kfe_joint
        //     links are: lf_lowerleg, lh_lowerleg, rf_lowerleg, rh_lowerleg
        // U = IA*S, D = S^T*U, DInv = 1/D, Minv[i,i] = Dinv
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 2 + (ind < 12 && ind >= 6) * 5 + (ind < 18 && ind >= 12) * 8 + (ind >= 18) * 11;
            int jid6 = 6*jid;
            s_temp[1296 + jid6 + row] = s_temp[864 + 6*jid6 + 6*2 + row];
            if(row == 2){
                s_temp[1368 + jid] = static_cast<T>(1)/s_temp[1296 + jid6 + 2];
                s_Minv[13 * jid] = s_temp[1368 + jid];
            }
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("U[2]\n"); printMat<T,1,6>(&s_temp[1296 + 6*2],1);
            printf("Dinv[2] = %f\n",s_temp[1368 + 2]);
            printf("U[5]\n"); printMat<T,1,6>(&s_temp[1296 + 6*5],1);
            printf("Dinv[5] = %f\n",s_temp[1368 + 5]);
            printf("U[8]\n"); printMat<T,1,6>(&s_temp[1296 + 6*8],1);
            printf("Dinv[8] = %f\n",s_temp[1368 + 8]);
            printf("U[11]\n"); printMat<T,1,6>(&s_temp[1296 + 6*11],1);
            printf("Dinv[11] = %f\n",s_temp[1368 + 11]);
            printf("Minv after Dinv setting before subtree\n"); printMat<T,12,12>(s_Minv,12);
        }
        __syncthreads();
        // Minv[i,subTreeInds] -= Dinv*F[i,Srow,SubTreeInds]
        // Temp Comp: F[i,:,subTreeInds] += U*Minv[i,subTreeInds] - to start Fparent Update
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 4; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind < 1) * 2 + (ind < 2 && ind >= 1) * 5 + (ind < 3 && ind >= 2) * 8 + (ind >= 3) * 11;
            int subTreeAdj = (ind < 1) * 0 + (ind < 2 && ind >= 1) * 1 + (ind < 3 && ind >= 2) * 2 + (ind >= 3) * 3;
            int jid_subtree = jid + (ind - subTreeAdj); int jid_subtree6 = 6*jid_subtree; int jid_subtreeN = 12*jid_subtree;
            s_Minv[jid_subtreeN + jid] -= s_temp[1368 + jid] * s_temp[0 + 72*jid + jid_subtree6 + 2];
            for(int row = 0; row < 6; row++) {
                s_temp[0 + 72*jid + jid_subtree6 + row] += s_temp[1296 + 6*jid + row] * s_Minv[jid_subtreeN + jid];
            }
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Minv after subtree updates\n"); printMat<T,12,12>(s_Minv,12);
            printf("F Temp += U*Minv[2]\n"); printMat<T,6,12>(&s_temp[144],6);
            printf("F Temp += U*Minv[5]\n"); printMat<T,6,12>(&s_temp[360],6);
            printf("F Temp += U*Minv[8]\n"); printMat<T,6,12>(&s_temp[576],6);
            printf("F Temp += U*Minv[11]\n"); printMat<T,6,12>(&s_temp[792],6);
        }
        __syncthreads();
        // Ia = IA - U^T Dinv U | to start IAparent Update
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind < 36) * 2 + (ind < 72 && ind >= 36) * 5 + (ind < 108 && ind >= 72) * 8 + (ind >= 108) * 11;
            int ind36 = (ind % 36); int row = ind36 % 6; int col = ind36 / 6; int jid6 = 6*jid;
            s_temp[1380 + ind] = s_temp[864 + 6*jid6 + ind36] - (s_temp[1296 + jid6 + row] * s_temp[1368 + jid] * s_temp[1296 + jid6 + col]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Ia[2]\n");
            printMat<T,6,6>(&s_temp[1380],6);
            printf("Ia[5]\n");
            printMat<T,6,6>(&s_temp[1416],6);
            printf("Ia[8]\n");
            printMat<T,6,6>(&s_temp[1452],6);
            printf("Ia[11]\n");
            printMat<T,6,6>(&s_temp[1488],6);
        }
        __syncthreads();
        // F[parent_ind,:,subTreeInds] += Xmat^T * F[ind,:,subTreeInds]
        // IA_Update_Temp = Xmat^T * Ia | for IAparent Update
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 168; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = ind / 6;
            // non-branching pointer selector
            int jid = (col < 1) * 2 + (col < 2 && col >= 1) * 5 + (col < 3 && col >= 2) * 8 + (col >= 3) * 11;
            int subTreeAdj = (col < 1) * 0 + (col < 2 && col >= 1) * 1 + (col < 3 && col >= 2) * 2 + (col >= 3) * 3;
            int jid_subtree = jid + (col - subTreeAdj);
            T *src = &s_temp[0 + 72*jid + 6*jid_subtree]; T *dst = &s_temp[0 + 72*s_topology_helpers[jid] + 6*jid_subtree];
            // adjust for temp comps
            if (col >= 4) {
                col -= 4; src = &s_temp[1380 + 6*col]; dst = &s_temp[1524 + 6*col];
                int jid_selector = col / 6;
                // non-branching pointer selector
                jid = (jid_selector == 0) * 2 + (jid_selector == 1) * 5 + (jid_selector == 2) * 8 + (jid_selector == 3) * 11;
            }
            dst[row] = dot_prod<T,6,1,1>(&s_XImats[36*jid + 6*row],src);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("F[1] = X^T F[2]\n");
            printMat<T,6,12>(&s_temp[72],6);
            printf("Ia*X[2]\n");
            printMat<T,6,6>(&s_temp[1524],6);
            printf("F[4] = X^T F[5]\n");
            printMat<T,6,12>(&s_temp[288],6);
            printf("Ia*X[5]\n");
            printMat<T,6,6>(&s_temp[1560],6);
            printf("F[7] = X^T F[8]\n");
            printMat<T,6,12>(&s_temp[504],6);
            printf("Ia*X[8]\n");
            printMat<T,6,6>(&s_temp[1596],6);
            printf("F[10] = X^T F[11]\n");
            printMat<T,6,12>(&s_temp[720],6);
            printf("Ia*X[11]\n");
            printMat<T,6,6>(&s_temp[1632],6);
        }
        __syncthreads();
        // IA[parent_ind] += IA_Update_Temp * Xmat
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            int col = ind / 6; int row = ind % 6;
            int col_max6 = col % 6; int jid_ind = col / 6;
            // non-branching pointer selector
            int jid = (jid_ind == 0) * 2 + (jid_ind == 1) * 5 + (jid_ind == 2) * 8 + (jid_ind == 3) * 11;
            T * src = &s_temp[1524 + 36*jid_ind + row]; T * dst = &s_temp[864 + 36*s_topology_helpers[jid] + 6*col_max6 + row];
            *dst += dot_prod<T,6,6,1>(src,&s_XImats[36*jid + 6*col_max6]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("IA[1] = X^T*(Ia*X)\n");
            printMat<T,6,6>(&s_temp[900],6);
            printf("IA[4] = X^T*(Ia*X)\n");
            printMat<T,6,6>(&s_temp[1008],6);
            printf("IA[7] = X^T*(Ia*X)\n");
            printMat<T,6,6>(&s_temp[1116],6);
            printf("IA[10] = X^T*(Ia*X)\n");
            printMat<T,6,6>(&s_temp[1224],6);
        }
        __syncthreads();
        // backward pass updates where bfs_level is 1
        //     joints are: lf_hfe_joint, lh_hfe_joint, rf_hfe_joint, rh_hfe_joint
        //     links are: lf_upperleg, lh_upperleg, rf_upperleg, rh_upperleg
        // U = IA*S, D = S^T*U, DInv = 1/D, Minv[i,i] = Dinv
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 1 + (ind < 12 && ind >= 6) * 4 + (ind < 18 && ind >= 12) * 7 + (ind >= 18) * 10;
            int jid6 = 6*jid;
            s_temp[1296 + jid6 + row] = s_temp[864 + 6*jid6 + 6*2 + row];
            if(row == 2){
                s_temp[1368 + jid] = static_cast<T>(1)/s_temp[1296 + jid6 + 2];
                s_Minv[13 * jid] = s_temp[1368 + jid];
            }
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("U[1]\n"); printMat<T,1,6>(&s_temp[1296 + 6*1],1);
            printf("Dinv[1] = %f\n",s_temp[1368 + 1]);
            printf("U[4]\n"); printMat<T,1,6>(&s_temp[1296 + 6*4],1);
            printf("Dinv[4] = %f\n",s_temp[1368 + 4]);
            printf("U[7]\n"); printMat<T,1,6>(&s_temp[1296 + 6*7],1);
            printf("Dinv[7] = %f\n",s_temp[1368 + 7]);
            printf("U[10]\n"); printMat<T,1,6>(&s_temp[1296 + 6*10],1);
            printf("Dinv[10] = %f\n",s_temp[1368 + 10]);
            printf("Minv after Dinv setting before subtree\n"); printMat<T,12,12>(s_Minv,12);
        }
        __syncthreads();
        // Minv[i,subTreeInds] -= Dinv*F[i,Srow,SubTreeInds]
        // Temp Comp: F[i,:,subTreeInds] += U*Minv[i,subTreeInds] - to start Fparent Update
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 8; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind < 2) * 1 + (ind < 4 && ind >= 2) * 4 + (ind < 6 && ind >= 4) * 7 + (ind >= 6) * 10;
            int subTreeAdj = (ind < 2) * 0 + (ind < 4 && ind >= 2) * 2 + (ind < 6 && ind >= 4) * 4 + (ind >= 6) * 6;
            int jid_subtree = jid + (ind - subTreeAdj); int jid_subtree6 = 6*jid_subtree; int jid_subtreeN = 12*jid_subtree;
            s_Minv[jid_subtreeN + jid] -= s_temp[1368 + jid] * s_temp[0 + 72*jid + jid_subtree6 + 2];
            for(int row = 0; row < 6; row++) {
                s_temp[0 + 72*jid + jid_subtree6 + row] += s_temp[1296 + 6*jid + row] * s_Minv[jid_subtreeN + jid];
            }
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Minv after subtree updates\n"); printMat<T,12,12>(s_Minv,12);
            printf("F Temp += U*Minv[1]\n"); printMat<T,6,12>(&s_temp[72],6);
            printf("F Temp += U*Minv[4]\n"); printMat<T,6,12>(&s_temp[288],6);
            printf("F Temp += U*Minv[7]\n"); printMat<T,6,12>(&s_temp[504],6);
            printf("F Temp += U*Minv[10]\n"); printMat<T,6,12>(&s_temp[720],6);
        }
        __syncthreads();
        // Ia = IA - U^T Dinv U | to start IAparent Update
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind < 36) * 1 + (ind < 72 && ind >= 36) * 4 + (ind < 108 && ind >= 72) * 7 + (ind >= 108) * 10;
            int ind36 = (ind % 36); int row = ind36 % 6; int col = ind36 / 6; int jid6 = 6*jid;
            s_temp[1380 + ind] = s_temp[864 + 6*jid6 + ind36] - (s_temp[1296 + jid6 + row] * s_temp[1368 + jid] * s_temp[1296 + jid6 + col]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Ia[1]\n");
            printMat<T,6,6>(&s_temp[1380],6);
            printf("Ia[4]\n");
            printMat<T,6,6>(&s_temp[1416],6);
            printf("Ia[7]\n");
            printMat<T,6,6>(&s_temp[1452],6);
            printf("Ia[10]\n");
            printMat<T,6,6>(&s_temp[1488],6);
        }
        __syncthreads();
        // F[parent_ind,:,subTreeInds] += Xmat^T * F[ind,:,subTreeInds]
        // IA_Update_Temp = Xmat^T * Ia | for IAparent Update
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 192; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = ind / 6;
            // non-branching pointer selector
            int jid = (col < 2) * 1 + (col < 4 && col >= 2) * 4 + (col < 6 && col >= 4) * 7 + (col >= 6) * 10;
            int subTreeAdj = (col < 2) * 0 + (col < 4 && col >= 2) * 2 + (col < 6 && col >= 4) * 4 + (col >= 6) * 6;
            int jid_subtree = jid + (col - subTreeAdj);
            T *src = &s_temp[0 + 72*jid + 6*jid_subtree]; T *dst = &s_temp[0 + 72*s_topology_helpers[jid] + 6*jid_subtree];
            // adjust for temp comps
            if (col >= 8) {
                col -= 8; src = &s_temp[1380 + 6*col]; dst = &s_temp[1524 + 6*col];
                int jid_selector = col / 6;
                // non-branching pointer selector
                jid = (jid_selector == 0) * 1 + (jid_selector == 1) * 4 + (jid_selector == 2) * 7 + (jid_selector == 3) * 10;
            }
            dst[row] = dot_prod<T,6,1,1>(&s_XImats[36*jid + 6*row],src);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("F[0] = X^T F[1]\n");
            printMat<T,6,12>(&s_temp[0],6);
            printf("Ia*X[1]\n");
            printMat<T,6,6>(&s_temp[1524],6);
            printf("F[3] = X^T F[4]\n");
            printMat<T,6,12>(&s_temp[216],6);
            printf("Ia*X[4]\n");
            printMat<T,6,6>(&s_temp[1560],6);
            printf("F[6] = X^T F[7]\n");
            printMat<T,6,12>(&s_temp[432],6);
            printf("Ia*X[7]\n");
            printMat<T,6,6>(&s_temp[1596],6);
            printf("F[9] = X^T F[10]\n");
            printMat<T,6,12>(&s_temp[648],6);
            printf("Ia*X[10]\n");
            printMat<T,6,6>(&s_temp[1632],6);
        }
        __syncthreads();
        // IA[parent_ind] += IA_Update_Temp * Xmat
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            int col = ind / 6; int row = ind % 6;
            int col_max6 = col % 6; int jid_ind = col / 6;
            // non-branching pointer selector
            int jid = (jid_ind == 0) * 1 + (jid_ind == 1) * 4 + (jid_ind == 2) * 7 + (jid_ind == 3) * 10;
            T * src = &s_temp[1524 + 36*jid_ind + row]; T * dst = &s_temp[864 + 36*s_topology_helpers[jid] + 6*col_max6 + row];
            *dst += dot_prod<T,6,6,1>(src,&s_XImats[36*jid + 6*col_max6]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("IA[0] = X^T*(Ia*X)\n");
            printMat<T,6,6>(&s_temp[864],6);
            printf("IA[3] = X^T*(Ia*X)\n");
            printMat<T,6,6>(&s_temp[972],6);
            printf("IA[6] = X^T*(Ia*X)\n");
            printMat<T,6,6>(&s_temp[1080],6);
            printf("IA[9] = X^T*(Ia*X)\n");
            printMat<T,6,6>(&s_temp[1188],6);
        }
        __syncthreads();
        // backward pass updates where bfs_level is 0
        //     joints are: lf_haa_joint, lh_haa_joint, rf_haa_joint, rh_haa_joint
        //     links are: lf_hipassembly, lh_hipassembly, rf_hipassembly, rh_hipassembly
        // U = IA*S, D = S^T*U, DInv = 1/D, Minv[i,i] = Dinv
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 0 + (ind < 12 && ind >= 6) * 3 + (ind < 18 && ind >= 12) * 6 + (ind >= 18) * 9;
            int jid6 = 6*jid;
            s_temp[1296 + jid6 + row] = s_temp[864 + 6*jid6 + 6*2 + row];
            if(row == 2){
                s_temp[1368 + jid] = static_cast<T>(1)/s_temp[1296 + jid6 + 2];
                s_Minv[13 * jid] = s_temp[1368 + jid];
            }
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("U[0]\n"); printMat<T,1,6>(&s_temp[1296 + 6*0],1);
            printf("Dinv[0] = %f\n",s_temp[1368 + 0]);
            printf("U[3]\n"); printMat<T,1,6>(&s_temp[1296 + 6*3],1);
            printf("Dinv[3] = %f\n",s_temp[1368 + 3]);
            printf("U[6]\n"); printMat<T,1,6>(&s_temp[1296 + 6*6],1);
            printf("Dinv[6] = %f\n",s_temp[1368 + 6]);
            printf("U[9]\n"); printMat<T,1,6>(&s_temp[1296 + 6*9],1);
            printf("Dinv[9] = %f\n",s_temp[1368 + 9]);
            printf("Minv after Dinv setting before subtree\n"); printMat<T,12,12>(s_Minv,12);
        }
        __syncthreads();
        // Minv[i,subTreeInds] -= Dinv*F[i,Srow,SubTreeInds]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind < 3) * 0 + (ind < 6 && ind >= 3) * 3 + (ind < 9 && ind >= 6) * 6 + (ind >= 9) * 9;
            int subTreeAdj = (ind < 3) * 0 + (ind < 6 && ind >= 3) * 3 + (ind < 9 && ind >= 6) * 6 + (ind >= 9) * 9;
            int jid_subtree = jid + (ind - subTreeAdj); int jid_subtree6 = 6*jid_subtree; int jid_subtreeN = 12*jid_subtree;
            s_Minv[jid_subtreeN + jid] -= s_temp[1368 + jid] * s_temp[0 + 72*jid + jid_subtree6 + 2];
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Minv after subtree updates\n"); printMat<T,12,12>(s_Minv,12);
        }
        __syncthreads();
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Ia[0]\n");
            printMat<T,6,6>(&s_temp[1380],6);
            printf("Ia[3]\n");
            printMat<T,6,6>(&s_temp[1416],6);
            printf("Ia[6]\n");
            printMat<T,6,6>(&s_temp[1452],6);
            printf("Ia[9]\n");
            printMat<T,6,6>(&s_temp[1488],6);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("-------------------\n");
            printf("After Backward Pass\n");
            printf("-------------------\n");
            printf("U\n"); printMat<T,6,12>(&s_temp[1314],6);
            printf("Dinv\n"); printMat<T,1,12>(&s_temp[1368],1);
            printf("F[%d]\n",0); printMat<T,6,12>(&s_temp[0],6);
            printf("F[%d]\n",1); printMat<T,6,12>(&s_temp[72],6);
            printf("F[%d]\n",2); printMat<T,6,12>(&s_temp[144],6);
            printf("F[%d]\n",3); printMat<T,6,12>(&s_temp[216],6);
            printf("F[%d]\n",4); printMat<T,6,12>(&s_temp[288],6);
            printf("F[%d]\n",5); printMat<T,6,12>(&s_temp[360],6);
            printf("F[%d]\n",6); printMat<T,6,12>(&s_temp[432],6);
            printf("F[%d]\n",7); printMat<T,6,12>(&s_temp[504],6);
            printf("F[%d]\n",8); printMat<T,6,12>(&s_temp[576],6);
            printf("F[%d]\n",9); printMat<T,6,12>(&s_temp[648],6);
            printf("F[%d]\n",10); printMat<T,6,12>(&s_temp[720],6);
            printf("F[%d]\n",11); printMat<T,6,12>(&s_temp[792],6);
            printf("Minv\n"); printMat<T,12,12>(s_Minv,12);
            printf("-------------------\n");
        }
        __syncthreads();
        //
        // Forward Pass
        //   Note that due to the i: operation we need to go serially over all n
        //
        // forward pass for jid: 0
        // F[i,:,i:] = S * Minv[i,i:] as parent is base so rest is skipped
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 72; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = ind / 6;
            s_temp[0 + ind] = (row == 2) * s_Minv[0 + 12 * col];
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("F[i,:,i:] += S*Minv[i,i:] for i = %d\n",0);
            printMat<T,6,12>(&s_temp[0],6);
        }
        __syncthreads();
        // forward pass for jid: 1
        // Minv[i,i:] -= Dinv*U^T*Xmat*F[parent,:,i:] across cols i...N
        // F[i,:,i:] = S * Minv[i,i:] + Xmat*F[parent,:,i:] across cols i...N
        //   Start this step with F[i,:,i:] = Xmat*F[parent,:,i:] and
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 66; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col_ind = ind - row + 6;
            s_temp[72 + col_ind + row] = dot_prod<T,6,6,1>(&s_XImats[36 + row], &s_temp[0 + col_ind]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("F[i,:,i:] = Xmat*F[parent,:,i:] for i[1]\n");
            printMat<T,6,12>(&s_temp[0 + 72],6);
        }
        __syncthreads();
        //   Finish this step with Minv[i,i:] -= Dinv*U^T*F[i,:,i:]
        //     and then update F[i,:,i:] += S*Minv[i,i:]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 11; ind += blockDim.x*blockDim.y){
            int col_ind = ind + 1;
            T *s_Fcol = &s_temp[72 + 6*col_ind];
            s_Minv[12 * col_ind + 1] -= s_temp[1369] * dot_prod<T,6,1,1>(s_Fcol,&s_temp[1302]);
            s_Fcol[2] += s_Minv[12 * col_ind + 1];
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Minv[i,i:] -= Dinv*U^T*F[i,:,i:] for i = %d\n",1);
            printMat<T,12,12>(s_Minv,12);
            printf("F[i,:,i:] += S*Minv[i,i:]");
            printMat<T,6,12>(&s_temp[72],6);
        }
        __syncthreads();
        // forward pass for jid: 2
        // Minv[i,i:] -= Dinv*U^T*Xmat*F[parent,:,i:] across cols i...N
        // F[i,:,i:] = S * Minv[i,i:] + Xmat*F[parent,:,i:] across cols i...N
        //   Start this step with F[i,:,i:] = Xmat*F[parent,:,i:] and
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 60; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col_ind = ind - row + 12;
            s_temp[144 + col_ind + row] = dot_prod<T,6,6,1>(&s_XImats[72 + row], &s_temp[72 + col_ind]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("F[i,:,i:] = Xmat*F[parent,:,i:] for i[2]\n");
            printMat<T,6,12>(&s_temp[0 + 144],6);
        }
        __syncthreads();
        //   Finish this step with Minv[i,i:] -= Dinv*U^T*F[i,:,i:]
        //     and then update F[i,:,i:] += S*Minv[i,i:]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 10; ind += blockDim.x*blockDim.y){
            int col_ind = ind + 2;
            T *s_Fcol = &s_temp[144 + 6*col_ind];
            s_Minv[12 * col_ind + 2] -= s_temp[1370] * dot_prod<T,6,1,1>(s_Fcol,&s_temp[1308]);
            s_Fcol[2] += s_Minv[12 * col_ind + 2];
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Minv[i,i:] -= Dinv*U^T*F[i,:,i:] for i = %d\n",2);
            printMat<T,12,12>(s_Minv,12);
            printf("F[i,:,i:] += S*Minv[i,i:]");
            printMat<T,6,12>(&s_temp[144],6);
        }
        __syncthreads();
        // forward pass for jid: 3
        // F[i,:,i:] = S * Minv[i,i:] as parent is base so rest is skipped
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 54; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = ind / 6;
            s_temp[234 + ind] = (row == 2) * s_Minv[39 + 12 * col];
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("F[i,:,i:] += S*Minv[i,i:] for i = %d\n",3);
            printMat<T,6,12>(&s_temp[216],6);
        }
        __syncthreads();
        // forward pass for jid: 4
        // Minv[i,i:] -= Dinv*U^T*Xmat*F[parent,:,i:] across cols i...N
        // F[i,:,i:] = S * Minv[i,i:] + Xmat*F[parent,:,i:] across cols i...N
        //   Start this step with F[i,:,i:] = Xmat*F[parent,:,i:] and
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 48; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col_ind = ind - row + 24;
            s_temp[288 + col_ind + row] = dot_prod<T,6,6,1>(&s_XImats[144 + row], &s_temp[216 + col_ind]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("F[i,:,i:] = Xmat*F[parent,:,i:] for i[4]\n");
            printMat<T,6,12>(&s_temp[0 + 288],6);
        }
        __syncthreads();
        //   Finish this step with Minv[i,i:] -= Dinv*U^T*F[i,:,i:]
        //     and then update F[i,:,i:] += S*Minv[i,i:]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 8; ind += blockDim.x*blockDim.y){
            int col_ind = ind + 4;
            T *s_Fcol = &s_temp[288 + 6*col_ind];
            s_Minv[12 * col_ind + 4] -= s_temp[1372] * dot_prod<T,6,1,1>(s_Fcol,&s_temp[1320]);
            s_Fcol[2] += s_Minv[12 * col_ind + 4];
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Minv[i,i:] -= Dinv*U^T*F[i,:,i:] for i = %d\n",4);
            printMat<T,12,12>(s_Minv,12);
            printf("F[i,:,i:] += S*Minv[i,i:]");
            printMat<T,6,12>(&s_temp[288],6);
        }
        __syncthreads();
        // forward pass for jid: 5
        // Minv[i,i:] -= Dinv*U^T*Xmat*F[parent,:,i:] across cols i...N
        // F[i,:,i:] = S * Minv[i,i:] + Xmat*F[parent,:,i:] across cols i...N
        //   Start this step with F[i,:,i:] = Xmat*F[parent,:,i:] and
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 42; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col_ind = ind - row + 30;
            s_temp[360 + col_ind + row] = dot_prod<T,6,6,1>(&s_XImats[180 + row], &s_temp[288 + col_ind]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("F[i,:,i:] = Xmat*F[parent,:,i:] for i[5]\n");
            printMat<T,6,12>(&s_temp[0 + 360],6);
        }
        __syncthreads();
        //   Finish this step with Minv[i,i:] -= Dinv*U^T*F[i,:,i:]
        //     and then update F[i,:,i:] += S*Minv[i,i:]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 7; ind += blockDim.x*blockDim.y){
            int col_ind = ind + 5;
            T *s_Fcol = &s_temp[360 + 6*col_ind];
            s_Minv[12 * col_ind + 5] -= s_temp[1373] * dot_prod<T,6,1,1>(s_Fcol,&s_temp[1326]);
            s_Fcol[2] += s_Minv[12 * col_ind + 5];
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Minv[i,i:] -= Dinv*U^T*F[i,:,i:] for i = %d\n",5);
            printMat<T,12,12>(s_Minv,12);
            printf("F[i,:,i:] += S*Minv[i,i:]");
            printMat<T,6,12>(&s_temp[360],6);
        }
        __syncthreads();
        // forward pass for jid: 6
        // F[i,:,i:] = S * Minv[i,i:] as parent is base so rest is skipped
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = ind / 6;
            s_temp[468 + ind] = (row == 2) * s_Minv[78 + 12 * col];
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("F[i,:,i:] += S*Minv[i,i:] for i = %d\n",6);
            printMat<T,6,12>(&s_temp[432],6);
        }
        __syncthreads();
        // forward pass for jid: 7
        // Minv[i,i:] -= Dinv*U^T*Xmat*F[parent,:,i:] across cols i...N
        // F[i,:,i:] = S * Minv[i,i:] + Xmat*F[parent,:,i:] across cols i...N
        //   Start this step with F[i,:,i:] = Xmat*F[parent,:,i:] and
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 30; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col_ind = ind - row + 42;
            s_temp[504 + col_ind + row] = dot_prod<T,6,6,1>(&s_XImats[252 + row], &s_temp[432 + col_ind]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("F[i,:,i:] = Xmat*F[parent,:,i:] for i[7]\n");
            printMat<T,6,12>(&s_temp[0 + 504],6);
        }
        __syncthreads();
        //   Finish this step with Minv[i,i:] -= Dinv*U^T*F[i,:,i:]
        //     and then update F[i,:,i:] += S*Minv[i,i:]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 5; ind += blockDim.x*blockDim.y){
            int col_ind = ind + 7;
            T *s_Fcol = &s_temp[504 + 6*col_ind];
            s_Minv[12 * col_ind + 7] -= s_temp[1375] * dot_prod<T,6,1,1>(s_Fcol,&s_temp[1338]);
            s_Fcol[2] += s_Minv[12 * col_ind + 7];
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Minv[i,i:] -= Dinv*U^T*F[i,:,i:] for i = %d\n",7);
            printMat<T,12,12>(s_Minv,12);
            printf("F[i,:,i:] += S*Minv[i,i:]");
            printMat<T,6,12>(&s_temp[504],6);
        }
        __syncthreads();
        // forward pass for jid: 8
        // Minv[i,i:] -= Dinv*U^T*Xmat*F[parent,:,i:] across cols i...N
        // F[i,:,i:] = S * Minv[i,i:] + Xmat*F[parent,:,i:] across cols i...N
        //   Start this step with F[i,:,i:] = Xmat*F[parent,:,i:] and
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col_ind = ind - row + 48;
            s_temp[576 + col_ind + row] = dot_prod<T,6,6,1>(&s_XImats[288 + row], &s_temp[504 + col_ind]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("F[i,:,i:] = Xmat*F[parent,:,i:] for i[8]\n");
            printMat<T,6,12>(&s_temp[0 + 576],6);
        }
        __syncthreads();
        //   Finish this step with Minv[i,i:] -= Dinv*U^T*F[i,:,i:]
        //     and then update F[i,:,i:] += S*Minv[i,i:]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 4; ind += blockDim.x*blockDim.y){
            int col_ind = ind + 8;
            T *s_Fcol = &s_temp[576 + 6*col_ind];
            s_Minv[12 * col_ind + 8] -= s_temp[1376] * dot_prod<T,6,1,1>(s_Fcol,&s_temp[1344]);
            s_Fcol[2] += s_Minv[12 * col_ind + 8];
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Minv[i,i:] -= Dinv*U^T*F[i,:,i:] for i = %d\n",8);
            printMat<T,12,12>(s_Minv,12);
            printf("F[i,:,i:] += S*Minv[i,i:]");
            printMat<T,6,12>(&s_temp[576],6);
        }
        __syncthreads();
        // forward pass for jid: 9
        // F[i,:,i:] = S * Minv[i,i:] as parent is base so rest is skipped
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 18; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = ind / 6;
            s_temp[702 + ind] = (row == 2) * s_Minv[117 + 12 * col];
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("F[i,:,i:] += S*Minv[i,i:] for i = %d\n",9);
            printMat<T,6,12>(&s_temp[648],6);
        }
        __syncthreads();
        // forward pass for jid: 10
        // Minv[i,i:] -= Dinv*U^T*Xmat*F[parent,:,i:] across cols i...N
        // F[i,:,i:] = S * Minv[i,i:] + Xmat*F[parent,:,i:] across cols i...N
        //   Start this step with F[i,:,i:] = Xmat*F[parent,:,i:] and
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col_ind = ind - row + 60;
            s_temp[720 + col_ind + row] = dot_prod<T,6,6,1>(&s_XImats[360 + row], &s_temp[648 + col_ind]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("F[i,:,i:] = Xmat*F[parent,:,i:] for i[10]\n");
            printMat<T,6,12>(&s_temp[0 + 720],6);
        }
        __syncthreads();
        //   Finish this step with Minv[i,i:] -= Dinv*U^T*F[i,:,i:]
        //     and then update F[i,:,i:] += S*Minv[i,i:]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 2; ind += blockDim.x*blockDim.y){
            int col_ind = ind + 10;
            T *s_Fcol = &s_temp[720 + 6*col_ind];
            s_Minv[12 * col_ind + 10] -= s_temp[1378] * dot_prod<T,6,1,1>(s_Fcol,&s_temp[1356]);
            s_Fcol[2] += s_Minv[12 * col_ind + 10];
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Minv[i,i:] -= Dinv*U^T*F[i,:,i:] for i = %d\n",10);
            printMat<T,12,12>(s_Minv,12);
            printf("F[i,:,i:] += S*Minv[i,i:]");
            printMat<T,6,12>(&s_temp[720],6);
        }
        __syncthreads();
        // forward pass for jid: 11
        // Minv[i,i:] -= Dinv*U^T*Xmat*F[parent,:,i:] across cols i...N
        // F[i,:,i:] = S * Minv[i,i:] + Xmat*F[parent,:,i:] across cols i...N
        //   Start this step with F[i,:,i:] = Xmat*F[parent,:,i:] and
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 6; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col_ind = ind - row + 66;
            s_temp[792 + col_ind + row] = dot_prod<T,6,6,1>(&s_XImats[396 + row], &s_temp[720 + col_ind]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("F[i,:,i:] = Xmat*F[parent,:,i:] for i[11]\n");
            printMat<T,6,12>(&s_temp[0 + 792],6);
        }
        __syncthreads();
        //   Finish this step with Minv[i,i:] -= Dinv*U^T*F[i,:,i:]
        //     and then update F[i,:,i:] += S*Minv[i,i:]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 1; ind += blockDim.x*blockDim.y){
            int col_ind = ind + 11;
            T *s_Fcol = &s_temp[792 + 6*col_ind];
            s_Minv[12 * col_ind + 11] -= s_temp[1379] * dot_prod<T,6,1,1>(s_Fcol,&s_temp[1362]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Minv[i,i:] -= Dinv*U^T*F[i,:,i:] for i = %d\n",11);
            printMat<T,12,12>(s_Minv,12);
        }
        __syncthreads();
    }

    /**
     * Compute the inverse of the mass matrix
     *
     * Notes:
     *   Outputs a SYMMETRIC_UPPER triangular matrix for Minv
     *
     * @param s_Minv is a pointer to memory for the final result
     * @param s_q is the vector of joint positions
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     */
    template <typename T>
    __device__
    void direct_minv_device(T *s_Minv, const T *s_q, const robotModel<T> *d_robotModel){
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
        direct_minv_inner<T>(s_Minv, s_q, s_XImats, s_topology_helpers, s_temp);
    }

    /**
     * Compute the inverse of the mass matrix
     *
     * Notes:
     *   Outputs a SYMMETRIC_UPPER triangular matrix for Minv
     *
     * @param d_Minv is a pointer to memory for the final result
     * @param d_q is the vector of joint positions
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void direct_minv_kernel_single_timing(T *d_Minv, const T *d_q, const int stride_q, const robotModel<T> *d_robotModel, const int NUM_TIMESTEPS){
        __shared__ T s_q[12];
        __shared__ T s_Minv[144];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        // load to shared mem
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
            s_q[ind] = d_q[ind];
        }
        __syncthreads();
        // compute with NUM_TIMESTEPS as NUM_REPS for timing
        for (int rep = 0; rep < NUM_TIMESTEPS; rep++){
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            direct_minv_inner<T>(s_Minv, s_q, s_XImats, s_topology_helpers, s_temp);
        }
        // save down to global
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            d_Minv[ind] = s_Minv[ind];
        }
        __syncthreads();
    }

    /**
     * Compute the inverse of the mass matrix
     *
     * Notes:
     *   Outputs a SYMMETRIC_UPPER triangular matrix for Minv
     *
     * @param d_Minv is a pointer to memory for the final result
     * @param d_q is the vector of joint positions
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void direct_minv_kernel(T *d_Minv, const T *d_q, const int stride_q, const robotModel<T> *d_robotModel, const int NUM_TIMESTEPS){
        __shared__ T s_q[12];
        __shared__ T s_Minv[144];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        for(int k = blockIdx.x + blockIdx.y*gridDim.x; k < NUM_TIMESTEPS; k += gridDim.x*gridDim.y){
            // load to shared mem
            const T *d_q_k = &d_q[k*stride_q];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
                s_q[ind] = d_q_k[ind];
            }
            __syncthreads();
            // compute
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            direct_minv_inner<T>(s_Minv, s_q, s_XImats, s_topology_helpers, s_temp);
            __syncthreads();
            // save down to global
            T *d_Minv_k = &d_Minv[k*144];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
                d_Minv_k[ind] = s_Minv[ind];
            }
            __syncthreads();
        }
    }

    /**
     * Compute the inverse of the mass matrix
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_COMPRESSED_MEM = false>
    __host__
    void direct_minv(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const int num_timesteps,
                     const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        // start code with memory transfer
        int stride_q;
        if (USE_COMPRESSED_MEM) {stride_q = NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q,hd_data->h_q,stride_q*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        else {stride_q = 3*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        gpuErrchk(cudaDeviceSynchronize());
        // then call the kernel
        if (USE_COMPRESSED_MEM) {direct_minv_kernel<T><<<block_dimms,thread_dimms,MINV_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_Minv,hd_data->d_q,stride_q,d_robotModel,num_timesteps);}
        else                    {direct_minv_kernel<T><<<block_dimms,thread_dimms,MINV_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_Minv,hd_data->d_q_qd_u,stride_q,d_robotModel,num_timesteps);}
        gpuErrchk(cudaDeviceSynchronize());
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_Minv,hd_data->d_Minv,NUM_JOINTS*NUM_JOINTS*num_timesteps*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Compute the inverse of the mass matrix
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_COMPRESSED_MEM = false>
    __host__
    void direct_minv_single_timing(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const int num_timesteps,
                                   const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        // start code with memory transfer
        int stride_q;
        if (USE_COMPRESSED_MEM) {stride_q = NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q,hd_data->h_q,stride_q*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        else {stride_q = 3*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        gpuErrchk(cudaDeviceSynchronize());
        // then call the kernel
        struct timespec start, end; clock_gettime(CLOCK_MONOTONIC,&start);
        if (USE_COMPRESSED_MEM) {direct_minv_kernel_single_timing<T><<<block_dimms,thread_dimms,MINV_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_Minv,hd_data->d_q,stride_q,d_robotModel,num_timesteps);}
        else                    {direct_minv_kernel_single_timing<T><<<block_dimms,thread_dimms,MINV_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_Minv,hd_data->d_q_qd_u,stride_q,d_robotModel,num_timesteps);}
        gpuErrchk(cudaDeviceSynchronize());
        clock_gettime(CLOCK_MONOTONIC,&end);
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_Minv,hd_data->d_Minv,NUM_JOINTS*NUM_JOINTS*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
        printf("Single Call Minv %fus\n",time_delta_us_timespec(start,end)/static_cast<double>(num_timesteps));
    }

    /**
     * Compute the inverse of the mass matrix
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_COMPRESSED_MEM = false>
    __host__
    void direct_minv_compute_only(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const int num_timesteps,
                                  const dim3 block_dimms, const dim3 thread_dimms) {
        int stride_q = USE_COMPRESSED_MEM ? NUM_JOINTS: 3*NUM_JOINTS;
        // then call the kernel
        if (USE_COMPRESSED_MEM) {direct_minv_kernel<T><<<block_dimms,thread_dimms,MINV_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_Minv,hd_data->d_q,stride_q,d_robotModel,num_timesteps);}
        else                    {direct_minv_kernel<T><<<block_dimms,thread_dimms,MINV_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_Minv,hd_data->d_q_qd_u,stride_q,d_robotModel,num_timesteps);}
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Finish the forward dynamics computation with qdd = Minv*(u-c)
     *
     * Notes:
     *   Assumes s_Minv and s_c are already computed
     *
     * @param s_qdd is a pointer to memory for the final result
     * @param s_u is the vector of joint input torques
     * @param s_c is the bias vector
     * @param s_Minv is the inverse mass matrix
     */
    template <typename T>
    __device__
    void forward_dynamics_finish(T *s_qdd, const T *s_u, const T *s_c, const T *s_Minv) {
        for(int row = threadIdx.x + threadIdx.y*blockDim.x; row < 12; row += blockDim.x*blockDim.y){
            T val = static_cast<T>(0);
            for(int col = 0; col < 12; col++) {
                // account for the fact that Minv is an SYMMETRIC_UPPER triangular matrix
                int index = (row <= col) * (col * 12 + row) + (row > col) * (row * 12 + col);
                val += s_Minv[index] * (s_u[col] - s_c[col]);
            }
            s_qdd[row] = val;
        }
    }

    /**
     * Computes forward dynamics
     *
     * Notes:
     *   Assumes s_XImats is updated already for the current s_q
     *
     * @param s_qdd is a pointer to memory for the final result
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param s_u is the vector of joint input torques
     * @param s_XImats is the (shared) memory holding the updated XI matricies for the given s_q
     * @param s_topology_helpers is the (shared) memory destination location for the topology_helpers
     * @param s_temp is the pointer to the shared memory needed of size: 2112
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void forward_dynamics_inner(T *s_qdd, const T *s_q, const T *s_qd, const T *s_u, T *s_XImats, int *s_topology_helpers, T *s_temp, const T gravity) {
        direct_minv_inner<T>(s_temp, s_q, s_XImats, s_topology_helpers, &s_temp[144]);
        inverse_dynamics_inner<T>(&s_temp[144], &s_temp[156], s_q, s_qd, s_XImats, s_topology_helpers, &s_temp[372], gravity);
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Minv\n"); printMat<T,12,12>(s_temp,12);
            printf("u\n"); printMat<T,1,12>(s_u,1);printf("c\n"); printMat<T,1,12>(&s_temp[144],1);
        }
        __syncthreads();
        forward_dynamics_finish<T>(s_qdd, s_u, &s_temp[144], s_temp);
    }

    /**
     * Computes forward dynamics
     *
     * @param s_qdd is a pointer to memory for the final result
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param s_u is the vector of joint input torques
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void forward_dynamics_device(T *s_qdd, const T *s_q, const T *s_qd, const T *s_u, const robotModel<T> *d_robotModel, const T gravity) {
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
        forward_dynamics_inner<T>(s_qdd, s_q, s_qd, s_u, s_XImats, s_topology_helpers, s_temp, gravity);
    }

    /**
     * Computes forward dynamics
     *
     * @param d_qdd is a pointer to memory for the final result
     * @param d_q_qd_u is the vector of joint positions, velocities, and input torques
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void forward_dynamics_kernel_single_timing(T *d_qdd, const T *d_q_qd_u, const int stride_q_qd_u, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_q_qd_u[36]; T *s_q = s_q_qd_u; T *s_qd = &s_q_qd_u[12]; T *s_u = &s_q_qd_u[24];
        __shared__ T s_qdd[12];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        // load to shared mem
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            s_q_qd_u[ind] = d_q_qd_u[ind];
        }
        __syncthreads();
        // compute with NUM_TIMESTEPS as NUM_REPS for timing
        for (int rep = 0; rep < NUM_TIMESTEPS; rep++){
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            forward_dynamics_inner<T>(s_qdd, s_q, s_qd, s_u, s_XImats, s_topology_helpers, s_temp, gravity);
        }
        // save down to global
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
            d_qdd[ind] = s_qdd[ind];
        }
        __syncthreads();
    }

    /**
     * Computes forward dynamics
     *
     * @param d_qdd is a pointer to memory for the final result
     * @param d_q_qd_u is the vector of joint positions, velocities, and input torques
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void forward_dynamics_kernel(T *d_qdd, const T *d_q_qd_u, const int stride_q_qd_u, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_q_qd_u[36]; T *s_q = s_q_qd_u; T *s_qd = &s_q_qd_u[12]; T *s_u = &s_q_qd_u[24];
        __shared__ T s_qdd[12];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        for(int k = blockIdx.x + blockIdx.y*gridDim.x; k < NUM_TIMESTEPS; k += gridDim.x*gridDim.y){
            // load to shared mem
            const T *d_q_qd_u_k = &d_q_qd_u[k*stride_q_qd_u];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
                s_q_qd_u[ind] = d_q_qd_u_k[ind];
            }
            __syncthreads();
            // compute
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            forward_dynamics_inner<T>(s_qdd, s_q, s_qd, s_u, s_XImats, s_topology_helpers, s_temp, gravity);
            __syncthreads();
            // save down to global
            T *d_qdd_k = &d_qdd[k*12];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
                d_qdd_k[ind] = s_qdd[ind];
            }
            __syncthreads();
        }
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T>
    __host__
    void forward_dynamics(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                          const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        int stride_q_qd_u = 3*NUM_JOINTS;
        // start code with memory transfer
        gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q_qd_u*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[0]));
        gpuErrchk(cudaDeviceSynchronize());
        // then call the kernel
        forward_dynamics_kernel<T><<<block_dimms,thread_dimms,FD_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_qdd,hd_data->d_q_qd_u,stride_q_qd_u,d_robotModel,gravity,num_timesteps);
        gpuErrchk(cudaDeviceSynchronize());
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_qdd,hd_data->d_qdd,NUM_JOINTS*num_timesteps*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T>
    __host__
    void forward_dynamics_single_timing(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                                        const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        int stride_q_qd_u = 3*NUM_JOINTS;
        // start code with memory transfer
        gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q_qd_u*sizeof(T),cudaMemcpyHostToDevice,streams[0]));
        gpuErrchk(cudaDeviceSynchronize());
        // then call the kernel
        struct timespec start, end; clock_gettime(CLOCK_MONOTONIC,&start);
        forward_dynamics_kernel_single_timing<T><<<block_dimms,thread_dimms,FD_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_qdd,hd_data->d_q_qd_u,stride_q_qd_u,d_robotModel,gravity,num_timesteps);
        gpuErrchk(cudaDeviceSynchronize());
        clock_gettime(CLOCK_MONOTONIC,&end);
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_qdd,hd_data->d_qdd,NUM_JOINTS*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
        printf("Single Call FD %fus\n",time_delta_us_timespec(start,end)/static_cast<double>(num_timesteps));
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T>
    __host__
    void forward_dynamics_compute_only(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                                       const dim3 block_dimms, const dim3 thread_dimms) {
        int stride_q_qd_u = 3*NUM_JOINTS;
        // then call the kernel
        forward_dynamics_kernel<T><<<block_dimms,thread_dimms,FD_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_qdd,hd_data->d_q_qd_u,stride_q_qd_u,d_robotModel,gravity,num_timesteps);
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Computes the gradient of inverse dynamics
     *
     * Notes:
     *   Assumes s_XImats is updated already for the current s_q
     *
     * @param s_dc_du is a pointer to memory for the final result of size 2*NUM_JOINTS*NUM_JOINTS = 288
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param s_vaf are the helper intermediate variables computed by inverse_dynamics
     * @param s_XImats is the (shared) memory holding the updated XI matricies for the given s_q
     * @param s_topology_helpers is the (shared) memory destination location for the topology_helpers
     * @param s_temp is a pointer to helper shared memory of size 66*NUM_JOINTS + 6*sparse_dv,da,df_col_needs = 1800
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void inverse_dynamics_gradient_inner(T *s_dc_du, const T *s_q, const T *s_qd, const T *s_vaf, T *s_XImats, int *s_topology_helpers, T *s_temp, const T gravity) {
        //
        // dv and da need 24 cols per dq,dqd
        // df needs 36 cols per dq,dqd
        //    out of a possible 144 cols per dq,dqd
        // Gradients are stored compactly as dv_i/dq_[0...a], dv_i+1/dq_[0...b], etc
        //    where a and b are the needed number of columns
        //
        // Temp memory offsets are as follows:
        // T *s_dv_dq = &s_temp[0]; T *s_dv_dqd = &s_temp[144]; T *s_da_dq = &s_temp[288];
        // T *s_da_dqd = &s_temp[432]; T *s_df_dq = &s_temp[576]; T *s_df_dqd = &s_temp[792];
        // T *s_FxvI = &s_temp[1008]; T *s_MxXv = &s_temp[1440]; T *s_MxXa = &s_temp[1512];
        // T *s_Mxv = &s_temp[1584]; T *s_Mxf = &s_temp[1656]; T *s_Iv = &s_temp[1728];
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("-------------------------\n");
            printf("Validating Function Inputs\n");
            printf("-------------------------\n");
            printf("q\n"); printMat<T,1,12>(s_q,1);
            printf("qd\n"); printMat<T,1,12>(s_qd,1);
            printf("vaf-v\n"); printMat<T,6,12>(s_vaf,6);
            printf("vaf-a\n"); printMat<T,6,12>(&s_vaf[6*12],6);
            printf("vaf-f\n"); printMat<T,6,12>(&s_vaf[12*12],6);
            for (int i = 0; i < 12; i++){printf("X[%d]\n",i); printMat<T,6,6>(&s_XImats[36*i],6);}
            for (int i = 0; i < 12; i++){printf("I[%d]\n",i); printMat<T,6,6>(&s_XImats[36*(i+12)],6);}
            printf("-------------------------\n");
        }
        __syncthreads();
        //
        // Initial Temp Comps
        //
        // First compute Imat*v and Xmat*v_parent, Xmat*a_parent (store in FxvI for now)
        // Note that if jid_parent == -1 then v_parent = 0 and a_parent = gravity
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 216; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = ind / 6; int jid = col % 12; int jid6 = 6*jid;
            bool parentIsBase = s_topology_helpers[jid] == -1;
            bool comp1 = col < 12; bool comp3 = col >= 24;
            int XIOffset  =  comp1 * 432 + 6*jid6 + row; // rowCol of I (comp1) or X (comp 2 and 3)
            int vaOffset  = comp1 * jid6 + !comp1 * 6*s_topology_helpers[jid] + comp3 * 72; // v_i (comp1) or va_parent (comp 2 and 3)
            int dstOffset = comp1 * 1728 + !comp1 * 1008 + comp3 * 72 + jid6 + row; // rowCol of dst
            s_temp[dstOffset] = (parentIsBase && !comp1) ? comp3 * s_XImats[XIOffset + 30] * gravity : 
                                                           dot_prod<T,6,6,1>(&s_XImats[XIOffset],&s_vaf[vaOffset]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("-------------------------\n");
            printf("Temp Comps Part 1\n");
            printf("-------------------------\n");
            printf("Iv\n"); printMat<T,6,12>(&s_temp[1728],6);
            printf("Xv\n"); printMat<T,6,12>(&s_temp[1008],6);
            printf("Xa\n"); printMat<T,6,12>(&s_temp[1080],6);
            printf("-------------------------\n");
        }
        __syncthreads();
        // Then compute Mx(Xv), Mx(Xa), Mx(v), Mx(f)
        for(int col = threadIdx.x + threadIdx.y*blockDim.x; col < 48; col += blockDim.x*blockDim.y){
            int dof_id = col / 4; int selector = col % 4; int dof_id6 = 6*dof_id;
            int jid6 = dof_id6;
            // branch to get pointer locations
            int dstOffset; const T * src;
                 if (selector == 0){ dstOffset = 1440; src = &s_temp[1008]; }
            else if (selector == 1){ dstOffset = 1512; src = &s_temp[1080]; }
            else if (selector == 2){ dstOffset = 1584; src = &s_vaf[0]; }
            else              { dstOffset = 1656; src = &s_vaf[144]; }
            mx2<T>(&s_temp[dstOffset + dof_id6], &src[jid6]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("-------------------------\n");
            printf("Temp Comps Part 2\n");
            printf("-------------------------\n");
            printf("Mx(Xv)\n"); printMat<T,6,12>(&s_temp[1440],6);
            printf("Mx(Xa)\n"); printMat<T,6,12>(&s_temp[1512],6);
            printf("Mx(v)\n"); printMat<T,6,12>(&s_temp[1584],6);
            printf("Mx(f)\n"); printMat<T,6,12>(&s_temp[1656],6);
            printf("-------------------------\n");
        }
        __syncthreads();
        //
        // Forward Pass
        //
        // We start with dv/du noting that we only have values
        //    for ancestors and for the current index else 0
        // dv/du where bfs_level is 0
        //     joints are: lf_haa_joint, lh_haa_joint, rf_haa_joint, rh_haa_joint
        //     links are: lf_hipassembly, lh_hipassembly, rf_hipassembly, rh_hipassembly
        // when parent is base dv_dq = 0, dv_dqd = S
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 48; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = ind / 6; int col_du = col % 4; bool dq_flag = col == col_du;
            // non-branching pointer selector
            int jid = (col_du < 1) * 0 + (col_du < 2 && col_du >= 1) * 3 + (col_du < 3 && col_du >= 2) * 6 + (col_du >= 3) * 9;
            int du_offset = dq_flag ? 0 : 144;
            s_temp[du_offset + 6*(s_topology_helpers[36 + jid] + jid) + row] = (!dq_flag && row == 2) * static_cast<T>(1);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("-------------------------\n");
            printf("dv/du in bfs waves\n");
            printf("-------------------------\n");
            printf("dv/du in for bfs wave[%d]\n",0);
            printf("dv[%d]/dq\n",0);
            printMat<T,6,1>(&s_temp[0],6);
            printf("dv[%d]/dqd\n",0);
            printMat<T,6,1>(&s_temp[144],6);
            printf("dv[%d]/dq\n",3);
            printMat<T,6,1>(&s_temp[36],6);
            printf("dv[%d]/dqd\n",3);
            printMat<T,6,1>(&s_temp[180],6);
            printf("dv[%d]/dq\n",6);
            printMat<T,6,1>(&s_temp[72],6);
            printf("dv[%d]/dqd\n",6);
            printMat<T,6,1>(&s_temp[216],6);
            printf("dv[%d]/dq\n",9);
            printMat<T,6,1>(&s_temp[108],6);
            printf("dv[%d]/dqd\n",9);
            printMat<T,6,1>(&s_temp[252],6);
        }
        __syncthreads();
        // dv/du where bfs_level is 1
        //     joints are: lf_hfe_joint, lh_hfe_joint, rf_hfe_joint, rh_hfe_joint
        //     links are: lf_upperleg, lh_upperleg, rf_upperleg, rh_upperleg
        // dv/du = Xmat*dv_parent/du + {Mx(Xv) or S for col ind}
        // first compute dv/du = Xmat*dv_parent/du
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 48; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = ind / 6; int col_du = col % 4; int col_jid = col_du % 1;
            int dq_flag = col == col_du;
            // non-branching pointer selector
            int jid = (col_du < 1) * 1 + (col_du < 2 && col_du >= 1) * 4 + (col_du < 3 && col_du >= 2) * 7 + (col_du >= 3) * 10;
            int du_col_offset = dq_flag * 0 + !dq_flag * 144 + 6 * col_jid;
            s_temp[du_col_offset + 6*(s_topology_helpers[36 + jid] + jid) + row] = 
                dot_prod<T,6,6,1>(&s_XImats[36*jid + row],&s_temp[du_col_offset + 6*(s_topology_helpers[36 + s_topology_helpers[jid]] + s_topology_helpers[jid])]);
            // then add {Mx(Xv) or S for col ind}
            s_temp[du_col_offset + 6*(s_topology_helpers[36 + jid] + jid) + 6 + row] = 
                dq_flag * s_temp[1440 + 6*jid + row] + (!dq_flag && row == 2) * static_cast<T>(1);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("dv/du in for bfs wave[%d]\n",1);
            printf("dv[%d]/dq\n",1);
            printMat<T,6,2>(&s_temp[6],6);
            printf("dv[%d]/dqd\n",1);
            printMat<T,6,2>(&s_temp[150],6);
            printf("dv[%d]/dq\n",4);
            printMat<T,6,2>(&s_temp[42],6);
            printf("dv[%d]/dqd\n",4);
            printMat<T,6,2>(&s_temp[186],6);
            printf("dv[%d]/dq\n",7);
            printMat<T,6,2>(&s_temp[78],6);
            printf("dv[%d]/dqd\n",7);
            printMat<T,6,2>(&s_temp[222],6);
            printf("dv[%d]/dq\n",10);
            printMat<T,6,2>(&s_temp[114],6);
            printf("dv[%d]/dqd\n",10);
            printMat<T,6,2>(&s_temp[258],6);
        }
        __syncthreads();
        // dv/du where bfs_level is 2
        //     joints are: lf_kfe_joint, lh_kfe_joint, rf_kfe_joint, rh_kfe_joint
        //     links are: lf_lowerleg, lh_lowerleg, rf_lowerleg, rh_lowerleg
        // dv/du = Xmat*dv_parent/du + {Mx(Xv) or S for col ind}
        // first compute dv/du = Xmat*dv_parent/du
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 96; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = ind / 6; int col_du = col % 8; int col_jid = col_du % 2;
            int dq_flag = col == col_du;
            // non-branching pointer selector
            int jid = (col_du < 2) * 2 + (col_du < 4 && col_du >= 2) * 5 + (col_du < 6 && col_du >= 4) * 8 + (col_du >= 6) * 11;
            int du_col_offset = dq_flag * 0 + !dq_flag * 144 + 6 * col_jid;
            s_temp[du_col_offset + 6*(s_topology_helpers[36 + jid] + jid) + row] = 
                dot_prod<T,6,6,1>(&s_XImats[36*jid + row],&s_temp[du_col_offset + 6*(s_topology_helpers[36 + s_topology_helpers[jid]] + s_topology_helpers[jid])]);
            // then add {Mx(Xv) or S for col ind}
            if (col_jid == 1) {
                s_temp[du_col_offset + 6*(s_topology_helpers[36 + jid] + jid) + 6 + row] = 
                    dq_flag * s_temp[1440 + 6*jid + row] + (!dq_flag && row == 2) * static_cast<T>(1);
            }
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("dv/du in for bfs wave[%d]\n",2);
            printf("dv[%d]/dq\n",2);
            printMat<T,6,3>(&s_temp[18],6);
            printf("dv[%d]/dqd\n",2);
            printMat<T,6,3>(&s_temp[162],6);
            printf("dv[%d]/dq\n",5);
            printMat<T,6,3>(&s_temp[54],6);
            printf("dv[%d]/dqd\n",5);
            printMat<T,6,3>(&s_temp[198],6);
            printf("dv[%d]/dq\n",8);
            printMat<T,6,3>(&s_temp[90],6);
            printf("dv[%d]/dqd\n",8);
            printMat<T,6,3>(&s_temp[234],6);
            printf("dv[%d]/dq\n",11);
            printMat<T,6,3>(&s_temp[126],6);
            printf("dv[%d]/dqd\n",11);
            printMat<T,6,3>(&s_temp[270],6);
        }
        __syncthreads();
        // start da/du by setting = MxS(dv/du)*qd + {MxXa, Mxv} for all n in parallel
        // start with da/du = MxS(dv/du)*qd
        for(int col = threadIdx.x + threadIdx.y*blockDim.x; col < 48; col += blockDim.x*blockDim.y){
            int col_du = col % 24;
            // non-branching pointer selector
            int jid = (col_du < 1) * 0 + (col_du < 3 && col_du >= 1) * 1 + (col_du < 6 && col_du >= 3) * 2 + (col_du < 7 && col_du >= 6) * 3 + (col_du < 9 && col_du >= 7) * 4 + (col_du < 12 && col_du >= 9) * 5 + (col_du < 13 && col_du >= 12) * 6 + (col_du < 15 && col_du >= 13) * 7 + (col_du < 18 && col_du >= 15) * 8 + (col_du < 19 && col_du >= 18) * 9 + (col_du < 21 && col_du >= 19) * 10 + (col_du >= 21) * 11;
            mx2_scaled<T>(&s_temp[288 + 6*col], &s_temp[0 + 6*col], s_qd[jid]);
            // then add {MxXa, Mxv} to the appropriate column
            int dq_flag = col == col_du; int src_offset = dq_flag * 1512 + !dq_flag * 1584 + 6*jid;
            if(col_du == ((s_topology_helpers[36 + jid + 1] + jid + 1) - 1)){
                for(int row = 0; row < 6; row++){
                    s_temp[288 + 6*col + row] += s_temp[src_offset + row];
                }
            }
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("-------------------------\n");
            printf("da/du part 1 = MxS(dv/du)*qd + {MxXa, Mxf}\n");
            printf("-------------------------\n");
            printf("da[%d]/dq\n",0);
            printMat<T,6,1>(&s_temp[288],6);
            printf("da[%d]/dqd\n",0);
            printMat<T,6,1>(&s_temp[432],6);
            printf("da[%d]/dq\n",1);
            printMat<T,6,2>(&s_temp[294],6);
            printf("da[%d]/dqd\n",1);
            printMat<T,6,2>(&s_temp[438],6);
            printf("da[%d]/dq\n",2);
            printMat<T,6,3>(&s_temp[306],6);
            printf("da[%d]/dqd\n",2);
            printMat<T,6,3>(&s_temp[450],6);
            printf("da[%d]/dq\n",3);
            printMat<T,6,1>(&s_temp[324],6);
            printf("da[%d]/dqd\n",3);
            printMat<T,6,1>(&s_temp[468],6);
            printf("da[%d]/dq\n",4);
            printMat<T,6,2>(&s_temp[330],6);
            printf("da[%d]/dqd\n",4);
            printMat<T,6,2>(&s_temp[474],6);
            printf("da[%d]/dq\n",5);
            printMat<T,6,3>(&s_temp[342],6);
            printf("da[%d]/dqd\n",5);
            printMat<T,6,3>(&s_temp[486],6);
            printf("da[%d]/dq\n",6);
            printMat<T,6,1>(&s_temp[360],6);
            printf("da[%d]/dqd\n",6);
            printMat<T,6,1>(&s_temp[504],6);
            printf("da[%d]/dq\n",7);
            printMat<T,6,2>(&s_temp[366],6);
            printf("da[%d]/dqd\n",7);
            printMat<T,6,2>(&s_temp[510],6);
            printf("da[%d]/dq\n",8);
            printMat<T,6,3>(&s_temp[378],6);
            printf("da[%d]/dqd\n",8);
            printMat<T,6,3>(&s_temp[522],6);
            printf("da[%d]/dq\n",9);
            printMat<T,6,1>(&s_temp[396],6);
            printf("da[%d]/dqd\n",9);
            printMat<T,6,1>(&s_temp[540],6);
            printf("da[%d]/dq\n",10);
            printMat<T,6,2>(&s_temp[402],6);
            printf("da[%d]/dqd\n",10);
            printMat<T,6,2>(&s_temp[546],6);
            printf("da[%d]/dq\n",11);
            printMat<T,6,3>(&s_temp[414],6);
            printf("da[%d]/dqd\n",11);
            printMat<T,6,3>(&s_temp[558],6);
        }
        __syncthreads();
        // Finish da/du with parent updates noting that we only have values
        //    for ancestors and for the current index and nothing for bfs 0
        // da/du where bfs_level is 1
        //     joints are: lf_hfe_joint, lh_hfe_joint, rf_hfe_joint, rh_hfe_joint
        //     links are: lf_upperleg, lh_upperleg, rf_upperleg, rh_upperleg
        // da/du += Xmat*da_parent/du
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 48; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = ind / 6; int col_du = col % 4;
            int dq_flag = col == col_du; int col_jid = col_du % 1;
            // non-branching pointer selector
            int jid = (col_du < 1) * 1 + (col_du < 2 && col_du >= 1) * 4 + (col_du < 3 && col_du >= 2) * 7 + (col_du >= 3) * 10;
            int du_col_offset = dq_flag * 288 + !dq_flag * 432 + 6 * col_jid;
            s_temp[du_col_offset + 6*(s_topology_helpers[36 + jid] + jid) + row] += 
                dot_prod<T,6,6,1>(&s_XImats[36*jid + row],&s_temp[du_col_offset + 6*(s_topology_helpers[36 + s_topology_helpers[jid]] + s_topology_helpers[jid])]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("-------------------------\n");
            printf("da/du in bfs waves\n");
            printf("-------------------------\n");
            printf("da/du for bfs wave[%d]\n",1);
            printf("da[%d]/dq\n",1);
            printMat<T,6,2>(&s_temp[294],6);
            printf("da[%d]/dqd\n",1);
            printMat<T,6,2>(&s_temp[438],6);
            printf("da[%d]/dq\n",4);
            printMat<T,6,2>(&s_temp[330],6);
            printf("da[%d]/dqd\n",4);
            printMat<T,6,2>(&s_temp[474],6);
            printf("da[%d]/dq\n",7);
            printMat<T,6,2>(&s_temp[366],6);
            printf("da[%d]/dqd\n",7);
            printMat<T,6,2>(&s_temp[510],6);
            printf("da[%d]/dq\n",10);
            printMat<T,6,2>(&s_temp[402],6);
            printf("da[%d]/dqd\n",10);
            printMat<T,6,2>(&s_temp[546],6);
        }
        __syncthreads();
        // da/du where bfs_level is 2
        //     joints are: lf_kfe_joint, lh_kfe_joint, rf_kfe_joint, rh_kfe_joint
        //     links are: lf_lowerleg, lh_lowerleg, rf_lowerleg, rh_lowerleg
        // da/du += Xmat*da_parent/du
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 96; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = ind / 6; int col_du = col % 8;
            int dq_flag = col == col_du; int col_jid = col_du % 2;
            // non-branching pointer selector
            int jid = (col_du < 2) * 2 + (col_du < 4 && col_du >= 2) * 5 + (col_du < 6 && col_du >= 4) * 8 + (col_du >= 6) * 11;
            int du_col_offset = dq_flag * 288 + !dq_flag * 432 + 6 * col_jid;
            s_temp[du_col_offset + 6*(s_topology_helpers[36 + jid] + jid) + row] += 
                dot_prod<T,6,6,1>(&s_XImats[36*jid + row],&s_temp[du_col_offset + 6*(s_topology_helpers[36 + s_topology_helpers[jid]] + s_topology_helpers[jid])]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("da/du for bfs wave[%d]\n",2);
            printf("da[%d]/dq\n",2);
            printMat<T,6,3>(&s_temp[306],6);
            printf("da[%d]/dqd\n",2);
            printMat<T,6,3>(&s_temp[450],6);
            printf("da[%d]/dq\n",5);
            printMat<T,6,3>(&s_temp[342],6);
            printf("da[%d]/dqd\n",5);
            printMat<T,6,3>(&s_temp[486],6);
            printf("da[%d]/dq\n",8);
            printMat<T,6,3>(&s_temp[378],6);
            printf("da[%d]/dqd\n",8);
            printMat<T,6,3>(&s_temp[522],6);
            printf("da[%d]/dq\n",11);
            printMat<T,6,3>(&s_temp[414],6);
            printf("da[%d]/dqd\n",11);
            printMat<T,6,3>(&s_temp[558],6);
        }
        __syncthreads();
        // Init df/du to 0
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 432; ind += blockDim.x*blockDim.y){
            s_temp[576 + ind] = static_cast<T>(0);
        }
        __syncthreads();
        // Start the df/du by setting = fx(dv/du)*Iv and also compute the temp = Fx(v)*I 
        //    aka do all of the Fx comps in parallel
        // note that while df has more cols than dva the dva cols are the first few df cols
        for(int col = threadIdx.x + threadIdx.y*blockDim.x; col < 120; col += blockDim.x*blockDim.y){
            int col_du = col % 24;
            // non-branching pointer selector
            int jid = (col_du < 1) * 0 + (col_du < 3 && col_du >= 1) * 1 + (col_du < 6 && col_du >= 3) * 2 + (col_du < 7 && col_du >= 6) * 3 + (col_du < 9 && col_du >= 7) * 4 + (col_du < 12 && col_du >= 9) * 5 + (col_du < 13 && col_du >= 12) * 6 + (col_du < 15 && col_du >= 13) * 7 + (col_du < 18 && col_du >= 15) * 8 + (col_du < 19 && col_du >= 18) * 9 + (col_du < 21 && col_du >= 19) * 10 + (col_du >= 21) * 11;
            // Compute Offsets and Pointers
            int dq_flag = col == col_du; int dva_to_df_adjust = (s_topology_helpers[36 + jid] + s_topology_helpers[49 + jid]) - (s_topology_helpers[36 + jid] + jid);
            int Offset_col_du_src = dq_flag * 0 + !dq_flag * 144 + 6*col_du;
            int Offset_col_du_dst = dq_flag * 576 + !dq_flag * 792 + 6*(col_du + dva_to_df_adjust);
            T *dst = &s_temp[Offset_col_du_dst]; const T *fx_src = &s_temp[Offset_col_du_src]; const T *mult_src = &s_temp[1728 + 6*jid];
            // Adjust pointers for temp comps (if applicable)
            if (col >= 48) {
                int comp = col - 48; int comp_col = comp % 6; // int jid = comp / 6;
                int jid6 = comp - comp_col; int jid36_col6 = 6*jid6 + 6*comp_col;
                dst = &s_temp[1008 + jid36_col6]; fx_src = &s_vaf[jid6]; mult_src = &s_XImats[432 + jid36_col6];
            }
            fx_times_v<T>(dst, fx_src, mult_src);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("-------------------------\n");
            printf("df/du part 1 = fx(dv/du)*Iv\n");
            printf("     and Temp = Fx(v)*I\n");
            printf("-------------------------\n");
            printf("df[%d]/dq\n",0);
            printMat<T,6,3>(&s_temp[576],6);
            printf("df[%d]/dqd\n",0);
            printMat<T,6,3>(&s_temp[792],6);
            printf("Fx(v)*I[%d]\n",0);
            printMat<T,6,6>(&s_temp[1008 + 36*0],6);
            printf("df[%d]/dq\n",1);
            printMat<T,6,3>(&s_temp[594],6);
            printf("df[%d]/dqd\n",1);
            printMat<T,6,3>(&s_temp[810],6);
            printf("Fx(v)*I[%d]\n",1);
            printMat<T,6,6>(&s_temp[1008 + 36*1],6);
            printf("df[%d]/dq\n",2);
            printMat<T,6,3>(&s_temp[612],6);
            printf("df[%d]/dqd\n",2);
            printMat<T,6,3>(&s_temp[828],6);
            printf("Fx(v)*I[%d]\n",2);
            printMat<T,6,6>(&s_temp[1008 + 36*2],6);
            printf("df[%d]/dq\n",3);
            printMat<T,6,3>(&s_temp[630],6);
            printf("df[%d]/dqd\n",3);
            printMat<T,6,3>(&s_temp[846],6);
            printf("Fx(v)*I[%d]\n",3);
            printMat<T,6,6>(&s_temp[1008 + 36*3],6);
            printf("df[%d]/dq\n",4);
            printMat<T,6,3>(&s_temp[648],6);
            printf("df[%d]/dqd\n",4);
            printMat<T,6,3>(&s_temp[864],6);
            printf("Fx(v)*I[%d]\n",4);
            printMat<T,6,6>(&s_temp[1008 + 36*4],6);
            printf("df[%d]/dq\n",5);
            printMat<T,6,3>(&s_temp[666],6);
            printf("df[%d]/dqd\n",5);
            printMat<T,6,3>(&s_temp[882],6);
            printf("Fx(v)*I[%d]\n",5);
            printMat<T,6,6>(&s_temp[1008 + 36*5],6);
            printf("df[%d]/dq\n",6);
            printMat<T,6,3>(&s_temp[684],6);
            printf("df[%d]/dqd\n",6);
            printMat<T,6,3>(&s_temp[900],6);
            printf("Fx(v)*I[%d]\n",6);
            printMat<T,6,6>(&s_temp[1008 + 36*6],6);
            printf("df[%d]/dq\n",7);
            printMat<T,6,3>(&s_temp[702],6);
            printf("df[%d]/dqd\n",7);
            printMat<T,6,3>(&s_temp[918],6);
            printf("Fx(v)*I[%d]\n",7);
            printMat<T,6,6>(&s_temp[1008 + 36*7],6);
            printf("df[%d]/dq\n",8);
            printMat<T,6,3>(&s_temp[720],6);
            printf("df[%d]/dqd\n",8);
            printMat<T,6,3>(&s_temp[936],6);
            printf("Fx(v)*I[%d]\n",8);
            printMat<T,6,6>(&s_temp[1008 + 36*8],6);
            printf("df[%d]/dq\n",9);
            printMat<T,6,3>(&s_temp[738],6);
            printf("df[%d]/dqd\n",9);
            printMat<T,6,3>(&s_temp[954],6);
            printf("Fx(v)*I[%d]\n",9);
            printMat<T,6,6>(&s_temp[1008 + 36*9],6);
            printf("df[%d]/dq\n",10);
            printMat<T,6,3>(&s_temp[756],6);
            printf("df[%d]/dqd\n",10);
            printMat<T,6,3>(&s_temp[972],6);
            printf("Fx(v)*I[%d]\n",10);
            printMat<T,6,6>(&s_temp[1008 + 36*10],6);
            printf("df[%d]/dq\n",11);
            printMat<T,6,3>(&s_temp[774],6);
            printf("df[%d]/dqd\n",11);
            printMat<T,6,3>(&s_temp[990],6);
            printf("Fx(v)*I[%d]\n",11);
            printMat<T,6,6>(&s_temp[1008 + 36*11],6);
        }
        __syncthreads();
        // Then in parallel finish df/du += I*da/du + (Fx(v)I)*dv/du
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = ind / 6; int col6 = ind - row; int col_du = (col % 24);
            // non-branching pointer selector
            int jid = (col_du < 1) * 0 + (col_du < 3 && col_du >= 1) * 1 + (col_du < 6 && col_du >= 3) * 2 + (col_du < 7 && col_du >= 6) * 3 + (col_du < 9 && col_du >= 7) * 4 + (col_du < 12 && col_du >= 9) * 5 + (col_du < 13 && col_du >= 12) * 6 + (col_du < 15 && col_du >= 13) * 7 + (col_du < 18 && col_du >= 15) * 8 + (col_du < 19 && col_du >= 18) * 9 + (col_du < 21 && col_du >= 19) * 10 + (col_du >= 21) * 11;
            // Compute Offsets and Pointers
            int dva_to_df_adjust = (s_topology_helpers[36 + jid] + s_topology_helpers[49 + jid]) - (s_topology_helpers[36 + jid] + jid);
            if (col >= 24){dva_to_df_adjust += 12;}
            T *df_row_col = &s_temp[576 + 6*dva_to_df_adjust + ind];
            const T *dv_col = &s_temp[0 + col6]; const T *da_col = &s_temp[288 + col6];
            int jid36 = 36*jid; const T *I_row = &s_XImats[432 + jid36 + row]; const T *FxvI_row = &s_temp[1008 + jid36 + row];
            // Compute the values
            *df_row_col += dot_prod<T,6,6,1>(I_row,da_col) + dot_prod<T,6,6,1>(FxvI_row,dv_col);
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("-------------------------\n");
            printf("df/du += I*da/du + FxvI*dv/du\n");
            printf("-------------------------\n");
            printf("df[%d]/dq\n",0);
            printMat<T,6,3>(&s_temp[576],6);
            printf("df[%d]/dqd\n",0);
            printMat<T,6,3>(&s_temp[792],6);
            printf("df[%d]/dq\n",1);
            printMat<T,6,3>(&s_temp[594],6);
            printf("df[%d]/dqd\n",1);
            printMat<T,6,3>(&s_temp[810],6);
            printf("df[%d]/dq\n",2);
            printMat<T,6,3>(&s_temp[612],6);
            printf("df[%d]/dqd\n",2);
            printMat<T,6,3>(&s_temp[828],6);
            printf("df[%d]/dq\n",3);
            printMat<T,6,3>(&s_temp[630],6);
            printf("df[%d]/dqd\n",3);
            printMat<T,6,3>(&s_temp[846],6);
            printf("df[%d]/dq\n",4);
            printMat<T,6,3>(&s_temp[648],6);
            printf("df[%d]/dqd\n",4);
            printMat<T,6,3>(&s_temp[864],6);
            printf("df[%d]/dq\n",5);
            printMat<T,6,3>(&s_temp[666],6);
            printf("df[%d]/dqd\n",5);
            printMat<T,6,3>(&s_temp[882],6);
            printf("df[%d]/dq\n",6);
            printMat<T,6,3>(&s_temp[684],6);
            printf("df[%d]/dqd\n",6);
            printMat<T,6,3>(&s_temp[900],6);
            printf("df[%d]/dq\n",7);
            printMat<T,6,3>(&s_temp[702],6);
            printf("df[%d]/dqd\n",7);
            printMat<T,6,3>(&s_temp[918],6);
            printf("df[%d]/dq\n",8);
            printMat<T,6,3>(&s_temp[720],6);
            printf("df[%d]/dqd\n",8);
            printMat<T,6,3>(&s_temp[936],6);
            printf("df[%d]/dq\n",9);
            printMat<T,6,3>(&s_temp[738],6);
            printf("df[%d]/dqd\n",9);
            printMat<T,6,3>(&s_temp[954],6);
            printf("df[%d]/dq\n",10);
            printMat<T,6,3>(&s_temp[756],6);
            printf("df[%d]/dqd\n",10);
            printMat<T,6,3>(&s_temp[972],6);
            printf("df[%d]/dq\n",11);
            printMat<T,6,3>(&s_temp[774],6);
            printf("df[%d]/dqd\n",11);
            printMat<T,6,3>(&s_temp[990],6);
        }
        __syncthreads();
        // At the same time compute the last temp var: -X^T * mx(f)
        // use Mx(Xv) temp memory as those values are no longer needed
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 72; ind += blockDim.x*blockDim.y){
            int XTcol = ind % 6; int jid6 = ind - XTcol;
            s_temp[1440 + ind] = -dot_prod<T,6,1,1>(&s_XImats[6*(jid6 + XTcol)], &s_temp[1656 + jid6]);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("-------------------------\n");
            printf("Temp = -X^T * mx(f)\n");
            printf("-------------------------\n");
            printf("-X^T*mx(f)[%d]\n",0);
            printMat<T,1,6>(&s_temp[1440 + 6*0],1);
            printf("-X^T*mx(f)[%d]\n",1);
            printMat<T,1,6>(&s_temp[1440 + 6*1],1);
            printf("-X^T*mx(f)[%d]\n",2);
            printMat<T,1,6>(&s_temp[1440 + 6*2],1);
            printf("-X^T*mx(f)[%d]\n",3);
            printMat<T,1,6>(&s_temp[1440 + 6*3],1);
            printf("-X^T*mx(f)[%d]\n",4);
            printMat<T,1,6>(&s_temp[1440 + 6*4],1);
            printf("-X^T*mx(f)[%d]\n",5);
            printMat<T,1,6>(&s_temp[1440 + 6*5],1);
            printf("-X^T*mx(f)[%d]\n",6);
            printMat<T,1,6>(&s_temp[1440 + 6*6],1);
            printf("-X^T*mx(f)[%d]\n",7);
            printMat<T,1,6>(&s_temp[1440 + 6*7],1);
            printf("-X^T*mx(f)[%d]\n",8);
            printMat<T,1,6>(&s_temp[1440 + 6*8],1);
            printf("-X^T*mx(f)[%d]\n",9);
            printMat<T,1,6>(&s_temp[1440 + 6*9],1);
            printf("-X^T*mx(f)[%d]\n",10);
            printMat<T,1,6>(&s_temp[1440 + 6*10],1);
            printf("-X^T*mx(f)[%d]\n",11);
            printMat<T,1,6>(&s_temp[1440 + 6*11],1);
        }
        __syncthreads();
        //
        // BACKWARD Pass
        //
        // df/du update where bfs_level is 2
        //     joints are: lf_kfe_joint, lh_kfe_joint, rf_kfe_joint, rh_kfe_joint
        //     links are: lf_lowerleg, lh_lowerleg, rf_lowerleg, rh_lowerleg
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("df[%d]/dq (parent update) BEFORE UPDATE\n",1);
            printMat<T,6,3>(&s_temp[594],6);
            printf("df[%d]/dqd (parent update) BEFORE UPDATE\n",1);
            printMat<T,6,3>(&s_temp[810],6);
            printf("df[%d]/dq (parent update) BEFORE UPDATE\n",10);
            printMat<T,6,3>(&s_temp[756],6);
            printf("df[%d]/dqd (parent update) BEFORE UPDATE\n",10);
            printMat<T,6,3>(&s_temp[972],6);
            printf("df[%d]/dq (parent update) BEFORE UPDATE\n",4);
            printMat<T,6,3>(&s_temp[648],6);
            printf("df[%d]/dqd (parent update) BEFORE UPDATE\n",4);
            printMat<T,6,3>(&s_temp[864],6);
            printf("df[%d]/dq (parent update) BEFORE UPDATE\n",7);
            printMat<T,6,3>(&s_temp[702],6);
            printf("df[%d]/dqd (parent update) BEFORE UPDATE\n",7);
            printMat<T,6,3>(&s_temp[918],6);
        }
        __syncthreads();
        // df_lambda/du += X^T * df/du + {Xmx(f), 0}
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = ind / 6; int col_du = col % 12;
            // non-branching pointer selector
            int jid = (col_du < 3) * 2 + (col_du < 6 && col_du >= 3) * 5 + (col_du < 9 && col_du >= 6) * 8 + (col_du >= 9) * 11;
            int col_adjust = (col_du < 3) * 0 + (col_du < 6 && col_du >= 3) * 3 + (col_du < 9 && col_du >= 6) * 6 + (col_du >= 9) * 9;
            int dq_flag = col == col_du;
            col_du -= col_adjust; // adjust for variable number of columns
            int du_col_offset = dq_flag * 576 + !dq_flag * 792 + 6*col_du;
            int dst_adjust = (col_du >= s_topology_helpers[12 + jid]) * 6 * 0; // adjust for sparsity compression offsets
            T *dst = &s_temp[du_col_offset + 6*(s_topology_helpers[36 + s_topology_helpers[jid]] + s_topology_helpers[49 + s_topology_helpers[jid]]) + dst_adjust + row];
            T update_val = dot_prod<T,6,1,1>(&s_XImats[36*jid + 6*row],&s_temp[du_col_offset + 6*(s_topology_helpers[36 + jid] + s_topology_helpers[49 + jid])])
                          + dq_flag * (col_du == s_topology_helpers[12 + jid]) * s_temp[1440 + 6*jid + row];
            *dst += update_val;
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("df/du for bfs wave[%d]\n",2);
            printf("df[%d]/dq (parent update)\n",1);
            printMat<T,6,3>(&s_temp[594],6);
            printf("df[%d]/dqd (parent update)\n",1);
            printMat<T,6,3>(&s_temp[810],6);
            printf("df[%d]/dq (parent update)\n",10);
            printMat<T,6,3>(&s_temp[756],6);
            printf("df[%d]/dqd (parent update)\n",10);
            printMat<T,6,3>(&s_temp[972],6);
            printf("df[%d]/dq (parent update)\n",4);
            printMat<T,6,3>(&s_temp[648],6);
            printf("df[%d]/dqd (parent update)\n",4);
            printMat<T,6,3>(&s_temp[864],6);
            printf("df[%d]/dq (parent update)\n",7);
            printMat<T,6,3>(&s_temp[702],6);
            printf("df[%d]/dqd (parent update)\n",7);
            printMat<T,6,3>(&s_temp[918],6);
        }
        __syncthreads();
        // df/du update where bfs_level is 1
        //     joints are: lf_hfe_joint, lh_hfe_joint, rf_hfe_joint, rh_hfe_joint
        //     links are: lf_upperleg, lh_upperleg, rf_upperleg, rh_upperleg
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("df[%d]/dq (parent update) BEFORE UPDATE\n",0);
            printMat<T,6,3>(&s_temp[576],6);
            printf("df[%d]/dqd (parent update) BEFORE UPDATE\n",0);
            printMat<T,6,3>(&s_temp[792],6);
            printf("df[%d]/dq (parent update) BEFORE UPDATE\n",9);
            printMat<T,6,3>(&s_temp[738],6);
            printf("df[%d]/dqd (parent update) BEFORE UPDATE\n",9);
            printMat<T,6,3>(&s_temp[954],6);
            printf("df[%d]/dq (parent update) BEFORE UPDATE\n",3);
            printMat<T,6,3>(&s_temp[630],6);
            printf("df[%d]/dqd (parent update) BEFORE UPDATE\n",3);
            printMat<T,6,3>(&s_temp[846],6);
            printf("df[%d]/dq (parent update) BEFORE UPDATE\n",6);
            printMat<T,6,3>(&s_temp[684],6);
            printf("df[%d]/dqd (parent update) BEFORE UPDATE\n",6);
            printMat<T,6,3>(&s_temp[900],6);
        }
        __syncthreads();
        // df_lambda/du += X^T * df/du + {Xmx(f), 0}
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = ind / 6; int col_du = col % 12;
            // non-branching pointer selector
            int jid = (col_du < 3) * 1 + (col_du < 6 && col_du >= 3) * 4 + (col_du < 9 && col_du >= 6) * 7 + (col_du >= 9) * 10;
            int col_adjust = (col_du < 3) * 0 + (col_du < 6 && col_du >= 3) * 3 + (col_du < 9 && col_du >= 6) * 6 + (col_du >= 9) * 9;
            int dq_flag = col == col_du;
            col_du -= col_adjust; // adjust for variable number of columns
            int du_col_offset = dq_flag * 576 + !dq_flag * 792 + 6*col_du;
            int dst_adjust = (col_du >= s_topology_helpers[12 + jid]) * 6 * 0; // adjust for sparsity compression offsets
            T *dst = &s_temp[du_col_offset + 6*(s_topology_helpers[36 + s_topology_helpers[jid]] + s_topology_helpers[49 + s_topology_helpers[jid]]) + dst_adjust + row];
            T update_val = dot_prod<T,6,1,1>(&s_XImats[36*jid + 6*row],&s_temp[du_col_offset + 6*(s_topology_helpers[36 + jid] + s_topology_helpers[49 + jid])])
                          + dq_flag * (col_du == s_topology_helpers[12 + jid]) * s_temp[1440 + 6*jid + row];
            *dst += update_val;
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("df/du for bfs wave[%d]\n",1);
            printf("df[%d]/dq (parent update)\n",0);
            printMat<T,6,3>(&s_temp[576],6);
            printf("df[%d]/dqd (parent update)\n",0);
            printMat<T,6,3>(&s_temp[792],6);
            printf("df[%d]/dq (parent update)\n",9);
            printMat<T,6,3>(&s_temp[738],6);
            printf("df[%d]/dqd (parent update)\n",9);
            printMat<T,6,3>(&s_temp[954],6);
            printf("df[%d]/dq (parent update)\n",3);
            printMat<T,6,3>(&s_temp[630],6);
            printf("df[%d]/dqd (parent update)\n",3);
            printMat<T,6,3>(&s_temp[846],6);
            printf("df[%d]/dq (parent update)\n",6);
            printMat<T,6,3>(&s_temp[684],6);
            printf("df[%d]/dqd (parent update)\n",6);
            printMat<T,6,3>(&s_temp[900],6);
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("-------------------------\n");
            printf("Final dvaf/du\n");
            printf("-------------------------\n");
            printf("dv[%d]/dq\n",0);
            printMat<T,6,1>(&s_temp[0],6);
            printf("dv[%d]/dq\n",1);
            printMat<T,6,2>(&s_temp[6],6);
            printf("dv[%d]/dq\n",2);
            printMat<T,6,3>(&s_temp[18],6);
            printf("dv[%d]/dq\n",3);
            printMat<T,6,1>(&s_temp[36],6);
            printf("dv[%d]/dq\n",4);
            printMat<T,6,2>(&s_temp[42],6);
            printf("dv[%d]/dq\n",5);
            printMat<T,6,3>(&s_temp[54],6);
            printf("dv[%d]/dq\n",6);
            printMat<T,6,1>(&s_temp[72],6);
            printf("dv[%d]/dq\n",7);
            printMat<T,6,2>(&s_temp[78],6);
            printf("dv[%d]/dq\n",8);
            printMat<T,6,3>(&s_temp[90],6);
            printf("dv[%d]/dq\n",9);
            printMat<T,6,1>(&s_temp[108],6);
            printf("dv[%d]/dq\n",10);
            printMat<T,6,2>(&s_temp[114],6);
            printf("dv[%d]/dq\n",11);
            printMat<T,6,3>(&s_temp[126],6);
            printf("dv[%d]/dqd\n",0);
            printMat<T,6,1>(&s_temp[144],6);
            printf("dv[%d]/dqd\n",1);
            printMat<T,6,2>(&s_temp[150],6);
            printf("dv[%d]/dqd\n",2);
            printMat<T,6,3>(&s_temp[162],6);
            printf("dv[%d]/dqd\n",3);
            printMat<T,6,1>(&s_temp[180],6);
            printf("dv[%d]/dqd\n",4);
            printMat<T,6,2>(&s_temp[186],6);
            printf("dv[%d]/dqd\n",5);
            printMat<T,6,3>(&s_temp[198],6);
            printf("dv[%d]/dqd\n",6);
            printMat<T,6,1>(&s_temp[216],6);
            printf("dv[%d]/dqd\n",7);
            printMat<T,6,2>(&s_temp[222],6);
            printf("dv[%d]/dqd\n",8);
            printMat<T,6,3>(&s_temp[234],6);
            printf("dv[%d]/dqd\n",9);
            printMat<T,6,1>(&s_temp[252],6);
            printf("dv[%d]/dqd\n",10);
            printMat<T,6,2>(&s_temp[258],6);
            printf("dv[%d]/dqd\n",11);
            printMat<T,6,3>(&s_temp[270],6);
            printf("da[%d]/dq\n",0);
            printMat<T,6,1>(&s_temp[288],6);
            printf("da[%d]/dq\n",1);
            printMat<T,6,2>(&s_temp[294],6);
            printf("da[%d]/dq\n",2);
            printMat<T,6,3>(&s_temp[306],6);
            printf("da[%d]/dq\n",3);
            printMat<T,6,1>(&s_temp[324],6);
            printf("da[%d]/dq\n",4);
            printMat<T,6,2>(&s_temp[330],6);
            printf("da[%d]/dq\n",5);
            printMat<T,6,3>(&s_temp[342],6);
            printf("da[%d]/dq\n",6);
            printMat<T,6,1>(&s_temp[360],6);
            printf("da[%d]/dq\n",7);
            printMat<T,6,2>(&s_temp[366],6);
            printf("da[%d]/dq\n",8);
            printMat<T,6,3>(&s_temp[378],6);
            printf("da[%d]/dq\n",9);
            printMat<T,6,1>(&s_temp[396],6);
            printf("da[%d]/dq\n",10);
            printMat<T,6,2>(&s_temp[402],6);
            printf("da[%d]/dq\n",11);
            printMat<T,6,3>(&s_temp[414],6);
            printf("da[%d]/dqd\n",0);
            printMat<T,6,1>(&s_temp[432],6);
            printf("da[%d]/dqd\n",1);
            printMat<T,6,2>(&s_temp[438],6);
            printf("da[%d]/dqd\n",2);
            printMat<T,6,3>(&s_temp[450],6);
            printf("da[%d]/dqd\n",3);
            printMat<T,6,1>(&s_temp[468],6);
            printf("da[%d]/dqd\n",4);
            printMat<T,6,2>(&s_temp[474],6);
            printf("da[%d]/dqd\n",5);
            printMat<T,6,3>(&s_temp[486],6);
            printf("da[%d]/dqd\n",6);
            printMat<T,6,1>(&s_temp[504],6);
            printf("da[%d]/dqd\n",7);
            printMat<T,6,2>(&s_temp[510],6);
            printf("da[%d]/dqd\n",8);
            printMat<T,6,3>(&s_temp[522],6);
            printf("da[%d]/dqd\n",9);
            printMat<T,6,1>(&s_temp[540],6);
            printf("da[%d]/dqd\n",10);
            printMat<T,6,2>(&s_temp[546],6);
            printf("da[%d]/dqd\n",11);
            printMat<T,6,3>(&s_temp[558],6);
            printf("df[%d]/dq\n",0);
            printMat<T,6,1>(&s_temp[576],6);
            printf("df[%d]/dq\n",1);
            printMat<T,6,2>(&s_temp[594],6);
            printf("df[%d]/dq\n",2);
            printMat<T,6,3>(&s_temp[612],6);
            printf("df[%d]/dq\n",3);
            printMat<T,6,1>(&s_temp[630],6);
            printf("df[%d]/dq\n",4);
            printMat<T,6,2>(&s_temp[648],6);
            printf("df[%d]/dq\n",5);
            printMat<T,6,3>(&s_temp[666],6);
            printf("df[%d]/dq\n",6);
            printMat<T,6,1>(&s_temp[684],6);
            printf("df[%d]/dq\n",7);
            printMat<T,6,2>(&s_temp[702],6);
            printf("df[%d]/dq\n",8);
            printMat<T,6,3>(&s_temp[720],6);
            printf("df[%d]/dq\n",9);
            printMat<T,6,1>(&s_temp[738],6);
            printf("df[%d]/dq\n",10);
            printMat<T,6,2>(&s_temp[756],6);
            printf("df[%d]/dq\n",11);
            printMat<T,6,3>(&s_temp[774],6);
            printf("df[%d]/dqd\n",0);
            printMat<T,6,1>(&s_temp[792],6);
            printf("df[%d]/dqd\n",1);
            printMat<T,6,2>(&s_temp[810],6);
            printf("df[%d]/dqd\n",2);
            printMat<T,6,3>(&s_temp[828],6);
            printf("df[%d]/dqd\n",3);
            printMat<T,6,1>(&s_temp[846],6);
            printf("df[%d]/dqd\n",4);
            printMat<T,6,2>(&s_temp[864],6);
            printf("df[%d]/dqd\n",5);
            printMat<T,6,3>(&s_temp[882],6);
            printf("df[%d]/dqd\n",6);
            printMat<T,6,1>(&s_temp[900],6);
            printf("df[%d]/dqd\n",7);
            printMat<T,6,2>(&s_temp[918],6);
            printf("df[%d]/dqd\n",8);
            printMat<T,6,3>(&s_temp[936],6);
            printf("df[%d]/dqd\n",9);
            printMat<T,6,1>(&s_temp[954],6);
            printf("df[%d]/dqd\n",10);
            printMat<T,6,2>(&s_temp[972],6);
            printf("df[%d]/dqd\n",11);
            printMat<T,6,3>(&s_temp[990],6);
        }
        // Finally dc[i]/du = S[i]^T*df[i]/du
        for(int jid_dq_qd = threadIdx.x + threadIdx.y*blockDim.x; jid_dq_qd < 24; jid_dq_qd += blockDim.x*blockDim.y){
            int jid = jid_dq_qd % 12; int dq_flag = jid == jid_dq_qd;
            // Note that this gets a tad complicated due to memory compression and variable column length
            //    so we need to fully unroll the loop -- this will not be the most efficient for a serial
            //    chain manipulator but will generalize to branched robots
            int Offset_src = dq_flag * 576 + !dq_flag * 792 + 6*(s_topology_helpers[36 + jid] + s_topology_helpers[49 + jid]) + 2;
            int Offset_dst = !dq_flag * 144 + jid; bool flag = 0;
            // dc[jid]/du[0]
            flag = ((jid == 0) || (jid == 1) || (jid == 2));
            s_dc_du[Offset_dst] = flag*s_temp[Offset_src]; Offset_src += flag*6; Offset_dst += 12;
            // dc[jid]/du[1]
            flag = ((jid == 0) || (jid == 1) || (jid == 2));
            s_dc_du[Offset_dst] = flag*s_temp[Offset_src]; Offset_src += flag*6; Offset_dst += 12;
            // dc[jid]/du[2]
            flag = ((jid == 0) || (jid == 1) || (jid == 2));
            s_dc_du[Offset_dst] = flag*s_temp[Offset_src]; Offset_src += flag*6; Offset_dst += 12;
            // dc[jid]/du[3]
            flag = ((jid == 3) || (jid == 4) || (jid == 5));
            s_dc_du[Offset_dst] = flag*s_temp[Offset_src]; Offset_src += flag*6; Offset_dst += 12;
            // dc[jid]/du[4]
            flag = ((jid == 3) || (jid == 4) || (jid == 5));
            s_dc_du[Offset_dst] = flag*s_temp[Offset_src]; Offset_src += flag*6; Offset_dst += 12;
            // dc[jid]/du[5]
            flag = ((jid == 3) || (jid == 4) || (jid == 5));
            s_dc_du[Offset_dst] = flag*s_temp[Offset_src]; Offset_src += flag*6; Offset_dst += 12;
            // dc[jid]/du[6]
            flag = ((jid == 6) || (jid == 7) || (jid == 8));
            s_dc_du[Offset_dst] = flag*s_temp[Offset_src]; Offset_src += flag*6; Offset_dst += 12;
            // dc[jid]/du[7]
            flag = ((jid == 6) || (jid == 7) || (jid == 8));
            s_dc_du[Offset_dst] = flag*s_temp[Offset_src]; Offset_src += flag*6; Offset_dst += 12;
            // dc[jid]/du[8]
            flag = ((jid == 6) || (jid == 7) || (jid == 8));
            s_dc_du[Offset_dst] = flag*s_temp[Offset_src]; Offset_src += flag*6; Offset_dst += 12;
            // dc[jid]/du[9]
            flag = ((jid == 9) || (jid == 10) || (jid == 11));
            s_dc_du[Offset_dst] = flag*s_temp[Offset_src]; Offset_src += flag*6; Offset_dst += 12;
            // dc[jid]/du[10]
            flag = ((jid == 9) || (jid == 10) || (jid == 11));
            s_dc_du[Offset_dst] = flag*s_temp[Offset_src]; Offset_src += flag*6; Offset_dst += 12;
            // dc[jid]/du[11]
            flag = ((jid == 9) || (jid == 10) || (jid == 11));
            s_dc_du[Offset_dst] = flag*s_temp[Offset_src]; Offset_src += flag*6; Offset_dst += 12;
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("-------------------------\n");
            printf("Final dc/du\n");
            printf("-------------------------\n");
            printf("dc/dq\n");
            printMat<T,12,12>(&s_dc_du[0],12);
            printf("dc/dqd\n");
            printMat<T,12,12>(&s_dc_du[144],12);
        }
        __syncthreads();
    }

    /**
     * Computes the gradient of inverse dynamics
     *
     * @param s_dc_du is a pointer to memory for the final result of size 2*NUM_JOINTS*NUM_JOINTS = 288
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param s_qdd is the vector of joint accelerations
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void inverse_dynamics_gradient_device(T *s_dc_du, const T *s_q, const T *s_qd, const T *s_qdd, const robotModel<T> *d_robotModel, const T gravity) {
        __shared__ T s_vaf[216];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
        inverse_dynamics_inner_vaf<T>(s_vaf, s_q, s_qd, s_qdd, s_XImats, s_topology_helpers, s_temp, gravity);
        inverse_dynamics_gradient_inner<T>(s_dc_du, s_q, s_qd, s_vaf, s_XImats, s_topology_helpers, s_temp, gravity);
    }

    /**
     * Computes the gradient of inverse dynamics
     *
     * Notes:
     *   optimized for qdd = 0
     *
     * @param s_dc_du is a pointer to memory for the final result of size 2*NUM_JOINTS*NUM_JOINTS = 288
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void inverse_dynamics_gradient_device(T *s_dc_du, const T *s_q, const T *s_qd, const robotModel<T> *d_robotModel, const T gravity) {
        __shared__ T s_vaf[216];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
        inverse_dynamics_inner_vaf<T>(s_vaf, s_q, s_qd, s_XImats, s_topology_helpers, s_temp, gravity);
        inverse_dynamics_gradient_inner<T>(s_dc_du, s_q, s_qd, s_vaf, s_XImats, s_topology_helpers, s_temp, gravity);
    }

    /**
     * Computes the gradient of inverse dynamics
     *
     * @param d_dc_du is a pointer to memory for the final result of size 2*NUM_JOINTS*NUM_JOINTS = 288
     * @param d_q_dq is the vector of joint positions and velocities
     * @param stride_q_qd is the stide between each q, qd
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param d_qdd is the vector of joint accelerations
     * @param gravity is the gravity constant
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void inverse_dynamics_gradient_kernel_single_timing(T *d_dc_du, const T *d_q_qd, const int stride_q_qd, const T *d_qdd, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_q_qd[24]; T *s_q = s_q_qd; T *s_qd = &s_q_qd[12];
        __shared__ T s_qdd[12]; 
        __shared__ T s_dc_du[288];
        __shared__ T s_vaf[216];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        // load to shared mem
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            s_q_qd[ind] = d_q_qd[ind];
        }
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
            s_qdd[ind] = d_qdd[ind];
        }
        __syncthreads();
        // compute with NUM_TIMESTEPS as NUM_REPS for timing
        for (int rep = 0; rep < NUM_TIMESTEPS; rep++){
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            inverse_dynamics_inner_vaf<T>(s_vaf, s_q, s_qd, s_qdd, s_XImats, s_topology_helpers, s_temp, gravity);
            inverse_dynamics_gradient_inner<T>(s_dc_du, s_q, s_qd, s_vaf, s_XImats, s_topology_helpers, s_temp, gravity);
        }
        // save down to global
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
            d_dc_du[ind] = s_dc_du[ind];
        }
        __syncthreads();
    }

    /**
     * Computes the gradient of inverse dynamics
     *
     * @param d_dc_du is a pointer to memory for the final result of size 2*NUM_JOINTS*NUM_JOINTS = 288
     * @param d_q_dq is the vector of joint positions and velocities
     * @param stride_q_qd is the stide between each q, qd
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param d_qdd is the vector of joint accelerations
     * @param gravity is the gravity constant
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void inverse_dynamics_gradient_kernel(T *d_dc_du, const T *d_q_qd, const int stride_q_qd, const T *d_qdd, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_q_qd[24]; T *s_q = s_q_qd; T *s_qd = &s_q_qd[12];
        __shared__ T s_qdd[12]; 
        __shared__ T s_dc_du[288];
        __shared__ T s_vaf[216];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        for(int k = blockIdx.x + blockIdx.y*gridDim.x; k < NUM_TIMESTEPS; k += gridDim.x*gridDim.y){
            // load to shared mem
            const T *d_q_qd_k = &d_q_qd[k*stride_q_qd];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
                s_q_qd[ind] = d_q_qd_k[ind];
            }
            const T *d_qdd_k = &d_qdd[k*12];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
                s_qdd[ind] = d_qdd_k[ind];
            }
            __syncthreads();
            // compute
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            inverse_dynamics_inner_vaf<T>(s_vaf, s_q, s_qd, s_qdd, s_XImats, s_topology_helpers, s_temp, gravity);
            inverse_dynamics_gradient_inner<T>(s_dc_du, s_q, s_qd, s_vaf, s_XImats, s_topology_helpers, s_temp, gravity);
            __syncthreads();
            // save down to global
            T *d_dc_du_k = &d_dc_du[k*288];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
                d_dc_du_k[ind] = s_dc_du[ind];
            }
            __syncthreads();
        }
    }

    /**
     * Computes the gradient of inverse dynamics
     *
     * Notes:
     *   optimized for qdd = 0
     *
     * @param d_dc_du is a pointer to memory for the final result of size 2*NUM_JOINTS*NUM_JOINTS = 288
     * @param d_q_dq is the vector of joint positions and velocities
     * @param stride_q_qd is the stide between each q, qd
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void inverse_dynamics_gradient_kernel_single_timing(T *d_dc_du, const T *d_q_qd, const int stride_q_qd, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_q_qd[24]; T *s_q = s_q_qd; T *s_qd = &s_q_qd[12];
        __shared__ T s_dc_du[288];
        __shared__ T s_vaf[216];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        // load to shared mem
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            s_q_qd[ind] = d_q_qd[ind];
        }
        __syncthreads();
        // compute with NUM_TIMESTEPS as NUM_REPS for timing
        for (int rep = 0; rep < NUM_TIMESTEPS; rep++){
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            inverse_dynamics_inner_vaf<T>(s_vaf, s_q, s_qd, s_XImats, s_topology_helpers, s_temp, gravity);
            inverse_dynamics_gradient_inner<T>(s_dc_du, s_q, s_qd, s_vaf, s_XImats, s_topology_helpers, s_temp, gravity);
        }
        // save down to global
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
            d_dc_du[ind] = s_dc_du[ind];
        }
        __syncthreads();
    }

    /**
     * Computes the gradient of inverse dynamics
     *
     * Notes:
     *   optimized for qdd = 0
     *
     * @param d_dc_du is a pointer to memory for the final result of size 2*NUM_JOINTS*NUM_JOINTS = 288
     * @param d_q_dq is the vector of joint positions and velocities
     * @param stride_q_qd is the stide between each q, qd
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void inverse_dynamics_gradient_kernel(T *d_dc_du, const T *d_q_qd, const int stride_q_qd, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_q_qd[24]; T *s_q = s_q_qd; T *s_qd = &s_q_qd[12];
        __shared__ T s_dc_du[288];
        __shared__ T s_vaf[216];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        for(int k = blockIdx.x + blockIdx.y*gridDim.x; k < NUM_TIMESTEPS; k += gridDim.x*gridDim.y){
            // load to shared mem
            const T *d_q_qd_k = &d_q_qd[k*stride_q_qd];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
                s_q_qd[ind] = d_q_qd_k[ind];
            }
            __syncthreads();
            // compute
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            inverse_dynamics_inner_vaf<T>(s_vaf, s_q, s_qd, s_XImats, s_topology_helpers, s_temp, gravity);
            inverse_dynamics_gradient_inner<T>(s_dc_du, s_q, s_qd, s_vaf, s_XImats, s_topology_helpers, s_temp, gravity);
            __syncthreads();
            // save down to global
            T *d_dc_du_k = &d_dc_du[k*288];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
                d_dc_du_k[ind] = s_dc_du[ind];
            }
            __syncthreads();
        }
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_QDD_FLAG = false, bool USE_COMPRESSED_MEM = false>
    __host__
    void inverse_dynamics_gradient(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                                   const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        // start code with memory transfer
        int stride_q_qd;
        if (USE_COMPRESSED_MEM) {stride_q_qd = 2*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd,hd_data->h_q_qd,stride_q_qd*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        else {stride_q_qd = 3*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q_qd*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        if (USE_QDD_FLAG) {gpuErrchk(cudaMemcpyAsync(hd_data->d_qdd,hd_data->h_qdd,NUM_JOINTS*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[1]));}
        gpuErrchk(cudaDeviceSynchronize());
        // then call the kernel
        if (USE_QDD_FLAG) {
            if (USE_COMPRESSED_MEM) {inverse_dynamics_gradient_kernel<T><<<block_dimms,thread_dimms,ID_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_dc_du,hd_data->d_q_qd,stride_q_qd,hd_data->d_qdd, d_robotModel,gravity,num_timesteps);}
            else                    {inverse_dynamics_gradient_kernel<T><<<block_dimms,thread_dimms,ID_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_dc_du,hd_data->d_q_qd_u,stride_q_qd,hd_data->d_qdd, d_robotModel,gravity,num_timesteps);}
        }
        else {
            if (USE_COMPRESSED_MEM) {inverse_dynamics_gradient_kernel<T><<<block_dimms,thread_dimms,ID_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_dc_du,hd_data->d_q_qd,stride_q_qd,d_robotModel,gravity,num_timesteps);}
            else                    {inverse_dynamics_gradient_kernel<T><<<block_dimms,thread_dimms,ID_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_dc_du,hd_data->d_q_qd_u,stride_q_qd,d_robotModel,gravity,num_timesteps);}
        }
        gpuErrchk(cudaDeviceSynchronize());
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_dc_du,hd_data->d_dc_du,NUM_JOINTS*2*NUM_JOINTS*num_timesteps*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_QDD_FLAG = false, bool USE_COMPRESSED_MEM = false>
    __host__
    void inverse_dynamics_gradient_single_timing(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                                                 const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        // start code with memory transfer
        int stride_q_qd;
        if (USE_COMPRESSED_MEM) {stride_q_qd = 2*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd,hd_data->h_q_qd,stride_q_qd*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        else {stride_q_qd = 3*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q_qd*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        if (USE_QDD_FLAG) {gpuErrchk(cudaMemcpyAsync(hd_data->d_qdd,hd_data->h_qdd,NUM_JOINTS*sizeof(T),cudaMemcpyHostToDevice,streams[1]));}
        gpuErrchk(cudaDeviceSynchronize());
        // then call the kernel
        struct timespec start, end; clock_gettime(CLOCK_MONOTONIC,&start);
        if (USE_QDD_FLAG) {
            if (USE_COMPRESSED_MEM) {inverse_dynamics_gradient_kernel_single_timing<T><<<block_dimms,thread_dimms,ID_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_dc_du,hd_data->d_q_qd,stride_q_qd,hd_data->d_qdd, d_robotModel,gravity,num_timesteps);}
            else                    {inverse_dynamics_gradient_kernel_single_timing<T><<<block_dimms,thread_dimms,ID_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_dc_du,hd_data->d_q_qd_u,stride_q_qd,hd_data->d_qdd, d_robotModel,gravity,num_timesteps);}
        }
        else {
            if (USE_COMPRESSED_MEM) {inverse_dynamics_gradient_kernel_single_timing<T><<<block_dimms,thread_dimms,ID_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_dc_du,hd_data->d_q_qd,stride_q_qd,d_robotModel,gravity,num_timesteps);}
            else                    {inverse_dynamics_gradient_kernel_single_timing<T><<<block_dimms,thread_dimms,ID_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_dc_du,hd_data->d_q_qd_u,stride_q_qd,d_robotModel,gravity,num_timesteps);}
        }
        gpuErrchk(cudaDeviceSynchronize());
        clock_gettime(CLOCK_MONOTONIC,&end);
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_dc_du,hd_data->d_dc_du,NUM_JOINTS*2*NUM_JOINTS*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
        printf("Single Call ID_DU %fus\n",time_delta_us_timespec(start,end)/static_cast<double>(num_timesteps));
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_QDD_FLAG = false, bool USE_COMPRESSED_MEM = false>
    __host__
    void inverse_dynamics_gradient_compute_only(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                                                const dim3 block_dimms, const dim3 thread_dimms) {
        int stride_q_qd = USE_COMPRESSED_MEM ? 2*NUM_JOINTS: 3*NUM_JOINTS;
        // then call the kernel
        if (USE_QDD_FLAG) {
            if (USE_COMPRESSED_MEM) {inverse_dynamics_gradient_kernel<T><<<block_dimms,thread_dimms,ID_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_dc_du,hd_data->d_q_qd,stride_q_qd,hd_data->d_qdd, d_robotModel,gravity,num_timesteps);}
            else                    {inverse_dynamics_gradient_kernel<T><<<block_dimms,thread_dimms,ID_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_dc_du,hd_data->d_q_qd_u,stride_q_qd,hd_data->d_qdd, d_robotModel,gravity,num_timesteps);}
        }
        else {
            if (USE_COMPRESSED_MEM) {inverse_dynamics_gradient_kernel<T><<<block_dimms,thread_dimms,ID_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_dc_du,hd_data->d_q_qd,stride_q_qd,d_robotModel,gravity,num_timesteps);}
            else                    {inverse_dynamics_gradient_kernel<T><<<block_dimms,thread_dimms,ID_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_dc_du,hd_data->d_q_qd_u,stride_q_qd,d_robotModel,gravity,num_timesteps);}
        }
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Computes the gradient of forward dynamics
     *
     * Notes:
     *   Uses the fd/du = -Minv*id/du trick as described in Carpentier and Mansrud 'Analytical Derivatives of Rigid Body Dynamics Algorithms'
     *
     * @param s_df_du is a pointer to memory for the final result of size 2*NUM_JOINTS*NUM_JOINTS = 288
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param s_u is the vector of input torques
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void forward_dynamics_gradient_device(T *s_df_du, const T *s_q, const T *s_qd, const T *s_u, const robotModel<T> *d_robotModel, const T gravity) {
        __shared__ T s_vaf[216];
        __shared__ T s_dc_du[288];
        __shared__ T s_Minv[144];
        __shared__ T s_qdd[12];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
        //TODO: there is a slightly faster way as s_v does not change -- thus no recompute needed
        direct_minv_inner<T>(s_Minv, s_q, s_XImats, s_topology_helpers, s_temp);
        inverse_dynamics_inner<T>(s_temp, s_vaf, s_q, s_qd, s_XImats, s_topology_helpers, &s_temp[12], gravity);
        forward_dynamics_finish<T>(s_qdd, s_u, s_temp, s_Minv);
        inverse_dynamics_inner_vaf<T>(s_vaf, s_q, s_qd, s_qdd, s_XImats, s_topology_helpers, s_temp, gravity);
        inverse_dynamics_gradient_inner<T>(s_dc_du, s_q, s_qd, s_vaf, s_XImats, s_topology_helpers, s_temp, gravity);
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Minv\n");
            printMat<T,12,12>(s_Minv,12);
            printf("qdd\n");
            printMat<T,1,12>(s_qdd,1);
            printf("v\n");
            printMat<T,6,12>(s_vaf,6);
            printf("a\n");
            printMat<T,6,12>(&s_vaf[6*12],6);
            printf("f\n");
            printMat<T,6,12>(&s_vaf[12*12],6);
            printf("dc/dq\n");
            printMat<T,12,12>(&s_dc_du[0],12);
            printf("dc/dqd\n");
            printMat<T,12,12>(&s_dc_du[144],12);
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
            int row = ind % 12; int dc_col_offset = ind - row;
            // account for the fact that Minv is an SYMMETRIC_UPPER triangular matrix
            T val = static_cast<T>(0);
            for(int col = 0; col < 12; col++) {
                int index = (row <= col) * (col * 12 + row) + (row > col) * (row * 12 + col);
                val += s_Minv[index] * s_dc_du[dc_col_offset + col];
            }
            s_df_du[ind] = -val;
        }
    }

    /**
     * Computes the gradient of forward dynamics
     *
     * Notes:
     *   Uses the fd/du = -Minv*id/du trick as described in Carpentier and Mansrud 'Analytical Derivatives of Rigid Body Dynamics Algorithms'
     *
     * @param s_df_du is a pointer to memory for the final result of size 2*NUM_JOINTS*NUM_JOINTS = 288
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param s_qdd is the vector of joint accelerations
     * @param s_Minv is the mass matrix
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void forward_dynamics_gradient_device(T *s_df_du, const T *s_q, const T *s_qd, const T *s_qdd, const T *s_Minv, const robotModel<T> *d_robotModel, const T gravity) {
        __shared__ T s_vaf[216];
        __shared__ T s_dc_du[288];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
        inverse_dynamics_inner_vaf<T>(s_vaf, s_q, s_qd, s_qdd, s_XImats, s_topology_helpers, s_temp, gravity);
        inverse_dynamics_gradient_inner<T>(s_dc_du, s_q, s_qd, s_vaf, s_XImats, s_topology_helpers, s_temp, gravity);
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("Minv\n");
            printMat<T,12,12>(s_Minv,12);
            printf("qdd\n");
            printMat<T,1,12>(s_qdd,1);
            printf("v\n");
            printMat<T,6,12>(s_vaf,6);
            printf("a\n");
            printMat<T,6,12>(&s_vaf[6*12],6);
            printf("f\n");
            printMat<T,6,12>(&s_vaf[12*12],6);
            printf("dc/dq\n");
            printMat<T,12,12>(&s_dc_du[0],12);
            printf("dc/dqd\n");
            printMat<T,12,12>(&s_dc_du[144],12);
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
            int row = ind % 12; int dc_col_offset = ind - row;
            // account for the fact that Minv is an SYMMETRIC_UPPER triangular matrix
            T val = static_cast<T>(0);
            for(int col = 0; col < 12; col++) {
                int index = (row <= col) * (col * 12 + row) + (row > col) * (row * 12 + col);
                val += s_Minv[index] * s_dc_du[dc_col_offset + col];
            }
            s_df_du[ind] = -val;
        }
    }

    /**
     * Computes the gradient of forward dynamics
     *
     * @param d_df_du is a pointer to memory for the final result of size 2*NUM_JOINTS*NUM_JOINTS = 288
     * @param d_q_dq is the vector of joint positions and velocities
     * @param stride_q_qd is the stide between each q, qd
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param d_qdd is the vector of joint accelerations
     * @param d_Minv is the mass matrix
     * @param gravity is the gravity constant
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void forward_dynamics_gradient_kernel_single_timing(T *d_df_du, const T *d_q_qd, const int stride_q_qd, const T *d_qdd, const T *d_Minv, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_q_qd[24]; T *s_q = s_q_qd; T *s_qd = &s_q_qd[12];
        __shared__ T s_dc_du[288];
        __shared__ T s_vaf[216];
        __shared__ T s_qdd[12];
        __shared__ T s_Minv[144];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        // load to shared mem
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            s_q_qd[ind] = d_q_qd[ind];
        }
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
            s_qdd[ind] = d_qdd[ind];
        }
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            s_Minv[ind] = d_Minv[ind];
        }
        __syncthreads();
        // compute with NUM_TIMESTEPS as NUM_REPS for timing
        for (int rep = 0; rep < NUM_TIMESTEPS; rep++){
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            inverse_dynamics_inner_vaf<T>(s_vaf, s_q, s_qd, s_qdd, s_XImats, s_topology_helpers, s_temp, gravity);
            inverse_dynamics_gradient_inner<T>(s_dc_du, s_q, s_qd, s_vaf, s_XImats, s_topology_helpers, s_temp, gravity);
            __syncthreads();
            if(threadIdx.x == 0 && threadIdx.y == 0){
                printf("Minv\n");
                printMat<T,12,12>(s_Minv,12);
                printf("qdd\n");
                printMat<T,1,12>(s_qdd,1);
                printf("v\n");
                printMat<T,6,12>(s_vaf,6);
                printf("a\n");
                printMat<T,6,12>(&s_vaf[6*12],6);
                printf("f\n");
                printMat<T,6,12>(&s_vaf[12*12],6);
                printf("dc/dq\n");
                printMat<T,12,12>(&s_dc_du[0],12);
                printf("dc/dqd\n");
                printMat<T,12,12>(&s_dc_du[144],12);
            }
            __syncthreads();
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
                int row = ind % 12; int dc_col_offset = ind - row;
                // account for the fact that Minv is an SYMMETRIC_UPPER triangular matrix
                T val = static_cast<T>(0);
                for(int col = 0; col < 12; col++) {
                    int index = (row <= col) * (col * 12 + row) + (row > col) * (row * 12 + col);
                    val += s_Minv[index] * s_dc_du[dc_col_offset + col];
                }
                s_temp[ind] = -val;
            }
        }
        // save down to global
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
            d_df_du[ind] = s_temp[ind];
        }
        __syncthreads();
    }

    /**
     * Computes the gradient of forward dynamics
     *
     * @param d_df_du is a pointer to memory for the final result of size 2*NUM_JOINTS*NUM_JOINTS = 288
     * @param d_q_dq is the vector of joint positions and velocities
     * @param stride_q_qd is the stide between each q, qd
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param d_qdd is the vector of joint accelerations
     * @param d_Minv is the mass matrix
     * @param gravity is the gravity constant
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void forward_dynamics_gradient_kernel(T *d_df_du, const T *d_q_qd, const int stride_q_qd, const T *d_qdd, const T *d_Minv, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_q_qd[24]; T *s_q = s_q_qd; T *s_qd = &s_q_qd[12];
        __shared__ T s_dc_du[288];
        __shared__ T s_vaf[216];
        __shared__ T s_qdd[12];
        __shared__ T s_Minv[144];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        for(int k = blockIdx.x + blockIdx.y*gridDim.x; k < NUM_TIMESTEPS; k += gridDim.x*gridDim.y){
            // load to shared mem
            const T *d_q_qd_k = &d_q_qd[k*stride_q_qd];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
                s_q_qd[ind] = d_q_qd_k[ind];
            }
            const T *d_qdd_k = &d_qdd[k*12];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
                s_qdd[ind] = d_qdd_k[ind];
            }
            const T *d_Minv_k = &d_Minv[k*144];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
                s_Minv[ind] = d_Minv_k[ind];
            }
            __syncthreads();
            // compute
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            inverse_dynamics_inner_vaf<T>(s_vaf, s_q, s_qd, s_qdd, s_XImats, s_topology_helpers, s_temp, gravity);
            inverse_dynamics_gradient_inner<T>(s_dc_du, s_q, s_qd, s_vaf, s_XImats, s_topology_helpers, s_temp, gravity);
            __syncthreads();
            if(threadIdx.x == 0 && threadIdx.y == 0){
                printf("Minv\n");
                printMat<T,12,12>(s_Minv,12);
                printf("qdd\n");
                printMat<T,1,12>(s_qdd,1);
                printf("v\n");
                printMat<T,6,12>(s_vaf,6);
                printf("a\n");
                printMat<T,6,12>(&s_vaf[6*12],6);
                printf("f\n");
                printMat<T,6,12>(&s_vaf[12*12],6);
                printf("dc/dq\n");
                printMat<T,12,12>(&s_dc_du[0],12);
                printf("dc/dqd\n");
                printMat<T,12,12>(&s_dc_du[144],12);
            }
            __syncthreads();
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
                int row = ind % 12; int dc_col_offset = ind - row;
                // account for the fact that Minv is an SYMMETRIC_UPPER triangular matrix
                T val = static_cast<T>(0);
                for(int col = 0; col < 12; col++) {
                    int index = (row <= col) * (col * 12 + row) + (row > col) * (row * 12 + col);
                    val += s_Minv[index] * s_dc_du[dc_col_offset + col];
                }
                s_temp[ind] = -val;
            }
            // save down to global
            T *d_df_du_k = &d_df_du[k*288];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
                d_df_du_k[ind] = s_temp[ind];
            }
            __syncthreads();
        }
    }

    /**
     * Computes the gradient of forward dynamics
     *
     * @param d_df_du is a pointer to memory for the final result of size 2*NUM_JOINTS*NUM_JOINTS = 288
     * @param d_q_dq is the vector of joint positions, velocities, and input torques
     * @param stride_q_qd_u is the stide between each q, qd, u
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void forward_dynamics_gradient_kernel_single_timing(T *d_df_du, const T *d_q_qd_u, const int stride_q_qd_u, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_q_qd_u[36]; T *s_q = s_q_qd_u; T *s_qd = &s_q_qd_u[12]; T *s_u = &s_q_qd_u[24];
        __shared__ T s_dc_du[288];
        __shared__ T s_vaf[216];
        __shared__ T s_qdd[12];
        __shared__ T s_Minv[144];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        // load to shared mem
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            s_q_qd_u[ind] = d_q_qd_u[ind];
        }
        __syncthreads();
        // compute with NUM_TIMESTEPS as NUM_REPS for timing
        for (int rep = 0; rep < NUM_TIMESTEPS; rep++){
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            //TODO: there is a slightly faster way as s_v does not change -- thus no recompute needed
            direct_minv_inner<T>(s_Minv, s_q, s_XImats, s_topology_helpers, s_temp);
            inverse_dynamics_inner<T>(s_temp, s_vaf, s_q, s_qd, s_XImats, s_topology_helpers, &s_temp[12], gravity);
            forward_dynamics_finish<T>(s_qdd, s_u, s_temp, s_Minv);
            inverse_dynamics_inner_vaf<T>(s_vaf, s_q, s_qd, s_qdd, s_XImats, s_topology_helpers, s_temp, gravity);
            inverse_dynamics_gradient_inner<T>(s_dc_du, s_q, s_qd, s_vaf, s_XImats, s_topology_helpers, s_temp, gravity);
            __syncthreads();
            if(threadIdx.x == 0 && threadIdx.y == 0){
                printf("Minv\n");
                printMat<T,12,12>(s_Minv,12);
                printf("qdd\n");
                printMat<T,1,12>(s_qdd,1);
                printf("v\n");
                printMat<T,6,12>(s_vaf,6);
                printf("a\n");
                printMat<T,6,12>(&s_vaf[6*12],6);
                printf("f\n");
                printMat<T,6,12>(&s_vaf[12*12],6);
                printf("dc/dq\n");
                printMat<T,12,12>(&s_dc_du[0],12);
                printf("dc/dqd\n");
                printMat<T,12,12>(&s_dc_du[144],12);
            }
            __syncthreads();
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
                int row = ind % 12; int dc_col_offset = ind - row;
                // account for the fact that Minv is an SYMMETRIC_UPPER triangular matrix
                T val = static_cast<T>(0);
                for(int col = 0; col < 12; col++) {
                    int index = (row <= col) * (col * 12 + row) + (row > col) * (row * 12 + col);
                    val += s_Minv[index] * s_dc_du[dc_col_offset + col];
                }
                s_temp[ind] = -val;
            }
        }
        // save down to global
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
            d_df_du[ind] = s_temp[ind];
        }
        __syncthreads();
    }

    /**
     * Computes the gradient of forward dynamics
     *
     * @param d_df_du is a pointer to memory for the final result of size 2*NUM_JOINTS*NUM_JOINTS = 288
     * @param d_q_dq is the vector of joint positions, velocities, and input torques
     * @param stride_q_qd_u is the stide between each q, qd, u
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void forward_dynamics_gradient_kernel(T *d_df_du, const T *d_q_qd_u, const int stride_q_qd_u, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_q_qd_u[36]; T *s_q = s_q_qd_u; T *s_qd = &s_q_qd_u[12]; T *s_u = &s_q_qd_u[24];
        __shared__ T s_dc_du[288];
        __shared__ T s_vaf[216];
        __shared__ T s_qdd[12];
        __shared__ T s_Minv[144];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        for(int k = blockIdx.x + blockIdx.y*gridDim.x; k < NUM_TIMESTEPS; k += gridDim.x*gridDim.y){
            // load to shared mem
            const T *d_q_qd_u_k = &d_q_qd_u[k*stride_q_qd_u];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
                s_q_qd_u[ind] = d_q_qd_u_k[ind];
            }
            __syncthreads();
            // compute
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            //TODO: there is a slightly faster way as s_v does not change -- thus no recompute needed
            direct_minv_inner<T>(s_Minv, s_q, s_XImats, s_topology_helpers, s_temp);
            inverse_dynamics_inner<T>(s_temp, s_vaf, s_q, s_qd, s_XImats, s_topology_helpers, &s_temp[12], gravity);
            forward_dynamics_finish<T>(s_qdd, s_u, s_temp, s_Minv);
            inverse_dynamics_inner_vaf<T>(s_vaf, s_q, s_qd, s_qdd, s_XImats, s_topology_helpers, s_temp, gravity);
            inverse_dynamics_gradient_inner<T>(s_dc_du, s_q, s_qd, s_vaf, s_XImats, s_topology_helpers, s_temp, gravity);
            __syncthreads();
            if(threadIdx.x == 0 && threadIdx.y == 0){
                printf("Minv\n");
                printMat<T,12,12>(s_Minv,12);
                printf("qdd\n");
                printMat<T,1,12>(s_qdd,1);
                printf("v\n");
                printMat<T,6,12>(s_vaf,6);
                printf("a\n");
                printMat<T,6,12>(&s_vaf[6*12],6);
                printf("f\n");
                printMat<T,6,12>(&s_vaf[12*12],6);
                printf("dc/dq\n");
                printMat<T,12,12>(&s_dc_du[0],12);
                printf("dc/dqd\n");
                printMat<T,12,12>(&s_dc_du[144],12);
            }
            __syncthreads();
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
                int row = ind % 12; int dc_col_offset = ind - row;
                // account for the fact that Minv is an SYMMETRIC_UPPER triangular matrix
                T val = static_cast<T>(0);
                for(int col = 0; col < 12; col++) {
                    int index = (row <= col) * (col * 12 + row) + (row > col) * (row * 12 + col);
                    val += s_Minv[index] * s_dc_du[dc_col_offset + col];
                }
                s_temp[ind] = -val;
            }
            // save down to global
            T *d_df_du_k = &d_df_du[k*288];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 288; ind += blockDim.x*blockDim.y){
                d_df_du_k[ind] = s_temp[ind];
            }
            __syncthreads();
        }
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_QDD_MINV_FLAG = false>
    __host__
    void forward_dynamics_gradient(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                          const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        int stride_q_qd= 3*NUM_JOINTS;
        // start code with memory transfer
        gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q_qd*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[0]));
        if (USE_QDD_MINV_FLAG) {
            gpuErrchk(cudaMemcpyAsync(hd_data->d_qdd,hd_data->h_qdd,NUM_JOINTS*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[1]));
            gpuErrchk(cudaMemcpyAsync(hd_data->d_Minv,hd_data->h_Minv,NUM_JOINTS*NUM_JOINTS*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[2]));
        }
        gpuErrchk(cudaDeviceSynchronize());
        // then call the kernel
        if (USE_QDD_MINV_FLAG) {forward_dynamics_gradient_kernel<T><<<block_dimms,thread_dimms,FD_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_df_du,hd_data->d_q_qd_u,stride_q_qd,hd_data->d_qdd, hd_data->d_Minv, d_robotModel,gravity,num_timesteps);}
        else {forward_dynamics_gradient_kernel<T><<<block_dimms,thread_dimms,FD_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_df_du,hd_data->d_q_qd_u,stride_q_qd,d_robotModel,gravity,num_timesteps);}
        gpuErrchk(cudaDeviceSynchronize());
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_df_du,hd_data->d_df_du,NUM_JOINTS*2*NUM_JOINTS*num_timesteps*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_QDD_MINV_FLAG = false>
    __host__
    void forward_dynamics_gradient_single_timing(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                                        const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        int stride_q_qd= 3*NUM_JOINTS;
        // start code with memory transfer
        gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q_qd*sizeof(T),cudaMemcpyHostToDevice,streams[0]));
        if (USE_QDD_MINV_FLAG) {
            gpuErrchk(cudaMemcpyAsync(hd_data->d_qdd,hd_data->h_qdd,NUM_JOINTS*sizeof(T),cudaMemcpyHostToDevice,streams[1]));
            gpuErrchk(cudaMemcpyAsync(hd_data->d_Minv,hd_data->h_Minv,NUM_JOINTS*NUM_JOINTS*sizeof(T),cudaMemcpyHostToDevice,streams[2]));
        }
        gpuErrchk(cudaDeviceSynchronize());
        // then call the kernel
        struct timespec start, end; clock_gettime(CLOCK_MONOTONIC,&start);
        if (USE_QDD_MINV_FLAG) {forward_dynamics_gradient_kernel_single_timing<T><<<block_dimms,thread_dimms,FD_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_df_du,hd_data->d_q_qd_u,stride_q_qd,hd_data->d_qdd, hd_data->d_Minv, d_robotModel,gravity,num_timesteps);}
        else {forward_dynamics_gradient_kernel_single_timing<T><<<block_dimms,thread_dimms,FD_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_df_du,hd_data->d_q_qd_u,stride_q_qd,d_robotModel,gravity,num_timesteps);}
        gpuErrchk(cudaDeviceSynchronize());
        clock_gettime(CLOCK_MONOTONIC,&end);
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_df_du,hd_data->d_df_du,NUM_JOINTS*2*NUM_JOINTS*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
        printf("Single Call FD_DU %fus\n",time_delta_us_timespec(start,end)/static_cast<double>(num_timesteps));
    }

    /**
     * Compute the RNEA (Recursive Newton-Euler Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_QDD_MINV_FLAG = false>
    __host__
    void forward_dynamics_gradient_compute_only(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                                       const dim3 block_dimms, const dim3 thread_dimms) {
        int stride_q_qd= 3*NUM_JOINTS;
        // then call the kernel
        if (USE_QDD_MINV_FLAG) {forward_dynamics_gradient_kernel<T><<<block_dimms,thread_dimms,FD_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_df_du,hd_data->d_q_qd_u,stride_q_qd,hd_data->d_qdd, hd_data->d_Minv, d_robotModel,gravity,num_timesteps);}
        else {forward_dynamics_gradient_kernel<T><<<block_dimms,thread_dimms,FD_DU_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_df_du,hd_data->d_q_qd_u,stride_q_qd,d_robotModel,gravity,num_timesteps);}
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Computes the Articulated Body Algorithm
     *
     * Notes:
     *   Assumes the XI matricies have already been updated for the given q
     *
     * @param s_qdd is the vector of joint accelerations
     * @param s_va is a pointer to shared memory of size 2*6*NUM_JOINTS = 144
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param s_tau is the vector of joint torques
     * @param s_XImats is the (shared) memory holding the updated XI matricies for the given s_q
     * @param s_topology_helpers is the (shared) memory destination location for the topology_helpers
     * @param s_temp is the pointer to the shared memory needed of size: 2112
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void aba_inner(T *s_qdd, T *s_va, const T *s_q, const T *s_qd, const T *s_tau, T *s_XImats, int *s_topology_helpers, T *s_temp, const T gravity) {
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("q\n"); printMat<T,1,12>(s_q,1);
            printf("qd\n"); printMat<T,1,12>(s_qd,1);
            for (int i = 0; i < 12; i++){printf("X[%d]\n",i); printMat<T,6,6>(&s_XImats[36*i],6);}
            for (int i = 0; i < 12; i++){printf("I[%d]\n",i); printMat<T,6,6>(&s_XImats[36*(i+12)],6);}
        }
        __syncthreads();
        //
        // Forward Pass
        //
        // s_v where parent is base
        //     joints are: lf_haa_joint, lh_haa_joint, rf_haa_joint, rh_haa_joint
        //     links are: lf_hipassembly, lh_hipassembly, rf_hipassembly, rh_hipassembly
        // s_v[k] = S[k]*qd[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 0 + (ind < 12 && ind >= 6) * 3 + (ind < 18 && ind >= 12) * 6 + (ind >= 18) * 9;
            int jid6 = 6*jid;
            s_va[jid6 + row] = static_cast<T>(0);
            if (row == 2){s_va[jid6 + 2] += s_qd[jid];}
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_v[0]\n"); printMat<T,1,6>(&s_va[6*0],1);
            printf("s_v[3]\n"); printMat<T,1,6>(&s_va[6*3],1);
            printf("s_v[6]\n"); printMat<T,1,6>(&s_va[6*6],1);
            printf("s_v[9]\n"); printMat<T,1,6>(&s_va[6*9],1);
        }
        __syncthreads();
        // s_v where bfs_level is 1
        //     joints are: lf_hfe_joint, lh_hfe_joint, rf_hfe_joint, rh_hfe_joint
        //     links are: lf_upperleg, lh_upperleg, rf_upperleg, rh_upperleg
        // s_v[k] = X[k]*v[parent_k] + S[k]*qd[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int comp_mod = comp % 4;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 1 + (comp_mod == 1) * 4 + (comp_mod == 2) * 7 + (comp_mod == 3) * 10;
            int jid6 = 6 * jid;
            T qd_val = (row == 2) * (s_qd[jid]);
            s_va[jid6 + row] = dot_prod<T,6,6,1>(&s_XImats[6*jid6 + row], &s_va[6*s_topology_helpers[jid]]) + qd_val;
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_v[1] = X*s_v[s_topology_helpers[jid]] + S*qd[1]\n"); printMat<T,1,6>(&s_va[6*1],1);
            printf("s_v[4] = X*s_v[s_topology_helpers[jid]] + S*qd[4]\n"); printMat<T,1,6>(&s_va[6*4],1);
            printf("s_v[7] = X*s_v[s_topology_helpers[jid]] + S*qd[7]\n"); printMat<T,1,6>(&s_va[6*7],1);
            printf("s_v[10] = X*s_v[s_topology_helpers[jid]] + S*qd[10]\n"); printMat<T,1,6>(&s_va[6*10],1);
        }
        __syncthreads();
        // s_v where bfs_level is 2
        //     joints are: lf_kfe_joint, lh_kfe_joint, rf_kfe_joint, rh_kfe_joint
        //     links are: lf_lowerleg, lh_lowerleg, rf_lowerleg, rh_lowerleg
        // s_v[k] = X[k]*v[parent_k] + S[k]*qd[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int comp_mod = comp % 4;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 2 + (comp_mod == 1) * 5 + (comp_mod == 2) * 8 + (comp_mod == 3) * 11;
            int jid6 = 6 * jid;
            T qd_val = (row == 2) * (s_qd[jid]);
            s_va[jid6 + row] = dot_prod<T,6,6,1>(&s_XImats[6*jid6 + row], &s_va[6*s_topology_helpers[jid]]) + qd_val;
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("s_v[2] = X*s_v[s_topology_helpers[jid]] + S*qd[2]\n"); printMat<T,1,6>(&s_va[6*2],1);
            printf("s_v[5] = X*s_v[s_topology_helpers[jid]] + S*qd[5]\n"); printMat<T,1,6>(&s_va[6*5],1);
            printf("s_v[8] = X*s_v[s_topology_helpers[jid]] + S*qd[8]\n"); printMat<T,1,6>(&s_va[6*8],1);
            printf("s_v[11] = X*s_v[s_topology_helpers[jid]] + S*qd[11]\n"); printMat<T,1,6>(&s_va[6*11],1);
        }
        __syncthreads();
        // c[k] = mxS(v[k])*qd[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
            int jid = ind;
            int jid6 = 6 * jid;
            mx2_scaled<T>(&s_temp[72 * 12+jid6], &s_va[jid6], s_qd[jid]);
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("c\n"); printMat<T,6,12>(&s_temp[72 * 12], 6);
        }
        __syncthreads();
        // Initialize IA = I
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 432; ind += blockDim.x*blockDim.y){
            s_temp[ind] = s_XImats[432 + ind];
        }
        // Initialize vcross[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
            int jid = ind;
            int jid6 = 6 * jid;
            vcross<T>(&s_temp[36*(12+jid)], &s_va[jid6]);
        }
        __syncthreads();
        // temp[k] = -vcross.T*I[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 432; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = (ind / 6) %6; int jid = ind / 36;
            int jid6 = 6 * jid;
            s_temp[98 * 12 + jid6*6 + row+col*6] = -1 * dot_prod<T,6,1,1>(&s_temp[36*(12+jid)+row*6], &s_XImats[36 * (12+jid) + col*6]);
        }
        __syncthreads();
        // pA[k] = temp[k]*v[k][0]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 72; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int jid = comp % 12;
            int jid6 = 6 * jid;
            s_temp[78 * 12 + jid6 + row] = dot_prod<T,6,6,1>(&s_temp[98 * 12 + 6*jid6+row], &s_va[jid6]);
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            for (int i = 0; i < 12; i++){printf("IA[%d]\n",i); printMat<T,6,6>(&s_temp[36*(i)],6);}
            printf("pA\n"); printMat<T,6,12>(&s_temp[78 * 12], 6);
        }
        __syncthreads();
        //
        // Backward Pass
        //
        // Backward pass where bfs_level is 2
        //     joints are: lf_kfe_joint, lh_kfe_joint, rf_kfe_joint, rh_kfe_joint
        //     links are: lf_lowerleg, lh_lowerleg, rf_lowerleg, rh_lowerleg
        // U[k] = IA[k]*S[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 2 + (ind < 12 && ind >= 6) * 5 + (ind < 18 && ind >= 12) * 8 + (ind >= 18) * 11;
            int jid6 = 6 * jid;
            s_temp[84*12+jid6+row] = s_temp[36*jid+row+6*(2)];
        }
        __syncthreads();
        // d[k] = S[k]*U[k], u[k] = tau[k] - S[k].T*pA[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 4; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind < 1) * 2 + (ind < 2 && ind >= 1) * 5 + (ind < 3 && ind >= 2) * 8 + (ind >= 3) * 11;
            int jid6 = 6 * jid;
            s_temp[96 * 12 + jid] = s_temp[84 * 12 + jid6 + 2];
            T tempval = s_temp[78 * 12 + jid6 + 2];
            s_temp[97 * 12 + jid] = s_tau[jid] - tempval;
        }
        __syncthreads();
        // Ia[k] = IA[k] - U[k]*U[k].T/d[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = (ind / 6) %6;
            // non-branching pointer selector
            int jid = (ind < 36) * 2 + (ind < 72 && ind >= 36) * 5 + (ind < 108 && ind >= 72) * 8 + (ind >= 108) * 11;
            int jid6 = 6 * jid;
            s_temp[36 * 12+6*jid6+row+6*col] = s_temp[84*12+jid6+row]*s_temp[84*12+jid6+col]/s_temp[96 *12+jid];
            s_temp[36 * 12+6*jid6+row+6*col] = s_temp[6*jid6+row+6*col] - s_temp[36 * 12+6*jid6+row+6*col];
        }
        __syncthreads();
        // pa[k] = pA[k] + Ia[k]*c[k]+U[k]*u[k]/d[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 2 + (ind < 12 && ind >= 6) * 5 + (ind < 18 && ind >= 12) * 8 + (ind >= 18) * 11;
            int jid6 = 6 * jid;
            T Uval = s_temp[84 * 12+jid6+row]*s_temp[97*12+jid]/s_temp[96*12+jid];
            s_temp[90 * 12 + jid6 + row] = s_temp[78 * 12 + jid6+row] + dot_prod<T,6,6,1>(&s_temp[36*(12+jid)+row], &s_temp[72*12+jid6]) + Uval;
        }
        // temp[k] = X[k].T*Ia[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = (ind / 6) %6;
            // non-branching pointer selector
            int jid = (ind < 36) * 2 + (ind < 72 && ind >= 36) * 5 + (ind < 108 && ind >= 72) * 8 + (ind >= 108) * 11;
            int jid6 = 6 * jid;
            s_temp[98 * 12 + 6 * jid6 + row + 6*col] = dot_prod<T,6,1,1>(&s_XImats[6*jid6+6*row], &s_temp[36 * 12+jid6*6+6*col]);
        }
        __syncthreads();
        // IA[parent] += temp[k]*X[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = (ind / 6) %6;
            // non-branching pointer selector
            int jid = (ind < 36) * 2 + (ind < 72 && ind >= 36) * 5 + (ind < 108 && ind >= 72) * 8 + (ind >= 108) * 11;
            int jid6 = 6 * jid;
            T prodtemp = static_cast<T>(0);
            prodtemp =  dot_prod<T,6,6,1>(&s_temp[98 * 12 + 6 * jid6 + row], &s_XImats[6*jid6+6*col]);
            atomicAdd(&s_temp[36 * s_topology_helpers[jid] + row + 6*col], prodtemp);
        }
        __syncthreads();
        // pA[parent] += X[k].T*pa[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 2 + (ind < 12 && ind >= 6) * 5 + (ind < 18 && ind >= 12) * 8 + (ind >= 18) * 11;
            int jid6 = 6 * jid;
            s_temp[134 * 12 + jid6 + row] = dot_prod<T,6,1,1>(&s_XImats[36*jid+6*row],&s_temp[90*12+jid6]);
            atomicAdd(&s_temp[78 * 12 + 6 * s_topology_helpers[jid] + row], s_temp[134 * 12 + jid6 + row]);
        }
        __syncthreads();
        // Backward pass where bfs_level is 1
        //     joints are: lf_hfe_joint, lh_hfe_joint, rf_hfe_joint, rh_hfe_joint
        //     links are: lf_upperleg, lh_upperleg, rf_upperleg, rh_upperleg
        // U[k] = IA[k]*S[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 1 + (ind < 12 && ind >= 6) * 4 + (ind < 18 && ind >= 12) * 7 + (ind >= 18) * 10;
            int jid6 = 6 * jid;
            s_temp[84*12+jid6+row] = s_temp[36*jid+row+6*(2)];
        }
        __syncthreads();
        // d[k] = S[k]*U[k], u[k] = tau[k] - S[k].T*pA[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 4; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind < 1) * 1 + (ind < 2 && ind >= 1) * 4 + (ind < 3 && ind >= 2) * 7 + (ind >= 3) * 10;
            int jid6 = 6 * jid;
            s_temp[96 * 12 + jid] = s_temp[84 * 12 + jid6 + 2];
            T tempval = s_temp[78 * 12 + jid6 + 2];
            s_temp[97 * 12 + jid] = s_tau[jid] - tempval;
        }
        __syncthreads();
        // Ia[k] = IA[k] - U[k]*U[k].T/d[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = (ind / 6) %6;
            // non-branching pointer selector
            int jid = (ind < 36) * 1 + (ind < 72 && ind >= 36) * 4 + (ind < 108 && ind >= 72) * 7 + (ind >= 108) * 10;
            int jid6 = 6 * jid;
            s_temp[36 * 12+6*jid6+row+6*col] = s_temp[84*12+jid6+row]*s_temp[84*12+jid6+col]/s_temp[96 *12+jid];
            s_temp[36 * 12+6*jid6+row+6*col] = s_temp[6*jid6+row+6*col] - s_temp[36 * 12+6*jid6+row+6*col];
        }
        __syncthreads();
        // pa[k] = pA[k] + Ia[k]*c[k]+U[k]*u[k]/d[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 1 + (ind < 12 && ind >= 6) * 4 + (ind < 18 && ind >= 12) * 7 + (ind >= 18) * 10;
            int jid6 = 6 * jid;
            T Uval = s_temp[84 * 12+jid6+row]*s_temp[97*12+jid]/s_temp[96*12+jid];
            s_temp[90 * 12 + jid6 + row] = s_temp[78 * 12 + jid6+row] + dot_prod<T,6,6,1>(&s_temp[36*(12+jid)+row], &s_temp[72*12+jid6]) + Uval;
        }
        // temp[k] = X[k].T*Ia[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = (ind / 6) %6;
            // non-branching pointer selector
            int jid = (ind < 36) * 1 + (ind < 72 && ind >= 36) * 4 + (ind < 108 && ind >= 72) * 7 + (ind >= 108) * 10;
            int jid6 = 6 * jid;
            s_temp[98 * 12 + 6 * jid6 + row + 6*col] = dot_prod<T,6,1,1>(&s_XImats[6*jid6+6*row], &s_temp[36 * 12+jid6*6+6*col]);
        }
        __syncthreads();
        // IA[parent] += temp[k]*X[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = (ind / 6) %6;
            // non-branching pointer selector
            int jid = (ind < 36) * 1 + (ind < 72 && ind >= 36) * 4 + (ind < 108 && ind >= 72) * 7 + (ind >= 108) * 10;
            int jid6 = 6 * jid;
            T prodtemp = static_cast<T>(0);
            prodtemp =  dot_prod<T,6,6,1>(&s_temp[98 * 12 + 6 * jid6 + row], &s_XImats[6*jid6+6*col]);
            atomicAdd(&s_temp[36 * s_topology_helpers[jid] + row + 6*col], prodtemp);
        }
        __syncthreads();
        // pA[parent] += X[k].T*pa[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 1 + (ind < 12 && ind >= 6) * 4 + (ind < 18 && ind >= 12) * 7 + (ind >= 18) * 10;
            int jid6 = 6 * jid;
            s_temp[134 * 12 + jid6 + row] = dot_prod<T,6,1,1>(&s_XImats[36*jid+6*row],&s_temp[90*12+jid6]);
            atomicAdd(&s_temp[78 * 12 + 6 * s_topology_helpers[jid] + row], s_temp[134 * 12 + jid6 + row]);
        }
        __syncthreads();
        // Backward pass where bfs_level is 0
        //     joints are: lf_haa_joint, lh_haa_joint, rf_haa_joint, rh_haa_joint
        //     links are: lf_hipassembly, lh_hipassembly, rf_hipassembly, rh_hipassembly
        // U[k] = IA[k]*S[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 0 + (ind < 12 && ind >= 6) * 3 + (ind < 18 && ind >= 12) * 6 + (ind >= 18) * 9;
            int jid6 = 6 * jid;
            s_temp[84*12+jid6+row] = s_temp[36*jid+row+6*(2)];
        }
        __syncthreads();
        // d[k] = S[k]*U[k], u[k] = tau[k] - S[k].T*pA[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 4; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind < 1) * 0 + (ind < 2 && ind >= 1) * 3 + (ind < 3 && ind >= 2) * 6 + (ind >= 3) * 9;
            int jid6 = 6 * jid;
            s_temp[96 * 12 + jid] = s_temp[84 * 12 + jid6 + 2];
            T tempval = s_temp[78 * 12 + jid6 + 2];
            s_temp[97 * 12 + jid] = s_tau[jid] - tempval;
        }
        __syncthreads();
        // Ia[k] = IA[k] - U[k]*U[k].T/d[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int col = (ind / 6) %6;
            // non-branching pointer selector
            int jid = (ind < 36) * 0 + (ind < 72 && ind >= 36) * 3 + (ind < 108 && ind >= 72) * 6 + (ind >= 108) * 9;
            int jid6 = 6 * jid;
            s_temp[36 * 12+6*jid6+row+6*col] = s_temp[84*12+jid6+row]*s_temp[84*12+jid6+col]/s_temp[96 *12+jid];
            s_temp[36 * 12+6*jid6+row+6*col] = s_temp[6*jid6+row+6*col] - s_temp[36 * 12+6*jid6+row+6*col];
        }
        __syncthreads();
        // pa[k] = pA[k] + Ia[k]*c[k]+U[k]*u[k]/d[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 0 + (ind < 12 && ind >= 6) * 3 + (ind < 18 && ind >= 12) * 6 + (ind >= 18) * 9;
            int jid6 = 6 * jid;
            T Uval = s_temp[84 * 12+jid6+row]*s_temp[97*12+jid]/s_temp[96*12+jid];
            s_temp[90 * 12 + jid6 + row] = s_temp[78 * 12 + jid6+row] + dot_prod<T,6,6,1>(&s_temp[36*(12+jid)+row], &s_temp[72*12+jid6]) + Uval;
        }
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("U \n"); printMat<T,6,12>(&s_temp[84 * 12], 6);
            printf("d \n"); printMat<T,1,12>(&s_temp[96 * 12], 1);
            printf("u \n"); printMat<T,1,12>(&s_temp[97 * 12], 1);
            for (int i = 0; i < 12; i++){printf("Ia[%d]\n",i); printMat<T,6,6>(&s_temp[36*(12+i)],6);}
            for (int i = 0; i < 12; i++){printf("IA[%d]\n",i); printMat<T,6,6>(&s_temp[36*(i)],6);}
            printf("pA\n"); printMat<T,6,12>(&s_temp[78 * 12], 6);
        }
        __syncthreads();
        //
        // Second Forward Pass
        //
        // s_a, qdd where parent is base
        //     joints are: lf_haa_joint, lh_haa_joint, rf_haa_joint, rh_haa_joint
        //     links are: lf_hipassembly, lh_hipassembly, rf_hipassembly, rh_hipassembly
        // a[k] = X[k]*gravity_vec + c[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6;
            // non-branching pointer selector
            int jid = (ind < 6) * 0 + (ind < 12 && ind >= 6) * 3 + (ind < 18 && ind >= 12) * 6 + (ind >= 18) * 9;
            int jid6 = 6*jid;
            T gravity_vec[] = {0,0,0,0,0,gravity};
            s_va[6*12+jid6+row] = dot_prod<T,6,6,1>(&s_XImats[36 * jid + row], &gravity_vec[0]) + s_temp[72*12+jid6+row];
        }
        __syncthreads();
        // qdd[k] = (u[k] - U[k].T*a[k])/d[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 4; ind += blockDim.x*blockDim.y){
            int comp_mod = ind % 4;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 0 + (comp_mod == 1) * 3 + (comp_mod == 2) * 6 + (comp_mod == 3) * 9;
            int jid6 = 6 * jid;
            T tempval = s_temp[97 * 12+jid] - dot_prod<T,6,1,1>(&s_temp[84*12+jid6], &s_va[6*12+jid6]);
            s_qdd[jid] = tempval / s_temp[96*12+jid];
        }
        __syncthreads();
        // a[k] += qdd[k]*S[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int comp_mod = comp % 4;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 0 + (comp_mod == 1) * 3 + (comp_mod == 2) * 6 + (comp_mod == 3) * 9;
            int jid6 = 6 * jid;
            T qdd_val = (row == 2) * (s_qdd[jid]);
            s_va[6*12+jid6+row] += qdd_val;
        }
        __syncthreads();
        // s_a, s_qdd where bfs_level is 1
        //     joints are: lf_hfe_joint, lh_hfe_joint, rf_hfe_joint, rh_hfe_joint
        //     links are: lf_upperleg, lh_upperleg, rf_upperleg, rh_upperleg
        // a[k] = X[k]*a[parent] + c[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int comp_mod = comp % 4;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 1 + (comp_mod == 1) * 4 + (comp_mod == 2) * 7 + (comp_mod == 3) * 10;
            int jid6 = 6 * jid;
            s_va[6*12+jid6+row] = dot_prod<T,6,6,1>(&s_XImats[36 * jid + row], &s_va[6*12+(6 * s_topology_helpers[jid])]) + s_temp[72*12+jid6+row];
        }
        __syncthreads();
        // qdd[k] = (u[k] - U[k].T*a[k])/d[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 4; ind += blockDim.x*blockDim.y){
            int comp_mod = ind % 4;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 1 + (comp_mod == 1) * 4 + (comp_mod == 2) * 7 + (comp_mod == 3) * 10;
            int jid6 = 6 * jid;
            T tempval = s_temp[97 * 12+jid] - dot_prod<T,6,1,1>(&s_temp[84*12+jid6], &s_va[6*12+jid6]);
            s_qdd[jid] = tempval / s_temp[96*12+jid];
        }
        __syncthreads();
        // a[k] += qdd[k]*S[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int comp_mod = comp % 4;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 1 + (comp_mod == 1) * 4 + (comp_mod == 2) * 7 + (comp_mod == 3) * 10;
            int jid6 = 6 * jid;
            T qdd_val = (row == 2) * (s_qdd[jid]);
            s_va[6*12+jid6+row] += qdd_val;
        }
        __syncthreads();
        // s_a, s_qdd where bfs_level is 2
        //     joints are: lf_kfe_joint, lh_kfe_joint, rf_kfe_joint, rh_kfe_joint
        //     links are: lf_lowerleg, lh_lowerleg, rf_lowerleg, rh_lowerleg
        // a[k] = X[k]*a[parent] + c[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int comp_mod = comp % 4;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 2 + (comp_mod == 1) * 5 + (comp_mod == 2) * 8 + (comp_mod == 3) * 11;
            int jid6 = 6 * jid;
            s_va[6*12+jid6+row] = dot_prod<T,6,6,1>(&s_XImats[36 * jid + row], &s_va[6*12+(6 * s_topology_helpers[jid])]) + s_temp[72*12+jid6+row];
        }
        __syncthreads();
        // qdd[k] = (u[k] - U[k].T*a[k])/d[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 4; ind += blockDim.x*blockDim.y){
            int comp_mod = ind % 4;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 2 + (comp_mod == 1) * 5 + (comp_mod == 2) * 8 + (comp_mod == 3) * 11;
            int jid6 = 6 * jid;
            T tempval = s_temp[97 * 12+jid] - dot_prod<T,6,1,1>(&s_temp[84*12+jid6], &s_va[6*12+jid6]);
            s_qdd[jid] = tempval / s_temp[96*12+jid];
        }
        __syncthreads();
        // a[k] += qdd[k]*S[k]
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 24; ind += blockDim.x*blockDim.y){
            int row = ind % 6; int comp = ind / 6; int comp_mod = comp % 4;
            // non-branching pointer selector
            int jid = (comp_mod == 0) * 2 + (comp_mod == 1) * 5 + (comp_mod == 2) * 8 + (comp_mod == 3) * 11;
            int jid6 = 6 * jid;
            T qdd_val = (row == 2) * (s_qdd[jid]);
            s_va[6*12+jid6+row] += qdd_val;
        }
        __syncthreads();
        __syncthreads();
        if(threadIdx.x == 0 && threadIdx.y == 0){
            printf("a\n"); printMat<T,6,12>(&s_va[6 * 12], 6);
            printf("qdd\n"); printMat<T,1,12>(s_qdd,1);
        }
        __syncthreads();
    }

    /**
     * Compute the ABA (Articulated Body Algorithm)
     *
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param s_tau is the vector of joint torques
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void aba_device(const T *s_q, const T *s_qd, const T *s_tau, const robotModel<T> *d_robotModel, const T gravity) {
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        extern __shared__ T s_va[2*6*12];
        extern __shared__ T s_qdd[12];
        load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
        aba_inner<T>(s_qdd, s_va, s_q, s_qd, s_tau, s_XImats, s_topology_helpers, s_temp, gravity);
    }

    /**
     * Compute the ABA (Articulated Body Algorithm)
     *
     * @param d_q_qd_tau is the vector of joint positions and velocities
     * @param stride_q_qd is the stride between each q, qd
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param d_tau is the vector of joint torques
     * @param gravity is the gravity constant
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void aba_kernel_single_timing(T *d_qdd, const T *d_q_qd_tau, const int stride_q_qd, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_qdd[12];
        __shared__ T s_q_qd_tau[3*12]; T *s_q = s_q_qd_tau; T *s_qd = &s_q_qd_tau[12]; T *s_tau = &s_q_qd_tau[2 * 12];
        __shared__ T s_va[144];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        // load to shared mem
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            s_q_qd_tau[ind] = d_q_qd_tau[ind];
        }
        __syncthreads();
        // compute with NUM_TIMESTEPS as NUM_REPS for timing
        for (int rep = 0; rep < NUM_TIMESTEPS; rep++){
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            aba_inner<T>(s_qdd, s_va, s_q, s_qd, s_tau, s_XImats, s_topology_helpers, s_temp, gravity);
        }
        // save down to global
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
            d_qdd[ind] = s_qdd[ind];
        }
        __syncthreads();
    }

    /**
     * Compute the ABA (Articulated Body Algorithm)
     *
     * @param d_q_qd_tau is the vector of joint positions and velocities
     * @param stride_q_qd is the stride between each q, qd
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param d_tau is the vector of joint torques
     * @param gravity is the gravity constant
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void aba_kernel(T *d_qdd, const T *d_q_qd_tau, const int stride_q_qd, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_qdd[12];
        __shared__ T s_q_qd_tau[3*12]; T *s_q = s_q_qd_tau; T *s_qd = &s_q_qd_tau[12]; T *s_tau = &s_q_qd_tau[2 * 12];
        __shared__ T s_va[144];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        for(int k = blockIdx.x + blockIdx.y*gridDim.x; k < NUM_TIMESTEPS; k += gridDim.x*gridDim.y){
            // load to shared mem
            const T *d_q_qd_tau_k = &d_q_qd_tau[k*stride_q_qd];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
                s_q_qd_tau[ind] = d_q_qd_tau_k[ind];
            }
            __syncthreads();
            // compute
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            aba_inner<T>(s_qdd, s_va, s_q, s_qd, s_tau, s_XImats, s_topology_helpers, s_temp, gravity);
            __syncthreads();
            // save down to global
            T *d_qdd_k = &d_qdd[k*1];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 12; ind += blockDim.x*blockDim.y){
                d_qdd_k[ind] = s_qdd[ind];
            }
            __syncthreads();
        }
    }

    /**
     * Compute the ABA (Articulated Body Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T>
    __host__
    void aba(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                          const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        int stride_q_qd = 3*NUM_JOINTS;
        // start code with memory transfer
        gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q_qd*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[0]));
        gpuErrchk(cudaDeviceSynchronize());
        // then call the kernel
        aba_kernel<T><<<block_dimms,thread_dimms,ABA_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_qdd,hd_data->d_q_qd_u,stride_q_qd,d_robotModel,gravity,num_timesteps);
        gpuErrchk(cudaDeviceSynchronize());
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_qdd,hd_data->d_qdd,NUM_JOINTS*num_timesteps*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Compute the ABA (Articulated Body Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T>
    __host__
    void aba_single_timing(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                                        const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        int stride_q_qd = 3*NUM_JOINTS;
        // start code with memory transfer
        gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q_qd*sizeof(T),cudaMemcpyHostToDevice,streams[0]));
        gpuErrchk(cudaDeviceSynchronize());
        // then call the kernel
        struct timespec start, end; clock_gettime(CLOCK_MONOTONIC,&start);
        aba_kernel_single_timing<T><<<block_dimms,thread_dimms,ABA_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_qdd,hd_data->d_q_qd_u,stride_q_qd,d_robotModel,gravity,num_timesteps);
        gpuErrchk(cudaDeviceSynchronize());
        clock_gettime(CLOCK_MONOTONIC,&end);
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_qdd,hd_data->d_qdd,NUM_JOINTS*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
        printf("Single Call ABA %fus\n",time_delta_us_timespec(start,end)/static_cast<double>(num_timesteps));
    }

    /**
     * Compute the ABA (Articulated Body Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T>
    __host__
    void aba_compute_only(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                                       const dim3 block_dimms, const dim3 thread_dimms) {
        int stride_q_qd = 3*NUM_JOINTS;
        // then call the kernel
        aba_kernel<T><<<block_dimms,thread_dimms,ABA_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_qdd,hd_data->d_q_qd_u,stride_q_qd,d_robotModel,gravity,num_timesteps);
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Compute the Composite Rigid Body Algorithm
     *
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param s_M is a pointer to the matrix of inertias_XI is the pointer to the transformation and inertia matricies 
     * @param s_XImats is the (shared) memory holding the updated XI matricies for the given s_q
     * @param s_topology_helpers is the (shared) memory destination location for the topology_helpers
     * @param s_temp is a pointer to helper shared memory of size 6*NUM_JOINTS = 1680
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void crba_inner(T *s_M, const T *s_q, const T *s_qd, T *s_XImats, int *s_topology_helpers, T *s_temp, const T gravity) {
        for(int i = threadIdx.x + threadIdx.y*blockDim.x; i < 144; i += blockDim.x*blockDim.y){
            s_M[i] = static_cast<T>(0);
        }
        __syncthreads();
        T *alpha = &s_temp[0];
        T *beta = &s_temp[432];
        T *s_fh = &s_temp[864];
        T *s_jid_list = &s_temp[948];
        //
        // first loop (split into 2 parallel loops in bfs loop)
        // each bfs level runs in parallel
        //
        // pass updates where bfs_level is 2
        //     joints are: lf_kfe_joint, lh_kfe_joint, rf_kfe_joint, rh_kfe_joint
        //     links are: lf_lowerleg, lh_lowerleg, rf_lowerleg, rh_lowerleg
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind <  6) * 2 + (ind <  12 && ind > 6) * 5 + (ind <  18 && ind > 12) * 8 + (ind > 18) * 11;
            s_jid_list[ind] = jid;
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int jid = s_jid_list[4];
            int row = ind % 6; int col = (ind / 6); int jid6 = jid * 6;
            alpha[6*jid6 + row + (6*col)] = dot_prod<T,6,1,1>(&s_XImats[6*jid6 + row*6],&s_XImats[36*(jid+5+7) + (col*6)]);
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int jid = s_jid_list[4];
            int parent_ind = s_topology_helpers[jid];
            int row = ind % 6; int col = (ind / 6) % 6; int jid6 = jid * 6;
            beta[6*jid6 + col + (6*row)] = dot_prod<T,6,6,1>(&alpha[6*jid6 + row],&s_XImats[6*jid6 + (col*6)]);
            s_XImats[36*(parent_ind +5+7) + col + (6*row)] += beta[6*jid6 + col + (6*row)];
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int jid = s_jid_list[10];
            int row = ind % 6; int col = (ind / 6); int jid6 = jid * 6;
            alpha[6*jid6 + row + (6*col)] = dot_prod<T,6,1,1>(&s_XImats[6*jid6 + row*6],&s_XImats[36*(jid+5+7) + (col*6)]);
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int jid = s_jid_list[10];
            int parent_ind = s_topology_helpers[jid];
            int row = ind % 6; int col = (ind / 6) % 6; int jid6 = jid * 6;
            beta[6*jid6 + col + (6*row)] = dot_prod<T,6,6,1>(&alpha[6*jid6 + row],&s_XImats[6*jid6 + (col*6)]);
            s_XImats[36*(parent_ind +5+7) + col + (6*row)] += beta[6*jid6 + col + (6*row)];
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int jid = s_jid_list[16];
            int row = ind % 6; int col = (ind / 6); int jid6 = jid * 6;
            alpha[6*jid6 + row + (6*col)] = dot_prod<T,6,1,1>(&s_XImats[6*jid6 + row*6],&s_XImats[36*(jid+5+7) + (col*6)]);
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int jid = s_jid_list[16];
            int parent_ind = s_topology_helpers[jid];
            int row = ind % 6; int col = (ind / 6) % 6; int jid6 = jid * 6;
            beta[6*jid6 + col + (6*row)] = dot_prod<T,6,6,1>(&alpha[6*jid6 + row],&s_XImats[6*jid6 + (col*6)]);
            s_XImats[36*(parent_ind +5+7) + col + (6*row)] += beta[6*jid6 + col + (6*row)];
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int jid = s_jid_list[22];
            int row = ind % 6; int col = (ind / 6); int jid6 = jid * 6;
            alpha[6*jid6 + row + (6*col)] = dot_prod<T,6,1,1>(&s_XImats[6*jid6 + row*6],&s_XImats[36*(jid+5+7) + (col*6)]);
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int jid = s_jid_list[22];
            int parent_ind = s_topology_helpers[jid];
            int row = ind % 6; int col = (ind / 6) % 6; int jid6 = jid * 6;
            beta[6*jid6 + col + (6*row)] = dot_prod<T,6,6,1>(&alpha[6*jid6 + row],&s_XImats[6*jid6 + (col*6)]);
            s_XImats[36*(parent_ind +5+7) + col + (6*row)] += beta[6*jid6 + col + (6*row)];
        }
        __syncthreads();
        // pass updates where bfs_level is 1
        //     joints are: lf_hfe_joint, lh_hfe_joint, rf_hfe_joint, rh_hfe_joint
        //     links are: lf_upperleg, lh_upperleg, rf_upperleg, rh_upperleg
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            // non-branching pointer selector
            int jid = (ind <  6) * 1 + (ind <  12 && ind > 6) * 4 + (ind <  18 && ind > 12) * 7 + (ind > 18) * 10;
            s_jid_list[ind] = jid;
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int jid = s_jid_list[4];
            int row = ind % 6; int col = (ind / 6); int jid6 = jid * 6;
            alpha[6*jid6 + row + (6*col)] = dot_prod<T,6,1,1>(&s_XImats[6*jid6 + row*6],&s_XImats[36*(jid+5+7) + (col*6)]);
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int jid = s_jid_list[4];
            int parent_ind = s_topology_helpers[jid];
            int row = ind % 6; int col = (ind / 6) % 6; int jid6 = jid * 6;
            beta[6*jid6 + col + (6*row)] = dot_prod<T,6,6,1>(&alpha[6*jid6 + row],&s_XImats[6*jid6 + (col*6)]);
            s_XImats[36*(parent_ind +5+7) + col + (6*row)] += beta[6*jid6 + col + (6*row)];
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int jid = s_jid_list[10];
            int row = ind % 6; int col = (ind / 6); int jid6 = jid * 6;
            alpha[6*jid6 + row + (6*col)] = dot_prod<T,6,1,1>(&s_XImats[6*jid6 + row*6],&s_XImats[36*(jid+5+7) + (col*6)]);
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int jid = s_jid_list[10];
            int parent_ind = s_topology_helpers[jid];
            int row = ind % 6; int col = (ind / 6) % 6; int jid6 = jid * 6;
            beta[6*jid6 + col + (6*row)] = dot_prod<T,6,6,1>(&alpha[6*jid6 + row],&s_XImats[6*jid6 + (col*6)]);
            s_XImats[36*(parent_ind +5+7) + col + (6*row)] += beta[6*jid6 + col + (6*row)];
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int jid = s_jid_list[16];
            int row = ind % 6; int col = (ind / 6); int jid6 = jid * 6;
            alpha[6*jid6 + row + (6*col)] = dot_prod<T,6,1,1>(&s_XImats[6*jid6 + row*6],&s_XImats[36*(jid+5+7) + (col*6)]);
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int jid = s_jid_list[16];
            int parent_ind = s_topology_helpers[jid];
            int row = ind % 6; int col = (ind / 6) % 6; int jid6 = jid * 6;
            beta[6*jid6 + col + (6*row)] = dot_prod<T,6,6,1>(&alpha[6*jid6 + row],&s_XImats[6*jid6 + (col*6)]);
            s_XImats[36*(parent_ind +5+7) + col + (6*row)] += beta[6*jid6 + col + (6*row)];
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int jid = s_jid_list[22];
            int row = ind % 6; int col = (ind / 6); int jid6 = jid * 6;
            alpha[6*jid6 + row + (6*col)] = dot_prod<T,6,1,1>(&s_XImats[6*jid6 + row*6],&s_XImats[36*(jid+5+7) + (col*6)]);
        }
        __syncthreads();
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            int jid = s_jid_list[22];
            int parent_ind = s_topology_helpers[jid];
            int row = ind % 6; int col = (ind / 6) % 6; int jid6 = jid * 6;
            beta[6*jid6 + col + (6*row)] = dot_prod<T,6,6,1>(&alpha[6*jid6 + row],&s_XImats[6*jid6 + (col*6)]);
            s_XImats[36*(parent_ind +5+7) + col + (6*row)] += beta[6*jid6 + col + (6*row)];
        }
        __syncthreads();
        //
        // Calculation of M[ind, ind] 
        //
        for(int jid = threadIdx.x + threadIdx.y*blockDim.x; jid < 12; jid += blockDim.x*blockDim.y){
            s_M[jid+jid*12] = s_XImats[432 + 36*jid + 6*2 + 2];
        }
        __syncthreads();
        //
        // Calculation of M[ind, parent]
        //
        for(int i = threadIdx.x + threadIdx.y*blockDim.x; i < 72; i += blockDim.x*blockDim.y){
            int jid = i / 6; int ind = i % 6;
            s_fh[i] = s_XImats[432 + 36*jid + 6*2 + ind];
        }
        for(int jid = threadIdx.x + threadIdx.y*blockDim.x; jid < 12; jid += blockDim.x*blockDim.y){
            int jid_parents[] = {-1, -1};
            int num_parents = 0;
            switch (jid) {
                case 0:
                    num_parents += 0;
                    break;
                case 1:
                    jid_parents[0] = 0;
                    num_parents += 1;
                    break;
                case 2:
                    jid_parents[0] = 1;
                    jid_parents[1] = 0;
                    num_parents += 2;
                    break;
                case 3:
                    num_parents += 0;
                    break;
                case 4:
                    jid_parents[0] = 3;
                    num_parents += 1;
                    break;
                case 5:
                    jid_parents[0] = 4;
                    jid_parents[1] = 3;
                    num_parents += 2;
                    break;
                case 6:
                    num_parents += 0;
                    break;
                case 7:
                    jid_parents[0] = 6;
                    num_parents += 1;
                    break;
                case 8:
                    jid_parents[0] = 7;
                    jid_parents[1] = 6;
                    num_parents += 2;
                    break;
                case 9:
                    num_parents += 0;
                    break;
                case 10:
                    jid_parents[0] = 9;
                    num_parents += 1;
                    break;
                case 11:
                    jid_parents[0] = 10;
                    jid_parents[1] = 9;
                    num_parents += 2;
                    break;
            }
            T s_alpha[6];
            for (int i = 0; i < num_parents; i++) {
                int X_ind = i==0 ? jid : jid_parents[i-1];
                for (int k = 0; k < 6; k++) s_alpha[k] = s_fh[jid*6+k];
                for (int k = 0; k < 6; k++) s_fh[jid*6 + k] = dot_prod<T,6,1,1>(&s_XImats[36*X_ind+k*6], &s_alpha[0]);
                int parent_ind = jid_parents[i];
                s_M[jid*12 + parent_ind] = s_fh[jid*6 + 2];
                s_M[parent_ind*12 + jid] = s_M[jid*12 + parent_ind];
            }
        }
        __syncthreads();
    }

    /**
     * Compute the CRBA (Composite Rigid Body Algorithm)
     *
     * @param s_M is a pointer to the matrix of inertia
     * @param s_q is the vector of joint positions
     * @param s_qd is the vector of joint velocities
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     */
    template <typename T>
    __device__
    void crba_device(T *s_M, const T *s_q, const T *s_qd,const robotModel<T> *d_robotModel, const T gravity) {
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
        crba_inner<T>(s_M, s_q, s_qd, s_XImats, s_topology_helpers, s_temp, gravity);
    }

    /**
     * Compute the CRBA (Composite Rigid Body Algorithm)
     *
     * @param d_M is the pointer to the matrix of inertia
     * @param d_q_qd is the vector of joint positions and velocities
     * @param stride_q_qd is the stride between each q, qd
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void crba_kernel_single_timing(T *d_M, const T *d_q_qd, const int stride_q_qd, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_M[144];
        __shared__ T s_q_qd[3*12]; T *s_q = s_q_qd; T *s_qd = &s_q_qd[12];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        // load to shared mem
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
            s_q_qd[ind] = d_q_qd[ind];
        }
        __syncthreads();
        // compute with NUM_TIMESTEPS as NUM_REPS for timing
        for (int rep = 0; rep < NUM_TIMESTEPS; rep++){
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            crba_inner<T>(s_M, s_q, s_qd, s_XImats, s_topology_helpers, s_temp, gravity);
        }
        // save down to global
        for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
            d_M[ind] = s_M[ind];
        }
        __syncthreads();
    }

    /**
     * Compute the CRBA (Composite Rigid Body Algorithm)
     *
     * @param d_M is the pointer to the matrix of inertia
     * @param d_q_qd is the vector of joint positions and velocities
     * @param stride_q_qd is the stride between each q, qd
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     */
    template <typename T>
    __global__
    void crba_kernel(T *d_M, const T *d_q_qd, const int stride_q_qd, const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {
        __shared__ T s_M[144];
        __shared__ T s_q_qd[3*12]; T *s_q = s_q_qd; T *s_qd = &s_q_qd[12];
        __shared__ int s_topology_helpers[61];
        extern __shared__ T s_XITemp[]; T *s_XImats = s_XITemp; T *s_temp = &s_XITemp[864];
        for(int k = blockIdx.x + blockIdx.y*gridDim.x; k < NUM_TIMESTEPS; k += gridDim.x*gridDim.y){
            // load to shared mem
            const T *d_q_qd_k = &d_q_qd[k*stride_q_qd];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 36; ind += blockDim.x*blockDim.y){
                s_q_qd[ind] = d_q_qd_k[ind];
            }
            __syncthreads();
            // compute
            load_update_XImats_helpers<T>(s_XImats, s_q, s_topology_helpers, d_robotModel, s_temp);
            crba_inner<T>(s_M, s_q, s_qd, s_XImats, s_topology_helpers, s_temp, gravity);
            __syncthreads();
            // save down to global
            T *d_M_k = &d_M[k*1];
            for(int ind = threadIdx.x + threadIdx.y*blockDim.x; ind < 144; ind += blockDim.x*blockDim.y){
                d_M_k[ind] = s_M[ind];
            }
            __syncthreads();
        }
    }

    /**
     * Compute the CRBA (Composite Rigid Body Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_COMPRESSED_MEM = false>
    __host__
    void crba(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                          const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        // start code with memory transfer
        int stride_q_qd;
        if (USE_COMPRESSED_MEM) {stride_q_qd = 2*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd,hd_data->h_q_qd,stride_q_qd*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        else {stride_q_qd = 3*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q_qd*num_timesteps*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        // then call the kernel
        if (USE_COMPRESSED_MEM) {crba_kernel<T><<<block_dimms,thread_dimms,CRBA_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_M,hd_data->d_q_qd,stride_q_qd,d_robotModel,gravity,num_timesteps);}
        else                    {crba_kernel<T><<<block_dimms,thread_dimms,CRBA_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_M,hd_data->d_q_qd_u,stride_q_qd,d_robotModel,gravity,num_timesteps);}
        gpuErrchk(cudaDeviceSynchronize());
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_M,hd_data->d_M,NUM_JOINTS*NUM_JOINTS*num_timesteps*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Compute the CRBA (Composite Rigid Body Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_COMPRESSED_MEM = false>
    __host__
    void crba_single_timing(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                                        const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {
        // start code with memory transfer
        int stride_q_qd;
        if (USE_COMPRESSED_MEM) {stride_q_qd = 2*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd,hd_data->h_q_qd,stride_q_qd*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        else {stride_q_qd = 3*NUM_JOINTS; gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q_qd*sizeof(T),cudaMemcpyHostToDevice,streams[0]));}
        // then call the kernel
        struct timespec start, end; clock_gettime(CLOCK_MONOTONIC,&start);
        if (USE_COMPRESSED_MEM) {crba_kernel_single_timing<T><<<block_dimms,thread_dimms,CRBA_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_M,hd_data->d_q_qd,stride_q_qd,d_robotModel,gravity,num_timesteps);}
        else                    {crba_kernel_single_timing<T><<<block_dimms,thread_dimms,CRBA_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_M,hd_data->d_q_qd_u,stride_q_qd,d_robotModel,gravity,num_timesteps);}
        gpuErrchk(cudaDeviceSynchronize());
        clock_gettime(CLOCK_MONOTONIC,&end);
        // finally transfer the result back
        gpuErrchk(cudaMemcpy(hd_data->h_M,hd_data->d_M,NUM_JOINTS*NUM_JOINTS*sizeof(T),cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize());
        printf("Single Call ID %fus\n",time_delta_us_timespec(start,end)/static_cast<double>(num_timesteps));
    }

    /**
     * Compute the CRBA (Composite Rigid Body Algorithm)
     *
     * @param hd_data is the packaged input and output pointers
     * @param d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)
     * @param gravity is the gravity constant,
     * @param num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)
     * @param streams are pointers to CUDA streams for async memory transfers (if needed)
     */
    template <typename T, bool USE_COMPRESSED_MEM = false>
    __host__
    void crba_compute_only(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,
                                       const dim3 block_dimms, const dim3 thread_dimms) {
        int stride_q_qd = USE_COMPRESSED_MEM ? 2*NUM_JOINTS: 3*NUM_JOINTS;
        // then call the kernel
        if (USE_COMPRESSED_MEM) {crba_kernel<T><<<block_dimms,thread_dimms,CRBA_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_M,hd_data->d_q_qd,stride_q_qd,d_robotModel,gravity,num_timesteps);}
        else                    {crba_kernel<T><<<block_dimms,thread_dimms,CRBA_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_M,hd_data->d_q_qd_u,stride_q_qd,d_robotModel,gravity,num_timesteps);}
        gpuErrchk(cudaDeviceSynchronize());
    }

    /**
     * Sets shared mem needed for gradient kernels and initializes streams for host functions
     *
     * @return A pointer to the array of streams
     */
    template <typename T>
    __host__
    cudaStream_t *init_grid(){
        // set the max temp memory for the gradient kernels to account for large robots
        auto id_kern1 = static_cast<void (*)(T *, const T *, const int, const T *, const robotModel<T> *, const T, const int)>(&inverse_dynamics_gradient_kernel<T>);
        auto id_kern2 = static_cast<void (*)(T *, const T *, const int, const robotModel<T> *, const T, const int)>(&inverse_dynamics_gradient_kernel<T>);
        auto id_kern_timing1 = static_cast<void (*)(T *, const T *, const int, const T *, const robotModel<T> *, const T, const int)>(&inverse_dynamics_gradient_kernel_single_timing<T>);
        auto id_kern_timing2 = static_cast<void (*)(T *, const T *, const int, const robotModel<T> *, const T, const int)>(&inverse_dynamics_gradient_kernel_single_timing<T>);
        auto fd_kern1 = static_cast<void (*)(T *, const T *, const int, const T *, const T *, const robotModel<T> *, const T, const int)>(&forward_dynamics_gradient_kernel<T>);
        auto fd_kern2 = static_cast<void (*)(T *, const T *, const int, const robotModel<T> *, const T, const int)>(&forward_dynamics_gradient_kernel<T>);
        auto fd_kern_timing1 = static_cast<void (*)(T *, const T *, const int, const T *, const T *, const robotModel<T> *, const T, const int)>(&forward_dynamics_gradient_kernel_single_timing<T>);
        auto fd_kern_timing2 = static_cast<void (*)(T *, const T *, const int, const robotModel<T> *, const T, const int)>(&forward_dynamics_gradient_kernel_single_timing<T>);
        cudaFuncSetAttribute(id_kern1,cudaFuncAttributeMaxDynamicSharedMemorySize, ID_DU_MAX_SHARED_MEM_COUNT*sizeof(T));
        cudaFuncSetAttribute(id_kern2,cudaFuncAttributeMaxDynamicSharedMemorySize, ID_DU_MAX_SHARED_MEM_COUNT*sizeof(T));
        cudaFuncSetAttribute(id_kern_timing1,cudaFuncAttributeMaxDynamicSharedMemorySize, ID_DU_MAX_SHARED_MEM_COUNT*sizeof(T));
        cudaFuncSetAttribute(id_kern_timing2,cudaFuncAttributeMaxDynamicSharedMemorySize, ID_DU_MAX_SHARED_MEM_COUNT*sizeof(T));
        cudaFuncSetAttribute(fd_kern1,cudaFuncAttributeMaxDynamicSharedMemorySize, FD_DU_MAX_SHARED_MEM_COUNT*sizeof(T));
        cudaFuncSetAttribute(fd_kern2,cudaFuncAttributeMaxDynamicSharedMemorySize, FD_DU_MAX_SHARED_MEM_COUNT*sizeof(T));
        cudaFuncSetAttribute(fd_kern_timing1,cudaFuncAttributeMaxDynamicSharedMemorySize, FD_DU_MAX_SHARED_MEM_COUNT*sizeof(T));
        cudaFuncSetAttribute(fd_kern_timing2,cudaFuncAttributeMaxDynamicSharedMemorySize, FD_DU_MAX_SHARED_MEM_COUNT*sizeof(T));
        gpuErrchk(cudaDeviceSynchronize());
        // allocate streams
        cudaStream_t *streams = (cudaStream_t *)malloc(3*sizeof(cudaStream_t));
        int priority, minPriority, maxPriority;
        gpuErrchk(cudaDeviceGetStreamPriorityRange(&minPriority, &maxPriority));
        for(int i=0; i<3; i++){
            int adjusted_max = maxPriority - i; priority = adjusted_max > minPriority ? adjusted_max : minPriority;
            gpuErrchk(cudaStreamCreateWithPriority(&(streams[i]),cudaStreamNonBlocking,priority));
        }
        return streams;
    }

    /**
     * Frees the memory used by grid
     *
     * @param streams allocated by init_grid
     * @param robotModel allocated by init_robotModel
     * @param data allocated by init_gridData
     */
    template <typename T>
    __host__
    void close_grid(cudaStream_t *streams, robotModel<T> *d_robotModel, gridData<T> *hd_data){
        gpuErrchk(cudaFree(d_robotModel));
        gpuErrchk(cudaFree(hd_data->d_q_qd_u)); gpuErrchk(cudaFree(hd_data->d_q_qd)); gpuErrchk(cudaFree(hd_data->d_q));
        gpuErrchk(cudaFree(hd_data->d_c)); gpuErrchk(cudaFree(hd_data->d_Minv)); gpuErrchk(cudaFree(hd_data->d_qdd));
        gpuErrchk(cudaFree(hd_data->d_dc_du)); gpuErrchk(cudaFree(hd_data->d_df_du));
        gpuErrchk(cudaFree(hd_data->d_eePos)); gpuErrchk(cudaFree(hd_data->d_deePos));
        free(hd_data->h_q_qd_u); free(hd_data->h_q_qd); free(hd_data->h_q);
        free(hd_data->h_c); free(hd_data->h_Minv); free(hd_data->h_qdd);
        free(hd_data->h_dc_du); free(hd_data->h_df_du);
        free(hd_data->h_eePos); free(hd_data->h_deePos);
        for(int i=0; i<3; i++){gpuErrchk(cudaStreamDestroy(streams[i]));} free(streams);
    }

}

