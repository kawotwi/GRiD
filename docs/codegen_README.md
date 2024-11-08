# GRiDCodeGenerator

A optimized CUDA C++ code generation engine for rigid body dynamics algorithms and their analytical gradients.

This package is written in Python and outputs CUDA C++ code. Helper functions have been written to speed up the algorithm implementation process and are detailed below. If your favorite rigid body dynamics algorithm is not yet implemented please either submit a PR to this repo with the code generation implementation or simply submit a PR to our [rbdReference](https://github.com/robot-acceleration/rbdReference) package with the Python implementation and we'll then try to get a GPU implementation designed as soon as possible.

## Usage:
This package relies on an already parsed ```robot``` object from our [URDFParser](https://github.com/robot-acceleration/URDFParser) package.
```python
GRiDCodeGenerator = GRiDCodeGenerator(robot, DEBUG_MODE = False)
GRiDCodeGenerator.gen_all_code()
```
A file named ```grid.cuh``` will be written to the current working directory and can then be included into your project. See the wrapper [GRiD](https://github.com/robot-acceleration/GRiD) package for more instructions on how to use and test this code.

## Instalation Instructions:
The only external dependencies needed to run this package are ```numpy,sympy``` which can be automatically installed by running:
```shell
pip3 install -r requirements.txt
```
This package also depends on our [URDFParser](https://github.com/robot-acceleration/URDFParser) package.

Running the CUDA C++ code output by the GRiDCodegenerator also requires CUDA to be installed on your system. Please see the [README.md in the GRID](https://github.com/robot-acceleration/GRiD) wrapper package for instalation notes for CUDA.

## C++ API
To enable GRiD to be used by both expert and novice GPU programmers we provide the following API interface for each rigid body dynamics algorithm:
+ ```ALGORITHM_inner```: a device function that computes the core computation. These functions assume that inputs are already loaded into GPU shared memory, require a pointer to additional scratch shared memory, and store the result back in shared memory.
+ ```ALGORITHM_device```: a device function that handles the shared memory allocation for the ```\_inner``` function. These functions assume that inputs are already loaded into, and return results to, GPU shared memory.
+ ```ALGORITHM_kernel```: a kernel that handles the shared memory allocation for the ```\_inner``` function. These functions assume that inputs are loaded into, and return results to, the global GPU memory.
+ ```ALGORITHM```: a host function that wraps the ```_kernel``` and handles the transfer of inputs to the GPU and the results back to the CPU.

## Code Generation API

For each algorithm (written as a ```_algorithm.py``` file in the ```algorithms``` folder) the following functions are generally written:
+ ```gen_algorithm_temp_mem_size```: returns a Python number noting the shared memory array size needed for all temporary variables
+ ```gen_algorithm_function_call```: generates a function call for that algorithm and is intended to be used inside other algorithms
+ ```gen_algorithm_inner```: generates a device function which computes the core computation. These functions assume that inputs are already loaded into GPU shared memory, require a pointer to additional scratch shared memory, and store the result back in shared memory.
+ ```gen_algorithm_device```: generates a device function which handles the shared memory allocation for the ```_inner``` function. These functions still assume that inputs are already loaded into, and return results to, GPU shared memory.
+ ```gen_algorithm_kernel```: generates a a kernel that handles the shared memory allocation for the ```_inner```  function. These functions assume that input are loaded into, and return results to, the global GPU memory.
+ ```gen_algorithm_host```: generates a host function that wraps the ```_kernel``` and handles the transfer of inputs to the GPU and the results back to the CPU.
+ ```gen_algorithm```: runs all of the above mention function generators

**Codegeneration helper functions are as follows:**

Note: most functions assume inputs are strings and are located in the ```helpers``` folder in the ```_code_generation_helpers.py``` file (and a few are also found in the ```_topology_helpers.py``` and ```_spatial_algebra_helpers.py``` files)

+ Add a string or list of strings of code with ```gen_add_code_line(new_code_line, add_indent_after = False)``` and ```gen_add_code_lines(new_code_lines, add_indent_after = False)```
+ Reduce the global indentation level and insert a close brace with ```gen_add_end_control_flow()``` and ```gen_add_end_function()```
+ Add a Doxygen formatted function description with ```gen_add_func_doc(description string, notes = [], params = [], return_val = None)```
+ Ensure that a block of code is only run by one thread per block ```gen_add_serial_ops(use_thread_group = False)``` and make sure to end this control flow later
+ Run a block of code with N parallel threads or blocks ```gen_add_parallel_loop(var_name, max_val, use_thread_group = False, block_level = False)``` and make sure to end this control flow later
+ Add a thread synchronization point ```gen_add_sync(self, use_thread_group = False)```
+ Test if a variable is or is not in a list ```gen_var_in_list(var_name, option_list)``` ```and gen_var_not_in_list(var_name, option_list)```
+ Generate an if, elif, else statement that can be either non-branching (if only one output variable or the flag is set) or branching selectors for multiple variables at the same time. Variable types, names, and resulting values are defined in the ```select_tuples = [(type, name, values)]``` and are selected when the ```loop_counter``` varaible satisfies the condition set by the ```comparator``` according to each ```count``` in an if, elif, else paradigm. This is done with ```gen_add_multi_threaded_select(loop_counter, comparator, counts, select_tuples, USE_NON_BRANCH_ALWAYS = False)```
+ Load values from global to shared memory (assuming varaibles are called ```s_name``` and ```d_name```) with ```gen_kernel_load_inputs(name, stride, amount, use_thread_group = False, name2 = None, stride2 = 1, amount2 = 1, name3 = None, stride3 = 1, amount3 = 1)```
+ Save values from shared to global memory (assuming varaibles are called ```s_name``` and ```d_name``` or overridden by ```load_from_name```) with ```gen_kernel_save_result(store_to_name, stride, amount, use_thread_group = False, load_from_name = None)```
+ Generate the optimized C++ code string to compute the matrix cross product operation on a set of links/joints ```gen_mx_func_call_for_cpp(inds = None, PEQ_FLAG = False, SCALE_FLAG = False, updated_var_names = None)```
+ **Get** variables that hold C++ code strings that represent the optimized topology pointers for a given set of joint/link indicies for a given robot mode (e.g., either indexing into shared memory to get parent indicies or optimized to simply return the current index minus one for a serial chain roboto) with ```parent_ind, S_ind, dva_col_offset_for_jid, df_col_offset_for_jid, dva_col_offset_for_parent, df_col_offset_for_parent, dva_col_offset_for_jid_p1, df_col_that_is_jid = gen_topology_helpers_pointers_for_cpp(inds = None, updated_var_names = None, NO_GRAD_FLAG = False)``` and similar Python numerical values can be returned through ```dva_cols_per_partial, dva_cols_per_jid, running_sum_dva_cols_per_jid, df_cols_per_partial,  df_cols_per_jid,  running_sum_df_cols_per_jid,  df_col_that_is_jid = gen_topology_sparsity_helpers_python()```

## Additonal Features:
This package also includes test functions which allow for code optimizations and refactorizations to be tested against reference implementations. This code is located in the ```_test.py``` file.
+ ```(c, v, a, f) = GRiDCodeGenerator.test_rnea(q, qd, qdd = None, GRAVITY = -9.81)```
+ ```Minv = GRiDCodeGenerator.test_minv(q, densify_Minv = False)```
+ ```dc_du = GRiDCodeGenerator.test_rnea_grad(q, qd, qdd = None, GRAVITY = -9.81)``` where ```dc_du = np.hstack((dc_dq,dc_dqd))```
+ ```df_du = GRiDCodeGenerator.test_fd_grad(q, qd, u, GRAVITY = -9.81)``` where ```df_du = np.hstack((df_dq,df_dqd))```

We also include functions that break these algorithms down into there different passes to enable easier testing.
