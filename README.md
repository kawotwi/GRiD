# GRiD
<!-- ALL-CONTRIBUTORS-BADGE:START - Do not remove or modify this section -->
[![All Contributors](https://img.shields.io/badge/all_contributors-9-orange.svg?style=flat-square)](#contributors-)
<!-- ALL-CONTRIBUTORS-BADGE:END -->
[![docs](https://github.com/kawotwi/GRiD/actions/workflows/gh-pages.yml/badge.svg?branch=floating-base)](https://github.com/kawotwi/GRiD/actions/workflows/gh-pages.yml)

A GPU-accelerated library for computing rigid body dynamics with analytical gradients.

GRiD wraps our [URDFParser](https://github.com/robot-acceleration/URDFParser), [GRiDCodeGenerator](https://github.com/robot-acceleration/GRiDCodeGenerator), and [RBDReference](https://github.com/robot-acceleration/RBDReference) packages. Using its scripts, users can easily generate and test optimized rigid body dynamics CUDA C++ code for their URDF files.

For additional information and links to our paper on this work, check out our [project website](https://brianplancher.com/publication/GRiD).

**This package contains submodules make sure to run ```git submodule update --init --recursive```** after cloning!

![The GRiD library package ecosystem, showing how a user's URDF file can be transformed into optimized CUDA C++ code which can then be validated against reference outputs and benchmarked for performance.](imgs/GRiD.png)

## Usage:
+ To generate the ```grid.cuh``` header file please run: ```generateGRiD.py PATH_TO_URDF (-D)``` where ```-D``` indicates full debug mode which will include print statements after ever step of ever algorithm
+ To test the python refactored algorithms against our reference implmentations please run ```testGRiDRefactorings.py PATH_TO_URDF (-D)``` where ```-D``` prints extra debug values as compared to just the comparisons
+ To print and compare GRiD to reference values please do the following steps: 
  1) Print the reference values by running ```printReferenceValues.py PATH_TO_URDF (-D)``` where ```-D``` prints the full debug reference values from the refactorings 
  2) Run ```printGrid.py PATH_TO_URDF (-D)``` to compile, run, and print the same values from CUDA C++

## Current Support
GRiD currently fully supports any robot model consisting of revolute, prismatic, and fixed joints that does not have closed kinematic loops.

GRiD currently implements the following rigid body dynamics algorithms:
+ Inverse Dynamics via the Recursive Newton Euler Algorithm (RNEA) from [Featherstone](https://link.springer.com/book/10.1007/978-1-4899-7560-7)
+ The Direct Inverse of Mass Matrix from [Carpentier](https://www.researchgate.net/publication/343098270_Analytical_Inverse_of_the_Joint_Space_Inertia_Matrix)
+ Forward Dynamics by combining the above algorithms as qdd = -M^{-1}(u-RNEA(q,qd,0))
+ Analytical Gradients of Inverse Dynamics from [Carpentier](https://hal.archives-ouvertes.fr/hal-01790971)
+ Analytical Gradient of Forward Dynamics from [Carpentier](https://hal.archives-ouvertes.fr/hal-01790971)

Additional algorithms and features are in development. If you have a particular algorithm or feature in mind please let us know by posting a GitHub issue. We'd also love your collaboration in implementing the Python reference implementation of any algorithm you'd like implemented!

## C++ API
To enable GRiD to be used by both expert and novice GPU programmers we provide the following API interface for each rigid body dynamics algorithm:
+ ```ALGORITHM_inner```: a device function that computes the core computation. These functions assume that inputs are already loaded into GPU shared memory, require a pointer to additional scratch shared memory, and store the result back in shared memory.
+ ```ALGORITHM_device```: a device function that handles the shared memory allocation for the ```\_inner``` function. These functions assume that inputs are already loaded into, and return results to, GPU shared memory.
+ ```ALGORITHM_kernel```: a kernel that handles the shared memory allocation for the ```\_inner``` function. These functions assume that inputs are loaded into, and return results to, the global GPU memory.
+ ```ALGORITHM```: a host function that wraps the ```_kernel``` and handles the transfer of inputs to the GPU and the results back to the CPU.

## Citing GRiD
To cite GRiD in your research, please use the following bibtex for our paper ["GRiD: GPU-Accelerated Rigid Body Dynamics with Analytical Gradients"](https://brianplancher.com/publication/grid/):
```
@inproceedings{plancher2022grid,
  title={GRiD: GPU-Accelerated Rigid Body Dynamics with Analytical Gradients}, 
  author={Brian Plancher and Sabrina M. Neuman and Radhika Ghosal and Scott Kuindersma and Vijay Janapa Reddi},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)}, 
  year={2022}, 
  month={May}
}
```

## Performance
When performing multiple computations of rigid body dynamics algorithms, GRiD provides as much as a 7.6x speedup over a state-of-the-art, multi-threaded CPU implementation, and maintains as much as a 2.6x speedup when accounting for I/O overhead. 

![Latency (including GPU I/O overhead) for N = 16, 32, 64, 128, and 256 computations of the gradient of forward dynamics for both the Pinocchio CPU baseline and the GRiD GPU library for various robot models (IIWA, HyQ, and Atlas). Overlayed is the speedup (or slowdown) of GRiD as compared to Pinocchio both in terms of pure computation and including I/O overhead.](imgs/benchmark_multi_fd_grad.png)

To learn more about GRiD's performance results and to run your own benchmark analysis of GRiD's performance please check out our [GRiDBenchmarks](https://github.com/robot-acceleration/GRiDBenchmarks) repository and our [paper](https://brianplancher.com/publication/GRiD/).

## Instalation Instructions:
### Install Python Dependencies
In order to support the wrapped packages there are 4 required external packages ```beautifulsoup4, lxml, numpy, sympy``` which can be automatically installed by running:
```shell
pip3 install -r requirements.txt
```
### Install CUDA Dependencies
```
sudo apt-get update
sudo apt-get -y install xorg xorg-dev linux-headers-$(uname -r) apt-transport-https
```
### Download and Install CUDA 
Note: for Ubuntu 20.04 see [https://developer.nvidia.com/cuda-downloads](https://developer.nvidia.com/cuda-downloads) for other distros
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
sudo apt-get update
sudo apt-get -y install cuda
```
### Add the following to ```~/.bashrc```
```
export PATH="/usr/local/cuda/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"
export PATH="opt/nvidia/nsight-compute/:$PATH"
```

## Contributors ✨

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tbody>
    <tr>
      <td align="center" valign="top" width="20%"><a href="https://github.com/kawotwi"><img src="https://avatars.githubusercontent.com/kawotwi?s=100" width="100px;" alt="Kwamena A"/><br /><sub><b>Kwamena A</b></sub></a><br /></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/plancherb1"><img src="https://avatars.githubusercontent.com/plancherb1?s=100" width="100px;" alt="Brian Plancher"/><br /><sub><b>Brian Plancher</b></sub></a><br /></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/harvard-edge/cs249r_book/graphs/contributors"><img src="https://www.gravatar.com/avatar/b619b0ff13333ce2a22bb110eda8f7a9?d=identicon&s=100?s=100" width="100px;" alt="Danelle Tuchman"/><br /><sub><b>Danelle Tuchman</b></sub></a><br /></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/Z4KH"><img src="https://avatars.githubusercontent.com/Z4KH?s=100" width="100px;" alt="Zachary Pestrikov"/><br /><sub><b>Zachary Pestrikov</b></sub></a><br /></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/EmreAdabag"><img src="https://avatars.githubusercontent.com/EmreAdabag?s=100" width="100px;" alt="EmreAdabag"/><br /><sub><b>EmreAdabag</b></sub></a><br /></td>
    </tr>
    <tr>
      <td align="center" valign="top" width="20%"><a href="https://github.com/naren-loganathan"><img src="https://avatars.githubusercontent.com/naren-loganathan?s=100" width="100px;" alt="Naren Loganathan"/><br /><sub><b>Naren Loganathan</b></sub></a><br /></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/emilyburnett2003"><img src="https://avatars.githubusercontent.com/emilyburnett2003?s=100" width="100px;" alt="emilyburnett2003"/><br /><sub><b>emilyburnett2003</b></sub></a><br /></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/harvard-edge/cs249r_book/graphs/contributors"><img src="https://www.gravatar.com/avatar/b6a2a74133224e85e9714e282a1576df?d=identicon&s=100?s=100" width="100px;" alt="Kimiya Shahamat"/><br /><sub><b>Kimiya Shahamat</b></sub></a><br /></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/harvard-edge/cs249r_book/graphs/contributors"><img src="https://www.gravatar.com/avatar/b6a2a74133224e85e9714e282a1576df?d=identicon&s=100?s=100" width="100px;" alt="Kimiya Shahamat"/><br /><sub><b>Kimiya Shahamat</b></sub></a><br /></td>
    </tr>
  </tbody>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->
This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!

