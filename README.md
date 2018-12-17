UAVMVS - UAV capture planning for MVS reconstructions
--------------------------------------------------------------------------------

The algorithm was published in Dec. 2018 at *SIGGRAPH Asia*.
Please refer to our project website
(https://vccimaging.org/Publications/Smith2018UAVPathPlanning/)
for the paper and further information.

Dependencies
--------------------------------------------------------------------------------

The code and the build system have the following prerequisites:

- git
- make
- gcc (>= 5.4.0)
- cuda (>= 8.0.0)
- libglfw, libGL, libgomp, libpng, libjpg, libtiff

Furthermore the code depends on the following projects, some with adaptions so
please clone and build the respective branches:

- Premake
    https://premake.github.io/
    https://github.com/nmoehrle/premake-core/tree/toolset-nvcc
- Multi-View Environment
    https://www.gcc.tu-darmstadt.de/home/proj/mve
    https://github.com/nmoehrle/mve/tree/master
- MVS-Texturing
    https://www.gcc.tu-darmstadt.de/home/proj/texrecon
    https://github.com/nmoehrle/mvs-texturing/tree/develop
- Eigen
    http://eigen.tuxfamily.org

Execution
--------------------------------------------------------------------------------

Starting any application without parameters will print a description and an
explanation of the parameters.

Compilation
--------------------------------------------------------------------------------

1.  `git clone https://github.com/nmoehrle/uavmvs.git`
2.  `cd uavmvs`
3.  `premake5 gmake`
4.  `cd build`
5.  `make` (or `make -j` for parallel compilation)

License and Citing
--------------------------------------------------------------------------------
Our software is licensed under the BSD 3-Clause license, for more details see
the LICENSE.txt file.

If you use our capture planning code for research purposes, please cite our paper:
```
@inproceedings{Smith2018Aerial,
  title={Aerial Path Planning for Urban Scene Reconstruction:
    a Continuous Optimization Method and Benchmark},
  author={Smith, Neil and Moehrle, Nils and Goesele, Michael and Heidrich, Wolfgang},
  year={2018},
  publisher={ACM}
}
```

Contact
--------------------------------------------------------------------------------
If you have trouble compiling or using this software, if you found a bug or if
you have an important feature request, please use the issue tracker of github:
https://github.com/nmoehrle/uavmvs
