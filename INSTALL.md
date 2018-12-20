# Installation Instructions

## Obtain the project

1. Clone project` Argus-distribution`.

```bash
git clone git@github.com:lijieumn/argus-distribution.git
cd Argus-distribution
```

2. Update the submodules.

```bash
git submodule update --init --recursive
```

> The two libraries  (`bogus-interface` and `libarcsim`) are put as submodules of this project `Argus-distribution`. The submodule `bogus-interface` serves as an API project, which exposures interfaces for the `so-bogus` library to the users. The `so-bogus` is the actual frictional contact solver and is organized as a submodule of `bogus-interface`.

Now you've downloaded all the source code needed for the cloth simulator.

## Compile

1. Dependencies.
You will need the following libraries installed to be able to compile and run ARGUS:

* BLAS
* Boost
* gfortran
* LAPACK
* libpng
* libxmu
* libxi
* Eigen3
* glfw3
* glew

2. Go the the root path of the `Argus-distribution` and go through the standard `cmake` process.

```
mkdir build
cd build
cmake ../
make -j
```

If you do everything correctly, you should be able to complete the compiling successfully. Now the executable file `argus-cloth` is put in `build/apps/`.


