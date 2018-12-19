# Installation Instructions

## Obtain the project

1. Clone project` Argus-distribution`.

```bash
git clone git@gitlab.inria.fr:mily/Argus-distribution.git
cd Argus-distribution
```

2. Update the submodules.

```bash
git submodule init
git submodule update
```

3. Update the `So-bogus` module. The submodule under `deps/bogus-interface` serves as an API project, which exposures interfaces for the `so-bogus` library to the users. The `so-bogus` is the actual frictional contact solver and is organized as a submodule of `bogus-interface`.
```bash
cd deps/bogus-interface
git submodule init
git submodule update
```
Now you've downloaded all the source code needed for the cloth simulator.

> Note: as the `so-bogus` library is served on [Bit Bucket](https://bitbucket.org), you might need to have a Bit Bucket account and set the SSH key properly to access it.

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

If you do everything correctly, you shoud be able to complete the compiling successfully. Now the executable file `argus-cloth` is put in `build/apps/`.


