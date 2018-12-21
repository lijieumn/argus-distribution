# Compilation Instructions

For Ubuntu 18.04, we have provided the script `install.sh` which installs all the dependencies and compiles the project. If compilation was successful, the executable `argus-cloth` will be placed in `build/apps`.

For other platforms, detailed instructions follow.

## Obtain the dependencies

We assume that you have already cloned the `argus-distribution` project and are in the project root directory.

1. Ensure that the following libraries are installed:

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

2. Update the submodules:

	```bash
	git submodule update --init --recursive
	```

	> Note: The two libraries  (`bogus-interface` and `libarcsim`) are put as submodules of this project `Argus-distribution`. The submodule `bogus-interface` serves as an API project, which exposures interfaces for the `so-bogus` library to the users. The `so-bogus` is the actual frictional contact solver and is organized as a submodule of `bogus-interface`.

Now you've downloaded all the source code needed for the cloth simulator.

## Compile

Go through the standard `cmake` process:

```
mkdir build
cd build
cmake ../
make -j
```

If the compilation completes successfully, the executable file `argus-cloth` will be put in `build/apps/`.
