# Tested on fresh install of Ubuntu 18.04 LTS
# If you want parallel compilation, run `MAKEFLAGS="-j$(nproc)" ./install.sh`

sudo apt-get -qy install git build-essential cmake libboost-dev libboost-filesystem-dev libboost-serialization-dev libeigen3-dev libglew-dev libglfw3-dev libpng-dev
git submodule init
git submodule update
pushd deps/bogus-interface
git submodule init
git submodule update
popd
mkdir -p build
pushd build
cmake ..
make
popd
