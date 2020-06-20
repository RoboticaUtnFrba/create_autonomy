#!/bin/bash

# Set RTIMULib paths
RTIMULib_PATH="/home/${USER}/RTIMULib"
RTIMULib_BUILD="${RTIMULib_PATH}/Linux/build"
RTIMULib_INSTALL="${RTIMULib_PATH}/Linux/install"

# Clone repo in RTIMULib_PATH
mkdir -p ${RTIMULib_PATH}
git clone https://github.com/RoboticaUtnFrba/RTIMULib.git ${RTIMULib_PATH}
cd ${RTIMULib_PATH}
# Build and install RTIMULib
mkdir -p ${RTIMULib_BUILD}
cd ${RTIMULib_BUILD}
# Disable OpenGL in Travis
# https://docs.travis-ci.com/user/environment-variables/#default-environment-variables
BUILD_GL=""
[ CI == "true" ] && BUILD_GL="-DBUILD_GL=OFF"
cmake ${BUILD_GL} -DCMAKE_INSTALL_PREFIX=${RTIMULib_INSTALL} ..
make -j4
make install
ldconfig
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${RTIMULib_INSTALL}"
