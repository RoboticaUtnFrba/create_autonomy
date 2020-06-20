#!/bin/bash

# Set RTIMULib paths
RTIMULib_PATH="/home/${USER}/RTIMULib"
RTIMULib_BUILD="${RTIMULib_PATH}/Linux/build"

# Clone repo in RTIMULib_PATH
mkdir -p ${RTIMULib_PATH}
git clone https://github.com/RoboticaUtnFrba/RTIMULib.git ${RTIMULib_PATH}
cd ${RTIMULib_PATH}
# Disable compilation with OpenGL in Travis
# https://docs.travis-ci.com/user/environment-variables/#default-environment-variables
BUILD_GL=""
[ "${CI}" == "true" ] && BUILD_GL="-DBUILD_GL=OFF"
# Build and install RTIMULib
mkdir -p ${RTIMULib_BUILD}
cd ${RTIMULib_BUILD}
cmake ${BUILD_GL} ..
make -j4
sudo make install
sudo ldconfig
