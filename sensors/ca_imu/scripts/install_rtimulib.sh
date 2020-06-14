#!/bin/bash

# Set RTIMULib paths
RTIMULib_PATH="/home/${USER}/RTIMULib"
RTIMULib_BUILD="${RTIMULib_PATH}/Linux/build"

# Clone repo in RTIMULib_PATH
mkdir -p ${RTIMULib_PATH}
git clone https://github.com/RoboticaUtnFrba/RTIMULib.git ${RTIMULib_PATH}
cd ${RTIMULib_PATH}
# Install required dependency (libqt4-dev)
sudo apt-get install -y libqt4-dev
# Build and install RTIMULib
mkdir -p ${RTIMULib_BUILD}
cd ${RTIMULib_BUILD}
cmake ..
make -j4
sudo make install
sudo ldconfig
