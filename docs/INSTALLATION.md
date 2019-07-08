# Installation

## Prerequisites

* [ROS](http://wiki.ros.org/ROS/Installation) _Kinetic_ or _Melodic_
* [Gazebo 9](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0#Alternativeinstallation:step-by-step) or higher
* Ubuntu packages: `python-rosdep`, `python-catkin-tools`

``` bash
sudo apt-get install -y python-rosdep python-catkin-tools
```

## Compiling

1. Create a catkin workspace

    ``` bash
    mkdir -p ~/create_ws/src
    ```

2. Clone this repo

    ``` bash
    cd ~/create_ws/src
    git clone https://github.com/RoboticaUtnFrba/create_autonomy.git
    ```

3. Compile RTIMULib2

    ```bash
    cd ~/create_ws/src/RTIMULib2/Linux && mkdir build && cd build
    sudo apt-get install -y libqt4-dev
    cmake ..
    make -j4
    sudo make install
    sudo ldconfig
    ```

4. Install dependencies

    ``` bash
    sudo apt-get install -y gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
    cd ~/create_ws
    sudo apt install -y python3-vcstool
    vcs import src < src/create_autonomy/<DEVICE>.repos
    rosdep update
    rosdep install --from-paths src -i
    ```

5. Build

    5.1. Build RTIMULib2 for Raspberry Pi

    ``` bash
    cd ~/create_ws/src/RTIMULib2/Linux
    mkdir build
    cd build
    cmake ..
    make -j4
    sudo make install
    sudo ldconfig
    ```

    5.2. Build workspace

    ``` bash
    cd ~/create_ws
    catkin_make -DCMAKE_BUILD_TYPE=Release
    ```

### USB Permissions

1. In order to connect to Create over USB, ensure your user is in the dialout group

    ``` bash
    sudo usermod -a -G dialout $USER
    ```

2. Logout and login for permission to take effect
