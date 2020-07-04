# Installation

## Prerequisites

* [ROS](http://wiki.ros.org/ROS/Installation) _Melodic_
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
    git clone https://github.com/RoboticaUtnFrba/libcreate.git
    ```

3. Install dependencies

    I. Install RTIMULib

    ```bash
    cd ~/create_ws
    ./src/create_autonomy/sensors/ca_imu/scripts/install_rtimulib.sh
    ```

    II. Install other dependencies

    ``` bash
    cd ~/create_ws
    sudo apt install -y python3-vcstool
    vcs import src < src/create_autonomy/dependencies.repos
    rosdep update
    rosdep install --from-paths src -yi
    ```

4. Build workspace

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
