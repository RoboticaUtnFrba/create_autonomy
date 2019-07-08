# Running the driver

## Setup

1. After compiling from source, **don't forget to source your workspace**:

    ``` bash
    source ~/create_ws/devel/setup.bash
    ```

2. Connect computer to Create's 7-pin serial port

3. Launch one of the existing launch files or adapt them to create your own.

## Launch files

For Create 2 (Roomba 600/700 series):

``` bash
roslaunch ca_driver create_2.launch
```

### Launch file arguments

* **config** - Absolute path to a configuration file (YAML). Default: `ca_driver/config/default.yaml`
* **desc** - Enable robot description (URDF/mesh). Default: `true`

For example, if you would like to disable the robot description and provide a custom configuration file:

``` bash
roslaunch ca_driver create_2.launch config:=/abs/path/to/config.yaml desc:=false
```
