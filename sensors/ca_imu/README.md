# IMU (MPU9255 9 DOF)

Interfacing **Nvidia Jetson Nano** with **MPU9255**:

> VCC --> 5V
> GND --> GND
> SCL --> PIN 5
> SDA --> PIN 3

## [Setup](https://elinux.org/Jetson/I2C)

```bash
sudo apt-get install -y i2c-tools

sudo usermod -aG i2c $USER

$ sudo i2cdetect -y -r 1
0x68
```

## Installation

### [RTIMULib](https://github.com/RoboticaUtnFrba/RTIMULib)

RTIMULib is automatically installed when the repository is compiled using the script located in `ca_imu/scripts/install_rtimulib.sh`.

## [Calibrate IMU](https://github.com/RoboticaUtnFrba/RTIMULib/blob/master/Calibration.pdf)

The normal process is to run the magnetometer min/max option followed by the magnetometer ellipsoid fit option followed finally by the accelerometer min/max option.

The resulting `RTIMULib.ini` can then be used by any other RTIMULib application.

```bash
RTIMULibCal
```

### Calibrating the magnetometer

[This video](https://www.youtube.com/watch?v=-Uq7AmSAjt8) shows how to wag your IMU in order to calibrate the magnetometer.

### Calibrating the accelerometer

[This video](https://www.youtube.com/watch?v=CnLtzwCbVc4) shows how the calibration process of the accelerometer has to be made.

### Running

```bash
roslaunch ca_imu mpu9255.launch
```
