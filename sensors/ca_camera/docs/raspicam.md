# Raspicam: camera for Raspberry

This tutorial will consider that you already have the hardware conected.

## Getting started

Firstly, ssh **into Raspberry** and start broadcasting video **to our server**:

```bash
roscd ca_camera/scripts

./stream_raspicam <HOST_IP>
```

Find the IP of the server using `ifconfig`.

Then launch the ROS nodes in the server:

```bash
roslaunch ca_camera stream_test.launch
```

## Calibrating the camera

You will need to have printed [this checkerboard](media/check-108.pdf).

Run the calibration node with this command:

```bash
roslaunch ca_camera calibrate.launch
```

![calibration node opened](media/calibration_01.png)

In order to get a good calibration you will need to move the checkerboard around in the camera frame such that:

* checkerboard on the camera’s left, right, top and bottom of field of view
  * X bar - left/right in field of view
  * Y bar - top/bottom in field of view
  * Size bar - toward/away and tilt from the camera
* checkerboard filling the whole field of view
* checkerboard tilted to the left, right, top and bottom

As you move the checkerboard around you will see three bars on the calibration sidebar increase in length. When the **CALIBRATE** button lights, you have enough data for calibration and can click **CALIBRATE** to see the results.

![calibration done](media/calibration_02.png)

Calibration can take about a minute. The windows might be greyed out but just wait, it is working.

 You will need to move the generated .yaml file that's in `/tmp/calibrationdata.tar.gz`. Extract and rename it to `ost.yaml` and move it to the `ca_camera/config` directory.
