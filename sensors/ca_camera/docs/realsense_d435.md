# Intel RealSense D435

<https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i>

## [Firmware update](https://dev.intelrealsense.com/docs/firmware-update-tool)

- Download binary package from [this page](https://dev.intelrealsense.com/docs/firmware-releases).

- You must have `librealsense` installed. It's recommended to use the provided Docker image.

- Make sure you have the device connected running:

```bash
rs-fw-update -l
```

- Update the firmware:

```bash
rs-fw-update -r -f /path/to/Signed_Image_XYZ.bin
```
