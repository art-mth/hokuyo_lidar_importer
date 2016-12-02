# hokuyo_lidar_importer
Imports data from a Hokuyo URG-04LX-UG01 LIDAR. The data points are transformed into vertices in the car cartesian plane and written to a data channel as a 2d point cloud. The import is running asynchronously and cycles without new data are NOP.

## Setup
Install the driver/library from [here](https://sourceforge.net/projects/urgnetwork/) or use the one from Conan.

## Data Channels
#### Write
- HOKUYO_LIDAR_DATA(lms::math::PointCloud2f)
- NEW_DATA(bool)

## Config
Offset values mark the position of the LIDAR relative to the "Origin". "Origin" is the origin of the car cartesian plane and in the current model is the position of the Camera. The offset is the position of the LIDAR in that plane. E.g. LIDAR position is (0.1, -0.1) -> xOffset = 0.1, yOffset = -0.1.
- xOffsetFromOriginMeter
- yOffsetFromOriginMeter

The Deg values mark the LIDAR range that is evaluated. The Lidar scans in anticlockwise direction with the common unit circle angles where 0 is the straight ahead view of the LIDAR. Going anticlockwise are positive values, going clockwise are negative values. The exact values for max and min can be retrieved from the driver.
- startAtDeg: Minimum Value ~= -119
- stopAtDeg: Maximum Value ~= 119

- deviceFile: The device file that acts as the interface to the LIDAR. On your Linux Box this will probably be something like `/dev/ttyACM0`. This might be different on the car.

## FAQ

#### The Lidar is plugged in, but I get an error along the lines of "failed opening serial device". What is the problem?

This is most likely a permission problem. There is a few ways to solve this. If you look at the permissions of `deviceFile` you will probably see something along the lines of `crw-rw---- 1 root dialout`. The easiest way to solve this is to add your user to the dialout group. `sudo adduser $USER dialout`. For this change to take effect either reboot or run `newgrp dialout`.
