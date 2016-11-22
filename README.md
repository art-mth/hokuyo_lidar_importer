#hokuyo_lidar_importer

###Setup
Install the driver/library from [here](https://sourceforge.net/projects/urgnetwork/) or use the one from Conan.

###FAQ:

####The Lidar is plugged in, but I get an error along the lines of "failed opening serial device". What is the problem?

This is most likely a permission problem. There is a few ways to solve this. If you look at the permissions of `/dev/ttyACM0` you will see something along the lines of `crw-rw---- 1 root dialout`. The easiest way to solve this is to add your user to the dialout group. `sudo adduser $USER dialout`. For this change to take effect either reboot or run `newgrp dialout`.
