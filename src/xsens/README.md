# Xsens MTw driver

This project contains a driver on which the Xsens MTw sensors send the data in topics published through a node managing the
connection with the Awinda base (USB port). Development based on [Xsens MTw SDK 4.6](https://www.xsens.com/mt-software-suite-mtw-awinda/).

### Hardware

- The Xsens Hardware is an MTw Development Kit

![Awinda station and MTw sensors](data:image/jpeg;base64,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)

### Software

- The driver is developed upon the Xsens SDK 4.6, with ROS Melodic on Ubuntu 18.04

## Usage

- All dependencies (libraries, headers and sources files) are in this folder structure. The _CMakeList.txt_ file is ready to use, contemplating the mininum XSens SDK required.

- Connect the Awinda Station USB in your computer and run the MTw Driver node: `$ rosrun xsens_mtw_drive mt_w_manager`

- Undock the MTW sensor and wait until the wireless connection being established: 

```
[ INFO] [1565393292.619168658]: Waiting for MTW to wirelessly connect...
[ INFO] [1565393436.611962400]: EVENT: MTW Connected -> 00342322
[ INFO] [1565393436.615162761]: Number of connected MTWs: 1. Press 'y' to start measurement or 'q' to end node.

```

- Each MTw sensor will connect at once. Remember, as described on the Xsens MTw User Manual:

| MTw  | desiredUpdateRate (max) |
|------|-------------------------|
|  1   |           150 Hz        |
|  2   |           120 Hz        |
|  4   |           100 Hz        |
|  6   |            75 Hz        |
|  12  |            50 Hz        |
|  18  |            40 Hz        |

## Troubleshooting

If happen some problem with device access, follow the recommendations on the [xsens_mti_driver page](http://wiki.ros.org/xsens_mti_driver):

- Make sure you are in the correct group:

```
$ ls -l /dev/ttyUSB0

crw-rw---- 1 root dialout 188, 0 May  6 16:21 /dev/ttyUSB0

$ groups

"username" adm cdrom sudo dip plugdev lpadmin sambashare
```

- Add yourself to the group: 
```
$ sudo usermod -G dialout -a $USER
$ newgrp dialout
```