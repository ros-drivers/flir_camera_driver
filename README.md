# flir_camera_driver

This repository contains ROS packages for machine vision cameras made by Teledyne/FLIR (formerly known as PointGrey).

## Packages

### spinnaker_camera_driver
A camera driver supporting USB3 and GIGE cameras and has been
successfully used for Blackfly, Blackfly S, Chameleon, and Grasshopper
cameras, but should work with any FLIR camera that supports the
Spinnaker SDK. See the
[spinnaker_camera_driver](spinnaker_camera_driver/README.md) for more.
This software is issued under the Apache License Version 2.0 and BSD.\
Build status:
[![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__spinnaker_camera_driver__ubuntu_jammy_amd64__binary&subject=Humble)](https://build.ros2.org/job/Hbin_uJ64__spinnaker_camera_driver__ubuntu_jammy_amd64__binary/)
[![Build Status](https://build.ros2.org/buildStatus/icon?job=Ibin_uJ64__spinnaker_camera_driver__ubuntu_jammy_amd64__binary&subject=Iron)](https://build.ros2.org/job/Ibin_uJ64__spinnaker_camera_driver__ubuntu_jammy_amd64__binary/)

### spinnaker_synchronized_camera_driver
Based on the spinnaker\_camera\_driver package, this driver is specifically designed for cameras hardware triggered by an external signal. Images triggered by the same external pulse will have identical ROS header time stamps. See the [spinnaker_synchronized_camera_driver](spinnaker_synchronized_camera_driver/README.md) for more.\
Build status:
[![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__spinnaker_synchronized_camera_driver__ubuntu_jammy_amd64__binary&subject=Humble)](https://build.ros2.org/job/Hbin_uJ64__spinnaker_synchronized_camera_driver__ubuntu_jammy_amd64__binary/)
[![Build Status](https://build.ros2.org/buildStatus/icon?job=Ibin_uJ64__spinnaker_synchronized_camera_driver__ubuntu_jammy_amd64__binary&subject=Iron)](https://build.ros2.org/job/Ibin_uJ64__spinnaker_synchronized_camera_driver__ubuntu_jammy_amd64__binary/)

### flir_camera_description
Package with [meshes and urdf](flir_camera_description/README.md) files.
This software is released under a BSD license.\
Build status:
[![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__flir_camera_description__ubuntu_jammy_amd64__binary&subject=Humble)](https://build.ros2.org/job/Hbin_uJ64__flir_camera_description__ubuntu_jammy_amd64__binary/)
[![Build Status](https://build.ros2.org/buildStatus/icon?job=Ibin_uJ64__flir_camera_description__ubuntu_jammy_amd64__binary&subject=Iron)](https://build.ros2.org/job/Ibin_uJ64__flir_camera_description__ubuntu_jammy_amd64__binary/)

### flir_camera_msgs
Package with with [image exposure and control messages](flir_camera_msgs/README.md).
These are used by the [spinnaker_camera_driver](spinnaker_camera_driver/README.md).
This software is issued under the Apache License Version 2.0.\
Build status:
[![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__flir_camera_msgs__ubuntu_jammy_amd64__binary&subject=Humble)](https://build.ros2.org/job/Hbin_uJ64__flir_camera_msgs__ubuntu_jammy_amd64__binary/)
[![Build Status](https://build.ros2.org/buildStatus/icon?job=Ibin_uJ64__flir_camera_msgs__ubuntu_jammy_amd64__binary&subject=Iron)](https://build.ros2.org/job/Ibin_uJ64__flir_camera_msgs__ubuntu_jammy_amd64__binary/)

