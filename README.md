# flir_camera_driver

This repository contains ROS packages for cameras made by FLIR Imaging (formerly known as PointGrey).

## Packages

### spinnaker_camera_driver
The camera driver supports USB3 and GIGE cameras. The driver has been
successfully used for Blackfly, Blackfly S, Chameleon, and Grasshopper
cameras, but should support any FLIR camera that is based on the
Spinnaker SDK. See the
[spinnaker_camera_driver](spinnaker_camera_driver/README.md) for more.
This software is issued under the Apache License Version 2.0 and BSD

### flir_camera_msgs
Package with with [image exposure and control messages](flir_camera_msgs/README.md).
These are used by the [spinnaker_camera_driver](spinnaker_camera_driver/README.md).
This software is issued under the Apache License Version 2.0.

### flir_camera_description
Package with [meshes and urdf](flir_camera_description/README.md) files.
This software is released under a BSD license.
