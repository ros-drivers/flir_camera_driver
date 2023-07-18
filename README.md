# flir_camera_driver

This repository contains packages for FlirImaging's line of cameras. This repositories intent is to make use of Flir's newly developed SDK: Spinnaker. The camera driver is an evolution of pointgrey_camera_driver. It has been updated to use the new methods provided by the SDK.

## Packages

### spinnaker_camera_driver
The camera driver supports USB3 and GIGE cameras. The driver has been
successfully used for Blackfly, Blackfly S, Chameleon, and Grasshopper
cameras, but should support any FLIR camera that is based on the
Spinnaker SDK. See the
[spinnaker_camera_driver](spinnaker_camera_driver/README.md) for more.
This software is issued under the Apache License Version 2.0 and BSD

### flir_camera_msgs
Meta messages with image exposure and control messages. These are used
by the [spinnaker_camera_driver](spinnaker_camera_driver/README.md).
This software is issued under the Apache License Version 2.0.

### flir_camera_description
Package with meshesMeta messages with image exposure and control messages. These are used
by the [spinnaker_camera_driver](spinnaker_camera_driver/README.md). This software is released under a BSD license.
