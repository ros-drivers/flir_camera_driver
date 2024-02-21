# spinnaker_synchronized_camera_driver: ROS driver for synchronized FLIR cameras based on the Spinnaker SDK

ROS driver for synchronized FLIR cameras using the
[Spinnaker SDK](http://softwareservices.flir.com/Spinnaker/latest/index.htmlspinnaker).

NOTE: This driver is not written or supported by FLIR.

## Tested cameras:

The following cameras have been used with this driver:

- Blackfly S (USB3)
- Blackfly (GigE)

Note: if you get other cameras to work, *please report back*, ideally
submit a pull request with the camera config file you have created.

## Tested platforms

Software:

- ROS2 Humble under Ubuntu 22.04 LTS
- Spinnaker 3.1.0.79 (other versions may work as well but this is
  what the continuous integration builds are using)

## How to install

It is recommended to first install the Spinnaker SDK from FLIR's web site because it has
tools (SpinView) that are very helpful for finding the correct camera configuration. You will also need to install the [ROS2 spinnaker camera driver](../spinnaker_camera_driver/README.md). It is recommended to first test the single camera drivers before proceeding with the synchronized setup. 

### Installing from packages
For some architectures and ROS2 distributions you can simply install an apt package:
```
sudo apt install ros-${ROS_DISTRO}-spinnaker-synchronized-camera-driver
```

### Building from source

1) Although not necessary, it is recommended to install the Spinnaker SDK.
2) Prepare the ROS2 driver build:
Make sure you have your ROS2 environment sourced:
```
source /opt/ros/<my_ros_distro>/setup.bash
```

Create a workspace (``~/ws``), clone this repo:
```
mkdir -p ~/ws/src
cd ~/ws/src
git clone --branch humble-devel https://github.com/ros-drivers/flir_camera_driver
cd ..
```

To automatically install all packages that the ``flir_camera_driver`` packages
depends upon, run this at the top of your workspace:
```
rosdep install --from-paths src --ignore-src
```

3) Build the driver and source the workspace:
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
. install/setup.bash
```

## Example usage

### Launch the example stereo node
The driver comes with an example launch file (``synchronized_driver_node.launch.py``)
that must be edited to e.g. adjust for camera type and serial numbers
```
ros2 launch spinnaker_synchronized_camera_driver synchronized_driver_node.launch.py
```
Note that the relevant camera parameters must be set here, in particular chunks need to be enabled that have the time stamps, and the cameras synchronization modes need to be set correctly as well.

## Features

The synchronized driver has the following parameters:

- ``cameras``: a list of strings, e.g. ["cam_0", "cam_1"] that gives
  the camera names. The driver will instantiate a camera for each name, and
  its parameters can then be set in the respective name space, e.g. for a blackfly_s camera you could set the trigger mode via ROS parameter ``cam_0.trigger_mode``.

The remaining per-camera parameters, *in particular for enabling the chunk mode time stamps* must be set via the launch files in the respective name spaces of each camera. See the launch file for how this is done.

## Known issues

See the caveats for the [ROS2 single-camera spinnaker driver](../spinnaker_camera_driver/README.md).

## How to contribute

Bug fixes and config files for new cameras are greatly appreciated. Before submitting a pull request, run this to see if your commit passes some basic lint tests:
```
colcon test --packages-select spinnaker_synchronized_camera_driver && colcon test-result --verbose
```

## License

This software is issued under the Apache License Version 2.0.
