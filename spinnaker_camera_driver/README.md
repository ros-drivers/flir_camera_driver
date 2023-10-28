# spinnaker_camera_driver: ROS driver for FLIR cameras based on the Spinnaker SDK

ROS driver for the FLIR cameras using the
[Spinnaker SDK](http://softwareservices.flir.com/Spinnaker/latest/index.htmlspinnaker).

NOTE: This driver is not written or supported by FLIR.

## Tested cameras:

The following cameras have been used with this driver:

- Blackfly S (USB3, GigE)
- Blackfly (GigE)
- Grashopper (USB3)
- Chameleon (USB3, tested on firmware v1.13.3.00)

Note: if you get other cameras to work, *please report back*, ideally
submit a pull request with the camera config file you have created.

## Tested platforms

Software:

- ROS2 Galactic under Ubuntu 20.04 LTS
- ROS2 Humble under Ubuntu 22.04 LTS
- Spinnaker 3.1.0.79 (other versions may work as well but this is
  what the continuous integration builds are using)

## How to install

This driver can be used with or without installing the Spinnaker SDK,
but installing the Spinnaker SDK is recommended because during its
installation the USB kernel configuration is modified as needed and
suitable access permissions are granted (udev rules).
If you choose to *not* use the Spinnaker SDK, you must either run the
[linux setup script](scripts/linux_setup_flir) by running `ros2 run spinnaker_camera_driver linux_setup_flir`
or perform the [required setup steps manually](docs/linux_setup_flir.md).
Without these setup steps,
*the ROS driver will not detect the camera*.
So you must either install the Spinnaker SDK (which also gives you the
useful ``spinview`` tool), or follow the manual setup steps mentioned earlier.

### Installing from packages
For some architectures and ros distributions you can simply install an apt package:
```
sudo apt install ros-${ROS_DISTRO}-spinnaker-camera-driver
```
The package will bring its own set of Spinnaker SDK libraries, so you don't
necessarily have to install the SDK, but it's recommended, see above

### Building from source

1) Install the FLIR spinnaker driver. If you skip this part, the
driver will attempt to download the Spinnaker SDK automatically to
obtain the header files and libraries.
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

### Single node launch
The driver comes with an example launch file (``driver_node.launch.py``)
that you can customize as needed.
```
# launch with --show-args to print out all available launch arguments
ros2 launch spinnaker_camera_driver driver_node.launch.py camera_type:=blackfly_s serial:="'20435008'"
```

### Stereo camera with synchronization

The launch file ``stereo_synced.launch.py`` provides a working example for launching
drivers for two hardware synchronized Blackfly S cameras. It requires two more packages
to be installed, 
[cam_sync_ros2](https://github.com/berndpfrommer/cam_sync_ros2)(for
time stamp syncing) and
[exposure_control_ros2](https://github.com/berndpfrommer/exposure_control_ros2)
(for external exposure control).
The launch file also demonstrates how to use the driver as a composable node.

## Features

The ROS driver itself has no notion of the camera type (Blackfly,
Grasshopper etc), nor does it explicitly support any of the many
features that the FLIR cameras have. Rather, all camera features
(called Spinnaker Nodes) are mapped to ROS parameters via a yaml file
that is specific to the camera. On startup the driver reads this
parameter definition file. In the ``config`` directory there are some
parameter definition files for popular cameras (blackfly_s.yaml etc)
that expose some of the more frequently used features like frame rate,
gain, etc. You can add more features by providing your own
parameter definition file. The ROS driver code is just a thin wrapper
around the Spinnaker SDK, and should allow you to access all features available in FLIR's
spinview program. *In addition to the parameters defined in the .yaml
files*, the driver has the following ROS parameters:

- ``serial_number``: must have the serial number of the camera. If you
  don't know it, put in anything you like and
  the driver will croak with an error message, telling you what
  cameras serial numbers are available
- ``frame_id``: the ROS frame id to put in the header of the published
  image messages.
- ``camerainfo_url``: where to find the camera calibration yaml file.
- ``parameter_file``: location of the .yaml file defining the camera
  (blackfly_s.yaml etc)
- ``compute_brightness``: if true, compute image brightness and
  publish it in meta data message. This is useful for external
  exposure control but incurs extra CPU load. Default: false.
- ``buffer_queue_size``: max number of images to queue internally
  before shoving them into the ROS output queue. Decouples the
  Spinnaker SDK thread from the ROS publishing thread. Default: 4.
- ``image_queue_size``: ROS output queue size (number of frames). Default: 4
- ``dump_node_map``: set this to true to get a dump of the node map. This
  feature is helpful when developing a new config file. Default: false.
- ``adjust_timestamp``: see below for more documentation
- ``acquisition_timeout``: timeout for expecting frames (in seconds).
  If no frame is received for this time, the driver restarts. Default is 3s.

## Setting up GigE cameras

The Spinnaker SDK abstracts away the transport layer so a GigE camera
should work the same way as USB3: you point it to the serial
number and you're set.

There are a few GigE-specific settings in the Transport Layer Control
group that are important, in particular enabling jumbo frames from the
camera per FLIR's recommendations. The following line in your
camera-specific config file will create a ROS2 parameter
``gev_scps_packet_size``:
```
gev_scps_packet_size int "TransportLayerControl/GigEVision/GevSCPSPacketSize"
```
that you can then set in your ROS2 launch file:
```
 "gev_scps_packet_size": 9000
```
As far as setting up the camera's IP address: you can set up DHCP on
your network or configure a static persistent IP using spinview 
in "Transport Layer Control">"GigE Vision". Check the box for "Current
IP Configuration Persistent IP" first to enable it, then set your
desired addresses under "Persistent IP Address", "Persistent Subnet
Mask" and "Persistent Gateway". NOTE: these look like regular IPs, but
to set them you have to enter the 32-bit integer representation of the
IP address/mask. By hand/calculator: convert the IP octets from
decimal to hex, then combine them and convert to a 32-bit integer, ex:
192.168.0.1 -> 0xC0A80001 -> 3232235521.

The "Transport Layer Control">"GigE Vision" section of spinview is
also where you'll find that "SCPS Packet Size" setting, which you can
change when not capturing frames, and verify it works in spinview and
without needing to spin up a custom launch file to get started, though
it helps, and you'll probably want one anyway to specify your camera's
serial number.

For more tips on GigE setup look at FLIR's support pages
[here](https://www.flir.com/support-center/iis/machine-vision/knowledge-base/lost-ethernet-data-packets-on-linux-systems/)
and
[here](https://www.flir.com/support-center/iis/machine-vision/application-note/troubleshooting-image-consistency-errors/).

### Time stamps

By default the driver will set the ROS header time stamp to be the
time when the image was delivered by the SDK. Such time stamps are not
very precise and may lag depending on host CPU load. However the
driver has a feature to use the much more accurate sensor-provided
camera time stamps. These are then converted to ROS time stamps by
estimating the offset between ROS and sensor time stamps via a simple
moving average. For the adjustment to work
*the camera must be configured to send time stamps*, and the
``adjust_timestamp`` flag must be set to true, and the relevant field
in the "chunk" must be populated by the camera. For the Blackfly S
the parameters look like this:
```
    'adjust_timestamp': True,
    'chunk_mode_active': True,
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
```

When running hardware synchronized cameras in a stereo configuration
two drivers will need to be run, one for each camera. This will mean
however that their published ROS header time stamps are *not*
identical which in turn may prevent down-stream ROS nodes from recognizing the
images as being hardware synchronized. You can use the
[cam_sync_ros2 node](https://github.com/berndpfrommer/cam_sync_ros2)
to force the time stamps to be aligned. In this scenario it is
mandatory to configure the driver to adjust the ROS time stamps as
described above.

### Automatic exposure

In most situations it is recommended to enable the built-in auto
exposure of the camera. However, in a
synchronized setting it is sometimes desirable to disable the built-in
auto-exposure and provide it externally. For instance in a stereo setup,
matching left and right image patches can be difficult when each
camera runs its own auto exposure independently. The
[exposure_control_ros2](https://github.com/berndpfrommer/exposure_control_ros2)
package can provide external automatic exposure control. To this end
the driver publishes
[meta data messages](https://github.com/ros-drivers/flir_camera_driver/flir_camera_msgs) and
subscribes to 
[camera control
messages](https://github.com/ros-drivers/flir_camera_driver/flir_camera_msgs). See
the launch file directory for examples.

## How to add new features and develop your own camera configuration file

[Check out this section](docs/camera_configuration_files.md) for more information on
how to add features.

## Known issues

1) If you run multiple drivers in separate nodes that all access USB based
devices, starting a new driver will stop the image acquisition of
currently running drivers. There is an ugly workaround for this
currently implemented: if image delivery stops for more than
``acquisition_timeout`` seconds, the acquisition is restarted. This
operation may not be thread safe so the driver already running could
possibly crash. This issue can be avoided by running all drivers in
the same address space with a composable node (see stereo launch file for
example).

## How to contribute
Please provide feedback if you cannot get your camera working or if
the code does not compile for you. Feedback is crucial for the
software development process. However, before opening issues on github first
verify that the problem is not present when using spinview.

Bug fixes and config files for new cameras are greatly
appreciated. Before submitting a pull request, run this to see if your
commit passes some basic lint tests:
```
colcon test --packages-select spinnaker_camera_driver && colcon test-result --verbose
```

## License

This software is issued under the Apache License Version 2.0.
The file [TargetArch.cmake](cmake/TargetArch.cmake) is released under
a custom license (see file)
