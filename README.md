# FLIR/Spinnaker ROS2 driver

Simple ROS2 driver for the FLIR cameras using the [Spinnaker
SDK](http://softwareservices.flir.com/Spinnaker/latest/index.htmlspinnaker).

NOTE: This driver is not written or supported by FLIR.

## Tested cameras:

The following cameras have been used with this driver:

- Blackfly S (USB3, GigE)
- Blackfly (GigE)
- Grashopper (USB3)
- Chameleon (USB3) tested on firmware v1.13.3.00

Note: if you get other cameras to work, *please report back*, ideally
submit a pull request with the camera config file you have created.

## Supported platforms

Software:

- Ubuntu 20.04 LTS
- ROS2 Galactic and Foxy
- Spinnaker 2.6.0.157 (other versions may work as well but this is
  what the continuous integration builds are using)

The code compiles under Ubuntu 22.04 / Humble but has not been tested
yet with real hardware.

## Features

Basic features are supported like setting exposure, gain, and external
triggering. It's straight forward to support new camera types and features by
editing the camera definition (.cfg) files. Unless you need new pixel
formats you may not have to modify any source code. The code is meant
to be a thin wrapper for setting the features available in FLIR's
SpinView program. The driver has following parameters,
*in addition to the parameters defined in the .cfg files*:

- ``serial_number``: must have the serial number of the camera. If you
  don't know it, put in anything you like and
  the driver will croak with an error message, telling you what
  cameras serial numbers are available
- ``frame_id``: the ROS frame id to put in the header of the published
  image messages.
- ``camerainfo_url``: where to find the camera calibration yaml file.
- ``parameter_file``: location of the .cfg file defining the camera
  (blackfly_s.cfg etc)
- ``compute_brightness``: if true, compute image brightness and
  publish it in meta data message. This is useful for external
  exposure control but incurs extra CPU load. Default: false.
- ``buffer_queue_size``: max number of images to queue internally
  before shoving them into the ROS output queue. Decouples the
  Spinnaker SDK thread from the ROS publishing thread. Default: 4.
- ``image_queue_size``: ROS output queue size (quality of
  service). Default: 4
- ``dump_node_map``: set this to true to get a dump of the node map. This
  feature is helpful when hacking a new config file. Default: false.
  

## How to build

1) Install the FLIR spinnaker driver.
2) Prepare the ROS2 driver build:
Make sure you have your ROS2 environment sourced:
```
source /opt/ros/galactic/setup.bash
```

Create a workspace (``flir_spinnaker_ros2_ws``), clone this repo, and use ``wstool``
to pull in the remaining dependencies:

```
mkdir -p ~/flir_spinnaker_ros2_ws/src
cd ~/flir_spinnaker_ros2_ws
git clone https://github.com/berndpfrommer/flir_spinnaker_ros2 src/flir_spinnaker_ros2
wstool init src src/flir_spinnaker_ros2/flir_spinnaker_ros2.rosinstall

# or to update an existing space
# wstool merge -t src src/flir_spinnaker_ros2/flir_spinnaker_ros2.rosinstall
# wstool update -t src
```

To automatically install all packages that the ``flir_spinnaker_ros2``
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

How to launch the example file:
```
ros2 launch flir_spinnaker_ros2 blackfly_s.launch.py camera_name:=blackfly_0 serial:="'20435008'"
```

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
your network or configure a static persistent IP using SpinView 
in "Transport Layer Control">"GigE Vision". Check the box for "Current
IP Configuration Persistent IP" first to enable it, then set your
desired addresses under "Persistent IP Address", "Persistent Subnet
Mask" and "Persistent Gateway". NOTE: these look like regular IPs, but
to set them you have to enter the 32-bit integer representation of the
IP address/mask. By hand/calculator: convert the IP octets from
decimal to hex, then combine them and convert to a 32-bit integer, ex:
192.168.0.1 -> 0xC0A80001 -> 3232235521.

The "Transport Layer Control">"GigE Vision" section of SpinView is
also where you'll find that "SCPS Packet Size" setting, which you can
change when not capturing frames, and verify it works in SpinView and
without needing to spin up a custom launch file to get started, though
it helps, and you'll probably want one anyway to specify your camera's
serial number.

For more tips on GigE setup look at FLIR's support pages
[here](https://www.flir.com/support-center/iis/machine-vision/knowledge-base/lost-ethernet-data-packets-on-linux-systems/)
and
[here](https://www.flir.com/support-center/iis/machine-vision/application-note/troubleshooting-image-consistency-errors/).

## Camera synchronization

In the ``launch`` folder you can find a working example for launching
drivers for two hardware synchronized Blackfly S cameras. The launch
file requires two more packages to be installed,
[cam_sync_ros2](https://github.com/berndpfrommer/cam_sync_ros2)(for
time stamp syncing) and
[exposure_control_ros2](https://github.com/berndpfrommer/exposure_control_ros2)
(for external exposure control). See below for more details on those packages.

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

While FLIR cameras generally have built-in exposure control, in a
synchronized setting it is sometimes desirable to also synchronize and
completely control the auto-exposure. For instance in a stereo setup
matching left and right image patches can be difficult when each
camera runs its own auto exposure. The
[exposure_control_ros2](https://github.com/berndpfrommer/exposure_control_ros2)
package can provide external automatic exposure control. To this end
the driver publishes
[meta data messages](https://github.com/berndpfrommer/image_meta_msgs_ros2) and
subscribes to 
[camera control messages](https://github.com/berndpfrommer/camera_control_msgs_ros2).

## How to add new features

For lack of a more systematic way to discover the camera configuration node
names the following procedure is recommended for adding new features:

- fire up FLIR's ``spinview`` application and find the parameter you want to add
  to the config.

- start with an existing config file in the ``config`` directory, make a copy
  and give the feature a name that follows the established convention of
  other parameters (all lower case, separate by underscores), for example
  ```
    device_link_throughput_limit int "DeviceControl/DeviceLinkThroughputLimit"
  ```
  The parameter name is followed by the parameter type (in this example ``int``),
  which you have to somewhat guess. If ``spinview`` shows a multiple choice drop-down box,
  the parameter is of type ``enum``, a check box translates to ``bool``,
  otherwise it's ``float`` or ``int`` (check what input ``spinview``) accepts.

- the hard part is the node name which is the last parameter of the line, in this
  example ``"DeviceControl/DeviceLinkThroughputLimit"``. It usually follows by
  removing spaces from the ``spinview`` names. If that doesn't work,
  launch the driver with the ``dump_node_map`` parameter set to "True"
  and look at the output for inspiration.

Once you have modified the config file, now just set the newly created
parameter in the launch file, done.

## Known issues

1) If you run multiple drivers in separate nodes that all access USB based
devices, starting a new driver will stop the image acquisition of
currently running drivers. There is an ugly workaround for this
currently implemented: if image delivery stops for more than
``acquisition_timeout`` seconds, the acquisition is restarted. This
operation may not be thread safe so the driver already running could
possibly crash. This issue can be avoided by running all drivers in
the same address space with a composable node.

## How to contribute
Please provide feedback if you cannot get your camera working or if
the code does not compile for you. Feedback is crucial for the
software development process.

Bug fixes and config files for new cameras are greatly
appreciated. Before submitting a pull request, run this to see if your
commit passes some basic lint tests:
```
colcon test --packages-select flir_spinnaker_ros2 && colcon test-result --verbose
```


## License

This software is issued under the Apache License Version 2.0.
