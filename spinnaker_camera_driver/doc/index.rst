============================
Spinnaker ROS2 Camera Driver
============================

.. toctree::
   :maxdepth: 2


This package provides a ROS2 driver for Teledyne/FLIR cameras using the
`Spinnaker
SDK <http://softwareservices.flir.com/Spinnaker/latest/index.html>`__.
For hardware-synchronized cameras use the Spinnaker synchronized camera driver by
following the link from the
`flir driver repository <https://github.com/ros-drivers/flir_camera_driver>`__.


NOTE: This driver is not written or supported by FLIR.

Tested Configurations
=====================
Cameras
-------

The following cameras have been successfully used with this driver:

-  Blackfly S (USB3, GigE)
-  Blackfly (GigE)
-  Grashopper (USB3)
-  Oryx (reported working)
-  Chameleon (USB3, tested on firmware v1.13.3.00)
-  FLIR AX5 (GigE)

Note: if you get other cameras to work, *please report back*, ideally
submit a pull request with the camera config file you have created.

Platforms
---------

-  ROS2 Galactic under Ubuntu 20.04 LTS (no longer actively tested)
-  ROS2 Humble/Iron under Ubuntu 22.04 LTS
   ROS2 Rolling/Jazzy under Ubuntu 24.04 LTS
-  Spinnaker 3.1.0.79 (other versions may work as well but this is what
   the continuous integration builds are using)

How to install
==============

This driver can be used with or without installing the Spinnaker SDK,
but installing the Spinnaker SDK is recommended because during its
installation the USB kernel configuration is modified as needed and
suitable access permissions are granted (udev rules). If you choose to
*not* use the Spinnaker SDK, you must either run the `linux setup
script <scripts/linux_setup_flir>`__ by running
``ros2 run spinnaker_camera_driver linux_setup_flir`` or perform the
required setup steps manually, see `Setting up Linux without Spinnaker SDK`_.
Without these setup steps, *the ROS driver will not detect the camera*.
So you must either install the Spinnaker SDK (which also gives you the useful
``spinview`` tool), or follow the manual setup steps mentioned earlier.

Installing from packages
------------------------

For some architectures and ros distributions you can simply install an
apt package:

::

   sudo apt install ros-${ROS_DISTRO}-spinnaker-camera-driver

The package will bring its own set of Spinnaker SDK libraries, so you
don’t necessarily have to install the SDK, but it’s recommended, see
above.

Building from source
--------------------

1) Install the FLIR spinnaker driver. If you skip this part, the driver
   will attempt to download the Spinnaker SDK automatically to obtain
   the header files and libraries.
2) Prepare the ROS2 driver build: Make sure you have your ROS2
   environment sourced:

::

   source /opt/ros/<my_ros_distro>/setup.bash

Create a workspace (``~/ws``), clone this repo:

::

   mkdir -p ~/ws/src
   cd ~/ws/src
   git clone --branch humble-devel https://github.com/ros-drivers/flir_camera_driver
   cd ..

To automatically install all packages that the ``flir_camera_driver``
packages depends upon, run this at the top of your workspace:

::

   rosdep install --from-paths src --ignore-src

3) Build the driver and source the workspace:

::

   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
   . install/setup.bash

How to use
==========

Topics
------

Published:

- ``~/image_raw``: the camera image (image_transport)
- ``~/image_raw/camera_info``: camera calibration
- ``~/meta``: meta data message containing e.g. exposure time.

Subscribed:

- ``~/control``: (only when ``enable_external_control`` is set to True)
   for external control exposure time and gain.

Parameters
----------

The driver itself has no notion of the camera type (Blackfly,
Grasshopper etc), nor does it explicitly support any features that the FLIR cameras have.
Rather, all camera features (called
Spinnaker Nodes) are mapped to ROS parameters via a yaml file that is
specific to that camera type. On startup the driver reads this parameter
definition file. In the ``config`` directory there are some parameter
definition files for popular cameras (blackfly_s.yaml etc) that expose
some of the more frequently used features like frame rate, gain, etc.
You can add more features by providing your own parameter definition
file, see `How to develop your own camera configuration file`_.
The ROS driver code is just a thin wrapper around the Spinnaker
SDK, and should allow you to access all features available in FLIR’s
spinview program.

*In addition to the parameters defined in the .yaml
files*, the driver has the following ROS parameters:

-  ``adjust_timestamp``: see `About time stamps`_ below for more documentation. Default: false.
-  ``acquisition_timeout``: timeout for expecting frames (in seconds).
   If no frame is received for this time, the driver restarts. Default
   is 3.
-  ``buffer_queue_size``: max number of images to queue internally
   before shoving them into the ROS output queue. Decouples the
   Spinnaker SDK thread from the ROS publishing thread. Default: 4.
-  ``camerainfo_url``: where to find the camera calibration yaml file.
-  ``compute_brightness``: if true, compute image brightness and publish
   it in meta data message. This is required when running with
   the synchronized driver, but incurs extra CPU load. Default: false.
-  ``connect_while_subscribed``: if true, connect to the SDK and pull
   data from the camera *only while subscribers to image or meta topics
   are present*. This feature reduces compute load and link utilization
   while no ROS subscribers are present, but adds latency on
   subscription: after a subscription the first image will be published up to 1s later
   than without this option.
-  ``dump_node_map``: set this to true to get a dump of the node map.
   Default: false.
-  ``enable_external_control``: set this to true to enable external exposure control.
   This feature is being deprecated, *do not use*.   Default: false.
-  ``frame_id``: the ROS frame id to put in the header of the published image messages.
-  ``image_queue_size``: ROS output queue size (number of frames). Default: 4
-  ``parameter_file``: location of the .yaml file defining the camera
   (blackfly_s.yaml etc)
-  ``serial_number``: serial number of the camera. If you
   don’t know it, put in anything you like and the driver will croak
   with an error message, telling you what cameras serial numbers are
   available

Example usage
-------------
The driver comes with an example launch file (``driver_node.launch.py``)
that you can customize as needed.

::

   # launch with --show-args to print out all available launch arguments
   ros2 launch spinnaker_camera_driver driver_node.launch.py camera_type:=blackfly_s serial:="'20435008'"


Using multiple cameras at the same time
=======================================

The FLIR Spinnaker does not support two programs accessing the Spinnaker SDK at the same time,
even if two different cameras are accessed. Strange things happen, in particular with USB3 cameras.
You can however run multiple cameras in the same
address space using ROS2's Composable Node concept, see the example
launch file ``multiple_cameras.launch.py``.

If you are using hardware synchronized cameras please use the ``spinnaker_synchronized_camera_driver``,
which will associate the images triggered by the same sync pulse with each other
and give them identical time stamps.


About time stamps
=================

By default the driver will set the ROS header time stamp to be the time
when the image was delivered by the SDK. Such time stamps are not very
precise and may lag depending on host CPU load. However the driver has a
feature to use the much more accurate sensor-provided camera time
stamps. These are then converted to ROS time stamps by estimating the
offset between ROS and sensor time stamps via a simple moving average.
For the adjustment to work *the camera must be configured to send time
stamps*, and the ``adjust_timestamp`` flag must be set to true, and the
relevant field in the “chunk” must be populated by the camera. For the
Blackfly S the parameters look like this:

::

       'adjust_timestamp': True,
       'chunk_mode_active': True,
       'chunk_selector_timestamp': 'Timestamp',
       'chunk_enable_timestamp': True,



Network Configuration for GigE cameras
======================================

The Spinnaker SDK abstracts away the transport layer so a GigE camera
should work the same way as USB3: you point it to the serial number and
you're set.

There are a few GigE-specific settings in the Transport Layer Control
group that are important, in particular enabling jumbo frames from the
camera per FLIR’s recommendations. The following line in your
camera-specific config file will create a ROS2 parameter
``gev_scps_packet_size``:

::

   gev_scps_packet_size int "TransportLayerControl/GigEVision/GevSCPSPacketSize"

that you can then set in your ROS2 launch file:

::

    "gev_scps_packet_size": 9000

As far as setting up the camera’s IP address: you can set up DHCP on
your network or configure a static persistent IP using spinview in
“Transport Layer Control”>“GigE Vision”. Check the box for “Current IP
Configuration Persistent IP” first to enable it, then set your desired
addresses under “Persistent IP Address”, “Persistent Subnet Mask” and
“Persistent Gateway”. NOTE: these look like regular IPs, but to set them
you have to enter the 32-bit integer representation of the IP
address/mask. By hand/calculator: convert the IP octets from decimal to
hex, then combine them and convert to a 32-bit integer, ex: 192.168.0.1
-> 0xC0A80001 -> 3232235521.

The “Transport Layer Control”>“GigE Vision” section of spinview is also
where you’ll find that “SCPS Packet Size” setting, which you can change
when not capturing frames, and verify it works in spinview and without
needing to spin up a custom launch file to get started, though it helps,
and you’ll probably want one anyway to specify your camera’s serial
number.

If you do not set up DHCP or set a static IP, your camera will probably 
assign itself an IP address according to the Link-Local address scheme. 
The address will be 169.254.xxx.xxx, where the x's are randomly generated.
In your computer's network settings, change the IPv4 Method for the port
that connects to your camera to "Link-Local Only." (You could also set 
the IPv4 Method to "Manual" with an address of 169.254.100.1 and a 
subnet mask of 255.255.0.0).

For more tips on GigE setup look at FLIR’s support pages
`here <https://www.flir.com/support-center/iis/machine-vision/knowledge-base/lost-ethernet-data-packets-on-linux-systems/>`__
and
`here <https://www.flir.com/support-center/iis/machine-vision/application-note/troubleshooting-image-consistency-errors/>`__.


How to develop your own camera configuration file
=================================================

The camera configuration file defines the available ROS parameters, and
how they relate to the corresponding `Spinnaker
nodes <https://www.flir.com/support-center/iis/machine-vision/application-note/spinnaker-nodes/>`__.
The Spinnaker API follows the GenICam standard, where each property
(e.g. exposure mode, gain, …) of the camera is represented by a node.
Many properties are of integer or floating point type, but some are
enumerations (“enum”). Before you modify a configuration file you can
explore the Spinnaker Nodes by using the spinview applications that
comes with the Spinnaker SDK. Once you know what property you want to
expose as a ROS parameter, you add a mapping entry to the yaml
configuration file, e.g.:

.. code::

     - name: image_width
       type: int
       node: ImageFormatControl/Width

With this entry in place, the ROS driver will now accept an integer
parameter of name ``image_width``, and whenever ``image_width`` changes,
it will apply this change to the Spinnaker Node
``ImageFormatControl/Width``.

Enumerations (``enum``) parameters are slightly trickier than float and
integers because their values are restricted to a set of strings. Any
other strings will be rejected by the Spinnaker API. Please document the
valid enum strings in the configuration file, e.g.:

.. code::

     - name: line1_linemode  # valid values: "Input", "Output"
       type: enum
       node: DigitalIOControl/LineMode

The hard part is often finding the node name, in the last example
``"DigitalIOControl/LineMode"``. It usually follows by removing spaces 
from the ``spinview`` display names. However, in the SpinView GUI you 
can also see the true name by double-clicking on a node under the 
features tab and looking for "Name" (as opposed to "Display Name"). 
If that doesn't work, launch the driver with the ``dump_node_map`` 
parameter set to "True" and look at the output for inspiration.

**NOTE: !!!! THE ORDER OF PARAMETER DEFINITION MATTERS !!!!**

On node startup, the parameters will be declared and initialized in the
order listed in the yaml file. For instance you must list the enum
``exposure_auto`` before the float ``exposure_time`` because on startup,
``exposure_auto`` must first be set to ``Off`` before ``exposure_time``
can be set, or else the camera refuses to set the exposure time.

Known issues
============

1) If you run multiple drivers in separate nodes that all access USB
   based devices, starting a new driver will stop the image acquisition
   of currently running drivers. There is an ugly workaround for this
   currently implemented: if image delivery stops for more than
   ``acquisition_timeout`` seconds, the acquisition is restarted. This
   operation may not be thread safe so the driver already running could
   possibly crash. This issue can be avoided by running all drivers in
   the same address space with a composable node (see stereo launch file
   for example).

Troubleshooting/Common Issues
=============================

1) Driver doesn't find camera.
   This is usually due to incorrect permissions, missing udev files etc. Install the Spinnaker SDK
   and get SpinView to work.

2) Driver doesn't publish images and/or warns about incomplete images for GigE cameras

   .. code::

      rate [Hz] in  39.76 out   0.00 drop   0% INCOMPLETE 100%

   The reason for the incomplete images is usually that you are exceeding the network
   bandwidth, causing packets to be dropped such that incomplete frames arrive at the host.
   Check for the MTU on all network cards and switches to be 9000 (jumbo frames). Sometimes
   the MTU for switches has to be set higher. Also make sure the GigE camera has jumbo frames
   enabled, i.e. ``gev_scps_packet_size`` is set to 9000.

3) Driver reports dropped packages. This means the connected subscriber is not picking up fast enough.
   Check CPU utilization of subscribers and the available network bandwidth between driver and subscriber.

4) Image seems laggy when viewed. This is usually not a camera driver issue, but related to ROS2 RMW
   or the image viewer. Check CPU utilization on displaying host and network bandwidth.

5) Camera doesn't reach desired frame rate.
   First play around in SpinView to reproduce the problem there. For GigE cameras, check network bandwidth.
   Switch to Bayer images to reduce network bandwidth by a factor of three.
   Check your exposure time. The frame rate cannot exceed the inverse of the exposure time.


Setting up Linux without Spinnaker SDK
======================================

Only use these instructions if you did not install the Spinnaker SDK on
your machine.

1) Somewhere in your ``.bashrc`` file, set the following env variable:

.. code::

   export SPINNAKER_GENTL64_CTI=/opt/ros/${ROS_DISTRO}/lib/spinnaker-gentl/Spinnaker_GenTL.cti

2) Add the “flirimaging” group and make yourself a member of it

.. code::

   sudo addgroup flirimaging
   sudo usermod -a -G flirimaging ${USER}

3) Bump the usbfs memory limits

The following was taken from
`here <https://www.flir.com/support-center/iis/machine-vision/application-note/using-linux-with-usb-3.1/>`__.
Edit the file ``/etc/default/grub`` and change the line default to:

::

   GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"

Then

::

   sudo update-grub

If your system does not have ``/etc/default/grub``, create the file
``/etc/rc.local``, and change its permissions to ‘executable’. Then
write the following text to it:

::

   #!/bin/sh -e
   sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'

   exit 0

4) Setup udev rules

.. code::

   echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1e10", GROUP="flirimaging"' | sudo tee -a /etc/udev/rules.d/40-flir-spinnaker.rules
   echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1724", GROUP="flirimaging"' | sudo tee -a /etc/udev/rules.d/40-flir-spinnaker.rules
   sudo service udev restart
   sudo udevadm trigger

5) Logout and log back in (or better, reboot)

``sudo reboot``


How to contribute
=================

Please provide feedback if you cannot get your camera working or if the
code does not compile for you. Feedback is crucial for the software
development process. However, before opening issues on github first
verify that the problem is not present when using spinview.

Bug fixes and config files for new cameras are greatly appreciated.
Before submitting a pull request, run this to see if your commit passes
some basic lint tests:

::

   colcon test --packages-select spinnaker_camera_driver && colcon test-result --verbose


License
=======

This software is issued under the Apache License Version 2.0. The file
``cmake/TargetArch.cmake`` is released under a custom license (see file).

