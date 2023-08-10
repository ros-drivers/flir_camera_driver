^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spinnaker_camera_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.4 (2023-08-10)
------------------
* install spinnaker libraries in spinnaker_camera_driver dir
* Contributors: Bernd Pfrommer

2.0.3 (2023-08-01)
------------------
* Hardcoding OS to jammy since it is the only one currently supported.
* Contributors: Tony Baltovski

2.0.2 (2023-07-28)
------------------
* replace lsb-release with python3-distro
* add dependencies for spinnaker download
* Contributors: Bernd Pfrommer

2.0.1 (2023-07-24)
------------------
* use cmake find_program to detect lsb_release
* Contributors: Bernd Pfrommer

2.0.0 (2023-07-20)
------------------
* Merge pull request `#113 <https://github.com/ros-drivers/flir_camera_driver/issues/113>`_ from berndpfrommer/humble-devel-new
  new driver for ROS2
* added spinnaker_camera_driver package
* deleted spinnaker ros2 driver, to be replaced by new version
* Contributors: Bernd Pfrommer, Tony Baltovski
