^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spinnaker_camera_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.13 (2024-03-13)
-------------------
* added blackfly GigE configuration file
* Contributors: Bernd Pfrommer

2.2.12 (2024-03-07)
-------------------
* track incomplete frames
* fixed licensing documentation
* provision camera driver for exposure control
* fixed bugs discovered when running on GigE cams
* avoid searching ROS path for library
* Contributors: Bernd Pfrommer

2.2.11 (2024-02-23)
-------------------
* added connect_while_subscribed feature
* Added binning parameter
* install spinnaker library in same place as driver library
* remove junk directories from search path
* prepare single-camera driver for use with sync'ed driver
* fixed stereo launch file serial nb bug
* Contributors: Bernd Pfrommer, Luis Camero, buckleytoby

2.2.10 (2023-12-02)
-------------------

2.0.8 (2023-11-14)
------------------
* Added linux_setup_flir script instructions to Readme
* Add newline echo before Done
* Rename script to remove extension
* Ask permission for usb change and don't limit detection to 1000
* Ask about usergroup and give feedback
* Added linux pc setup script
* fix python formatting to satisfy linter
* fix formatting of BSD license to satisfy linter
* Contributors: Bernd Pfrommer, Hilary Luo

2.0.7 (2023-10-03)
------------------
* Restricted the device permissions
* Added Teledyne to udev as requested
* Added udev rule
* Contributors: Hilary Luo

2.0.6 (2023-08-12)
------------------
* fix arm64 build: use correct file name when downloading spinnaker from clearpath
* fix broken build when the Spinnaker SDK is present
* allow building with older version (0.6) of yaml library
* Contributors: Bernd Pfrommer

2.0.5 (2023-08-11)
------------------
* add ffmpeg dependency to fix build failures on ROS farm
* switch from custom config files to standard yaml format
* Contributors: Bernd Pfrommer

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
