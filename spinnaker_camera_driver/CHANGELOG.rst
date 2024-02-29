^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spinnaker_camera_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.11 (2024-02-29)
-------------------
* provision camera driver for exposure control
* fixed bugs discovered when running on GigE cams
* avoid searching ROS path for library
* added connect_while_subscribed feature
* Added binning parameter
* install spinnaker library in same place as driver library
* remove junk directories from search path
* added first implementation of synchronized driver
* prepare single-camera driver for use with sync'ed driver
* fixed stereo launch file serial nb bug
* removed changelogs
* Contributors: Bernd Pfrommer, Luis Camero, buckleytoby

2.0.8 (2023-11-14)
------------------
* Changes.
* Added linux_setup_flir script instructions to Readme
* Add newline echo before Done
* Rename script to remove extension
* Ask permission for usb change and don't limit detection to 1000
* Ask about usergroup and give feedback
* Added linux pc setup script
* fix python formatting to satisfy linter
* fix formatting of BSD license to satisfy linter
* Contributors: Bernd Pfrommer, Hilary Luo, Tony Baltovski

2.0.7 (2023-10-03)
------------------
* Changes.
* Merge pull request `#132 <https://github.com/ros-drivers/flir_camera_driver/issues/132>`_ from hilary-luo/humble-devel
  Added udev rule
* Restricted the device permissions
* Added Teledyne to udev as requested
* Added udev rule
* Contributors: Hilary Luo, Tony Baltovski

2.0.6 (2023-08-12)
------------------
* updated changelog
* use correct file name when downloading spinnaker from clearpath web site
* resurrect building when the Spinnaker SDK is present
* also compile with older version (0.6) of yaml library
* git clone needs --branch humble-devel
* Contributors: Bernd Pfrommer

2.0.5 (2023-08-11)
------------------
* updated changelog
* add ffmpeg depency to fix build failures on ROS farm
* reference correct yaml dependency in rosdep
* switch from config files to standard yaml format
* Contributors: Bernd Pfrommer

2.0.4 (2023-08-10)
------------------
* updated changelogs
* install spinnaker libraries in spinnaker_camera_driver dir
* Contributors: Bernd Pfrommer

2.0.3 (2023-08-01)
------------------
* Changes.
* Merge pull request `#119 <https://github.com/ros-drivers/flir_camera_driver/issues/119>`_ from ros-drivers/fix/tmp-hardcode-os
  Hardcoding OS to jammy since it is the only one currently supported.
* Hardcoding OS to jammy since it is the only one currently supported.
* Contributors: Tony Baltovski

2.0.2 (2023-07-28)
------------------
* Changes.
* Merge pull request `#117 <https://github.com/ros-drivers/flir_camera_driver/issues/117>`_ from ros-drivers/humble-devel-fix-lsb-release
  add dependencies for spinnaker download.
  @tonybaltovski can you release this fix? I'd like to see if it works. Thanks!
* replace lsb-release with python3-distro
* add dependencies for spinnaker download
* Contributors: Bernd Pfrommer, Tony Baltovski

2.0.1 (2023-07-24)
------------------
* Changes.
* Merge pull request `#116 <https://github.com/ros-drivers/flir_camera_driver/issues/116>`_ from ros-drivers/humble-devel-fix-lsb-release
  use cmake find_program to detect lsb_release
* use cmake find_program to detect lsb_release
* Contributors: Bernd Pfrommer, Tony Baltovski

2.0.0 (2023-07-20)
------------------
* Changes.
* Merge pull request `#113 <https://github.com/ros-drivers/flir_camera_driver/issues/113>`_ from berndpfrommer/humble-devel-new
  new driver for ROS2
* added spinnaker_camera_driver package
* deleted spinnaker ros2 driver, to be replaced by new version
* Contributors: Bernd Pfrommer, Tony Baltovski

0.2.5 (2023-01-06 20:18)
------------------------
* Changes.
* Fixed arm64 folder name.
* Contributors: Tony Baltovski

0.2.4 (2023-01-06 11:43)
------------------------
* Changes.
* Fixed typo in arm64 arch.
* Contributors: Tony Baltovski

0.2.3 (2022-04-19)
------------------
* Changes.
* Merge pull request `#96 <https://github.com/ros-drivers/flir_camera_driver/issues/96>`_ from luis-camero/noetic-devel
  Only copy necessary libraries
* Merge branch 'ros-drivers:noetic-devel' into noetic-devel
* Only install necessary libraries
* Contributors: Luis Camero, Tony Baltovski, luis-camero

0.2.2 (2022-03-28)
------------------
* Changes.
* Merge pull request `#94 <https://github.com/ros-drivers/flir_camera_driver/issues/94>`_ from luis-camero/noetic-devel
  Copy Spinnaker Libraries to build/usr/lib
* Added new-line at EOF
* Spinnaker libraries are now all copied to usr/lib
* Reordered definitions to prevent compiler warnings
* Contributors: Luis Camero, Tony Baltovski

0.2.1 (2022-03-21)
------------------
* Changes.
* Merge pull request `#93 <https://github.com/ros-drivers/flir_camera_driver/issues/93>`_ from luis-camero/noetic-devel
  Removed check for build/usr/lib
* Removed check for build/usr/lib which would cause build to skip Spinnaker SDK install
* Contributors: Luis Camero, Tony Baltovski

0.2.0 (2022-03-11)
------------------
* Changes.
* Changes.
* Merge pull request `#91 <https://github.com/ros-drivers/flir_camera_driver/issues/91>`_ from luis-camero/noetic-devel
  ROS Industrial CI
* Fixed all issues reported by roslint
* Updated file paths to /opt/spinnaker instead of /usr/spinnaker
* Updated download_spinnaker look-up table
* Merge pull request `#88 <https://github.com/ros-drivers/flir_camera_driver/issues/88>`_ from luis-camero/noetic-devel
  Add readable check to SDK parameters
* Add readable check to SDK parameters
* URDF Description, Diagnostics, ISP Enable, and Launch Files (`#81 <https://github.com/ros-drivers/flir_camera_driver/issues/81>`_)
  * Changes required to use GigE Blackfly S version
  * Added blackfly mesh
  * Added URDF of blackflys and CHANGELOG
  * Added new_line at end of flir_blackflys.urdf.xacro
  * Added DiagnosticAnalyzers and more detailed diagnostic messages
  * Added ISP enable and disable config and updated camera launch file to be more descriptive
  * Switched order of configuration to put ISP enable next to color encoding
  * Updated config to include enumeration for Off, Once, Continuous parameters, and udpated diagnostics.launch
  * Handled issue where no namespace prevents diagnostics_agg from loading from analyzer paramaters
* Branch to Support GigE Cameras (`#79 <https://github.com/ros-drivers/flir_camera_driver/issues/79>`_)
  * Changes required to use GigE Blackfly S version
  * Update SpinnakerCamera.cpp
* Add new parameter to apply an offset to image time stamps (`#56 <https://github.com/ros-drivers/flir_camera_driver/issues/56>`_)
* Fixes SpinnakerCamera teardown (`#16 <https://github.com/ros-drivers/flir_camera_driver/issues/16>`_)
  * fixes error on destroying SpinnakerCamera with multiple cameras
  * adds clarifying comment
* Add /opt/spinnaker to spinnaker discovery options (`#63 <https://github.com/ros-drivers/flir_camera_driver/issues/63>`_)
* increase maximum value of exposure_time/auto_exposure_time_upper_limit (`#55 <https://github.com/ros-drivers/flir_camera_driver/issues/55>`_)
* add option to set queue_size for ros publisher (`#54 <https://github.com/ros-drivers/flir_camera_driver/issues/54>`_)
* Added support for Grasshopper3. Identical to Chameleon3, split into separate files for clarity. (`#26 <https://github.com/ros-drivers/flir_camera_driver/issues/26>`_)
* Feature: horizontal and vertical image reverse (`#41 <https://github.com/ros-drivers/flir_camera_driver/issues/41>`_)
  * Add horizontal/vertical inverse to reconfigure cfg
  * Add ReverseX/ReverseY with setProperty
  Co-authored-by: Fabian Schilling <fabian.schilling@me.com>
* Update Spinnaker.cfg (`#50 <https://github.com/ros-drivers/flir_camera_driver/issues/50>`_)
  Fix for correct spelling with capital letter for bool type
* Add auto exposure ROI parameters (`#52 <https://github.com/ros-drivers/flir_camera_driver/issues/52>`_)
  * spinnaker_camera_driver: setProperty: report available enum values
  Only done on failure. This helps to figure out which enum values are
  available on a particular camera model.
  * spinnaker_camera_driver: expose AE ROI parameters
  This is highly useful when using fisheye lenses, which illuminate only
  a circle in the center of the image. The AE gets confused by the black
  regions around it and overexposes the image.
  This also exposes the "AutoExposureLightingMode" parameter, which allows
  the user to choose a lighting preset (front/back/normal).
* Fix/frame rate params (`#20 <https://github.com/ros-drivers/flir_camera_driver/issues/20>`_)
  * [spinnaker_camera_driver] Fixed naming of frame rate control params
  * [spinnaker_camera_driver] Format of mono and stereo launchfiles
  * [spinnaker_camera_driver] Updated diagnostics launchfile
* Removed opencv as depend. (`#46 <https://github.com/ros-drivers/flir_camera_driver/issues/46>`_)
* Changed the download script to check for destination folder and moved unpack directory. (`#44 <https://github.com/ros-drivers/flir_camera_driver/issues/44>`_)
* Merge pull request `#42 <https://github.com/ros-drivers/flir_camera_driver/issues/42>`_ from civerachb-cpr/rpsw-185
  Fix Flycap & Spinnaker endpoints
* Create the directory if it doesn't exist
* Remove an unnecessary deb
* Spinnaker driver now successfully downloads & builds
* Start overhauling the spinnaker download script so it works with the correct endpoint & matches the general structure of the pointgrey_camera_driver
* Contributors: Adam Romlein, Chris I-B, Evan Bretl, Fabian Schilling, Ferdinand, Joseph Curtis, Luis Camero, Max Schwarz, Stephan, Tony Baltovski, Yoshua Nava, Yuki Furuta, luis-camero

0.1.3 (2018-09-25)
------------------
* Update Changelog.
* Fix install targets when Spinnaker is installed locally. Tabs in FindSpinnaker.
* Add missing target (Cm3) and switch to find_package script. (`#11 <https://github.com/ros-drivers/flir_camera_driver/issues/11>`_)
  * Add missing target (Cm3) and switch to find_package script.
  * Clean up message.
* Adding support of feeding some camera diagnostics to the diagnostic a… (`#4 <https://github.com/ros-drivers/flir_camera_driver/issues/4>`_)
  * Adding support of feeding some camera diagnostics to the diagnostic aggregator
  * Creating a seperate diagnostics launch example
* Fix null conversion and unsigned comparison Warnings.
* Contributors: Helen Oleynikova, Michael Hosmar, mlowe-ascent

0.1.2 (2018-07-27)
------------------
* Update Changelog.
* Add ARM Build Support (`#3 <https://github.com/ros-drivers/flir_camera_driver/issues/3>`_)
  * Added ARM Build Support.
* Contributors: Michael Hosmar

0.1.1 (2018-07-25)
------------------
* Update Changelog.
* Add opencv3 as build dependency.
* Contributors: Michael Hosmar

0.1.0 (2018-07-24)
------------------
* Add Changelog
* Change TODO's to me.
* Flir = Spinnaker
* Add timeout and fix reconnection. Replace Pointgrey references from e4b1493. Changed some prints away from "Once".
* Move to std::shared_ptr and removed unnecessary install directive.
* Remove old changelog.
* Remove unnecessary config files.
* line length.
* flir_camera_driver = spinnaker_camera_driver
* Contributors: Michael Hosmar
