^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spinnaker_camera_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.3 (2025-07-14)
------------------
* fix compiler warnings
* add dependency on libomp-dev
* use spinnaker v4.2 and clean up
* Contributors: Bernd Pfrommer

3.0.2 (2025-05-25)
------------------
* avoid ament_target_dependencies
* fixed doc formatting
* PTP support for spinnaker_camera_driver
* added reverse x/y for blackfly s
* Contributors: Bernd Pfrommer

3.0.1 (2025-04-01)
------------------
* do not use user_set_selector for blackfly_s in launch file
* document software trigger
* Contributors: Bernd Pfrommer

3.0.0 (2024-11-21)
------------------
* Remove "NOT TESTED" from tested Chameleon params
  - Tested with `CM3-U3-50S5C` camera model, `trigger_source: "Line0"`
  and `trigger_selector: "FrameStart"`
* Re-order Chameleon config params to match Blackfly
  - Match Chameleon param order with Blackfly config params
* Add Reverse and VideoMode Chameleon config params
  - ReverseX/Y params flip the image as expected. This changes the pixel
  format accordingly (e.g. BayerRG8 -> BayerBG8 for an X and Y flip)
  - Video Mode options can affect effective resolution, frame rate and
  brightness depending on camera model, please check the documentation.
* support brightness computation for all bayer image types
* better documentation for setting GENTL variable
* only access camera with correct serial
* better diagnostics on incomplete images
* work around compile error on Iron
* fix multiple network interface refreshCameraList
* add support for transport layer and stream params
* added option to launch blackfly type
* mention SPINNAKER_GENTL64_CTI
* updated docs for jazzy, adjust download script
* remove unnecessary debs from package
* point to new spinnaker sdk for noble
* renamed stereo_synced file and added doc
* added user set control examples for blackfly/blackfly_s
* fix broken composable node by installing in correct location
* Add FLIR-AX5 Camera (`#176 <https://github.com/ros-drivers/flir_camera_driver/issues/176>`_)
  * added config file for flir_ax5
  ---------
  Co-authored-by: Bernd Pfrommer <bernd.pfrommer@gmail.com>
* add option to disable external control (default!)
* updated docs for sync driver, switch to RST
* widened the ExposureController interface
* fix build errors on rolling/noble
* added blacklevel and whitebalance support for blackfly
* use proper name for camerainfo when using sync driver
* fixes to compile on focal/galactic
* Oryx parameter file
* lint
* allow for unreadable command nodes
* Initial support for command nodes
* remove more spinnaker imports
* make Spinnaker private
* added blackfly GigE configuration file
* track incomplete frames
* fixed licensing documentation
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
* Contributors: Alejandro Bordallo, Bernd Pfrommer, Luis Camero, Sir-Photch, anonymousarmadillo100, buckleytoby, iagogomes

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
* Hardcoding OS to jammy since it is the only one currently supported.
* Contributors: Tony Baltovski

2.0.2 (2023-07-28)
------------------
* Changes.
* replace lsb-release with python3-distro
* add dependencies for spinnaker download
* Contributors: Bernd Pfrommer, Tony Baltovski

2.0.1 (2023-07-24)
------------------
* Changes.
* use cmake find_program to detect lsb_release
* Contributors: Bernd Pfrommer, Tony Baltovski

2.0.0 (2023-07-20)
------------------
* Changes.
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
* Only install necessary libraries
* Contributors: Luis Camero, Tony Baltovski

0.2.2 (2022-03-28)
------------------
* Changes.
* Added new-line at EOF
* Spinnaker libraries are now all copied to usr/lib
* Reordered definitions to prevent compiler warnings
* Contributors: Luis Camero, Tony Baltovski

0.2.1 (2022-03-21)
------------------
* Changes.
* Removed check for build/usr/lib which would cause build to skip Spinnaker SDK install
* Contributors: Luis Camero, Tony Baltovski

0.2.0 (2022-03-11)
------------------
* Changes.
* Changes.
* Fixed all issues reported by roslint
* Updated file paths to /opt/spinnaker instead of /usr/spinnaker
* Updated download_spinnaker look-up table
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
