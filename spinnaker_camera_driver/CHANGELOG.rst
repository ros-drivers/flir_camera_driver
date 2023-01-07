^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spinnaker_camera_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2022-03-28)
------------------
* Added new-line at EOF
* Spinnaker libraries are now all copied to usr/lib
* Reordered definitions to prevent compiler warnings
* Contributors: Luis Camero

0.2.1 (2022-03-21)
------------------
* Removed check for build/usr/lib which would cause build to skip Spinnaker SDK install
* Contributors: Luis Camero

0.2.0 (2022-03-11)
------------------
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

0.2.5 (2023-01-06)
------------------
* Fixed arm64 folder name.
* Contributors: Tony Baltovski

0.2.4 (2023-01-06)
------------------
* Fixed typo in arm64 arch.
* Contributors: Tony Baltovski

0.2.3 (2022-04-19)
------------------
* Only install necessary libraries
* 0.2.2
* Changes.
* Added new-line at EOF
* Spinnaker libraries are now all copied to usr/lib
* Reordered definitions to prevent compiler warnings
* 0.2.1
* Changes.
* Removed check for build/usr/lib which would cause build to skip Spinnaker SDK install
* 0.2.0
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
* Add ARM Build Support (`#3 <https://github.com/ros-drivers/flir_camera_driver/issues/3>`_)
  * Added ARM Build Support.
* Contributors: Michael Hosmar

0.1.1 (2018-07-25)
------------------
* Add opencv3 as build dependency.
* Contributors: Michael Hosmar

0.1.0 (2018-07-24)
------------------
* Initial Release
* Contributors: Michael Hosmar
