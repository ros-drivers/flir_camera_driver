^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pointgrey_camera_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2022-03-28)
------------------

0.2.1 (2022-03-21)
------------------

0.2.0 (2022-03-11)
------------------
* Bump CMake version to avoid CMP0048 warning.
* Bumped flir_camera_description verison.
* Changes.
* Merge pull request `#91 <https://github.com/ros-drivers/flir_camera_driver/issues/91>`_ from luis-camero/noetic-devel
  ROS Industrial CI
* Removed launch and rviz folders from CMakeLists
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
* Contributors: Tony Baltovski, luis-camero

0.2.5 (2023-01-06)
------------------

0.2.4 (2023-01-06)
------------------

0.2.3 (2022-04-19)
------------------
* 0.2.2
* Changes.
* 0.2.1
* Changes.
* 0.2.0
* Changes.
* Bump CMake version to avoid CMP0048 warning.
* Bumped flir_camera_description verison.
* Changes.
* Removed launch and rviz folders from CMakeLists
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
* Contributors: Tony Baltovski, luis-camero

0.1.0 (2021-11-10)
-------------------
* Initial Release
* Contributors: Luis Camero

