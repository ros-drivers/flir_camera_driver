^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pointgrey_camera_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2022-11-22)
------------------
* Fixed package version of flir_camera_description to match the rest.
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

