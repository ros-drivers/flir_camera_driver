#  ----------------------------------------------------------------------------
#  2020 Bernd Pfrommer bernd.pfrommer@gmail.com
#

cmake_minimum_required(VERSION 3.5)
project(flir_spinnaker_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
# list of all packages that use ament for export (may work for others too)

set(ROS2_DEPENDENCIES
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs"
  "camera_info_manager"
  "image_transport"
  "flir_spinnaker_common"
  "image_meta_msgs_ros2"
  "camera_control_msgs_ros2"
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)

foreach(pkg ${ROS2_DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()


ament_environment_hooks(findshared.sh.in)

ament_auto_find_build_dependencies(REQUIRED ${ROS2_DEPENDENCIES})

ament_auto_add_library(camera_driver SHARED
  src/camera_driver.cpp
)

ament_auto_add_executable(camera_driver_node
  src/camera_driver_node.cpp
)

rclcpp_components_register_nodes(camera_driver "flir_spinnaker_ros2::CameraDriver")


target_include_directories(camera_driver PRIVATE include)


# the node must go into the project specific lib directory or else
# the launch file will not find it

install(TARGETS
  camera_driver_node
  DESTINATION lib/${PROJECT_NAME}/)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(TARGETS
  camera_driver
  DESTINATION lib
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_flake8()
  ament_lint_cmake()
  ament_pep257()
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_xmllint()
endif()

ament_package()
