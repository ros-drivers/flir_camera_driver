# -*- cmake -*-
#
# Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.20)
project(spinnaker_synchronized_camera_driver)

include(CMakePrintHelpers)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# The spinnaker_camera_driver links against the spinnaker libraries,
# and also exports them as a dependency. However, if the spinnaker SDK
# is not installed system-wide, then the spinnaker_camera_driver actually
# installs a private copy of the spinnaker SDK libraries alongside its own
# libraries. The hack below works detects this scenario and works around it.

find_package(SPINNAKER)
if(NOT SPINNAKER_FOUND)
  # assumes that "spinnaker_camera_driver" is installed at the same root
  # as "spinnaker_synchronized_camera_driver"
  cmake_path(GET CMAKE_INSTALL_FULL_LIBDIR PARENT_PATH SPINNAKER_ROOT_DIR)
  cmake_path(GET SPINNAKER_ROOT_DIR PARENT_PATH SPINNAKER_ROOT_DIR)
  cmake_path(APPEND SPINNAKER_ROOT_DIR "spinnaker_camera_driver")
  message(STATUS "searching for spinnaker here: ${SPINNAKER_ROOT_DIR}")
  # SPINNAKER_ROOT_DIR will be actually used when
  # find_package(spinnaker_camera_driver) executes a find_package(SPINNAKER).
  # This find_package(SPINNAKER) is actually triggered by
  # spinnaker_camera_driverConfig.cmake.in
  # But the FindSPINNAKER.cmake used for that search is actually the one
  # in spinnaker_synchronized_camera_driver/cmake/,
  # which has been stripped to not search for the include path.
endif()

find_package(spinnaker_camera_driver REQUIRED)

# list of all packages that use ament for export (may work for others too)

set(ROS_DEPENDENCIES
  "rclcpp"
  "rclcpp_components")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

foreach(pkg ${ROS_DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

#
# shared library for composable node
#

cmake_print_properties(TARGETS spinnaker_camera_driver::camera_driver PROPERTIES INTERFACE_INCLUDE_DIRECTORIES)

add_library(synchronized_camera_driver SHARED
  src/synchronized_camera_driver.cpp
  src/time_estimator.cpp
  src/time_keeper.cpp
  src/exposure_controller_factory.cpp
  src/individual_exposure_controller.cpp)

ament_target_dependencies(synchronized_camera_driver PUBLIC ${ROS_DEPENDENCIES})
target_link_libraries(synchronized_camera_driver PUBLIC spinnaker_camera_driver::camera_driver PRIVATE yaml-cpp)

target_include_directories(synchronized_camera_driver
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(synchronized_camera_driver PRIVATE include)

rclcpp_components_register_nodes(synchronized_camera_driver "spinnaker_synchronized_camera_driver::SynchronizedCameraDriver")

#
# the driver node uses the shared library
#
add_executable(synchronized_camera_driver_node
  src/synchronized_camera_driver_node.cpp)

target_link_libraries(synchronized_camera_driver_node synchronized_camera_driver)

target_include_directories(synchronized_camera_driver_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# the node must go into the project specific lib directory or else
# the launch file will not find it

install(TARGETS
  synchronized_camera_driver_node
  DESTINATION lib/${PROJECT_NAME}/)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(TARGETS
  synchronized_camera_driver
  DESTINATION lib
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
  find_package(ament_cmake_black REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_black()
  ament_lint_cmake()
  ament_pep257()
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_xmllint()
endif()

ament_package()
