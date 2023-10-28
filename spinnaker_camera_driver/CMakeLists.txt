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

cmake_minimum_required(VERSION 3.5)
project(spinnaker_camera_driver)

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

# the spinnaker SDK does not provide a cmake file
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# If Spinnacker is already present, use the found version. If not, download it.
# We can't resolve this dependency using the usual rosdep means because
# the Point Grey EULA prohibits redistributing the headers or the packages which
# contains them. We work around this by downloading the archive directly from
# their website during this step in the build process.

# Spinnaker does not provide a cmake discovery mechanism so we
# provide a find script in the cmake path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(SPINNAKER QUIET)
message(STATUS
  "Found variable: ${SPINNAKER_FOUND} at ${SPINNAKER_LIBRARIES} and ${SPINNAKER_INCLUDE_DIRS}")

set(SPINNAKER_WAS_DOWNLOADED FALSE)
if(NOT SPINNAKER_FOUND)
  message(STATUS "spinnaker not found")
  message(STATUS "libSpinnaker not found in system library path")
  include(cmake/DownloadSpinnaker.cmake)
  download_spinnaker(SPINNAKER_LIBRARIES SPINNAKER_INCLUDE_DIRS)
  set(SPINNAKER_WAS_DOWNLOADED TRUE)
else()
  message(STATUS "spinnaker SDK found installed")
endif()

message(STATUS "libSpinnaker library location: ${SPINNAKER_LIBRARIES}")
message(STATUS "libSpinnaker include location: ${SPINNAKER_INCLUDE_DIRS}")

find_package(SPINNAKER REQUIRED)

include_directories(SYSTEM
  ${SPINNAKER_INCLUDE_DIRS})

# list of all packages that use ament for export (may work for others too)

set(ROS_DEPENDENCIES
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs"
  "camera_info_manager"
  "image_transport"
  "flir_camera_msgs")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

foreach(pkg ${ROS_DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()
find_package(yaml-cpp REQUIRED)

#
# shared library for composable node
#

add_library(camera_driver SHARED
  src/camera_driver.cpp
  src/spinnaker_wrapper.cpp
  src/spinnaker_wrapper_impl.cpp
  src/image.cpp
  src/pixel_format.cpp
  src/genicam_utils.cpp
)

ament_target_dependencies(camera_driver PUBLIC ${ROS_DEPENDENCIES})
target_link_libraries(camera_driver PUBLIC Spinnaker::Spinnaker PRIVATE yaml-cpp)

target_include_directories(camera_driver
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_nodes(camera_driver "spinnaker_camera_driver::CameraDriver")

#
# the driver node uses the shared library
#
add_executable(camera_driver_node
  src/camera_driver_node.cpp)

target_link_libraries(camera_driver_node camera_driver)

target_include_directories(camera_driver_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

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

install(PROGRAMS
  scripts/linux_setup_flir
  DESTINATION lib/${PROJECT_NAME}
)

if(SPINNAKER_WAS_DOWNLOADED)
  set(SPINNAKER_BUILD_SUBDIR "opt/spinnaker/lib")
  set(SPINNAKER_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/${SPINNAKER_BUILD_SUBDIR}")
  message(STATUS "installing spinnaker libraries from ${SPINNAKER_BUILD_DIR}")
  install(DIRECTORY  "${SPINNAKER_BUILD_DIR}" DESTINATION ${CMAKE_INSTALL_PREFIX})
endif()

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
