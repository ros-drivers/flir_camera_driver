#  Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#   limitations under the License.

#
# sets the following variables:
#
# SPINNAKER_INCLUDE_DIRS
# SPINNAKER_LIBRARIES
# SPINNAKER_FOUND
#
# or for the more modern cmake usage, sets the target Spinnaker::Spinnaker
#
#
# searches first in "SPINNAKER_ROOT_DIR" for location of spinnaker SDK

#
include(FindPackageHandleStandardArgs)

if( EXISTS "$ENV{SPINNAKER_ROOT_DIR}" )
  file( TO_CMAKE_PATH "$ENV{SPINNAKER_ROOT_DIR}" SPINNAKER_ROOT_DIR )
  set( SPINNAKER_ROOT_DIR "${SPINNAKER_ROOT_DIR}" CACHE PATH "Prefix for Spinnaker installation." )
endif()

find_path(SPINNAKER_INCLUDE_DIRS
  NAMES Spinnaker.h
  HINTS
  ${SPINNAKER_ROOT_DIR}/include
  /opt/spinnaker/include
  /usr/include/spinnaker
  /usr/local/include/spinnaker
  /__w/flir_ros2_camera_driver/flir_ros2_camera_driver/ros_ws/build/flir_spinnaker_common/opt/spinnaker/include
)

find_library(SPINNAKER_LIBRARIES
  NAMES Spinnaker
  HINTS
  ${SPINNAKER_ROOT_DIR}/lib
  /opt/spinnaker/lib
  /usr/lib/
  /usr/local/lib
  /__w/flir_ros2_camera_driver/flir_ros2_camera_driver/ros_ws/build/flir_spinnaker_common/usr/lib
  PATH_SUFFIXES Release Debug
)

set(SPINNAKER_INCLUDE_DIRS ${SPINNAKER_INCLUDE_DIRS})
set(SPINNAKER_LIBRARIES ${SPINNAKER_LIBRARIES})
message(STATUS "Found inc dir: ${SPINNAKER_INCLUDE_DIRS}")

find_package_handle_standard_args(SPINNAKER
  FOUND_VAR SPINNAKER_FOUND
  REQUIRED_VARS SPINNAKER_INCLUDE_DIRS SPINNAKER_LIBRARIES)


if(SPINNAKER_FOUND AND NOT TARGET Spinnaker::Spinnaker)
  add_library(Spinnaker::Spinnaker UNKNOWN IMPORTED)
  set_target_properties(Spinnaker::Spinnaker PROPERTIES
    IMPORTED_LOCATION                 "${SPINNAKER_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES     "${SPINNAKER_INCLUDE_DIRS}"
    IMPORTED_LINK_INTERFACE_LANGUAGES "CXX")
endif()
