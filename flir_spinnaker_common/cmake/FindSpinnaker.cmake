#  Copyright 2020 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

if( EXISTS "$ENV{Spinnaker_ROOT_DIR}" )
  file( TO_CMAKE_PATH "$ENV{Spinnaker_ROOT_DIR}" Spinnaker_ROOT_DIR )
  set( Spinnaker_ROOT_DIR "${Spinnaker_ROOT_DIR}" CACHE PATH "Prefix for Spinnaker installation." )
endif()

cmake_path(SET PATH2 "${CMAKE_CURRENT_BINARY_DIR}")
cmake_path(GET PATH2 PARENT_PATH PARENTDIR )

find_path(Spinnaker_INCLUDE_DIRS
  NAMES Spinnaker.h
  HINTS
  ${Spinnaker_ROOT_DIR}/include
  /opt/spinnaker/include
  /usr/include/spinnaker
  /usr/local/include/spinnaker
  /__w/flir_ros2_camera_driver/flir_ros2_camera_driver/ros_ws/build/flir_spinnaker_common/opt/spinnaker/include
)

find_library(Spinnaker_LIBRARIES
  NAMES Spinnaker
  HINTS
  ${Spinnaker_ROOT_DIR}/lib
  /opt/spinnaker/lib
  /usr/lib/
  /usr/local/lib
  /__w/flir_ros2_camera_driver/flir_ros2_camera_driver/ros_ws/build/flir_spinnaker_common/usr/lib
  PATH_SUFFIXES Release Debug
)

set(Spinnaker_INCLUDE_DIRS ${Spinnaker_INCLUDE_DIRS})
set(Spinnaker_LIBRARIES ${Spinnaker_LIBRARIES})

find_package_handle_standard_args(Spinnaker
  FOUND_VAR Spinnaker_FOUND
  REQUIRED_VARS Spinnaker_INCLUDE_DIRS Spinnaker_LIBRARIES)


if(Spinnaker_FOUND AND NOT TARGET Spinnaker::Spinnaker)
  add_library(Spinnaker::Spinnaker UNKNOWN IMPORTED)
  set_target_properties(Spinnaker::Spinnaker PROPERTIES
    IMPORTED_LOCATION                 "${Spinnaker_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES     "${Spinnaker_INCLUDE_DIRS}"
    IMPORTED_LINK_INTERFACE_LANGUAGES "CXX")
endif()
