#  Copyright 2023, Clearpath Robotics, Inc., All rights reserved.
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

function(download_spinnaker FLIR_LIB_VAR FLIR_INCLUDE_DIR_VAR)
  if(NOT UNIX)
    message(FATAL_ERROR "Downloading libSpinnaker for non-linux systems not supported")
  endif()

  #find_program(LSB_RELEASE_EXEC lsb_release REQUIRED)
  execute_process(
    #COMMAND ${LSB_RELEASE_EXEC} -cs
    COMMAND /usr/bin/echo jammy
    OUTPUT_VARIABLE OS_CODE_NAME
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  include(cmake/TargetArch.cmake)
  target_architecture(FLIR_ARCH)
  set(FLIR_DIR ${CMAKE_CURRENT_BINARY_DIR}/usr/lib)
  set(DOWNLOAD_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/download_spinnaker")

  message(STATUS "Running download_spinnaker script with arguments: ${FLIR_ARCH} ${FLIR_DIR} ${OS_CODE_NAME}")
  execute_process(
    COMMAND ${DOWNLOAD_SCRIPT} ${FLIR_ARCH} "${FLIR_DIR}" ${OS_CODE_NAME} OUTPUT_VARIABLE RET)
  message(STATUS "PROCESS RESULT: ${RET}")
  set(${FLIR_LIB_VAR} "${CMAKE_BINARY_DIR}/usr/lib/libSpinnaker.so" PARENT_SCOPE)
  set(${FLIR_INCLUDE_DIR_VAR} "${CMAKE_BINARY_DIR}/opt/spinnaker/include" PARENT_SCOPE)
endfunction()
