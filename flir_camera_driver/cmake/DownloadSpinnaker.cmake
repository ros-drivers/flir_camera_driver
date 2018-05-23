function(download_spinnaker POINTGREY_LIB_VAR POINTGREY_INCLUDE_DIR_VAR)
  if(NOT UNIX)
    message(FATAL_ERROR "Downloading libSpinnaker for non-linux systems not supported")
  endif()

  include(cmake/TargetArch.cmake)
  target_architecture(POINTGREY_ARCH)
  message(STATUS "Running download_spinnaker script with arguments: ${POINTGREY_ARCH} ${CATKIN_DEVEL_PREFIX}/lib/pg_spinnaker_camera/ WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}")
  set(DOWNLOAD_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/download_spinnaker")
  execute_process(
    COMMAND ${DOWNLOAD_SCRIPT} ${POINTGREY_ARCH} "${CATKIN_DEVEL_PREFIX}/lib/pg_spinnaker_camera/"
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

  set(${POINTGREY_LIB_VAR} "${CMAKE_CURRENT_BINARY_DIR}/usr/lib/libSpinnaker.so" PARENT_SCOPE)
  set(${POINTGREY_INCLUDE_DIR_VAR} "${CMAKE_CURRENT_BINARY_DIR}/usr/include" PARENT_SCOPE)
endfunction()
