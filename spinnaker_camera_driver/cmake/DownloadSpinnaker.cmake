function(download_spinnaker FLIR_LIB_VAR FLIR_INCLUDE_DIR_VAR)
  if(NOT UNIX)
    message(FATAL_ERROR "Downloading libSpinnaker for non-linux systems not supported")
  endif()

  execute_process(
    COMMAND lsb_release -cs
    OUTPUT_VARIABLE OS_CODE_NAME
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  include(cmake/TargetArch.cmake)
  target_architecture(TARGET_ARCH)
  set(DOWNLOAD_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/download_spinnaker")

  set(SPINNAKER_SDK_DIR ${CMAKE_CURRENT_BINARY_DIR}/spinnaker-sdk)

  message(STATUS "Running download_spinnaker script with arguments: ${TARGET_ARCH} ${OS_CODE_NAME} ${SPINNAKER_SDK_DIR}")
  execute_process(
    COMMAND ${DOWNLOAD_SCRIPT} ${TARGET_ARCH} ${FLIR_DIR} ${OS_CODE_NAME} ${SPINNAKER_SDK_DIR}
    RESULT_VARIABLE DOWNLOAD_EXIT_CODE)
  if(NOT DOWNLOAD_EXIT_CODE EQUAL "0")
    message(FATAL_ERROR "Could not download Spinnaker SDK!")
  endif()

  set(SPINNAKER_SEARCH_PATHS ${SPINNAKER_SDK_DIR}/usr ${SPINNAKER_SDK_DIR}/opt/spinnaker)
  message("Downloaded Spinnaker search paths: ${SPINNAKER_SEARCH_PATHS}")

  find_path(_Spinnaker_INCLUDE_DIR NAMES Spinnaker.h PATHS ${SPINNAKER_SEARCH_PATHS} NO_DEFAULT_PATH PATH_SUFFIXES include include/spinnaker REQUIRED)
  find_library(_Spinnaker_LIBRARY NAMES Spinnaker PATHS ${SPINNAKER_SEARCH_PATHS} NO_DEFAULT_PATH PATH_SUFFIXES lib REQUIRED)
  get_filename_component(_Spinnaker_LIBRARY_RESOLVED ${_Spinnaker_LIBRARY} REALPATH)
  if(IS_SYMLINK ${_Spinnaker_LIBRARY})
    message(STATUS "SYMLINK: ${_Spinnaker_LIBRARY}")
    get_filename_component(test ${_Spinnaker_LIBRARY} REALPATH)
    message(STATUS "Real Path: ${test}")
  endif()

  message("Downloaded Spinnaker include dir: ${_Spinnaker_INCLUDE_DIR}")
  message("Downloaded Spinnaker library: ${_Spinnaker_LIBRARY}")
  message("Downloaded Spinnaker library resolved: ${_Spinnaker_LIBRARY_RESOLVED}")

  set(${FLIR_INCLUDE_DIR_VAR} "${_Spinnaker_INCLUDE_DIR}" PARENT_SCOPE)
  set(${FLIR_LIB_VAR} "${_Spinnaker_LIBRARY}" PARENT_SCOPE)
  #set(${FLIR_LIB_VAR} "${SPINNAKER_SDK_DIR}/usr/lib/libSpinnaker.so.1.27.0.48" PARENT_SCOPE)
endfunction()
