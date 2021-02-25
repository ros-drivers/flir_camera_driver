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

  find_path(Spinnaker_INCLUDE_DIR NAMES Spinnaker.h PATHS ${SPINNAKER_SEARCH_PATHS} PATH_SUFFIXES include include/spinnaker)
  find_library(Spinnaker_LIBRARIES NAMES Spinnaker PATHS ${SPINNAKER_SEARCH_PATHS} PATH_SUFFIXES lib)

  set(${FLIR_INCLUDE_DIR_VAR} "${Spinnaker_INCLUDE_DIR}" PARENT_SCOPE)
  set(${FLIR_LIB_VAR} "${Spinnaker_LIBRARIES}" PARENT_SCOPE)
endfunction()
