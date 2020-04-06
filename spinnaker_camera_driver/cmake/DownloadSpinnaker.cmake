function(download_spinnaker FLIR_LIB_VAR FLIR_INCLUDE_DIR_VAR)
  if(NOT UNIX)
    message(FATAL_ERROR "Downloading libSpinnaker for non-linux systems not supported")
  endif()

  execute_process(
    COMMAND lsb_release -cs
    OUTPUT_VARIABLE OS_CODE_NAME
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  include(cmake/TargetArch.cmake)
  target_architecture(FLIR_ARCH)
  set(FLIR_DIR ${CMAKE_CURRENT_BINARY_DIR}/usr/lib)
  set(DOWNLOAD_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/download_spinnaker")

  message(STATUS "Running download_spinnaker script with arguments: ${FLIR_ARCH} ${FLIR_DIR} ${OS_CODE_NAME}")
  execute_process(
    COMMAND ${DOWNLOAD_SCRIPT} ${FLIR_ARCH} "${FLIR_DIR}" ${OS_CODE_NAME})
  set(${FLIR_LIB_VAR} "${FLIR_DIR}/libSpinnaker.so" PARENT_SCOPE)
  set(${FLIR_INCLUDE_DIR_VAR} "${CMAKE_CURRENT_BINARY_DIR}/usr/include/spinnaker" PARENT_SCOPE)
endfunction()
