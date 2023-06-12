#  ----------------------------------------------------------------------------
#  2020 Bernd Pfrommer bernd.pfrommer@gmail.com
#
cmake_minimum_required(VERSION 3.5)
project(flir_spinnaker_common LANGUAGES CXX)

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

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
set(Spinnaker_FOUND 0)
message(STATUS "Found variable: " ${Spinnaker_FOUND})
find_package(Spinnaker QUIET)
message(STATUS "Found variable: " ${Spinnaker_FOUND} " at " ${Spinnaker_LIBRARIES} " and "${Spinnaker_INCLUDE_DIRS} )

if(NOT Spinnaker_FOUND)
  message(STATUS "spinnaker not found")
  message(STATUS "libSpinnaker not found in system library path")
  include(cmake/DownloadSpinnaker.cmake)
  download_spinnaker(Spinnaker_LIBRARIES Spinnaker_INCLUDE_DIRS)
endif()

message(STATUS "libSpinnaker library: ${Spinnaker_LIBRARIES}")
message(STATUS "libSpinnaker include: ${Spinnaker_INCLUDE_DIRS}")

find_package(Spinnaker REQUIRED)
include_directories(SYSTEM
                    ${Spinnaker_INCLUDE_DIRS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)

add_library(flir_spinnaker_common
  src/driver.cpp
  src/driver_impl.cpp
  src/image.cpp
  src/pixel_format.cpp
  src/genicam_utils.cpp
)

target_link_libraries(flir_spinnaker_common PUBLIC ${Spinnaker_LIBRARIES})

target_include_directories(flir_spinnaker_common
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# associates an "export_*" target with the library target
install(
  TARGETS flir_spinnaker_common
  EXPORT export_flir_spinnaker_common
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include
  DESTINATION ${CMAKE_INSTALL_PREFIX}
)

install(
  DIRECTORY ${Spinnaker_INCLUDE_DIRS}
  DESTINATION ${Spinnaker_INCLUDE_DIRS}
)

# this is what probably builds the cmake config files
ament_export_targets(export_flir_spinnaker_common)


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

