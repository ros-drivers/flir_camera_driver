cmake_minimum_required(VERSION 3.0.2)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Wextra")


project(spinnaker_camera_driver)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager diagnostic_updater dynamic_reconfigure
  image_exposure_msgs image_transport nodelet roscpp sensor_msgs
  wfov_camera_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Spinnaker.cfg
)

catkin_package(CATKIN_DEPENDS
  image_exposure_msgs nodelet roscpp sensor_msgs wfov_camera_msgs
)


# If Spinnacker is already present, use the found version. If not, download it.
# We can't resolve this dependency using the usual rosdep means because
# the Point Grey EULA prohibits redistributing the headers or the packages which
# contains them. We work around this by downloading the archive directly from
# their website during this step in the build process.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
find_package(Spinnaker QUIET)
if(NOT Spinnaker_FOUND)
  message(STATUS "libSpinnaker not found in system library path")
  include(cmake/DownloadSpinnaker.cmake)
  download_spinnaker(Spinnaker_LIBRARIES Spinnaker_INCLUDE_DIRS)
endif()

message(STATUS "libSpinnaker library: ${Spinnaker_LIBRARIES}")
message(STATUS "libSpinnaker include: ${Spinnaker_INCLUDE_DIRS}")


include_directories(SYSTEM
                    ${Spinnaker_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(SpinnakerCameraLib src/SpinnakerCamera.cpp)

# Include the Spinnaker Libs
target_link_libraries(SpinnakerCameraLib
                      Camera
                      ${Spinnaker_LIBRARIES}
                      ${catkin_LIBRARIES})


# Test Executable for Spinnaker
add_executable(spinnaker_test_node
              src/spinnaker_test_node.cpp
)

target_link_libraries(spinnaker_test_node
  SpinnakerCameraLib
  ${catkin_LIBRARIES}
)


add_dependencies(SpinnakerCameraLib ${PROJECT_NAME}_gencfg)


add_library(Camera src/camera.cpp)
target_link_libraries(Camera ${catkin_LIBRARIES})
add_dependencies(Camera ${PROJECT_NAME}_gencfg)

add_library(Cm3 src/cm3.cpp)
target_link_libraries(Cm3 Camera ${catkin_LIBRARIES})
add_dependencies(Cm3 ${PROJECT_NAME}_gencfg)

add_library(Gh3 src/gh3.cpp)
target_link_libraries(Gh3 Camera ${catkin_LIBRARIES})
add_dependencies(Gh3 ${PROJECT_NAME}_gencfg)

add_library(Diagnostics src/diagnostics.cpp)
target_link_libraries(Diagnostics Camera SpinnakerCameraLib ${catkin_LIBRARIES})
add_dependencies(Diagnostics ${PROJECT_NAME}_gencfg)

add_library(SpinnakerCameraNodelet src/nodelet.cpp)
target_link_libraries(SpinnakerCameraNodelet Diagnostics SpinnakerCameraLib Camera Gh3 Cm3 ${catkin_LIBRARIES})

add_executable(spinnaker_camera_node src/node.cpp)
target_link_libraries(spinnaker_camera_node SpinnakerCameraLib ${catkin_LIBRARIES})
set_target_properties(spinnaker_camera_node PROPERTIES OUTPUT_NAME camera_node PREFIX "")

install(TARGETS
  SpinnakerCameraLib
  SpinnakerCameraNodelet
  Camera
  Cm3
  Gh3
  Diagnostics
  spinnaker_camera_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Redistributing the flycapture .so file is permitted by the SDK EULA:
# http://www.ptgrey.com/support/kb/data/PGR-FlyCap-SDK-LA.pdf
if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/usr/lib)
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/usr/lib/ DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
           PATTERN libQt* EXCLUDE)
else()
  get_filename_component(Spinnaker_LIB_DIR ${Spinnaker_LIBRARIES} DIRECTORY)
  file(READ ${PROJECT_SOURCE_DIR}/cmake/install_files filenames)
  foreach(filename ${filenames})
    install(FILES ${Spinnaker_LIB_DIR}/${filename} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  endforeach()
endif()

install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/camera.launch)

  find_package(roslint REQUIRED)
  set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
  roslint_cpp()
  roslint_add_test()
endif()
