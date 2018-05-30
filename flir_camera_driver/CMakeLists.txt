cmake_minimum_required(VERSION 2.8.3)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Wextra")


project(flir_camera_driver)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager diagnostic_updater dynamic_reconfigure
  image_exposure_msgs image_transport nodelet roscpp sensor_msgs
  wfov_camera_msgs
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Flir.cfg
)

catkin_package(CATKIN_DEPENDS
  image_exposure_msgs nodelet roscpp sensor_msgs wfov_camera_msgs
  DEPENDS OpenCV
)


# If Spinnacker is already present, use the found version. If not, download it.
# We can't resolve this dependency using the usual rosdep means because
# the Point Grey EULA prohibits redistributing the headers or the packages which
# contains them. We work around this by downloading the archive directly from
# their website during this step in the build process.
find_library(Spinnaker_LIBRARIES Spinnaker)
# if(NOT Spinnaker_LIBRARIES)
  message(STATUS "libSpinnaker not found in system library path")
  include(cmake/DownloadSpinnaker.cmake)
  download_spinnaker(Spinnaker_LIBRARIES Spinnaker_INCLUDE_DIRS)
# endif()

message(STATUS "libSpinnaker library: ${Spinnaker_LIBRARIES}")
message(STATUS "libSpinnaker include: ${Spinnaker_INCLUDE_DIRS}")


include_directories(SYSTEM
                    ${Spinnaker_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_library(FlirCameraLib src/FlirCamera.cpp)

# Include the Spinnaker Libs
target_link_libraries(FlirCameraLib
                      ${Spinnaker_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES})


# Test Executable for Spinnaker
add_executable(spinnaker_test_node
              src/spinnaker_test_node.cpp
)

target_link_libraries(spinnaker_test_node
  FlirCameraLib
  ${catkin_LIBRARIES}
)


add_dependencies(FlirCameraLib ${PROJECT_NAME}_gencfg)



add_library(FlirCameraNodelet src/nodelet.cpp)
target_link_libraries(FlirCameraNodelet FlirCameraLib ${catkin_LIBRARIES})

# add_library(FlirStereoCameraNodelet src/stereo_nodelet.cpp)
# target_link_libraries(FlirStereoCameraNodelet FlirCameraLib ${catkin_LIBRARIES})

# add_executable(flir_camera_node src/node.cpp)
# target_link_libraries(flir_camera_node FlirCameraLib ${catkin_LIBRARIES})
# set_target_properties(flir_camera_node
#                       PROPERTIES OUTPUT_NAME camera_node PREFIX "")

# add_executable(flir_stereo_node src/stereo_node.cpp)
# target_link_libraries(flir_stereo_node FlirCameraLib ${catkin_LIBRARIES})
# set_target_properties(flir_stereo_node
#                       PROPERTIES OUTPUT_NAME stereo_node PREFIX "")

# add_executable(flir_list_cameras src/list_cameras.cpp)
# target_link_libraries(flir_list_cameras FlirCameraLib ${catkin_LIBRARIES})
# set_target_properties(flir_list_cameras
#                       PROPERTIES OUTPUT_NAME list_cameras PREFIX "")

install(TARGETS
  FlirCameraLib
  FlirCameraNodelet
  # FlirStereoCameraNodelet
  # flir_camera_node
  # flir_list_cameras
  # flir_stereo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Redistributing the flycapture .so file is permitted by the SDK EULA:
# http://www.ptgrey.com/support/kb/data/PGR-FlyCap-SDK-LA.pdf
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/usr/lib/ DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         PATTERN libQt* EXCLUDE)

install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/bumblebee.launch)
  roslaunch_add_file_check(launch/camera.launch)
  roslaunch_add_file_check(launch/stereo.launch)

  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
