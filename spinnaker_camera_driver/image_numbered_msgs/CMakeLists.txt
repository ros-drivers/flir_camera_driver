cmake_minimum_required(VERSION 2.8.3)
project(image_numbered_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation sensor_msgs)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  ImageNumbered.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
