cmake_minimum_required(VERSION 3.0.2)
project(code)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  local_pkg
  roscpp
  roslib
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  min_idx.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  local_pkg
  #   nav_msgs#   sensor_msgs#   std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES code
#  CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mission src/Mission.cpp)
target_link_libraries(mission ${catkin_LIBRARIES})

add_executable(pathlogger src/Path_Logger.cpp)
target_link_libraries(pathlogger ${catkin_LIBRARIES})
