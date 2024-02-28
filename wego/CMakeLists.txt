cmake_minimum_required(VERSION 3.0.2)
project(wego)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  Lidar_pkg
  dh_pkg
  code
  local_pkg
  message_generation
)
# Generate messages in the 'msg' folder
add_message_files(
  FILES
  parking.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wego
 CATKIN_DEPENDS roscpp rospy 
 message_runtime

#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(lidar_convert src/convert_lidar.cpp)
target_link_libraries(lidar_convert ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/pub_odom.py
  scripts/navigation_oneway.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
