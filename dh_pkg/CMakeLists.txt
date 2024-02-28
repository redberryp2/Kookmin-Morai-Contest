cmake_minimum_required(VERSION 3.0.2)
project(dh_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  cam_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

catkin_install_python(PROGRAMS
  scripts/dh_cam.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
