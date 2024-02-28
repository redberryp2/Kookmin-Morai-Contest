cmake_minimum_required(VERSION 3.0.2)
project(local_pkg)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

add_message_files(
	DIRECTORY msg
	FILES
  VescState.msg
  VescStateStamped.msg
  PosPos.msg
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
   sensor_msgs
)
 
catkin_package(
  CATKIN_DEPENDS 
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
)

include_directories(
 include 
 ${catkin_INCLUDE_DIRS}
)

add_executable(gnss src/gps.cpp)
target_link_libraries(gnss ${catkin_LIBRARIES})
add_dependencies(gnss ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
