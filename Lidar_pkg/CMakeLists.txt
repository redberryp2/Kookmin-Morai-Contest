cmake_minimum_required(VERSION 3.0.2)
project(Lidar_pkg)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   pcl_conversions
   pcl_ros
   std_msgs
   message_generation
)
add_message_files(
  FILES
  pointCloud.msg
  pointinfo.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs 

)

include_directories(
    include
  ${catkin_INCLUDE_DIRS}
)

add_executable(Lidar_code src/Lidar_code.cpp)
target_link_libraries(Lidar_code ${catkin_LIBRARIES})
add_dependencies(Lidar_code ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
