cmake_minimum_required(VERSION 2.8.3)
project(octomap_merger)

find_package(catkin REQUIRED COMPONENTS
  octomap_ros
  octomap_msgs
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  message_generation
  pcl_conversions
  pcl_ros
)

add_message_files(
  FILES
  OctomapArray.msg
  OctomapNeighbors.msg
)

generate_messages(
   DEPENDENCIES
   nav_msgs
   octomap_msgs
 )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_library(icp_align src/icp_align.cpp)
target_link_libraries(icp_align ${catkin_LIBRARIES})

add_library(map_merger src/map_merger.cpp)
target_link_libraries(map_merger ${catkin_LIBRARIES})

add_executable(octomap_merger src/octomap_merger.cpp)
target_link_libraries(octomap_merger icp_align map_merger ${catkin_LIBRARIES})

add_executable(octomap_merger_node src/octomap_merger_node.cpp)
target_link_libraries(octomap_merger_node icp_align map_merger ${catkin_LIBRARIES})
