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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(octomap_merger src/octomap_merger.cpp)
target_link_libraries(octomap_merger ${catkin_LIBRARIES})
