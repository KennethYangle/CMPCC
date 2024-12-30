cmake_minimum_required(VERSION 2.8.3)
project(offboard_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  mavros_msgs
  std_msgs
  quadrotor_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main_node
  src/main.cpp
  src/utils_offboard.cpp
)
target_link_libraries(main_node ${catkin_LIBRARIES})