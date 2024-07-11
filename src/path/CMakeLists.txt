cmake_minimum_required(VERSION 3.0.2)
project(path)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  quadrotor_msgs
  roscpp
  rospy
  roslib
  std_msgs
  swarm_msgs
)
find_package(yaml-cpp)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/eigen3
  ${YAML_CPP_INCLUDE_DIR}
)

add_executable(sim_balloon_node
  src/sim_balloon_motion.cpp)
target_link_libraries(sim_balloon_node
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(path_finding_node
  src/path_finding.cpp)
target_link_libraries(path_finding_node
  ${catkin_LIBRARIES}
)