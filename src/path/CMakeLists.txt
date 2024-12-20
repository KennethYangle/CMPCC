cmake_minimum_required(VERSION 3.0.2)
project(path)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_BUILD_TYPE "RELEASE")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

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
find_package(PythonLibs REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/eigen3
  ${YAML_CPP_INCLUDE_DIR}
)
include_directories(../../matplotlib)

add_executable(sim_balloon_node
  src/sim_balloon_motion.cpp)
target_link_libraries(sim_balloon_node
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(receive_balloon_node
  src/receive_balloon_motion_from_lidar.cpp)
target_link_libraries(receive_balloon_node
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(path_finding_node
  src/path_finding.cpp)
target_link_libraries(path_finding_node
  ${catkin_LIBRARIES}
)

add_executable(time_optimal_PMM_node
  src/time_optimal_PMM_test.cpp
  src/time_optimal_PMM.cpp)
include_directories(${PYTHON_INCLUDE_DIRS})
target_include_directories(time_optimal_PMM_node PRIVATE ${Python_INCLUDE_DIRS})
target_link_libraries(time_optimal_PMM_node 
  ${catkin_LIBRARIES}
  ${Python_LIBRARIES}
)

add_executable(path_planning_node
  src/path_planning.cpp
  src/time_optimal_PMM.cpp)
target_include_directories(path_planning_node PRIVATE ${Python_INCLUDE_DIRS})
target_link_libraries(path_planning_node
  ${catkin_LIBRARIES}
  ${Python_LIBRARIES}
  yaml-cpp
)