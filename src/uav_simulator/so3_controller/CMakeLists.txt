cmake_minimum_required(VERSION 3.0.2)
project(so3_controller)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  quadrotor_msgs
  roscpp
  nodelet
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS quadrotor_msgs roscpp nodelet
)

include_directories(
  include
  /usr/local/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_nodelet
  src/${PROJECT_NAME}_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet 
  ${catkin_LIBRARIES}
)