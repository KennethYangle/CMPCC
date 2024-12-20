cmake_minimum_required(VERSION 2.8...3.27)
project(bs_assis)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  rospy
  std_msgs
  geometry_msgs
  swarm_msgs
)

if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
endif()

if(NOT fastrtps_FOUND)
    find_package(fastrtps REQUIRED)
endif()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bs_assis
#  CATKIN_DEPENDS mavros roscpp rospy std_msgs
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
  "dds_gen"
)


file(GLOB DDS_GENERATE_CXX "dds_gen/*.cxx")
file(GLOB SOURCE_CPP "src/*.cpp")
file(GLOB SOURCE_HPP "src/*.hpp")

add_executable(dds_node ${SOURCE_CPP} ${SOURCE_HPP} ${DDS_GENERATE_CXX})
target_link_libraries(dds_node ${catkin_LIBRARIES} fastrtps fastcdr)
