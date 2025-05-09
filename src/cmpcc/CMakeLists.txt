cmake_minimum_required(VERSION 2.8.3)
project(cmpcc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w -g -fPIC")

find_package(catkin REQUIRED COMPONENTS
        message_generation
        message_runtime
        # decomp_ros_msgs
        quadrotor_msgs
        roscpp
        roslib
        )

find_package(yaml-cpp)
find_package(osqp REQUIRED)
find_package(PythonLibs REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/eigen3
  ${YAML_CPP_INCLUDE_DIR}
)
include_directories(../../matplotlib)

add_executable(osqp_test
    src/osqp_test.cpp
    src/osqp_interface.cpp
)
target_link_libraries(osqp_test osqp::osqp)

# add_executable(fly_node
#     src/fly.cpp
#     src/display_msg.cpp
#     src/mpc_solver.cpp
#     src/osqp_interface.cpp
#     src/model.cpp
#     src/map.cpp
#     src/bezier_base.cpp
#     src/my_spline.cpp
#     src/corridor.cpp) 
# target_include_directories(fly_node PRIVATE ${Python_INCLUDE_DIRS})
# add_dependencies(fly_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(fly_node
#     ${catkin_LIBRARIES}
#     ${Python_LIBRARIES}
#     yaml-cpp
#     osqp::osqp
# )

add_executable(pva_chasing_node
    src/pva_chasing.cpp
    src/osqp_interface.cpp
    src/model.cpp
    src/map.cpp
    src/bezier_base.cpp
    src/my_spline.cpp
    src/corridor.cpp) 
target_include_directories(pva_chasing_node PRIVATE ${Python_INCLUDE_DIRS})
add_dependencies(pva_chasing_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pva_chasing_node
    ${catkin_LIBRARIES}
    ${Python_LIBRARIES}
    yaml-cpp
    osqp::osqp
)
