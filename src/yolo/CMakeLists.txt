cmake_minimum_required(VERSION 3.0.2)
project(yolo)

add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

SET(cv_bridge_DIR ~/CMPCC/devel/share/cv_bridge/cmake)
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp std_msgs std_srvs sensor_msgs swarm_msgs camera_info_manager)

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
enable_language(CUDA)
catkin_package(
)

include_directories(/usr/local/cuda/include)
include_directories(/usr/local/cuda/targets/aarch64-linux/include/)
link_directories(/usr/local/cuda/lib64)

SET(OpenCV_DIR  ~/opencv-4.5.4-D/lib/cmake/opencv4/)

find_package(OpenCV 4.5.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
  src/src
  src/plugin
  
)
file(GLOB_RECURSE SRCS src/src/*.cpp src/src/*.cu)
file(GLOB_RECURSE PLUGIN_SRCS src/plugin/*.cu)



add_library(myplugins SHARED ${PLUGIN_SRCS} ${SRCS})
target_link_libraries(myplugins nvinfer cudart)

add_executable(det src/det.cpp)
add_executable(det_org1 src/det_org1.cpp)
add_executable(det_org2 src/det_org2.cpp)

target_link_libraries(det ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(det nvinfer)
target_link_libraries(det cudart)
target_link_libraries(det myplugins)

target_link_libraries(det_org2 ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(det_org2 nvinfer)
target_link_libraries(det_org2 cudart)
target_link_libraries(det_org2 myplugins)

target_link_libraries(det_org1 ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(det_org1 nvinfer)
target_link_libraries(det_org1 cudart)
target_link_libraries(det_org1 myplugins)


message(STATUS "library status:")
message(STATUS "OpenCV_VERSION: ${OpenCV_VERSION}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_DIR: ${OpenCV_DIR}")
message(STATUS "catkin libraries: ${catkin_LIBRARIES}")
message(STATUS "cv_bridge_DIR: ${cv_bridge_DIR}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")