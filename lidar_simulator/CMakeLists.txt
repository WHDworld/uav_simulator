cmake_minimum_required(VERSION 3.0.2)
project(lidar_simulator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# CUDA support
find_package(CUDA REQUIRED)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -use_fast_math)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}" "--ptxas-options=-v")
# Set compute capability based on GPU (4060 = sm_89, 3080/3090 = sm_86, 2080 = sm_75)
set(CUDA_NVCC_FLAGS -gencode arch=compute_89,code=sm_89;)
set(CUDA_PROPAGATE_HOST_FLAGS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${CUDA_INCLUDE_DIRS}
  LIBRARIES lidar_render_cuda lidar_simulator
  CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs geometry_msgs nav_msgs pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
)

# CUDA library for LiDAR rendering
cuda_add_library(lidar_render_cuda
  src/lidar_render.cu
)

# LiDAR simulator library
add_library(lidar_simulator
  src/lidar_simulator.cpp
)
target_link_libraries(lidar_simulator
  lidar_render_cuda
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# LiDAR simulator node
add_executable(lidar_simulator_node
  src/lidar_simulator_node.cpp
)
target_link_libraries(lidar_simulator_node
  lidar_simulator
  lidar_render_cuda
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Install
install(TARGETS lidar_render_cuda lidar_simulator lidar_simulator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
