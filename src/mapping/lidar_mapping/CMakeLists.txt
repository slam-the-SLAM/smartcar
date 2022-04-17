cmake_minimum_required(VERSION 3.0.2)
project(lidar_mapping)

set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  tf
  custom_msg
)

find_package(Eigen3)
find_package(PCL REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES lidar_mapping
 CATKIN_DEPENDS
 sensor_msgs
 pcl_conversions
 pcl_msgs
 pcl_ros
 tf
 custom_msg
 DEPENDS EIGEN3 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# add_definitions(-DUSE_NDT)

add_executable(${PROJECT_NAME}_node 
  src/lidar_mapping.cpp
  src/lidar_mapping_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
