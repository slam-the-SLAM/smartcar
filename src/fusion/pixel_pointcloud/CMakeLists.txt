cmake_minimum_required(VERSION 3.0.2)
project(pixel_pointcloud)
SET(CMAKE_BUILD_TYPE "Debug")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  cv_bridge
  image_transport
  tf
  #此处加入aruco会加载apt安装的ros-melodic-aruco包
  # aruco
  message_filters
)

find_package(Eigen3)
set(OpenCV_DIR /usr/local/OpenCV-3.4.8/share/OpenCV)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
set(aruco_DIR "/home/wjq/workspace/aruco/aruco-3.1.12/build")
find_package(aruco REQUIRED)
find_package(OpenMP)
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
 INCLUDE_DIRS include
 LIBRARIES pixel_pointcloud
 CATKIN_DEPENDS
#  roscpp
 sensor_msgs
 pcl_conversions
 pcl_msgs
 pcl_ros
 cv_bridge
 image_transport
 tf
 DEPENDS EIGEN3 PCL OpenCV aruco
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${aruco_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/pixel_pointcloud.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node 
  src/pixel_pointcloud.cpp
  src/pixel_pointcloud_node.cpp
  )

add_executable(pointcloud_rotate_node 
  src/pointcloud_rotate_node.cpp
  )
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
if(OPENMP_FOUND)
set_target_properties(${PROJECT_NAME}_node PROPERTIES
  COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
  LINK_FLAGS ${OpenMP_CXX_FLAGS})
endif()
## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${aruco_LIBS}
  ${PCL_LIBRARIES}
)

target_link_libraries(pointcloud_rotate_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

