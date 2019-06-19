cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0015 NEW)
project(bartolo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  cv_bridge
  image_transport
  std_msgs
)

# Find Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find Opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(... ${PCL_INCLUDE_DIRS})
link_directories(...${PCL_LIBRARY_DIRS})

## System dependencies are found with CMake's conventions find_package(Boost REQUIRED COMPONENTS system) 
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES alberto
#  CATKIN_DEPENDS roscpp sensor_msgs tf cv_bridge image_transport std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src/pcl
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)
