cmake_minimum_required(VERSION 2.8.3)
project(alberto)

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
  src/graphic
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)
