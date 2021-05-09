cmake_minimum_required(VERSION 2.8.3)
project(mbsrobot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  rosserial_python
  sensor_msgs
  mbs_msgs
  std_msgs
  tf
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mbsrobot
  CATKIN_DEPENDS roscpp rospy tf nav_msgs mbs_msgs geometry_msgs sensor_msgs std_msgs 
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mbs_base src/mbs_base.cpp)
add_executable(mbs_base_node src/mbs_base_node.cpp)
target_link_libraries(mbs_base_node mbs_base ${catkin_LIBRARIES})

