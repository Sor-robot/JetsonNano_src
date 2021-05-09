cmake_minimum_required(VERSION 2.8.3)
project(csi_camera)

find_package(catkin REQUIRED cv_bridge rospy roscpp std_msgs genmsg geometry_msgs image_transport OpenCV)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination


catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES 
   CATKIN_DEPENDS roscpp nodelet sensor_msgs
#  DEPENDS system_lib
)


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(csi_camera src/csi_camera.cpp)
target_link_libraries(csi_camera ${Boost_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(csi_camera csi_camera_node)
