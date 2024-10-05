cmake_minimum_required(VERSION 3.0.2)
project(camera_rospkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES camera_rospkg
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs image_transport cv_bridge
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_publisher src/main.cpp src/camera_publisher.cpp)
target_link_libraries(camera_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(camera_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
