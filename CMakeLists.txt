cmake_minimum_required(VERSION 3.0.2)
project(camera_rospkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  cv_bridge
  image_transport
  message_generation
)

find_package(OpenCV REQUIRED)

# add_service_files(
#   FILES
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES camera_rospkg
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs image_transport cv_bridge message_runtime 
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

add_executable(camera_publisher src/main.cpp src/camera_publisher.cpp src/utils.cpp)
target_link_libraries(camera_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(camera_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
