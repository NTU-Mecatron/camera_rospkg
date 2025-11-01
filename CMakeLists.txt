cmake_minimum_required(VERSION 3.8)
project(camera_rospkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_publisher_node
  src/cam_pub_main.cpp
  src/camera_publisher.cpp
)

add_executable(video_saver_node
  src/video_saver_main.cpp
  src/video_saver.cpp
)

target_include_directories(camera_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(video_saver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(camera_publisher_node
  rclcpp sensor_msgs image_transport cv_bridge camera_info_manager)

ament_target_dependencies(video_saver_node
  rclcpp sensor_msgs image_transport cv_bridge camera_info_manager)

target_link_libraries(camera_publisher_node ${OpenCV_LIBS})

target_link_libraries(video_saver_node ${OpenCV_LIBS})

install(TARGETS camera_publisher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(TARGETS video_saver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()