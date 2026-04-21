cmake_minimum_required(VERSION 3.10)
project(camera_rospkg)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-DAPI_EXPORTS)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# Find ROS 2 Dependencies
# =============================================================================
find_package(ament_cmake REQUIRED)

set(PACKAGE_DEPENDENCIES
  "rclcpp"
  "rclcpp_lifecycle"
  "rclcpp_components"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "OpenCV"
)

find_package(yaml-cpp REQUIRED)

foreach(PKGDEP IN LISTS PACKAGE_DEPENDENCIES)
  find_package(${PKGDEP} REQUIRED)
endforeach()

# =============================================================================
# Component Library
# =============================================================================
add_library(${PROJECT_NAME}_component SHARED
  src/camera_publisher.cpp
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_component ${OpenCV_LIBS} yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_component ${PACKAGE_DEPENDENCIES})

# =============================================================================
# Register Component
# =============================================================================
rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "camera_rospkg::CameraPublisher"
  EXECUTABLE camera_publisher
)

# =============================================================================
# Installation
# =============================================================================
install(
  TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# =============================================================================
# Testing
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# =============================================================================
# Export
# =============================================================================
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
