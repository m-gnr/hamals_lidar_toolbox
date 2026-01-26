cmake_minimum_required(VERSION 3.8)
project(hamals_lidar_toolbox)

# =========================
# Compiler options
# =========================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Find dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hamals_lidar_msgs REQUIRED)


# =========================
# Core library (ROS'suz)
# =========================
add_library(hamals_lidar_toolbox_core
  src/core/ScanData.cpp
  src/core/ScanSanitizer.cpp
  src/core/ScanSegmenter.cpp
  src/core/ScanMetrics.cpp
  src/core/ObstacleDetector.cpp
)

target_include_directories(hamals_lidar_toolbox_core
  PUBLIC
    include
)

# =========================
# ROS adapter library
# =========================
add_library(hamals_lidar_toolbox_ros
  ros/adapters/LaserScanAdapter.cpp
)

target_include_directories(hamals_lidar_toolbox_ros
  PUBLIC
    include
  PRIVATE
    ros
)

ament_target_dependencies(hamals_lidar_toolbox_ros
  rclcpp
  sensor_msgs
)

target_link_libraries(hamals_lidar_toolbox_ros
  hamals_lidar_toolbox_core
)

# =========================
# Scan processor node (EXECUTABLE)
# =========================
add_executable(scan_processor_node
  ros/nodes/scan_processor_node.cpp
)

ament_target_dependencies(scan_processor_node
  rclcpp
  sensor_msgs
  hamals_lidar_msgs
)

target_link_libraries(scan_processor_node
  hamals_lidar_toolbox_ros
  hamals_lidar_toolbox_core
)

# =========================
# Install targets
# =========================
install(
  TARGETS
    hamals_lidar_toolbox_core
    hamals_lidar_toolbox_ros
    scan_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# =========================
# Testing (lint)
# =========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# =========================
# Ament package
# =========================
ament_package()
