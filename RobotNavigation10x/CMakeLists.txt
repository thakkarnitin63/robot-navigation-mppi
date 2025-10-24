cmake_minimum_required(VERSION 3.8)
project(RobotNavigation10x)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenMP for parallel MPPI sampling
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found - MPPI will use parallel sampling")
else()
    message(STATUS "OpenMP not found - MPPI will run sequentially")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rclpy REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Build the main library (includes MPPI)
add_library(robot_nav_lib
  src/path_smoother.cpp
  src/trajectory_generator.cpp
  src/controller.cpp
  src/mppi_controller.cpp
)

target_include_directories(robot_nav_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies to the library
ament_target_dependencies(robot_nav_lib
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
)

# Link OpenMP if available (must be AFTER ament_target_dependencies)
if(OpenMP_CXX_FOUND)
    target_link_libraries(robot_nav_lib OpenMP::OpenMP_CXX)
endif()

# ============================================
# Executables
# ============================================

# 1. Original Pure Pursuit tracker (Tasks 1-3)
add_executable(trajectory_tracker_node src/trajectory_tracker_node.cpp)
ament_target_dependencies(trajectory_tracker_node
  rclcpp
  nav_msgs
  geometry_msgs
)
target_link_libraries(trajectory_tracker_node robot_nav_lib)

# 2. MPPI tracker with hybrid obstacle detection (Extra Credit!)
add_executable(mppi_tracker_node src/mppi_tracker_node.cpp)
ament_target_dependencies(mppi_tracker_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)
target_link_libraries(mppi_tracker_node robot_nav_lib)


# ============================================
# Installation
# ============================================

install(TARGETS robot_nav_lib
  EXPORT export_robot_nav_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)


install(TARGETS
  trajectory_tracker_node
  mppi_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  trajectory_tracker_node
  mppi_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_robot_nav_lib HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()