cmake_minimum_required(VERSION 3.8)
project(easydocking_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(easydocking_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/docking_controller.cpp
  src/relative_estimator.cpp
  src/guidance_law.cpp
  src/backstepping_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  easydocking_msgs
  Eigen3
)

# Create executable
add_executable(docking_controller_node src/docking_controller_node.cpp)
target_link_libraries(docking_controller_node ${PROJECT_NAME})
ament_target_dependencies(docking_controller_node
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  easydocking_msgs
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  docking_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(PROGRAMS
  scripts/experiment_logger.py
  scripts/rviz_visualizer.py
  scripts/px4_offboard_bridge.py
  scripts/px4_fixed_wing_bridge.py
  scripts/px4_odom_bridge.py
  scripts/auto_start_docking.py
  scripts/simple_dual_uav_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ../../config/docking.rviz
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
