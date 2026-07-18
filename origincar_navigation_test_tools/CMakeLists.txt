cmake_minimum_required(VERSION 3.8)
project(origincar_navigation_test_tools)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(origincar_navigation REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(local_avoidance_manual_test_node
  src/local_avoidance_manual_test_node.cpp
)

ament_target_dependencies(local_avoidance_manual_test_node
  geometry_msgs
  nav2_costmap_2d
  nav_msgs
  origincar_navigation
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

install(
  TARGETS local_avoidance_manual_test_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch doc rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()