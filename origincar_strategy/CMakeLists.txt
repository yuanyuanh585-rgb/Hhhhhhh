cmake_minimum_required(VERSION 3.8)

project(origincar_strategy)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(route_strategy_controller
  src/route_strategy_controller.cpp
)

ament_target_dependencies(route_strategy_controller
  ament_index_cpp
  angles
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_link_libraries(route_strategy_controller
  yaml-cpp
)

install(
  TARGETS route_strategy_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
