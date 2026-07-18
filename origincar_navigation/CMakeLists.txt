# ROS 2 ament_cmake 包的最低 CMake 版本要求。
cmake_minimum_required(VERSION 3.8)

# 包名必须和 package.xml 中的 <name> 保持一致。
project(origincar_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/segment_bypass_planner.cpp
  src/tracking_route_provider.cpp
  src/noop_progress_checker.cpp
  # 统一 GoalChecker 与控制器终点判定语义，避免 action success 与停车条件分裂。
  src/origincar_goal_checker.cpp
  src/origincar_ackermann_pure_pursuit_controller.cpp
  src/origincar_ackermann_controller.cpp
  src/origincar_global_planner.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  angles
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_navfn_planner
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

add_executable(route_markers_publisher
  src/route_markers_publisher.cpp
)

ament_target_dependencies(route_markers_publisher
  ament_index_cpp
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

target_link_libraries(route_markers_publisher
  yaml-cpp
)

pluginlib_export_plugin_description_file(nav2_core plugins/origincar_nav_plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS route_markers_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/route_point_formatter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# 安装配置、行为树、launch 和插件描述目录。
install(DIRECTORY behavior_trees config launch plugins doc maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  ament_index_cpp
  angles
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_navfn_planner
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

# 生成 ament 包索引信息。
ament_package()
