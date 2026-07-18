# ROS 2 ament_cmake 包的最低 CMake 版本要求。
cmake_minimum_required(VERSION 3.8)

# 包名必须和 package.xml 中的 <name> 保持一致。
project(origincar_mapping)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(laser_global_localizer src/laser_global_localizer.cpp)
target_compile_features(laser_global_localizer PUBLIC cxx_std_17)
ament_target_dependencies(laser_global_localizer
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(laser_predictive_localizer src/laser_predictive_localizer.cpp)
target_compile_features(laser_predictive_localizer PUBLIC cxx_std_17)
ament_target_dependencies(laser_predictive_localizer
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# 安装可执行 Python 脚本到 lib/origincar_mapping。
# ROS 2 的 Node(package=..., executable=...) 会在这个目录下查找可执行程序。
install(PROGRAMS
  scripts/localization_lifecycle_bringup.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  laser_global_localizer
  laser_predictive_localizer
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置和 launch 文件到 share/origincar_mapping。
# get_package_share_directory('origincar_mapping') 返回的就是安装后的 share 目录。
install(DIRECTORY config launch doc
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# 生成 ament 包索引信息。
ament_package()
