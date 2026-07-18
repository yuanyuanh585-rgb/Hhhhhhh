# ROS 2 ament_cmake 包的最低 CMake 版本要求。
cmake_minimum_required(VERSION 3.8)

# 包名必须和 package.xml 中的 <name> 保持一致。
project(origincar_sim)

# GCC/Clang 下打开常用告警，帮助尽早发现未使用变量、可疑写法等问题。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找构建和运行自定义 Gazebo Fortress 插件需要的依赖。
# ament_cmake: ROS 2 CMake 构建系统。
# ackermann_msgs/geometry_msgs/nav_msgs: 控制输入和里程计消息。
# ignition-gazebo6/ignition-plugin1: Gazebo Fortress System 插件接口。
# rclcpp/tf2/tf2_ros: C++ ROS 节点、四元数和 TF 广播。
find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

# 构建 Gazebo Fortress System 插件动态库。Gazebo 会按文件名
# libackermann_kinematic_plugin.so 加载它。
add_library(ackermann_kinematic_plugin SHARED
  src/ackermann_kinematic_plugin.cpp
)
set_property(TARGET ackermann_kinematic_plugin PROPERTY CXX_STANDARD 17)

# 声明目标依赖，ament 会自动补充 include 路径、库路径和编译定义。
ament_target_dependencies(ackermann_kinematic_plugin
  ackermann_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
)

# 链接 Gazebo Fortress 和插件注册库，使插件能调用 ECS / System API。
target_link_libraries(ackermann_kinematic_plugin
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
)

# 安装插件库到 install/origincar_sim/lib。
# sim_world.launch.py 会把这个目录加入 IGN_GAZEBO_SYSTEM_PLUGIN_PATH。
install(TARGETS ackermann_kinematic_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装 Python 可执行脚本到 lib/origincar_sim，供 launch_ros.actions.Node 启动。
install(PROGRAMS
  scripts/move_cone.py
  scripts/texture_marker_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装仿真所需资源目录：
# config 参数、launch 启动文件、models Gazebo 模型、rviz 配置、
# urdf/xacro 机器人模型、worlds Gazebo 世界。
install(DIRECTORY config launch models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# 生成 ament 包索引信息。
ament_package()
