# ROS 2 ament_cmake 包的最低 CMake 版本要求。
cmake_minimum_required(VERSION 3.8)

# 包名必须和 package.xml 中的 <name> 保持一致。
project(origincar_bringup)

# bringup 包本身不编译 C++/Python 节点，只需要 ament_cmake 来完成安装和导出。
find_package(ament_cmake REQUIRED)

install(FILES
  launch/real_mapping.launch.py
  launch/real_navigation.launch.py
  launch/real_robot.launch.py
  launch/real_save_map.launch.py
  launch/real_view_navigation.launch.py
  launch/sim.launch.py
  launch/sim_mapping.launch.py
  launch/sim_navigation.launch.py
  launch/sim_view_navigation.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)


# 生成 ament 包索引信息，让 ROS 2 工具链能够发现这个包。
ament_package()
