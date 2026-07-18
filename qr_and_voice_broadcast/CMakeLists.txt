cmake_minimum_required(VERSION 3.8)
project(qr_and_voice_broadcast)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(origincar_msg REQUIRED)

# system dependencies
find_package(OpenCV REQUIRED)
find_library(ZBAR_LIBRARY zbar REQUIRED)

# --- qr_decoder node ---
add_executable(qr_decoder src/qr_decoder.cpp)
ament_target_dependencies(qr_decoder
  rclcpp sensor_msgs std_msgs cv_bridge ai_msgs origincar_msg OpenCV
)
target_link_libraries(qr_decoder ${ZBAR_LIBRARY})
target_include_directories(qr_decoder PRIVATE /usr/include/zbar)

install(TARGETS
  qr_decoder
  DESTINATION lib/${PROJECT_NAME}
)

# --- Python scripts ---
install(PROGRAMS
  scripts/speak_result.py
  scripts/image_to_string.py
  scripts/yuying.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Launch files ---
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
