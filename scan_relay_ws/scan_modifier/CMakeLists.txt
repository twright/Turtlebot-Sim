cmake_minimum_required(VERSION 3.8)
project(scan_modifier)

# Set C++ standard to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(scan_node src/scan_node.cpp)
ament_target_dependencies(scan_node rclcpp sensor_msgs)

install(TARGETS
  scan_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()