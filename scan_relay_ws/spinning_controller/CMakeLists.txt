cmake_minimum_required(VERSION 3.5)
project(spinning_controller)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(spin_interfaces REQUIRED)

# Controller packages:
find_package(dwb_core REQUIRED)
# find_package(nav2_regulated_pure_pursuit_controller REQUIRED)

nav2_package()

include_directories(
  include
)

set(dependencies
  nav_2d_msgs
  pluginlib
  rclcpp
  nav2_util
  spin_interfaces
  # Controller packages
  dwb_core
  # nav2_regulated_pure_pursuit_controller
)

add_library(spinning_controller SHARED
        src/spinning_controller.cpp)

# prevent pluginlib from using boost
target_compile_definitions(spinning_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(spinning_controller
  ${dependencies}
)

install(TARGETS spinning_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(spinning_controller)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core spinning_controller.xml)

ament_package()
