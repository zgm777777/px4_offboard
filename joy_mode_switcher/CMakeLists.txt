cmake_minimum_required(VERSION 3.8)
project(joy_mode_switcher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(offboard_manager REQUIRED)

add_executable(joy_mode_switcher_node
  src/joy_mode_switcher_node.cpp
)

ament_target_dependencies(joy_mode_switcher_node
  rclcpp
  geometry_msgs
  sensor_msgs
  offboard_manager
)

install(TARGETS joy_mode_switcher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
