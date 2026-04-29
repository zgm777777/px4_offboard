cmake_minimum_required(VERSION 3.8)
project(px4_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(offboard_manager REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Charts)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_executable(px4_gui_node
  src/px4_gui_node.cpp
  include/px4_gui/px4_gui_node.hpp
)

target_include_directories(px4_gui_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(px4_gui_node
  rclcpp
  geometry_msgs
  std_msgs
  px4_msgs
  offboard_manager
)

target_link_libraries(px4_gui_node
  Qt5::Widgets
  Qt5::Charts
)

ament_target_dependencies(px4_gui_node Qt5)

install(TARGETS px4_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
