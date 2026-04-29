cmake_minimum_required(VERSION 3.8)
project(offboard_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RequestMode.srv"
  "srv/RequestArm.srv"
  "srv/EmergencyDisarm.srv"
  "srv/Takeoff.srv"
)

add_executable(offboard_manager_node
  src/offboard_manager.cpp
)

target_include_directories(offboard_manager_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(offboard_manager_node
  rclcpp
  std_msgs
  px4_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(offboard_manager_node "${cpp_typesupport_target}")

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS offboard_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
