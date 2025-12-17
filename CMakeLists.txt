cmake_minimum_required(VERSION 3.8)
project(base_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(base_controller_node
  src/base_controller_node.cpp
  src/async_driver_client.cpp
)

target_include_directories(base_controller_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(base_controller_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
  std_msgs
  diagnostic_msgs
)

target_link_libraries(base_controller_node
  nlohmann_json::nlohmann_json
)

install(TARGETS
  base_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
