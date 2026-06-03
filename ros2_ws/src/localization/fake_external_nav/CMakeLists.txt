cmake_minimum_required(VERSION 3.8)
project(fake_external_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(fake_external_nav_node
  src/fake_external_nav_node.cpp
)

ament_target_dependencies(fake_external_nav_node
  nav_msgs
  rclcpp
  std_msgs
)

install(TARGETS
  fake_external_nav_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
