cmake_minimum_required(VERSION 3.8)
project(ros2-ackermann)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(
    ackermann_tf2_broadcaster
    src/ackermann_tf2_broadcaster.cpp
)

ament_target_dependencies(
    ackermann_tf2_broadcaster
    nav_msgs
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
)

install(
  TARGETS
  ackermann_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  src
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
