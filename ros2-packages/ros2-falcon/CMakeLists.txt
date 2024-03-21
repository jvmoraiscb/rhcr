cmake_minimum_required(VERSION 3.8)
project(ros2-falcon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
    include
)

add_executable(
    main
    src/main.cpp
    src/falcon.cpp
    src/falcon_node.cpp
)

target_link_libraries(
	main
	nifalcon
	usb-1.0
)

ament_target_dependencies(
    main
    rclcpp
    std_msgs
    geometry_msgs
)

install(
    TARGETS
    main
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
