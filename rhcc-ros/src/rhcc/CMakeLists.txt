cmake_minimum_required(VERSION 3.0.2)
project(rhcc)

find_package(
  catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

add_executable(
  main
  src/main.cpp
)

target_link_libraries(
  main
  ${catkin_LIBRARIES}
  nifalcon
  usb-1.0
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
