cmake_minimum_required(VERSION 3.0.2)
project(rhcr)

find_package(
  catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

add_executable(
  main
  src/main.cpp
  src/falcon.cpp
  src/node.cpp
)
target_link_libraries(
  main
  ${catkin_LIBRARIES}
  nifalcon
  usb-1.0
)
  
add_executable(
  turtlesim
  src/turtlesim.cpp
  src/falcon.cpp
)
target_link_libraries(
  turtlesim
  ${catkin_LIBRARIES}
  nifalcon
  usb-1.0
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
