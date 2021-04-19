cmake_minimum_required(VERSION 3.0.2)
project(serial_node)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  turtlebot3_msgs
)

catkin_package(

  CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs std_msgs tf turtlebot3_msgs

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_example src/serial_example.cpp)

target_link_libraries(serial_example
  ${catkin_LIBRARIES}
)