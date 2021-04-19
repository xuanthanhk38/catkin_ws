cmake_minimum_required(VERSION 3.0.2)
project(blink_led)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(led_listener src/led_listener.cpp)
add_dependencies(led_listener std_msgs_generate_messages_cpp)
 target_link_libraries(led_listener
   ${catkin_LIBRARIES}
 )

