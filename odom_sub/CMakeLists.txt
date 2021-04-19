cmake_minimum_required(VERSION 3.0.2)
project(odom_sub)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  nav_msgs
  roscpp
)

catkin_package()

include_directories(s
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_listener src/odom_listener.cpp)
add_dependencies(odom_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_listener
	${catkin_LIBRARIES})

add_executable(goals src/simple_navigation_goals.cpp)
target_link_libraries(goals ${catkin_LIBRARIES})
target_link_libraries(goals
	${catkin_LIBRARIES})
