cmake_minimum_required(VERSION 3.0.2)
project(scout_mini_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_nav_node src/waypoint_nav.cpp)
target_link_libraries(waypoint_nav_node
  ${catkin_LIBRARIES}
)
