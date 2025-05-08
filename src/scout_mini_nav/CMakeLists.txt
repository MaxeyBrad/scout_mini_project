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

add_executable(waypoint_follower_node src/waypoint_follower.cpp)
target_link_libraries(waypoint_follower_node
  ${catkin_LIBRARIES}
)

add_executable(simple_square_nav_node src/simple_square_nav.cpp)
target_link_libraries(simple_square_nav_node
  ${catkin_LIBRARIES}
)


add_executable(basic_waypoint_follower_node src/basic_waypoint_follower.cpp)
target_link_libraries(basic_waypoint_follower_node ${catkin_LIBRARIES})
