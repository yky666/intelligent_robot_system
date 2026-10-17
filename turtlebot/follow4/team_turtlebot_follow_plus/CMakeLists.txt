cmake_minimum_required(VERSION 3.0.2)
project(team_turtlebot_follow_plus)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar_msgs
  geometry_msgs
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS ar_track_alvar_msgs geometry_msgs roscpp sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ar_laser_follower_plus src/ar_laser_follower.cpp)
add_dependencies(ar_laser_follower_plus ${catkin_EXPORTED_TARGETS})
target_link_libraries(ar_laser_follower_plus ${catkin_LIBRARIES})
