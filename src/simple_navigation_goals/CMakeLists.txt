cmake_minimum_required(VERSION 3.0.2)
project(simple_navigation_goals)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
