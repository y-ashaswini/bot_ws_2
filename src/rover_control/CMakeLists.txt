cmake_minimum_required(VERSION 3.0.2)
project(rover_control)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  rover_wheels.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rover_control
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
