cmake_minimum_required(VERSION 3.0.2)
project(publisher_subscriber)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp)
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(FILES MsgTutorial.msg)
add_service_files(FILES SrvFile.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
LIBRARIES publisher_subscriber
CATKIN_DEPENDS std_msgs roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


##### topic_publisher 
# add_executable(topic_publisher ${PROJECT_NAME}/src/topic_publisher.cpp)
add_executable(topic_publisher src/topic_publisher.cpp)
add_dependencies(topic_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(topic_publisher ${catkin_LIBRARIES})

add_executable(service_server src/service_server.cpp)
add_dependencies(service_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(service_server ${catkin_LIBRARIES})

##### topic_subscriber
add_executable(topic_subscriber src/topic_subscriber.cpp)
add_dependencies(topic_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(topic_subscriber ${catkin_LIBRARIES})

add_executable(service_client src/service_client.cpp)
add_dependencies(service_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(service_client ${catkin_LIBRARIES})
