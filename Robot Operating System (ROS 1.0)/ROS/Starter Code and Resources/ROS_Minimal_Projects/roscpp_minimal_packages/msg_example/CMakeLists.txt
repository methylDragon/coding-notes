cmake_minimum_required(VERSION 2.8.3)
project(msg_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  msg_example_msgs
  message_generation
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp msg_example_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(msg_example src/msg_example)
target_link_libraries(msg_example ${catkin_LIBRARIES})
add_dependencies(msg_example msg_example_msgs_generate_messages_cpp)
