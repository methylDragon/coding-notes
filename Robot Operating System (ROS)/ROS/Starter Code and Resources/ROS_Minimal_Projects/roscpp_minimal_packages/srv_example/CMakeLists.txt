cmake_minimum_required(VERSION 2.8.3)
project(srv_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
)

add_service_files(
  FILES
  AddTwoInts.srv # <-- Add your srv files
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp std_msgs message_runtime # <-- Add this 
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(srv_example_server src/srv_example_server)
add_dependencies(
  srv_example_server
  ${catkin_EXPORTED_TARGETS}
  ${srv_example_EXPORTED_TARGETS}
)

target_link_libraries(srv_example_server ${catkin_LIBRARIES})

add_executable(srv_example_client src/srv_example_client)
add_dependencies(
  srv_example_client
  ${catkin_EXPORTED_TARGETS}
  ${srv_example_EXPORTED_TARGETS}
)


target_link_libraries(srv_example_client ${catkin_LIBRARIES})
