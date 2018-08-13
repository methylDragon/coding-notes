cmake_minimum_required(VERSION 2.8.3)
project(srv_example)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

catkin_python_setup()

add_service_files(
  FILES
  AddTwoInts.srv # <-- Add your srv files
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime # <-- Add this 
)
