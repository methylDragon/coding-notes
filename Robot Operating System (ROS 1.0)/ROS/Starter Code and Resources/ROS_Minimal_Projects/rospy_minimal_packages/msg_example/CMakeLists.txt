cmake_minimum_required(VERSION 2.8.3)
project(msg_example)

find_package(catkin REQUIRED COMPONENTS
  rospy
  msg_example_msgs
)

catkin_package()
catkin_python_setup()

install(PROGRAMS
  src/msg_example/msg_example
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
