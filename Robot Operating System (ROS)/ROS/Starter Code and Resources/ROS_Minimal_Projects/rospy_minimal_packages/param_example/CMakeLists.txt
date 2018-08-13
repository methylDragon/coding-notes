cmake_minimum_required(VERSION 2.8.3)
project(param_example)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()
catkin_python_setup()
