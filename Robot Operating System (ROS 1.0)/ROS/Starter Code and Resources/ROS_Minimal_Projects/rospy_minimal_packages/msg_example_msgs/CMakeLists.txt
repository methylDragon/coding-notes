cmake_minimum_required(VERSION 2.8.3)
project(msg_example_msgs)

# Standard dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# Your message files!
add_message_files(
  FILES
  My_msg.msg # <-- Like this one!
)

# You need to include this in as well
# Call it BEFORE catkin_package
generate_messages(
  DEPENDENCIES
  std_msgs # You might have additional message files from other packages
)

# Additional catkin dependencies
catkin_package(CATKIN_DEPENDS
  message_runtime
)


