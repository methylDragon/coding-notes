cmake_minimum_required(VERSION 2.8.3)
project(basic_pub_sub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# Initialise the export variables
catkin_package(
  INCLUDE_DIRS src
  # LIBRARIES blah
  CATKIN_DEPENDS roscpp std_msgs 
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# One per executable!
add_executable(basic_pub src/basic_pub.cpp)
target_link_libraries(basic_pub ${catkin_LIBRARIES})

add_executable(basic_sub src/basic_sub.cpp)
target_link_libraries(basic_sub ${catkin_LIBRARIES})

add_executable(hello_world src/hello_world.cpp)
target_link_libraries(hello_world ${catkin_LIBRARIES})


install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(DIRECTORY param
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(DIRECTORY maps
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(DIRECTORY rviz
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
