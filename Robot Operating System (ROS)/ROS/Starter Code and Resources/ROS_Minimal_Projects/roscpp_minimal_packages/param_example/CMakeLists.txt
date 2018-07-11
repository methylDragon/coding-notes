cmake_minimum_required(VERSION 2.8.3)
project(param_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(param_example src/param_example.cpp)
target_link_libraries(param_example ${catkin_LIBRARIES})
