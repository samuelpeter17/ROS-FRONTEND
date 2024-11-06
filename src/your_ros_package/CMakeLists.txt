cmake_minimum_required(VERSION 3.10)

project(your_package_name)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy std_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(your_node_name src/your_node.cpp)

target_link_libraries(your_node_name
  ${catkin_LIBRARIES}
)
