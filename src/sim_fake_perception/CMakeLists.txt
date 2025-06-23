cmake_minimum_required(VERSION 3.0.2)
project(sim_fake_perception)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  shape_msgs
  vader_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sim_fake_perception
#  CATKIN_DEPENDS geometry_msgs rospy shape_msgs vader_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
