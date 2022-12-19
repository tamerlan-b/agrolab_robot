cmake_minimum_required(VERSION 3.0.2)
project(state_machine)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS scripts/state_machine_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
