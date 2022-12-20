cmake_minimum_required(VERSION 3.0.2)
project(apple_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
)

catkin_package(
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS scripts/apple_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)