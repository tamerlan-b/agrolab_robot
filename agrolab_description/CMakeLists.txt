cmake_minimum_required(VERSION 3.0.2)
project(agrolab_description)

find_package(catkin REQUIRED COMPONENTS
        rospy
        roslint
        std_msgs
        sensor_msgs
        geometry_msgs)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY images meshes launch rviz urdf worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)
