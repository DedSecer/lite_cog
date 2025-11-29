cmake_minimum_required(VERSION 3.0.2)
project(fastlio_local_nav)

find_package(catkin REQUIRED COMPONENTS
    rospy
    tf2_ros
    nav_msgs
    geometry_msgs
    std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
