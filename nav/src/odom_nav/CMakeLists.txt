cmake_minimum_required(VERSION 3.0.2)
project(odom_nav)

find_package(catkin REQUIRED COMPONENTS
    rospy
    tf2_ros
    nav_msgs
    geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
    scripts/odom_to_tf.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
