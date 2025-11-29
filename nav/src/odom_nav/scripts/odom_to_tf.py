#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class OdomToTfNode:
    def __init__(self):
        rospy.init_node('odom_to_tf_node', anonymous=False)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Subscribe to /leg_odom2 topic
        self.odom_sub = rospy.Subscriber('/leg_odom2', Odometry, self.odom_callback)
        
        rospy.loginfo("odom_to_tf_node started, subscribing to /leg_odom2")

    def odom_callback(self, msg):
        """
        Callback function for odometry messages.
        Publishes a TF transform from map to base_link based on the odometry data.
        """
        t = TransformStamped()
        
        # Set header
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        
        # Set translation from odometry position
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Set rotation from odometry orientation
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = OdomToTfNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
