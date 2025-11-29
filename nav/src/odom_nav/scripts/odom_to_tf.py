#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf.transformations as tft
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Empty, EmptyResponse


class OdomToTfNode:
    def __init__(self):
        rospy.init_node('odom_to_tf_node', anonymous=False)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Offset for odometry reset (stores the pose at reset time)
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0
        self.offset_yaw = 0.0
        
        # Store the last received odometry for reset
        self.last_odom = None
        
        # Reset service
        self.reset_srv = rospy.Service('~reset_odom', Empty, self.reset_callback)
        
        # Subscribe to /leg_odom2 topic
        self.odom_sub = rospy.Subscriber('/leg_odom2', Odometry, self.odom_callback)
        
        rospy.loginfo("odom_to_tf_node started, subscribing to /leg_odom2")
        rospy.loginfo("Call service '~reset_odom' to reset odometry to origin")

    def reset_callback(self, req):
        """
        Service callback to reset odometry.
        Records the current position as offset so that current position becomes origin.
        """
        if self.last_odom is not None:
            self.offset_x = self.last_odom.pose.pose.position.x
            self.offset_y = self.last_odom.pose.pose.position.y
            self.offset_z = self.last_odom.pose.pose.position.z
            
            # Get current yaw
            q = self.last_odom.pose.pose.orientation
            _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.offset_yaw = yaw
            
            rospy.loginfo("Odometry reset! Offset: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f",
                          self.offset_x, self.offset_y, self.offset_z, self.offset_yaw)
        else:
            rospy.logwarn("No odometry data received yet, cannot reset")
        
        return EmptyResponse()

    def odom_callback(self, msg):
        """
        Callback function for odometry messages.
        Publishes a TF transform from map to base_link based on the odometry data.
        """
        self.last_odom = msg
        
        # Get current orientation as quaternion
        q = msg.pose.pose.orientation
        _, _, current_yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # Apply offset: rotate the position difference by negative offset_yaw
        dx = msg.pose.pose.position.x - self.offset_x
        dy = msg.pose.pose.position.y - self.offset_y
        
        cos_offset = np.cos(-self.offset_yaw)
        sin_offset = np.sin(-self.offset_yaw)
        
        new_x = dx * cos_offset - dy * sin_offset
        new_y = dx * sin_offset + dy * cos_offset
        new_z = msg.pose.pose.position.z - self.offset_z
        new_yaw = current_yaw - self.offset_yaw
        
        # Create new quaternion from adjusted yaw (keeping roll and pitch from original)
        roll, pitch, _ = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        new_q = tft.quaternion_from_euler(roll, pitch, new_yaw)
        
        t = TransformStamped()
        
        # Set header
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        
        # Set translation with offset applied
        t.transform.translation.x = new_x
        t.transform.translation.y = new_y
        t.transform.translation.z = new_z
        
        # Set rotation with offset applied
        t.transform.rotation.x = new_q[0]
        t.transform.rotation.y = new_q[1]
        t.transform.rotation.z = new_q[2]
        t.transform.rotation.w = new_q[3]
        
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
