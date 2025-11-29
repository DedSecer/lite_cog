#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf.transformations as tft
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
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
        
        # Initial pose in map frame (set via parameter or initialpose topic)
        self.init_pos_x = rospy.get_param('~init_pos_x', 0.0)
        self.init_pos_y = rospy.get_param('~init_pos_y', 0.0)
        self.init_pos_z = rospy.get_param('~init_pos_z', 0.0)
        self.init_yaw = rospy.get_param('~init_yaw', 0.0)
        
        # Store the last received odometry for reset
        self.last_odom = None
        
        # Reset service
        self.reset_srv = rospy.Service('~reset_odom', Empty, self.reset_callback)
        
        # Subscribe to /initialpose topic (from rviz "2D Pose Estimate")
        self.initialpose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialpose_callback)
        
        # Subscribe to /leg_odom2 topic
        self.odom_sub = rospy.Subscriber('/leg_odom2', Odometry, self.odom_callback)
        
        rospy.loginfo("odom_to_tf_node started, subscribing to /leg_odom2")
        rospy.loginfo("Initial pose: x=%.3f, y=%.3f, yaw=%.3f", self.init_pos_x, self.init_pos_y, self.init_yaw)
        rospy.loginfo("Use rviz '2D Pose Estimate' or service '~reset_odom' to set position")

    def initialpose_callback(self, msg):
        """
        Callback for /initialpose topic (from rviz 2D Pose Estimate).
        Sets the current position to the specified pose in map frame.
        """
        if self.last_odom is None:
            rospy.logwarn("No odometry data received yet, cannot set initial pose")
            return
        
        # Get target pose from message
        target_x = msg.pose.pose.position.x
        target_y = msg.pose.pose.position.y
        target_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        _, _, target_yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # Update initial pose
        self.init_pos_x = target_x
        self.init_pos_y = target_y
        self.init_pos_z = target_z
        self.init_yaw = target_yaw
        
        # Record current odometry as offset
        self.offset_x = self.last_odom.pose.pose.position.x
        self.offset_y = self.last_odom.pose.pose.position.y
        self.offset_z = self.last_odom.pose.pose.position.z
        q_odom = self.last_odom.pose.pose.orientation
        _, _, self.offset_yaw = tft.euler_from_quaternion([q_odom.x, q_odom.y, q_odom.z, q_odom.w])
        
        rospy.loginfo("Initial pose set to: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f",
                      target_x, target_y, target_z, target_yaw)

    def reset_callback(self, req):
        """
        Service callback to reset odometry.
        Records the current position as offset so that current position becomes the initial pose.
        """
        if self.last_odom is not None:
            self.offset_x = self.last_odom.pose.pose.position.x
            self.offset_y = self.last_odom.pose.pose.position.y
            self.offset_z = self.last_odom.pose.pose.position.z
            
            # Get current yaw
            q = self.last_odom.pose.pose.orientation
            _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.offset_yaw = yaw
            
            rospy.loginfo("Odometry reset! Current pose set to initial pose: x=%.3f, y=%.3f, yaw=%.3f",
                          self.init_pos_x, self.init_pos_y, self.init_yaw)
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
        
        # Transform to initial pose frame, then add initial position
        rotated_x = dx * cos_offset - dy * sin_offset
        rotated_y = dx * sin_offset + dy * cos_offset
        
        # Apply initial pose rotation to get final position
        cos_init = np.cos(self.init_yaw)
        sin_init = np.sin(self.init_yaw)
        
        new_x = self.init_pos_x + rotated_x * cos_init - rotated_y * sin_init
        new_y = self.init_pos_y + rotated_x * sin_init + rotated_y * cos_init
        new_z = self.init_pos_z + msg.pose.pose.position.z - self.offset_z
        new_yaw = self.init_yaw + (current_yaw - self.offset_yaw)
        
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
