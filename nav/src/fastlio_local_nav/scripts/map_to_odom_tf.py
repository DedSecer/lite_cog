#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Map to Odom TF Publisher for Fast-LIO Navigation

功能:
1. 订阅 Fast-LIO 的 /Odometry 话题（camera_init 坐标系下的位姿）
2. 发布完整的 TF 链: map -> odom -> base_link
3. 支持通过 /initialpose 话题设置机器人在 map 中的初始位置

"""

import rospy
import tf2_ros
import tf.transformations as tft
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse


class MapToOdomTF:
    def __init__(self):
        rospy.init_node('map_to_odom_tf_node', anonymous=False)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Parameters - 初始位置（机器人在 map 中的起始位置）
        self.map_x = rospy.get_param('~init_x', 0.0)
        self.map_y = rospy.get_param('~init_y', 0.0)
        self.map_z = rospy.get_param('~init_z', 0.0)
        self.map_yaw = rospy.get_param('~init_yaw', 0.0)
        
        # 记录设置初始位置时 Fast-LIO 坐标系下的位置
        self.lio_offset_x = 0.0
        self.lio_offset_y = 0.0
        self.lio_offset_z = 0.0
        self.lio_offset_yaw = 0.0
        
        # 当前 Fast-LIO 的位姿（camera_init -> body）
        self.current_lio_pose = None
        self.initialized = False
        
        # 发布频率
        self.publish_rate = rospy.get_param('~publish_rate', 50.0)
        
        # 订阅 /initialpose 话题 (来自 rviz 的 2D Pose Estimate)
        self.initialpose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialpose_callback)
        
        # 订阅 Fast-LIO 的 Odometry 话题，获取当前位姿
        self.odom_sub = rospy.Subscriber('/Odometry', Odometry, self.odom_callback)
        
        # 重置服务
        self.reset_srv = rospy.Service('~reset_pose', Empty, self.reset_callback)
        
        rospy.loginfo("map_to_odom_tf_node started")
        rospy.loginfo("Initial pose: x=%.3f, y=%.3f, yaw=%.3f", self.map_x, self.map_y, self.map_yaw)
        rospy.loginfo("Publishing TF: map -> odom -> base_link")
        rospy.loginfo("Use rviz '2D Pose Estimate' to set robot position on map")
        
    def odom_callback(self, msg):
        """
        接收 Fast-LIO 发布的 Odometry，记录当前位姿
        这是 body 在 camera_init 坐标系中的位姿
        """
        self.current_lio_pose = msg
        
        # 首次接收时初始化
        if not self.initialized:
            self.lio_offset_x = msg.pose.pose.position.x
            self.lio_offset_y = msg.pose.pose.position.y
            self.lio_offset_z = msg.pose.pose.position.z
            q = msg.pose.pose.orientation
            _, _, self.lio_offset_yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.initialized = True
            rospy.loginfo("Initialized with first odometry message")
    
    def initialpose_callback(self, msg):
        """
        处理 /initialpose 话题（来自 rviz 2D Pose Estimate）
        设置机器人在 map 中的当前位置
        """
        if self.current_lio_pose is None:
            rospy.logwarn("No odometry data received yet, cannot set initial pose")
            return
        
        # 获取目标位置（在 map 坐标系中）
        self.map_x = msg.pose.pose.position.x
        self.map_y = msg.pose.pose.position.y
        self.map_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        _, _, self.map_yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # 记录当前 Fast-LIO 坐标系下的位置作为偏移
        self.lio_offset_x = self.current_lio_pose.pose.pose.position.x
        self.lio_offset_y = self.current_lio_pose.pose.pose.position.y
        self.lio_offset_z = self.current_lio_pose.pose.pose.position.z
        q_lio = self.current_lio_pose.pose.pose.orientation
        _, _, self.lio_offset_yaw = tft.euler_from_quaternion([q_lio.x, q_lio.y, q_lio.z, q_lio.w])
        
        rospy.loginfo("Initial pose set to: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f (%.1f deg)",
                      self.map_x, self.map_y, self.map_z, self.map_yaw, np.degrees(self.map_yaw))
    
    def reset_callback(self, req):
        """
        重置到初始参数位置
        """
        if self.current_lio_pose is not None:
            self.map_x = rospy.get_param('~init_x', 0.0)
            self.map_y = rospy.get_param('~init_y', 0.0)
            self.map_z = rospy.get_param('~init_z', 0.0)
            self.map_yaw = rospy.get_param('~init_yaw', 0.0)
            
            self.lio_offset_x = self.current_lio_pose.pose.pose.position.x
            self.lio_offset_y = self.current_lio_pose.pose.pose.position.y
            self.lio_offset_z = self.current_lio_pose.pose.pose.position.z
            q = self.current_lio_pose.pose.pose.orientation
            _, _, self.lio_offset_yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            rospy.loginfo("Pose reset to: x=%.3f, y=%.3f, yaw=%.3f", 
                          self.map_x, self.map_y, self.map_yaw)
        return EmptyResponse()
    
    def compute_map_to_odom_tf(self):
        """
        计算 map -> odom 的变换
        
        这里 odom 等同于 Fast-LIO 的 camera_init frame
        
        已知:
        - 当前 base_link 在 map 中的期望位置: (map_x, map_y, map_yaw)
        - 设置初始位置时 base_link 在 LIO 坐标系中的位置: (lio_offset_x, lio_offset_y, lio_offset_yaw)
        
        需要计算 map -> odom 的变换
        """
        # 计算 lio_offset 的逆变换
        cos_offset = np.cos(self.lio_offset_yaw)
        sin_offset = np.sin(self.lio_offset_yaw)
        
        # lio_base 的逆: 先反向旋转，再反向平移
        inv_x = -(self.lio_offset_x * cos_offset + self.lio_offset_y * sin_offset)
        inv_y = -(-self.lio_offset_x * sin_offset + self.lio_offset_y * cos_offset)
        inv_yaw = -self.lio_offset_yaw
        
        # map_odom = map_base * base_odom (base_odom 是 lio_base 的逆)
        cos_map = np.cos(self.map_yaw)
        sin_map = np.sin(self.map_yaw)
        
        tf_x = self.map_x + inv_x * cos_map - inv_y * sin_map
        tf_y = self.map_y + inv_x * sin_map + inv_y * cos_map
        tf_z = self.map_z - self.lio_offset_z
        tf_yaw = self.map_yaw + inv_yaw
        
        return tf_x, tf_y, tf_z, tf_yaw
    
    def publish_tf(self):
        """
        发布 TF 变换:
        1. map -> odom (动态计算，支持初始位置设置)
        2. odom -> base_link (来自 Fast-LIO 的位姿)
        """
        if not self.initialized or self.current_lio_pose is None:
            return
        
        now = rospy.Time.now()
        
        # 1. 发布 map -> odom
        tf_x, tf_y, tf_z, tf_yaw = self.compute_map_to_odom_tf()
        
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = "map"
        t_map_odom.child_frame_id = "odom"
        
        t_map_odom.transform.translation.x = tf_x
        t_map_odom.transform.translation.y = tf_y
        t_map_odom.transform.translation.z = tf_z
        
        q = tft.quaternion_from_euler(0, 0, tf_yaw)
        t_map_odom.transform.rotation.x = q[0]
        t_map_odom.transform.rotation.y = q[1]
        t_map_odom.transform.rotation.z = q[2]
        t_map_odom.transform.rotation.w = q[3]
        
        # 2. 发布 odom -> base_link (直接使用 Fast-LIO 的位姿)
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = now
        t_odom_base.header.frame_id = "odom"
        t_odom_base.child_frame_id = "base_link"
        
        t_odom_base.transform.translation.x = self.current_lio_pose.pose.pose.position.x
        t_odom_base.transform.translation.y = self.current_lio_pose.pose.pose.position.y
        t_odom_base.transform.translation.z = self.current_lio_pose.pose.pose.position.z
        t_odom_base.transform.rotation = self.current_lio_pose.pose.pose.orientation
        
        # 同时发布两个 TF
        self.tf_broadcaster.sendTransform([t_map_odom, t_odom_base])
    
    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.publish_tf()
            rate.sleep()


if __name__ == '__main__':
    try:
        node = MapToOdomTF()
        node.run()
    except rospy.ROSInterruptException:
        pass
