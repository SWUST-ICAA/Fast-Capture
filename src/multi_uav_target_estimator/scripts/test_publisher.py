#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
测试数据发布器
模拟4架无人机对运动目标的观测
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import numpy as np
import sys

class FakeDronePublisher:
    def __init__(self, num_drones=4):
        rospy.init_node('fake_drone_publisher', anonymous=True)
        
        self.num_drones = num_drones
        self.publishers = []
        
        # 创建发布器
        for i in range(num_drones):
            topic = '/drone_{}_target'.format(i)
            pub = rospy.Publisher(topic, Odometry, queue_size=10)
            self.publishers.append(pub)
            rospy.loginfo("Publishing to: {}".format(topic))
        
        # 参数
        self.noise_std = rospy.get_param('~noise_std', 0.1)  # 测量噪声标准差
        self.outlier_prob = rospy.get_param('~outlier_prob', 0.0)  # 异常值概率
        self.outlier_scale = rospy.get_param('~outlier_scale', 3.0)  # 异常值尺度
        
        # 运动模式
        self.motion_type = rospy.get_param('~motion_type', 'circle')  # circle, line, figure8
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("Fake Drone Publisher Started")
        rospy.loginfo("=" * 50)
        rospy.loginfo("Number of drones: {}".format(num_drones))
        rospy.loginfo("Noise std: {} m".format(self.noise_std))
        rospy.loginfo("Motion type: {}".format(self.motion_type))
        rospy.loginfo("Outlier probability: {}".format(self.outlier_prob))
        rospy.loginfo("=" * 50)
    
    def get_true_position(self, t):
        """根据时间生成真实位置"""
        if self.motion_type == 'circle':
            # 圆周运动
            x = 5.0 * np.cos(0.5 * t)
            y = 5.0 * np.sin(0.5 * t)
            z = 2.0 + 0.5 * np.sin(t)
        
        elif self.motion_type == 'line':
            # 直线运动
            x = 0.5 * t
            y = 0.3 * t
            z = 2.0 + 0.1 * np.sin(2.0 * t)
        
        elif self.motion_type == 'figure8':
            # 8字形运动
            x = 5.0 * np.sin(0.5 * t)
            y = 5.0 * np.sin(t)
            z = 2.0 + 0.5 * np.sin(t)
        
        elif self.motion_type == 'spiral':
            # 螺旋上升
            r = 5.0
            x = r * np.cos(0.5 * t)
            y = r * np.sin(0.5 * t)
            z = 0.0 + 0.3 * t
        
        else:
            # 静止
            x, y, z = 0.0, 0.0, 2.0
        
        return np.array([x, y, z])
    
    def add_noise(self, position):
        """添加测量噪声"""
        noise = np.random.randn(3) * self.noise_std
        
        # 以一定概率添加异常值
        if np.random.rand() < self.outlier_prob:
            noise *= self.outlier_scale
            rospy.logwarn("Outlier generated!")
        
        return position + noise
    
    def publish_measurements(self):
        """发布测量数据"""
        rate = rospy.Rate(10)  # 10 Hz
        t = 0.0
        dt = 0.1
        
        while not rospy.is_shutdown():
            # 获取真实位置
            true_pos = self.get_true_position(t)
            
            # 为每架无人机发布带噪声的测量
            for i, pub in enumerate(self.publishers):
                msg = Odometry()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'world'
                
                # 添加测量噪声 (每架无人机的噪声独立)
                measured_pos = self.add_noise(true_pos)
                
                msg.pose.pose.position.x = measured_pos[0]
                msg.pose.pose.position.y = measured_pos[1]
                msg.pose.pose.position.z = measured_pos[2]
                
                # 设置协方差 (对角阵)
                cov = [0.0] * 36
                cov[0] = self.noise_std ** 2   # x
                cov[7] = self.noise_std ** 2   # y
                cov[14] = self.noise_std ** 2  # z
                msg.pose.covariance = cov
                
                pub.publish(msg)
            
            # 每秒打印一次真实位置
            if int(t * 10) % 10 == 0:
                rospy.loginfo("t={:.1f}s | True position: [{:.2f}, {:.2f}, {:.2f}]".format(
                    t, true_pos[0], true_pos[1], true_pos[2]))
            
            t += dt
            rate.sleep()

def main():
    try:
        num_drones = rospy.get_param('~num_drones', 4)
        publisher = FakeDronePublisher(num_drones)
        publisher.publish_measurements()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()