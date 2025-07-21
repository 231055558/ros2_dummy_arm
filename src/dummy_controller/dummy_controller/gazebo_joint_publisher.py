#!/usr/bin/env python3
"""
简单的Gazebo关节位置发布器
用于测试Gazebo中的机械臂关节控制
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time


class GazeboJointPublisher(Node):
    def __init__(self):
        super().__init__('gazebo_joint_publisher')
        
        # 创建发布器
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/dummy_arm_controller/commands',
            10
        )
        
        # 创建定时器，每秒发布一次
        self.timer = self.create_timer(1.0, self.publish_joint_positions)
        
        # 关节位置（初始化为全零）
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.step = 0
        
        self.get_logger().info('Gazebo关节发布器已启动')
        self.get_logger().info('发布话题: /dummy_arm_controller/commands')
        
    def publish_joint_positions(self):
        """发布关节位置"""
        msg = Float64MultiArray()
        
        # 简单的测试序列：让第一个关节慢慢旋转
        if self.step < 50:  # 前50秒保持初始位置
            self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            # 让第一个关节慢慢旋转
            angle = (self.step - 50) * 0.1  # 每秒增加0.1弧度
            self.joint_positions[0] = math.sin(angle * 0.5) * 1.57  # 摆动-90到90度
            
        msg.data = self.joint_positions
        self.publisher.publish(msg)
        
        self.get_logger().info(f'发布关节位置: {[f"{pos:.2f}" for pos in self.joint_positions]}')
        self.step += 1


def main(args=None):
    rclpy.init(args=args)
    
    node = GazeboJointPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 