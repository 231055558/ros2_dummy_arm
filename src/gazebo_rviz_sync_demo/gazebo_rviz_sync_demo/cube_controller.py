#!/usr/bin/env python3
"""
方块控制节点
让 Gazebo 中的方块自动移动，用于演示同步效果
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class CubeController(Node):
    """控制 Gazebo 中方块移动的节点"""
    
    def __init__(self):
        super().__init__('cube_controller')
        
        # 发布速度命令到方块
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/demo_cube/cmd_vel',
            10
        )
        
        # 创建定时器控制移动
        self.timer = self.create_timer(0.1, self.move_cube)
        
        # 移动模式参数
        self.start_time = time.time()
        self.mode = 'circle'  # 'circle', 'square', 'line'
        self.speed = 1.0
        
        self.get_logger().info('方块控制节点已启动 - 将开始自动移动方块')
        
        # 声明参数
        self.declare_parameter('movement_mode', 'circle')
        self.declare_parameter('movement_speed', 1.0)
    
    def move_cube(self):
        """控制方块移动"""
        current_time = time.time() - self.start_time
        
        # 获取参数
        mode = self.get_parameter('movement_mode').get_parameter_value().string_value
        speed = self.get_parameter('movement_speed').get_parameter_value().double_value
        
        cmd = Twist()
        
        if mode == 'circle':
            # 圆形移动
            radius = 2.0
            angular_freq = 0.5  # rad/s
            
            cmd.linear.x = radius * angular_freq * math.cos(angular_freq * current_time)
            cmd.linear.y = radius * angular_freq * math.sin(angular_freq * current_time)
            cmd.angular.z = 0.0
            
        elif mode == 'square':
            # 正方形移动
            cycle_time = 8.0  # 8秒完成一个正方形
            side_time = cycle_time / 4.0  # 每边2秒
            
            phase = (current_time % cycle_time) / side_time
            
            if phase < 1.0:  # 向前
                cmd.linear.x = speed
                cmd.linear.y = 0.0
            elif phase < 2.0:  # 向左
                cmd.linear.x = 0.0
                cmd.linear.y = speed
            elif phase < 3.0:  # 向后
                cmd.linear.x = -speed
                cmd.linear.y = 0.0
            else:  # 向右
                cmd.linear.x = 0.0
                cmd.linear.y = -speed
                
        elif mode == 'line':
            # 直线往返移动
            cycle_time = 4.0
            phase = (current_time % cycle_time) / cycle_time
            
            if phase < 0.5:
                cmd.linear.x = speed
            else:
                cmd.linear.x = -speed
            cmd.linear.y = 0.0
            
        elif mode == 'stop':
            # 停止
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
        
        # 发布命令
        self.cmd_publisher.publish(cmd)
    
    def change_mode(self, new_mode):
        """改变移动模式"""
        self.mode = new_mode
        self.start_time = time.time()  # 重置计时
        self.get_logger().info(f'切换到 {new_mode} 移动模式')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = CubeController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('方块控制节点被用户中断')
    finally:
        # 停止方块移动
        stop_cmd = Twist()
        node.cmd_publisher.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 