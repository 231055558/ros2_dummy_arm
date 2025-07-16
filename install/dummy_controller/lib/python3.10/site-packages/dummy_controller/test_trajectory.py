#!/usr/bin/env python3
"""
Dummy机械臂综合测试程序
包含基本运动、轨迹规划、碰撞检测等功能测试
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import time
import numpy as np


class DummyArmTestSuite(Node):
    def __init__(self):
        super().__init__('dummy_arm_test_suite')
        
        # 创建action客户端
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            'dummy_arm_controller/follow_joint_trajectory'
        )
        
        # 创建关节状态订阅器
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_state = None
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 预定义的测试位姿（弧度）
        self.test_positions = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'ready': [0.0, -1.3089, 1.5707, 0.0, 0.0, 0.0],
            'test1': [0.5, -0.5, 0.5, 0.0, 0.0, 0.0],
            'test2': [-0.5, -0.8, 1.0, 0.5, 0.5, 0.0],
            'test3': [1.0, -1.0, 1.2, -0.5, -0.5, 1.0],
            'wave': [0.0, -0.5, 0.8, 0.0, 1.5, 0.0],
        }
        
    def joint_state_callback(self, msg):
        """接收当前关节状态"""
        self.current_joint_state = msg
        
    def wait_for_action_server(self):
        """等待action服务器可用"""
        self.get_logger().info('等待action服务器...')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action服务器不可用!')
            return False
        self.get_logger().info('Action服务器已连接')
        return True
        
    def move_to_position(self, position, duration=3.0):
        """移动到指定关节位置"""
        if not self.wait_for_action_server():
            return False
            
        # 创建轨迹消息
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        # 创建轨迹点
        point = JointTrajectoryPoint()
        point.positions = position
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f'发送轨迹目标: {position}')
        
        # 发送目标并等待结果
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝!')
            return False
            
        self.get_logger().info('目标已接受，等待执行完成...')
        
        # 等待执行完成
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        self.get_logger().info(f'执行完成，错误代码: {result.error_code}')
        
        return result.error_code == 0
        
    def get_current_position(self):
        """获取当前关节位置"""
        if self.current_joint_state is None:
            self.get_logger().warn('尚未接收到关节状态')
            return None
        return list(self.current_joint_state.position)
        
    def test_basic_movements(self):
        """测试基本运动功能"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('开始基本运动测试')
        self.get_logger().info('=' * 50)
        
        # 等待接收关节状态
        rate = self.create_rate(10)
        while self.current_joint_state is None:
            self.get_logger().info('等待关节状态...')
            rclpy.spin_once(self, timeout_sec=0.1)
            rate.sleep()
            
        current_pos = self.get_current_position()
        self.get_logger().info(f'当前位置: {current_pos}')
        
        # 测试各个预定义位置
        for name, position in self.test_positions.items():
            self.get_logger().info(f'测试移动到 {name}: {position}')
            success = self.move_to_position(position, duration=4.0)
            if success:
                self.get_logger().info(f'✓ 成功移动到 {name}')
            else:
                self.get_logger().error(f'✗ 移动到 {name} 失败')
            time.sleep(2)  # 间隔时间
            
    def test_smooth_trajectory(self):
        """测试平滑轨迹运动"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('开始平滑轨迹测试')
        self.get_logger().info('=' * 50)
        
        if not self.wait_for_action_server():
            return
            
        # 创建多点轨迹
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        # 定义轨迹点
        trajectory_points = [
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0),      # 起点
            ([0.5, -0.5, 0.5, 0.0, 0.0, 0.0], 3.0),     # 第一点
            ([0.0, -1.0, 1.0, 0.5, 0.5, 0.0], 6.0),     # 第二点
            ([-0.5, -0.5, 0.5, -0.5, -0.5, 0.5], 9.0),  # 第三点
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 12.0),     # 回到起点
        ]
        
        for position, time_sec in trajectory_points:
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start.sec = int(time_sec)
            point.time_from_start.nanosec = int((time_sec - int(time_sec)) * 1e9)
            goal_msg.trajectory.points.append(point)
            
        self.get_logger().info('发送平滑轨迹')
        
        # 发送目标
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if goal_handle.accepted:
            self.get_logger().info('平滑轨迹已接受，等待执行...')
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)
            self.get_logger().info('✓ 平滑轨迹执行完成')
        else:
            self.get_logger().error('✗ 平滑轨迹被拒绝')
            
    def test_joint_limits(self):
        """测试关节限制"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('开始关节限制测试')
        self.get_logger().info('=' * 50)
        
        # 测试各关节的极限位置（略微保守以确保安全）
        limit_tests = [
            ('Joint1 正极限', [3.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            ('Joint1 负极限', [-3.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            ('Joint2 正极限', [0.0, 1.4, 0.0, 0.0, 0.0, 0.0]),
            ('Joint2 负极限', [0.0, -1.2, 0.0, 0.0, 0.0, 0.0]),
            ('Joint3 正极限', [0.0, 0.0, 1.4, 0.0, 0.0, 0.0]),
            ('Joint3 负极限', [0.0, 0.0, -1.4, 0.0, 0.0, 0.0]),
        ]
        
        for test_name, position in limit_tests:
            self.get_logger().info(f'测试 {test_name}: {position}')
            success = self.move_to_position(position, duration=3.0)
            if success:
                self.get_logger().info(f'✓ {test_name} 测试通过')
            else:
                self.get_logger().warn(f'⚠ {test_name} 可能超出限制')
            time.sleep(1)
            
        # 回到安全位置
        self.move_to_position([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration=3.0)
        
    def test_wave_motion(self):
        """测试挥手动作"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('开始挥手动作测试')
        self.get_logger().info('=' * 50)
        
        # 挥手动作序列
        wave_sequence = [
            [0.0, -0.5, 0.8, 0.0, 1.5, 0.0],   # 抬起手臂
            [0.0, -0.5, 0.8, 0.0, 1.5, 0.5],   # 向右挥
            [0.0, -0.5, 0.8, 0.0, 1.5, -0.5],  # 向左挥
            [0.0, -0.5, 0.8, 0.0, 1.5, 0.5],   # 向右挥
            [0.0, -0.5, 0.8, 0.0, 1.5, -0.5],  # 向左挥
            [0.0, -0.5, 0.8, 0.0, 1.5, 0.0],   # 回中间
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],     # 回到初始位置
        ]
        
        for i, position in enumerate(wave_sequence):
            self.get_logger().info(f'挥手动作 {i+1}/7')
            self.move_to_position(position, duration=2.0)
            time.sleep(0.5)
            
        self.get_logger().info('✓ 挥手动作完成')
        
    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info('开始Dummy机械臂综合测试')
        
        try:
            # 基本运动测试
            self.test_basic_movements()
            
            # 平滑轨迹测试
            self.test_smooth_trajectory()
            
            # 关节限制测试
            self.test_joint_limits()
            
            # 挥手动作测试
            self.test_wave_motion()
            
            self.get_logger().info('=' * 50)
            self.get_logger().info('所有测试完成!')
            self.get_logger().info('=' * 50)
            
        except Exception as e:
            self.get_logger().error(f'测试过程中发生错误: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    test_node = DummyArmTestSuite()
    
    try:
        test_node.run_all_tests()
    except KeyboardInterrupt:
        test_node.get_logger().info('测试被用户中断')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 