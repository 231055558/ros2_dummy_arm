#!/usr/bin/env python3
"""
MoveIt自动运动序列演示脚本
按照启动指南中的目标1-4执行自动运动序列
"""

import rclpy
from rclpy.node import Node
import sys
import math
import time
from threading import Thread

# MoveIt相关导入
from moveit_msgs.msg import DisplayTrajectory, RobotState
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

# 使用MoveIt Python接口
try:
    from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
except ImportError:
    print("❌ 未找到moveit_commander，请安装: sudo apt install ros-humble-moveit-commander")
    sys.exit(1)


class MoveItSequenceDemo(Node):
    def __init__(self):
        super().__init__('moveit_sequence_demo')
        
        # 初始化MoveIt Commander
        rclpy.logging.set_logger_level('moveit_commander', rclpy.logging.LoggingSeverity.WARN)
        
        try:
            # 初始化robot commander
            self.robot = RobotCommander()
            
            # 初始化planning scene接口
            self.scene = PlanningSceneInterface()
            
            # 初始化move group commander
            self.move_group = MoveGroupCommander("dummy_arm")
            
            # 获取planning frame和end effector link
            self.planning_frame = self.move_group.get_planning_frame()
            self.eef_link = self.move_group.get_end_effector_link()
            
            self.get_logger().info(f"📝 Planning frame: {self.planning_frame}")
            self.get_logger().info(f"🎯 End effector link: {self.eef_link}")
            
        except Exception as e:
            self.get_logger().error(f"❌ MoveIt初始化失败: {e}")
            sys.exit(1)
        
        # 设置规划参数
        self.move_group.set_planning_time(10.0)  # 规划时间限制
        self.move_group.set_num_planning_attempts(10)  # 规划尝试次数
        self.move_group.set_goal_position_tolerance(0.01)  # 位置容差 1cm
        self.move_group.set_goal_orientation_tolerance(0.1)  # 姿态容差
        
        # 从启动指南中提取的目标位置（关节角度，弧度制）
        self.target_positions = {
            'reset': [0.0, -1.1276, 1.4052, 0.0, 0.0, 0.0],  # 重置位置
            
            # 目标1: joint1=24.85°, joint2=22.71°, joint3=50.36°, joint4=25.68°, joint5=75.83°, joint6=7.30°
            'target1': [
                math.radians(24.85),   # 0.434 rad
                math.radians(22.71),   # 0.396 rad  
                math.radians(50.36),   # 0.879 rad
                math.radians(25.68),   # 0.448 rad
                math.radians(75.83),   # 1.324 rad
                math.radians(7.30)     # 0.127 rad
            ],
            
            # 目标2: joint1=47.38°, joint2=-11.21°, joint3=-41.28°, joint4=139.12°, joint5=89.89°, joint6=153.47°
            'target2': [
                math.radians(47.38),   # 0.827 rad
                math.radians(-11.21),  # -0.196 rad
                math.radians(-41.28),  # -0.721 rad
                math.radians(139.12),  # 2.428 rad
                math.radians(89.89),   # 1.569 rad
                math.radians(153.47)   # 2.679 rad
            ],
            
            # 目标3: joint1=20.98°, joint2=-30.23°, joint3=-30.43°, joint4=161.43°, joint5=89.66°, joint6=169.94°
            'target3': [
                math.radians(20.98),   # 0.366 rad
                math.radians(-30.23),  # -0.528 rad
                math.radians(-30.43),  # -0.531 rad
                math.radians(161.43),  # 2.818 rad
                math.radians(89.66),   # 1.565 rad
                math.radians(169.94)   # 2.966 rad
            ],
            
            # 目标4: joint1=169.89°, joint2=-11.35°, joint3=65.22°, joint4=-0.09°, joint5=-37.67°, joint6=8.22°
            'target4': [
                math.radians(169.89),  # 2.965 rad
                math.radians(-11.35),  # -0.198 rad
                math.radians(65.22),   # 1.138 rad
                math.radians(-0.09),   # -0.002 rad
                math.radians(-37.67),  # -0.657 rad
                math.radians(8.22)     # 0.144 rad
            ]
        }
        
        self.get_logger().info('🤖 MoveIt运动序列演示器已启动')

    def wait_for_move_group(self, timeout=10.0):
        """等待MoveGroup连接"""
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            try:
                current_joints = self.move_group.get_current_joint_values()
                if current_joints:
                    return True
            except:
                time.sleep(0.1)
        return False

    def plan_to_joint_goal(self, joint_values, target_name=""):
        """规划到指定关节角度"""
        try:
            joint_degrees = [math.degrees(j) for j in joint_values]
            self.get_logger().info(f'📐 规划到{target_name}: {[f"{j:.1f}°" for j in joint_degrees]}')
            
            # 设置关节目标
            self.move_group.set_joint_value_target(joint_values)
            
            # 进行规划
            success, plan, planning_time, error_code = self.move_group.plan()
            
            if success:
                self.get_logger().info(f'✅ {target_name}规划成功，用时: {planning_time:.2f}秒')
                return plan
            else:
                self.get_logger().error(f'❌ {target_name}规划失败，错误码: {error_code}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'❌ {target_name}规划异常: {e}')
            return None

    def execute_plan(self, plan, target_name=""):
        """执行规划"""
        try:
            self.get_logger().info(f'🚀 执行{target_name}运动中...')
            success = self.move_group.execute(plan, wait=True)
            
            if success:
                self.get_logger().info(f'✅ {target_name}执行完成')
                self.move_group.stop()  # 确保停止
                return True
            else:
                self.get_logger().error(f'❌ {target_name}执行失败')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ {target_name}执行异常: {e}')
            return False

    def move_to_target(self, target_name):
        """移动到指定目标位置"""
        if target_name not in self.target_positions:
            self.get_logger().error(f'❌ 未知目标: {target_name}')
            return False
        
        joint_values = self.target_positions[target_name]
        
        # 规划
        plan = self.plan_to_joint_goal(joint_values, target_name)
        if not plan:
            return False
        
        # 执行
        return self.execute_plan(plan, target_name)

    def get_current_joint_angles_degrees(self):
        """获取当前关节角度（度数）"""
        try:
            current_joints = self.move_group.get_current_joint_values()
            return [math.degrees(j) for j in current_joints]
        except:
            return None

    def get_current_end_effector_pose(self):
        """获取当前末端执行器位姿"""
        try:
            current_pose = self.move_group.get_current_pose().pose
            return current_pose
        except:
            return None

    def display_current_status(self, step_name):
        """显示当前状态"""
        print("\n" + "="*60)
        print(f"📍 {step_name}")
        print("="*60)
        
        # 显示关节角度
        joint_angles = self.get_current_joint_angles_degrees()
        if joint_angles:
            print("📐 当前关节角度:")
            for i, angle in enumerate(joint_angles):
                print(f"  joint{i+1}: {angle:8.2f}°")
        
        # 显示末端位姿
        pose = self.get_current_end_effector_pose()
        if pose:
            print("\n🎯 末端执行器位姿:")
            print(f"  位置: ({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}) m")
            print(f"  姿态: ({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, {pose.orientation.z:.4f}, {pose.orientation.w:.4f})")
        
        print("="*60)

    def perform_sequence_demo(self):
        """执行完整的运动序列演示"""
        self.get_logger().info('🎭 开始执行启动指南运动序列演示')
        print("\n🎯 运动序列: 目标2 → 目标3 → 目标1 → 目标4 → 重置位置")
        print("⏱️  每个位置停留3秒")
        
        # 运动序列：2 → 3 → 1 → 4 → reset
        sequence = [
            ('target2', '目标2'),
            ('target3', '目标3'), 
            ('target1', '目标1'),
            ('target4', '目标4'),
            ('reset', '重置位置')
        ]
        
        for i, (target_key, target_name) in enumerate(sequence):
            step_num = i + 1
            total_steps = len(sequence)
            
            print(f"\n🔄 步骤 {step_num}/{total_steps}: 移动到{target_name}")
            
            # 移动到目标位置
            if not self.move_to_target(target_key):
                self.get_logger().error(f'❌ 移动到{target_name}失败，序列终止')
                return False
            
            # 显示当前状态
            self.display_current_status(f"已到达{target_name}")
            
            # 如果不是最后一个位置，等待3秒
            if step_num < total_steps:
                print(f"⏳ 在{target_name}停留3秒...")
                for remaining in range(3, 0, -1):
                    print(f"  倒计时: {remaining}秒", end='\r')
                    time.sleep(1.0)
                print("  继续下一个目标...     ")
            else:
                print(f"🏁 序列完成，已到达{target_name}")
        
        self.get_logger().info('🎉 启动指南运动序列演示完成')
        return True

    def show_menu(self):
        """显示菜单"""
        print("\n" + "="*70)
        print("🎭 MoveIt运动序列演示器")
        print("="*70)
        print("预设目标位置:")
        print("  1. target1   - 目标1位置")
        print("  2. target2   - 目标2位置") 
        print("  3. target3   - 目标3位置")
        print("  4. target4   - 目标4位置")
        print("  5. reset     - 重置位置")
        print()
        print("自动序列:")
        print("  6. sequence  - 执行完整序列 (2→3→1→4→reset)")
        print()
        print("信息:")
        print("  7. current   - 显示当前状态")
        print()
        print("  0. 退出")
        print("="*70)

    def show_current_state(self):
        """显示当前状态详情"""
        self.display_current_status("当前机械臂状态")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo = MoveItSequenceDemo()
        
        # 等待连接建立
        print("⏳ 等待MoveIt连接建立...")
        if not demo.wait_for_move_group():
            print("❌ MoveIt连接失败")
            return
        
        print("✅ MoveIt连接成功")
        
        # 启动ROS spin线程
        ros_thread = Thread(target=rclpy.spin, args=(demo,))
        ros_thread.daemon = True
        ros_thread.start()
        
        # 显示初始状态
        demo.show_current_state()
        
        # 主菜单循环
        while True:
            demo.show_menu()
            user_input = input("\n请输入命令: ").strip()
            
            if user_input == '0' or user_input.lower() in ['exit', 'quit']:
                break
            elif user_input == '1':
                demo.move_to_target('target1')
            elif user_input == '2':
                demo.move_to_target('target2')
            elif user_input == '3':
                demo.move_to_target('target3')
            elif user_input == '4':
                demo.move_to_target('target4')
            elif user_input == '5':
                demo.move_to_target('reset')
            elif user_input == '6':
                demo.perform_sequence_demo()
            elif user_input == '7':
                demo.show_current_state()
            else:
                print("❌ 未知命令")
    
    except KeyboardInterrupt:
        print("\n🛑 用户中断")
    except Exception as e:
        print(f"❌ 程序异常: {e}")
    finally:
        try:
            demo.destroy_node()
        except:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 