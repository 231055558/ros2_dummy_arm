#!/usr/bin/env python3
"""
MoveIt RViz自动规划执行器
通过RViz的MoveIt环境进行规划，规划成功后自动执行
使用RViz中的障碍物检查和安全规划
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import time
from threading import Thread

# MoveIt相关服务和消息
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene
from moveit_msgs.msg import (
    MotionPlanRequest, 
    WorkspaceParameters, 
    Constraints,
    JointConstraint,
    RobotState,
    DisplayTrajectory
)
from moveit_msgs.action import MoveGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from std_srvs.srv import SetBool


class RVizMoveItDemo(Node):
    def __init__(self):
        super().__init__('rviz_moveit_demo')
        
        # MoveIt规划服务客户端 - 这个会使用RViz中的障碍物
        self.plan_service = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # MoveGroup Action客户端 - 用于执行规划好的轨迹
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # 夹爪控制服务客户端
        self.gripper_open_service = self.create_client(SetBool, 'dummy_arm/gripper_open')
        self.gripper_close_service = self.create_client(SetBool, 'dummy_arm/gripper_close')
        
        # 发布规划结果到RViz显示
        self.display_trajectory_publisher = self.create_publisher(
            DisplayTrajectory, 
            '/move_group/display_planned_path', 
            10
        )
        
        # 订阅当前关节状态
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_positions = None
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
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
        
        self.get_logger().info('🎯 RViz MoveIt自动规划执行器已启动')

    def joint_state_callback(self, msg):
        """接收当前关节状态"""
        if len(msg.position) >= 6:
            self.current_joint_positions = list(msg.position[:6])

    def wait_for_services(self, timeout=10.0):
        """等待MoveIt服务连接"""
        self.get_logger().info('⏳ 等待MoveIt服务连接...')
        
        if not self.plan_service.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('❌ MoveIt规划服务不可用')
            return False
            
        if not self.move_group_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('❌ MoveGroup Action服务不可用')
            return False
            
        if not self.gripper_open_service.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('❌ 夹爪打开服务不可用')
            return False
            
        if not self.gripper_close_service.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('❌ 夹爪关闭服务不可用')
            return False
            
        self.get_logger().info('✅ MoveIt服务连接成功')
        return True

    def create_motion_plan_request(self, target_positions):
        """创建运动规划请求"""
        request = GetMotionPlan.Request()
        
        # 设置运动规划请求
        motion_plan_request = MotionPlanRequest()
        
        # 设置群组名称
        motion_plan_request.group_name = "dummy_arm"
        
        # 设置起始状态（当前状态）
        motion_plan_request.start_state.joint_state.header = Header()
        motion_plan_request.start_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        motion_plan_request.start_state.joint_state.name = self.joint_names
        motion_plan_request.start_state.joint_state.position = self.current_joint_positions or [0.0] * 6

        # 设置目标关节约束
        joint_constraints = []
        for i, (name, position) in enumerate(zip(self.joint_names, target_positions)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            joint_constraints.append(joint_constraint)
        
        goal_constraints = Constraints()
        goal_constraints.joint_constraints = joint_constraints
        motion_plan_request.goal_constraints = [goal_constraints]
        
        # 设置工作空间参数
        motion_plan_request.workspace_parameters = WorkspaceParameters()
        motion_plan_request.workspace_parameters.header.frame_id = "base_link"
        motion_plan_request.workspace_parameters.min_corner.x = -2.0
        motion_plan_request.workspace_parameters.min_corner.y = -2.0 
        motion_plan_request.workspace_parameters.min_corner.z = -2.0
        motion_plan_request.workspace_parameters.max_corner.x = 2.0
        motion_plan_request.workspace_parameters.max_corner.y = 2.0
        motion_plan_request.workspace_parameters.max_corner.z = 2.0
        
        # 设置规划器参数
        motion_plan_request.planner_id = "RRTConnectkConfigDefault"
        motion_plan_request.num_planning_attempts = 10
        motion_plan_request.allowed_planning_time = 5.0
        motion_plan_request.max_velocity_scaling_factor = 0.1
        motion_plan_request.max_acceleration_scaling_factor = 0.1
        
        request.motion_plan_request = motion_plan_request
        return request

    def create_move_group_goal(self, target_positions):
        """创建MoveGroup执行目标"""
        goal = MoveGroup.Goal()
        
        # 创建规划请求
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = "dummy_arm"
        
        # 设置起始状态
        motion_plan_request.start_state.joint_state.header = Header()
        motion_plan_request.start_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        motion_plan_request.start_state.joint_state.name = self.joint_names
        motion_plan_request.start_state.joint_state.position = self.current_joint_positions or [0.0] * 6

        # 设置目标关节约束
        joint_constraints = []
        for i, (name, position) in enumerate(zip(self.joint_names, target_positions)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            joint_constraints.append(joint_constraint)
        
        goal_constraints = Constraints()
        goal_constraints.joint_constraints = joint_constraints
        motion_plan_request.goal_constraints = [goal_constraints]
        
        # 设置规划器参数
        motion_plan_request.planner_id = "RRTConnectkConfigDefault"
        motion_plan_request.num_planning_attempts = 10
        motion_plan_request.allowed_planning_time = 10.0  # 增加规划时间
        motion_plan_request.max_velocity_scaling_factor = 0.3  # 提高速度
        motion_plan_request.max_acceleration_scaling_factor = 0.3  # 提高加速度
        
        goal.request = motion_plan_request
        goal.planning_options.plan_only = False  # 规划并执行
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 10  # 增加重新规划次数
        
        return goal

    def plan_and_execute_target(self, target_name):
        """规划并自动执行到目标位置"""
        if target_name not in self.target_positions:
            self.get_logger().error(f'❌ 未知目标: {target_name}')
            return False
        
        target_positions = self.target_positions[target_name]
        joint_degrees = [math.degrees(j) for j in target_positions]
        
        self.get_logger().info(f'🎯 通过RViz规划并执行到{target_name}: {[f"{j:.1f}°" for j in joint_degrees]}')
        
        try:
            # 创建MoveGroup目标
            goal = self.create_move_group_goal(target_positions)
            
            # 发送目标到MoveGroup
            self.get_logger().info(f'📤 发送{target_name}目标到MoveGroup...')
            send_goal_future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
            
            if send_goal_future.result() is None:
                self.get_logger().error(f'❌ {target_name}目标发送超时')
                return False
                
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'❌ {target_name}目标被拒绝')
                return False
            
            self.get_logger().info(f'✅ {target_name}目标已接受，开始规划和执行...')
            
            # 等待执行完成 - 增加超时时间
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=60.0)
            
            if get_result_future.result() is None:
                self.get_logger().error(f'❌ {target_name}执行超时')
                return False
                
            result = get_result_future.result()
            
            if result.result.error_code.val == 1:  # SUCCESS
                self.get_logger().info(f'🎉 {target_name}规划和执行成功完成！')
                return True
            else:
                error_code = result.result.error_code.val
                self.get_logger().error(f'❌ {target_name}执行失败，错误码: {error_code}')
                if error_code == -1:
                    self.get_logger().error('   可能原因：目标位置不可达或存在碰撞')
                elif error_code == -2:
                    self.get_logger().error('   可能原因：规划超时')
                elif error_code == -3:
                    self.get_logger().error('   可能原因：无效的机器人状态')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ {target_name}规划执行异常: {e}')
            import traceback
            traceback.print_exc()
            return False

    def control_gripper(self, close_gripper=True):
        """控制夹爪开关"""
        try:
            request = SetBool.Request()
            request.data = True  # 服务只需要触发，数据内容不重要
            
            action_name = "关闭" if close_gripper else "打开"
            service_client = self.gripper_close_service if close_gripper else self.gripper_open_service
            
            self.get_logger().info(f'🤏 {action_name}夹爪...')
            
            future = service_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is None:
                self.get_logger().error(f'❌ 夹爪{action_name}请求超时')
                return False
                
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ 夹爪{action_name}成功')
                return True
            else:
                self.get_logger().error(f'❌ 夹爪{action_name}失败: {response.message}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ 夹爪控制异常: {e}')
            return False

    def get_current_joint_angles_degrees(self):
        """获取当前关节角度（度数）"""
        if self.current_joint_positions:
            return [math.degrees(j) for j in self.current_joint_positions]
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
        else:
            print("⚠️  等待关节状态数据...")
        
        print("="*60)

    def perform_sequence_demo(self):
        """执行完整的自动序列演示"""
        self.get_logger().info('🎭 开始执行RViz自动规划执行序列')
        print("\n🎯 运动序列: 目标2 → 目标3 → 目标1 → 目标4 → 重置位置")
        print("🤖 通过RViz MoveIt环境规划，规划成功后自动执行")
        print("🤏 特殊操作：目标2处关闭夹爪，目标4处打开夹爪")
        
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
            
            print(f"\n🔄 步骤 {step_num}/{total_steps}: 规划并执行到{target_name}")
            
            # 规划并执行到目标位置
            if not self.plan_and_execute_target(target_key):
                self.get_logger().error(f'❌ {target_name}失败，序列终止')
                return False
            
            # 显示当前状态
            self.display_current_status(f"已到达{target_name}")
            
            # 特殊操作：在目标2关闭夹爪，在目标4打开夹爪
            if target_key == 'target1':
                print("🤏 到达目标2，3秒后关闭夹爪...")
                time.sleep(3.0)
                self.control_gripper(close_gripper=True)  # 关闭夹爪
            elif target_key == 'target4':
                print("🤏 到达目标4，3秒后打开夹爪...")
                time.sleep(3.0)
                self.control_gripper(close_gripper=False)  # 打开夹爪
            
            # 如果不是最后一个位置，等待3秒
            if step_num < total_steps:
                if target_key not in ['target2', 'target4']:  # 目标2和4已经等待过3秒了
                    print(f"⏳ 在{target_name}停留3秒...")
                    for remaining in range(3, 0, -1):
                        print(f"  倒计时: {remaining}秒", end='\r')
                        time.sleep(1.0)
                    print("  继续下一个目标...     ")
                else:
                    print("  继续下一个目标...")
            else:
                print(f"🏁 序列完成，已到达{target_name}")
        
        self.get_logger().info('🎉 RViz自动规划执行序列完成')
        return True

    def show_menu(self):
        """显示菜单"""
        print("\n" + "="*70)
        print("🎯 RViz MoveIt自动规划执行器")
        print("="*70)
        print("💡 通过RViz MoveIt环境规划，规划成功后自动执行")
        print("🛡️  使用RViz中的障碍物进行安全检查")
        print("="*70)
        print("目标位置:")
        print("  1. target1   - 规划并执行到目标1位置")
        print("  2. target2   - 规划并执行到目标2位置") 
        print("  3. target3   - 规划并执行到目标3位置")
        print("  4. target4   - 规划并执行到目标4位置")
        print("  5. reset     - 规划并执行到重置位置")
        print()
        print("自动序列:")
        print("  6. sequence  - 执行完整序列 (2→3→1→4→reset)")
        print()
        print("夹爪控制:")
        print("  8. close     - 关闭夹爪")
        print("  9. open      - 打开夹爪")
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
        demo = RVizMoveItDemo()
        
        # 等待MoveIt服务连接
        if not demo.wait_for_services():
            print("❌ MoveIt服务连接失败，请确认已启动demo_real_arm.launch.py")
            return
        
        # 启动ROS spin线程
        ros_thread = Thread(target=rclpy.spin, args=(demo,))
        ros_thread.daemon = True
        ros_thread.start()
        
        # 等待获取初始关节状态
        print("⏳ 等待关节状态数据...")
        for _ in range(50):  # 等待5秒
            if demo.current_joint_positions:
                break
            time.sleep(0.1)
        
        # 显示初始状态
        demo.show_current_state()
        
        # 主菜单循环
        while True:
            demo.show_menu()
            user_input = input("\n请输入命令: ").strip()
            
            if user_input == '0' or user_input.lower() in ['exit', 'quit']:
                break
            elif user_input == '1':
                demo.plan_and_execute_target('target1')
            elif user_input == '2':
                demo.plan_and_execute_target('target2')
            elif user_input == '3':
                demo.plan_and_execute_target('target3')
            elif user_input == '4':
                demo.plan_and_execute_target('target4')
            elif user_input == '5':
                demo.plan_and_execute_target('reset')
            elif user_input == '6':
                demo.perform_sequence_demo()
            elif user_input == '7':
                demo.show_current_state()
            elif user_input == '8' or user_input.lower() == 'close':
                demo.control_gripper(close_gripper=True)
            elif user_input == '9' or user_input.lower() == 'open':
                demo.control_gripper(close_gripper=False)
            else:
                print("❌ 未知命令")
    
    except KeyboardInterrupt:
        print("\n🛑 用户中断")
    except Exception as e:
        print(f"❌ 程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            demo.destroy_node()
        except:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 