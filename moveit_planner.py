#!/usr/bin/env python3
"""
MoveIt路径规划脚本
在RViz环境中使用MoveIt进行自动路径规划和执行
"""

import rclpy
from rclpy.node import Node
import sys
import math
import time
from threading import Thread

# MoveIt相关导入
from moveit_msgs.msg import DisplayTrajectory, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

# 使用MoveIt Python接口
try:
    from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
    from moveit_commander.conversions import pose_to_list, list_to_pose
except ImportError:
    print("❌ 未找到moveit_commander，请安装: sudo apt install ros-humble-moveit-commander")
    sys.exit(1)


class MoveItPlanner(Node):
    def __init__(self):
        super().__init__('moveit_planner')
        
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
        
        # 夹爪控制服务客户端
        self.gripper_open_client = self.create_client(SetBool, 'dummy_arm/gripper_open')
        self.gripper_close_client = self.create_client(SetBool, 'dummy_arm/gripper_close')
        
        # 订阅当前关节状态
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_values = None
        self.current_pose = None
        
        # 设置规划参数
        self.move_group.set_planning_time(10.0)  # 规划时间限制
        self.move_group.set_num_planning_attempts(10)  # 规划尝试次数
        self.move_group.set_goal_position_tolerance(0.01)  # 位置容差 1cm
        self.move_group.set_goal_orientation_tolerance(0.1)  # 姿态容差
        
        # 预定义的关节位置
        self.joint_positions = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'ready': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'reset': [0.0, -1.1276, 1.4052, 0.0, 0.0, 0.0],
            'pick_pose': [0.0, -0.5, 0.8, 0.0, 0.3, 0.0],
            'place_pose': [1.57, -0.5, 0.8, 0.0, 0.3, 0.0],
        }
        
        # 预定义的笛卡尔位置
        self.cartesian_poses = {
            'front_high': {'position': [0.3, 0.0, 0.4], 'orientation': [0.0, 0.0, 0.0, 1.0]},
            'front_low': {'position': [0.3, 0.0, 0.2], 'orientation': [0.0, 0.0, 0.0, 1.0]},
            'left_side': {'position': [0.2, 0.3, 0.3], 'orientation': [0.0, 0.0, 0.707, 0.707]},
            'right_side': {'position': [0.2, -0.3, 0.3], 'orientation': [0.0, 0.0, -0.707, 0.707]},
        }
        
        self.get_logger().info('🤖 MoveIt规划器已启动')

    def joint_state_callback(self, msg):
        """接收当前关节状态"""
        if len(msg.position) >= 6:
            self.current_joint_values = list(msg.position[:6])
            # 更新当前末端位姿
            try:
                self.current_pose = self.move_group.get_current_pose().pose
            except:
                pass

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

    def plan_to_joint_goal(self, joint_values):
        """规划到指定关节角度"""
        try:
            self.get_logger().info(f'📐 规划到关节位置: {[f"{j:.3f}" for j in joint_values]}')
            
            # 设置关节目标
            self.move_group.set_joint_value_target(joint_values)
            
            # 进行规划
            success, plan, planning_time, error_code = self.move_group.plan()
            
            if success:
                self.get_logger().info(f'✅ 规划成功，用时: {planning_time:.2f}秒')
                return plan
            else:
                self.get_logger().error(f'❌ 规划失败，错误码: {error_code}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'❌ 规划异常: {e}')
            return None

    def plan_to_pose_goal(self, target_pose):
        """规划到指定位姿"""
        try:
            if isinstance(target_pose, dict):
                # 从字典创建Pose
                pose = Pose()
                pose.position.x = target_pose['position'][0]
                pose.position.y = target_pose['position'][1]
                pose.position.z = target_pose['position'][2]
                pose.orientation.x = target_pose['orientation'][0]
                pose.orientation.y = target_pose['orientation'][1]
                pose.orientation.z = target_pose['orientation'][2]
                pose.orientation.w = target_pose['orientation'][3]
                target_pose = pose
            
            self.get_logger().info(f'🎯 规划到位姿: 位置({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})')
            
            # 设置位姿目标
            self.move_group.set_pose_target(target_pose)
            
            # 进行规划
            success, plan, planning_time, error_code = self.move_group.plan()
            
            if success:
                self.get_logger().info(f'✅ 规划成功，用时: {planning_time:.2f}秒')
                return plan
            else:
                self.get_logger().error(f'❌ 规划失败，错误码: {error_code}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'❌ 规划异常: {e}')
            return None

    def execute_plan(self, plan):
        """执行规划"""
        try:
            self.get_logger().info('🚀 执行规划中...')
            success = self.move_group.execute(plan, wait=True)
            
            if success:
                self.get_logger().info('✅ 执行完成')
                self.move_group.stop()  # 确保停止
                return True
            else:
                self.get_logger().error('❌ 执行失败')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ 执行异常: {e}')
            return False

    def plan_and_execute_joint_goal(self, joint_values):
        """规划并执行关节运动"""
        plan = self.plan_to_joint_goal(joint_values)
        if plan:
            return self.execute_plan(plan)
        return False

    def plan_and_execute_pose_goal(self, target_pose):
        """规划并执行位姿运动"""
        plan = self.plan_to_pose_goal(target_pose)
        if plan:
            return self.execute_plan(plan)
        return False

    def plan_cartesian_path(self, waypoints, eef_step=0.01, jump_threshold=0.0):
        """规划笛卡尔路径"""
        try:
            self.get_logger().info(f'📍 规划笛卡尔路径，路径点数: {len(waypoints)}')
            
            # 规划笛卡尔路径
            plan, fraction = self.move_group.compute_cartesian_path(
                waypoints, eef_step, jump_threshold
            )
            
            self.get_logger().info(f'📊 路径完成度: {fraction*100:.1f}%')
            
            if fraction > 0.8:  # 至少80%的路径规划成功
                self.get_logger().info('✅ 笛卡尔路径规划成功')
                return plan
            else:
                self.get_logger().error('❌ 笛卡尔路径规划失败')
                return None
                
        except Exception as e:
            self.get_logger().error(f'❌ 笛卡尔规划异常: {e}')
            return None

    def gripper_control(self, open_gripper=True):
        """控制夹爪"""
        client = self.gripper_open_client if open_gripper else self.gripper_close_client
        action = "打开" if open_gripper else "关闭"
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'❌ 夹爪{action}服务不可用')
            return False
        
        request = SetBool.Request()
        request.data = True
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.get_logger().info(f'✅ 夹爪已{action}')
            return True
        else:
            self.get_logger().error(f'❌ 夹爪{action}失败')
            return False

    def perform_pick_and_place_demo(self):
        """执行抓取放置演示"""
        self.get_logger().info('🎭 开始执行MoveIt抓取放置演示')
        
        # 步骤1: 移动到准备位置
        if not self.plan_and_execute_joint_goal(self.joint_positions['ready']):
            return False
        time.sleep(1.0)
        
        # 步骤2: 打开夹爪
        if not self.gripper_control(True):
            return False
        time.sleep(1.0)
        
        # 步骤3: 移动到前方低位置
        if not self.plan_and_execute_pose_goal(self.cartesian_poses['front_low']):
            return False
        time.sleep(1.0)
        
        # 步骤4: 关闭夹爪
        if not self.gripper_control(False):
            return False
        time.sleep(1.0)
        
        # 步骤5: 抬起到高位置
        if not self.plan_and_execute_pose_goal(self.cartesian_poses['front_high']):
            return False
        time.sleep(1.0)
        
        # 步骤6: 移动到侧面放置
        if not self.plan_and_execute_pose_goal(self.cartesian_poses['left_side']):
            return False
        time.sleep(1.0)
        
        # 步骤7: 打开夹爪放置
        if not self.gripper_control(True):
            return False
        time.sleep(1.0)
        
        # 步骤8: 返回home
        if not self.plan_and_execute_joint_goal(self.joint_positions['home']):
            return False
        
        self.get_logger().info('🎉 MoveIt抓取放置演示完成')
        return True

    def perform_cartesian_demo(self):
        """执行笛卡尔路径演示"""
        self.get_logger().info('📐 开始执行笛卡尔路径演示')
        
        # 获取当前位姿
        current_pose = self.move_group.get_current_pose().pose
        
        # 创建路径点
        waypoints = []
        waypoints.append(current_pose)
        
        # 向前移动10cm
        wpose = Pose()
        wpose.position.x = current_pose.position.x + 0.1
        wpose.position.y = current_pose.position.y
        wpose.position.z = current_pose.position.z
        wpose.orientation = current_pose.orientation
        waypoints.append(wpose)
        
        # 向上移动10cm
        wpose.position.z += 0.1
        waypoints.append(wpose)
        
        # 向右移动10cm
        wpose.position.y -= 0.1
        waypoints.append(wpose)
        
        # 向下移动10cm
        wpose.position.z -= 0.1
        waypoints.append(wpose)
        
        # 规划并执行笛卡尔路径
        plan = self.plan_cartesian_path(waypoints)
        if plan:
            success = self.execute_plan(plan)
            if success:
                self.get_logger().info('✅ 笛卡尔路径演示完成')
                return True
        
        self.get_logger().error('❌ 笛卡尔路径演示失败')
        return False

    def show_menu(self):
        """显示操作菜单"""
        print("\n" + "="*70)
        print("🤖 MoveIt路径规划器")
        print("="*70)
        print("关节空间规划:")
        print("  1. home      - 回到零位")
        print("  2. ready     - 准备位置") 
        print("  3. reset     - 重置位置")
        print("  4. pick      - 抓取位置")
        print("  5. place     - 放置位置")
        print()
        print("笛卡尔空间规划:")
        print("  6. front_high - 前方高位置")
        print("  7. front_low  - 前方低位置")
        print("  8. left_side  - 左侧位置")
        print("  9. right_side - 右侧位置")
        print()
        print("复合任务:")
        print("  10. pick_demo     - 抓取放置演示")
        print("  11. cartesian_demo - 笛卡尔路径演示")
        print()
        print("夹爪控制:")
        print("  12. open      - 打开夹爪")
        print("  13. close     - 关闭夹爪")
        print()
        print("信息查询:")
        print("  14. current   - 显示当前状态")
        print()
        print("  0. 退出")
        print("="*70)

    def show_current_state(self):
        """显示当前状态"""
        try:
            print("\n📊 当前机械臂状态:")
            print("-" * 50)
            
            # 关节角度
            joint_values = self.move_group.get_current_joint_values()
            print("关节角度 (弧度):")
            for i, value in enumerate(joint_values):
                print(f"  joint{i+1}: {value:8.3f} rad ({math.degrees(value):8.1f}°)")
            
            # 末端位姿
            current_pose = self.move_group.get_current_pose().pose
            print("\n末端执行器位姿:")
            print(f"  位置: ({current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f})")
            print(f"  姿态: ({current_pose.orientation.x:.3f}, {current_pose.orientation.y:.3f}, {current_pose.orientation.z:.3f}, {current_pose.orientation.w:.3f})")
            
            # 规划组信息
            print(f"\n规划组: {self.move_group.get_name()}")
            print(f"规划坐标系: {self.planning_frame}")
            print(f"末端执行器: {self.eef_link}")
            
        except Exception as e:
            print(f"❌ 获取状态失败: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        planner = MoveItPlanner()
        
        # 等待连接建立
        print("⏳ 等待MoveIt连接建立...")
        if not planner.wait_for_move_group():
            print("❌ MoveIt连接失败")
            return
        
        print("✅ MoveIt连接成功")
        
        # 启动ROS spin线程
        ros_thread = Thread(target=rclpy.spin, args=(planner,))
        ros_thread.daemon = True
        ros_thread.start()
        
        # 主菜单循环
        while True:
            planner.show_menu()
            user_input = input("\n请输入命令: ").strip()
            
            if user_input == '0' or user_input.lower() in ['exit', 'quit']:
                break
            elif user_input == '1':
                planner.plan_and_execute_joint_goal(planner.joint_positions['home'])
            elif user_input == '2':
                planner.plan_and_execute_joint_goal(planner.joint_positions['ready'])
            elif user_input == '3':
                planner.plan_and_execute_joint_goal(planner.joint_positions['reset'])
            elif user_input == '4':
                planner.plan_and_execute_joint_goal(planner.joint_positions['pick_pose'])
            elif user_input == '5':
                planner.plan_and_execute_joint_goal(planner.joint_positions['place_pose'])
            elif user_input == '6':
                planner.plan_and_execute_pose_goal(planner.cartesian_poses['front_high'])
            elif user_input == '7':
                planner.plan_and_execute_pose_goal(planner.cartesian_poses['front_low'])
            elif user_input == '8':
                planner.plan_and_execute_pose_goal(planner.cartesian_poses['left_side'])
            elif user_input == '9':
                planner.plan_and_execute_pose_goal(planner.cartesian_poses['right_side'])
            elif user_input == '10':
                planner.perform_pick_and_place_demo()
            elif user_input == '11':
                planner.perform_cartesian_demo()
            elif user_input == '12':
                planner.gripper_control(True)
            elif user_input == '13':
                planner.gripper_control(False)
            elif user_input == '14':
                planner.show_current_state()
            else:
                print("❌ 未知命令")
    
    except KeyboardInterrupt:
        print("\n🛑 用户中断")
    except Exception as e:
        print(f"❌ 程序异常: {e}")
    finally:
        try:
            planner.destroy_node()
        except:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 