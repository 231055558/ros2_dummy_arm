# 现在似乎 一切都正常了，但是还是存在一个问题，就是我的机械臂似乎会和rivz中总是存在初始的角度偏差，也就是我明明设置的初始位置其实就是0,0,0,0,0,0，但是rivz似乎会认为是0,73,-90,0,0,0，紧接着出现的问题就是，我在rivz中设置了一个新的位置后，机械臂也可以运动到这个位置了，但是同时rivz中的机械臂反而不是这个位置了，比如我在rivz中通过拖动达到了15,0,0,0,0,0，execute之后真实机械臂也会到这个位姿，但是rivz中反而到了15,73,-90,0,0,0
# 但是注意当前的运动一点问题都没有请不要随意修改，你需要作的就是，让rviz在保存当前机械臂真实角度的时候自动把j2-73把j3+90就可以了，这样可以保证他的运动规划算法没有问题
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from std_srvs.srv import SetBool
from std_msgs.msg import Float64, Bool
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import numpy as np
import dummy_controller.dummy_cli_tool.ref_tool

class JointTrajectoryActionServer(Node):
    
    my_driver = None
    rad_volumn_diff = np.array([-0.05236,-0.05,1.57079,0,0,0])
    
    # ✅ 分离指令和状态的方向控制
    # 指令方向：RViz目标角度 → 机械臂执行 (用于move_rad)
    rad_direct_diff = np.array([1,1,1,1,-1,-1])  # J6改为-1，使指令方向正确
    
    pai = 3.1415926
    ready_rad = np.array([0,0,0,0,0,0])
    home_rad = np.array([0,-1.3089,1.5707,0,0,0])
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # 保持原有6个关节
    # Correction in degrees to align RVIZ with the real robot's reported state
    state_correction_deg = np.array([3, -72.0, 90.0, 0, 0, 0])
    
    # ✅ 电流限制配置 (单位: A)
    current_limits = {
        'joint_1': 1.5,  # Joint1电流限制
        'joint_2': 3.0,  # Joint2电流限制 (负载最大)
        'joint_3': 3.0,  # Joint3电流限制
        'joint_4': 1.2,  # Joint4电流限制
        'joint_5': 1.5,  # Joint5电流限制
        'joint_6': 1.2,  # Joint6电流限制
        'hand': 1.2      # 夹爪电流限制
    }

    def __init__(self):
        super().__init__('dummy_arm_controller_real')
        self.get_logger().info('Ready to setup dummy arm')
        self.my_driver = dummy_controller.dummy_cli_tool.ref_tool.find_any()

        # We can now remove the debug dump
        # self.get_logger().info("Dumping robot attributes...")
        # self.get_logger().info(self.my_driver.robot._dump(indent="  ", depth=3))

        self.my_driver.robot.set_enable(1)
        self.my_driver.robot.set_rgb_mode(4)  #green light is ready
        self.my_driver.robot.set_command_mode(0) # Set to position control mode
        
        # ✅ 设置电流限制
        self.setup_current_limits()
        
        # ✅ 初始化夹爪
        self.setup_gripper()
        
        self.move_rad(self.ready_rad)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'dummy_arm_controller/follow_joint_trajectory',
            self.execute_callback
        )
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)

        # ✅ 创建电流限制调整服务 (可选功能)
        self.current_limit_service = self.create_service(
            SetBool,
            'dummy_arm/reset_current_limits',
            self.reset_current_limits_callback
        )
        self.get_logger().info('电流限制调整服务已启动: /dummy_arm/reset_current_limits')

        # ✅ 夹爪控制接口
        # 夹爪开合控制服务
        self.gripper_open_service = self.create_service(
            SetBool,
            'dummy_arm/gripper_open',
            self.gripper_open_callback
        )
        
        self.gripper_close_service = self.create_service(
            SetBool,
            'dummy_arm/gripper_close', 
            self.gripper_close_callback
        )
        
        # 夹爪角度控制订阅者 (0-100，0=完全打开，100=完全闭合)
        self.gripper_angle_subscriber = self.create_subscription(
            Float64,
            'dummy_arm/gripper_angle',
            self.gripper_angle_callback,
            10
        )
        
        self.get_logger().info('夹爪控制接口已启动:')
        self.get_logger().info('  服务: /dummy_arm/gripper_open, /dummy_arm/gripper_close')  
        self.get_logger().info('  话题: /dummy_arm/gripper_angle (Float64, 0-100)')

    def setup_current_limits(self):
        """
        设置机械臂各关节和夹爪的电流限制
        """
        try:
            self.get_logger().info('设置电流限制...')
            
            # 设置各关节电流限制
            for joint_name, current_limit in self.current_limits.items():
                if joint_name == 'hand':
                    # 设置夹爪电流限制
                    if hasattr(self.my_driver.robot, 'hand'):
                        if hasattr(self.my_driver.robot.hand, 'set_current'):
                            self.my_driver.robot.hand.set_current(current_limit)
                            self.get_logger().info(f'夹爪电流设置为: {current_limit}A')
                        elif hasattr(self.my_driver.robot.hand, 'set_current_limit'):
                            self.my_driver.robot.hand.set_current_limit(current_limit)
                            self.get_logger().info(f'夹爪电流限制设置为: {current_limit}A')
                        else:
                            self.get_logger().warn('夹爪不支持电流设置')
                    else:
                        self.get_logger().warn('未检测到夹爪')
                else:
                    # 设置关节电流限制
                    joint_obj = getattr(self.my_driver.robot, joint_name, None)
                    if joint_obj and hasattr(joint_obj, 'set_current_limit'):
                        joint_obj.set_current_limit(current_limit)
                        self.get_logger().info(f'{joint_name} 电流限制设置为: {current_limit}A')
                    else:
                        self.get_logger().warn(f'{joint_name} 不支持电流限制设置')
            
            self.get_logger().info('电流限制设置完成!')
            
        except Exception as e:
            self.get_logger().error(f'设置电流限制时出错: {e}')

    def setup_gripper(self):
        """
        初始化夹爪设置
        """
        try:
            if hasattr(self.my_driver.robot, 'hand'):
                # 使能夹爪
                self.my_driver.robot.hand.set_enable(True)
                self.get_logger().info('夹爪已使能')
                
                # 设置夹爪到半开状态
                self.my_driver.robot.hand.set_angle_with_speed_limit(100.0)
                self.get_logger().info('夹爪初始化到半开状态(50%)')
            else:
                self.get_logger().warn('未检测到夹爪，跳过夹爪初始化')
        except Exception as e:
            self.get_logger().error(f'夹爪初始化失败: {e}')

    def reset_current_limits_callback(self, request, response):
        """
        ROS服务回调：重新设置电流限制
        """
        try:
            self.setup_current_limits()
            response.success = True
            response.message = "电流限制重新设置成功"
        except Exception as e:
            response.success = False
            response.message = f"电流限制设置失败: {str(e)}"
        
        return response

    def publish_joint_states(self):
        """
        Publish joint states with accurate timestamps and complete information
        """
        joint_state_msg = JointState()
        # Use the node's clock for consistent timestamps
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = ""
        joint_state_msg.name = self.joint_names
        
        try:
            # Get joint angles from the correct attributes
            current_angles_deg = np.array([
                self.my_driver.robot.joint_1.angle,
                self.my_driver.robot.joint_2.angle,
                self.my_driver.robot.joint_3.angle,
                self.my_driver.robot.joint_4.angle,
                self.my_driver.robot.joint_5.angle,
                self.my_driver.robot.joint_6.angle
            ])
            
            # Apply the correction to align RVIZ with the physical robot's zero position
            corrected_angles_deg = current_angles_deg + self.state_correction_deg
            
            # ✅ 只对J6进行方向调整，保持其他关节不变
            # J6方向翻转（因为指令时已经翻转了，状态反馈也要翻转回来）
            corrected_angles_deg[5] = -corrected_angles_deg[5]  # J6方向调整
            
            # Invert J5 direction for RVIZ display (保持原有逻辑)
            corrected_angles_deg[4] = -corrected_angles_deg[4]
            
            current_angles_rad = self.degree2rad(corrected_angles_deg)
            
            # Set position data
            joint_state_msg.position = current_angles_rad.tolist()
            
            # Initialize velocity and effort arrays (set to zero for now)
            # In the future, these could be populated with actual sensor data
            joint_state_msg.velocity = [0.0] * len(self.joint_names)
            joint_state_msg.effort = [0.0] * len(self.joint_names)
            
            # Publish the joint state
            self.joint_state_publisher.publish(joint_state_msg)
            
            # Log occasionally for debugging (every 50 publishes = 5 seconds at 10Hz)
            if not hasattr(self, '_publish_count'):
                self._publish_count = 0
            self._publish_count += 1
            
            if self._publish_count % 50 == 0:
                pos_str = [f"{pos:.3f}" for pos in joint_state_msg.position]
                self.get_logger().debug(f'Published joint states: {pos_str}')
                
        except Exception as e:
            self.get_logger().error(f"Could not get joint states: {e}")
            # Publish a valid message with zero positions as fallback
            joint_state_msg.position = [0.0] * len(self.joint_names)
            joint_state_msg.velocity = [0.0] * len(self.joint_names)
            joint_state_msg.effort = [0.0] * len(self.joint_names)
            self.joint_state_publisher.publish(joint_state_msg)

    def gripper_open_callback(self, request, response):
        """
        夹爪打开服务回调 (注意：CLI中close是打开)
        """
        try:
            if hasattr(self.my_driver.robot, 'hand'):
                self.my_driver.robot.hand.close()  # CLI中close是打开！
                response.success = True
                response.message = "夹爪已打开"
                self.get_logger().info("夹爪打开命令执行成功")
            else:
                response.success = False
                response.message = "未检测到夹爪"
        except Exception as e:
            response.success = False
            response.message = f"夹爪打开失败: {str(e)}"
            self.get_logger().error(f"夹爪打开失败: {e}")
        
        return response

    def gripper_close_callback(self, request, response):
        """
        夹爪闭合服务回调 (注意：CLI中open是闭合)
        """
        try:
            if hasattr(self.my_driver.robot, 'hand'):
                self.my_driver.robot.hand.open()  # CLI中open是闭合！
                response.success = True
                response.message = "夹爪已闭合"
                self.get_logger().info("夹爪闭合命令执行成功")
            else:
                response.success = False
                response.message = "未检测到夹爪"
        except Exception as e:
            response.success = False
            response.message = f"夹爪闭合失败: {str(e)}"
            self.get_logger().error(f"夹爪闭合失败: {e}")
        
        return response

    def gripper_angle_callback(self, msg):
        """
        夹爪角度控制回调 (-100-100: -100=完全打开, 100=完全闭合)
        """
        try:
            angle = max(-100, min(100, msg.data))  # 限制在-100-100范围内
            if hasattr(self.my_driver.robot, 'hand'):
                self.my_driver.robot.hand.set_angle_with_speed_limit(angle)
                self.get_logger().info(f"夹爪角度设置为: {angle}")
            else:
                self.get_logger().error("未检测到夹爪")
        except Exception as e:
            self.get_logger().error(f"夹爪角度控制失败: {e}")

    def rad_fix(self,arr_rad):
        # ✅ 指令变换：RViz目标角度 → 机械臂执行
        return (arr_rad+self.rad_volumn_diff)*self.rad_direct_diff

    def rad2degree(self,arr_rad):
        arr_degree = arr_rad/self.pai*180
        return arr_degree

    def degree2rad(self,arr_degree):
        arr_rad = arr_degree/180*self.pai
        return arr_rad

    def move_rad(self,arr_rad):
        arr_rad = self.rad_fix(arr_rad)
        arr_degree = self.rad2degree(arr_rad)
        self.my_driver.robot.move_j(arr_degree[0],arr_degree[1],arr_degree[2],arr_degree[3],arr_degree[4],arr_degree[5])
        return True

    async def execute_callback(self, goal_handle):
        self.get_logger().info('okok,Received trajectory goal.')
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        points = trajectory.points
        start_time = time.time()
        
        for idx, point in enumerate(points):
            target_positions = point.positions
            time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            
            # wait for time
            now = time.time()
            wait_time = start_time + time_from_start - now
            if wait_time > 0:
                time.sleep(wait_time)
            
            # sent to hardware (joint_names, target_positions)
            self.get_logger().info(f'[{idx}] Sending positions: {target_positions}')
            np_target_positions = np.array(target_positions)
            self.move_rad(np_target_positions)
                
        self.get_logger().info('Trajectory execution complete.')
        # execute succeed
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result

    def cleanup(self):
        self.move_rad(self.home_rad)
        #self.my_driver.robot.set_enable(0)
        self.my_driver.robot.set_rgb_mode(0)
        pass

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

