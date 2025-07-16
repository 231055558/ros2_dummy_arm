# 现在似乎 一切都正常了，但是还是存在一个问题，就是我的机械臂似乎会和rivz中总是存在初始的角度偏差，也就是我明明设置的初始位置其实就是0,0,0,0,0,0，但是rivz似乎会认为是0,73,-90,0,0,0，紧接着出现的问题就是，我在rivz中设置了一个新的位置后，机械臂也可以运动到这个位置了，但是同时rivz中的机械臂反而不是这个位置了，比如我在rivz中通过拖动达到了15,0,0,0,0,0，execute之后真实机械臂也会到这个位姿，但是rivz中反而到了15,73,-90,0,0,0
# 但是注意当前的运动一点问题都没有请不要随意修改，你需要作的就是，让rviz在保存当前机械臂真实角度的时候自动把j2-73把j3+90就可以了，这样可以保证他的运动规划算法没有问题
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
import time
import numpy as np
import dummy_controller.dummy_cli_tool.ref_tool

class JointTrajectoryActionServer(Node):
    
    my_driver = None
    rad_volumn_diff = np.array([0,0,1.57079,0,0,0])
    rad_direct_diff = np.array([1,1,1,1,-1,1])
    pai = 3.1415926
    ready_rad = np.array([0,0,0,0,0,0])
    home_rad = np.array([0,-1.3089,1.5707,0,0,0])
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    # Correction in degrees to align RVIZ with the real robot's reported state
    state_correction_deg = np.array([0, -73.0, 90.0, 0, 0, 0])

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
        self.move_rad(self.ready_rad)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'dummy_arm_controller/follow_joint_trajectory',
            self.execute_callback
        )
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
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
            # Invert J5 direction for RVIZ display
            corrected_angles_deg[4] = -corrected_angles_deg[4]
            current_angles_rad = self.degree2rad(corrected_angles_deg)
            joint_state_msg.position = current_angles_rad.tolist()
            self.joint_state_publisher.publish(joint_state_msg)
        except Exception as e:
            self.get_logger().error(f"Could not get joint states: {e}")

    def rad_fix(self,arr_rad):
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

