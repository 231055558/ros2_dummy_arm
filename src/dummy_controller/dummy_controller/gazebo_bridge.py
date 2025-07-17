#!/usr/bin/env python3
"""
Gazebo Bridge Node - Synchronizes real robot with Gazebo simulation
This node subscribes to joint states from the real robot and publishes them to Gazebo
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import threading


class GazeboBridge(Node):
    def __init__(self):
        super().__init__('gazebo_bridge')
        
        # Joint names for the robot
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Correction factors (same as in dummy_arm_controller.py)
        self.state_correction_deg = np.array([3, -73.0, 90.0, 0, 0, 0])
        self.pai = 3.1415926
        
        # Mutex for thread safety
        self.joint_state_mutex = threading.Lock()
        self.latest_joint_state = None
        
        # Subscribe to real robot joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for Gazebo joint states (not commands!)
        self.gazebo_joint_state_publisher = self.create_publisher(
            JointState,
            '/gazebo_joint_states',
            10
        )
        
        # Timer to publish to Gazebo at regular intervals
        self.timer = self.create_timer(0.05, self.publish_to_gazebo)  # 20Hz
        
        self.get_logger().info('Gazebo Bridge Node started - synchronizing real robot with Gazebo')

    def degree2rad(self, arr_degree):
        """Convert degrees to radians"""
        return arr_degree / 180.0 * self.pai

    def joint_state_callback(self, msg):
        """Callback for joint state messages from real robot"""
        with self.joint_state_mutex:
            self.latest_joint_state = msg

    def publish_to_gazebo(self):
        """Publish joint states to Gazebo"""
        with self.joint_state_mutex:
            if self.latest_joint_state is None:
                return

            try:
                # Create a copy of the joint state for Gazebo
                gazebo_joint_state = JointState()
                gazebo_joint_state.header.stamp = self.get_clock().now().to_msg()
                gazebo_joint_state.header.frame_id = ""

                # Use the same joint names and positions from real robot
                gazebo_joint_state.name = self.latest_joint_state.name
                gazebo_joint_state.position = self.latest_joint_state.position
                gazebo_joint_state.velocity = self.latest_joint_state.velocity if self.latest_joint_state.velocity else [0.0] * len(self.latest_joint_state.position)
                gazebo_joint_state.effort = self.latest_joint_state.effort if self.latest_joint_state.effort else [0.0] * len(self.latest_joint_state.position)

                # Publish to Gazebo joint states topic
                self.gazebo_joint_state_publisher.publish(gazebo_joint_state)

                # Log occasionally for debugging
                if hasattr(self, '_log_counter'):
                    self._log_counter += 1
                else:
                    self._log_counter = 0

                if self._log_counter % 100 == 0:  # Log every 5 seconds at 20Hz
                    positions_str = [f"{pos:.3f}" for pos in gazebo_joint_state.position[:6]]
                    self.get_logger().info(f'Syncing to Gazebo: {positions_str}')

            except Exception as e:
                self.get_logger().error(f'Error publishing to Gazebo: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge_node = GazeboBridge()
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in Gazebo Bridge: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
