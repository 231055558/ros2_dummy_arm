#!/usr/bin/env python3
"""
Gazebo Sync Node
Synchronizes Gazebo robot with real robot by subscribing to /joint_states
and publishing corresponding /gazebo_joint_states
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import threading


class GazeboSync(Node):
    def __init__(self):
        super().__init__('gazebo_sync')
        
        # Joint names for the robot
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Mutex for thread safety
        self.joint_state_mutex = threading.Lock()
        self.latest_joint_state = None
        self.sync_count = 0
        
        # Subscribe to real robot joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publish joint states for Gazebo robot state publisher
        self.gazebo_joint_state_publisher = self.create_publisher(
            JointState,
            '/gazebo_joint_states',
            10
        )
        
        # Publishers for individual joint position commands
        self.joint_position_publishers = {}
        for joint_name in self.joint_names:
            topic_name = f'/{joint_name}_position_controller/commands'
            self.joint_position_publishers[joint_name] = self.create_publisher(
                Float64,
                topic_name,
                10
            )
        
        # Timer to sync with Gazebo at regular intervals
        self.timer = self.create_timer(0.1, self.sync_with_gazebo)  # 10Hz
        
        self.get_logger().info('Gazebo Sync Node started - using Ignition position commands')
        self.get_logger().info(f'Publishing to topics: {list(self.joint_position_publishers.keys())}')

    def joint_state_callback(self, msg):
        """Callback for joint state messages from real robot"""
        with self.joint_state_mutex:
            self.latest_joint_state = msg

    def sync_with_gazebo(self):
        """Sync real robot joint states with Gazebo robot by publishing joint states"""
        
        with self.joint_state_mutex:
            if self.latest_joint_state is None:
                return
            
            try:
                # Get joint positions from the latest joint state
                if len(self.latest_joint_state.position) >= 6:
                    joint_positions = list(self.latest_joint_state.position[:6])
                    
                    # Send position commands to individual joints
                    for i, (joint_name, position) in enumerate(zip(self.joint_names, joint_positions)):
                        if joint_name in self.joint_position_publishers:
                            cmd_msg = Float64()
                            cmd_msg.data = float(position)
                            self.joint_position_publishers[joint_name].publish(cmd_msg)
                    
                    # Also publish joint states for visualization
                    gazebo_joint_state = JointState()
                    gazebo_joint_state.header.stamp = self.get_clock().now().to_msg()
                    gazebo_joint_state.header.frame_id = ""
                    gazebo_joint_state.name = self.joint_names
                    gazebo_joint_state.position = joint_positions
                    gazebo_joint_state.velocity = [0.0] * len(self.joint_names)
                    gazebo_joint_state.effort = [0.0] * len(self.joint_names)
                    
                    self.gazebo_joint_state_publisher.publish(gazebo_joint_state)
                    
                    # Log occasionally for debugging
                    self.sync_count += 1
                    if self.sync_count % 50 == 0:  # Log every 5 seconds at 10Hz
                        positions_str = [f"{pos:.3f}" for pos in joint_positions]
                        self.get_logger().info(f'Commanding Gazebo joints: {positions_str}')
                        
            except Exception as e:
                self.get_logger().error(f'Error syncing with Gazebo: {e}')


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        gazebo_sync = GazeboSync()
        rclpy.spin(gazebo_sync)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in Gazebo Sync: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 