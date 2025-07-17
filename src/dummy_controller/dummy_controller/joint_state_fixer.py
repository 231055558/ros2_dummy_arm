#!/usr/bin/env python3
"""
Joint State Fixer Node
This node ensures that joint states are correctly published and synchronized
It acts as a relay to ensure RViz gets the correct joint states
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading


class JointStateFixer(Node):
    def __init__(self):
        super().__init__('joint_state_fixer')
        
        # Mutex for thread safety
        self.joint_state_mutex = threading.Lock()
        self.latest_joint_state = None
        
        # Subscribe to the original joint states from dummy_arm_controller
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Republish joint states to ensure they reach all subscribers
        self.joint_state_republisher = self.create_publisher(
            JointState,
            '/joint_states_fixed',
            10
        )
        
        # Timer to republish joint states
        self.timer = self.create_timer(0.05, self.republish_joint_states)  # 20Hz
        
        self.get_logger().info('Joint State Fixer Node started - ensuring correct joint state propagation')

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        with self.joint_state_mutex:
            self.latest_joint_state = msg
            # Immediately republish to ensure propagation
            self.joint_state_republisher.publish(msg)

    def republish_joint_states(self):
        """Republish the latest joint states"""
        with self.joint_state_mutex:
            if self.latest_joint_state is not None:
                # Update timestamp and republish
                republished_msg = JointState()
                republished_msg.header.stamp = self.get_clock().now().to_msg()
                republished_msg.header.frame_id = self.latest_joint_state.header.frame_id
                republished_msg.name = self.latest_joint_state.name
                republished_msg.position = self.latest_joint_state.position
                republished_msg.velocity = self.latest_joint_state.velocity
                republished_msg.effort = self.latest_joint_state.effort
                
                self.joint_state_republisher.publish(republished_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        fixer_node = JointStateFixer()
        rclpy.spin(fixer_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in Joint State Fixer: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
