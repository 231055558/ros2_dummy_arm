#!/usr/bin/env python3
"""
Gazebo 到 RViz 方块同步节点
监听 Gazebo 中方块的位置信息，并在 RViz 中显示相应的标记
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class CubeSynchronizer(Node):
    """同步 Gazebo 方块到 RViz 的节点"""
    
    def __init__(self):
        super().__init__('cube_synchronizer')
        
        # 创建 TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 订阅 Gazebo 方块的里程计信息
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/demo_cube/odom',
            self.odom_callback,
            10
        )
        
        # 发布 RViz 标记
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/cube_markers',
            10
        )
        
        # 存储当前方块状态
        self.cube_pose = None
        
        # 创建定时器定期发布标记
        self.timer = self.create_timer(0.1, self.publish_markers)
        
        self.get_logger().info('方块同步节点已启动')
    
    def odom_callback(self, msg):
        """处理来自 Gazebo 的里程计消息"""
        self.cube_pose = msg.pose.pose
        self.get_logger().debug(f'收到方块位置: x={self.cube_pose.position.x:.2f}, '
                               f'y={self.cube_pose.position.y:.2f}, '
                               f'z={self.cube_pose.position.z:.2f}')
    
    def publish_markers(self):
        """发布 RViz 标记"""
        if self.cube_pose is None:
            return
        
        marker_array = MarkerArray()
        
        # 创建方块标记
        cube_marker = Marker()
        cube_marker.header.frame_id = "odom"
        cube_marker.header.stamp = self.get_clock().now().to_msg()
        cube_marker.ns = "gazebo_sync"
        cube_marker.id = 0
        cube_marker.type = Marker.CUBE
        cube_marker.action = Marker.ADD
        
        # 设置位置和方向
        cube_marker.pose = self.cube_pose
        
        # 设置尺寸 (与 Gazebo 中的方块一致)
        cube_marker.scale.x = 1.0
        cube_marker.scale.y = 1.0
        cube_marker.scale.z = 1.0
        
        # 设置颜色 (红色，与 Gazebo 一致)
        cube_marker.color.r = 1.0
        cube_marker.color.g = 0.0
        cube_marker.color.b = 0.0
        cube_marker.color.a = 0.8  # 半透明
        
        # 设置生命周期
        cube_marker.lifetime.sec = 1
        
        marker_array.markers.append(cube_marker)
        
        # 创建文本标记显示坐标
        text_marker = Marker()
        text_marker.header.frame_id = "odom"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "gazebo_sync"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        # 文本位置稍微偏移
        text_marker.pose.position.x = self.cube_pose.position.x
        text_marker.pose.position.y = self.cube_pose.position.y
        text_marker.pose.position.z = self.cube_pose.position.z + 0.8
        text_marker.pose.orientation.w = 1.0
        
        # 设置文本内容
        text_marker.text = f"Cube\\nX: {self.cube_pose.position.x:.2f}\\n" \
                          f"Y: {self.cube_pose.position.y:.2f}\\n" \
                          f"Z: {self.cube_pose.position.z:.2f}"
        
        # 设置文本样式
        text_marker.scale.z = 0.2  # 文字大小
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        text_marker.lifetime.sec = 1
        
        marker_array.markers.append(text_marker)
        
        # 发布标记数组
        self.marker_publisher.publish(marker_array)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = CubeSynchronizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 