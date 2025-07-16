#!/usr/bin/env python3
"""
Gazebo到MoveIt规划场景的桥接节点
自动从Gazebo世界同步碰撞对象到MoveIt规划场景
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetModelList, GetModelState
from moveit_msgs.msg import CollisionObject, PlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import time


class GazeboMoveItBridge(Node):
    def __init__(self):
        super().__init__('gazebo_moveit_bridge')
        
        # 创建服务客户端
        self.get_model_list_client = self.create_client(GetModelList, '/get_model_list')
        self.get_model_state_client = self.create_client(GetModelState, '/get_model_state')
        
        # 创建发布器
        self.collision_object_pub = self.create_publisher(
            CollisionObject, 
            '/collision_object', 
            10
        )
        
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
        # 定时器，定期同步场景
        self.sync_timer = self.create_timer(2.0, self.sync_scene)
        
        # 已同步的模型列表
        self.synced_models = set()
        
        # 需要忽略的模型（机器人本身和地面等）
        self.ignore_models = {
            'ground_plane', 'dummy_robot', 'dummy-ros2', 'sun'
        }
        
        self.get_logger().info("Gazebo-MoveIt桥接器已启动")
        
    def wait_for_services(self):
        """等待Gazebo服务可用"""
        self.get_logger().info("等待Gazebo服务...")
        
        while not self.get_model_list_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("等待/get_model_list服务...")
            
        while not self.get_model_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("等待/get_model_state服务...")
            
        self.get_logger().info("Gazebo服务已连接")
        
    def sync_scene(self):
        """同步Gazebo场景到MoveIt"""
        try:
            # 获取模型列表
            request = GetModelList.Request()
            future = self.get_model_list_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if not future.done():
                return
                
            response = future.result()
            if not response:
                return
                
            current_models = set(response.model_names)
            
            # 移除不需要的模型
            current_models -= self.ignore_models
            
            # 添加新模型
            new_models = current_models - self.synced_models
            for model_name in new_models:
                self.add_model_to_planning_scene(model_name)
                
            # 移除已删除的模型
            removed_models = self.synced_models - current_models
            for model_name in removed_models:
                self.remove_model_from_planning_scene(model_name)
                
            self.synced_models = current_models
            
        except Exception as e:
            self.get_logger().warn(f"同步场景时发生错误: {e}")
            
    def add_model_to_planning_scene(self, model_name):
        """将Gazebo模型添加到MoveIt规划场景"""
        try:
            # 获取模型状态
            request = GetModelState.Request()
            request.model_name = model_name
            request.relative_entity_name = ""
            
            future = self.get_model_state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if not future.done():
                return
                
            response = future.result()
            if not response or not response.success:
                return
                
            # 创建碰撞对象
            collision_object = CollisionObject()
            collision_object.header = Header()
            collision_object.header.frame_id = "base_link"
            collision_object.header.stamp = self.get_clock().now().to_msg()
            collision_object.id = model_name
            collision_object.operation = CollisionObject.ADD
            
            # 根据模型名称创建不同的形状
            if 'table' in model_name:
                self.create_box_primitive(collision_object, [1.2, 0.8, 0.05])
            elif 'box' in model_name:
                self.create_box_primitive(collision_object, [0.15, 0.15, 0.15])
            elif 'cylinder' in model_name:
                self.create_cylinder_primitive(collision_object, 0.08, 0.3)
            elif 'sphere' in model_name:
                self.create_sphere_primitive(collision_object, 0.1)
            elif 'wall' in model_name:
                self.create_box_primitive(collision_object, [0.05, 1.5, 1.0])
            else:
                # 默认创建小盒子
                self.create_box_primitive(collision_object, [0.1, 0.1, 0.1])
                
            # 设置位置
            collision_object.primitive_poses = [response.pose]
            
            # 发布碰撞对象
            self.collision_object_pub.publish(collision_object)
            self.get_logger().info(f"已添加模型到规划场景: {model_name}")
            
        except Exception as e:
            self.get_logger().warn(f"添加模型 {model_name} 时发生错误: {e}")
            
    def remove_model_from_planning_scene(self, model_name):
        """从MoveIt规划场景移除模型"""
        try:
            collision_object = CollisionObject()
            collision_object.header = Header()
            collision_object.header.frame_id = "base_link"
            collision_object.header.stamp = self.get_clock().now().to_msg()
            collision_object.id = model_name
            collision_object.operation = CollisionObject.REMOVE
            
            self.collision_object_pub.publish(collision_object)
            self.get_logger().info(f"已从规划场景移除模型: {model_name}")
            
        except Exception as e:
            self.get_logger().warn(f"移除模型 {model_name} 时发生错误: {e}")
            
    def create_box_primitive(self, collision_object, dimensions):
        """创建盒子形状"""
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions
        collision_object.primitives = [box]
        
    def create_cylinder_primitive(self, collision_object, radius, length):
        """创建圆柱体形状"""
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [length, radius]  # 注意：长度在前，半径在后
        collision_object.primitives = [cylinder]
        
    def create_sphere_primitive(self, collision_object, radius):
        """创建球体形状"""
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [radius]
        collision_object.primitives = [sphere]


def main(args=None):
    rclpy.init(args=args)
    
    bridge = GazeboMoveItBridge()
    
    try:
        # 等待服务可用
        bridge.wait_for_services()
        
        # 初始同步
        time.sleep(2.0)
        bridge.sync_scene()
        
        # 持续运行
        rclpy.spin(bridge)
        
    except KeyboardInterrupt:
        bridge.get_logger().info("桥接器被用户中断")
    except Exception as e:
        bridge.get_logger().error(f"桥接器发生错误: {e}")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 