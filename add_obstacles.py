#!/usr/bin/env python3
"""
直接运行的障碍物添加脚本
添加三个长方体障碍物到MoveIt planning scene
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
import time
import sys


def create_box_obstacle(name, x, y, z, size_x, size_y, size_z, frame_id="world"):
    """创建长方体障碍物"""
    collision_object = CollisionObject()
    collision_object.header.frame_id = frame_id
    collision_object.id = name

    # 创建长方体
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [size_x, size_y, size_z]  # 长方体，三个维度可以不同

    # 设置位置
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0

    collision_object.primitives.append(box)
    collision_object.primitive_poses.append(pose)
    collision_object.operation = CollisionObject.ADD

    return collision_object


def main():
    rclpy.init()
    
    # 创建临时节点
    node = Node('obstacle_adder')
    
    # 创建发布器
    scene_publisher = node.create_publisher(PlanningScene, 'planning_scene', 10)
    
    # 等待连接建立
    print("等待连接建立...")
    time.sleep(2.0)
    
    # 创建planning scene消息
    planning_scene = PlanningScene()
    planning_scene.is_diff = True
    
    # 定义三个长方体障碍物
    obstacles = []
    
    # 长方体1
    box1 = create_box_obstacle(
        name="box1",
        x=0.15,     # 机械臂右侧20cm
        y=0.2,     # 前方20cm
        z=0.5,    # 高度5cm
        size_x=0.01,  # 长度10cm
        size_y=1.0, # 宽度15cm
        size_z=1.0   # 高度10cm
    )
    obstacles.append(box1)
    
    # 长方体2
    box2 = create_box_obstacle(
        name="box2",
        x=-0.15,     # 机械臂左侧20cm
        y=0.2,     # 前方20cm
        z=0.5,    # 高度5cm
        size_x=0.01,  # 长度10cm
        size_y=1.0, # 宽度15cm
        size_z=1.0   # 高度10cm
    )
    obstacles.append(box2)
    
    # 长方体3
    box3 = create_box_obstacle(
        name="box3",
        x=0.0,     # 机械臂左侧20cm
        y=0.0,     # 前方20cm
        z=0.5,    # 高度5cm
        size_x=1.0,  # 长度10cm
        size_y=1.0, # 宽度15cm
        size_z=0.1   # 高度10cm
    )
    obstacles.append(box3)

    # 长方体4
    box4 = create_box_obstacle(
        name="box4",
        x=0.0,     # 机械臂左侧20cm
        y=0.4,     # 前方20cm
        z=0.07,    # 高度5cm
        size_x=0.5,  # 长度10cm
        size_y=0.3, # 宽度15cm
        size_z=0.1   # 高度10cm
    )
    obstacles.append(box4)
    
    # 添加到planning scene
    planning_scene.world.collision_objects = obstacles
    
    # 发布障碍物
    scene_publisher.publish(planning_scene)
    
    print(f"✅ 已添加 {len(obstacles)} 个长方体障碍物:")
    for i, obs in enumerate(obstacles, 1):
        box = obs.primitives[0]
        pos = obs.primitive_poses[0].position
        print(f"   长方体{i}: 位置({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), 尺寸({box.dimensions[0]:.2f}×{box.dimensions[1]:.2f}×{box.dimensions[2]:.2f})m")
    
    print("\n💡 障碍物已添加到planning scene，在RViz中应该可以看到")
    print("💡 脚本将保持运行5秒以确保消息发送成功...")
    
    # 保持运行一段时间确保消息发送
    time.sleep(5.0)
    
    # 清理
    node.destroy_node()
    rclpy.shutdown()
    
    print("🎯 完成!")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 用户中断")
    except Exception as e:
        print(f"❌ 错误: {e}")
        sys.exit(1) 