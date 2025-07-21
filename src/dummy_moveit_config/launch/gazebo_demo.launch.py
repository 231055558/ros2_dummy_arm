#!/usr/bin/env python3
"""
简单的Gazebo机械臂演示启动文件
这个文件将在Gazebo中启动机械臂，并允许通过关节状态发布器控制
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    """
    生成Gazebo演示的启动描述
    """
    
    # 包路径
    pkg_name = 'dummy-ros2_description'
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    
    # URDF文件路径
    urdf_file = os.path.join(pkg_share, 'urdf', 'dummy-ros2-gazebo.xacro')
    
    # 处理URDF文件
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}
    
    # Gazebo控制器配置文件
    config_pkg_share = get_package_share_directory('dummy_moveit_config')
    controllers_file = os.path.join(config_pkg_share, 'config', 'gazebo_controllers.yaml')
    
    # 启动Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # 在Gazebo中生成机器人
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'dummy_robot',
            '-z', '0.1'  # 稍微抬高避免接触地面
        ],
        output='screen'
    )
    
    # 控制器管理器
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        output='screen'
    )
    
    # 关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # 位置控制器
    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['dummy_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        controller_manager,
        joint_state_broadcaster_spawner,
        position_controller_spawner,
    ]) 