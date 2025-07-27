#!/usr/bin/env python3
"""
Gazebo 和 RViz 同步演示启动文件
同时启动 Gazebo 仿真环境、RViz 可视化和同步节点
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成启动描述"""
    
    # 包路径
    pkg_share = get_package_share_directory('gazebo_rviz_sync_demo')
    
    # 世界文件路径
    world_file = os.path.join(pkg_share, 'worlds', 'simple_cube.world')
    
    # RViz 配置文件路径
    rviz_config = os.path.join(pkg_share, 'rviz', 'cube_sync.rviz')
    
    # 启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间'
    )
    
    declare_auto_move = DeclareLaunchArgument(
        'auto_move',
        default_value='true',
        description='是否自动移动方块'
    )
    
    declare_movement_mode = DeclareLaunchArgument(
        'movement_mode',
        default_value='circle',
        description='移动模式: circle, square, line, stop'
    )
    
    # 启动 Gazebo
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so',
            world_file
        ],
        output='screen'
    )
    
    # 启动 RViz
    rviz = TimerAction(
        period=3.0,  # 等待 Gazebo 启动
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                output='screen'
            )
        ]
    )
    
    # 方块同步节点
    cube_synchronizer = TimerAction(
        period=2.0,  # 等待 Gazebo 稍微启动
        actions=[
            Node(
                package='gazebo_rviz_sync_demo',
                executable='cube_synchronizer',
                name='cube_synchronizer',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                output='screen'
            )
        ]
    )
    
    # 方块控制节点 (可选)
    cube_controller = TimerAction(
        period=5.0,  # 等待所有其他节点启动
        actions=[
            Node(
                package='gazebo_rviz_sync_demo',
                executable='cube_controller',
                name='cube_controller',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'movement_mode': LaunchConfiguration('movement_mode')},
                    {'movement_speed': 1.0}
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('auto_move'))
            )
        ]
    )
    
    # 静态 TF 发布器 (world -> odom)
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_world_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_auto_move,
        declare_movement_mode,
        static_tf_world_odom,
        gazebo,
        cube_synchronizer,
        rviz,
        cube_controller,
    ]) 