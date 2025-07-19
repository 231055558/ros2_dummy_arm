#!/usr/bin/env python3
"""
Gazebo Sync Launch File
Launch Gazebo with the robot model and sync it with real robot joint states
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    """
    Generate launch description for Gazebo sync
    """
    
    # Process the xacro file to get proper URDF
    urdf_file = os.path.join(
        get_package_share_directory("dummy-ros2_description"),
        "urdf",
        "dummy-ros2.xacro"
    )
    
    # Process xacro file to convert to URDF
    import subprocess
    robot_description = subprocess.check_output([
        'xacro', urdf_file
    ]).decode('utf-8')

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': '',
            'pause': 'false',
            'use_sim_time': 'false'  # Use real time to sync with real robot
        }.items()
    )

    # Robot State Publisher for Gazebo (separate namespace to avoid conflicts)
    gazebo_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="gazebo_robot_state_publisher",
        namespace="gazebo",
        output="both",
        parameters=[{"robot_description": robot_description}],
        remappings=[
            ("joint_states", "/gazebo_joint_states")
        ]
    )

    # Spawn robot in Gazebo using robot_description topic
    spawn_entity = TimerAction(
        period=3.0,  # Wait for Gazebo to be ready
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'dummy_robot',
                    '-topic', '/gazebo/robot_description',
                    '-x', '0.0',
                    '-y', '0.0', 
                    '-z', '0.0'
                ],
                output='screen'
            )
        ]
    )

    # Gazebo sync node (will be created next)
    gazebo_sync_node = TimerAction(
        period=5.0,  # Wait for everything to be ready
        actions=[
            Node(
                package="dummy_controller",
                executable="gazebo_sync",
                name="gazebo_sync",
                output="screen",
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        gazebo_robot_state_publisher,
        spawn_entity,
        gazebo_sync_node,
    ]) 