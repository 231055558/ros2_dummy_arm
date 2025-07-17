#!/usr/bin/env python3
"""
Gazebo Only Launch File
This launch file only starts Gazebo for testing synchronization
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

    # Create robot description for Gazebo
    gazebo_urdf_file = os.path.join(
        get_package_share_directory("dummy_moveit_config"),
        "config",
        "dummy-ros2.gazebo.urdf.xacro"
    )
    
    gazebo_robot_description_content = xacro.process_file(gazebo_urdf_file).toxml()

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
            'use_sim_time': 'false'
        }.items()
    )

    # Robot description publisher for Gazebo
    gazebo_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="gazebo_robot_state_publisher",
        output="both",
        parameters=[{"robot_description": gazebo_robot_description_content}],
        remappings=[
            ("robot_description", "gazebo_robot_description"),
            ("joint_states", "gazebo_joint_states")
        ]
    )

    # Spawn robot in Gazebo
    spawn_entity = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'gazebo_robot_description',
                    '-entity', 'dummy_robot',
                    '-x', '0.0',
                    '-y', '0.0', 
                    '-z', '0.0'
                ],
                output='screen'
            )
        ]
    )

    # Gazebo Bridge Node
    gazebo_bridge_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="dummy_controller",
                executable="gazebo_bridge",
                name="gazebo_bridge",
                output="screen",
            )
        ]
    )

    return LaunchDescription(
        [
            gazebo_launch,
            gazebo_robot_state_publisher,
            spawn_entity,
            gazebo_bridge_node,
        ]
    )
