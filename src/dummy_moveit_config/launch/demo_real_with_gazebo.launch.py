#!/usr/bin/env python3
"""
Launch file for running RViz + Gazebo + Real Robot synchronization
This launch file starts:
1. Gazebo simulation
2. RViz with MoveIt planning
3. Bridge node to sync real robot with Gazebo
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def generate_launch_description():

    # Declare launch arguments
    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='Whether to start Gazebo simulation'
    )
    
    use_bridge_arg = DeclareLaunchArgument(
        'use_bridge',
        default_value='true',
        description='Whether to start the Gazebo bridge for real robot sync'
    )
    
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_bridge = LaunchConfiguration('use_bridge')

    # Use original URDF for RViz and MoveIt - this is CRITICAL for joint state sync
    urdf_file = "config/dummy-ros2.urdf.xacro"

    moveit_config = (
        MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config")
        .robot_description(file_path=urdf_file)
        .robot_description_semantic(file_path="config/dummy-ros2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Gazebo launch (conditional)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        condition=IfCondition(use_gazebo),
        launch_arguments={
            'world': '',
            'pause': 'false',
            'use_sim_time': 'false'  # Important: use real time for sync
        }.items()
    )

    # Create separate robot description for Gazebo
    gazebo_robot_description_content = xacro.process_file(
        os.path.join(
            get_package_share_directory("dummy_moveit_config"),
            "config",
            "dummy-ros2.gazebo.urdf.xacro"
        )
    ).toxml()

    # Gazebo robot description publisher (separate from main robot_description)
    gazebo_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="gazebo_robot_state_publisher",
        condition=IfCondition(use_gazebo),
        output="both",
        parameters=[{"robot_description": gazebo_robot_description_content}],
        remappings=[
            ("robot_description", "gazebo_robot_description"),
            ("joint_states", "gazebo_joint_states")
        ]
    )

    # Spawn robot in Gazebo (conditional)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        condition=IfCondition(use_gazebo),
        arguments=[
            '-topic', 'gazebo_robot_description',
            '-entity', 'dummy_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("dummy_moveit_config"),
        "config",
        "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # MoveGroup node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Controller spawner for Gazebo (conditional, with delay)
    controller_spawner = TimerAction(
        period=3.0,  # Wait 3 seconds for Gazebo to be ready
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                condition=IfCondition(use_gazebo),
                arguments=["gazebo_dummy_arm_controller", "joint_state_broadcaster"],
                output="screen",
            )
        ]
    )

    # Gazebo Bridge Node (conditional, with delay)
    gazebo_bridge_node = TimerAction(
        period=5.0,  # Wait 5 seconds for everything to be ready
        actions=[
            Node(
                package="dummy_controller",
                executable="gazebo_bridge",
                name="gazebo_bridge",
                condition=IfCondition(use_bridge),
                output="screen",
            )
        ]
    )

    return LaunchDescription(
        [
            use_gazebo_arg,
            use_bridge_arg,
            gazebo_launch,
            gazebo_robot_state_publisher,
            spawn_entity,
            rviz_node,
            static_tf,
            robot_state_publisher,
            move_group_node,
            controller_spawner,
            gazebo_bridge_node,
        ]
    )
