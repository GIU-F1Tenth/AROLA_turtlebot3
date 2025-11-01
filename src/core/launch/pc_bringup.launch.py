#!/usr/bin/env python3
"""
PC Bringup Launch File for TurtleBot3 Distributed Architecture

This launch file starts all computationally intensive nodes on the PC:
- Pure Pursuit Controller
- AMCL Localization
- Simple Path Planner
- Ackermann to Twist Converter
- Watchdog
- TF transforms

The Raspberry Pi runs only the lightweight PiIO node for hardware interface.

Author: Adapted for TurtleBot3 Distributed Architecture
License: MIT
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate PC-side launch description for TurtleBot3 system."""

    # Get package directories
    core_share_dir = get_package_share_directory('core')
    config_file = os.path.join(core_share_dir, 'config', 'params_turtlebot3.yaml')

    # Declare launch arguments
    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the TurtleBot3 configuration file'
    )
    
    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for all nodes'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (set true for Gazebo)'
    )
    
    declare_map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file for AMCL'
    )
    
    declare_waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value='',
        description='Full path to CSV waypoint file'
    )

    config_file_path = LaunchConfiguration('config_file')
    log_level = LaunchConfiguration('log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    waypoint_file = LaunchConfiguration('waypoint_file')

    # ========== Core Transform Node ==========
    odom_to_base_link_node = Node(
        package='core',
        executable='odom_to_base_link_node',
        name='odom_to_base_link_node',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    # ========== Pure Pursuit Controller ==========
    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    # ========== Ackermann to Twist Converter ==========
    ackermann_to_twist_node = Node(
        package='ackermann_to_twist',
        executable='ackermann_to_twist_node',
        name='ackermann_to_twist_node',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    # ========== AMCL Localization ==========
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    # ========== Map Server (if map provided) ==========
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(
            PythonExpression(["'", map_file, "' != ''"])
        )
    )

    # ========== Lifecycle Manager for Map Server ==========
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    # ========== Simple Path Planner ==========
    simple_planner_node = Node(
        package='simple_planner',
        executable='simple_path_publisher',
        name='simple_planner',
        output='screen',
        parameters=[
            config_file_path,
            {'csv_path': waypoint_file},
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    # ========== Watchdog Safety Node ==========
    watchdog_node = Node(
        package='watchdog',
        executable='watchdog_node',
        name='watchdog_node',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    # Create and return launch description
    return LaunchDescription([
        declare_config_file_arg,
        declare_log_level_arg,
        declare_use_sim_time_arg,
        declare_map_file_arg,
        declare_waypoint_file_arg,
        
        LogInfo(msg='Starting TurtleBot3 PC-side compute nodes...'),
        
        odom_to_base_link_node,
        pure_pursuit_node,
        ackermann_to_twist_node,
        amcl_node,
        # map_server_node,  # Uncomment when you have launch.conditions imported
        # lifecycle_manager_node,
        simple_planner_node,
        watchdog_node,
        
        LogInfo(msg='TurtleBot3 PC-side nodes started. Make sure Pi-side nodes are running!'),
    ])
