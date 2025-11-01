#!/usr/bin/env python3
"""
TurtleBot3 Simulation Launch File with Distributed Architecture

This launch file simulates the complete distributed architecture in Gazebo:
- Launches TurtleBot3 in Gazebo (replaces PiIO node)
- Starts all PC-side compute nodes
- Configures topic remapping for simulation

Use this for testing before deploying to real hardware.

Prerequisites:
- Install turtlebot3_gazebo: sudo apt install ros-humble-turtlebot3-gazebo
- Set TURTLEBOT3_MODEL: export TURTLEBOT3_MODEL=burger

Author: Adapted for TurtleBot3 Distributed Architecture
License: MIT
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate simulation launch description."""

    # Get package directories
    core_share_dir = get_package_share_directory('core')
    config_file = os.path.join(core_share_dir, 'config', 'params_turtlebot3.yaml')

    # Try to find turtlebot3_gazebo
    try:
        tb3_gazebo_launch = PathJoinSubstitution([
            FindPackageShare('turtlebot3_gazebo'),
            'launch',
            'turtlebot3_world.launch.py'
        ])
        gazebo_available = True
    except:
        gazebo_available = False

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
    
    declare_waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value='',
        description='Full path to CSV waypoint file'
    )
    
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world',
        description='Gazebo world to load'
    )

    config_file_path = LaunchConfiguration('config_file')
    log_level = LaunchConfiguration('log_level')
    waypoint_file = LaunchConfiguration('waypoint_file')
    world = LaunchConfiguration('world')

    # ========== TurtleBot3 Gazebo Simulation ==========
    # This replaces the PiIO node in simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_gazebo_launch),
        launch_arguments={'world': world}.items()
    ) if gazebo_available else LogInfo(msg='TurtleBot3 Gazebo not found. Please install: sudo apt install ros-humble-turtlebot3-gazebo')

    # ========== Core Transform Node ==========
    odom_to_base_link_node = Node(
        package='core',
        executable='odom_to_base_link_node',
        name='odom_to_base_link_node',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': True}
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
            {'use_sim_time': True}
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
            {'use_sim_time': True}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    # ========== AMCL Localization ==========
    # Note: You'll need to load a map separately or use SLAM
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': True}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
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
            {'use_sim_time': True}
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
            {'use_sim_time': True}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    # Create and return launch description
    ld = LaunchDescription([
        declare_config_file_arg,
        declare_log_level_arg,
        declare_waypoint_file_arg,
        declare_world_arg,
        
        LogInfo(msg='Starting TurtleBot3 Simulation with Distributed Architecture...'),
    ])
    
    if gazebo_available:
        ld.add_action(gazebo_launch)
    else:
        ld.add_action(LogInfo(msg='ERROR: TurtleBot3 Gazebo not found!'))
    
    ld.add_action(odom_to_base_link_node)
    ld.add_action(pure_pursuit_node)
    ld.add_action(ackermann_to_twist_node)
    ld.add_action(amcl_node)
    ld.add_action(simple_planner_node)
    ld.add_action(watchdog_node)
    ld.add_action(LogInfo(msg='TurtleBot3 simulation nodes started.'))
    
    return ld
