#!/usr/bin/env python3
"""
Raspberry Pi Bringup Launch File for TurtleBot3 Distributed Architecture

This launch file starts only the lightweight PiIO node on the Raspberry Pi.
The PiIO node handles:
- Publishing odometry from encoders
- Publishing sensor data (LIDAR, IMU)
- Subscribing to /cmd_vel and controlling motors
- Watchdog safety monitoring

All heavy computation (planning, control, localization) runs on the PC.

Network Requirements:
- Both PC and Pi must be on the same network
- Set ROS_DOMAIN_ID to same value on both machines (e.g., export ROS_DOMAIN_ID=30)
- Ensure firewall allows ROS2 DDS communication (ports 7400-7500 UDP)

Author: Adapted for TurtleBot3 Distributed Architecture
License: MIT
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate Pi-side launch description for TurtleBot3 system."""

    # Get package directory
    try:
        pi_io_share_dir = get_package_share_directory('pi_io')
        config_file = os.path.join(pi_io_share_dir, 'config', 'pi_io_params.yaml')
    except:
        # Fallback to core package config
        core_share_dir = get_package_share_directory('core')
        config_file = os.path.join(core_share_dir, 'config', 'params_turtlebot3.yaml')

    # Declare launch arguments
    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the Pi IO configuration file'
    )
    
    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for Pi IO node'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (should be false on real hardware)'
    )

    config_file_path = LaunchConfiguration('config_file')
    log_level = LaunchConfiguration('log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ========== Raspberry Pi IO Node ==========
    pi_io_node = Node(
        package='pi_io',
        executable='pi_io_node',
        name='pi_io_node',
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
        
        LogInfo(msg='Starting TurtleBot3 Raspberry Pi IO node...'),
        LogInfo(msg='Ensure ROS_DOMAIN_ID matches PC side!'),
        
        pi_io_node,
        
        LogInfo(msg='TurtleBot3 Pi-side node started.'),
    ])
