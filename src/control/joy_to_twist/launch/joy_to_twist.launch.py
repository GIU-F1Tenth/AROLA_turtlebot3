#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch file for joy_to_twist node.
    This will:
    1. Start the joy node to read from joystick
    2. Start the joy_to_twist_node to convert Joy to Twist
    """
    
    # Get package directory
    pkg_dir = get_package_share_directory('joy_to_twist')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'joy_to_twist_params.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    # Joy node (reads from joystick hardware)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    # Joy to Twist converter node
    joy_to_twist_node = Node(
        package='joy_to_twist',
        executable='joy_to_twist_node',
        name='joy_to_twist_node',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        joy_dev_arg,
        joy_node,
        joy_to_twist_node,
    ])
