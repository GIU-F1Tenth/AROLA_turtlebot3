#!/usr/bin/env python3

"""
Combined launch file for TurtleBot3 manual control.
Launches joy_to_twist node along with TurtleBot3 bringup (if needed).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch file for complete TurtleBot3 manual control setup.
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
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic to publish velocity commands'
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
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic')
            }
        ],
        remappings=[
            ('/cmd_vel', LaunchConfiguration('cmd_vel_topic'))
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        joy_dev_arg,
        cmd_vel_topic_arg,
        joy_node,
        joy_to_twist_node,
    ])
