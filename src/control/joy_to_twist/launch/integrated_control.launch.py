#!/usr/bin/env python3

"""
Integration launch file for TurtleBot3 with both manual and autonomous control.
This allows switching between manual joystick control and gap follower autonomous mode.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch both manual control and autonomous control systems.
    Use /joy_enable topic or buttons to switch between modes.
    """
    
    # Get package directories
    joy_to_twist_dir = get_package_share_directory('joy_to_twist')
    
    # Paths to config files
    joy_config = os.path.join(joy_to_twist_dir, 'config', 'joy_to_twist_params.yaml')
    
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
    
    # Joy node
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
    
    # Joy to Twist for manual control
    joy_to_twist_node = Node(
        package='joy_to_twist',
        executable='joy_to_twist_node',
        name='joy_to_twist_node',
        parameters=[
            joy_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cmd_vel_topic': '/manual/cmd_vel'
            }
        ],
        remappings=[
            ('/cmd_vel', '/manual/cmd_vel')
        ],
        output='screen'
    )
    
    # TODO: Add gap follower launch if desired
    # You can include the gap_follower launch file here
    # It should publish to /autonomous/cmd_vel
    
    # Velocity multiplexer node (simple version - switches based on /joy_enable)
    # You could create a more sophisticated mux node that handles priorities
    
    return LaunchDescription([
        use_sim_time_arg,
        joy_dev_arg,
        joy_node,
        joy_to_twist_node,
    ])
