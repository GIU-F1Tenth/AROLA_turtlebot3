#!/usr/bin/env python3
"""
Ackermann to Twist Converter Node

Converts AckermannDriveStamped messages (used by F1TENTH controllers)
to Twist messages (used by TurtleBot3 and other differential drive robots).

This enables using F1TENTH control algorithms with differential drive platforms.

Author: Adapted for TurtleBot3 Distributed Architecture
License: MIT
"""

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
import math


class AckermannToTwistNode(Node):
    """
    Converts Ackermann steering commands to differential drive Twist commands.
    """

    def __init__(self):
        super().__init__('ackermann_to_twist_node')
        
        # Declare parameters
        self.declare_parameter('ackermann_topic', '/drive')
        self.declare_parameter('twist_topic', '/cmd_vel')
        self.declare_parameter('wheelbase', 0.160)  # TurtleBot3 Burger wheelbase in meters
        self.declare_parameter('max_steering_angle', 0.5)  # Maximum steering angle in radians
        
        # Get parameters
        self.ackermann_topic = self.get_parameter('ackermann_topic').value
        self.twist_topic = self.get_parameter('twist_topic').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        
        # Create subscriber and publisher
        self.ackermann_sub = self.create_subscription(
            AckermannDriveStamped,
            self.ackermann_topic,
            self.ackermann_callback,
            10
        )
        
        self.twist_pub = self.create_publisher(Twist, self.twist_topic, 10)
        
        self.get_logger().info(f'Ackermann to Twist Converter initialized')
        self.get_logger().info(f'Subscribing to: {self.ackermann_topic}')
        self.get_logger().info(f'Publishing to: {self.twist_topic}')
        self.get_logger().info(f'Wheelbase: {self.wheelbase}m')

    def ackermann_callback(self, msg: AckermannDriveStamped):
        """
        Convert Ackermann message to Twist message.
        
        For Ackermann steering:
        - linear velocity = msg.drive.speed
        - angular velocity = v * tan(steering_angle) / wheelbase
        
        For small angles: tan(angle) ≈ angle
        Angular velocity = v * steering_angle / wheelbase
        """
        twist = Twist()
        
        # Linear velocity (forward speed)
        twist.linear.x = msg.drive.speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        
        # Calculate angular velocity from steering angle
        # For differential drive approximation of Ackermann steering
        steering_angle = msg.drive.steering_angle
        
        # Clamp steering angle to limits
        steering_angle = max(-self.max_steering_angle, 
                           min(self.max_steering_angle, steering_angle))
        
        # Calculate angular velocity
        if abs(steering_angle) > 0.001:  # Avoid division by zero
            # For small angles: omega = v * tan(delta) / L
            # where delta is steering angle and L is wheelbase
            twist.angular.z = msg.drive.speed * math.tan(steering_angle) / self.wheelbase
        else:
            twist.angular.z = 0.0
        
        # Zero out unused angular components
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        
        # Publish converted command
        self.twist_pub.publish(twist)
        
        # Optional: log conversion for debugging (comment out in production)
        # self.get_logger().debug(
        #     f'Converted: speed={msg.drive.speed:.2f}, '
        #     f'steering={steering_angle:.3f} -> '
        #     f'linear={twist.linear.x:.2f}, angular={twist.angular.z:.3f}'
        # )


def main(args=None):
    rclpy.init(args=args)
    node = AckermannToTwistNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
