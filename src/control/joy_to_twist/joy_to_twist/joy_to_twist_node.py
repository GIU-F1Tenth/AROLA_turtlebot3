#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class JoyToTwistNode(Node):
    """
    ROS2 node that converts joystick (Joy) messages to Twist messages
    for manual control of TurtleBot3.
    
    Default joystick mapping (standard gamepad):
    - Left stick vertical (axis 1): Linear velocity (forward/backward)
    - Left stick horizontal (axis 0) or Right stick horizontal (axis 3): Angular velocity (left/right)
    - Button 4 (LB): Enable button (must be held to move)
    - Button 5 (RB): Turbo mode (increases speed)
    - Button 1 (B): Emergency stop (immediately stops the robot)
    """
    
    def __init__(self):
        super().__init__('joy_to_twist_node')
        
        # Declare parameters
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enable_topic', '/joy_enable')
        
        # Joystick axes mapping
        self.declare_parameter('axis_linear', 1)  # Left stick vertical
        self.declare_parameter('axis_angular', 0)  # Left stick horizontal (or use 3 for right stick)
        
        # Button mapping
        self.declare_parameter('enable_button', 4)  # LB button (index 4)
        self.declare_parameter('turbo_button', 5)   # RB button (index 5)
        self.declare_parameter('stop_button', 1)    # B button (index 1) - Emergency stop
        
        # Velocity scaling parameters
        self.declare_parameter('scale_linear', 0.5)  # Max linear velocity (m/s)
        self.declare_parameter('scale_angular', 1.0)  # Max angular velocity (rad/s)
        self.declare_parameter('scale_linear_turbo', 1.0)  # Turbo linear velocity
        self.declare_parameter('scale_angular_turbo', 1.5)  # Turbo angular velocity
        
        # Deadzone for joystick (to avoid drift)
        self.declare_parameter('deadzone', 0.1)
        
        # Get parameters
        joy_topic = self.get_parameter('joy_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        enable_topic = self.get_parameter('enable_topic').value
        
        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value
        self.enable_button = self.get_parameter('enable_button').value
        self.turbo_button = self.get_parameter('turbo_button').value
        self.stop_button = self.get_parameter('stop_button').value
        
        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value
        self.scale_linear_turbo = self.get_parameter('scale_linear_turbo').value
        self.scale_angular_turbo = self.get_parameter('scale_angular_turbo').value
        
        self.deadzone = self.get_parameter('deadzone').value
        
        # Create subscribers and publishers
        self.joy_sub = self.create_subscription(
            Joy,
            joy_topic,
            self.joy_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )
        
        self.enable_pub = self.create_publisher(
            Bool,
            enable_topic,
            10
        )
        
        # State variables
        self.enabled = False
        self.emergency_stop = False  # Emergency stop flag
        self.last_joy_msg = None
        
        self.get_logger().info('Joy to Twist node started')
        self.get_logger().info(f'Listening to: {joy_topic}')
        self.get_logger().info(f'Publishing to: {cmd_vel_topic}')
        self.get_logger().info(f'Enable button: {self.enable_button}, Turbo button: {self.turbo_button}, Stop button: {self.stop_button}')
        self.get_logger().info(f'Linear axis: {self.axis_linear}, Angular axis: {self.axis_angular}')
        
    def apply_deadzone(self, value):
        """Apply deadzone to joystick value to prevent drift."""
        if abs(value) < self.deadzone:
            return 0.0
        return value
    
    def joy_callback(self, msg: Joy):
        """
        Process joystick messages and convert to Twist commands.
        
        Args:
            msg (Joy): Joystick message containing axes and button states
        """
        self.last_joy_msg = msg
        
        # Check if we have enough axes and buttons
        if len(msg.axes) <= max(self.axis_linear, self.axis_angular):
            self.get_logger().warn(
                f'Joy message has {len(msg.axes)} axes, but need at least '
                f'{max(self.axis_linear, self.axis_angular) + 1}'
            )
            return
        
        if len(msg.buttons) <= max(self.enable_button, self.turbo_button, self.stop_button):
            self.get_logger().warn(
                f'Joy message has {len(msg.buttons)} buttons, but need at least '
                f'{max(self.enable_button, self.turbo_button, self.stop_button) + 1}'
            )
            return
        
        # Check emergency stop button (highest priority)
        stop_pressed = msg.buttons[self.stop_button] == 1
        if stop_pressed:
            self.emergency_stop = True
            self.get_logger().warn('EMERGENCY STOP ACTIVATED!', throttle_duration_sec=0.5)
            # Send stop command immediately
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            
            # Publish disabled state
            enable_msg = Bool()
            enable_msg.data = False
            self.enable_pub.publish(enable_msg)
            return
        
        # Release emergency stop when button is released
        if self.emergency_stop and not stop_pressed:
            self.emergency_stop = False
            self.get_logger().info('Emergency stop released - manual control available')
        
        # Don't allow movement if emergency stop is active
        if self.emergency_stop:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return
        
        # Check enable button
        enable_pressed = msg.buttons[self.enable_button] == 1
        turbo_pressed = msg.buttons[self.turbo_button] == 1
        
        # Publish enable state
        enable_msg = Bool()
        enable_msg.data = enable_pressed
        self.enable_pub.publish(enable_msg)
        
        # Create Twist message
        twist = Twist()
        
        if enable_pressed:
            # Get joystick values with deadzone applied
            linear_raw = self.apply_deadzone(msg.axes[self.axis_linear])
            angular_raw = self.apply_deadzone(msg.axes[self.axis_angular])
            
            # Select scale based on turbo mode
            if turbo_pressed:
                scale_linear = self.scale_linear_turbo
                scale_angular = self.scale_angular_turbo
            else:
                scale_linear = self.scale_linear
                scale_angular = self.scale_angular
            
            # Apply scaling
            twist.linear.x = linear_raw * scale_linear
            twist.angular.z = angular_raw * scale_angular
            
            # Log status (throttled to avoid spam)
            if abs(linear_raw) > 0.01 or abs(angular_raw) > 0.01:
                mode = "TURBO" if turbo_pressed else "NORMAL"
                self.get_logger().info(
                    f'[{mode}] Linear: {twist.linear.x:.2f} m/s, '
                    f'Angular: {twist.angular.z:.2f} rad/s',
                    throttle_duration_sec=1.0
                )
        else:
            # Not enabled, send zero velocities
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        # Publish Twist message
        self.cmd_vel_pub.publish(twist)
        
        # Update enabled state
        if enable_pressed != self.enabled:
            self.enabled = enable_pressed
            if self.enabled:
                self.get_logger().info('Manual control ENABLED')
            else:
                self.get_logger().info('Manual control DISABLED')


def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwistNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.cmd_vel_pub.publish(twist)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
