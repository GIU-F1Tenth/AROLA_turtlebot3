#!/usr/bin/env python3
"""
PiIO Node for TurtleBot3 Burger - Distributed Architecture

This lightweight node runs on the Raspberry Pi onboard the TurtleBot3.
It handles all low-level IO operations:
- Publishes odometry data from encoders
- Publishes LaserScan data from LIDAR
- Subscribes to /cmd_vel and controls motors
- Implements a watchdog to stop motors if no commands received

Author: Adapted for TurtleBot3 Distributed Architecture
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Header
import math
import time


class PiIONode(Node):
    """
    Lightweight IO node for Raspberry Pi.
    Handles sensor publishing and motor control with watchdog safety.
    """

    def __init__(self):
        super().__init__('pi_io_node')
        
        # Declare parameters
        self.declare_parameter('watchdog_timeout', 0.5)  # seconds
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('use_imu', True)
        
        # TurtleBot3 Burger physical parameters
        self.declare_parameter('wheel_radius', 0.033)  # meters
        self.declare_parameter('wheel_separation', 0.160)  # meters
        self.declare_parameter('max_linear_velocity', 0.22)  # m/s
        self.declare_parameter('max_angular_velocity', 2.84)  # rad/s
        
        # Get parameters
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_imu = self.get_parameter('use_imu').value
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        
        # QoS Profiles for distributed system
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, sensor_qos)
        if self.use_imu:
            self.imu_pub = self.create_publisher(Imu, self.imu_topic, sensor_qos)
        
        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            cmd_qos
        )
        
        # Watchdog variables
        self.last_cmd_time = self.get_clock().now()
        self.current_cmd = Twist()
        self.emergency_stop = False
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Motor control placeholders (replace with actual hardware interface)
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        
        # Timers
        publish_period = 1.0 / self.publish_rate
        self.publish_timer = self.create_timer(publish_period, self.publish_sensors)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)
        
        self.get_logger().info(f'PiIO Node initialized')
        self.get_logger().info(f'Watchdog timeout: {self.watchdog_timeout}s')
        self.get_logger().info(f'Publishing at {self.publish_rate} Hz')
        self.get_logger().info(f'Subscribed to: {self.cmd_vel_topic}')
        self.get_logger().info(f'Publishing odom to: {self.odom_topic}')
        self.get_logger().info(f'Publishing scan to: {self.scan_topic}')

    def cmd_vel_callback(self, msg: Twist):
        """
        Receive velocity commands from PC.
        Apply safety limits and send to motor controller.
        """
        # Update watchdog
        self.last_cmd_time = self.get_clock().now()
        self.emergency_stop = False
        
        # Apply velocity limits
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))
        
        # Convert to differential drive wheel velocities
        # v_left = (2*v - omega*L) / (2*r)
        # v_right = (2*v + omega*L) / (2*r)
        self.left_wheel_velocity = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        self.right_wheel_velocity = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        
        # Store command for state estimation
        self.current_cmd.linear.x = linear_vel
        self.current_cmd.angular.z = angular_vel
        
        # TODO: Send commands to actual motor controller
        # Example: self.send_motor_commands(self.left_wheel_velocity, self.right_wheel_velocity)
        
        # For now, log the command (comment out in production)
        # self.get_logger().debug(f'Cmd: lin={linear_vel:.3f}, ang={angular_vel:.3f}')

    def watchdog_check(self):
        """
        Safety watchdog: stop motors if no command received within timeout.
        """
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_cmd > self.watchdog_timeout and not self.emergency_stop:
            self.get_logger().warn('Watchdog timeout! Stopping motors.')
            self.emergency_stop = True
            self.stop_motors()

    def stop_motors(self):
        """
        Emergency stop: send zero velocities to motors.
        """
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = 0.0
        
        # TODO: Send stop command to actual motor controller
        # Example: self.send_motor_commands(0.0, 0.0)

    def publish_sensors(self):
        """
        Publish odometry and sensor data at configured rate.
        """
        current_time = self.get_clock().now()
        
        # Publish odometry
        self.publish_odometry(current_time)
        
        # Publish laser scan (if hardware available)
        self.publish_laser_scan(current_time)
        
        # Publish IMU (if hardware available)
        if self.use_imu:
            self.publish_imu(current_time)

    def publish_odometry(self, current_time):
        """
        Compute and publish odometry from encoder data or dead reckoning.
        """
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Dead reckoning from commanded velocities (replace with actual encoder data)
        # In production, read actual encoder values here
        linear_vel = self.current_cmd.linear.x
        angular_vel = self.current_cmd.angular.z
        
        # Update pose
        delta_theta = angular_vel * dt
        delta_x = linear_vel * math.cos(self.theta + delta_theta / 2.0) * dt
        delta_y = linear_vel * math.sin(self.theta + delta_theta / 2.0) * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Create odometry message
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Quaternion from theta
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_vel
        
        # Covariance (simplified - tune for your robot)
        odom.pose.covariance = [0.001] * 36
        odom.twist.covariance = [0.001] * 36
        
        self.odom_pub.publish(odom)

    def publish_laser_scan(self, current_time):
        """
        Publish laser scan data from LIDAR.
        Replace with actual LIDAR driver integration.
        """
        # TODO: Replace with actual LIDAR data reading
        # For now, create a placeholder scan
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = current_time.to_msg()
        scan.header.frame_id = 'base_scan'
        
        scan.angle_min = -3.14159  # -180 degrees
        scan.angle_max = 3.14159   # +180 degrees
        scan.angle_increment = 0.0174533  # 1 degree
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.12
        scan.range_max = 3.5
        
        # Placeholder: fill with dummy data (replace with actual LIDAR)
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [float('inf')] * num_readings
        scan.intensities = [0.0] * num_readings
        
        # Only publish if you have actual LIDAR connected
        # self.scan_pub.publish(scan)

    def publish_imu(self, current_time):
        """
        Publish IMU data if available.
        Replace with actual IMU driver integration.
        """
        # TODO: Replace with actual IMU data reading
        imu = Imu()
        imu.header = Header()
        imu.header.stamp = current_time.to_msg()
        imu.header.frame_id = 'imu_link'
        
        # Placeholder values (replace with actual IMU)
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = math.sin(self.theta / 2.0)
        imu.orientation.w = math.cos(self.theta / 2.0)
        
        imu.orientation_covariance = [0.01] * 9
        imu.angular_velocity_covariance = [0.01] * 9
        imu.linear_acceleration_covariance = [0.01] * 9
        
        # Only publish if you have actual IMU connected
        # self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = PiIONode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
