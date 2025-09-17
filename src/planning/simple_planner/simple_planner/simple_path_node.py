#!/usr/bin/env python3
"""
Simple Path Node for AROLA (Autonomous Racing Open Layered Architecture)

This node reads waypoints from a CSV file and publishes them as a Path message.
The velocity is encoded in the angular z (w) field of each pose in the path.

Expected CSV format: x_m, y_m, v_m/s (with optional header comments starting with #)
"""

import csv
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math


class SimplePathNode(Node):
    """Node that publishes a path from a CSV file with velocity encoded in angular.z"""

    def __init__(self):
        super().__init__('simple_path_node')

        # Declare parameters
        self.declare_parameter('csv_path', '')
        self.declare_parameter('invert', False)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)

        # Get parameters
        self.csv_path = self.get_parameter(
            'csv_path').get_parameter_value().string_value
        self.invert = self.get_parameter(
            'invert').get_parameter_value().bool_value
        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter(
            'publish_rate').get_parameter_value().double_value

        # Validate parameters
        if not self.csv_path:
            self.get_logger().error(
                "csv_path parameter is empty. Please provide a valid CSV file path.")
            return

        # Handle relative paths - make them relative to the workspace root
        if not os.path.isabs(self.csv_path):
            # Get the workspace root (assuming we're in a ROS 2 workspace)
            workspace_root = os.environ.get('ROS_WORKSPACE', os.getcwd())
            self.csv_path = os.path.join(workspace_root, self.csv_path)

        if not os.path.exists(self.csv_path):
            self.get_logger().error(f"CSV file not found: {self.csv_path}")
            # Try alternative path resolution
            alt_path = os.path.join(os.getcwd(), self.get_parameter(
                'csv_path').get_parameter_value().string_value)
            if os.path.exists(alt_path):
                self.csv_path = alt_path
                self.get_logger().info(
                    f"Found CSV file at alternative path: {self.csv_path}")
            else:
                return

        # Create publisher
        self.path_publisher = self.create_publisher(Path, '~/planned_path', 10)

        # Load waypoints from CSV
        self.waypoints = self.load_waypoints_from_csv()
        if not self.waypoints:
            self.get_logger().error("No valid waypoints loaded from CSV file")
            return

        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints from {self.csv_path}")

        # Create timer to publish path at specified rate
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_path)

        self.get_logger().info(
            f"Simple Path Node initialized. Publishing at {self.publish_rate} Hz")

    def load_waypoints_from_csv(self):
        """Load waypoints from CSV file"""
        waypoints = []

        try:
            with open(self.csv_path, 'r') as csvfile:
                # Skip comment lines that start with #
                lines = []
                for line in csvfile:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        lines.append(line)

                # Parse CSV data
                reader = csv.reader(lines)
                for row_num, row in enumerate(reader, 1):
                    try:
                        if len(row) >= 3:
                            x = float(row[0].strip())
                            y = float(row[1].strip())
                            v = float(row[2].strip())
                            waypoints.append((x, y, v))
                        else:
                            self.get_logger().warn(
                                f"Row {row_num} has insufficient columns: {row}")
                    except ValueError as e:
                        self.get_logger().warn(
                            f"Error parsing row {row_num}: {row} - {e}")

        except Exception as e:
            self.get_logger().error(f"Error reading CSV file: {e}")
            return []

        # Invert waypoints if requested
        if self.invert:
            waypoints.reverse()
            self.get_logger().info("Waypoint order inverted")

        return waypoints

    def publish_path(self):
        """Publish the path message"""
        if not self.waypoints:
            return

        # Create Path message
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id

        # Add each waypoint as a PoseStamped
        for i, (x, y, v) in enumerate(self.waypoints):
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header

            # Set position
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0

            # Calculate orientation based on direction to next waypoint
            if i < len(self.waypoints) - 1:
                next_x, next_y, _ = self.waypoints[i + 1]
                dx = next_x - x
                dy = next_y - y
                yaw = math.atan2(dy, dx)

                # Convert yaw to quaternion
                pose_stamped.pose.orientation.x = 0.0
                pose_stamped.pose.orientation.y = 0.0
                pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
                pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                # For the last waypoint, use the same orientation as the previous one
                if i > 0:
                    prev_x, prev_y, _ = self.waypoints[i - 1]
                    dx = x - prev_x
                    dy = y - prev_y
                    yaw = math.atan2(dy, dx)

                    pose_stamped.pose.orientation.x = 0.0
                    pose_stamped.pose.orientation.y = 0.0
                    pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
                    pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)
                else:
                    # Single waypoint case - default orientation
                    pose_stamped.pose.orientation.x = 0.0
                    pose_stamped.pose.orientation.y = 0.0
                    pose_stamped.pose.orientation.z = 0.0
                    pose_stamped.pose.orientation.w = 1.0

            # IMPORTANT: Encode velocity in angular.z field as specified
            # WARNING: This overwrites the z component of the quaternion orientation!
            # This breaks the quaternion norm but follows the specific requirement
            # to encode velocity in the angular.z (w) field
            pose_stamped.pose.orientation.z = v

            path_msg.poses.append(pose_stamped)

        # Publish the path
        self.path_publisher.publish(path_msg)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)

    try:
        node = SimplePathNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in simple_path_node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
