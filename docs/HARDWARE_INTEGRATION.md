# Raspberry Pi Hardware Integration Guide

## Overview

This guide explains how to integrate the PiIO node with actual TurtleBot3 hardware (motors, encoders, LIDAR, IMU).

## Current Implementation

The `pi_io_node.py` currently has **placeholder** functions for hardware interface. You need to replace these with actual hardware drivers.

## Hardware Interfaces to Implement

### 1. Motor Control (DYNAMIXEL)

The TurtleBot3 uses DYNAMIXEL XL430-W250-T motors controlled via the OpenCR board.

**Current placeholder:**
```python
# In cmd_vel_callback():
# TODO: Send commands to actual motor controller
# Example: self.send_motor_commands(self.left_wheel_velocity, self.right_wheel_velocity)
```

**Implementation options:**

#### Option A: Use ROBOTIS OpenCR SDK

```python
from opencr import OpenCR

class PiIONode(Node):
    def __init__(self):
        # ... existing code ...
        self.opencr = OpenCR()
        self.opencr.connect()
    
    def cmd_vel_callback(self, msg: Twist):
        # ... existing velocity calculation ...
        
        # Send to motors
        self.opencr.set_wheel_velocity(
            left=self.left_wheel_velocity,
            right=self.right_wheel_velocity
        )
```

#### Option B: Use dynamixel_sdk

```python
from dynamixel_sdk import *

class PiIONode(Node):
    def __init__(self):
        # ... existing code ...
        
        # Initialize Dynamixel
        self.port_handler = PortHandler('/dev/ttyUSB0')
        self.packet_handler = PacketHandler(2.0)
        self.port_handler.openPort()
        self.port_handler.setBaudRate(1000000)
        
        self.LEFT_MOTOR_ID = 1
        self.RIGHT_MOTOR_ID = 2
    
    def send_motor_commands(self, left_vel, right_vel):
        # Convert velocities to DYNAMIXEL values
        left_val = int(left_vel * 41.7)  # Conversion factor
        right_val = int(right_vel * 41.7)
        
        # Write goal velocity
        self.packet_handler.write4ByteTxRx(
            self.port_handler, self.LEFT_MOTOR_ID, 104, left_val
        )
        self.packet_handler.write4ByteTxRx(
            self.port_handler, self.RIGHT_MOTOR_ID, 104, right_val
        )
```

### 2. Encoder Reading (Odometry)

**Current placeholder:**
```python
# In publish_odometry():
# Dead reckoning from commanded velocities (replace with actual encoder data)
linear_vel = self.current_cmd.linear.x
angular_vel = self.current_cmd.angular.z
```

**Implementation:**

```python
def read_encoders(self):
    """Read actual encoder values from motors."""
    # Read present position from DYNAMIXEL
    left_pos, _, _ = self.packet_handler.read4ByteTxRx(
        self.port_handler, self.LEFT_MOTOR_ID, 132
    )
    right_pos, _, _ = self.packet_handler.read4ByteTxRx(
        self.port_handler, self.RIGHT_MOTOR_ID, 132
    )
    
    return left_pos, right_pos

def compute_odometry_from_encoders(self, left_pos, right_pos):
    """Compute odometry from encoder positions."""
    # Calculate wheel travel
    delta_left = (left_pos - self.last_left_pos) * self.encoder_resolution
    delta_right = (right_pos - self.last_right_pos) * self.encoder_resolution
    
    # Compute linear and angular displacement
    delta_s = (delta_left + delta_right) / 2.0
    delta_theta = (delta_right - delta_left) / self.wheel_separation
    
    # Update pose
    self.theta += delta_theta
    self.x += delta_s * math.cos(self.theta)
    self.y += delta_s * math.sin(self.theta)
    
    # Update last positions
    self.last_left_pos = left_pos
    self.last_right_pos = right_pos
    
    # Compute velocities
    dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9
    linear_vel = delta_s / dt
    angular_vel = delta_theta / dt
    
    return linear_vel, angular_vel
```

### 3. LIDAR Integration (LDS-01/02)

**Current placeholder:**
```python
# In publish_laser_scan():
# TODO: Replace with actual LIDAR data reading
scan.ranges = [float('inf')] * num_readings
```

**Implementation options:**

#### Option A: Use urg_node2 (recommended for Hokuyo/similar)

Don't implement in PiIO - use existing ROS2 driver:

```bash
# Install
sudo apt install ros-humble-urg-node

# Launch separately
ros2 run urg_node urg_node_driver --ros-args -p serial_port:=/dev/ttyUSB0
```

Then **remove** the `publish_laser_scan()` call from PiIO.

#### Option B: Use hls_lfcd_lds_driver (for TurtleBot3 LDS)

```bash
# Install TurtleBot3 LDS driver
sudo apt install ros-humble-hls-lfcd-lds-driver

# Launch
ros2 run hls_lfcd_lds_driver hlds_laser_publisher
```

#### Option C: Integrate directly (if custom LIDAR)

```python
from rplidar import RPLidar

class PiIONode(Node):
    def __init__(self):
        # ... existing code ...
        self.lidar = RPLidar('/dev/ttyUSB1')
        self.lidar.start_motor()
    
    def publish_laser_scan(self, current_time):
        scan = LaserScan()
        scan.header.stamp = current_time.to_msg()
        scan.header.frame_id = 'base_scan'
        
        # Get LIDAR data
        scans = self.lidar.iter_scans()
        scan_data = next(scans)
        
        # Process and publish
        # ... convert scan_data to LaserScan format ...
        self.scan_pub.publish(scan)
```

### 4. IMU Integration

**Current placeholder:**
```python
# In publish_imu():
# TODO: Replace with actual IMU data reading
```

**Implementation:**

#### Option A: Use OpenCR IMU

```python
def read_imu(self):
    """Read IMU data from OpenCR."""
    imu_data = self.opencr.read_imu()
    return imu_data

def publish_imu(self, current_time):
    imu_data = self.read_imu()
    
    imu = Imu()
    imu.header.stamp = current_time.to_msg()
    imu.header.frame_id = 'imu_link'
    
    imu.orientation.x = imu_data['quat_x']
    imu.orientation.y = imu_data['quat_y']
    imu.orientation.z = imu_data['quat_z']
    imu.orientation.w = imu_data['quat_w']
    
    imu.angular_velocity.x = imu_data['gyro_x']
    imu.angular_velocity.y = imu_data['gyro_y']
    imu.angular_velocity.z = imu_data['gyro_z']
    
    imu.linear_acceleration.x = imu_data['accel_x']
    imu.linear_acceleration.y = imu_data['accel_y']
    imu.linear_acceleration.z = imu_data['accel_z']
    
    self.imu_pub.publish(imu)
```

## Complete Integration Example

Here's a complete example integrating all hardware:

```python
#!/usr/bin/env python3
"""
PiIO Node with Full Hardware Integration
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from dynamixel_sdk import *
import math

class PiIONode(Node):
    def __init__(self):
        super().__init__('pi_io_node')
        
        # ... existing parameter declarations ...
        
        # Initialize DYNAMIXEL
        self.init_dynamixel()
        
        # ... existing publishers/subscribers ...
        
        # Encoder state
        self.last_left_pos = 0
        self.last_right_pos = 0
        
        # ... existing timers ...
    
    def init_dynamixel(self):
        """Initialize DYNAMIXEL motors."""
        self.port_handler = PortHandler('/dev/ttyUSB0')
        self.packet_handler = PacketHandler(2.0)
        
        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open DYNAMIXEL port")
            return False
        
        if not self.port_handler.setBaudRate(1000000):
            self.get_logger().error("Failed to set baudrate")
            return False
        
        self.LEFT_MOTOR_ID = 1
        self.RIGHT_MOTOR_ID = 2
        
        # Enable torque
        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.LEFT_MOTOR_ID, 64, 1
        )
        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.RIGHT_MOTOR_ID, 64, 1
        )
        
        self.get_logger().info("DYNAMIXEL initialized")
        return True
    
    def cmd_vel_callback(self, msg: Twist):
        """Send velocity commands to motors."""
        self.last_cmd_time = self.get_clock().now()
        self.emergency_stop = False
        
        # Apply velocity limits
        linear_vel = max(-self.max_linear_vel, 
                        min(self.max_linear_vel, msg.linear.x))
        angular_vel = max(-self.max_angular_vel, 
                         min(self.max_angular_vel, msg.angular.z))
        
        # Convert to wheel velocities
        left_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        right_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        
        # Convert to DYNAMIXEL velocity units
        left_dxl = int(left_vel * 41.7)
        right_dxl = int(right_vel * 41.7)
        
        # Send commands
        self.packet_handler.write4ByteTxRx(
            self.port_handler, self.LEFT_MOTOR_ID, 104, left_dxl
        )
        self.packet_handler.write4ByteTxRx(
            self.port_handler, self.RIGHT_MOTOR_ID, 104, right_dxl
        )
    
    def stop_motors(self):
        """Emergency stop."""
        self.packet_handler.write4ByteTxRx(
            self.port_handler, self.LEFT_MOTOR_ID, 104, 0
        )
        self.packet_handler.write4ByteTxRx(
            self.port_handler, self.RIGHT_MOTOR_ID, 104, 0
        )
    
    def publish_odometry(self, current_time):
        """Publish odometry from encoder data."""
        # Read encoders
        left_pos, _, _ = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.LEFT_MOTOR_ID, 132
        )
        right_pos, _, _ = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.RIGHT_MOTOR_ID, 132
        )
        
        # Compute odometry
        # ... (use compute_odometry_from_encoders from above) ...
        
        # Create and publish message
        # ... (same as before) ...

# ... rest of the node ...

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
```

## Testing Hardware Integration

### 1. Test Motors

```bash
# On Raspberry Pi
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10

# Robot should move forward slowly
# Press Ctrl+C to stop
```

### 2. Test Encoders/Odometry

```bash
# Check odometry is being published
ros2 topic echo /odom

# Should show position changing as robot moves
```

### 3. Test LIDAR

```bash
# Check scan data
ros2 topic echo /scan

# Visualize in RViz
rviz2
# Add LaserScan display, set topic to /scan
```

### 4. Test Watchdog

```bash
# Publish command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" -r 10

# Stop publishing (Ctrl+C)
# Robot should automatically stop within watchdog_timeout
```

## Troubleshooting

### Motors not responding
- Check `/dev/ttyUSB0` exists: `ls /dev/ttyUSB*`
- Check permissions: `sudo chmod 666 /dev/ttyUSB0`
- Check DYNAMIXEL IDs match code
- Check power supply to motors

### Odometry drifting
- Calibrate wheel radius and separation
- Check encoder resolution
- Consider adding IMU fusion for better accuracy

### LIDAR not working
- Check LIDAR device: `ls /dev/ttyUSB*`
- Check baudrate matches LIDAR specs
- Try LIDAR manufacturer's test tool

## Additional Resources

- [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [DYNAMIXEL SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [OpenCR Documentation](https://emanual.robotis.com/docs/en/parts/controller/opencr10/)
