# TurtleBot3 Burger Distributed Architecture - Racing Software Stack

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Platform: TurtleBot3](https://img.shields.io/badge/Platform-TurtleBot3-green.svg)](https://emanual.robotis.com/docs/en/platform/turtlebot3/)

## Overview

This repository contains a **distributed racing software stack** for the **TurtleBot3 Burger**, adapted from the F1TENTH AROLA (Autonomous Racing Open Layered Architecture). 

The system uses a **distributed architecture** where:
- 🖥️ **PC (Workstation)**: Runs all computationally intensive nodes (planning, control, localization, perception)
- 🥧 **Raspberry Pi (onboard)**: Runs only lightweight IO operations (sensor publishing, motor control)

This architecture enables:
- Advanced control algorithms on powerful hardware
- Real-time motor control on embedded system
- Easy development and testing on PC
- Seamless deployment to hardware

## Table of Contents

- [Architecture](#architecture)
- [System Requirements](#system-requirements)
- [Quick Start](#quick-start)
  - [Simulation Testing](#simulation-testing)
  - [Hardware Deployment](#hardware-deployment)
- [Network Configuration](#network-configuration)
- [Package Overview](#package-overview)
- [Configuration](#configuration)
- [Development Workflow](#development-workflow)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Architecture

### Distributed System Design

```
┌─────────────────────────────────────────────────────────────┐
│                    PC (Workstation)                         │
├─────────────────────────────────────────────────────────────┤
│  • AMCL Localization                                        │
│  • Pure Pursuit Controller                                  │
│  • Path Planner                                             │
│  • Ackermann→Twist Converter                                │
│  • Watchdog Safety Monitor                                  │
│  • TF Transforms                                            │
└────────────────────┬────────────────────────────────────────┘
                     │
                     │ ROS2 DDS (Network)
                     │ Topics: /odom, /scan, /cmd_vel
                     │
┌────────────────────▼────────────────────────────────────────┐
│         Raspberry Pi (TurtleBot3 Onboard)                   │
├─────────────────────────────────────────────────────────────┤
│  • PiIO Node (Lightweight)                                  │
│    - Publishes /odom (from encoders)                        │
│    - Publishes /scan (from LIDAR)                           │
│    - Publishes /imu (from IMU)                              │
│    - Subscribes to /cmd_vel                                 │
│    - Controls DYNAMIXEL motors                              │
│    - Watchdog safety (auto-stop if no commands)            │
└─────────────────────────────────────────────────────────────┘
```

### Topic Flow

```
PC Side:                                          Pi Side:
┌──────────────┐     /drive      ┌──────────────┐
│ Pure Pursuit ├────────────────►│  Ackermann   │
└──────────────┘  (Ackermann)    │  to Twist    │
                                 └──────┬───────┘
                                        │ /cmd_vel (Twist)
                                        ▼
                  ┌─────────────────────┼─────────────────┐
                  │                     ▼                 │
                  │              ┌──────────────┐         │
                  │              │   PiIO Node  │         │
                  │              └──────┬───────┘         │
                  │                     │                 │
                  │    /odom ◄──────────┤                 │
                  │    /scan ◄──────────┤                 │
                  │    /imu  ◄──────────┘                 │
                  └─────────────────────────────────────────┘
```

## System Requirements

### PC (Workstation)

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **RAM**: 8GB+ recommended
- **Processor**: Multi-core (4+ cores recommended)
- **Network**: Gigabit Ethernet or 5GHz WiFi

### Raspberry Pi (TurtleBot3 Onboard)

- **Model**: Raspberry Pi 3B+ or 4 (2GB+ RAM)
- **OS**: Ubuntu 22.04 Server (ARM64)
- **ROS2**: Humble Hawksbill
- **Storage**: 16GB+ microSD card
- **Network**: Must be on same network as PC

### Additional Hardware

- TurtleBot3 Burger with:
  - DYNAMIXEL motors (XL430-W250-T)
  - LDS-01 or LDS-02 LIDAR
  - IMU (optional but recommended)
  - OpenCR board (motor controller)

## Quick Start

### Installation

#### 1. PC Setup

```bash
# Install ROS2 Humble (if not already installed)
# Follow: https://docs.ros.org/en/humble/Installation.html

# Install dependencies
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-description \
    ros-humble-ackermann-msgs \
    python3-pip

# Clone this repository
cd ~
mkdir -p turtlebot3_ws/src
cd turtlebot3_ws/src
git clone <this-repository-url>
cd ..

# Initialize and update submodules
git submodule update --init --recursive

# Install Python dependencies
pip3 install -r src/<repo-name>/requirements.txt

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Set TurtleBot3 model
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

#### 2. Raspberry Pi Setup

```bash
# On Raspberry Pi (SSH into it)
# Install ROS2 Humble
# Follow: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# Install lightweight dependencies
sudo apt update
sudo apt install -y \
    ros-humble-rclpy \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs

# Clone ONLY the pi_io package (lightweight)
cd ~
mkdir -p turtlebot3_ws/src
cd turtlebot3_ws/src
# Copy pi_io package from PC or clone repo
# For minimal install, you only need: src/io/pi_io/

# Build only PiIO package
cd ~/turtlebot3_ws
colcon build --packages-select pi_io --symlink-install

# Source the workspace
source install/setup.bash
```

### Simulation Testing

Before deploying to hardware, test everything in simulation:

```bash
# On PC

# Terminal 1: Launch Gazebo simulation
export TURTLEBOT3_MODEL=burger
ros2 launch core simulation.launch.py

# The simulation includes:
# - TurtleBot3 in Gazebo
# - All PC-side compute nodes
# - Ackermann to Twist converter

# Terminal 2: Visualize in RViz
rviz2

# Terminal 3: Check topics
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /cmd_vel

# Terminal 4: Test with teleop
ros2 run turtlebot3_teleop teleop_keyboard
```

### Hardware Deployment

#### Network Configuration

**CRITICAL**: Both PC and Pi must be configured correctly for ROS2 discovery.

##### On PC:

```bash
# Set ROS Domain ID (use same on both machines)
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc

# Set your PC's IP address for faster discovery (optional but recommended)
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc

# Source
source ~/.bashrc
```

##### On Raspberry Pi:

```bash
# Set same ROS Domain ID
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
source ~/.bashrc
```

##### Firewall Configuration

```bash
# On both PC and Pi, allow ROS2 DDS ports
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp
```

#### Running the System

##### 1. Start Pi-side (on Raspberry Pi via SSH):

```bash
# SSH into Raspberry Pi
ssh ubuntu@<raspberry-pi-ip>

# Source workspace
cd ~/turtlebot3_ws
source install/setup.bash

# Set ROS_DOMAIN_ID
export ROS_DOMAIN_ID=30

# Launch Pi IO node
ros2 launch pi_io pi_bringup.launch.py

# You should see:
# [INFO] [pi_io_node]: PiIO Node initialized
# [INFO] [pi_io_node]: Watchdog timeout: 0.5s
# [INFO] [pi_io_node]: Publishing at 50.0 Hz
```

##### 2. Start PC-side (on your workstation):

```bash
# Terminal 1: Launch PC compute nodes
export ROS_DOMAIN_ID=30
cd ~/turtlebot3_ws
source install/setup.bash

ros2 launch core pc_bringup.launch.py \
    waypoint_file:=/path/to/waypoints.csv \
    map:=/path/to/map.yaml

# You should see all nodes starting:
# [INFO] Starting TurtleBot3 PC-side compute nodes...
# [INFO] [pure_pursuit_node]: Pure Pursuit initialized
# [INFO] [amcl]: AMCL initialized
# etc.
```

##### 3. Verify Communication:

```bash
# Terminal 2: Check topics
ros2 topic list

# Should see:
# /odom (from Pi)
# /scan (from Pi)
# /cmd_vel (to Pi)
# /drive (from pure_pursuit)
# etc.

# Check that data is flowing
ros2 topic hz /odom    # Should show ~50 Hz
ros2 topic hz /scan    # Should show LIDAR rate
ros2 topic echo /cmd_vel # Should show commands when moving
```

## Package Overview

### Core Packages

#### `pi_io`
**Location**: `src/io/pi_io/`  
**Runs on**: Raspberry Pi  
**Purpose**: Lightweight IO handler for hardware interface
- Publishes odometry from encoders
- Publishes LIDAR scans
- Publishes IMU data (if available)
- Subscribes to `/cmd_vel` and controls motors
- Implements watchdog safety (auto-stop if no commands)

#### `ackermann_to_twist`
**Location**: `src/converters/ackermann_to_twist/`  
**Runs on**: PC  
**Purpose**: Converts Ackermann steering to differential drive Twist
- Subscribes to `/drive` (AckermannDriveStamped)
- Publishes to `/cmd_vel` (Twist)
- Handles kinematic conversion for differential drive

#### `pure_pursuit`
**Location**: `src/control/pure_pursuit/` (submodule)  
**Runs on**: PC  
**Purpose**: Trajectory following controller
- Outputs Ackermann steering commands
- Configurable lookahead distance and velocity

#### `simple_planner`
**Location**: `src/planning/simple_planner/`  
**Runs on**: PC  
**Purpose**: Publishes waypoint paths from CSV files
- Reads waypoint files (x, y, velocity)
- Publishes as nav_msgs/Path

#### `core`
**Location**: `src/core/`  
**Runs on**: PC  
**Purpose**: TF transforms and system configuration
- Manages coordinate frame transforms
- Contains configuration files and launch files

### Launch Files

| Launch File | Location | Purpose |
|------------|----------|---------|
| `pc_bringup.launch.py` | `src/core/launch/` | All PC-side compute nodes |
| `pi_bringup.launch.py` | `src/io/pi_io/launch/` | Raspberry Pi IO node |
| `simulation.launch.py` | `src/core/launch/` | Full system simulation in Gazebo |

## Configuration

### Main Configuration File

**File**: `src/core/config/params_turtlebot3.yaml`

Key parameters to adjust:

```yaml
# TurtleBot3 Physical Parameters
pi_io_node:
  ros__parameters:
    wheel_radius: 0.033          # meters
    wheel_separation: 0.160      # meters
    max_linear_velocity: 0.22    # m/s
    max_angular_velocity: 2.84   # rad/s

# Control Parameters
pure_pursuit_node:
  ros__parameters:
    max_velocity: 0.22           # Match TurtleBot3 limits
    min_velocity: 0.05
    max_lookahead_distance: 0.8  # Adjusted for scale
    min_lookahead_distance: 0.3
    kp: 0.8                      # Proportional gain
    kd: 0.5                      # Derivative gain
```

### Waypoint File Format

Create CSV files with waypoints:

```csv
# x_m, y_m, v_m/s
0.0, 0.0, 0.15
1.0, 0.0, 0.20
1.0, 1.0, 0.18
0.0, 1.0, 0.15
0.0, 0.0, 0.10
```

## Development Workflow

### 1. Develop on PC in Simulation

```bash
# Test your changes in Gazebo first
ros2 launch core simulation.launch.py waypoint_file:=/path/to/test_waypoints.csv

# Use RViz for visualization
rviz2
```

### 2. Test Network Communication

```bash
# On PC: publish test messages
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"

# On Pi: verify reception
ros2 topic echo /cmd_vel
```

### 3. Deploy to Hardware

```bash
# Build on PC
colcon build --symlink-install

# Copy only pi_io to Raspberry Pi (if needed)
rsync -avz src/io/pi_io/ ubuntu@<pi-ip>:~/turtlebot3_ws/src/pi_io/

# Build on Pi
ssh ubuntu@<pi-ip>
cd ~/turtlebot3_ws
colcon build --packages-select pi_io
```

### 4. Debug with Logging

```bash
# Increase logging verbosity
ros2 launch core pc_bringup.launch.py log_level:=debug

# View logs
ros2 run rqt_console rqt_console
```

## Troubleshooting

### Issue: Nodes can't discover each other

**Symptoms**: PC can't see Pi's topics or vice versa

**Solutions**:
1. Check ROS_DOMAIN_ID matches on both machines:
   ```bash
   echo $ROS_DOMAIN_ID  # Should be same on both
   ```

2. Check network connectivity:
   ```bash
   ping <other-machine-ip>
   ```

3. Check firewall:
   ```bash
   sudo ufw status
   sudo ufw allow 7400:7500/udp
   ```

4. Try using `ROS_STATIC_PEERS`:
   ```bash
   export ROS_STATIC_PEERS="<pc-ip>,<pi-ip>"
   ```

### Issue: Watchdog stops motors immediately

**Symptoms**: Motors stop as soon as they start

**Solutions**:
1. Check `/cmd_vel` is being published:
   ```bash
   ros2 topic hz /cmd_vel
   ```

2. Increase watchdog timeout in config:
   ```yaml
   pi_io_node:
     ros__parameters:
       watchdog_timeout: 1.0  # Increase from 0.5
   ```

### Issue: Robot doesn't follow path accurately

**Solutions**:
1. Tune pure pursuit gains in `params_turtlebot3.yaml`
2. Adjust lookahead distances
3. Check odometry accuracy:
   ```bash
   ros2 topic echo /odom
   ```

### Issue: High latency in distributed system

**Solutions**:
1. Use wired Ethernet instead of WiFi
2. Reduce publishing rates in config
3. Use QoS profiles optimized for your network

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Test in simulation first
4. Submit a pull request with clear description

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Original F1TENTH AROLA architecture
- TurtleBot3 platform by ROBOTIS
- ROS2 Navigation Stack (Nav2)

## Support

For issues and questions:
- Open an issue on GitHub
- Check [TurtleBot3 manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- Check [ROS2 documentation](https://docs.ros.org/en/humble/)

---

**Happy Racing! 🏁🤖**
