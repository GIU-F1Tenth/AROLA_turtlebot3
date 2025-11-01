# TurtleBot3 Distributed Architecture - Quick Reference

## System Architecture

```
PC (Heavy Compute)              Raspberry Pi (Lightweight IO)
==================              =============================
• Pure Pursuit                  • PiIO Node
• AMCL Localization              - Publish /odom
• Path Planner                   - Publish /scan
• Ackermann→Twist                - Subscribe /cmd_vel
• Watchdog                       - Control motors
                                 - Watchdog safety

              ROS2 Network
        (Same ROS_DOMAIN_ID)
```

## Quick Commands

### Simulation

```bash
# Terminal 1: Launch simulation
ros2 launch core simulation.launch.py

# Terminal 2: Teleop control
ros2 run turtlebot3_teleop teleop_keyboard
```

### Hardware Deployment

```bash
# On Raspberry Pi (SSH)
export ROS_DOMAIN_ID=30
ros2 launch pi_io pi_bringup.launch.py

# On PC
export ROS_DOMAIN_ID=30
ros2 launch core pc_bringup.launch.py \
    waypoint_file:=/path/to/waypoints.csv \
    map:=/path/to/map.yaml
```

## Essential Topics

| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|------------|---------|
| `/odom` | Odometry | Pi | PC (AMCL, Controller) | Robot pose |
| `/scan` | LaserScan | Pi | PC (AMCL) | LIDAR data |
| `/cmd_vel` | Twist | PC (Converter) | Pi | Motor commands |
| `/drive` | AckermannDrive | PC (Pure Pursuit) | PC (Converter) | Control output |
| `/pp_path` | Path | PC (Planner) | PC (Pure Pursuit) | Waypoints |

## Key Parameters (params_turtlebot3.yaml)

### Robot Physical
```yaml
wheel_radius: 0.033          # meters
wheel_separation: 0.160      # meters
max_linear_velocity: 0.22    # m/s
max_angular_velocity: 2.84   # rad/s
```

### Control Tuning
```yaml
max_lookahead_distance: 0.8  # meters
min_lookahead_distance: 0.3  # meters
kp: 0.8                      # Proportional gain
kd: 0.5                      # Derivative gain
```

### Safety
```yaml
watchdog_timeout: 0.5        # seconds (motors stop if no cmd)
```

## Network Setup

### Environment Variables
```bash
export ROS_DOMAIN_ID=30              # Same on PC and Pi
export ROS_LOCALHOST_ONLY=0          # Enable network discovery
export TURTLEBOT3_MODEL=burger       # TurtleBot3 model
```

### Firewall
```bash
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp
```

## Debugging Commands

### Check Topics
```bash
ros2 topic list                # List all topics
ros2 topic hz /odom            # Check publishing rate
ros2 topic echo /cmd_vel       # View messages
ros2 topic info /scan          # Topic details
```

### Check Nodes
```bash
ros2 node list                 # List all nodes
ros2 node info /pi_io_node     # Node details
```

### Check Network Discovery
```bash
# On Pi: publish test
ros2 topic pub /test std_msgs/msg/String "data: Hello" -r 1

# On PC: check if visible
ros2 topic echo /test
```

### Monitor Performance
```bash
ros2 topic bw /scan            # Bandwidth usage
ros2 topic hz /odom            # Publishing frequency
```

## Directory Structure

```
src/
├── converters/
│   └── ackermann_to_twist/    # Ackermann→Twist converter
├── control/
│   └── pure_pursuit/          # Trajectory controller (submodule)
├── core/                      # TF transforms, launch files, config
│   ├── config/
│   │   └── params_turtlebot3.yaml
│   └── launch/
│       ├── pc_bringup.launch.py
│       └── simulation.launch.py
├── io/
│   └── pi_io/                 # Raspberry Pi IO node
│       ├── launch/
│       │   └── pi_bringup.launch.py
│       └── pi_io/
│           └── pi_io_node.py
├── monitor/
│   └── race_monitor/          # Performance monitoring (submodule)
├── planning/
│   └── simple_planner/        # Waypoint path publisher
└── watchdog/
    └── watchdog/              # Safety monitor (submodule)
```

## Waypoint File Format

```csv
# x_m, y_m, v_m/s
0.0, 0.0, 0.15
1.0, 0.0, 0.20
1.0, 1.0, 0.18
0.0, 1.0, 0.15
0.0, 0.0, 0.10
```

## Common Issues & Fixes

### Can't discover nodes
```bash
# Check ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID  # On both PC and Pi

# Check firewall
sudo ufw status

# Try static peers
export ROS_STATIC_PEERS="<pc-ip>,<pi-ip>"
```

### Motors stop immediately
```bash
# Check cmd_vel is being published
ros2 topic hz /cmd_vel

# Increase watchdog timeout in config
```

### High latency
```bash
# Use wired Ethernet if possible
# Reduce publishing rates in config
# Check network with: ping <other-machine>
```

### Robot doesn't follow path
```bash
# Check path is being published
ros2 topic echo /pp_path

# Check localization
ros2 topic echo /odom

# Tune pure_pursuit gains in config
```

## Build & Install

```bash
# Clone repository
git clone <repo-url>
cd <repo-dir>

# Run setup script
./setup_turtlebot3.sh

# Or manual:
colcon build --symlink-install
source install/setup.bash
```

## Documentation Files

- `README_TURTLEBOT3.md` - Main documentation
- `docs/NETWORK_SETUP.md` - Network configuration details
- `docs/HARDWARE_INTEGRATION.md` - Hardware interface guide
- `docs/QUICK_REFERENCE.md` - This file

## Useful Links

- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
