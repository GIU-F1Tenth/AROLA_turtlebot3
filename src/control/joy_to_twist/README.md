# Joy to Twist - Manual Control for TurtleBot3

This ROS2 package converts joystick (Joy) messages to Twist messages for manual control of the TurtleBot3 robot.

## Features

- **Deadman switch**: Must hold enable button to move (safety feature)
- **Turbo mode**: Press turbo button for increased speed
- **Configurable mapping**: Easily customize button and axis assignments
- **Deadzone support**: Prevents joystick drift
- **Enable state publishing**: Publishes enable state for other nodes

## Requirements

- ROS2 Humble
- `joy` package (for joystick driver)
- A compatible joystick/gamepad

## Installation

1. Install the joy package if not already installed:
```bash
sudo apt-get install ros-humble-joy
```

2. Build the package:
```bash
cd /home/user/turtulebot3/minimal_sofware_example
colcon build --packages-select joy_to_twist
source install/setup.bash
```

## Default Joystick Mapping

### Standard Gamepad (Logitech F710, Xbox Controller, etc.)

**Axes:**
- **Left Stick Vertical** (axis 1): Linear velocity (forward/backward)
- **Left Stick Horizontal** (axis 0): Angular velocity (turn left/right)

**Buttons:**
- **LB (button 4)**: Enable button - **MUST HOLD to move the robot**
- **RB (button 5)**: Turbo mode - Hold for higher speed

## Usage

### Basic Launch

Launch with default settings:
```bash
ros2 launch joy_to_twist joy_to_twist.launch.py
```

### Custom Joystick Device

If your joystick is on a different device:
```bash
ros2 launch joy_to_twist joy_to_twist.launch.py joy_dev:=/dev/input/js1
```

### Run Individual Nodes

Run joy node only:
```bash
ros2 run joy joy_node
```

Run converter node only (if joy node already running):
```bash
ros2 run joy_to_twist joy_to_twist_node --ros-args --params-file install/joy_to_twist/share/joy_to_twist/config/joy_to_twist_params.yaml
```

## Testing Your Joystick

### 1. Test if joystick is detected:
```bash
ls /dev/input/js*
```
You should see `/dev/input/js0` (or js1, js2, etc.)

### 2. Test joystick input:
```bash
sudo jstest /dev/input/js0
```
Move sticks and press buttons to see the values

### 3. Check Joy messages in ROS2:
```bash
ros2 topic echo /joy
```
Move your joystick and you should see the messages

### 4. Check Twist output:
```bash
ros2 topic echo /cmd_vel
```
Hold the enable button (LB) and move the left stick

## Configuration

Edit `config/joy_to_twist_params.yaml` to customize:

```yaml
joy_to_twist_node:
  ros__parameters:
    # Topics
    joy_topic: "/joy"
    cmd_vel_topic: "/cmd_vel"
    
    # Axis mapping
    axis_linear: 1   # Forward/backward
    axis_angular: 0  # Left/right turn
    
    # Buttons
    enable_button: 4  # LB - must hold to move
    turbo_button: 5   # RB - turbo speed
    
    # Speed limits (m/s and rad/s)
    scale_linear: 0.22        # Normal speed
    scale_angular: 2.84       # Normal turn rate
    scale_linear_turbo: 0.5   # Turbo speed
    scale_angular_turbo: 3.5  # Turbo turn rate
    
    # Deadzone
    deadzone: 0.1
```

## Topics

### Subscribed Topics
- `/joy` (`sensor_msgs/Joy`): Joystick input from joy_node

### Published Topics
- `/cmd_vel` (`geometry_msgs/Twist`): Velocity commands for the robot
- `/joy_enable` (`std_msgs/Bool`): Enable state (true when enable button pressed)

## Safety Features

1. **Deadman Switch**: Robot only moves when enable button is held
2. **Zero on Release**: Automatically sends zero velocity when button released
3. **Deadzone**: Prevents unintended movement from joystick drift
4. **Configurable Limits**: Set maximum velocities to safe values

## Troubleshooting

### Joystick not detected
```bash
# Check permissions
sudo chmod a+rw /dev/input/js0

# Or add user to input group
sudo usermod -a -G input $USER
# Then log out and back in
```

### Wrong button/axis mapping
- Run `jstest /dev/input/js0` to see your joystick's button/axis numbers
- Update the config file with correct numbers
- Rebuild: `colcon build --packages-select joy_to_twist`

### Robot not moving
1. Check if enable button is held (LB button)
2. Verify joy messages: `ros2 topic echo /joy`
3. Check cmd_vel output: `ros2 topic echo /cmd_vel`
4. Ensure joystick has power (wireless controllers)

## Common Joystick Models

### Logitech F710 (DirectInput mode)
- Enable: Button 4 (LB)
- Turbo: Button 5 (RB)
- Linear: Axis 1 (Left stick vertical)
- Angular: Axis 0 (Left stick horizontal)

### Xbox Wireless Controller
- Enable: Button 4 (LB)
- Turbo: Button 5 (RB)
- Linear: Axis 1 (Left stick vertical)
- Angular: Axis 0 (Left stick horizontal)

### PS4 Controller
- Enable: Button 4 (L1)
- Turbo: Button 5 (R1)
- Linear: Axis 1 (Left stick vertical)
- Angular: Axis 0 (Left stick horizontal)

## Integration with TurtleBot3

This package is designed to work seamlessly with TurtleBot3. The default velocity limits match TurtleBot3's capabilities:
- Max linear velocity: ~0.22 m/s
- Max angular velocity: ~2.84 rad/s

## License

MIT License
