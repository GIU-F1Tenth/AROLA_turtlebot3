# Network Setup Guide for Distributed TurtleBot3

## Overview

The distributed architecture requires proper network configuration for ROS2 DDS discovery between PC and Raspberry Pi.

## Prerequisites

- PC and Raspberry Pi on the same network
- Both running ROS2 Humble on Ubuntu 22.04
- Network with multicast support (most home/office networks)

## Quick Setup

### On PC

```bash
# Set environment variables
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0

# Make permanent
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
source ~/.bashrc
```

### On Raspberry Pi

```bash
# Set same environment variables
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0

# Make permanent
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
source ~/.bashrc
```

### Firewall Configuration

**On both machines:**

```bash
# Allow ROS2 DDS ports
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp

# Verify
sudo ufw status
```

## Advanced Configuration

### Using Static Peers (for difficult networks)

If automatic discovery fails, use static peer configuration:

**On PC:**

```bash
export ROS_STATIC_PEERS="<raspberry-pi-ip>"
```

**On Raspberry Pi:**

```bash
export ROS_STATIC_PEERS="<pc-ip>"
```

### Cyclone DDS Configuration (Recommended)

Create a Cyclone DDS config file for better performance:

**File**: `~/cyclonedds.xml`

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
        </General>
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <Peers>
                <Peer address="<pc-ip>"/>
                <Peer address="<raspberry-pi-ip>"/>
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
```

**Enable on both machines:**

```bash
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
echo "export CYCLONEDDS_URI=file://\$HOME/cyclonedds.xml" >> ~/.bashrc
```

## Network Testing

### 1. Test Basic Connectivity

```bash
# From PC to Pi
ping <raspberry-pi-ip>

# From Pi to PC
ping <pc-ip>
```

### 2. Test ROS2 Discovery

**On Raspberry Pi:**

```bash
# Publish a test topic
ros2 topic pub /test std_msgs/msg/String "data: Hello from Pi" -r 1
```

**On PC:**

```bash
# Check if topic is visible
ros2 topic list | grep test

# Echo the topic
ros2 topic echo /test
```

You should see "Hello from Pi" messages.

### 3. Test Latency

```bash
# On PC
ros2 topic pub /test std_msgs/msg/String "data: test" -r 100

# On Pi (or another terminal on PC)
ros2 topic hz /test
```

Expected: ~100 Hz with low jitter

## Troubleshooting

### Issue: Can't discover nodes

**Check 1: Domain ID**
```bash
# On both machines, should be same
echo $ROS_DOMAIN_ID
```

**Check 2: Localhost only**
```bash
# Should be 0 on both
echo $ROS_LOCALHOST_ONLY
```

**Check 3: Firewall**
```bash
sudo ufw status
# Should show 7400:7500 allowed
```

### Issue: High latency or packet loss

**Solution 1: Use wired Ethernet**
- WiFi can have significant latency
- Use Gigabit Ethernet for best performance

**Solution 2: Optimize WiFi**
- Use 5GHz band if available
- Reduce interference (move away from microwaves, other networks)
- Position router for best signal

**Solution 3: Reduce publishing rates**
- Lower `publish_rate` in `pi_io_params.yaml`
- Reduce sensor data rates

### Issue: Connection drops randomly

**Solution: Use QoS Reliability**

The PiIO node already uses appropriate QoS:
- Best-effort for sensor data (allows some loss)
- Reliable for commands (ensures delivery)

Check configuration in `pi_io_node.py`.

## Network Monitoring

### Monitor bandwidth usage

```bash
# Install iftop
sudo apt install iftop

# Monitor network
sudo iftop -i <interface>  # e.g., eth0 or wlan0
```

### Monitor ROS2 topics

```bash
# List all topics
ros2 topic list

# Check publishing rates
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic hz /cmd_vel

# Check bandwidth per topic
ros2 topic bw /scan
```

## Optimal Network Setup

**Best Performance:**
```
PC (Gigabit Ethernet) ←→ Router ←→ Raspberry Pi (Gigabit Ethernet)
```

**Good Performance:**
```
PC (5GHz WiFi) ←→ Router ←→ Raspberry Pi (Ethernet)
```

**Acceptable Performance:**
```
PC (5GHz WiFi) ←→ Router ←→ Raspberry Pi (5GHz WiFi)
```

**Not Recommended:**
```
PC (2.4GHz WiFi) ←→ Router ←→ Raspberry Pi (2.4GHz WiFi)
```

## Security Considerations

### Change Default ROS_DOMAIN_ID

```bash
# Use a unique domain ID (0-101)
export ROS_DOMAIN_ID=42  # Choose your own
```

This isolates your robots from others on the same network.

### Use VPN for Remote Access

For accessing TurtleBot over internet:
1. Setup WireGuard VPN on both PC and Pi
2. Route ROS2 traffic through VPN
3. Keep ROS_DOMAIN_ID private

## Reference

- [ROS2 DDS Tuning](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)
- [CycloneDDS Configuration](https://github.com/eclipse-cyclonedds/cyclonedds)
- [ROS2 Network Configuration](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
