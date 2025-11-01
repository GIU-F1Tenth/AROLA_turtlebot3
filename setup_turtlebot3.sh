#!/bin/bash
# Quick setup script for TurtleBot3 distributed architecture
# Run this on PC after cloning the repository

set -e

echo "================================================"
echo "TurtleBot3 Distributed Architecture Setup"
echo "================================================"
echo ""

# Check if ROS2 Humble is installed
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ROS2 not found. Please install ROS2 Humble first."
    echo "Visit: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo "✓ ROS2 Humble detected"

# Check if we're in a workspace
if [ ! -f "src/CMakeLists.txt" ] && [ ! -d "src" ]; then
    echo "ERROR: Please run this script from the workspace root (where src/ directory is)"
    exit 1
fi

echo "✓ Workspace structure detected"
echo ""

# Install dependencies
echo "Installing system dependencies..."
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-description \
    ros-humble-ackermann-msgs \
    python3-pip \
    python3-colcon-common-extensions

echo "✓ System dependencies installed"
echo ""

# Initialize submodules
echo "Initializing git submodules..."
if [ -d ".git" ]; then
    git submodule update --init --recursive
    echo "✓ Submodules initialized"
else
    echo "⚠ Not a git repository, skipping submodules"
fi
echo ""

# Install Python requirements if file exists
if [ -f "requirements.txt" ]; then
    echo "Installing Python dependencies..."
    pip3 install -r requirements.txt
    echo "✓ Python dependencies installed"
elif [ -f "src/requirements.txt" ]; then
    echo "Installing Python dependencies..."
    pip3 install -r src/requirements.txt
    echo "✓ Python dependencies installed"
fi
echo ""

# Build the workspace
echo "Building workspace..."
colcon build --symlink-install
echo "✓ Workspace built"
echo ""

# Setup environment
echo "Setting up environment..."
if ! grep -q "export TURTLEBOT3_MODEL=burger" ~/.bashrc; then
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    echo "✓ Added TURTLEBOT3_MODEL to ~/.bashrc"
fi

if ! grep -q "export ROS_DOMAIN_ID=30" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
    echo "✓ Added ROS_DOMAIN_ID to ~/.bashrc"
fi

if ! grep -q "export ROS_LOCALHOST_ONLY=0" ~/.bashrc; then
    echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
    echo "✓ Added ROS_LOCALHOST_ONLY to ~/.bashrc"
fi

# Configure firewall
echo ""
echo "Configuring firewall for ROS2..."
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp
echo "✓ Firewall configured"
echo ""

echo "================================================"
echo "Setup Complete! 🎉"
echo "================================================"
echo ""
echo "Next steps:"
echo "  1. Source your workspace:"
echo "     source install/setup.bash"
echo ""
echo "  2. Test in simulation:"
echo "     ros2 launch core simulation.launch.py"
echo ""
echo "  3. For hardware deployment:"
echo "     - Setup Raspberry Pi (see README_TURTLEBOT3.md)"
echo "     - Launch PC nodes: ros2 launch core pc_bringup.launch.py"
echo "     - Launch Pi nodes: ros2 launch pi_io pi_bringup.launch.py"
echo ""
echo "Read README_TURTLEBOT3.md for detailed instructions."
echo ""
