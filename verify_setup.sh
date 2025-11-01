#!/bin/bash
# Verification script for TurtleBot3 distributed architecture
# Checks that all necessary files and packages are in place

echo "================================================"
echo "TurtleBot3 Distributed Architecture Verification"
echo "================================================"
echo ""

ERRORS=0
WARNINGS=0

# Function to check file exists
check_file() {
    if [ -f "$1" ]; then
        echo "✓ Found: $1"
    else
        echo "✗ Missing: $1"
        ((ERRORS++))
    fi
}

# Function to check directory exists
check_dir() {
    if [ -d "$1" ]; then
        echo "✓ Found: $1"
    else
        echo "✗ Missing: $1"
        ((ERRORS++))
    fi
}

# Function to check optional file
check_optional() {
    if [ -f "$1" ]; then
        echo "✓ Found: $1"
    else
        echo "⚠ Optional: $1 (not found)"
        ((WARNINGS++))
    fi
}

echo "Checking package structure..."
echo ""

# Check new packages
echo "PiIO Package:"
check_dir "src/io/pi_io"
check_file "src/io/pi_io/package.xml"
check_file "src/io/pi_io/setup.py"
check_file "src/io/pi_io/pi_io/pi_io_node.py"
check_file "src/io/pi_io/launch/pi_bringup.launch.py"
check_file "src/io/pi_io/config/pi_io_params.yaml"
echo ""

echo "Ackermann to Twist Converter:"
check_dir "src/converters/ackermann_to_twist"
check_file "src/converters/ackermann_to_twist/package.xml"
check_file "src/converters/ackermann_to_twist/setup.py"
check_file "src/converters/ackermann_to_twist/ackermann_to_twist/ackermann_to_twist_node.py"
echo ""

echo "Core Package Updates:"
check_file "src/core/config/params_turtlebot3.yaml"
check_file "src/core/launch/pc_bringup.launch.py"
check_file "src/core/launch/simulation.launch.py"
echo ""

echo "Documentation:"
check_file "README_TURTLEBOT3.md"
check_file "IMPLEMENTATION_SUMMARY.md"
check_file "docs/NETWORK_SETUP.md"
check_file "docs/HARDWARE_INTEGRATION.md"
check_file "docs/QUICK_REFERENCE.md"
echo ""

echo "Setup Scripts:"
check_file "setup_turtlebot3.sh"
echo ""

# Check for existing packages
echo "Checking existing packages (should be present):"
check_dir "src/core"
check_dir "src/planning/simple_planner"
check_optional "src/control/pure_pursuit"
check_optional "src/monitor/race_monitor"
check_optional "src/watchdog/watchdog"
echo ""

# Check ROS2 environment
echo "Checking ROS2 environment..."
if command -v ros2 &> /dev/null; then
    echo "✓ ROS2 command found"
    ROS2_VERSION=$(ros2 --version 2>&1 | grep -o 'humble' || echo "unknown")
    if [ "$ROS2_VERSION" = "humble" ]; then
        echo "✓ ROS2 Humble detected"
    else
        echo "⚠ Warning: Expected ROS2 Humble, found: $ROS2_VERSION"
        ((WARNINGS++))
    fi
else
    echo "✗ ROS2 not found in PATH"
    ((ERRORS++))
fi
echo ""

# Check environment variables
echo "Checking environment variables..."
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo "⚠ ROS_DOMAIN_ID not set (recommended: export ROS_DOMAIN_ID=30)"
    ((WARNINGS++))
else
    echo "✓ ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
fi

if [ -z "$TURTLEBOT3_MODEL" ]; then
    echo "⚠ TURTLEBOT3_MODEL not set (recommended: export TURTLEBOT3_MODEL=burger)"
    ((WARNINGS++))
else
    echo "✓ TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
fi
echo ""

# Check if workspace is built
echo "Checking workspace build..."
if [ -d "install" ]; then
    echo "✓ Workspace appears to be built (install/ directory exists)"
    
    # Check for specific packages
    if [ -d "install/pi_io" ]; then
        echo "✓ pi_io package installed"
    else
        echo "⚠ pi_io package not found in install/"
        ((WARNINGS++))
    fi
    
    if [ -d "install/ackermann_to_twist" ]; then
        echo "✓ ackermann_to_twist package installed"
    else
        echo "⚠ ackermann_to_twist package not found in install/"
        ((WARNINGS++))
    fi
else
    echo "⚠ Workspace not built yet (run: colcon build)"
    ((WARNINGS++))
fi
echo ""

# Summary
echo "================================================"
echo "Verification Summary"
echo "================================================"
if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    echo "✓ All checks passed! Repository is ready."
    echo ""
    echo "Next steps:"
    echo "  1. Build workspace: colcon build --symlink-install"
    echo "  2. Source workspace: source install/setup.bash"
    echo "  3. Test simulation: ros2 launch core simulation.launch.py"
    exit 0
elif [ $ERRORS -eq 0 ]; then
    echo "✓ Core files present ($WARNINGS warnings)"
    echo ""
    echo "Warnings can usually be ignored, but check above for details."
    echo ""
    echo "If workspace not built, run:"
    echo "  colcon build --symlink-install"
    echo "  source install/setup.bash"
    exit 0
else
    echo "✗ Found $ERRORS error(s) and $WARNINGS warning(s)"
    echo ""
    echo "Please fix the errors above before proceeding."
    echo "Missing files may indicate incomplete setup or failed file creation."
    exit 1
fi
