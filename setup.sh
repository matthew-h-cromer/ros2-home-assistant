#!/bin/bash
# Setup script for ros2-home-assistant

set -e

echo "Setting up ros2-home-assistant..."

# Check for ROS2
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ROS2 is not installed or not sourced."
    echo "Please install ROS2 Kilted and source it first:"
    echo "  source /opt/ros/kilted/setup.bash"
    exit 1
fi

# Check for colcon
if ! command -v colcon &> /dev/null; then
    echo "ERROR: colcon is not installed."
    echo "Please install colcon:"
    echo "  sudo apt install python3-colcon-common-extensions"
    exit 1
fi

# Install system dependencies
echo "Installing system dependencies..."
sudo apt update
sudo apt install -y portaudio19-dev

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install -r requirements.txt

# Build the workspace
echo "Building ROS2 workspace..."
colcon build --symlink-install

echo ""
echo "Setup complete!"
echo ""
echo "Add this to your ~/.bashrc for convenience:"
echo '  alias ros2-home-assistant="cd ~/Documents/ros2-home-assistant && source install/setup.bash"'
echo ""
echo "Then run: source install/setup.bash"
