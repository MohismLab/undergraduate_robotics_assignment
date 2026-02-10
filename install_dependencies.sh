#!/bin/bash
set -e

# Ask for sudo password upfront
sudo -v

echo "Updating apt repositories..."
sudo apt update

echo "Installing ROS 2 and System Dependencies..."
# Dependencies listed in README and required for simulation/navigation
sudo apt install -y \
    ros-jazzy-webots-ros2 \
    ros-jazzy-turtlebot3-description \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-slam-toolbox \
    ros-jazzy-tf-transformations

echo "Installing Python libraries via pip..."
pip3 install numpy matplotlib scipy PyYAML

echo "Running rosdep to ensure all package.xml dependencies are satisfied..."
if command -v rosdep &> /dev/null; then
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        echo "Initializing rosdep..."
        sudo rosdep init
    fi
    echo "Updating rosdep..."
    rosdep update
    
    echo "Installing dependencies for source packages..."
    rosdep install --from-paths src --ignore-src -r -y
else
    echo "Warning: rosdep command not found. Skipping rosdep install."
fi

echo "All dependencies installed successfully."
