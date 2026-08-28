#!/bin/bash

set -e

echo "========================================="
echo "Installing dependencies for rover_description"
echo "========================================="

sudo apt update

echo "Installing build tools..."

sudo apt install -y \
    git \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep

echo "Installing ROS packages..."

sudo apt install -y \
    ros-humble-xacro \
    ros-humble-rclpy \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-forward-command-controller \
    ros-humble-joint-trajectory-controller \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-interfaces \
    ros-humble-gz-ros2-control

echo "Installing Python packages..."

pip3 install --user \
    numpy

echo "Initializing rosdep..."

sudo rosdep init 2>/dev/null || true
rosdep update

echo ""
echo "========================================="
echo "Installation Complete!"
echo "========================================="
echo ""
echo "Next steps:"
echo ""
echo "1. source /opt/ros/humble/setup.bash"
echo "2. cd <your_workspace>"
echo "3. rosdep install --from-paths src --ignore-src -r -y"
echo "4. colcon build"
echo "5. source install/setup.bash"
echo ""