#!/bin/bash
set -e

# Build ROS2 Humble for macOS

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Building ROS2 Humble for macOS..."

# Install dependencies
echo "Installing dependencies..."
brew install cmake python@3.11 pyyaml tinyxml2 eigen console_bridge openssl@3 spdlog assimp bullet

# Clone and build ROS2 workspace
WORKSPACE="${SCRIPT_DIR}/../ros2_workspace"
mkdir -p "${WORKSPACE}/src"

echo "Cloning ROS2 repositories..."
cd "${WORKSPACE}/src"
git clone --branch humble https://github.com/ros2/ros2 .

echo "Installing Python dependencies..."
pip3 install -U setuptools pip
pip3 install vcstool colcon-common-extensions

echo "Installing ROS dependencies..."
rosdep init || true
rosdep update

echo "Building ROS2..."
cd "${WORKSPACE}"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Build complete!"
echo "Source the setup with:"
echo "  source ${WORKSPACE}/install/setup.bash"
