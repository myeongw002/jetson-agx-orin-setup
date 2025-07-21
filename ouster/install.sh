#!/usr/bin/env bash
set -euo pipefail

# -----------------------------------------------------------------------------
# Script to install Ouster ROS2 driver and its dependencies
# -----------------------------------------------------------------------------

# 1. Install ROS packages for PCL, TF2 Eigen bindings, and RViz2
echo "1/4: Installing ROS PCL, TF2-Eigen, and RViz2 packages..."
sudo apt update
sudo apt install -y \
    ros-"${ROS_DISTRO}"-pcl-ros \
    ros-"${ROS_DISTRO}"-tf2-eigen \
    ros-"${ROS_DISTRO}"-rviz2

# 2. Install build tools and libraries
echo "2/4: Installing build-essential and library dependencies..."
sudo apt install -y \
    build-essential \
    libeigen3-dev \
    libjsoncpp-dev \
    libspdlog-dev \
    libcurl4-openssl-dev \
    cmake \
    python3-colcon-common-extensions

# 3. Clone the Ouster ROS2 driver into your workspace
echo "3/4: Cloning ouster-ros repository (ros2 branch)..."
WORKSPACE="${HOME}/ROS2/ros2_ws"
SRC_DIR="${WORKSPACE}/src"
mkdir -p "${SRC_DIR}"
cd "${SRC_DIR}"
git clone --branch ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git

# 4. Build the workspace with colcon
echo "4/4: Building the workspace (Release mode)..."
cd "${WORKSPACE}"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Done. To start using the Ouster driver, source your workspace in each new shell:"
echo "  source \"${WORKSPACE}/install/setup.bash\""

