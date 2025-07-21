#!/usr/bin/env bash
set -euo pipefail

# -----------------------------------------------------------------------------
# Script to clone, build, and install the MicroStrain Inertial driver on ROS2
# -----------------------------------------------------------------------------

# Configuration
WORKSPACE="${HOME}/ROS2/ros2_ws"
SRC_DIR="${WORKSPACE}/src"
SETUP_DIR="${HOME}/setup/microstrain"
UDEV_SRC="${SRC_DIR}/microstrain_inertial/microstrain_inertial_driver/debian/udev"
UDEV_DST="/etc/udev/rules.d/100-microstrain.rules"
CONFIG_SRC="${SETUP_DIR}/gv7.yml"
CONFIG_DST="${SRC_DIR}/microstrain_inertial/microstrain_inertial_driver/config"

# 1. Prepare workspace src folder
echo "1/8: Creating workspace src directory at $SRC_DIR..."
mkdir -p "$SRC_DIR"

# 2. Clone the driver repository
echo "2/8: Cloning microstrain_inertial (ros2 branch)..."
cd "$SRC_DIR"
git clone --recursive --branch ros2 https://github.com/LORD-MicroStrain/microstrain_inertial.git

# 3. Install ROS dependencies
echo "3/8: Installing ROS dependencies via rosdep..."
cd "$WORKSPACE"
rosdep update
rosdep install --from-paths src -y --ignore-src

# 4. Build the workspace
echo "4/8: Building workspace with colcon..."
colcon build --symlink-install

# 5. Install udev rule
echo "5/8: Copying udev rule to $UDEV_DST..."
sudo cp "$UDEV_SRC" "$UDEV_DST"

# 6. Reload udev
echo "6/8: Reloading udev service..."
sudo service udev reload
sudo service udev restart

# 7. Copy camera configuration
echo "7/8: Copying gv7.yml config to driver config folder..."
cp "$CONFIG_SRC" "$CONFIG_DST"

# 8. Final instructions
echo "8/8: Complete."
echo "→ Remember to source your workspace before running any nodes:"
echo "     source ${WORKSPACE}/install/setup.bash"

exit 0

