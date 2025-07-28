#!/usr/bin/env bash
set -euo pipefail

# -----------------------------------------------------------------------------
# Script to install Spinnaker SDK, clone and build the FLIR camera driver for ROS2
# -----------------------------------------------------------------------------

# 1. Run the Spinnaker installation script
echo "1/6: Installing Spinnaker SDK..."
cd ~/setup/flir/spinnaker-3.1.0.79-arm64
bash ./install_spinnaker_arm.sh

# 2. Define workspace paths
WORKSPACE="$HOME/ROS2/ros2_ws"
SRC_DIR="$WORKSPACE/src"

# 3. Clone the FLIR camera driver into your ROS2 workspace
echo "2/6: Preparing ROS2 workspace at $WORKSPACE..."
mkdir -p "$SRC_DIR"
echo "3/6: Cloning flir_camera_driver (humble-devel branch)..."
git clone --branch humble-devel https://github.com/ros-drivers/flir_camera_driver.git "$SRC_DIR/flir_camera_driver"

# 4. Install ROS dependencies via rosdep
echo "4/6: Installing ROS dependencies with rosdep..."
cd "$WORKSPACE"
rosdep update
rosdep install --from-paths src --ignore-src -y

# 5. Build the workspace with colcon
echo "5/6: Building workspace with colcon..."
colcon build \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 6. Run the FLIR setup node
echo "6/6: Running FLIR camera setup..."
source "$WORKSPACE/install/setup.bash"
ros2 run spinnaker_camera_driver linux_setup_flir

echo "All done. If you need to source your workspace in future sessions, run:"
echo "  source $WORKSPACE/install/setup.bash"

