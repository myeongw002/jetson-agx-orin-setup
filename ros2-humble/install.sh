#!/usr/bin/env bash
set -euo pipefail

echo "1/9: Updating package lists…"
sudo apt update

echo "2/9: Installing software-properties-common…"
sudo apt install -y software-properties-common

echo "3/9: Enabling the universe repository…"
sudo add-apt-repository -y universe

echo "4/9: Installing curl…"
sudo apt update
sudo apt install -y curl

echo "5/9: Fetching latest ros-apt-source version…"
export ROS_APT_SOURCE_VERSION=$(\
  curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' \
  | awk -F\" '{print $4}'\
)

echo "6/9: Downloading ros2-apt-source .deb…"
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo \$VERSION_CODENAME)_all.deb"

echo "7/9: Installing ros2-apt-source…"
sudo dpkg -i /tmp/ros2-apt-source.deb

echo "8/9: Updating & upgrading all packages…"
sudo apt update
sudo apt upgrade -y

echo "9/9: Installing ROS 2 Humble & tools…"
sudo apt install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions ros-humble-rmw-zenoh-cpp ros-humble-image-transport-plugins

sudo apt install -y python3-rosdep 
sudo rosdep init
rosdep update

echo "Adding ROS setup and colcon-argcomplete to ~/.bashrc…"
{
  echo "source /opt/ros/humble/setup.bash"
  echo "source ~/ROS2/ros2_ws/install/setup.bash"
  echo "/usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
  echo "alias sb='source ~/.bashrc'"
  echo "alias cb='cd ~/ros2_ws && colcon build --symlink-install'"
  echo "# export RMW_IMPLEMENTATION=rmw_zenoh_cpp" 
  echo "export ROS_DOMAIN_ID=7 # 0~101"
} >> ~/.bashrc

mkdir -p ~/ROS2/ros2_ws/src

source ~/.bashrc

echo "Done!  Please run 'source ~/.bashrc' or open a new terminal to start using ROS 2 Humble."

