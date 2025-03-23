#!/usr/bin/env bash
set -eu

CHOOSE_ROS_DISTRO=humble
INSTALL_PACKAGE=ros-base
TARGET_OS=jammy

# Check OS version
if ! which lsb_release > /dev/null ; then
	sudo apt-get update
	sudo apt-get install -y curl lsb-release
fi

if ! dpkg --print-architecture | grep -q 64; then
	printf '\033[33m%s\033[m\n' "=================================================="
	printf '\033[33m%s\033[m\n' "ERROR: This architecture ($(dpkg --print-architecture)) is not supported"
	printf '\033[33m%s\033[m\n' "See https://www.ros.org/reps/rep-2000.html"
	printf '\033[33m%s\033[m\n' "=================================================="
	exit 1
fi

printf '\033[33m%s\033[m\n' "=================================================="
printf '\033[33m%s\033[m\n' "|           Update Ubuntu 22.05                  |"
printf '\033[33m%s\033[m\n' "|           TODO: HERE                           |"
printf '\033[33m%s\033[m\n' "=================================================="



printf '\033[33m%s\033[m\n' "=================================================="
printf '\033[33m%s\033[m\n' "|           Get ROS Humble key                   |"
printf '\033[33m%s\033[m\n' "|           TODO: HERE                           |"
printf '\033[33m%s\033[m\n' "=================================================="



printf '\033[33m%s\033[m\n' "=================================================="
printf '\033[33m%s\033[m\n' "|           Install ROS Humble                   |"
printf '\033[33m%s\033[m\n' "|           TODO: HERE                           |"
printf '\033[33m%s\033[m\n' "=================================================="



printf '\033[33m%s\033[m\n' "=================================================="
printf '\033[33m%s\033[m\n' "|           Install ROS dependencie              |"
printf '\033[33m%s\033[m\n' "=================================================="

sudo apt install -y python3-pip
sudo apt install -y libserial-dev ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-robot-localization ros-humble-joy ros-humble-joy-teleop ros-humble-tf-transformations ros-humble-joint-state-broadcaster ros-humble-velocity-controllers ros-humble-diff-drive-controller
pip install smbus

printf '\033[33m%s\033[m\n' "=================================================="
printf '\033[33m%s\033[m\n' "|           Update Environment Variables         |"
printf '\033[33m%s\033[m\n' "=================================================="


grep -F "source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash" ~/.bashrc ||
echo "source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash" >> ~/.bashrc

grep -F "export ROS_DOMAIN_ID=30" ~/.bashrc ||
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
source ~/.bashrc

printf '\033[33m%s\033[m\n' "=================================================="
printf '\033[33m%s\033[m\n' "|           ROS Installation Successful          |"
printf '\033[33m%s\033[m\n' "=================================================="

