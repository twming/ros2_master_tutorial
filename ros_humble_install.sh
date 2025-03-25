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
printf '\033[33m%s\033[m\n' "|           TODO: HERE (Step 8)                  |"
printf '\033[33m%s\033[m\n' "=================================================="



printf '\033[33m%s\033[m\n' "=================================================="
printf '\033[33m%s\033[m\n' "|           Get ROS Humble key                   |"
printf '\033[33m%s\033[m\n' "|           TODO: HERE (Step 8)                  |"
printf '\033[33m%s\033[m\n' "=================================================="



printf '\033[33m%s\033[m\n' "=================================================="
printf '\033[33m%s\033[m\n' "|           Install ROS Humble                   |"
printf '\033[33m%s\033[m\n' "|           TODO: HERE (Step 8)                  |"
printf '\033[33m%s\033[m\n' "=================================================="



printf '\033[33m%s\033[m\n' "=================================================="
printf '\033[33m%s\033[m\n' "|           Update ROS Environment Variables     |"
printf '\033[33m%s\033[m\n' "|           TODO: HERE (Step 9)                  |"
printf '\033[33m%s\033[m\n' "=================================================="



printf '\033[33m%s\033[m\n' "=================================================="
printf '\033[33m%s\033[m\n' "|           Install OpenCR firmware dependencies |"
printf '\033[33m%s\033[m\n' "|           TODO: HERE (Step 10)                 |"
printf '\033[33m%s\033[m\n' "=================================================="



printf '\033[33m%s\033[m\n' "=================================================="
printf '\033[33m%s\033[m\n' "|           Download OpenCR firmware             |"
printf '\033[33m%s\033[m\n' "|           TODO: HERE (Step 11)                 |"
printf '\033[33m%s\033[m\n' "=================================================="



printf '\033[33m%s\033[m\n' "=================================================="
printf '\033[33m%s\033[m\n' "|           ROS Installation Complete            |"
printf '\033[33m%s\033[m\n' "=================================================="

source ~/.bashrc

