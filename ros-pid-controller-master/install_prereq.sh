#!/bin/sh
# Install the prerequisites for the ROS exploring code

sudo apt update

sudo apt -y install \
  git \
  python3-pip \
  ros-${ROS_DISTRO}-turtlebot3*