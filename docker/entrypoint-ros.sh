#!/bin/bash
set -e

# Setup ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash

exec "$@"
