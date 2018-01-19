#!/bin/bash
cd ~
mkdir -p ~/robots_ws/src

# Clone repos
cd ~/robotx_ws/src
REPOS=(
    robotx_gazebo \
	wamv_description \
	usv_utils \
	usv_msgs \
	usv_gazebo_plugins \
	buoyancy_gazebo_plugin \
)
for REPO in "${REPOS[@]}"
do
    git clone git@github.com:bsb808/${REPO}.git
done

# Change to branch
cd ~/robotx_ws/src/usv_gazebo_plugins
git fetch origin
git checkout usvmsg

# Make
cd ~/robotx_ws
catkin_make
source ./devel/setup.bash

# Go
roslaunch robotx_gazebo sandisland.launch 
