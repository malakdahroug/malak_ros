#!/bin/bash

# Partial start
# 1. Starts gazebo, the assessment world, waits 5s so it is fully loaded
# 2. Spawns the robot
# 3. Spawns obstacles
source /home/ros/catkin_ws/devel/setup.bash

gnome-terminal -- roslaunch assessment_world assessment_world.launch
sleep 5
gnome-terminal -- roslaunch malakrobo_description gazebo.launch
gnome-terminal -- roslaunch assessment_world objects.launch
