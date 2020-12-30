#!/bin/bash

# Partial start that starts gazebo, the assesment_world, waits 5s and spawns the robot
# All components will be started in separate terminal windows
source /home/ros/catkin_ws/devel/setup.bash

gnome-terminal -- roslaunch assessment_world assessment_world.launch
sleep 5
gnome-terminal -- roslaunch malakrobo_description gazebo.launch
