#!/bin/bash

source /home/ros/catkin_ws/devel/setup.bash

gnome-terminal -- roslaunch assessment_world assessment_world.launch
sleep 5
gnome-terminal -- roslaunch malakrobo_description gazebo.launch

