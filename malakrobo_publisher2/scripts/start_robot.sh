#!/bin/bash

# Spawns the robot - if it suceeded it starts the control package (the version with joints not Twist)
source /home/ros/catkin_ws/devel/setup.bash
roslaunch malakrobo_description gazebo.launch && roslaunch malakrobo_control malakrobo_control.launch
