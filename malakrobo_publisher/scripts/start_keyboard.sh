#!/bin/bash

# Starts keyboard operation for the robot (the version with joints not Twist)
source /home/ros/catkin_ws/devel/setup.bash
roslaunch malakrobo_publisher key_control.launch
