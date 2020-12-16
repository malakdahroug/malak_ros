#!/bin/bash

source /home/ros/catkin_ws/devel/setup.bash
roslaunch malakrobo_description gazebo.launch && roslaunch malakrobo_control malakrobo_control.launch

