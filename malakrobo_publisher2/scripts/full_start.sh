#!/bin/bash

gnome-terminal -- start_gazebo.sh
sleep 5
gnome-terminal -- add_obstacles.sh
gnome-terminal -- start_robot.sh
sleep 5
gnome-terminal -- start_keyboard.sh
sleep 3
gnome-terminal -- rosrun image_view image_view image:=/malakrobo/camera1/image_raw

