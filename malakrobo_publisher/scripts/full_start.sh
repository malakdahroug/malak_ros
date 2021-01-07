#!/bin/bash

# Full start Script
# 1. Starts gazebo and assessment_world, waits 5s so it's fully loaded
# 2. Adds obstacles
# 3. Spawns the robot and starts controls (the version with joints not Twist), waits 5s so it's fully loaded
# 4. Starts keyboard operation (the version with joints not Twist) and waits 3s
# 5. Starts the camera viewer on the robot
# All elements will be started in separate terminal windows

gnome-terminal -- start_gazebo.sh
sleep 5
gnome-terminal -- add_obstacles.sh
gnome-terminal -- start_robot.sh
sleep 5
gnome-terminal -- start_keyboard.sh
sleep 3
gnome-terminal -- rosrun image_view image_view image:=/malakrobo/camera1/image_raw
