#!/usr/bin/env python
import threading

import cv2
import rospy
import numpy as np
from std_msgs.msg import String, Float32MultiArray, Float64, Header, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseAction
from std_srvs.srv import Empty
from PyQt5 import uic
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import icons
import random
import sys
import time
import actionlib
from pynput import keyboard  # Contains keyboard operations
import roslib
import math
import tf2_ros
import PyKDL
from math import pi

# ROBOT MOVEMENT CONTROL PARAMS #
move_cmd = Twist()
move_cmd.linear.x = 0.0
move_cmd.linear.y = 0.0
move_cmd.linear.z = 0.0
move_cmd.angular.x = 0.0
move_cmd.angular.y = 0.0
move_cmd.angular.z = 0.0

movement = 0
stopped = False
tab_index = 0
stop_threads = False
amcl_text = "Loading..."
goal_text = "Loading..."
app = QApplication([])
window = uic.loadUi(rospy.get_param("/gui_file"))

# GUI STATES #
current_target = -1
gripper_state = 0
ball_detected = -1
current_action = -1
arrow_pressed = False

# WORKING VARS #
pose_count = 0
current_amcl = PoseWithCovarianceStamped()
movement_thread_count = 0

# AUTONOMOUS OBJECT RECOGNITION #
camera_center = 400
max_ang_vel = 0.1
min_ang_vel = -0.1
ang_vel = 0
detection = False
sml_ball_grabbed = False
med_ball_grabbed = False
lrg_ball_grabbed = False
no_detection_count = 0
detection_recovery_count = 0
detect_cycle_count = 0

# THE AUTONOMOUS PATH #
wait_for_ball = True
ball_position_1 = None
ball_position_2 = None
ball_position_3 = None
ball_position_4 = None
auto_pose_1 = [0, 0, 0, 0]
auto_pose_2 = [0, 0, 0, 0]
auto_pose_3 = [0, 0, 0, 0]


# Callback function, called when the key is released - it sets twist msg properties to 0.0, so the robot stops moving
def on_release(key):
    global move_cmd, tab_index
    if not tab_index == 0:
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
    else:
        # If key up/down is release it is supposed to reset linear x speed
        if key == keyboard.Key.up or key == keyboard.Key.down:
            move_cmd.linear.x = 0.0  # Reset linear x movement
        # If key left/right is release it is supposed to reset angular z speed
        if key == keyboard.Key.left or key == keyboard.Key.right:
            move_cmd.angular.z = 0.0  # Reset angular z movement


# Callback function, called when key is pressed on the keyboard
def on_press(key):
    global move_cmd, tab_index  # Defining globals as the values are meant to be use outside this function
    if not tab_index == 0:
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
    else:
        if key == keyboard.Key.up:  # Arrow up - twist message set for the robot to go forward
            move_cmd.linear.x = 0.5
        elif key == keyboard.Key.down:  # Arrow down - twist message set for the robot to go back
            move_cmd.linear.x = -0.5
        elif key == keyboard.Key.left:  # Arrow left - twist message set for the robot to turn right
            move_cmd.angular.z = 0.5
        elif key == keyboard.Key.right:  # Arrow right - twist message set for the robot to turn left
            move_cmd.angular.z = -0.5


def forward():
    global move_cmd, tab_index, arrow_pressed
    if tab_index == 0:
        arrow_pressed = True
        move_cmd.linear.x = 0.5


def forward_left():
    global move_cmd, tab_index, arrow_pressed
    if tab_index == 0:
        arrow_pressed = True
        move_cmd.linear.x = 0.5
        move_cmd.angular.z = 0.5


def forward_right():
    global move_cmd, tab_index, arrow_pressed
    if tab_index == 0:
        arrow_pressed = True
        move_cmd.linear.x = 0.5
        move_cmd.angular.z = -0.5


def backward():
    global move_cmd, tab_index, arrow_pressed
    if tab_index == 0:
        arrow_pressed = True
        move_cmd.linear.x = -0.5


def backward_left():
    global move_cmd, tab_index, arrow_pressed
    if tab_index == 0:
        arrow_pressed = True
        move_cmd.linear.x = -0.5
        move_cmd.angular.z = -0.5


def backward_right():
    global move_cmd, tab_index, arrow_pressed
    if tab_index == 0:
        arrow_pressed = True
        move_cmd.linear.x = -0.5
        move_cmd.angular.z = 0.5


def reset_fb():
    global move_cmd, arrow_pressed
    arrow_pressed = False
    move_cmd.linear.x = 0.0


def left():
    global move_cmd, tab_index, arrow_pressed
    if tab_index == 0:
        arrow_pressed = True
        move_cmd.angular.z = 0.5


def right():
    global move_cmd, tab_index, arrow_pressed
    if tab_index == 0:
        arrow_pressed = True
        move_cmd.angular.z = -0.5


def reset_lr():
    global move_cmd, arrow_pressed
    arrow_pressed = False
    move_cmd.angular.z = 0.0


def reset_both():
    reset_lr()
    reset_fb()


def exit_handler():
    global stop_threads
    stop_threads = True
    rospy.loginfo("Closing...")
    sys.exit(0)


# ACTION BUTTONS FUNCTIONS #
def go_to_goal():
    global pose_count, current_target, current_action, movement_thread_count
    if movement_thread_count <= 0:
        current_target = 0
        current_action = 0
        pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        time.sleep(0.5)
        goal = PoseStamped()
        goal.header.seq = pose_count + 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        pub.publish(goal)


def take_ball_home():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    pose_list = [
        [1.5, 0.0, 1.0, 0.0],
        [0.2, 0.0, 1.0, 0.0]
    ]

    for x in pose_list:
        global pose_count
        goal = MoveBaseGoal()
        goal.target_pose.header.seq = pose_count + 1
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x[0]
        goal.target_pose.pose.position.y = x[1]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = x[2]
        goal.target_pose.pose.orientation.w = x[3]

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")

    stop_suction()


def move_to_pose_single(client, x, y, z, w):
    global pose_count

    goal = MoveBaseGoal()
    goal.target_pose.header.seq = pose_count + 1
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
        return False

    if wait:
        return True


def get_ball_position(obj):
    global ball_position_1, ball_position_2, ball_position_3, ball_position_4, stop_threads
    tfbuffer = tf2_ros.Buffer()

    listener = tf2_ros.TransformListener(tfbuffer)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        if stop_threads:
            break

        try:
            if obj == 1:
                ball_position_1 = tfbuffer.lookup_transform('map', "object_107", rospy.Time())
            elif obj == 2:
                ball_position_2 = tfbuffer.lookup_transform('map', "object_108", rospy.Time())
            elif obj == 3:
                ball_position_3 = tfbuffer.lookup_transform('map', "object_109", rospy.Time())
            elif obj == 4:
                ball_position_4 = tfbuffer.lookup_transform('map', "object_110", rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            continue

        rate.sleep()


    # if trg == 1:
    #     upper_limit = 1.7
    #     lower_limit = 0
    # elif trg == 2:
    #     upper_limit = 1.7
    #     lower_limit = 2.8
    # elif trg == 3:
    #     upper_limit = 2.8
    #     lower_limit = 3.6
def calculate_pose_1():
    global ball_position_1, ball_position_2, ball_position_3, ball_position_4, auto_pose_1, current_amcl, stop_threads
    target = 0
    upper_limit = 1.7
    lower_limit = 0
    while True:
        if stop_threads:
            break

        if ball_position_1 is not None:
            if lower_limit < ball_position_1.transform.translation.x < upper_limit:
                target = ball_position_1.transform.translation

        if ball_position_2 is not None:
            if lower_limit < ball_position_2.transform.translation.x < upper_limit:
                target = ball_position_2.transform.translation

        if ball_position_3 is not None:
            if lower_limit < ball_position_3.transform.translation.x < upper_limit:
                target = ball_position_3.transform.translation

        if ball_position_4 is not None:
            if lower_limit < ball_position_4.transform.translation.x < upper_limit:
                target = ball_position_4.transform.translation

        if not target == 0:
            auto_pose_1 = [target.x, target.y, current_amcl.orientation.z, current_amcl.orientation.w]


def calculate_pose_2():
    global ball_position_1, ball_position_2, ball_position_3, ball_position_4, auto_pose_2, current_amcl, stop_threads
    target = 0
    upper_limit = 2.8
    lower_limit = 1.7
    while True:
        if stop_threads:
            break

        if ball_position_1 is not None:
            if lower_limit < ball_position_1.transform.translation.x < upper_limit:
                target = ball_position_1.transform.translation

        if ball_position_2 is not None:
            if lower_limit < ball_position_2.transform.translation.x < upper_limit:
                target = ball_position_2.transform.translation

        if ball_position_3 is not None:
            if lower_limit < ball_position_3.transform.translation.x < upper_limit:
                target = ball_position_3.transform.translation

        if ball_position_4 is not None:
            if lower_limit < ball_position_4.transform.translation.x < upper_limit:
                target = ball_position_4.transform.translation

        if not target == 0:
            auto_pose_2 = [target.x, target.y, current_amcl.orientation.z, current_amcl.orientation.w]


def calculate_pose_3():
    global ball_position_1, ball_position_2, ball_position_3, ball_position_4, auto_pose_3, current_amcl, stop_threads
    target = 0
    upper_limit = 3.9
    lower_limit = 2.8
    while True:
        if stop_threads:
            break

        if ball_position_1 is not None:
            if lower_limit < ball_position_1.transform.translation.x < upper_limit:
                target = ball_position_1.transform.translation

        if ball_position_2 is not None:
            if lower_limit < ball_position_2.transform.translation.x < upper_limit:
                target = ball_position_2.transform.translation

        if ball_position_3 is not None:
            if lower_limit < ball_position_3.transform.translation.x < upper_limit:
                target = ball_position_3.transform.translation

        if ball_position_4 is not None:
            if lower_limit < ball_position_4.transform.translation.x < upper_limit:
                target = ball_position_4.transform.translation

        if not target == 0:
            auto_pose_3 = [target.x, target.y, current_amcl.orientation.z, current_amcl.orientation.w]


def row(pose_index):
    global pose_count, current_amcl, move_cmd, stop_threads, movement_thread_count, current_target, current_action, detection, wait_for_ball, auto_pose_1, auto_pose_2, auto_pose_3

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    pose_list = []

    if current_amcl.orientation.z > 0.95 and current_amcl.orientation.w < 0.15 or current_amcl.orientation.z < -0.95 and current_amcl.orientation.w < 0.15 and -0.125 < current_amcl.position.x < 0.125 and -0.125 < current_amcl.position.y < 0.125:
        pub = rospy.Publisher('/malakrobo/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        rate = rospy.Rate(25)  # Publishing rate for messages - 25 msgs a second
        while current_amcl.position.x < 1.2:
            move_cmd.linear.x = -0.25
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)
            rate.sleep()

        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)

    if pose_index < 10:
        if pose_index == -1:
            pose_list = [
                [1.45, 0.0, 1.0, 0.0],
                [0.2, 0.0, 1.0, 0.0]
            ]
        elif pose_index == 0:
            pose_list = [
                [1.55, 1.4, -0.707, 0.707]
            ]
        elif pose_index == 1:
            pose_list = [
                [2.65, 1.4, -0.707, 0.707]
            ]
        elif pose_index == 2:
            pose_list = [
                [2.65, 1.50, -0.270813487105, 0.962631837829],
                [3.1, 1.35, -0.270813487105, 0.962631837829],
                [3.8, 0.8, -0.707, 0.707]
            ]

        for x in pose_list:
            if stop_threads:
                break

            goal = MoveBaseGoal()
            goal.target_pose.header.seq = pose_count + 1
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.header.frame_id = "map"

            goal.target_pose.pose.position.x = x[0]
            goal.target_pose.pose.position.y = x[1]
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = x[2]
            goal.target_pose.pose.orientation.w = x[3]

            client.send_goal(goal)
            wait = client.wait_for_result()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
                break

        if pose_index == -1:
            stop_suction()

    else:
        wait_for_ball = True
        completed = False

        goal = MoveBaseGoal()
        goal.target_pose.header.seq = pose_count + 1
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = 0.0
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 0.0

        if pose_index == 30:
            target = 0
            detected = False
            if move_to_pose_single(client, 1.05, 0.0, 0.0, 1.0):
                detection = True

            while wait_for_ball:
                if current_amcl.position.x > 1.68:
                    detection = False
                    break

                if lrg_ball_grabbed:
                    detection = False
                    take_ball_home()
                    completed = True
                    break

                if not auto_pose_1 == [0, 0, 0, 0]:
                    print('in')
                    move_to_pose_single(client, auto_pose_1[0], auto_pose_1[1], auto_pose_1[2], auto_pose_1[3])
                    detected = True
                    break
                time.sleep(0.2)

            while detected:
                if lrg_ball_grabbed:
                    detection = False
                    take_ball_home()
                    completed = True
                    break
                time.sleep(0.2)

            if not completed:
                wait_for_ball = True
                detection = False
                detected = False
                if move_to_pose_single(client, 1.55, 0.0, 0.707, 0.707):
                    detection = True

                while wait_for_ball:

                    if lrg_ball_grabbed:
                        detection = False
                        take_ball_home()
                        completed = True
                        break

                    if not auto_pose_1 == [0, 0, 0, 0]:
                        print('in')
                        move_to_pose_single(client, auto_pose_1[0], auto_pose_1[1], auto_pose_1[2], auto_pose_1[3])
                        detected = True
                        break
                    time.sleep(0.2)

                while detected:
                    if lrg_ball_grabbed:
                        detection = False
                        take_ball_home()
                        completed = True
                        break
                    time.sleep(0.2)

            else:
                current_target = -1
                current_action = -1
                movement_thread_count -= 1
                wait_for_ball = False
                rospy.loginfo("Closing position control thread")
                return

            if not completed:
                wait_for_ball = True
                detection = False
                detected = False
                if move_to_pose_single(client, 1.55, 0.6, 0.707, 0.707):
                    detection = True

                while wait_for_ball:

                    if lrg_ball_grabbed:
                        detection = False
                        take_ball_home()
                        completed = True
                        break

                    if not auto_pose_1 == [0, 0, 0, 0]:
                        print('in')
                        move_to_pose_single(client, auto_pose_1[0], auto_pose_1[1], auto_pose_1[2], auto_pose_1[3])
                        detected = True
                        break
                    time.sleep(0.2)

                while detected:
                    if lrg_ball_grabbed:
                        detection = False
                        take_ball_home()
                        completed = True
                        break
                    time.sleep(0.2)
            else:
                current_target = -1
                current_action = -1
                movement_thread_count -= 1
                wait_for_ball = False
                rospy.loginfo("Closing position control thread")
                return

            if not completed:
                detection = False
                wait_for_ball = True
                if move_to_pose_single(client, 1.55, 1.4, -0.707, 0.707):
                    if move_to_pose_single(client, 1.55, 0.6, -0.707, 0.707):
                        detection = True

                detected = False
                while wait_for_ball:
                    if lrg_ball_grabbed:
                        detection = False
                        if current_amcl.position.y < -0.6 and 1.1 < current_amcl.position.x < 1.7:
                            if move_to_pose_single(client, 1.55, -2.25, 0.707, 0.707):
                                take_ball_home()
                                completed = True
                                break

                    if not auto_pose_1 == [0, 0, 0, 0]:
                        print('in')
                        move_to_pose_single(client, auto_pose_1[0], auto_pose_1[1], auto_pose_1[2], auto_pose_1[3])
                        detected = True
                        break
                    time.sleep(0.2)

                while detected:
                    if lrg_ball_grabbed:
                        detection = False
                        if current_amcl.position.y < -0.6 and 1.1 < current_amcl.position.x < 1.7:
                            if move_to_pose_single(client, 1.55, -2.25, 0.707, 0.707):
                                take_ball_home()
                                completed = True
                                break
                    time.sleep(0.2)
            else:
                current_target = -1
                current_action = -1
                movement_thread_count -= 1
                wait_for_ball = False
                rospy.loginfo("Closing position control thread")
                return

            if not completed:
                wait_for_ball = True
                detection = False
                detected = False
                if move_to_pose_single(client, 1.55, -0.6, -0.707, 0.707):
                    detection = True

                while wait_for_ball:

                    if lrg_ball_grabbed:
                        detection = False
                        if current_amcl.position.y < -0.6 and 1.1 < current_amcl.position.x < 1.7:
                            if move_to_pose_single(client, 1.55, -2.25, 0.707, 0.707):
                                take_ball_home()
                                completed = True
                                break

                    if not auto_pose_1 == [0, 0, 0, 0]:
                        print('in')
                        move_to_pose_single(client, auto_pose_1[0], auto_pose_1[1], auto_pose_1[2], auto_pose_1[3])
                        detected = True
                        break
                    time.sleep(0.2)

                while detected:
                    if lrg_ball_grabbed:
                        detection = False
                        if current_amcl.position.y < -0.6 and 1.1 < current_amcl.position.x < 1.7:
                            if move_to_pose_single(client, 1.55, -2.25, 0.707, 0.707):
                                take_ball_home()
                                completed = True
                                break
                    time.sleep(0.2)
            else:
                current_target = -1
                current_action = -1
                movement_thread_count -= 1
                wait_for_ball = False
                rospy.loginfo("Closing position control thread")
                return

            if not completed:
                wait_count = 0
                while not lrg_ball_grabbed:
                    if not auto_pose_1 == [0, 0, 0, 0]:
                        if move_to_pose_single(client, auto_pose_1[0], auto_pose_1[1], auto_pose_1[2], auto_pose_1[3]):
                            if lrg_ball_grabbed:
                                detection = False
                                if current_amcl.position.y < -0.6 and 1.1 < current_amcl.position.x < 1.7:
                                    if move_to_pose_single(client, 1.55, -2.25, 0.707, 0.707):
                                        take_ball_home()
                                        completed = True
                                        break

                    wait_count += 1
                    if wait_count >= 90:
                        rospy.logerr("90 second elapsed and the ball was not found! Going back to goal!")
                        stop_suction()
                        go_to_goal()
                    time.sleep(1)

            if lrg_ball_grabbed:
                detection = False
                take_ball_home()
                completed = True

            current_target = -1
            current_action = -1
            movement_thread_count -= 1
            wait_for_ball = False
            rospy.loginfo("Closing position control thread")
            return

        if pose_index == 20:
            target = 0
            detected = False
            if move_to_pose_single(client, 2.15, 0.0, 0.0, 1.0):
                detection = True

            while wait_for_ball:
                if current_amcl.position.x > 2.78:
                    detection = False
                    break

                if lrg_ball_grabbed or med_ball_grabbed:
                    detection = False
                    take_ball_home()
                    completed = True
                    break

                if not auto_pose_2 == [0, 0, 0, 0]:
                    print('in')
                    move_to_pose_single(client, auto_pose_2[0], auto_pose_2[1], auto_pose_2[2], auto_pose_2[3])
                    detected = True
                    break
                time.sleep(0.2)

            while detected:
                if lrg_ball_grabbed or med_ball_grabbed:
                    detection = False
                    take_ball_home()
                    completed = True
                    break
                time.sleep(0.2)

            if not completed:
                wait_for_ball = True
                detection = False
                detected = False
                if move_to_pose_single(client, 2.65, 0.0, 0.707, 0.707):
                    detection = True

                while wait_for_ball:

                    if lrg_ball_grabbed or med_ball_grabbed:
                        detection = False
                        take_ball_home()
                        completed = True
                        break

                    if not auto_pose_2 == [0, 0, 0, 0]:
                        print('in')
                        move_to_pose_single(client, auto_pose_2[0], auto_pose_2[1], auto_pose_2[2], auto_pose_2[3])
                        detected = True
                        break
                    time.sleep(0.2)

                while detected:
                    if lrg_ball_grabbed or med_ball_grabbed:
                        detection = False
                        take_ball_home()
                        completed = True
                        break
                    time.sleep(0.2)

            else:
                current_target = -1
                current_action = -1
                movement_thread_count -= 1
                wait_for_ball = False
                rospy.loginfo("Closing position control thread")
                return

            if not completed:
                wait_for_ball = True
                detection = False
                detected = False
                if move_to_pose_single(client, 2.65, 0.6, 0.707, 0.707):
                    detection = True

                while wait_for_ball:

                    if lrg_ball_grabbed or med_ball_grabbed:
                        detection = False
                        take_ball_home()
                        completed = True
                        break

                    if not auto_pose_2 == [0, 0, 0, 0]:
                        print('in')
                        move_to_pose_single(client, auto_pose_2[0], auto_pose_2[1], auto_pose_2[2], auto_pose_2[3])
                        detected = True
                        break
                    time.sleep(0.2)

                while detected:
                    if lrg_ball_grabbed or med_ball_grabbed:
                        detection = False
                        take_ball_home()
                        completed = True
                        break
                    time.sleep(0.2)
            else:
                current_target = -1
                current_action = -1
                movement_thread_count -= 1
                wait_for_ball = False
                rospy.loginfo("Closing position control thread")
                return

            if not completed:
                detection = False
                wait_for_ball = True
                if move_to_pose_single(client, 2.65, 1.4, -0.707, 0.707):
                    if move_to_pose_single(client, 2.65, 0.6, -0.707, 0.707):
                        detection = True

                detected = False
                while wait_for_ball:
                    if lrg_ball_grabbed or med_ball_grabbed:
                        detection = False
                        if current_amcl.position.y < -0.6 and 1.7 < current_amcl.position.x < 2.65:
                            if move_to_pose_single(client, 2.65, -2.25, 0.707, 0.707):
                                take_ball_home()
                                completed = True
                                break

                    if not auto_pose_2 == [0, 0, 0, 0]:
                        print('in')
                        move_to_pose_single(client, auto_pose_2[0], auto_pose_2[1], auto_pose_2[2], auto_pose_2[3])
                        detected = True
                        break
                    time.sleep(0.2)

                while detected:
                    if lrg_ball_grabbed or med_ball_grabbed:
                        detection = False
                        if current_amcl.position.y < -0.6 and 1.7 < current_amcl.position.x < 2.65:
                            if move_to_pose_single(client, 2.65, -2.25, 0.707, 0.707):
                                take_ball_home()
                                completed = True
                                break
                    time.sleep(0.2)
            else:
                current_target = -1
                current_action = -1
                movement_thread_count -= 1
                wait_for_ball = False
                rospy.loginfo("Closing position control thread")
                return

            if not completed:
                wait_for_ball = True
                detection = False
                detected = False
                if move_to_pose_single(client, 2.65, -0.6, -0.707, 0.707):
                    detection = True

                while wait_for_ball:

                    if lrg_ball_grabbed or med_ball_grabbed:
                        detection = False
                        if current_amcl.position.y < -0.6 and 1.7 < current_amcl.position.x < 2.65:
                            if move_to_pose_single(client, 2.65, -2.25, 0.707, 0.707):
                                take_ball_home()
                                completed = True
                                break

                    if not auto_pose_2 == [0, 0, 0, 0]:
                        print('in')
                        move_to_pose_single(client, auto_pose_2[0], auto_pose_2[1], auto_pose_2[2], auto_pose_2[3])
                        detected = True
                        break
                    time.sleep(0.2)

                while detected:
                    if lrg_ball_grabbed or med_ball_grabbed:
                        detection = False
                        if current_amcl.position.y < -0.6 and 1.7 < current_amcl.position.x < 2.65:
                            if move_to_pose_single(client, 2.65, -2.25, 0.707, 0.707):
                                take_ball_home()
                                completed = True
                                break
                    time.sleep(0.2)
            else:
                current_target = -1
                current_action = -1
                movement_thread_count -= 1
                wait_for_ball = False
                rospy.loginfo("Closing position control thread")
                return

            if not completed:
                wait_count = 0
                while not lrg_ball_grabbed or med_ball_grabbed:
                    if not auto_pose_2 == [0, 0, 0, 0]:
                        if move_to_pose_single(client, auto_pose_2[0], auto_pose_2[1], auto_pose_2[2], auto_pose_2[3]):
                            if lrg_ball_grabbed or med_ball_grabbed:
                                detection = False
                                if current_amcl.position.y < -0.6 and 1.7 < current_amcl.position.x < 2.65:
                                    if move_to_pose_single(client, 2.65, -2.25, 0.707, 0.707):
                                        take_ball_home()
                                        completed = True
                                        break

                    wait_count += 1
                    if wait_count >= 90:
                        rospy.logerr("90 second elapsed and the ball was not found! Going back to goal!")
                        stop_suction()
                        go_to_goal()
                    time.sleep(1)

            if lrg_ball_grabbed or med_ball_grabbed:
                detection = False
                take_ball_home()
                completed = True
            else:
                current_target = -1
                current_action = -1
                movement_thread_count -= 1
                wait_for_ball = False
                rospy.loginfo("Closing position control thread")
                return

        if pose_index == 10:
            target = 0
            detected = False
            if move_to_pose_single(client, 3.25, 0.0, 0.0, 1.0):
                detection = True

            while wait_for_ball:
                if current_amcl.position.x > 4.0:
                    detection = False
                    break

                if sml_ball_grabbed or med_ball_grabbed:
                    detection = False
                    take_ball_home()
                    completed = True
                    break

                if not auto_pose_3 == [0, 0, 0, 0]:
                    print('in')
                    move_to_pose_single(client, auto_pose_3[0], auto_pose_3[1], auto_pose_3[2], auto_pose_3[3])
                    detected = True
                    break
                time.sleep(0.2)

            while detected:
                if sml_ball_grabbed or med_ball_grabbed:
                    detection = False
                    take_ball_home()
                    completed = True
                    break
                time.sleep(0.2)

            if not completed:
                wait_for_ball = True
                detection = False
                detected = False
                if move_to_pose_single(client, 3.8, 0.0, 0.707, 0.707):
                    detection = True

                while wait_for_ball:

                    if sml_ball_grabbed or med_ball_grabbed:
                        detection = False
                        take_ball_home()
                        completed = True
                        break

                    if not auto_pose_3 == [0, 0, 0, 0]:
                        print('in')
                        move_to_pose_single(client, auto_pose_3[0], auto_pose_3[1], auto_pose_3[2], auto_pose_3[3])
                        detected = True
                        break
                    time.sleep(0.2)

                while detected:
                    if sml_ball_grabbed or med_ball_grabbed:
                        detection = False
                        take_ball_home()
                        completed = True
                        break
                    time.sleep(0.2)

            else:
                current_target = -1
                current_action = -1
                movement_thread_count -= 1
                wait_for_ball = False
                rospy.loginfo("Closing position control thread")
                return

            if not completed:
                wait_for_ball = True
                detection = False
                detected = False
                if move_to_pose_single(client, 3.8, 0.0, -0.707, 0.707):
                    detection = True

                while wait_for_ball:

                    if sml_ball_grabbed or med_ball_grabbed:
                        detection = False
                        take_ball_home()
                        completed = True
                        break

                    if not auto_pose_3 == [0, 0, 0, 0]:
                        print('in')
                        move_to_pose_single(client, auto_pose_3[0], auto_pose_3[1], auto_pose_3[2], auto_pose_3[3])
                        detected = True
                        break
                    time.sleep(0.2)

                while detected:
                    if sml_ball_grabbed or med_ball_grabbed:
                        detection = False
                        take_ball_home()
                        completed = True
                        break
                    time.sleep(0.2)
            else:
                current_target = -1
                current_action = -1
                movement_thread_count -= 1
                wait_for_ball = False
                rospy.loginfo("Closing position control thread")
                return

            if not completed:
                wait_for_ball = True
                detection = False
                detected = False
                if move_to_pose_single(client, 3.8, -0.6, -0.707, 0.707):
                    detection = True

                while wait_for_ball:

                    if sml_ball_grabbed or med_ball_grabbed:
                        detection = False
                        take_ball_home()
                        completed = True
                        break

                    if not auto_pose_3 == [0, 0, 0, 0]:
                        print('in')
                        move_to_pose_single(client, auto_pose_3[0], auto_pose_3[1], auto_pose_3[2], auto_pose_3[3])
                        detected = True
                        break
                    time.sleep(0.2)

                while detected:
                    if sml_ball_grabbed or med_ball_grabbed:
                        detection = False
                        take_ball_home()
                        completed = True
                        break
                    time.sleep(0.2)
            else:
                current_target = -1
                current_action = -1
                movement_thread_count -= 1
                wait_for_ball = False
                rospy.loginfo("Closing position control thread")
                return

            if not completed:
                wait_for_ball = True
                detection = False
                detected = False
                if move_to_pose_single(client, 3.8, -1.0, -0.707, 0.707):
                    detection = True

                while wait_for_ball:

                    if sml_ball_grabbed or med_ball_grabbed:
                        detection = False
                        take_ball_home()
                        completed = True
                        break

                    if not auto_pose_3 == [0, 0, 0, 0]:
                        print('in')
                        move_to_pose_single(client, auto_pose_3[0], auto_pose_3[1], auto_pose_3[2], auto_pose_3[3])
                        detected = True
                        break
                    time.sleep(0.2)

                while detected:
                    if sml_ball_grabbed or med_ball_grabbed:
                        detection = False
                        take_ball_home()
                        completed = True
                        break
                    time.sleep(0.2)
            else:
                current_target = -1
                current_action = -1
                movement_thread_count -= 1
                wait_for_ball = False
                rospy.loginfo("Closing position control thread")
                return


            if not completed:
                wait_count = 0
                while not sml_ball_grabbed or med_ball_grabbed:
                    if not auto_pose_3 == [0, 0, 0, 0]:
                        if move_to_pose_single(client, auto_pose_3[0], auto_pose_3[1], auto_pose_3[2], auto_pose_3[3]):
                            if sml_ball_grabbed or med_ball_grabbed:
                                detection = False
                                take_ball_home()
                                completed = True
                                break

                    wait_count += 1
                    if wait_count >= 90:
                        rospy.logerr("90 second elapsed and the ball was not found! Going back to goal!")
                        stop_suction()
                        go_to_goal()
                    time.sleep(1)

            if sml_ball_grabbed or med_ball_grabbed:
                detection = False
                take_ball_home()
                completed = True
            else:
                current_target = -1
                current_action = -1
                movement_thread_count -= 1
                wait_for_ball = False
                rospy.loginfo("Closing position control thread")
                return
        # if pose_index == 20:
        #     completed = False
        #     if move_to_pose_single(client, 2.15, 0.0, 0.0, 1.0):
        #         detection = True
        #
        #     while wait_for_ball:
        #         if current_amcl.position.x > 2.78:
        #             detection = False
        #             break
        #
        #         if med_ball_grabbed or lrg_ball_grabbed:
        #             detection = False
        #             take_ball_home()
        #             completed = True
        #         time.sleep(0.2)
        #
        #     if not completed:
        #         wait_for_ball = False
        #         detection = False
        #         if move_to_pose_single(client, 2.65, 0.0, 0.707, 0.707):
        #             detection = True
        #
        #         while wait_for_ball:
        #             if med_ball_grabbed or lrg_ball_grabbed:
        #                 detection = False
        #                 take_ball_home()
        #                 completed = True
        #                 break
        #
        #             time.sleep(0.2)
        #     else:
        #         current_target = -1
        #         current_action = -1
        #         movement_thread_count -= 1
        #         wait_for_ball = False
        #         rospy.loginfo("Closing position control thread")
        #         return
        #
        #     if not completed:
        #         wait_for_ball = False
        #         detection = False
        #         if move_to_pose_single(client, 2.65, 0.6, 0.707, 0.707):
        #             detection = True
        #
        #         while wait_for_ball:
        #             if med_ball_grabbed or lrg_ball_grabbed:
        #                 detection = False
        #                 take_ball_home()
        #                 completed = True
        #                 break
        #
        #             time.sleep(0.2)
        #     else:
        #         current_target = -1
        #         current_action = -1
        #         movement_thread_count -= 1
        #         wait_for_ball = False
        #         rospy.loginfo("Closing position control thread")
        #         return
        #
        #     if not completed:
        #         wait_for_ball = False
        #         detection = False
        #         pose_list = [
        #             [2.65, 1.4, -0.707, 0.707],
        #             [2.65, 0.6, -0.707, 0.707]
        #         ]
        #
        #         for x in pose_list:
        #             goal.target_pose.header.seq = pose_count + 1
        #             goal.target_pose.header.stamp = rospy.Time.now()
        #             goal.target_pose.pose.position.x = x[0]
        #             goal.target_pose.pose.position.y = x[1]
        #             goal.target_pose.pose.orientation.z = x[2]
        #             goal.target_pose.pose.orientation.w = x[3]
        #
        #             client.send_goal(goal)
        #             wait = client.wait_for_result()
        #             if not wait:
        #                 rospy.logerr("Action server not available!")
        #                 rospy.signal_shutdown("Action server not available!")
        #                 break
        #
        #         detection = True
        #         while wait_for_ball:
        #             if med_ball_grabbed or lrg_ball_grabbed:
        #                 detection = False
        #                 take_ball_home()
        #                 completed = True
        #                 break
        #     else:
        #         current_target = -1
        #         current_action = -1
        #         movement_thread_count -= 1
        #         wait_for_ball = False
        #         rospy.loginfo("Closing position control thread")
        #         return
        #
        #     if not completed:
        #         wait_for_ball = False
        #         detection = False
        #         if move_to_pose_single(client, 2.65, -0.6, -0.707, 0.707):
        #             detection = True
        #
        #         while wait_for_ball:
        #             if med_ball_grabbed or lrg_ball_grabbed:
        #                 detection = False
        #                 take_ball_home()
        #                 completed = True
        #                 break
        #
        #             time.sleep(0.2)
        #     else:
        #         current_target = -1
        #         current_action = -1
        #         movement_thread_count -= 1
        #         wait_for_ball = False
        #         rospy.loginfo("Closing position control thread")
        #         return
        #
        #     if not completed:
        #         wait_count = 0
        #         while not med_ball_grabbed or lrg_ball_grabbed:
        #             wait_count += 1
        #             if wait_count == 90:
        #                 rospy.logerr("90 second elapsed and the ball was not found! Going back to goal!")
        #                 stop_suction()
        #                 go_to_goal()
        #             time.sleep(1)
        #
        #         if med_ball_grabbed:
        #             detection = False
        #             if current_amcl.position.y < -0.6 and 2.2 < current_amcl.position.x < 2.8:
        #                 if move_to_pose_single(client, 2.65, -2.25, 0.707, 0.707):
        #                     take_ball_home()
        #                     completed = True
        #     else:
        #         current_target = -1
        #         current_action = -1
        #         movement_thread_count -= 1
        #         wait_for_ball = False
        #         rospy.loginfo("Closing position control thread")
        #         return

    current_target = -1
    current_action = -1
    movement_thread_count -= 1
    wait_for_ball = False
    rospy.loginfo("Closing position control thread")


def go_to_row_1():
    global movement_thread_count, current_target, current_action
    pub_thread = threading.Thread(target=row, args=(0,))
    if movement_thread_count <= 0:
        current_target = 1
        current_action = 1
        pub_thread.start()
        movement_thread_count += 1


def go_to_row_2():
    global movement_thread_count, current_target, current_action
    pub_thread = threading.Thread(target=row, args=(1,))
    if movement_thread_count <= 0:
        current_target = 2
        current_action = 1
        pub_thread.start()
        movement_thread_count += 1


def go_to_row_3():
    global movement_thread_count, current_target, current_action
    pub_thread = threading.Thread(target=row, args=(2,))
    if movement_thread_count <= 0:
        current_target = 3
        current_action = 1
        pub_thread.start()
        movement_thread_count += 1


def leave_ball():
    global movement_thread_count, current_target, current_action, lrg_ball_grabbed, med_ball_grabbed, sml_ball_grabbed
    pub_thread = threading.Thread(target=row, args=(-1,))
    if movement_thread_count <= 0 and (lrg_ball_grabbed or med_ball_grabbed or sml_ball_grabbed):
        current_target = 0
        current_action = 3
        pub_thread.start()
        movement_thread_count += 1


def object_detection():
    global detection, movement_thread_count, current_action, gripper_state, detection_recovery_count, no_detection_count
    if movement_thread_count <= 0:
        detection = True
        gripper_state = 1
        start_suction()
        movement_thread_count += 1
        current_action = 2
        no_detection_count = 0
        detection_recovery_count = 0


def control_suction():
    global gripper_state
    if gripper_state == 0:
        gripper_state = 1
        start_suction()
    else:
        gripper_state = 0
        stop_suction()


def small_ball_auto():
    global movement_thread_count, current_action, gripper_state, detection_recovery_count, no_detection_count, current_target
    pub_thread = threading.Thread(target=row, args=(10,))
    if movement_thread_count <= 0:
        gripper_state = 1
        start_suction()
        current_action = 5
        no_detection_count = 0
        detection_recovery_count = 0
        current_target = 4
        pub_thread.start()
        movement_thread_count += 1


def medium_ball_auto():
    global movement_thread_count, current_action, gripper_state, detection_recovery_count, no_detection_count, current_target
    pub_thread = threading.Thread(target=row, args=(20,))
    if movement_thread_count <= 0:
        gripper_state = 1
        start_suction()
        current_action = 5
        no_detection_count = 0
        detection_recovery_count = 0
        current_target = 5
        pub_thread.start()
        movement_thread_count += 1


def large_ball_auto():
    global movement_thread_count, current_action, gripper_state, detection_recovery_count, no_detection_count, current_target
    pub_thread = threading.Thread(target=row, args=(30,))
    if movement_thread_count <= 0:
        gripper_state = 1
        start_suction()
        current_action = 5
        no_detection_count = 0
        detection_recovery_count = 0
        current_target = 6
        pub_thread.start()
        movement_thread_count += 1


# ACTION BUTTONS #
window.go_to_goal_btn.pressed.connect(go_to_goal)
window.go_to_row_1_btn.pressed.connect(go_to_row_1)
window.go_to_row_2_btn.pressed.connect(go_to_row_2)
window.go_to_row_3_btn.pressed.connect(go_to_row_3)
window.obj_det_btn.pressed.connect(object_detection)
window.suction_btn.pressed.connect(control_suction)
window.take_ball_goal_btn.pressed.connect(leave_ball)
window.small_ball_btn.pressed.connect(small_ball_auto)
window.medium_ball_btn.pressed.connect(medium_ball_auto)
window.large_ball_btn.pressed.connect(large_ball_auto)

# DIRECTION BUTTONS - PRESSED ACTION #
window.forBut.pressed.connect(forward)
window.flBut.pressed.connect(forward_left)
window.frBut.pressed.connect(forward_right)
window.bacBut.pressed.connect(backward)
window.blBut.pressed.connect(backward_left)
window.brBut.pressed.connect(backward_right)
window.lefBut.pressed.connect(left)
window.rigBut.pressed.connect(right)

# DIRECTION BUTTONS - RELEASED ACTION #
window.forBut.released.connect(reset_fb)
window.bacBut.released.connect(reset_fb)
window.lefBut.released.connect(reset_lr)
window.rigBut.released.connect(reset_lr)
window.flBut.released.connect(reset_both)
window.frBut.released.connect(reset_both)
window.blBut.released.connect(reset_both)
window.brBut.released.connect(reset_both)
app.aboutToQuit.connect(exit_handler)


def amcl_window_update(data):
    global amcl_text, current_amcl
    ref_frame = data.header.frame_id
    amcl_position = str(data.pose.pose)
    current_amcl = data.pose.pose
    amcl_text = "Reference frame: " + ref_frame + "\n\n" + amcl_position


def goal_update(data):
    global goal_text, pose_count
    pose_count = data.goal.target_pose.header.seq
    seq = str(data.goal.target_pose.header.seq)
    ref_frame = data.goal.target_pose.header.frame_id
    goal_pose = str(data.goal.target_pose.pose)
    goal_text = "Sequential number: " + seq + "\nReference frame: " + ref_frame + "\n\n" + goal_pose
    rospy.loginfo("Updating goal")


def gui_update():
    global amcl_text, gripper_state, ball_detected, current_target, stop_threads, current_action, goal_text, sml_ball_grabbed, med_ball_grabbed, lrg_ball_grabbed, movement_thread_count, detection, movement_thread_count

    targets = ["Goal", "ROW 1", "ROW 2", "ROW 3", "SML BALL AUTO", "MED BALL AUTO", "LRG BALL AUTO"]
    actions = ["GO TO GOAL", "GO TO ROW", "DETECT BALLS", "TAKE BALL TO GOAL", "BALL GRABBED", "AUTONOMOUS"]

    while True:
        if stop_threads:
            break

        if movement_thread_count < 0:
            movement_thread_count = 0

        window.amcl_pose_text.setText(amcl_text)
        window.amcl_pose_text_2.setText(amcl_text)
        window.amcl_pose_text_3.setText(amcl_text)
        window.goal_pose_text.setText(goal_text)

        if gripper_state == 0:
            window.gripper_state.setText("Off")
            window.gripper_state_2.setText("Off")
        else:
            window.gripper_state.setText("On")
            window.gripper_state_2.setText("On")

        if sml_ball_grabbed or med_ball_grabbed or lrg_ball_grabbed:
            window.ball_grabbed.setText("True")
            window.ball_grabbed_2.setText("True")
        else:
            window.ball_grabbed.setText("False")
            window.ball_grabbed_2.setText("False")

        if ball_detected == -1:
            window.ball_detected.setText("No data yet")
            window.ball_detected_2.setText("No data yet")
        elif ball_detected == 0:
            window.ball_detected.setText("False")
            window.ball_detected_2.setText("False")
        elif ball_detected == 1:
            window.ball_detected.setText("True")
            window.ball_detected_2.setText("True")

        if current_target == -1:
            window.current_target.setText("None")
            window.current_target_2.setText("None")
        else:
            window.current_target.setText(targets[current_target])
            window.current_target_2.setText(targets[current_target])

        if current_action == -1:
            window.current_action.setText("None")
        else:
            window.current_action.setText(actions[current_action])

        time.sleep(1)
    rospy.loginfo("GUI thread closing")


def start_suction():
    global gripper_state
    rospy.wait_for_service('/malakrobo/vacuum_gripper_large/on')
    rospy.wait_for_service('/malakrobo/vacuum_gripper_medium/on')
    rospy.wait_for_service('/malakrobo/vacuum_gripper_small/on')

    large_srv = rospy.ServiceProxy('/malakrobo/vacuum_gripper_large/on', Empty)
    medium_srv = rospy.ServiceProxy('/malakrobo/vacuum_gripper_medium/on', Empty)
    small_srv = rospy.ServiceProxy('/malakrobo/vacuum_gripper_small/on', Empty)

    large_srv()
    medium_srv()
    small_srv()
    gripper_state = 1


def stop_suction():
    global gripper_state
    rospy.wait_for_service('/malakrobo/vacuum_gripper_large/off')
    rospy.wait_for_service('/malakrobo/vacuum_gripper_medium/off')
    rospy.wait_for_service('/malakrobo/vacuum_gripper_small/off')

    large_srv = rospy.ServiceProxy('/malakrobo/vacuum_gripper_large/off', Empty)
    medium_srv = rospy.ServiceProxy('/malakrobo/vacuum_gripper_medium/off', Empty)
    small_srv = rospy.ServiceProxy('/malakrobo/vacuum_gripper_small/off', Empty)

    large_srv()
    medium_srv()
    small_srv()
    gripper_state = 0


# Function responsible for publishing Twist messages
def talker():
    global move_cmd, stopped, stop_threads  # Defining globals - it will indicate to use variable from outer scope
    # Defining publishers that will be publishing twist messages
    pub = rospy.Publisher('/malakrobo/mobile_base_controller/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(25)  # Publishing rate for messages - 25 msgs a second
    while True:  # Keeps publishing messages
        if stop_threads:
            break

        if move_cmd.angular.z == 0 and move_cmd.linear.x == 0:
            if not stopped:
                pub.publish(move_cmd)
                stopped = True
        else:
            stopped = False

        if not stopped:
            pub.publish(move_cmd)  # Publishing values

        rate.sleep()  # Pauses execution for duration based on the rate given above

    rospy.loginfo("Publisher thread closing")


def auto_object_target(data):
    global camera_center, max_ang_vel, move_cmd, ang_vel, detection, ball_detected, sml_ball_grabbed, med_ball_grabbed, lrg_ball_grabbed, current_action, movement_thread_count, no_detection_count, detection_recovery_count, detect_cycle_count, arrow_pressed, current_target, wait_for_ball
    if len(data.data) > 0:
        ball_detected = 1
    else:
        ball_detected = 0

    if (lrg_ball_grabbed or med_ball_grabbed or sml_ball_grabbed) and detection:
        detection = False
        movement_thread_count -= 1
        current_action = 4

    if len(data.data) > 0 and detection:
        no_detection_count = 0
        detect_cycle_count = 0
        object_width = data.data[1]
        object_height = data.data[2]

        speed_coefficient = float(camera_center / max_ang_vel / 4)

        in_pts = np.array([[0.0, 0.0], [object_width, 0.0], [0.0, object_height], [object_width, object_height]],
                          dtype=np.float32)
        in_pts = np.array([in_pts])
        homography = np.array([[data.data[3], data.data[6], data.data[9]],
                               [data.data[4], data.data[7], data.data[10]],
                               [data.data[5], data.data[8], data.data[11]]], dtype=np.float32)

        out_pts = np.empty((3, 3), dtype=np.float32)

        result = cv2.perspectiveTransform(in_pts, out_pts, homography)
        x_pos = (result[0][0][0] + result[0][1][0] + result[0][2][0] + result[0][3][0]) / 4
        ang_vel = - (x_pos - camera_center) / speed_coefficient

        # print(x_pos, ang_vel)
        if x_pos >= (camera_center + 15) or x_pos <= (camera_center - 15):
            if ang_vel < 0:
                move_cmd.angular.z = min_ang_vel

            elif ang_vel > 0:
                move_cmd.angular.z = max_ang_vel

        if (camera_center - 25) <= x_pos <= (camera_center + 25):
            move_cmd.linear.x = 0.15

        if sml_ball_grabbed or med_ball_grabbed or lrg_ball_grabbed:
            detection = False
    else:
        if not arrow_pressed:
            move_cmd.angular.z = 0
            move_cmd.linear.x = 0

    if len(data.data) == 0 and detection:
        no_detection_count += 1
        if no_detection_count > 15:
            if no_detection_count == 16:
                rospy.loginfo("No objects were detected for 15 frames, starting recovery rotation")

            if detection_recovery_count == 0:
                detect_cycle_count += 1

            if detection_recovery_count < 3:
                detection_recovery_count += 1
                move_cmd.angular.z = max_ang_vel * 4
            elif detection_recovery_count < 8:
                detection_recovery_count += 1
                move_cmd.angular.z = min_ang_vel * 4
            else:
                detection_recovery_count = -3

        if detect_cycle_count == 2 and detection:
            move_cmd.angular.z = 0
            rospy.loginfo("No objects were detected in 2 recovery cycles, aborting!")
            detection = False
            detection_recovery_count = 0
            detect_cycle_count = 0
            no_detection_count = 0
            wait_for_ball = False

    # print(detected)


def lrg_ball_state_update(data):
    global lrg_ball_grabbed
    lrg_ball_grabbed = data.data


def med_ball_state_update(data):
    global med_ball_grabbed
    med_ball_grabbed = data.data


def sml_ball_state_update(data):
    global sml_ball_grabbed
    sml_ball_grabbed = data.data


def watch_window_tab():
    global tab_index, stop_threads
    while True:
        if stop_threads:
            break

        tab_index = window.tabWidget.currentIndex()

    rospy.loginfo("Operation mode polling thread closing")


talker_thread = threading.Thread(target=talker)
tab_watch_thread = threading.Thread(target=watch_window_tab)
gui_thread = threading.Thread(target=gui_update)


# Setup function containing node definition and call to publishing function
def setup():
    global talker_thread, tab_watch_thread, gui_thread
    rospy.init_node('twist_keyboard_publisher', anonymous=True)  # Node definition
    amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_window_update)
    targets_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, goal_update)
    object_sub = rospy.Subscriber('/objects', Float32MultiArray, auto_object_target)
    lrg_ball_sub = rospy.Subscriber('/malakrobo/vacuum_gripper_large/grasping_large', Bool, lrg_ball_state_update)
    med_ball_sub = rospy.Subscriber('/malakrobo/vacuum_gripper_medium/grasping_medium', Bool, med_ball_state_update)
    sml_ball_sub = rospy.Subscriber('/malakrobo/vacuum_gripper_small/grasping_small', Bool, sml_ball_state_update)
    b1_det_thread = threading.Thread(target=get_ball_position, args=(1,))
    b2_det_thread = threading.Thread(target=get_ball_position, args=(2,))
    b3_det_thread = threading.Thread(target=get_ball_position, args=(3,))
    b4_det_thread = threading.Thread(target=get_ball_position, args=(4,))
    auto_pose_1_thread = threading.Thread(target=calculate_pose_1)
    auto_pose_2_thread = threading.Thread(target=calculate_pose_2)
    auto_pose_3_thread = threading.Thread(target=calculate_pose_3)
    b1_det_thread.start()
    b2_det_thread.start()
    b3_det_thread.start()
    b4_det_thread.start()
    auto_pose_1_thread.start()
    auto_pose_2_thread.start()
    auto_pose_3_thread.start()
    talker_thread.start()  # Calling publishing function
    tab_watch_thread.start()
    gui_thread.start()
    window.show()
    app.exec_()


if __name__ == '__main__':
    try:
        rospy.loginfo('Starting...')
        # Defining listener for keyboard events
        # Assigning local functions as callbacks (when key is pressed / released these functions will be called)
        listenerX = keyboard.Listener(
            on_press=on_press,
            on_release=on_release)

        # Starting keyboard listener in a separate thread - it won't block our
        # main thread while waiting for an event from the user
        listenerX.start()
        setup()  # Calling function that performs setup (create a node and call publishing function)
    except rospy.ROSInterruptException:
        pass
