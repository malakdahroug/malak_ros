#!/usr/bin/env python
import threading

import rospy
from std_msgs.msg import String, Float64, Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseAction
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

move_cmd = Twist()  # Defining a message that will be modified depending on the keyboard keys pressed

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

current_target = -1
gripper_state = -1
ball_detected = -1
ball_grabbed = -1
current_action = -1
pose_count = 0
current_amcl = PoseWithCovarianceStamped()
movement_thread_count = 0


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
    global move_cmd, tab_index
    if tab_index == 0:
        move_cmd.linear.x = 0.5


def forward_left():
    global move_cmd, tab_index
    if tab_index == 0:
        move_cmd.linear.x = 0.5
        move_cmd.angular.z = 0.5


def forward_right():
    global move_cmd, tab_index
    if tab_index == 0:
        move_cmd.linear.x = 0.5
        move_cmd.angular.z = -0.5


def backward():
    global move_cmd, tab_index
    if tab_index == 0:
        move_cmd.linear.x = -0.5


def backward_left():
    global move_cmd, tab_index
    if tab_index == 0:
        move_cmd.linear.x = -0.5
        move_cmd.angular.z = -0.5


def backward_right():
    global move_cmd, tab_index
    if tab_index == 0:
        move_cmd.linear.x = -0.5
        move_cmd.angular.z = 0.5


def reset_fb():
    move_cmd.linear.x = 0.0


def left():
    global move_cmd, tab_index
    if tab_index == 0:
        move_cmd.angular.z = 0.5


def right():
    global move_cmd, tab_index
    if tab_index == 0:
        move_cmd.angular.z = -0.5


def reset_lr():
    global move_cmd
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
    if movement_thread_count == 0:
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


def row(pose_index):
    global pose_count, current_amcl, move_cmd, stop_threads, movement_thread_count

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    pose_list = []

    if current_amcl.orientation.z > 0.95 and current_amcl.orientation.w < 0.15 or current_amcl.orientation.z < -0.95 and current_amcl.orientation.w < 0.15 and -0.075 < current_amcl.position.x < 0.075 and -0.075 < current_amcl.position.y < 0.075:
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

    if pose_index == 0:
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

    movement_thread_count -= 1
    rospy.loginfo("Closing position control thread")


def go_to_row_1():
    global movement_thread_count, current_target, current_action
    pub_thread = threading.Thread(target=row, args=(0,))
    if movement_thread_count == 0:
        current_target = 1
        current_action = 1
        pub_thread.start()
        movement_thread_count += 1


def go_to_row_2():
    global movement_thread_count, current_target, current_action
    pub_thread = threading.Thread(target=row, args=(1,))
    if movement_thread_count == 0:
        current_target = 2
        current_action = 1
        pub_thread.start()
        movement_thread_count += 1


def go_to_row_3():
    global movement_thread_count, current_target, current_action
    pub_thread = threading.Thread(target=row, args=(2,))
    if movement_thread_count == 0:
        current_target = 3
        current_action = 1
        pub_thread.start()
        movement_thread_count += 1


# ACTION BUTTONS #
window.go_to_goal_btn.pressed.connect(go_to_goal)
window.go_to_row_1_btn.pressed.connect(go_to_row_1)
window.go_to_row_2_btn.pressed.connect(go_to_row_2)
window.go_to_row_3_btn.pressed.connect(go_to_row_3)

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
    global amcl_text, gripper_state, ball_grabbed, ball_detected, current_target, stop_threads, current_action, goal_text

    targets = ["Goal", "ROW 1", "ROW 2", "ROW 3"]
    actions = ["GO TO GOAL", "GO TO ROW"]
    while True:
        if stop_threads:
            break

        window.amcl_pose_text.setText(amcl_text)
        window.amcl_pose_text_2.setText(amcl_text)
        window.amcl_pose_text_3.setText(amcl_text)
        window.goal_pose_text.setText(goal_text)

        if gripper_state == -1:
            window.gripper_state.setText("No data yet")
            window.gripper_state_2.setText("No data yet")

        if ball_grabbed == -1:
            window.ball_grabbed.setText("No data yet")
            window.ball_grabbed_2.setText("No data yet")

        if ball_detected == -1:
            window.ball_detected.setText("No data yet")
            window.ball_detected_2.setText("No data yet")

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
