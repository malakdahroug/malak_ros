#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from pynput import keyboard  # Contains keyboard operations

move_cmd = Twist()  # Defining a message that will be modified depedning on the keyboard keys pressed

move_cmd.linear.x = 0.0
move_cmd.linear.y = 0.0
move_cmd.linear.z = 0.0
move_cmd.angular.x = 0.0
move_cmd.angular.y = 0.0
move_cmd.angular.z = 0.0
movement = 0
stopped = False

# Callback function, called when the key is released - it sets twist message properties to 0.0 so the robot stops moving
def on_release(key):
    global move_cmd, stopped
    if key == keyboard.Key.up or key == keyboard.Key.down:  # If key up/down is release it is supposed to reset linear x speed
        move_cmd.linear.x = 0.0  # Reset linear x movement
    if key == keyboard.Key.left or key == keyboard.Key.right:  # If key left/right is release it is supposed to reset angular z speed
        move_cmd.angular.z = 0.0  # Reset angular z movement


# Callback function, called when key is pressed on the keyboard
def on_press(key):
    global move_cmd, stopped  # Defining globals as the values are meant to be use outside of this function
    if key == keyboard.Key.up:  # Arrow up - twist message set for the robot to go forward
        move_cmd.linear.x = 0.5
    elif key == keyboard.Key.down:  # Arrow down - twist message set for the robot to go back
        move_cmd.linear.x = -0.5
    elif key == keyboard.Key.left:  # Arrow left - twist message set for the robot to turn right
        move_cmd.angular.z = 0.5
    elif key == keyboard.Key.right:  # Arrow right - twist message set for the robot to turn left
        move_cmd.angular.z = -0.5



# Function responsible for publishing Twist messages
def talker():
    global move_cmd, stopped  # Defining globals - it will indicate to use variable from outer scope
    # Defining publishers that will be publishing twist messages
    pub = rospy.Publisher('/malakrobo/mobile_base_controller/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(25)  # Publishing rate for messages - 25 msgs a second
    while True:  # Keeps publishing messages

        if move_cmd.angular.z == 0 and move_cmd.linear.x == 0:
            if not stopped:
                pub.publish(move_cmd)
                stopped = True
        else:
            stopped = False
        if not stopped:
            pub.publish(move_cmd)  # Publishing values
        rate.sleep()  # Pauses execution for duration based on the rate given above


# Setup function containing node definition and call to publishing function
def setup():
    rospy.init_node('twist_keyboard_publisher', anonymous=True)  # Node definition
    talker()  # Calling publishing function


if __name__ == '__main__':
    try:
        # Defining listener for keyboard events
        listenerX = keyboard.Listener(
            on_press=on_press,
            on_release=on_release)  # Assigning local functions as callbacks (when key is pressed / released these functions will be called)
        listenerX.start()  # Starting keyboard listener in a separate thread - it won't block our main thread while waiting for an event from the user
        setup()  # Calling function that performs setup (create node and call publishing function)
    except rospy.ROSInterruptException:
        pass
