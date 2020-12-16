#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Header
from sensor_msgs.msg import JointState
from pynput import keyboard # Contains keyboard operations

msgCount = 0 # Obsolete, not used anymore, to be removed
direction = True # Obsolete, not used anymore, to be removed
modLeft = 0.0 # Modifier of the left wheel position, used to pass the right values to the publisher
modRight = 0.0 # Modifier of the right wheel position, used to pass the right values to the publisher

# Callback function, called when key is pressed on the keyboard
def on_press(key):
	global modLeft, modRight # Defining globals as the values are meant to be use outside of this function
	if key == keyboard.Key.up: # Arrow up - modifiers set for the robot to go forward
		modLeft = -0.05
		modRight = -0.05
	elif key == keyboard.Key.down: # Arrow down - modifiers set for the robot to go back
		modLeft = 0.05
		modRight = 0.05
	elif key == keyboard.Key.left: # Arrow left - modifiers set for the robot to turn right
		modLeft = 0.05
		modRight = -0.05
	elif key == keyboard.Key.right: # Arrow right - modifiers set for the robot to turn left
		modLeft = -0.05
		modRight = 0.05

# Callback function, called when the key is released - it resets modifiers so the robot stops moving
def on_release(key):
	global modLeft, modRight # Defining globals as the values are meant to be use outside of this function
        if key == keyboard.Key.up or key == keyboard.Key.down or key == keyboard.Key.left or key == keyboard.Key.right:
                modLeft = 0.0
                modRight = 0.0

# Callback function for a subscriber - whenever new message arrives this function is being executed
# It updates the state of joints by adding a modLeft / modRight value to corresponding previous values of joints
def talker(data):
    global msgCount, direction, modLeft, modRight # Defining globals
    countLeft = data.position[0] # Assigning previous left wheel joint position to a local variable
    countRight = data.position[1] # Assigning previous right wheel joint position to a local variable
    
    # Defining publishers that will be publishing updated joint positions
    pub = rospy.Publisher('/malakrobo/l_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/malakrobo/r_position_controller/command', Float64, queue_size=10)
    
    # Working out a new joint position based on the old joint value and the modifier (value dependant on the key press)
    countLeft = countLeft + modLeft
    countRight = countRight + modRight
    
    #rospy.loginfo(countLeft)
    #rospy.loginfo(countRight)
    
    pub.publish(countLeft) # Publishing values
    pub2.publish(countRight)

# Not used anymore, left for reference - to be deleted during cleanup
def callback(data):
    print(data.position)

# Function containg node definition and subscriber
def listener():
    rospy.init_node('pose_publisher', anonymous=True) # Node definition
    rospy.Subscriber('/malakrobo/joint_states', JointState, talker) # Subscriber definition
    rospy.spin() # Keeps program alive


if __name__ == '__main__':
    try:
	# Defining listener for keyboard events
	listenerX = keyboard.Listener(
    		on_press=on_press,
		on_release=on_release) # Assigning local functions as callbacks (when key is pressed / released these functions will be called)
	listenerX.start() # Starting keyboard listener in a separate thread - it won't block our main thread while waiting for an event from the user
	listener() # Calling function containing subscriber
    except rospy.ROSInterruptException:
        pass
