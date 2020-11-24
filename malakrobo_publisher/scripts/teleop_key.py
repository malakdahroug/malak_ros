#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Header
from sensor_msgs.msg import JointState
from pynput import keyboard

msgCount = 0
direction = True
modLeft = 0.0
modRight = 0.0

def on_press(key):
	global modLeft, modRight
	if key == keyboard.Key.up:
		modLeft = 0.05
		modRight = 0.05
	elif key == keyboard.Key.down:
		modLeft = -0.05
		modRight = -0.05
	elif key == keyboard.Key.left:
		modLeft = 0.05
		modRight = -0.05
	elif key == keyboard.Key.right:
		modLeft = -0.05
		modRight = 0.05

def on_release(key):
	global modLeft, modRight
        if key == keyboard.Key.up or key == keyboard.Key.down or key == keyboard.Key.left or key == keyboard.Key.right:
                modLeft = 0.0
                modRight = 0.0


def talker(data):
    global msgCount, direction, modLeft, modRight
    countLeft = data.position[0]
    countRight = data.position[1]
    
    pub = rospy.Publisher('/malakrobo/l_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/malakrobo/r_position_controller/command', Float64, queue_size=10)
    
    countLeft = countLeft + modLeft
    countRight = countRight + modRight
    
    #rospy.loginfo(countLeft)
    #rospy.loginfo(countRight)
    
    pub.publish(countLeft)
    pub2.publish(countRight)

def callback(data):
    print(data.position)

def listener():
    rospy.init_node('pose_publisher', anonymous=True)
    rospy.Subscriber('/malakrobo/joint_states', JointState, talker)
    rospy.spin()


if __name__ == '__main__':
    try:
	listenerX = keyboard.Listener(
    		on_press=on_press,
		on_release=on_release)
	listenerX.start()
	listener()
    except rospy.ROSInterruptException:
        pass
