#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Header
from sensor_msgs.msg import JointState

msgCount = 0
direction = True
def talker(data):
    global msgCount, direction
    countLeft = data.position[0]
    countRight = data.position[1]
    pub = rospy.Publisher('/malakrobo/l_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/malakrobo/r_position_controller/command', Float64, queue_size=10)
    if direction:
	if msgCount > 200:
		direction = False
		msgCount = 0
	else:
		msgCount = msgCount + 1
		countLeft = countLeft + 0.05
		countRight = countRight + 0.05
    else:
	if msgCount > 200:
		direction = True
		msgCount = 0
	else:
		msgCount = msgCount + 1
		countLeft = countLeft - 0.05
		countRight = countRight - 0.05
    rospy.loginfo(countLeft)
    rospy.loginfo(countRight)
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
	#talker()
	listener()
    except rospy.ROSInterruptException:
        pass
