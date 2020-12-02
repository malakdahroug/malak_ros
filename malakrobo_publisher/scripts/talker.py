#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Header
from sensor_msgs.msg import JointState

# Testing script for robot movement

# Storing the count of how many messages where sent to the robot
msgCount = 0

# Controls direction of the movement of the robot
direction = True

# Callback function called when message is received from the subscriber
# Subscribed to joint_states topic that contains information about current postion of joints
def talker(data):
    # Variables defined as global as they have to be storing the value between executions of the function
    global msgCount, direction
    countLeft = data.position[0] # Left wheel joint angle
    countRight = data.position[1] # Right wheel joint angle

    # Defining publishers that write to topics that control position of robot wheel joints in Gazebo
    pub = rospy.Publisher('/malakrobo/l_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/malakrobo/r_position_controller/command', Float64, queue_size=10)
    
    # Controls direction of the robot, if true robot will be moving forward, else backwards
    if direction:
	# If threshold is reached it changes direction and resets msgCount (assigns 0 to it)
	if msgCount > 200:
		direction = False
		msgCount = 0
	else:
		msgCount = msgCount + 1 # Message is being prepared so increasing the count
		countLeft = countLeft + 0.05 # Taking previous joint position and adding 0.05 to it
		countRight = countRight + 0.05 # Taking previous joint position and adding 0.05 to it
    # Responsible for robot backwards movement
    else:
	# If threshold is reached it flips the direction and resets msgCount
	if msgCount > 200:
		direction = True
		msgCount = 0
	else:
		msgCount = msgCount + 1 # Increases message count as message is being prepared
		countLeft = countLeft - 0.05 # Takes previous joint position and subtracts 0.05 from it
		countRight = countRight - 0.05 # Takes previous joint position and subtracts 0.05 from it
    rospy.loginfo(countLeft) # Display new joint positions
    rospy.loginfo(countRight)

    pub.publish(countLeft) #Publish new joint positions
    pub2.publish(countRight)

# Not used anymore, left for reference - will be cleaned up
def callback(data):
    print(data.position)

# Defining subscriber that will be retrieving joint positions from /malakrobo/joint_states topic
def listener():
    rospy.init_node('pose_publisher', anonymous=True) # Defining node
    rospy.Subscriber('/malakrobo/joint_states', JointState, talker) # Defining subscriber
    rospy.spin() # Keeping node alive

if __name__ == '__main__':
    try:
	#talker()
	listener()
    except rospy.ROSInterruptException:
        pass
