#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    print("\nPosition:\n\tX:\t" + "{:.7f}".format(position.x) + "\n\tY:\t" + "{:.7f}".format(position.y) + "\n\tZ:\t" + "{:.7f}".format(position.z))
    print("Orientation:\n\tX:\t" + "{:.7f}".format(orientation.x) + "\n\tY:\t" + "{:.7f}".format(orientation.y) + "\n\tZ:\t" + "{:.7f}".format(orientation.z))



rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/malakrobo/mobile_base_controller/odom', Odometry, callback)
rospy.spin()