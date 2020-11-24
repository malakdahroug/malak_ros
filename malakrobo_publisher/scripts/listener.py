#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Pose, Quaternion, Twist, Vector3


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    vel_msg = Twist()
    vel_msg.linear.x = 4.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 2.5 
    print(data.data)
    if(data.data == 'Spin!'):
        pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        pub.publish(vel_msg)
        print('Publishing')

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/give_me_commands', String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()