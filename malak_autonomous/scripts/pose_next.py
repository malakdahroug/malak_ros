#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

pose_list = []

f = open("poses_new", "r")
while True:
    line = f.readline()
    if not line == "":
        if line == "pose: \n":
            f.readline()
            f.readline()
            pose_read = []
            pose_read.append(float(f.readline().replace("\n", "").replace("x:", "").replace(" ", "")))
            pose_read.append(float(f.readline().replace("\n", "").replace("y:", "").replace(" ", "")))
            f.readline()
            f.readline()
            f.readline()
            f.readline()
            pose_read.append(float(f.readline().replace("\n", "").replace("z:", "").replace(" ", "")))
            pose_read.append(float(f.readline().replace("\n", "").replace("w:", "").replace(" ", "")))
            pose_list.append(pose_read)
    else:
        break

pose_sent = False
pose_count = 0
status_count = 0


def talker():
    global pose_count, pose_list, pose_sent, status_count

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()


    while True:
        next_pose = raw_input("Next pose? y/n")

        if next_pose == "y":
            goal = MoveBaseGoal()
            goal.target_pose.header.seq = pose_count + 1
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.header.frame_id = "odom"

            goal.target_pose.pose.position.x = pose_list[pose_count][0]
            goal.target_pose.pose.position.y = pose_list[pose_count][1]
            goal.target_pose.pose.position.z = 0.0

            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = pose_list[pose_count][2]
            goal.target_pose.pose.orientation.w = pose_list[pose_count][3]
            print('Setting goal:')
            print(goal)
            print('--------------------------------------------------')

            client.send_goal(goal)
            wait = client.wait_for_result()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                pose_count += 1
                print(client.get_result())

        elif next_pose == "n":
            break

def listener():
    rospy.init_node('auto_pose_control', anonymous=True)
    rospy.Subscriber('/move_base/status', GoalStatusArray, talker)
    rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('auto_pose_control', anonymous=True)
        talker()
        # listener()
    except rospy.ROSInterruptException:
        pass
