import rospy
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64, Header
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray

pose_list = []

pose_list.append([0.0, 0.0, 0.0, 1.0])
pose_list.append([1.0, 0.0, 0.0, 1.0])
pose_list.append([1.5, 0.0, 0.7, 0.7])
pose_list.append([1.5, 1.5, -0.7, 0.7])
pose_list.append([1.5, -0.5, -0.7, 0.7])
pose_list.append([1.5, -1.6, -0.7, 0.7])
pose_list.append([2.0, -2.75, 0.0, 1.0])
pose_list.append([2.65, -2.5, 0.7, 0.7])
pose_list.append([2.65, -1.6, 0.7, 0.7])
pose_list.append([2.65, -0.5, 0.7, 0.7])
pose_list.append([2.65, 0.5, 0.7, 0.7])
pose_list.append([2.65, 1.6, -0.236625353902, 0.971600968449])
pose_list.append([3.75, 1.0, -0.7, 0.7])
pose_list.append([3.75, -0.7, -0.7, 0.7])
pose_list.append([3.75, -1.7, -0.7, 0.7])
pose_list.append([2.5, -2.5, 0.7, 0.7])
pose_list.append([2.6, -0.25, 0.999876044897, 0.0157446765743])
pose_list.append([0.0, 0.0, 0.0, 1.0])



pose_sent = False
pose_count = 0
status_count = 0

def talker(data):
    global pose_count, pose_list, pose_sent, status_count

    goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)

    if pose_count == 0:
        goal = PoseStamped()
        goal.header.seq = pose_count + 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "odom"

        goal.pose.position.x = pose_list[pose_count][0]
        goal.pose.position.y = pose_list[pose_count][1]
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = pose_list[pose_count][2]
        goal.pose.orientation.w = pose_list[pose_count][3]
        print('Setting goal:')
        print(goal)
        print('--------------------------------------------------')
        goal_publisher.publish(goal)

        pose_count += 1
        pose_sent = True
        status_count = 0
    elif (pose_count <= (len(pose_list) - 1) and not pose_sent):
        goal = PoseStamped()
        goal.header.seq = pose_count
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "odom"

        goal.pose.position.x = pose_list[pose_count][0]
        goal.pose.position.y = pose_list[pose_count][1]
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = pose_list[pose_count][2]
        goal.pose.orientation.w = pose_list[pose_count][3]
        print('Setting goal:')
        print(goal)
        print('--------------------------------------------------')
        goal_publisher.publish(goal)

        pose_count += 1
        pose_sent = True
        status_count = 0

    if not len(data.status_list) == 0:
        if data.status_list[0].status == 3:
            status_count += 1
    else:
        pose_count = 0
        pose_sent = False
        status_count = 0

    if status_count == 30:
        pose_sent = False

def listener():
    rospy.init_node('auto_pose_control', anonymous=True)
    rospy.Subscriber('/move_base/status', GoalStatusArray, talker)
    rospy.spin()

if __name__ == '__main__':
    try:
	#talker()
	listener()
    except rospy.ROSInterruptException:
        pass
