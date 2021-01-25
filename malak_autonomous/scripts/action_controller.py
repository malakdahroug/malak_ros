#!/usr/bin/env python

import cv2
import rospyros
import numpy as np
from std_msgs.msg import Float32MultiArray, Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

camera_center = 400

max_ang_vel = 0.1
min_ang_vel = -0.1
ang_vel = 0
msg_count = 0
detected = True
angular_speed = 0
linear_speed = 0

def talker(data):
    global camera_center, max_ang_vel, detected, angular_speed, linear_speed
    if len(data.data) > 0:
        object_width = data.data[1]
        object_height = data.data[2]

        speed_coefficient = float(camera_center / max_ang_vel / 4)

        in_pts = np.array([[0.0, 0.0], [object_width, 0.0], [0.0, object_height], [object_width, object_height]], dtype=np.float32)
        in_pts = np.array([in_pts])
        homography = np.array([[data.data[3], data.data[6], data.data[9]],
                      [data.data[4], data.data[7], data.data[10]],
                      [data.data[5], data.data[8], data.data[11]]], dtype=np.float32)

        out_pts = np.empty((3, 3), dtype=np.float32)

        result = cv2.perspectiveTransform(in_pts, out_pts, homography)
        x_pos = (result[0][0][0] + result[0][1][0] + result[0][2][0] + result[0][3][0]) / 4
        ang_vel = - (x_pos - camera_center) / speed_coefficient

        print(x_pos, ang_vel)
        if x_pos >= (camera_center + 15) or x_pos <= (camera_center - 15):
            detected = True
            if ang_vel < 0:
                angular_speed = min_ang_vel

            elif ang_vel > 0:
                angular_speed = max_ang_vel

        if (camera_center - 25) <= x_pos <= (camera_center + 25):
            detected = True
            linear_speed = 0.1
        else:
            detected = False

        # print(object_id, object_width, object_height, speed_coefficient, result)
    else:
        detected = False
        angular_speed = 0
        linear_speed = 0


def listener():
    rospy.Subscriber('/objects', Float32MultiArray , talker)
    control_robot()
    rospy.spin()



def control_robot():
    global detected, angular_speed, linear_speed
    pub_velocity = rospy.Publisher('/malakrobo/mobile_base_controller/cmd_vel', Twist, queue_size=1)


    rate = rospy.Rate(5)  # Publishing rate for messages - 25 msgs a second
    while True:
        move_cmd = Twist()  # Defining a message that will be modified depedning on the keyboard keys pressed

        move_cmd.linear.x = linear_speed
        move_cmd.linear.y = 0.0
        move_cmd.linear.z = 0.0
        move_cmd.angular.x = 0.0
        move_cmd.angular.y = 0.0
        move_cmd.angular.z = angular_speed
        pub_velocity.publish(move_cmd)  # Publishing values
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('object_recognition', anonymous=True)
        # talker()
        listener()
    except rospy.ROSInterruptException:
        pass
