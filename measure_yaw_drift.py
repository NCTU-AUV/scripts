#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

def callback_function(data):
    if not hasattr(callback_function, "start_angle"):
        callback_function.start_angle = data.data[2]
        callback_function.start_time = rospy.get_rostime()
    else:
        rospy.loginfo("Drift rate: %f degree/min", (data.data[2] - callback_function.start_angle) / (rospy.get_rostime() - callback_function.start_time).to_sec())

def listener():
    rospy.init_node('yaw_drift_measurer', anonymous=True)
    rospy.Subscriber('rpi_to_stm32', Float32MultiArray, callback=callback_function)
    rospy.spin()

if __name__ == '__main__':
	listener()
