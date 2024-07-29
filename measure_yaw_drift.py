#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

def callback_function(data):
    rospy.loginfo("I heard: %f", data.data[2])

def listener():
    rospy.init_node('yaw_drift_measurer', anonymous=True)
    rospy.Subscriber('rpi_to_stm32', Float32MultiArray, callback=callback_function)
    rospy.spin()

if __name__ == '__main__':
	listener()
