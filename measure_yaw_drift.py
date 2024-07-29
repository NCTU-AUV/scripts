#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

def callback_function(data):
    rospy.loginfo("I heard:")

def listener():
    rospy.init_node('yaw_drift_measurer', anonymous=True)
	sub = rospy.Subscriber('rpi_to_stm32', Float64MultiArray, callback=callback_function)
	sub.spin()

if __name__ == '__main__':
    listener()
