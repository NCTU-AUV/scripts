#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

def callback(data):
    rospy.loginfo("I heard: %f", data.data[2])

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('rpi_to_stm32', Float64MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()