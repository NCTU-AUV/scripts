#! /usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

def stm32_cb(msg):
    global array
    array[1] = msg.data[2]
	array[2] = msg.data[3]

def state_cb(msg):
    global array
    array[0] = msg.data[2]


if __name__ == "__main__":
    rospy.init_node("rpi_to_oring_py")
    stm32_sub = rospy.Subscriber("stm32_to_rpi", Float32MultiArray, callback = stm32_cb)
    rpi_sub = rospy.Subscriber("rpi_to_stm32", Float32MultiArray, callback = state_cb)
    pub = rospy.Publisher('rpi_to_oring', Float32MultiArray, queue_size=10)

    #array 0 : error_angle
    #array 1 : current_arm
    #array 2 : depth 
    array = [0, 0, 0]
    
    #data for pub
    data_to_send = Float32MultiArray()  # the data to be sent, initialise the array
    data_to_send.data = array # assign the array with the value you want to send

    #rate
    rate = rospy.Rate(10)

    while (not rospy.is_shutdown()):

        data_to_send.data = array
        pub.publish(data_to_send)
        #rospy.loginfo("Publish")
        #rospy.loginfo(array)

        rate.sleep()


