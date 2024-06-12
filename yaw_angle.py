#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import numpy as np

from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
#from geometry_msgs.msg import PoseStamped
#from mavros_msgs.msg import State
#from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

#from collections import deque

def adjust_angle(angle):
	if angle >= -180 and angle <= 180:
		return angle
	elif angle > 180:
		return angle - 360
	elif angle < -180:
		return angle + 360
	else:
		raise ValueError("Angle should be between -180 and 180 degrees")

def state_cb_imu(msg):
	global angle, enable
	if msg.orientation.w != 0 :
		enable = 1
	
	limit = 0.005
	param = 0.89

	if 	msg.angular_velocity.z > limit:
		param = 0.12
	
	yaw_angular = param * msg.angular_velocity.z

	dt = 1/100.0
	if enable :
		
		angle[0] += yaw_angular * dt
		# 0~360
		if angle[0] < 0:
			angle[0] += 360
		if angle[0] >= 360:
			angle[0] -= 360
	

if __name__ == "__main__":
	rospy.init_node("yaw_angle_py")
	imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, callback = state_cb_imu)
	pub = rospy.Publisher('yaw_angle', Float32MultiArray, queue_size = 1)

	time = 0.0
	enable = 0
	angle = [0]

	#data for pub
	data_to_send = Float32MultiArray()  # the data to be sent, initialise the array
	data_to_send.data = angle # assign the array with the value you want to send

	#rate
	rate = rospy.Rate(100)

	while (not rospy.is_shutdown()):
		
		angle[0] = adjust_angle(angle[0])
		data_to_send.data = angle
		pub.publish(data_to_send)	
		rate.sleep()


