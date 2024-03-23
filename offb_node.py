#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import numpy as np

from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


def quaternion_to_euler(quaternion):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    :param quaternion: A numpy array representing the quaternion [w, x, y, z].
    :return: A numpy array representing the Euler angles [roll, pitch, yaw] in degrees.
    """
    # Extract components
    w, x, y, z = quaternion

    # Calculate roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.degrees(np.arctan2(t0, t1))

    # Calculate pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2 > 1.0, 1.0, t2)
    t2 = np.where(t2 < -1.0, -1.0, t2)
    pitch_y = np.degrees(np.arcsin(t2))

    # Calculate yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.degrees(np.arctan2(t3, t4))

    # Return Euler angles in degrees
    return np.array([roll_x, pitch_y, yaw_z])

def euler_to_quaternion(angle):

    yaw = np.radians(angle[2])
    pitch = np.radians(angle[1])
    roll = np.radians(angle[0])

    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    return np.array([qw, qx, qy, qz])


def state_cb_imu(msg):
    global array
    temp_array = [0, 0, 0, 0]
    temp_array[0] = msg.orientation.w
    temp_array[1] = msg.orientation.x
    temp_array[2] = msg.orientation.y
    temp_array[3] = msg.orientation.z

    quaternion = (temp_array[0], temp_array[1], temp_array[2], temp_array[3])

    global origin_state
    global fix
    if fix > 0:
        fix -= 1
        array = [0, 0, 0, 0, 0, 0, 0, 0]
    elif fix == 0:
        fix = -1
        origin_state = quaternion_to_euler(quaternion)
        array = [0, 0, 0, 0, 0, 0, 0, 0]
    else :
        error_angle = [0, 0, 0]
        error_angle = quaternion_to_euler(quaternion) - (origin_state[0], origin_state[1], origin_state[2])


        array[0] = error_angle[0] * -1
        array[1] = error_angle[1]
        array[2] = error_angle[2]

def target_cb(msg):
    global array
    global delay
    global target_angle
        #if delay > 0:
        #    array[3] = msg.data[0]
        #    array[4] = msg.data[1]

        #if array[3] != msg.data[0] | array[4] != msg.data[1]:
        #    delay = 10

    array[3] = msg.data[0]
    array[4] = msg.data[1]
    array[5] = msg.data[2]
    target_angle = msg.data[3]
    array[6] = msg.data[4]
    array[7] = msg.data[5]

if __name__ == "__main__":
    rospy.init_node("offb_node_py")
    imu_sub = rospy.Subscriber("mavros/imu/data", Imu, callback = state_cb_imu)
    target_sub = rospy.Subscriber("target", Float32MultiArray, callback = target_cb)
    pub = rospy.Publisher('rpi_to_stm32', Float32MultiArray, queue_size=10)

    #array 0~2 : error_angle
    #array 3~4 : ex.x ex.y
    #array 5   : desired_depth
    #array 6   : arm_state
        #array 7   : target_arm
    array = [0, 0, 0, 0, 0, 0, 0, 0]
    origin_state = [0, 0, 0]
    fix = 100
    delay = 0
    target_angle = 0
    #data for pub
    data_to_send = Float32MultiArray()  # the data to be sent, initialise the array
    data_to_send.data = array # assign the array with the value you want to send

    #rate
    rate = rospy.Rate(10)

    while (not rospy.is_shutdown()):

        array[2] += target_angle
        if array[2] > 180:
            array[2] -= 360
        if array[2] < -180:
            array[2] += 360
        data_to_send.data = array
        pub.publish(data_to_send)
        #rospy.loginfo("Publish")
        #rospy.loginfo(array)

        rate.sleep()
