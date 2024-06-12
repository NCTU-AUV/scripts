#! /usr/bin/env python3
import time
import sys
from select import select
import termios
import tty
import threading
import rospy
from std_msgs.msg import Float32MultiArray



class KeyboardController:

    def __init__(self):
        rospy.init_node("keyboard_py") 
        self.dx = 0.0
        self.dy = 0.0
        self.depth = 0.1
        self.target_yaw = 0.0
        self.drop = 0.0
        self.arm_mode = 0.0

        self.LIN_VEL_STEP = 1.0
        self.DEP_VEL_STEP = 0.1
        self.ANG_VEL_STEP = 10.0

        self.settings = termios.tcgetattr(sys.stdin)
        self.key_timeout = 0.5

        self.pub = rospy.Publisher(
            'target',
			Float32MultiArray,
			queue_size = 10)
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], self.key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        key = self.get_key()
        if key == 'w':
            if (not self.dx == 0.0):
                self.dx += self.LIN_VEL_STEP
            else:
                self.dx = self.LIN_VEL_STEP
            self.dy = 0.0
            rospy.loginfo("front")
        elif key == 'a':
            self.dx = 0.0
            self.dy = -self.LIN_VEL_STEP
            rospy.loginfo("left")
        elif key == 's':
            if (not self.dx == 0.0):
                self.dx -= self.LIN_VEL_STEP
            else:
                self.dx = -self.LIN_VEL_STEP
            self.dy = 0.0
            rospy.loginfo("back")
        elif key == 'd':
            self.dx = 0.0
            self.dy = self.LIN_VEL_STEP
            rospy.loginfo("right")
        elif key == 'k':
            if (self.depth - self.DEP_VEL_STEP) >= 0.0:
                self.depth -= self.DEP_VEL_STEP
                rospy.loginfo("up: depth = %f" % self.depth)
        elif key == 'j':
            self.depth += self.DEP_VEL_STEP
            rospy.loginfo("down: depth = %f" % self.depth)
        elif key == 'h':
            self.target_yaw -= self.ANG_VEL_STEP
            rospy.loginfo("turn_left: yaw = %f" % self.target_yaw)
        elif key == 'l':
            self.target_yaw += self.ANG_VEL_STEP
            rospy.loginfo("turn_right: yaw = %f" % self.target_yaw)
        elif key == 'q':
            self.dx = 0.0
            self.dy = 0.0
            rospy.loginfo("stop")
        elif key == 'u':
            if (self.drop == 1.0):
                self.drop = 0.0
                rospy.loginfo("drop close")
            else:
                self.drop = 1.0
                rospy.loginfo("drop open")
        elif key == 'i':
            self.arm_mode = 0.0
            rospy.loginfo("arm idle")
        elif key == 'o':
            self.arm_mode = 1.0
            rospy.loginfo("arm grab")
        elif key == 'p':
            self.arm_mode = 2.0
            rospy.loginfo("arm release")

        msg = Float32MultiArray()
        msg.data = [self.dx, self.dy, self.depth, self.target_yaw, self.drop, self.arm_mode]
        self.pub.publish(msg)

def main():

    node = KeyboardController()
	
    rate = rospy.Rate(10)

    while (not rospy.is_shutdown()):
        node.run()
        rate.sleep()


if __name__ == "__main__":
    main()
