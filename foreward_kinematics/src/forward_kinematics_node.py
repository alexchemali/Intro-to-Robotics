#!/usr/bin/env python

import numpy as np
import scipy as sp
import rospy
import baxter_forward_kinematics as bfk
from sensor_msgs.msg import JointState

def motion(joint_state):

	return bfk.baxter_forward_kinematics_from_joint_state(joint_state)

def listener():

	rospy.Subscriber("/robot/joint_states", JointState, motion)

	rospy.spin()

if __name__=='__main__':
	rospy.init_node('listener', anonymous=True)

listener()
