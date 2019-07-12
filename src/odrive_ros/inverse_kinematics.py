#!/usr/bin/env python

# Author: Gabrielle Conard, July 2019
# File: inverse_kinematics.py
# This python script takes a given position of the foot (with respect to the hip)...
# ...and finds the angles of the upper and lower leg (femur and tibia) required to achieve that position.

#basics
import rospy
roslib.load_manifest('odrive_ros')


import tf.transformations
import tf_conversions
import tf2_ros

# Imports message types and services from several libraries
from std_msgs.msg import Float64Stamped, Int32Stamped
from geometry_msgs.msg import TwistStamped, TransformStamped, PoseStamped
import std_srvs.srv

import time
from numpy import *
import traceback
import Queue   # might not be needed


class InverseKinematics:
	def __init__(self):
		#if you need parameters, use the following
		#self.mything = rospy.get_param('param_name',default_value)

		# Publishers and Subscribers
		#subscribe to a foot position P: (xP, yP)
		# calls function to calculate d and thetaP

		#publish leg angles (two separate publishers)
		#do I need to publish on a timer or only when I get a new value???


		# Class Variables

		# Measured Values:
		# *********Values to be determined**********
		self.length_f = None
		self.length_t = None
		self.theta_K_shift = None
		self.theta_HKP_shift = None
		self.theta_H = None
		self.theta_t_shift = None

		# To be calculated...
		self.d = None
		self.theta_P = None
		self.theta_K = None
		self.theta_HKP = None
		# Goal:
		self.theta_f = None
		self.theta_t = None

	#def 





def main(args):
	rospy.init_node('inverse_kinematics',anonymous=True)
	IK = InverseKinematics()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)

