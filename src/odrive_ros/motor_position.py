#!/usr/bin/env python

# Author: Gabrielle Conard, July 2019
# File: motor_position.py
# This python script takes the leg angles from inverse_kinematics...
# ...and finds the positions of the two motors required to achieve those angles.

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


class MotorPosition:
	def __init__(self):
		#if you need parameters, use the following
		#self.mything = rospy.get_param('param_name',default_value)

		#subscribe to theta_f and theta_t from inverse_kinematics
		self.sub_F = rospy.Subscriber("/theta_f", Float64Stamped, femur_motor_callback)
		self.sub_T = rospy.Subscriber("/theta_t", Float64Stamped, tibia_motor_callback)

		#publish motor positions
		# How do I get both in one publisher to output a Pose msg?
		# Do I set up a timer?

		# Class variables
		# Measured Values:
		# *********Values to be determined**********
		self.length_constraint = None
		self.length_linkH = None
		self.length_mountH = None

		self.length_cons_K = None
		self.length_link_K = None
		self.length_mountK = None

		self.ball_screw = None

		# To be calculated...
		self.theta_HLN = None
		self.theta_HNL = None
		self.theta_KLN = None
		self.theta_KNL = None

		self.length_HBN = None
		self.length_KBN = None

		self.des_pos_f = None
		self.des_pos_t = None



	def femur_motor_callback(self, data):
		self.theta_HNL = asin((self.length_constraint * sin(self.theta_f)) / self.length_linkH)
		self.theta_HLN = 180 - self.theta_f - self.theta_HNL

		self.length_HBN = ((self.length_linkH * sin(self.theta_HLN)) / sin(self.theta_f)) - self.length_mountH
		# reference to bottom mount?
		self.des_pos_f = self.ball_screw - self.length_HBN

	def tibia_motor_callback(self, data):


	# function that takes these two distances and converts them into motor positions
	# need a self.last_pos_f too so that motor knows where it needs to turn to
	# consider direction (cw or ccw)




def main(args):
	rospy.init_node('motor_position',anonymous=True)
	MP = MotorPosition()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)
