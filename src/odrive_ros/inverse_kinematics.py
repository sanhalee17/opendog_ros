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
		self.sub = rospy.Subscriber("/footPositionX",PoseStamped,pos_callback)

		#publish leg angles (two separate publishers)
		#do I need to publish on a timer or only when I get a new value???
		self.femur = rospy.Publisher("/theta_f", Float64Stamped, queue_size = 1)
		self.tibia = rospy.Publisher("/theta_t",Float64Stamped, queue_size = 1)


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

	def pos_callback(self, data):
		# Calculate distance from hip joint to foot (d)...
		# ...and angle with respect to x-axis
		d = sqrt(data.position.x^2 + data.position.y^2)
		theta_P = atan(data.position.y / data.position.x)

		# Compute angles foot-hio-knee (theta_K) and hip-knee-foot (theta_HKP)
		theta_K = acos((length_f^2 + d^2 - length_t^2)/(2 * d * length_f))
		theta_HKP = acos((length_t^2 + length_f^2 - d^2) / (2 * length_f * length_t))

		# Calculate desired angles of femur and tibia (including offsets)...
		# ...and publish results
		theta_f = Float64Stamped()
		theta_f.header.stamp = rospy.Time.now()
		theta_f = theta_P - theta_K - theta_K_shift + theta_H
		self.femur.publish(theta_f)

		theta_t = Float64Stamped()
		theta_t.header.stamp = rospy.Time.now()
		theta_t = theta_HKP - theta_HKP_shift - theta_t_shift
		self.tibia.publish(theta_t)





def main(args):
	rospy.init_node('inverse_kinematics',anonymous=True)
	IK = InverseKinematics()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)

