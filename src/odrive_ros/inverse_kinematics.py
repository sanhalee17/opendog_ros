#!/usr/bin/env python

# Author: Gabrielle Conard, July 2019
# File: inverse_kinematics.py
# This python script takes a given position of the foot (with respect to the hip)...
# ...and finds the angles of the upper and lower leg (femur and tibia) required to achieve that position.

#basics
import rospy
import sys
import roslib
roslib.load_manifest('odrive_ros')


import tf.transformations
import tf_conversions
import tf2_ros

# Imports message types and services from several libraries
from std_msgs.msg import Float64  #,Float64Stamped, Int32Stamped
from geometry_msgs.msg import TwistStamped, TransformStamped, Pose #PoseStamped
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
		# Subscribe to a foot position P: (xP, yP)
		# self.sub = rospy.Subscriber("/footPosition",PoseStamped,pos_callback)
		self.sub = rospy.Subscriber("/footPosition",Pose,self.pos_callback)

		#publish leg angles (two separate publishers)
		#do I need to publish on a timer or only when I get a new value???
		# self.femur = rospy.Publisher("/theta_f", Float64Stamped, queue_size = 1)
		# self.tibia = rospy.Publisher("/theta_t",Float64Stamped, queue_size = 1)
		self.femur = rospy.Publisher("/theta_f", Float64, queue_size = 1)
		self.tibia = rospy.Publisher("/theta_t",Float64, queue_size = 1)


		# Class Variables

		# Measured Values:
		# All lengths are in inches and angles are in degrees (and converted to radians)
		self.length_f = 14.125
		self.length_t = 13.25
		self.theta_K_shift = 16.6*(pi/180)
		self.theta_HKP_shift = 1*(pi/180)
		self.theta_H = 16.9*(pi/180)
		self.theta_t_shift = 15.8*(pi/180)

		# Range of Motion
		# self.R = 27.6236  # empirical
		self.R = 27.375   # sum of length_f and length_t
		self.r = 17.9904
		self.theta_min_E = 39.91*(pi/180)
		self.theta_max_E = 119.29*(pi/180)
		self.theta_min_C = 4.0002*(pi/180)
		self.theta_max_C = 71.1922*(pi/180)

		# To be calculated...
		self.d = None
		self.theta_P = None
		self.theta_K = None
		self.theta_HKP = None
		# Goal:
		self.theta_f = None
		self.theta_t = None



	def pos_callback(self, data):
		print("Received position!")
		# Calculate distance from hip joint to foot (d)...
		# ...and angle with respect to x-axis
		self.d = sqrt(data.position.x**2 + data.position.y**2)
		self.theta_P = pi -  arctan2(data.position.y, data.position.x)
		print(self.theta_P)

		if(self.d > self.R):
			print("Too far, can't reach that!")
		elif(self.d < self.r):
			print("Too close, can't reach that either")
		elif(self.theta_P > self.theta_max_E):
			print("Angle is too large")
		elif(self.theta_P < self.theta_min_C):
			print("Angle is too small")
		else:
			# Compute angles foot-hip-knee (theta_K) and hip-knee-foot (theta_HKP)
			self.theta_K = arccos((self.length_f**2 + self.d**2 - self.length_t**2)/(2 * self.d * self.length_f))
			self.theta_HKP = arccos((self.length_t**2 + self.length_f**2 - self.d**2) / (2 * self.length_f * self.length_t))

			# Calculate desired angles of femur and tibia (including offsets)...
			# ...and publish results
			# self.theta_f = Float64Stamped()
			# self.theta_f.header.stamp = rospy.Time.now()
			self.theta_f = Float64()
			self.theta_f = self.theta_P - self.theta_K - self.theta_K_shift + self.theta_H
			print("theta_f: " + str(self.theta_f))
			self.femur.publish(self.theta_f)

			# self.theta_t = Float64Stamped()
			# self.theta_t.header.stamp = rospy.Time.now()
			self.theta_t = Float64()
			self.theta_t = self.theta_HKP - self.theta_HKP_shift - self.theta_t_shift
			print("theta_t: " + str(self.theta_t))
			self.tibia.publish(self.theta_t)





def main(args):
	rospy.init_node('inverse_kinematics',anonymous=True)
	IK = InverseKinematics()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)

