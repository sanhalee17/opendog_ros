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
		self.pub = rospy.Publisher("/cmd_pos", PoseStamped, queue_size = 1)

		# Set up a timed loop
		rospy.Timer(rospy.Duration(0.01), self.timer_callback, oneshot=False)

		# Class variables
		# Measured Values:
		# All lengths are in inches and angles are in degrees
		self.constraint_H = 4.047
		self.link_H = 7.047
		self.mount_H = 2.294

		self.constraint_K = 3.739
		self.link_K = 7.047
		self.mount_K = 2.177

		self.ball_screw = 7.087    # full possible range, but really can't go this far

		# To be calculated...
		self.theta_HLN = None
		self.theta_HNL = None
		self.theta_KLN = None
		self.theta_KNL = None

		self.length_HBN = None
		self.length_KBN = None

		self.des_BN_f = None
		self.last_BN_f = None
		self.delta_BN_f = None
		self.des_BN_t = None
		self.last_BN_t = None
		self.delta_BN_t = None

		self.des_pos_f = None
		self.last_pos_f = None
		self.delta_motor_f = None
		self.des_pos_t = None
		self.last_pos_t = None
		self.delta_motor_t = None


		# Conversion
		self.mm_to_in = 1 / 25.4   # 1 in = 25.4 mm
		self.distance_to_motor_pos = (2.2114 / (self.mm_to_in * 5)) * (8192)   # 5 mm = 2.2114 rev = 2.2114 * 8192 counts
		self.rev_to_count = 8192
		self.deg_to_count = 8192 / 360   # 1 revolution = 360 degrees = 8192 counts



	def femur_motor_callback(self, data):
		self.theta_HNL = arcsin((self.constraint_H * sin(self.theta_f)) / self.link_H)
		self.theta_HLN = 180 - self.theta_f - self.theta_HNL

		self.length_HBN = ((self.link_H * sin(self.theta_HLN)) / sin(self.theta_f)) - self.mount_H
		# reference to bottom mount?
		self.des_pos_f = self.ball_screw - self.length_HBN
		self.delta_BN_f = self.des_BN_f - self.last_BN_f

		# Reset value of last ball nut positon
		# Do we want real feedback?
		self.last_BN_f = self.des_BN_f

	def tibia_motor_callback(self, data):
		self.theta_KNL = arcsin((self.constraint_K * sin(self.theta_t)) / self.link_K)
		self.theta_KLN = 180 - self.theta_KNL - self.theta_t

		self.length_KBN = ((self.link_K * sin(self.theta_KLN)) / sin(self.theta_t)) - self.mount_K
		self.des_BN_t = self.ball_screw - self.length_KBN
		self.delta_BN_t = self.des_BN_t - self.last_BN_t

		# Reset value of last ball nut positon
		# Do we want real feedback?
		self.last_BN_t = self.des_BN_t


	# function that takes these two distances and converts them into motor positions
	# need a self.last_pos_f too so that motor knows where it needs to turn to
	# consider direction (cw or ccw)
	def timer_callback(self, data):
		self.delta_motor_f = self.delta_BN_f * self.distance_to_motor_pos
		self.des_pos_f = self.delta_motor_f + self.last_pos_f

		self.delta_motor_t = self.delta_BN_t * self.distance_to_motor_pos
		self.des_pos_t = self.delta_motor_t + self.last_pos_t

		motor_pos = PoseStamped()
		header.stamp = rospy.Time.now()
		motor_pos = (self.des_pos_f, self.des_pos_t)
		self.pub.publish(motor_pos)



def main(args):
	rospy.init_node('motor_position',anonymous=True)
	MP = MotorPosition()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)
