#!/usr/bin/env python

# Author: Gabrielle Conard, July 2019
# File: motor_position.py
# This python script takes the leg angles from inverse_kinematics...
# ...and finds the positions of the two motors required to achieve those angles.

#basics
import rospy
import sys
import roslib
roslib.load_manifest('odrive_ros')


import tf.transformations
import tf_conversions
import tf2_ros

# Imports message types and services from several libraries
from std_msgs.msg import  Float64, Int32   #,Float64Stamped, Int32Stamped,
from geometry_msgs.msg import TwistStamped, TransformStamped, Pose #PoseStamped
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
		# self.sub_F = rospy.Subscriber("/theta_f", Float64Stamped, femur_motor_callback)
		# self.sub_T = rospy.Subscriber("/theta_t", Float64Stamped, tibia_motor_callback)
		self.sub_F = rospy.Subscriber("/theta_f", Float64, self.femur_motor_callback)
		self.sub_T = rospy.Subscriber("/theta_t", Float64, self.tibia_motor_callback)

		# actual position (counts) from /encoder_right topic (encoder position) published by odrive_node.py
		self.sub_init_t = rospy.Subscriber("odrive/raw_odom/encoder_right", Int32, self.init_t_callback)
		self.sub_init_f = rospy.Subscriber("odrive/raw_odom/encoder_left", Int32, self.init_f_callback)


		#publish motor positions
		# How do I get both in one publisher to output a Pose msg?
		# Do I set up a timer?
		# self.pub = rospy.Publisher("/cmd_pos", PoseStamped, queue_size = 1)
		self.pub = rospy.Publisher("/cmd_pos", Pose, queue_size = 1)


		# Set up a timed loop
		# rospy.Timer(rospy.Duration(0.01), self.timer_callback, oneshot=False)

		# Class variables
		# Measured Values:
		# All lengths are in inches and angles are in degrees
		self.constraint_H = 4.047
		self.link_H = 7.047
		self.mount_H = 2.294

		self.constraint_K = 3.739
		self.link_K = 7.047
		self.mount_K = 2.177

		# Full length of both ball screws is 180 mm (7.087 in)...
		# ...but ball nuts can't really travel the full range.  
		# Otherwise, things will crash into each other (eg: the links will run into the encoder mounts).
		# These effective lengths were measured empirically.
		self.ball_screw_H = 5
		self.ball_screw_K = 4.6063

		# To be received from subscribed topics
		self.theta_f = None
		self.theta_t = None

		self.init_motor_f = None
		self.init_motor_t = None

		# To be calculated...
		self.theta_HLN = None
		self.theta_HNL = None
		self.theta_KLN = None
		self.theta_KNL = None

		self.length_HBN = None
		self.length_KBN = None

		self.des_BN_f = None
		self.last_BN_f = 0
		self.delta_BN_f = None
		self.des_BN_t = None
		self.last_BN_t = 0
		self.delta_BN_t = None

		self.des_pos_f = None
		self.last_pos_f = 0
		self.delta_motor_f = None
		self.des_pos_t = None
		self.last_pos_t = 0
		self.delta_motor_t = None


		# Conversion
		self.mm_to_in = 1 / 25.4   # 1 in = 25.4 mm
		self.distance_to_motor_pos = (2.2114 / (self.mm_to_in * 5)) * (8192)   # 5 mm = 2.2114 rev = 2.2114 * 8192 counts
		self.rev_to_count = 8192
		self.deg_to_count = 8192 / 360   # 1 revolution = 360 degrees = 8192 counts

	def init_f_callback(self, data):
		if(self.init_motor_f is None):
			self.init_motor_f = data.data
			self.last_pos_f = self.init_motor_f
		else:
			pass


	def init_t_callback(self, data):
		if(self.init_motor_t is None):
			self.init_motor_t = data.data
			self.last_pos_t = self.init_motor_t
		else:
			pass



	def femur_motor_callback(self, data):
		#Femer angle is calculated from Inverse Kinematics code, zero is when the hip is tucked 
		self.theta_f = data.data 
		print(self.theta_f)

		#Motor will not run if no data has been fed. 
		if(self.init_motor_f is not None):
			#motor will not run if theta_f does not have a value
			if(self.theta_f is not None):
				print("Received theta_f!")
				
				#calculating angle between ball screw and link
				self.theta_HNL = arcsin((self.constraint_H * sin(self.theta_f)) / self.link_H)
				#angle between the constarint to linkage rod
				self.theta_HLN = pi - self.theta_f - self.theta_HNL

				#length from hip to ball nut
				self.length_HBN = ((self.link_H * sin(self.theta_HLN)) / sin(self.theta_f)) 
				# desired change in length (should be positive) of the ball screw
				self.des_BN_f = self.ball_screw_H - (self.length_HBN - self.mount_H)
				#To Do: add a check, make sure this is positive. Crash if negative.

				#
				#self.delta_BN_f = self.des_BN_f - self.last_BN_f
				#print(self.delta_BN_f)

				# Reset value of last ball nut positon
				# Do we want real feedback?
				#self.last_BN_f = self.des_BN_f
			else:
				pass
		else:
			pass

	def tibia_motor_callback(self, data):
		if(self.init_motor_t is not None):
			if(self.theta_f is not None):
				print("Received theta_f and theta_t!")
				self.theta_t = data.data
				print("theta_t = " + str(self.theta_t))

				self.theta_KNL = arcsin((self.constraint_K * sin(self.theta_t)) / self.link_K)
				print("theta_KNL = " + str(self.theta_KNL))
				self.theta_KLN = pi - self.theta_KNL - self.theta_t
				print("theta_KLN = " + str(self.theta_KNL))

				self.length_KBN = ((self.link_K * sin(self.theta_KLN)) / sin(self.theta_t)) - self.mount_K
				print("length_KBN = " + str(self.length_KBN))
				# self.des_BN_t = self.ball_screw_K - self.length_KBN
				self.des_BN_t = self.length_KBN
				#self.delta_BN_t = self.des_BN_t - self.last_BN_t
				#print("delta_BN_t = " + str(self.delta_BN_t))

				# Reset value of last ball nut positon
				# Do we want real feedback?
				#self.last_BN_t = self.des_BN_t

				# Finds motor positions based on ball nut positions
				self.delta_motor_f = self.des_BN_f * self.distance_to_motor_pos

				self.des_pos_f = self.delta_motor_f #+ self.last_pos_f

				self.delta_motor_t = self.des_BN_t * self.distance_to_motor_pos
				print("delta_motor_t = " + str(self.delta_motor_t))
				self.des_pos_t = self.delta_motor_t #+ self.last_pos_t

				# motor_pos = PoseStamped()
				# motor_pos.header.stamp = rospy.Time.now()
				motor_pos = Pose()
				motor_pos.position.x = self.des_pos_t
				motor_pos.position.y = self.des_pos_f   # Order depends on ODrive set-up
				motor_pos.position.z = 0.0
				motor_pos.orientation.x = 0.0
				motor_pos.orientation.y = 0.0
				motor_pos.orientation.z = 0.0
				motor_pos.orientation.w = 0.0
				self.pub.publish(motor_pos)
				print("Motor Positions: " + str(motor_pos))

				# Reset value of last motor positons
				# Do we want real feedback?
				self.last_pos_f = self.des_pos_f
				self.last_pos_t = self.des_pos_t
			else:
				pass
		else:
			pass


	# function that takes these two distances and converts them into motor positions
	# need a self.last_pos_f too so that motor knows where it needs to turn to
	# consider direction (cw or ccw)
	# def timer_callback(self, data):
	# 	if(self.delta_BN_f and self.delta_BN_t is not None):
	# 		self.delta_motor_f = self.delta_BN_f * self.distance_to_motor_pos
	# 		self.des_pos_f = self.delta_motor_f + self.last_pos_f

	# 		self.delta_motor_t = self.delta_BN_t * self.distance_to_motor_pos
	# 		self.des_pos_t = self.delta_motor_t + self.last_pos_t

	# 		motor_pos = PoseStamped()
	# 		header.stamp = rospy.Time.now()
	# 		motor_pos = (self.des_pos_f, self.des_pos_t)
	# 		self.pub.publish(motor_pos)

	# 		# Reset value of last motor positons
	# 		# Do we want real feedback?
	# 		self.last_pos_f = self.des_pos_f
	# 		self.last_pos_t = self.des_pos_t
	# 	else:
	# 		pass




def main(args):
	rospy.init_node('motor_position',anonymous=True)
	MP = MotorPosition()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)
