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

import tf
import tf.transformations
import tf_conversions
import tf2_ros

# Imports message types and services from several libraries
from std_msgs.msg import  Float64, Int32   #,Float64Stamped, Int32Stamped,
from geometry_msgs.msg import TwistStamped, TransformStamped, Pose #PoseStamped
import std_srvs.srv
from visualization_msgs.msg import Marker

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



		#publish motor positions
		# How do I get both in one publisher to output a Pose msg?
		# Do I set up a timer?
		# self.pub = rospy.Publisher("/cmd_pos", PoseStamped, queue_size = 1)
		self.femur_pub = rospy.Publisher("/femur_marker", Marker, queue_size = 1)
		self.tibia_pub = rospy.Publisher("/tibia_marker", Marker, queue_size = 1)


		# Set up a timed loop
		rospy.Timer(rospy.Duration(0.1), self.timer_callback, oneshot=False)

		# Class variables
		self.br = tf.TransformBroadcaster()

		# Measured Values:
		# All lengths are in inches and angles are in radians
		self.theta_K_shift = 16.6*(pi/180)
		self.theta_HKP_shift = 1*(pi/180)
		self.theta_H = 16.9*(pi/180)
		self.theta_t_shift = 15.8*(pi/180)

		# To be received from subscribed topics
		self.theta_f = 0
		self.theta_t = 0

		self.theta_f_prime = 0
		self.theta_t_prime = 0


		



	def femur_motor_callback(self, data):
		self.theta_f = data.data
		self.theta_f_prime = pi - self.theta_f + self.theta_H   #TODO: Fill in theta_H (constant offset)

		

	def tibia_motor_callback(self, data):
		self.theta_t = data.data
		# Need to twist back towards world x-axis (so rotate negatively)
		self.theta_t_prime = -(pi - self.theta_t - self.theta_t_shift)  #TODO: Fill in theta_t_shift (constant offset from femur)
		
	def timer_callback(self, data):
 		self.br.sendTransform((0, 0, -5),
				         tf.transformations.quaternion_from_euler(0, 0, self.theta_f_prime),
				         rospy.Time.now(),
				         'femur',
				         "world")
 		femur = Marker()
		femur.header.frame_id = "femur";
		femur.header.stamp = rospy.Time.now();
		# femur.ns = "my_namespace";
		# femur.id = 0;
		femur.type = femur.CUBE;
		femur.action = femur.MODIFY;
		femur.pose.position.x = 6.5;
		femur.pose.position.y = -2.5;
		femur.pose.position.z = 0;
		femur.pose.orientation.x = 0.0;
		femur.pose.orientation.y = 0.0;
		femur.pose.orientation.z = 0.0;
		femur.pose.orientation.w = 1.0;
		femur.scale.x = 16;
		femur.scale.y = 0.875;
		femur.scale.z = 2.5;
		femur.color.a = 1.0; # Don't forget to set the alpha!
		femur.color.r = 0.0;
		femur.color.g = 1.0;
		femur.color.b = 0.0;
		#only if using a MESH_RESOURCE femur type:
		# femur.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		self.femur_pub.publish( femur );

		# offset 1.625, 3/4 up, length 13.25, width 1.75, thickness 0.875
		self.br.sendTransform((13.5625, -4, 0),
				         tf.transformations.quaternion_from_euler(0, 0, self.theta_t_prime),
				         rospy.Time.now(),
				         'tibia',
				         "femur")
 		tibia = Marker()
		tibia.header.frame_id = "tibia";
		tibia.header.stamp = rospy.Time.now();
		# tibia.ns = "my_namespace";
		# tibia.id = 0;
		tibia.type = tibia.CUBE;
		tibia.action = tibia.MODIFY;
		tibia.pose.position.x = 6.625;
		tibia.pose.position.y = 1.1875;
		tibia.pose.position.z = 0;
		tibia.pose.orientation.x = 0.0;
		tibia.pose.orientation.y = 0.0;
		tibia.pose.orientation.z = 0.0;
		tibia.pose.orientation.w = 1.0;
		tibia.scale.x = 13.25;
		tibia.scale.y = 0.875;
		tibia.scale.z = 1.5;
		tibia.color.a = 1.0; # Don't forget to set the alpha!
		tibia.color.r = 0.0;
		tibia.color.g = 1.0;
		tibia.color.b = 0.0;
		#only if using a MESH_RESOURCE tibia type:
		# tibia.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		self.tibia_pub.publish( tibia );




def main(args):
	rospy.init_node('motor_position',anonymous=True)
	MP = MotorPosition()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)
