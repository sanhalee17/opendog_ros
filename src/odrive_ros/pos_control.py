#!/usr/bin/env python

#basics
import rospy
import sys
import roslib
roslib.load_manifest('odrive_ros')

#load all the message types you need to publish or subscribe to
from std_msgs.msg import String
from std_msgs.msg import Float32  # std_msgs don't have stamped? http://answers.ros.org/question/9715/stamped-std_msgs/
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

#import any relevant python packages for you (numpy, opencv, etc.)
import numpy as np



class pos_control:
	def __init__(self):

		#if you need parameters, use the following
		#self.mything = rospy.get_param('param_name',default_value)

		#now set up any subscribers
		# desired position (degrees) from tF_des topic (theta femur) published by ____(node TBD)__
		self.sub_des = rospy.Subscriber("tF_des",Float32,self.des_callback)
		# actual position (counts) from /encoder_right topic (encoder position) published by odrive_node.py
		self.sub_act = rospy.Subscriber("odrive/raw_odom/encoder_right", Int32, self.act_callback)
		
		# Define class-owned variables
		self.des_pos = None
		self.act_pos = None
		self.k = 0.5  # damping
		self.deg_to_rad = np.pi / 180
		self.rad_to_deg = 180 / np.pi
		self.count_to_rad = (2 * np.pi) / 8192
		self.rad_to_count = 8192 / (2 * np.pi)
		
		#now set up your publisher
		#this publisher will publish on a timer.
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

		#now set up a timed loop
		rospy.Timer(rospy.Duration(0.01), self.timercallback, oneshot=False)


	def des_callback(self,data):
		#right now, this time is LOCAL. I can't access it from another function in the class.
		#time_this_happened = data.header.stamp
		
		# Assign subscribed desired position value to class variable, convert to radians
		self.des_pos = data.data * self.deg_to_rad

	def act_callback(self,data):
		# Assign subscribed actual position value to class variable, convert to radians
		self.act_pos = data.data * self.count_to_rad
		#print(self.act_pos)   # Debugging
		

	def timercallback(self,data):
		# Calculate velocity, u
		u = self.k * (self.des_pos-self.act_pos)
		#print(u)  # Debugging

		# Create ouput message type and publish it
		output = Twist()
		# output.header.stamp = rospy.Time.now()
		output.angular.z = u
		self.pub.publish(output)

def main(args):
	rospy.init_node('pos_control',anonymous=True)
	PS = pos_control()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)


