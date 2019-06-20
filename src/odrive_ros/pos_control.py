#!/usr/bin/env python

#basics
import rospy
roslib.load_manifest('odrive_tutorial')

#load all the message types you need to publish or subscribe to
from std_msgs.msg import String
from std_msgs.msg import Float32Stamped
from std_msgs.msg import Int32

#import any relevant python packages for you (numpy, opencv, etc.)
import numpy as np



class pos_control:
	def __init__(self):

		#if you need parameters, use the following
		#self.mything = rospy.get_param('param_name',default_value)

		#now set up any subscribers
		# desired position from tF_des topic (theta femur) published by ____(node TBD)__
		self.sub_des = rospy.Subscriber("tF_des",Float32Stamped,self.des_callback)
		# actual position from /encoder_right topic (encoder position) published by odrive_node.py
		self.sub_act = rospy.Subscriber("odrive/raw_odom/encoder_right", Int32, self.act_callback)
		
		# Define class-owned variables
		self.des_pos = None
		self.act_pos = None
		self.k = 0.5  # damping
		
		#now set up your publisher
		#this publisher will publish on a timer.
		self.pub = rospy.Publisher("/cmd_vel",Float32Stamped,queue_size=1)

		#now set up a timed loop
		rospy.Timer(rospy.Duration(0.01),self.timercallback,oneshot=False)


	def des_callback(self,data):
		#right now, this time is LOCAL. I can't access it from another function in the class.
		time_this_happened = data.header.stamp
		
		# Assign subscribed desired position value to class variable
		self.des_pos = data.data 

	def act_callback(self,data):
		# Assign subscribed actual position value to class variable
		self.act_pos = data.data
		

	def timercallback(self,data):
		# Calculate velocity, u
		u = k * (self.des_pos-self.act_pos)
		
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


