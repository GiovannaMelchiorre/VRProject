#!/usr/bin/env python

import rospy
import random

import xml.dom.minidom
from math import pi
from threading import Thread
from functools import partial
import numpy as np
from optparse import OptionParser
import sys
import signal
import math
import json
import argparse
import almath
import motion
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

from std_msgs.msg import String
from geometry_msgs.msg import Twist


NAO_IP = "nao.local"
# Global variable to store the HumanGreeter module instance
memory = None

RANGE = 10000


def get_param(name, value=None):
	private = "~%s" % name
	if rospy.has_param(private):
		return rospy.get_param(private)
	elif rospy.has_param(name):
		return rospy.get_param(name)
	else:
		return value


class naoListenMovement():
   

	def __init__(self):
		
		self.connectNaoQi()
		self.initModules()
		
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

		msg = Twist()
		msg.linear = [0.0, 0.0, 0.0]
		msg.angular = [0.0, 0.0, 0.0]

		self.pub.publish(msg)


	def loop(self):
		names  = 'Body'
		stiffnesses  = 1.0
		self.motion.setStiffnesses(names, stiffnesses)
		hz = get_param("rate", 1)  # 10hz
		r = rospy.Rate(hz)
		name            = "Torso"
		frame           = motion.FRAME_WORLD
		useSensorValues = True
		initialPosition = self.motion.getPosition(name, frame, useSensorValues)
		previous = initialPosition
		msg = Twist()

		while not rospy.is_shutdown():
			   
			result = self.motion.getPosition(name, frame, useSensorValues)
			if self.compaire(result, previous):
				
				newPosition = []
				
				
				for i in range(len(result)):
					newPosition.append(result[i]-previous[i])		
				
				
				msg.linear.x = newPosition[0]
				msg.linear.y = newPosition[1]
				msg.linear.z = newPosition[2]
				msg.angular.x = newPosition[3]
				msg.angular.y = newPosition[4]
				msg.angular.z = newPosition[5]
				
				self.pub.publish(msg)
				previous = result


				try:
					r.sleep()
				except rospy.exceptions.ROSTimeMovedBackwardsException:
					pass
	

	def compaire(self, val1, val2):
		different = False
		for i in range(0, len(val1)-1):
			if val1[i] != val2[i]:
				different = True
				return different
		return different


	
	def initModules(self):
		self.motion   = ALProxy("ALMotion")        
		self.posture  = ALProxy("ALRobotPosture")        




	def connectNaoQi(self):
		try:
			parser = OptionParser()
			parser.add_option("--pip",
				help="Parent broker port. The IP address or your robot",
				dest="pip")
			parser.add_option("--pport",
				help="Parent broker port. The port NAOqi is listening to",
				dest="pport",
				type="int")
			parser.set_defaults(
				pip="127.0.0.1",
				pport=9559)
			parser.add_option("--robot_description",
				help="Parent broker port. The IP address or your robot",
				dest="name")
			(opts, args_) = parser.parse_args()
			self.pip   = get_param('pip', None)
			self.pport = get_param('pport', None)
			self.description = opts.name
			myBroker = ALBroker("myBroker","0.0.0.0",0,self.pip, self.pport)
		
			
		except KeyboardInterrupt:
			print "Interrupted by user, shutting down"
			myBroker.shutdown()
			sys.exit(0)




if __name__ == '__main__':
	try:
		rospy.init_node('nao_listen_movement', anonymous=True)
		nao = naoListenMovement()
		nao.loop()
		

	except rospy.ROSInterruptException:
		pass
