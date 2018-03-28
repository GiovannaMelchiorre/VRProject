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
from geometry_msgs.msg import Vector3


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


class naoMovementListener():
   

	def __init__(self):
		
		self.connectNaoQi()
		self.initModules()
		
	def callback(self, data):
		x = data.x
		y = data.y
		z = data.z
		theta = math.acos(x/y)
		self.motion.moveTo(x, y, theta)



	def listen(self):
		rospy.init_node('nao_movement_listener', anonymous=True)
		rospy.Subscriber("coordinates", Vector3, callback)
    		rospy.spin()
		


	


	def initModules(self):
		global memory
		self.motion   = ALProxy("ALMotion")        
		self.posture  = ALProxy("ALRobotPosture")        
		self.tts      = ALProxy("ALTextToSpeech")
		memory = ALProxy("ALMemory")


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
		
		nao = naoMovementListener()
		nao.listen()
		

	except rospy.ROSInterruptException:
		pass
