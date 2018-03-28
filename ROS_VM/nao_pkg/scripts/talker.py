#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64



import sys
import time

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

import argparse
import motion
import almath
import math
import time 

from functools import partial
import numpy as np
from optparse import OptionParser

NAO_IP = "nao.local"


# Global variable to store the HumanGreeter module instance
memory = None


class naoTalker():
    """ A simple module able to react
    to facedetection events

    """
    def __init__(self):

    	self.connectNaoQi()

        self.motion   = ALProxy("ALMotion")        
        self.posture  = ALProxy("ALRobotPosture")        
        self.tts      = ALProxy("ALTextToSpeech")
        global memory
        memory = ALProxy("ALMemory")
        
        #self.tts.say('ball found')

        pub = rospy.Publisher('nao_chatter', Float64, queue_size=10)
        rospy.init_node('nao_talker', anonymous=True)
        rate = rospy.Rate(10)
        names="RShoulderPitch"
        useSensors=False
        
        
        while not rospy.is_shutdown():        	
        	commandAngles=self.motion.getAngles(names,useSensors)
        	print type(commandAngles)
        	print commandAngles[0]
        	rospy.loginfo(commandAngles[0])
        	#angle = float(commandAngles[0])
        	pub.publish(commandAngles[0])
	        rate.sleep()
		
		


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
	    	(opts, args_) = parser.parse_args()
	    	#pip   = opts.pip
	    	#pport = opts.pport

	    	myBroker = ALBroker("myBroker","0.0.0.0",0,self.pip, self.pport)
    	
        	
    	except KeyboardInterrupt:
	    	print "Interrupted by user, shutting down"
	        myBroker.shutdown()
	        sys.exit(0)



       


if __name__ == "__main__":
    try:
        nao = naoTalker()
    except rospy.ROSInterruptException:
        pass
