#!/usr/bin/env python

import rospy
import random
import sys
import signal
import redis

import math
import json

from sensor_msgs.msg import PointCloud, Image
from std_msgs.msg import String
import numpy as np
import pylab
from cv_bridge import CvBridge, CvBridgeError
import cv2
from matplotlib import pyplot as plt
 
import message_filters
from sensor_msgs.msg import Image, CameraInfo

refresh_rate=30
contatore = 0

class pointsManager():
	
	def __init__(self):
		rospy.loginfo("nodo inizial")
		#self.pub = rospy.Publisher('string_depth', String, queue_size=5)
		self.rpub =redis.Redis()		
		self.bridge = CvBridge()
		
	
	def listener(self):				
		rospy.Subscriber("/camera/depth/image", Image, self.analysis,queue_size=1)
		rospy.spin()

     		
	def analysis(self,data):
		global contatore
		global refresh_rate
		if(contatore % refresh_rate == 0):
			#pointsStr = String()

			try:
			    cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
			    depth_array = np.array(cv_image, dtype = np.float32)
			    cv2.waitKey(3)
			except CvBridgeError as e:
			    print(e)

			intervallo = 64
			array_width = np.arange(0,640,intervallo)
			passo=0.2
			array_depth = np.arange(0.4, 2.0, passo)
			matrice = []

			for i in range(len(array_depth)):
				mask = cv2.inRange(depth_array, array_depth[i], array_depth[i]+passo)	
				punti = []
				for j in range(len(array_width)):
					punti.append( np.count_nonzero(mask[:,array_width[j]:array_width[j]+intervallo], axis=0))
				matrice.append(punti)

			msg_str = ''
			for i in range(np.shape(matrice)[0]):
				for j in range(np.shape(matrice)[1]):
					if(matrice[i][j]>20):
						coor_x_frame = j * intervallo
						coor_x_spazio = ((coor_x_frame - 320) / intervallo) * 0.15
						#rospy.loginfo(coor_x_spazio)						
						coor_z_spazio = array_depth[i]
						msg_str += str(coor_x_spazio)+ ',' + str(coor_z_spazio) + '!'
			

			#rospy.loginfo(msg_str)		
			#pointsStr.data = msg_str
			#self.pub.publish(pointsStr)
			self.rpub.publish('ASTRACAM:DEPTH', msg_str)
			cv2.waitKey(3)

		contatore += 1



if __name__ == '__main__':
	try:
		
	    points = pointsManager()
	    rospy.init_node('listenerPC2', anonymous=True)
	    points.listener()
		

	except rospy.ROSInterruptException:
	    pass
