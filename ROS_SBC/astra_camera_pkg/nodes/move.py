#!/usr/bin/env python


import rospy
import time
import redis
from geometry_msgs.msg import Twist
import json
import math
import subprocess
import sys
import os
import signal
import threading


def move():
    R = redis.Redis()
    pubsub = R.pubsub()
    pubsub.subscribe('KOBUKI:DESTINATION')
    
    pubROS = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)
    rospy.init_node('coordinates_kobuki_proxy', anonymous=True)
    rate = rospy.Rate(1)
    message=pubsub.get_message()
    position_msg = Twist()
    rotation_msg = Twist()
    k=2.0
    
    while not rospy.is_shutdown():
                	
		message = pubsub.get_message()
		if message:
			msg = json.loads(message['data'])
	        	x = 2*msg['linear_x']
			theta = msg['angular_z']
		
			rotation_msg.linear.x = 0
			rotation_msg.linear.y = 0 
			rotation_msg.linear.z = 0
			rotation_msg.angular.x = 0
			rotation_msg.angular.y = 0
			rotation_msg.angular.z = 2*theta
			pubROS.publish(rotation_msg)
			rospy.sleep(1)
			rospy.loginfo(x/0.15)
			p=subprocess.Popen(['/bin/bash','/home/ubuntu/catkin_ws/src/astra_camera_pkg/nodes/velocity.sh'],shell=False)
			rospy.sleep(x/0.15)
			os.kill(p.pid+1, signal.SIGTERM)
			
			position_msg.linear.x = 0
			position_msg.linear.y = 0 
			position_msg.linear.z = 0
			position_msg.angular.x = 0
			position_msg.angular.y = 0
			position_msg.angular.z = 0
			pubROS.publish(position_msg)

			rate.sleep()
			    




if __name__ == '__main__':
    try:

        move()
    except rospy.ROSInterruptException:
        pass
