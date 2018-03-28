#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64MultiArray
from nao_pkg.msg import Float64Array, HeadPitch
from geometry_msgs.msg import Vector3

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.x)
    
def listener():

    rospy.init_node('nao_movement_listener', anonymous=True)
    rospy.Subscriber("coordinates", Vector3, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
