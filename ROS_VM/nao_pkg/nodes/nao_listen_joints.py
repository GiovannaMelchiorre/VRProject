#!/usr/bin/env python

import rospy
import random
import xml.dom.minidom
import sys
import signal
import math
import json
import argparse
import motion
import almath
from math import pi
from threading import Thread

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState

from functools import partial
import numpy as np
from optparse import OptionParser



def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value


class naoListenJoints():

    def __init__(self):
        description = get_param('robot_description')
        
        self.free_joints = {}
        self.joint_list = [] 
        self.dependent_joints = get_param("dependent_joints", {})
        self.use_mimic = get_param('use_mimic_tags', True)
        self.use_small = get_param('use_smallest_joint_limits', True)

        self.zeros = get_param("zeros")

        self.pub_def_positions = get_param("publish_default_positions", True)
        self.pub_def_vels = get_param("publish_default_velocities", False)
        self.pub_def_efforts = get_param("publish_default_efforts", False)

        self.connectNaoQi()
        self.initModules()

        robot = xml.dom.minidom.parseString(description)
        if robot.getElementsByTagName('COLLADA'):
            self.read_collada(robot)
        else:
            self.read_urdf(robot)

        
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=5)


    def read_collada(self, robot):
        robot = robot.getElementsByTagName('kinematics_model')[0].getElementsByTagName('technique_common')[0]
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                name = child.getAttribute('name')
                if child.getElementsByTagName('revolute'):
                    joint = child.getElementsByTagName('revolute')[0]
                else:
                    rospy.logwarn("Unknown joint type %s", child)
                    continue

                if joint:
                    limit = joint.getElementsByTagName('limits')[0]
                    minval = float(limit.getElementsByTagName('min')[0].childNodes[0].nodeValue)
                    maxval = float(limit.getElementsByTagName('max')[0].childNodes[0].nodeValue)
                if minval == maxval:  # this is fixed joint
                    continue

                self.joint_list.append(name)
                joint = {'min':minval*pi/180.0, 'max':maxval*pi/180.0, 'zero':0, 'position':0, 'velocity':0, 'effort':0}
                self.free_joints[name] = joint

    def read_urdf(self, robot):
        robot = robot.getElementsByTagName('robot')[0]
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed' or jtype == 'floating':
                    continue
                name = child.getAttribute('name')
                #limits = self.motion.getLimits(name)
                self.joint_list.append(name)
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    
                    try:
                        #questa parte puo essere sostituita con la funzione 
                        limit = child.getElementsByTagName('limit')[0]
                        minval = float(limit.getAttribute('lower'))
                        maxval = float(limit.getAttribute('upper'))

                    except:
                        rospy.logwarn("%s is not fixed, nor continuous, but limits are not specified!" % name)
                        continue
              
                mimic_tags = child.getElementsByTagName('mimic')
                if self.use_mimic and len(mimic_tags) == 1:
                    tag = mimic_tags[0]
                    entry = {'parent': tag.getAttribute('joint')}
                    if tag.hasAttribute('multiplier'):
                        entry['factor'] = float(tag.getAttribute('multiplier'))
                    if tag.hasAttribute('offset'):
                        entry['offset'] = float(tag.getAttribute('offset'))

                    self.dependent_joints[name] = entry
                    continue

                if name in self.dependent_joints:
                    continue

                if self.zeros and name in self.zeros:
                    zeroval = self.zeros[name]
                elif minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min': minval, 'max': maxval, 'zero': zeroval}
                if self.pub_def_positions:
                    joint['position'] = zeroval
                if self.pub_def_vels:
                    joint['velocity'] = 0.0
                if self.pub_def_efforts:
                    joint['effort'] = 0.0

                if jtype == 'continuous':
                    joint['continuous'] = True
                self.free_joints[name] = joint




    def loop(self):
        hz = get_param("rate", 10)  # 10hz
        r = rospy.Rate(hz)

        delta = get_param("delta", 0.0)

        nomiGiunti = self.motion.getBodyNames("Body")
        nomiGiuntiExtra = ['RFinger13', 'RFinger12', 'LFinger21', 'LFinger13','LFinger11', 
                    'RFinger22', 'LFinger22', 'RFinger21', 'LFinger12', 'RFinger23', 'RFinger11', 
                    'LFinger23', 'LThumb1', 'RThumb1', 'RThumb2', 'LThumb2']
        nomiGiuntiCompleti = nomiGiunti + nomiGiuntiExtra

        statoGiunti = self.motion.getAngles("Body", False)
        statoGiuntiExtra = len(nomiGiuntiExtra) * [0.0]
        statoGiuntiCompleti = statoGiunti + statoGiuntiExtra
        
        self.reorder(nomiGiuntiCompleti, statoGiuntiCompleti)
        statoPrecedenteGiuntiCompleti = statoGiuntiCompleti
        
        # Publish Joint States
        while not rospy.is_shutdown():
            
            statoGiunti = self.motion.getAngles("Body", False)
            statoGiuntiExtra = len(nomiGiuntiExtra) * [0.0]
            statoGiuntiCompleti = statoGiunti + statoGiuntiExtra
            statoGiuntiCompleti = self.reorder(nomiGiuntiCompleti, statoGiuntiCompleti)

            
            if self.compaire(statoGiuntiCompleti, statoPrecedenteGiuntiCompleti):

                msg = JointState()
                msg.header.stamp = rospy.Time.now()

                statoPrecedenteGiuntiCompleti = statoGiuntiCompleti
                msg.name = self.reorderForUnity(self.joint_list) 
                msg.position = self.reorderForUnity(statoGiuntiCompleti) 
                msg.velocity = len(self.joint_list) * [0.0]
                msg.effort = len(self.joint_list) * [0.0]

                self.pub.publish(msg)

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


    def update(self, delta):
        for name, joint in self.free_joints.iteritems():
            forward = joint.get('forward', True)
            if forward:
                joint['position'] += delta
                if joint['position'] > joint['max']:
                    if joint.get('continuous', False):
                        joint['position'] = joint['min']
                    else:
                        joint['position'] = joint['max']
                        joint['forward'] = not forward
            else:
                joint['position'] -= delta
                if joint['position'] < joint['min']:
                    joint['position'] = joint['min']
                    joint['forward'] = not forward


    def reorder(self, nomiGiuntiCompleti, statoGiuntiCompleti):
        #riordinare statoGiuntiCompleti, associati a nomiGiuntiCompleti, in base a joint_list
        #ritorna lo stato ordinato e completo
        jointsList = []
        new_position=[]
        for i in range(0, len(self.joint_list)):           
            indice = nomiGiuntiCompleti.index(self.joint_list[i])
            jointsList.append(statoGiuntiCompleti[indice])

       
        return jointsList


    def reorderForUnity(self, val):
        newValues=[]
        for i in range(0,20):
            newValues.append(val[i])
        newValues.append(val[28])
        newValues.append(val[32])
        newValues.append(val[37])
        newValues.append(val[30])
        newValues.append(val[34])
        newValues.append(val[29])
        newValues.append(val[38])
        newValues.append(val[41])
        newValues.append(val[20])
        newValues.append(val[21])
        newValues.append(val[22])
        newValues.append(val[23])
        newValues.append(val[24])
        newValues.append(val[25])
        newValues.append(val[33])
        newValues.append(val[31])
        newValues.append(val[35])
        newValues.append(val[36])
        newValues.append(val[27])
        newValues.append(val[26])
        newValues.append(val[39])
        newValues.append(val[40])
        return newValues

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
        rospy.init_node('nao_listen_joints', anonymous=True)
        nao = naoListenJoints()
        nao.loop()
        

    except rospy.ROSInterruptException:
        pass
