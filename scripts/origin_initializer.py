#!/usr/bin/env python

import sys
import copy
import rospy
#import moveit_commander
#import moveit_msgs.msg
import time
import math
from std_msgs.msg import Bool,Float64,String
from geometry_msgs.msg import Pose
from dynamixel_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

class Manipulation:
    def __init__(self):
        #Definition of Moters    
        self.m0_sub = rospy.Subscriber('/m0_controller/state',JointState,self.M0StateCB)
        self.m1_sub = rospy.Subscriber('/m1_controller/state',JointState,self.M1StateCB)
        self.m2_sub = rospy.Subscriber('/m2_controller/state',JointState,self.M2StateCB)
        self.m3_sub = rospy.Subscriber('/m3_controller/state',JointState,self.M3StateCB)
        self.m4_sub = rospy.Subscriber('/m4_controller/state',JointState,self.M4StateCB)
        self.m5_sub = rospy.Subscriber('/m5_controller/state',JointState,self.M5StateCB)
        #Define each moter origin
        self.m0_angle = 999.9
        self.m1_angle = 999.9
        self.m2_angle = 999.9
        self.m3_angle = 999.9
        self.m4_angle = 999.9
        self.m5_angle = 999.9
        
    def Inspection(self):
        print "m0:",self.m0_angle    
        print "m1:",self.m1_angle    
        print "m2:",self.m2_angle    
        print "m3:",self.m3_angle    
        print "m4:",self.m4_angle    
        print "m5:",self.m5_angle    

    def M0StateCB(self,state):
        self.m0_angle = state.current_pos
    def M1StateCB(self,state):
        self.m1_angle = state.current_pos
    def M2StateCB(self,state):
        self.m2_angle = state.current_pos
    def M3StateCB(self,state):
        self.m3_angle = state.current_pos
    def M4StateCB(self,state):
        self.m4_angle = state.current_pos
    def M5StateCB(self,state):
        self.m5_angle = state.current_pos

if __name__ == '__main__':
    rospy.init_node('Manipulation',anonymous=True)
    manipulation = Manipulation()
    time.sleep(1)
    manipulation.Inspection()
    
