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
        self.changing_pose_req_sub = rospy.Subscriber('/arm/changing_pose_req',String,self.ChangePoseReqCB)
        self.xyz_centroid_sub = rospy.Subscriber('/object/xyz_centroid',Point,self.GraspObjectCB)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size = 1)
        self.retry_pub = rospy.Publisher('/object/recog_req',String,queue_size = 1)
        self.grasp_res_pub = rospy.Publisher('/object/grasp_res',Bool,queue_size = 1)
        self.retry_pub = rospy.Publisher('/object/grasp_req',String,queue_size = 1)

        #Definition of Moters    
        self.m0_sub = rospy.Subscriber('/m0_controller/state',JointState,self.M0StateCB)
        self.m1_sub = rospy.Subscriber('/m1_controller/state',JointState,self.M1StateCB)
        self.m0_pub = rospy.Publisher('/m0_controller/command',Float64,queue_size = 1)
        self.m1_pub = rospy.Publisher('/m1_controller/command',Float64,queue_size = 1)
        self.m2_pub = rospy.Publisher('/m2_controller/command',Float64,queue_size = 1)
        self.m3_pub = rospy.Publisher('/m3_controller/command',Float64,queue_size = 1)
        self.m4_pub = rospy.Publisher('/m4_controller/command',Float64,queue_size = 1)
        self.m5_pub = rospy.Publisher('/m5_controller/command',Float64,queue_size = 1)
        #Define each moter origin
        self.M0_ORIGIN_ANGLE = -0.026
        self.M1_ORIGIN_ANGLE = -0.031
        self.M2_ORIGIN_ANGLE = -0.036
        self.M3_ORIGIN_ANGLE = 0.036
        self.M4_ORIGIN_ANGLE = 0.0
        self.M5_ORIGIN_ANGLE = 0.9
        
        self.m0_angle = 0
        self.m1_angle = 0

    def ChangePoseReqCB(self,cmd):
        print "command is",cmd
        if cmd.data == 'carry':
            self.m0_pub.publish(self.M0_ORIGIN_ANGLE-1.2527)
            self.m1_pub.publish(self.M1_ORIGIN_ANGLE+1.278)
            self.m2_pub.publish(self.M2_ORIGIN_ANGLE-1.67)
            self.m3_pub.publish(self.M3_ORIGIN_ANGLE+1.31)
            self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
        elif cmd.data == 'release':
            self.m5_pub.publish(self.M5_ORIGIN_ANGLE)

    def MoveBase(self,rad_speed):
        cmd = Twist()
        for speed_i in range(10):
            cmd.linear.x = rad_speed*0.05*speed_i
            cmd.angular.z = 0
            self.cmd_vel_pub.publish(cmd)
            rospy.sleep(0.1)
        for speed_i in range(10):
            cmd.linear.x = rad_speed*0.05*(10-speed_i)
            cmd.angular.z = 0
            self.cmd_vel_pub.publish(cmd)
            rospy.sleep(0.1)
        cmd.linear.x = 0
        cmd.angular.z = 0
        self.cmd_vel_pub.publish(cmd)
            
    def GraspObjectCB(self,obj_cog):
        print obj_cog
        #add for qualification video
        #obj_cog.z = obj_cog.z+0.05
        #---
        if math.isnan(obj_cog.x):
            self.MoveBase(0.3)
            retry_cmd = String()
            retry_cmd.data = 'retry'
            self.retry_pub.publish(retry_cmd)
            return
        obj_angle = math.atan2(obj_cog.y,obj_cog.x)
        print obj_angle
        if obj_angle < -0.1 or 0.1 <obj_angle:
            print "There is not object in front."
            #move kobuki
            #revolve to object
            cmd = Twist()
            cmd.linear.x = 0
            cmd.angular.z = obj_angle*3.0
            if 0 < cmd.angular.z  and cmd.angular.z < 0.4:
                cmd.angular.z = 0.4
            elif 0 > cmd.angular.z and cmd.angular.z > -0.4:
                cmd.angular.z = -0.4
            print 'cmd.angular.z is'
            print cmd.angular.z
            self.cmd_vel_pub.publish(cmd)
            time.sleep(3)
            cmd.angular.z = 0
            self.cmd_vel_pub.publish(cmd)
            retry_cmd = String()
            retry_cmd.data = 'retry'
            self.retry_pub.publish(retry_cmd)
            print "finish roll"
            return
        l1 = 0.273#mm
        l2 = 0.115
        l3 = 0.23#0.285#0.22
        x = obj_cog.x - l3
        y = obj_cog.z - 0.81#0.785
        dist = x*x+y*y
        if dist < l1*l1+l2*l2:
            self.MoveBase((dist - (l1*l1+l2*l2))*2-0.1)#
            gpsr_cmd = String()
            gpsr_cmd.data = 'retry'
            self.retry_pub.publish(gpsr_cmd)
            print "finish back"
        elif (l1+l2)*(l1+l2) < dist:
            print dist - (l1+l2)*(l1+l2)
            self.MoveBase((dist - (l1+l2)*(l1+l2))*2+0.1)
            retry_cmd = String()
            retry_cmd.data = 'retry'
            self.retry_pub.publish(retry_cmd)
            print "finish advance"
        else:
            #calculate inverse kinematicst!
            d1 = (x*x+y*y+l1*l1-l2*l2)/(2*l1)
            d2 = (x*x+y*y-l1*l1+l2*l2)/(2*l2)
            joint0_angle = 0
            joint1_angle = 0
            joint2_angle = 0
            singlarity_flg = False
            
            try:
                joint0_angle = -1*math.atan2(y,x)-math.atan2(math.sqrt(x*x+y*y-d1*d1),d1)
                joint1_angle = -1*math.atan2(math.sqrt(x*x+y*y-d1*d1),d1)+math.atan2(math.sqrt(x*x+y*y-d2*d2),d2)
                joint2_angle = -1*(joint0_angle + joint1_angle)
                print joint0_angle,joint1_angle,joint2_angle
                
            except:
                print 'the end points is singlarity'
                singlarity_flg = True
            if joint0_angle < -1.57 or 1.57 < joint0_angle:
                singlarity_flg = True
            if joint1_angle < -1.57 or 1.57 < joint1_angle:
                singlarity_flg = True
            if joint2_angle < -1.57 or 1.57 < joint2_angle:
                singlarity_flg = True
            if singlarity_flg == True:
                print 'I can not move arm'
                return
            self.MoveBase(-1.0)#0.7
            self.m0_pub.publish(joint0_angle+self.M0_ORIGIN_ANGLE)
            self.m1_pub.publish(-1*joint0_angle+self.M1_ORIGIN_ANGLE)
            self.m2_pub.publish(joint1_angle+self.M2_ORIGIN_ANGLE)
            #self.m2_pub.publish(-1*joint1_angle+self.M2_ORIGIN_ANGLE)
            #self.m3_pub.publish(joint2_angle+self.M3_ORIGIN_ANGLE)
            self.m3_pub.publish(joint2_angle+self.M3_ORIGIN_ANGLE)
            self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
            self.m5_pub.publish(self.M5_ORIGIN_ANGLE)
            for i in range(0,10):
                double_moters_diff = self.m0_angle+self.m1_angle
                print double_moters_diff
                self.m0_pub.publish(-1*joint0_angle+double_moters_diff+self.M0_ORIGIN_ANGLE)
                self.m1_pub.publish(joint0_angle+self.M1_ORIGIN_ANGLE)
                #self.m2_pub.publish(-1*joint1_angle+self.M2_ORIGIN_ANGLE)
                self.m2_pub.publish(joint1_angle+self.M2_ORIGIN_ANGLE)
                self.m3_pub.publish(-1*joint2_angle+self.M3_ORIGIN_ANGLE)
                #self.m3_pub.publish(joint2_angle+self.M3_ORIGIN_ANGLE)
                time.sleep(0.3)
            time.sleep(0.3)
            print "finish"
            self.MoveBase(1.1)
            #self.m5_pub.publish(self.M5_ORIGIN_ANGLE-0.95)
            self.m5_pub.publish(self.M5_ORIGIN_ANGLE-0.9)
            rospy.sleep(3)
            self.MoveBase(-0.7)
            self.grasp_res_pub.publish(True)
            
    def M0StateCB(self,state):
        self.m0_angle = state.current_pos
        #print "diff = ",self.m0_angle+self.m1_angle
    def M1StateCB(self,state):
        self.m1_angle = state.current_pos

if __name__ == '__main__':
    rospy.init_node('Manipulation',anonymous=True)
    manipulation = Manipulation()
    rospy.spin()
