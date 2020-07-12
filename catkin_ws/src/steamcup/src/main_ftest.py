#!/usr/bin/env python
import rospy
import operator
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16,Float64,UInt8
from sensor_msgs.msg import Joy
from PID import *
#import rospy
import numpy as np
from std_msgs.msg import Float64,UInt8,Int64
#from geometry_msgs.msg import Twist

x=0
y=2
tickgreen=1
twist = Twist()
twist2 = Twist() 
twist3 = Twist()
joycontrol=0
speed=0
#feedback = 50.0
outputpid = 0.0
mode = 1
stage = 0
mission = 0
kp = 0.01
pid = PID(kp,0,0)


 
class ControlLane():
    def __init__(self):
        self.sign_stage=10
        self.linedot =10
        self.ob=0
        self.sub_lane = rospy.Subscriber('/angle', Float64, self.cbFollowLane, queue_size = 10)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel',Twist, queue_size = 1)
        self.pub_feedback_error = rospy.Publisher('pid/error', Float64,queue_size=10)
        self.pub_feedback_cmd = rospy.Publisher('pid/mv',Float64,queue_size=10)
        #self.sub_fin_park = rospy.Subscriber('/pak_or_st',UInt8,self.getPark, queue_size = 10)
        #self.pub_pak_or = rospy.Publisher('/pak_or',UInt8,queue_size = 1)
        self.sub_ob = rospy.Subscriber('/ob',UInt8,self.getObs, queue_size = 1)
        self.sub_light = rospy.Subscriber('/traffic',Int64,self.getlight, queue_size = 10)
        #self.sub_sign = rospy.Subscriber('sign_park',Int64,self.get_park,queue_size=10)
        #self.sub_line_dot = rospy.Subscriber('line_dot',Int64,self.get_line_dot,queue_size=10)
        rospy.on_shutdown(self.fnShutDown)
    def getPark(self,pak_or_st):
            self.park_fin = pak_or_st.data
    def getObs(self,ob):
            self.ob = ob.data
    def getlight(self,traffic):
            self.light_stage = traffic.data
    def get_park(self,sign_park):
            self.sign_stage = sign_park.data
    def get_line_dot(self,line_dot):
            self.linedot = line_dot.data
    def cbFollowLane(self, angle):
        global x 
        global twist
        global twist2
        global twist3
        global joycontrol
        global y
        global tickgreen
        global pid
        global mission
        global kp
        feedback=angle.data
        if feedback < 0:
            feedback = 0
        if feedback > 100:
            feedback = 100
        pid.SetPoint = 50.0
        #error=50-feedback#
        pid.update(feedback)
        outputpid = pid.output
        twist = Twist()
        kp_s = 0.014
        kp_c = 0.052
        l_s = 2.2
        l_c = 2.1
        #g=2#
        #print(kp)
        print(feedback)
        print(outputpid)
        #print('kp=',kp)
        #twist.angular.z = outputpid
        if mission == 0:
            if self.ob ==0:
                rospy.loginfo("lane tracking")
                #print(self.light_stage)
                twist.angular.z = outputpid#
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.linear.x = l_c
                if feedback >= 15 and feedback <=85:
                #twist.linear.x = l_s
                #twist.angular.z = outputpid
                    kp = kp_s
                if feedback < 15:
                    kp = kp_c
                #twist.linear.x = l_c
                #twist.angular.z = outputpid
                if feedback >85:
                    kp = kp_c
                #twist.linear.x = l_c
                #twist.angular.z = outputpid
                self.pub_cmd_vel.publish(twist)
            
            elif self.ob ==1:
                twist.angular.z = 1.7
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.linear.x = 2.2
                self.pub_cmd_vel.publish(twist)
                rospy.loginfo("I am matrix")
            elif self.ob ==2:
                twist.angular.z = -0.8
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.linear.x = 1
                self.pub_cmd_vel.publish(twist)
                rospy.loginfo("theta is over")
            elif self.ob ==3:
                twist.angular.z = -1.5
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.linear.x = 0
                rospy.loginfo("Beware in font-right")
                self.pub_cmd_vel.publish(twist)
            elif self.ob ==4:
                twist.angular.z = 1.2
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.linear.x = 1.5
                self.pub_cmd_vel.publish(twist)
                rospy.loginfo("Beware in font-right")
            if self.light_stage ==0 and tickgreen==1 :
                mission = 1
            #if self.sign_stage == 1:
               # mission = 2
        if mission == 1:#before parking
            if self.light_stage == 0 :
                twist.linear.x = 0
                twist.angular.z = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
            if self.light_stage == 1:
                mission = 0
                tickgreen = 0
        if mission == 2:#
            if self.linedot == 0:
                twist.angular.z = outputpid
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                if feedback >= 20 and feedback <=80:
                    twist.linear.x = l_s
                    kp = kp_s
                if feedback < 30:
                    kp = kp_c
                    twist.linear.x = l_c
                if feedback >70:
                    kp = kp_c
                    twist.linear.x = l_c
            # if self.linedot ==1:
            #     if self.obs_seach == 1:
            #         twist.angular.z = outputpid
            #         twist.linear.y = 0
            #         twist.linear.z = 00
            #         twist.angular.x = 0
            #         twist.angular.y = 0
            #         if feedback >= 20 and feedback <=80:
            #             twist.linear.x = l_s
            #             kp = kp_s
            #         if feedback < 20:
            #             kp = kp_c
            #             twist.linear.x = l_c
            #         if feedback >80:
            #             kp = kp_c
            #             twist.linear.x = l_c
            #     if self.obs_seach == 0:
            #         self.pub_pak_or.publish(1)
        pid = PID(kp,0,0)
        #self.pub_cmd_vel.publish(twist) 
        
     #self.pub_cmd_vel.publish(twist)
    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)


    def main(self):
        #global kp
        rospy.spin()
        
        #pid = PID(kp,0,0)
        #pid.SetPoint=50.0
        #pid.setSampleTime(0.01)
        
        

if __name__ == '__main__':
    #global kp
    rospy.init_node('control_lane')
    node = ControlLane()
    #pid = PID(kp,0,0)
    pid.SetPoint=50.0
    pid.setSampleTime(0.01)
    node.main()
    

