#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
import os 
import cv2
from enum import Enum
from std_msgs.msg import UInt8, Float64, String
from sensor_msgs.msg import LaserScan, Image, CompressedImage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from PID import *



kp = 0.01
pid = PID(kp,0,0)






class rida():
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.sub_scan_obstacle = rospy.Subscriber('/scan', LaserScan, self.cbScanObstacle, queue_size=1)
        self.pub_ob = rospy.Publisher('/ob',Float64,queue_size = 1)
        # self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size=1)
        # rospy.on_shutdown(self.fnShutDown)
        self.current_theta = 0.0
        self.last_current_theta=0.0
        self.fist_theta = 0.0
        self.mode=0
        self.left_scan=10
        self.right_scan=10
    def cbScanObstacle(self, scan): 
        global twist
        global pid
        global kp
        self.left_scan=10
        self.right_scan=10
        for i in range(0,90):
            if scan.ranges[i]>0.01:
                self.left_scan=min(scan.ranges[i],self.left_scan)
                #print(self.left_scan)
        for i in range(270,360):
            if scan.ranges[i]>0.01:
                self.right_scan=min(scan.ranges[i],self.right_scan)
                # print(self.right_scan)
        scann=min(self.left_scan,self.right_scan)
        feedback_ob=scann
        pid.SetPoint = -0.3
        pid.update(feedback_ob)
        outputpid_ob = pid.output
        twist_ob = Twist()
        print(outputpid_ob)
         



           

        #print(scann)
        self.pub_ob.publish(scann)
    pid = PID(kp,0,0)
    def main(self):
        rospy.spin()
if __name__ == '__main__':
    rospy.init_node('eiei')
    node = rida()
    node.main()