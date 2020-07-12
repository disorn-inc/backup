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
class Gory():
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.sub_scan_obstacle = rospy.Subscriber('/scan', LaserScan, self.cbScanObstacle, queue_size=1)
        self.pub_ob = rospy.Publisher('/ob',UInt8,queue_size = 1)
        rospy.on_shutdown(self.fnShutDown)
    def cbScanObstacle(self, scan):
        twist = Twist()
        scan_start = 0
        scan_end = 30
        threshold_distance = 0.2
        hh=0
        for i in range(scan_start, scan_end):
            if scan.ranges[i] < threshold_distance and scan.ranges[i] > 0.01:
               hh=1
        for i in range(85,95):
            if scan.ranges[i] < 0.17 and scan.ranges[i] > 0.01:
               hh=2
        self.pub_ob.publish(hh)
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
        rospy.spin()
if __name__ == '__main__':
    rospy.init_node('Gory')
    node = Gory()
    node.main()