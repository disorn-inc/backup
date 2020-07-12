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
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size=1)
        rospy.on_shutdown(self.fnShutDown)
        self.sub_lane = rospy.Subscriber('/angle', Float64, self.cbFollowLane, queue_size = 10)
        self.current_theta = 0.0
        self.last_current_theta=0.0
        self.fist_theta = 0.0
        self.mode=0
        self.angle=0
        self.ob_n=0
    def cbFollowLane(self,angle):
        self.angle = angle.data
    def cbScanObstacle(self, scan):
        if self.mode==0:
            rospy.loginfo("Lane Flowering")
            detect=0
            if scan.ranges[0] < 0.35 and scan.ranges[0] > 0.01:
                if self.ob_n%2==0:
                    self.desired_theta = self.current_theta + 1.7
                else:
                    self.desired_theta = self.current_theta + 1.7
                self.fist_theta = self.current_theta
                if self.angle>72:
                    self.angle=50
                self.mode=1
        elif self.mode==1:
            detect=1
            # for i in range(95,100):
                # if scan.ranges[i] < 0.15 and scan.ranges[i] > 0.01:
                #     detect=1
            for i in range(0, 34):
                if scan.ranges[i] < 0.32 and scan.ranges[i] > 0.01:
                    detect=3
                    rospy.loginfo("Obstact Detect")
            if  self.current_theta > self.desired_theta:
                    detect=2
                    rospy.loginfo("Theta Over") 
            for i in range(300,350): 
                if scan.ranges[i] < 0.2 and scan.ranges[i] > 0.01:
                    detect=4 
                    rospy.loginfo("Obstact 2 is Detect")           
            if self.angle>85: 
                self.mode=0
                self.ob_n=self.ob_n+1
        self.pub_ob.publish(detect)
        print(detect)
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
    def cbOdom(self, odom_msg):
        quaternion = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
        self.current_theta = self.euler_from_quaternion(quaternion)
        self.odom_msg = odom_msg
        if (self.current_theta - self.last_current_theta) < -math.pi:
            self.current_theta = 2. * math.pi + self.current_theta
            self.last_current_theta = math.pi
        elif (self.current_theta - self.last_current_theta) > math.pi:
            self.current_theta = -2. * math.pi + self.current_theta
            self.last_current_theta = -math.pi
        else:
            self.last_current_theta = self.current_theta

        self.current_pos_x = odom_msg.pose.pose.position.x
        self.current_pos_y = odom_msg.pose.pose.position.y
    def euler_from_quaternion(self, quaternion):
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        return theta
    def main(self):
        rospy.spin()
if __name__ == '__main__':
    rospy.init_node('Gory')
    node = Gory()
    node.main()