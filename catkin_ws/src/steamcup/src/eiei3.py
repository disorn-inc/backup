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
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.sub_scan_obstacle = rospy.Subscriber('/scan', LaserScan, self.cbScanObstacle, queue_size=10)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size=1)
        self.pub_ob = rospy.Publisher('/ob',UInt8,queue_size = 1)
        self.theta = 0.0
        self.current_theta = 0.0
        self.last_current_theta = 0.0
        self.is_step_start = False
        self.lastError = 0.0
        self.step=0
        self.st=0
        
    def cbScanObstacle(self, scan):
        twist = Twist()
        if self.step == 0:
            rospy.loginfo("Now lane_following")
            if scan.ranges[0] < 0.25 and scan.ranges[0] > 0.01:
                self.step=1
        elif self.step == 1 :
            rospy.loginfo("turn 90")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta - 1.6
                self.is_step_start = True

            error = self.fnTurn()

            if math.fabs(error) < 0.07:
                rospy.loginfo("outer_turn_first finished")
                self.step = 2
                self.is_step_start = False
        elif self.step == 2 :
            # if self.st==0:
            #     rospy.loginfo("85  90  95 : %f  %f  %f", scan.ranges[80], scan.ranges[90], scan.ranges[100])
            #     twist.angular.z = 0.0
            #     self.pub_cmd_vel.publish(twist)
                # if scan.ranges[85] < scan.ranges[90] and scan.ranges[85] < scan.ranges[95]:
                #     twist.linear.x = 0
                #     twist.linear.y = 0
                #     twist.linear.z = 0
                #     twist.angular.x = 0
                #     twist.angular.y = 0
                #     twist.angular.z = 0.1
                #     self.st=2
                #     self.pub_cmd_vel.publish(twist)
                # elif scan.ranges[95] < scan.ranges[85] and scan.ranges[95] < scan.ranges[90]:
                #     twist.linear.x = 0
                #     twist.linear.y = 0
                #     twist.linear.z = 0
                #     twist.angular.x = 0
                #     twist.angular.y = 0
                #     twist.angular.z = -0.1
                #     self.st=3
                #     self.pub_cmd_vel.publish(twist)
                # elif scan.ranges[90] < scan.ranges[85] and scan.ranges[90] < scan.ranges[95]:
                #     self.st=1
            # elif self.st==1:
            rospy.loginfo(scan.ranges[100])
            twist.linear.x = 1.5
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.pub_cmd_vel.publish(twist)
            for i in range(130, 135):
                if scan.ranges[i] > 0.25 and scan.ranges[0] > 0.01:
                    self.st=0
                    self.step=3
        elif self.step == 3 :
            rospy.loginfo("turn 90")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta + 1.6
                self.is_step_start = True

            error = self.fnTurn()

            if math.fabs(error) < 0.07:
                rospy.loginfo("outer_turn_first finished")
                self.step = 4
                self.is_step_start = False
        elif self.step == 4 :
            rospy.loginfo("parking_lot_exit")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.start_pos_x = self.current_pos_x
                self.start_pos_y = self.current_pos_y
                self.is_step_start = True

            error = self.fnStraight(0.46)

            if math.fabs(error) < 0.07:
                rospy.loginfo("parking_lot_exit finished")
                self.step = 5
                self.is_step_start = False 
        elif self.step == 5 :
            rospy.loginfo("turn 90")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta + 1.57
                self.is_step_start = True

            error = self.fnTurn()

            if math.fabs(error) < 0.07:
                rospy.loginfo("outer_turn_first finished")
                self.step = 6
                self.is_step_start = False     
        elif self.step == 6 :  
            rospy.loginfo("back2")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.start_pos_x = self.current_pos_x
                self.start_pos_y = self.current_pos_y
                self.is_step_start = True

            error = self.fnStraight(0.35)

            if math.fabs(error) < 0.07:
                rospy.loginfo("parking_lot_exit finished")
                self.step = 7
                self.is_step_start = False 
        elif self.step == 7 :
            rospy.loginfo("turn 90")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta - 1.57
                self.is_step_start = True

            error = self.fnTurn()

            if math.fabs(error) < 0.05:
                rospy.loginfo("outer_turn_first finished")
                self.step = 0
                self.is_step_start = False
        # elif self.step == 8 :
        #     twist.linear.x = 0
        #     twist.linear.y = 0
        #     twist.linear.z = 0
        #     twist.angular.x = 0
        #     twist.angular.y = 0
        #     twist.angular.z = 0
        #     self.pub_cmd_vel.publish(twist)   
        self.pub_ob.publish(self.step)


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

    def fnStraight(self, desired_dist):
        err_pos = math.sqrt((self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) - desired_dist
        
        rospy.loginfo("Tunnel_Straight")
        rospy.loginfo("err_pos  desired_dist : %f  %f  %f", err_pos, desired_dist, self.lastError)

        self.lastError = err_pos

        twist = Twist()
        twist.linear.x = 0.1
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

        return err_pos
    def fnTurn(self):
        err_theta = self.current_theta - self.desired_theta
        
        rospy.loginfo("Parking_Turn")
        rospy.loginfo("err_theta  desired_theta  current_theta : %f  %f  %f", err_theta, self.desired_theta, self.current_theta)
        Kp = 0.8

        Kd = 0.03

        angular_z = Kp * err_theta + Kd * (err_theta - self.lastError)
        self.lastError = err_theta

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)

        rospy.loginfo("angular_z : %f", angular_z)

        return err_theta    
    def fnStop(self):
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
